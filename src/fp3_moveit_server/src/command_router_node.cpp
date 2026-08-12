#include <chrono>
#include <memory>
#include <atomic>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "franka_demo_interfaces/action/move_to_pose.hpp"
#include "franka_demo_interfaces/action/pick_object.hpp"

using MoveToPose = franka_demo_interfaces::action::MoveToPose;
using PickObject = franka_demo_interfaces::action::PickObject;
using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPose>;
using GoalHandlePickObject = rclcpp_action::ServerGoalHandle<PickObject>;

// Single public entry point for the arm: exposes move_to_pose and
// pick_object, and forwards each to the internal node that actually owns
// MoveGroupInterface/MTC for it (motion_server_node / pick_place_node).
// A single busy_ flag, shared across both action types, guarantees the two
// backends are never commanding the arm at the same time -- clients only
// ever see this node, never the internal ones directly.
class CommandRouter : public rclcpp::Node
{
public:
  CommandRouter()
  : Node("command_router_node")
  {
    move_to_pose_client_ = rclcpp_action::create_client<MoveToPose>(
      this, "/internal/motion_server/move_to_pose");
    pick_object_client_ = rclcpp_action::create_client<PickObject>(
      this, "/internal/pick_place/pick_object");

    // Public action servers are created only after both internal backends
    // are confirmed reachable, not before. Without this gate, this node's
    // own public move_to_pose/pick_object servers become discoverable the
    // instant it's constructed -- well before motion_server_node/
    // pick_place_node (which load a robot model, wait on move_group, etc.)
    // are actually up. A client's wait_for_server() on the PUBLIC action
    // would then return true immediately, only for the goal to be flatly
    // rejected the moment it's sent, since handle_move_goal/handle_pick_goal
    // check the INTERNAL client's readiness at goal time. Observed live
    // with apriltag_move_once_node racing ahead of motion_server_node.
    RCLCPP_INFO(get_logger(), "Waiting for motion_server_node...");
    if (!move_to_pose_client_->wait_for_action_server(std::chrono::seconds(30))) {
      RCLCPP_FATAL(get_logger(), "motion_server_node unavailable after 30s, aborting");
      throw std::runtime_error("motion_server_node unavailable");
    }
    RCLCPP_INFO(get_logger(), "Waiting for pick_place_node...");
    if (!pick_object_client_->wait_for_action_server(std::chrono::seconds(30))) {
      RCLCPP_FATAL(get_logger(), "pick_place_node unavailable after 30s, aborting");
      throw std::runtime_error("pick_place_node unavailable");
    }

    move_to_pose_server_ = rclcpp_action::create_server<MoveToPose>(
      this,
      "move_to_pose",
      std::bind(&CommandRouter::handle_move_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&CommandRouter::handle_move_cancel, this, std::placeholders::_1),
      std::bind(&CommandRouter::handle_move_accepted, this, std::placeholders::_1));

    pick_object_server_ = rclcpp_action::create_server<PickObject>(
      this,
      "pick_object",
      std::bind(&CommandRouter::handle_pick_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&CommandRouter::handle_pick_cancel, this, std::placeholders::_1),
      std::bind(&CommandRouter::handle_pick_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "command_router_node ready (move_to_pose -> motion_server, pick_object -> pick_place)");
  }

private:
  rclcpp_action::Client<MoveToPose>::SharedPtr move_to_pose_client_;
  rclcpp_action::Client<PickObject>::SharedPtr pick_object_client_;
  rclcpp_action::Server<MoveToPose>::SharedPtr move_to_pose_server_;
  rclcpp_action::Server<PickObject>::SharedPtr pick_object_server_;

  std::atomic<bool> busy_{false};
  rclcpp_action::ClientGoalHandle<MoveToPose>::SharedPtr active_move_goal_;
  rclcpp_action::ClientGoalHandle<PickObject>::SharedPtr active_pick_goal_;

  // ---- move_to_pose -> motion_server_node ----

  rclcpp_action::GoalResponse handle_move_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveToPose::Goal>)
  {
    if (busy_.load()) {
      RCLCPP_WARN(get_logger(), "move_to_pose rejected: router busy with another command");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!move_to_pose_client_->action_server_is_ready()) {
      RCLCPP_ERROR(get_logger(), "motion_server_node not reachable, rejecting move_to_pose");
      return rclcpp_action::GoalResponse::REJECT;
    }
    busy_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_move_cancel(const std::shared_ptr<GoalHandleMoveToPose>)
  {
    if (active_move_goal_) {
      move_to_pose_client_->async_cancel_goal(active_move_goal_);
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_move_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
  {
    rclcpp_action::Client<MoveToPose>::SendGoalOptions options;

    options.feedback_callback =
      [goal_handle](
      rclcpp_action::ClientGoalHandle<MoveToPose>::SharedPtr,
      const std::shared_ptr<const MoveToPose::Feedback> feedback) {
        goal_handle->publish_feedback(std::make_shared<MoveToPose::Feedback>(*feedback));
      };

    options.goal_response_callback =
      [this, goal_handle](rclcpp_action::ClientGoalHandle<MoveToPose>::SharedPtr internal_handle) {
        if (!internal_handle) {
          auto result = std::make_shared<MoveToPose::Result>();
          result->success = false;
          result->message = "motion_server_node rejected the goal";
          goal_handle->abort(result);
          busy_ = false;
          return;
        }
        active_move_goal_ = internal_handle;
      };

    options.result_callback =
      [this, goal_handle](const rclcpp_action::ClientGoalHandle<MoveToPose>::WrappedResult & wrapped) {
        active_move_goal_.reset();
        auto result = std::make_shared<MoveToPose::Result>(*wrapped.result);
        switch (wrapped.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            goal_handle->succeed(result);
            break;
          case rclcpp_action::ResultCode::CANCELED:
            goal_handle->canceled(result);
            break;
          default:
            goal_handle->abort(result);
            break;
        }
        busy_ = false;
      };

    move_to_pose_client_->async_send_goal(*goal_handle->get_goal(), options);
  }

  // ---- pick_object -> pick_place_node ----

  rclcpp_action::GoalResponse handle_pick_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const PickObject::Goal>)
  {
    if (busy_.load()) {
      RCLCPP_WARN(get_logger(), "pick_object rejected: router busy with another command");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!pick_object_client_->action_server_is_ready()) {
      RCLCPP_ERROR(get_logger(), "pick_place_node not reachable, rejecting pick_object");
      return rclcpp_action::GoalResponse::REJECT;
    }
    busy_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_pick_cancel(const std::shared_ptr<GoalHandlePickObject>)
  {
    if (active_pick_goal_) {
      pick_object_client_->async_cancel_goal(active_pick_goal_);
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_pick_accepted(const std::shared_ptr<GoalHandlePickObject> goal_handle)
  {
    rclcpp_action::Client<PickObject>::SendGoalOptions options;

    options.feedback_callback =
      [goal_handle](
      rclcpp_action::ClientGoalHandle<PickObject>::SharedPtr,
      const std::shared_ptr<const PickObject::Feedback> feedback) {
        goal_handle->publish_feedback(std::make_shared<PickObject::Feedback>(*feedback));
      };

    options.goal_response_callback =
      [this, goal_handle](rclcpp_action::ClientGoalHandle<PickObject>::SharedPtr internal_handle) {
        if (!internal_handle) {
          auto result = std::make_shared<PickObject::Result>();
          result->success = false;
          result->message = "pick_place_node rejected the goal";
          goal_handle->abort(result);
          busy_ = false;
          return;
        }
        active_pick_goal_ = internal_handle;
      };

    options.result_callback =
      [this, goal_handle](const rclcpp_action::ClientGoalHandle<PickObject>::WrappedResult & wrapped) {
        active_pick_goal_.reset();
        auto result = std::make_shared<PickObject::Result>(*wrapped.result);
        switch (wrapped.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            goal_handle->succeed(result);
            break;
          case rclcpp_action::ResultCode::CANCELED:
            goal_handle->canceled(result);
            break;
          default:
            goal_handle->abort(result);
            break;
        }
        busy_ = false;
      };

    pick_object_client_->async_send_goal(*goal_handle->get_goal(), options);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<CommandRouter>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("command_router_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
