#include <chrono>
#include <memory>
#include <atomic>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "franka_demo_interfaces/action/mtc_pick.hpp"

using MtcPick = franka_demo_interfaces::action::MtcPick;
using GoalHandleMtcPick = rclcpp_action::ServerGoalHandle<MtcPick>;


class CommandRouter : public rclcpp::Node
{
public:
  CommandRouter() : Node("command_router_node")
  {
    mtc_pick_client_ = rclcpp_action::create_client<MtcPick>(this, "/internal/pick_place/mtc_pick");

    RCLCPP_INFO(get_logger(), "Waiting for pick_place_node...");
    if (!mtc_pick_client_->wait_for_action_server(std::chrono::seconds(30))) 
    {
      RCLCPP_FATAL(get_logger(), "pick_place_node unavailable after 30s, aborting");
      throw std::runtime_error("pick_place_node unavailable");
    }

    mtc_pick_server_ = rclcpp_action::create_server<MtcPick>(
      this, "mtc_pick",
      std::bind(&CommandRouter::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&CommandRouter::handle_cancel, this, std::placeholders::_1),
      std::bind(&CommandRouter::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "command_router_node ready (mtc_pick -> pick_place_node)");
  }

private:
  rclcpp_action::Client<MtcPick>::SharedPtr mtc_pick_client_;
  rclcpp_action::Server<MtcPick>::SharedPtr mtc_pick_server_;
  rclcpp_action::ClientGoalHandle<MtcPick>::SharedPtr active_goal_;
  std::atomic<bool> busy_{false};

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const MtcPick::Goal>)
  {
    if (busy_.load()) 
    {
      RCLCPP_WARN(get_logger(), "mtc_pick rejected: router busy");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!mtc_pick_client_->action_server_is_ready()) 
    {
      RCLCPP_ERROR(get_logger(), "pick_place_node not reachable, rejecting mtc_pick");
      return rclcpp_action::GoalResponse::REJECT;
    }
    busy_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMtcPick>)
  {
    if (active_goal_) mtc_pick_client_->async_cancel_goal(active_goal_);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMtcPick> goal_handle)
  {
    rclcpp_action::Client<MtcPick>::SendGoalOptions options;

    options.feedback_callback =
      [goal_handle](
        rclcpp_action::ClientGoalHandle<MtcPick>::SharedPtr,
        const std::shared_ptr<const MtcPick::Feedback> feedback) {
          goal_handle->publish_feedback(
            std::make_shared<MtcPick::Feedback>(*feedback));
        };

    options.goal_response_callback =
      [this, goal_handle](rclcpp_action::ClientGoalHandle<MtcPick>::SharedPtr h) {
        if (!h) {
          auto r = std::make_shared<MtcPick::Result>();
          r->success = false;
          r->message = "pick_place_node rejected the goal";
          goal_handle->abort(r);
          busy_ = false;
          return;
        }
        active_goal_ = h;
      };

    options.result_callback =
      [this, goal_handle](
        const rclcpp_action::ClientGoalHandle<MtcPick>::WrappedResult & w) {
          active_goal_.reset();
          auto r = std::make_shared<MtcPick::Result>(*w.result);
          switch (w.code) {
            case rclcpp_action::ResultCode::SUCCEEDED: goal_handle->succeed(r); break;
            case rclcpp_action::ResultCode::CANCELED:  goal_handle->canceled(r); break;
            default:                                   goal_handle->abort(r); break;
          }
          busy_ = false;
        };

    mtc_pick_client_->async_send_goal(*goal_handle->get_goal(), options);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try 
  {
    rclcpp::spin(std::make_shared<CommandRouter>());
  } 
  catch (const std::exception & e) 
  {
    RCLCPP_FATAL(rclcpp::get_logger("command_router_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
