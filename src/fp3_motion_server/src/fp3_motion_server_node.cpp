#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "franka_demo_interfaces/action/move_to_pose.hpp"

using MoveToPose = franka_demo_interfaces::action::MoveToPose;
using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPose>;

// v1: minimal layer 1. Receives a target pose via the MoveToPose action and
// directly does setPoseTarget() + plan() + execute() (no pre-grasp, no
// cartesian approach). Network client of move_group (running in
// franka_ros2_ws) via MoveGroupInterface, loads no robot model here.
class Fp3MotionServer
{
public:
  Fp3MotionServer(
    const rclcpp::Node::SharedPtr & node,
    const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> & move_group)
  : node_(node), move_group_(move_group)
  {
    action_server_ = rclcpp_action::create_server<MoveToPose>(
      node_,
      "move_to_pose",
      std::bind(&Fp3MotionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&Fp3MotionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&Fp3MotionServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "fp3_motion_server ready (planning group 'fp3_arm')");
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const MoveToPose::Goal> goal)
  {
    RCLCPP_INFO(
      node_->get_logger(),
      "Goal received: pose (%.3f, %.3f, %.3f) frame='%s'",
      goal->target_pose.pose.position.x,
      goal->target_pose.pose.position.y,
      goal->target_pose.pose.position.z,
      goal->target_pose.header.frame_id.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMoveToPose>)
  {
    // v1: canceling an in-progress MoveIt execution is not implemented.
    RCLCPP_WARN(node_->get_logger(), "Cancel requested but not supported in v1");
    return rclcpp_action::CancelResponse::REJECT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
  {
    // plan()/execute() are blocking: we run them off the executor thread
    // so as not to block the rest of the node (including MoveGroupInterface's
    // internal calls).
    std::thread{std::bind(&Fp3MotionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<MoveToPose::Result>();

    move_group_->setPoseTarget(goal->target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    RCLCPP_INFO(node_->get_logger(), "Planning to target pose...");
    bool plan_ok = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!plan_ok) {
      result->success = false;
      result->message = "Planning failed";
      RCLCPP_ERROR(node_->get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }

    RCLCPP_INFO(node_->get_logger(), "Executing plan...");
    bool exec_ok = (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    result->success = exec_ok;
    result->message = exec_ok ? "Movement executed successfully" : "Execution failed";

    if (exec_ok) {
      RCLCPP_INFO(node_->get_logger(), "%s", result->message.c_str());
      goal_handle->succeed(result);
    } else {
      RCLCPP_ERROR(node_->get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("fp3_motion_server");

  // MoveGroupInterface makes synchronous calls (parameters, action
  // clients...) at construction: the node must already be spinning on a
  // separate thread, otherwise construction blocks indefinitely.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  // MoveGroupInterface builds its robot model LOCALLY and needs
  // robot_description/robot_description_semantic as parameters on this
  // node. franka_fp3_moveit_config (franka_ros2_ws) has no static .srdf
  // file: everything is computed by xacro when move_group launches. Rather
  // than duplicating that xacro logic here, we fetch these two values
  // directly from /move_group via a ROS2 parameter client.
  auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(node, "move_group");
  RCLCPP_INFO(node->get_logger(), "Waiting for move_group parameter server...");
  if (!param_client->wait_for_service(std::chrono::seconds(30))) {
    RCLCPP_FATAL(node->get_logger(), "move_group unavailable after 30s, aborting");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  auto results = param_client->get_parameters(
    {"robot_description", "robot_description_semantic"}).get();
  for (const auto & param : results) {
    node->declare_parameter<std::string>(param.get_name(), param.as_string());
  }
  RCLCPP_INFO(node->get_logger(), "Robot model retrieved from move_group");

  auto move_group =
    std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "fp3_arm");

  Fp3MotionServer server(node, move_group);

  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
