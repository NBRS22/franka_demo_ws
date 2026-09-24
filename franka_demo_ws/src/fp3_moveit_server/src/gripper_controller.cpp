#include "fp3_moveit_server/gripper_controller.hpp"

GripperController::GripperController(
  rclcpp::Node::SharedPtr node,
  const std::string & grasp_action_name,
  const std::string & move_action_name,
  bool simulate)
: node_(node),
  simulate_(simulate),
  grasp_action_name_(grasp_action_name),
  move_action_name_(move_action_name)
{
  grasp_client_ = rclcpp_action::create_client<Grasp>(node_, grasp_action_name_);
  move_client_  = rclcpp_action::create_client<Move>(node_, move_action_name_);
}

bool GripperController::open(double width, double speed)
{
  if (simulate_) {
    RCLCPP_INFO(node_->get_logger(), "Opening gripper (simulated): width=%.3f", width);
    return true;
  }

  if (!move_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(
      node_->get_logger(), "Gripper move action '%s' unavailable",
      move_action_name_.c_str());
    return false;
  }

  auto goal = Move::Goal();
  goal.width = width;
  goal.speed = speed;

  auto gh = move_client_->async_send_goal(goal).get();
  if (!gh) {
    RCLCPP_ERROR(node_->get_logger(), "Gripper open goal rejected");
    return false;
  }

  auto result = move_client_->async_get_result(gh).get();
  return result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success;
}

bool GripperController::close(
  double width,
  double speed,
  double force,
  double epsilon_inner,
  double epsilon_outer)
{
  if (simulate_) {
    RCLCPP_INFO(node_->get_logger(), "Closing gripper (simulated): width=%.3f", width);
    return true;
  }

  if (!grasp_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(
      node_->get_logger(), "Gripper grasp action '%s' unavailable",
      grasp_action_name_.c_str());
    return false;
  }

  auto goal = Grasp::Goal();
  goal.width             = width;
  goal.speed             = speed;
  goal.force             = force;
  goal.epsilon.inner     = epsilon_inner;
  goal.epsilon.outer     = epsilon_outer;

  auto gh = grasp_client_->async_send_goal(goal).get();
  if (!gh) {
    RCLCPP_ERROR(node_->get_logger(), "Grasp goal rejected");
    return false;
  }

  auto result = grasp_client_->async_get_result(gh).get();
  return result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success;
}
