#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "franka_msgs/action/grasp.hpp"
#include "franka_msgs/action/move.hpp"

// Wraps the two franka_gripper actions (Move to open, Grasp to close).
// In simulate mode every call returns true immediately without touching
// the hardware -- wire simulate=true when use_fake_hardware:=true since
// franka_gripper_node is not launched in that mode.
class GripperController
{
public:
  GripperController(
    rclcpp::Node::SharedPtr node,
    const std::string & grasp_action_name,
    const std::string & move_action_name,
    bool simulate);

  bool open(double width, double speed);

  bool close(
    double width,
    double speed,
    double force,
    double epsilon_inner,
    double epsilon_outer);

private:
  using Grasp = franka_msgs::action::Grasp;
  using Move  = franka_msgs::action::Move;

  rclcpp::Node::SharedPtr node_;
  bool simulate_;
  std::string grasp_action_name_;
  std::string move_action_name_;
  rclcpp_action::Client<Grasp>::SharedPtr grasp_client_;
  rclcpp_action::Client<Move>::SharedPtr  move_client_;
};
