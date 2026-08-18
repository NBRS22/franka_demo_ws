#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// Parameters shared by both MTC tasks.
struct MtcParams
{
  std::string planning_group;
  std::string eef_name;
  std::string tcp_frame;
  std::string object_id;
  std::vector<std::string> hand_touch_links;
  double velocity_scaling_factor;
  double acceleration_scaling_factor;
};

// Parameters specific to the approach phase.
struct ApproachParams
{
  double min_distance;
  double max_distance;
};

// Parameters specific to the lift phase.
struct LiftParams
{
  double min_distance;
  double max_distance;
};

// MTC Task: free-space motion (Connect/OMPL) to a pre-grasp position, then
// cartesian approach along TCP +Z, arriving at the grasp pose. Allows
// hand/object collision via ModifyPlanningScene so the hand can touch the
// object at the grasp pose. Returns true if plan + execute both succeed.
bool planAndExecuteApproach(
  rclcpp::Node::SharedPtr node,
  const MtcParams & mtc,
  const ApproachParams & approach,
  const geometry_msgs::msg::PoseStamped & pose);

// MTC Task: cartesian lift along world Z+ (fp3_link0 = fixed = world).
// Called after the object is attached, so the lift is planned with the
// object's collision geometry included. Returns true if plan + execute succeed.
bool planAndExecuteLift(
  rclcpp::Node::SharedPtr node,
  const MtcParams & mtc,
  const LiftParams & lift);
