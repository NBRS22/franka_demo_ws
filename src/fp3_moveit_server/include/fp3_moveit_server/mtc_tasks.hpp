#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "visualization_msgs/msg/marker.hpp"

// Same remap as graspgen_bridge/visualize_grasps_node.py's
// _approach_to_arrow_orientation: RViz draws a pose-based Marker.ARROW along
// local +X, but this pipeline's approach axis (gripper -> object) is local
// +Z (cf. GraspGen's docs/GRIPPER_DESCRIPTION.md). Remaps a grasp
// orientation so the rendered arrow's local +X points along the true
// approach direction. Verified against scipy as an independent ground
// truth (max error ~1e-16 over 5000 random rotations) before porting here.
geometry_msgs::msg::Quaternion approachToArrowOrientation(
  const geometry_msgs::msg::Quaternion & q);

// local +Z rotated into world by q -- the approach direction (gripper -> object).
std::array<double, 3> approachAxisWorld(const geometry_msgs::msg::Quaternion & q);

// ARROW marker whose tip lands exactly on pose.position, tail offset
// backward along the approach direction by 0.08m -- same convention as
// graspgen_bridge/visualize_grasps_node.py's grasp candidate markers.
// Defaults match the original single-marker use (executed grasp, amber).
visualization_msgs::msg::Marker approachArrowMarker(
  const geometry_msgs::msg::PoseStamped & pose,
  const std::string & ns = "executed_grasp",
  int32_t id = 0,
  float r = 1.0f, float g = 0.85f, float b = 0.0f, float a = 1.0f);

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
// pose_pub, if non-null, is published with `pose` right after planning
// succeeds and right before execute() is called -- i.e. the grasp pose that
// is actually about to be executed, not every candidate merely attempted.
// marker_pub, if non-null, gets an ARROW marker at the same instant, tip on
// the grasp point and orientation showing the true approach direction (cf.
// graspgen_bridge/visualize_grasps_node.py's _approach_to_arrow_orientation/
// _approach_axis_world -- same remap, reimplemented here since pose_pub's
// raw PoseStamped is meant for programmatic consumers and shouldn't be
// visualization-remapped itself).
bool planAndExecuteApproach(
  rclcpp::Node::SharedPtr node,
  const MtcParams & mtc,
  const ApproachParams & approach,
  const geometry_msgs::msg::PoseStamped & pose,
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub = nullptr,
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub = nullptr);

// MTC Task: cartesian lift along world Z+ (fp3_link0 = fixed = world).
// Called after the object is attached, so the lift is planned with the
// object's collision geometry included. Returns true if plan + execute succeed.
bool planAndExecuteLift(
  rclcpp::Node::SharedPtr node,
  const MtcParams & mtc,
  const LiftParams & lift);
