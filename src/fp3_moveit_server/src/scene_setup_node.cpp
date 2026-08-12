#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "moveit_msgs/msg/allowed_collision_entry.hpp"
#include "moveit_msgs/msg/allowed_collision_matrix.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"
#include "moveit_msgs/msg/planning_scene_components.hpp"
#include "moveit_msgs/srv/apply_planning_scene.hpp"
#include "moveit_msgs/srv/get_planning_scene.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "geometry_msgs/msg/pose.hpp"

using ApplyPlanningScene = moveit_msgs::srv::ApplyPlanningScene;
using GetPlanningScene = moveit_msgs::srv::GetPlanningScene;

// One-shot node: applies the static scene (currently just the table) via
// /apply_planning_scene, then exits. Deliberately independent of
// MoveGroupInterface (only needs the raw service), so it stays a light,
// standalone step in bringup.launch.py rather than something bundled into
// motion_server_node or pick_place_node.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("scene_setup_node");

  const std::string frame_id =
    node->declare_parameter<std::string>("table.frame_id", "fp3_link0");
  const std::vector<double> position =
    node->declare_parameter<std::vector<double>>("table.position", {0.5, 0.0, -0.05});
  const std::vector<double> dimensions =
    node->declare_parameter<std::vector<double>>("table.dimensions", {1.2, 0.8, 0.1});
  // Links physically expected to touch the table (the robot's own mounting
  // point) -- collision between these and "table" is allowed explicitly, so
  // a large table (e.g. one sized/positioned to reach under the robot base)
  // doesn't get flagged as a permanent collision. Defaults to just the base
  // link: observed live, "fp3_link0 colliding with table" broke every MTC
  // plan once the table was resized to span the area under the base.
  const std::vector<std::string> allowed_touch_links =
    node->declare_parameter<std::vector<std::string>>("table.allowed_touch_links", {frame_id});

  moveit_msgs::msg::CollisionObject table;
  table.header.frame_id = frame_id;
  table.id = "table";
  table.operation = table.ADD;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions = {dimensions.at(0), dimensions.at(1), dimensions.at(2)};
  table.primitives.push_back(primitive);

  geometry_msgs::msg::Pose pose;
  pose.position.x = position.at(0);
  pose.position.y = position.at(1);
  pose.position.z = position.at(2);
  pose.orientation.w = 1.0;
  table.primitive_poses.push_back(pose);

  auto get_scene_client = node->create_client<GetPlanningScene>("get_planning_scene");
  auto apply_scene_client = node->create_client<ApplyPlanningScene>("apply_planning_scene");
  RCLCPP_INFO(node->get_logger(), "Waiting for planning scene services...");
  if (!get_scene_client->wait_for_service(std::chrono::seconds(10)) ||
    !apply_scene_client->wait_for_service(std::chrono::seconds(10)))
  {
    RCLCPP_FATAL(node->get_logger(), "Planning scene services unavailable after 10s, aborting");
    rclcpp::shutdown();
    return 1;
  }

  // Fetch the CURRENT AllowedCollisionMatrix first, then extend it, rather
  // than sending a fresh ACM of our own: moveit_msgs::msg::PlanningScene's
  // allowed_collision_matrix field is NOT merged incrementally by
  // /apply_planning_scene the way world.collision_objects is (each
  // CollisionObject carries its own ADD/REMOVE operation, but
  // AllowedCollisionMatrix has no such per-entry semantics) -- supplying a
  // partial matrix REPLACES the whole ACM outright, silently discarding
  // every disable_collisions pair the SRDF normally seeds it with. Hit this
  // live: after adding just {"table", frame_id}, MTC started reporting
  // "fp3_hand colliding with fp3_rightfinger" -- a pair the SRDF explicitly
  // disables ("Adjacent") -- because that whole default matrix had been
  // wiped down to the 2 entries this node had just sent.
  auto get_scene_request = std::make_shared<GetPlanningScene::Request>();
  get_scene_request->components.components =
    moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  auto get_scene_future = get_scene_client->async_send_request(get_scene_request);
  if (rclcpp::spin_until_future_complete(node, get_scene_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_FATAL(node->get_logger(), "Failed to call /get_planning_scene");
    rclcpp::shutdown();
    return 1;
  }
  auto acm = get_scene_future.get()->scene.allowed_collision_matrix;
  const size_t original_size = acm.entry_names.size();

  std::vector<size_t> allowed_indices;
  for (const auto & link : allowed_touch_links) {
    auto it = std::find(acm.entry_names.begin(), acm.entry_names.end(), link);
    if (it == acm.entry_names.end()) {
      RCLCPP_WARN(
        node->get_logger(), "table.allowed_touch_links entry '%s' not found in the current "
        "AllowedCollisionMatrix (robot not fully loaded yet?), skipping it", link.c_str());
      continue;
    }
    allowed_indices.push_back(static_cast<size_t>(std::distance(acm.entry_names.begin(), it)));
  }

  // Grow every existing row by one column (vs "table"), default false
  // (still checked) except at allowed_indices.
  for (auto & entry : acm.entry_values) {
    entry.enabled.push_back(false);
  }
  for (size_t idx : allowed_indices) {
    acm.entry_values[idx].enabled[original_size] = true;
  }
  // New "table" row, sized to match the now-(original_size + 1)-wide matrix.
  moveit_msgs::msg::AllowedCollisionEntry table_row;
  table_row.enabled.assign(original_size + 1, false);
  for (size_t idx : allowed_indices) {
    table_row.enabled[idx] = true;
  }
  acm.entry_names.push_back("table");
  acm.entry_values.push_back(table_row);

  moveit_msgs::msg::PlanningScene scene_diff;
  scene_diff.is_diff = true;
  scene_diff.world.collision_objects.push_back(table);
  scene_diff.allowed_collision_matrix = acm;

  auto apply_request = std::make_shared<ApplyPlanningScene::Request>();
  apply_request->scene = scene_diff;

  auto apply_future = apply_scene_client->async_send_request(apply_request);
  if (rclcpp::spin_until_future_complete(node, apply_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_FATAL(node->get_logger(), "Failed to call /apply_planning_scene");
    rclcpp::shutdown();
    return 1;
  }

  if (!apply_future.get()->success) {
    RCLCPP_FATAL(node->get_logger(), "/apply_planning_scene reported failure");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Table collision object applied to planning scene (frame='%s'), ACM extended (%zu -> %zu "
    "entries)",
    frame_id.c_str(), original_size, acm.entry_names.size());
  rclcpp::shutdown();
  return 0;
}
