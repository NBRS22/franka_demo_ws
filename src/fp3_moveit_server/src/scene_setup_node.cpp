#include <algorithm>
#include <optional>
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

// One-shot node: applies the static scene (table + wall) via
// /apply_planning_scene, then exits. Deliberately independent of
// MoveGroupInterface (only needs the raw service), so it stays a light,
// standalone step in bringup.launch.py rather than something bundled into
// motion_server_node or pick_place_node.
class SceneSetup : public rclcpp::Node
{
public:
  SceneSetup() : Node("scene_setup_node")
  {
    // Parameters for the table and wall collision objects
    table_frame_id_ = declare_parameter<std::string>("table.frame_id", "fp3_link0");
    table_position_ = declare_parameter<std::vector<double>>("table.position", {0.10, -0.50, -0.05});
    table_dimensions_ = declare_parameter<std::vector<double>>("table.dimensions", {0.60, 1.40, 0.10});
    table_allowed_touch_links_ = declare_parameter<std::vector<std::string>>("table.allowed_touch_links", {table_frame_id_});

    wall_frame_id_ = declare_parameter<std::string>("wall.frame_id", "fp3_link0");
    wall_position_ = declare_parameter<std::vector<double>>("wall.position", {0, 0, 0});
    wall_dimensions_ = declare_parameter<std::vector<double>>("wall.dimensions", {1, 1.40, 1.00});
    wall_allowed_touch_links_ = declare_parameter<std::vector<std::string>>(
      "wall.allowed_touch_links", std::vector<std::string>{});

    // The planning scene services
    get_scene_client_ = create_client<GetPlanningScene>("get_planning_scene");
    apply_scene_client_ = create_client<ApplyPlanningScene>("apply_planning_scene");
  }

  // Run the one-shot scene setup
  bool run()
  {
    RCLCPP_INFO(get_logger(), "Waiting for planning scene services...");
    if (!get_scene_client_->wait_for_service(std::chrono::seconds(10)) || !apply_scene_client_->wait_for_service(std::chrono::seconds(10)))
    {
      RCLCPP_FATAL(get_logger(), "Planning scene services unavailable after 10s, aborting");
      return false;
    }

    auto acm = fetchAcm();

    if (!acm)
    { 
      return false; 
    }

    const size_t original_size = acm->entry_names.size();

    auto extended_acm = extendAcm(*acm, "table", table_allowed_touch_links_);
    extended_acm = extendAcm(extended_acm, "wall", wall_allowed_touch_links_);

    moveit_msgs::msg::PlanningScene scene_diff;
    scene_diff.is_diff = true;
    scene_diff.world.collision_objects.push_back(buildObject("table", table_frame_id_, table_position_, table_dimensions_));
    scene_diff.world.collision_objects.push_back(buildObject("wall", wall_frame_id_, wall_position_, wall_dimensions_));
    scene_diff.allowed_collision_matrix = extended_acm;

    auto request = std::make_shared<ApplyPlanningScene::Request>();
    request->scene = scene_diff;

    auto future = apply_scene_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(shared_from_this(), future) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_FATAL(get_logger(), "Failed to call /apply_planning_scene");
      return false;
    }
    if (!future.get()->success) 
    {
      RCLCPP_FATAL(get_logger(), "/apply_planning_scene reported failure");
      return false;
    }

    RCLCPP_INFO(get_logger(), "Scene applied : table + wall (frame='%s'), ACM extended (%zu -> %zu entries)", table_frame_id_.c_str(), original_size, extended_acm.entry_names.size());
    return true;
  }

private:
  std::string table_frame_id_;
  std::vector<double> table_position_;
  std::vector<double> table_dimensions_;
  std::vector<std::string> table_allowed_touch_links_;

  std::string wall_frame_id_;
  std::vector<double> wall_position_;
  std::vector<double> wall_dimensions_;
  std::vector<std::string> wall_allowed_touch_links_;

  rclcpp::Client<GetPlanningScene>::SharedPtr get_scene_client_;
  rclcpp::Client<ApplyPlanningScene>::SharedPtr apply_scene_client_;

  // Generic box collision object builder.
  moveit_msgs::msg::CollisionObject buildObject(
    const std::string & id,
    const std::string & frame_id,
    const std::vector<double> & position,
    const std::vector<double> & dimensions)
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = frame_id;
    obj.id = id;
    obj.operation = obj.ADD;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {dimensions.at(0), dimensions.at(1), dimensions.at(2)};
    obj.primitives.push_back(primitive);

    geometry_msgs::msg::Pose pose;
    pose.position.x = position.at(0);
    pose.position.y = position.at(1);
    pose.position.z = position.at(2);
    pose.orientation.w = 1.0;
    obj.primitive_poses.push_back(pose);

    return obj;
  }

  std::optional<moveit_msgs::msg::AllowedCollisionMatrix> fetchAcm()
  {
    auto request = std::make_shared<GetPlanningScene::Request>();
    request->components.components = moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    auto future = get_scene_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(shared_from_this(), future) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_FATAL(get_logger(), "Failed to call /get_planning_scene");
      return std::nullopt;
    }
    return future.get()->scene.allowed_collision_matrix;
  }

  moveit_msgs::msg::AllowedCollisionMatrix extendAcm(moveit_msgs::msg::AllowedCollisionMatrix acm, const std::string & object_name, const std::vector<std::string> & allowed_touch_links)
  {
    const size_t original_size = acm.entry_names.size();

    std::vector<size_t> allowed_indices;
    for (const auto & link : allowed_touch_links) 
    {
      auto it = std::find(acm.entry_names.begin(), acm.entry_names.end(), link);
      if (it == acm.entry_names.end()) 
      {
        RCLCPP_WARN(get_logger(), "'%s' allowed_touch_links entry '%s' not found in ACM (robot not fully loaded yet?), skipping", object_name.c_str(), link.c_str());
        continue;
      }
      allowed_indices.push_back(static_cast<size_t>(std::distance(acm.entry_names.begin(), it)));
    }

    for (auto & entry : acm.entry_values) 
    {
      entry.enabled.push_back(false);
    }
    for (size_t idx : allowed_indices) 
    {
      acm.entry_values[idx].enabled[original_size] = true;
    }

    moveit_msgs::msg::AllowedCollisionEntry object_row;
    object_row.enabled.assign(original_size + 1, false);
    for (size_t idx : allowed_indices) 
    {
      object_row.enabled[idx] = true;
    }
    acm.entry_names.push_back(object_name);
    acm.entry_values.push_back(object_row);

    return acm;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SceneSetup>();
  const bool ok = node->run();
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
