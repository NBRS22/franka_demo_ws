#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"
#include "moveit_msgs/srv/apply_planning_scene.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

#include "franka_demo_interfaces/action/mtc_pick.hpp"

#include "fp3_moveit_server/gripper_controller.hpp"
#include "fp3_moveit_server/mtc_tasks.hpp"

using MtcPick = franka_demo_interfaces::action::MtcPick;
using GoalHandleMtcPick = rclcpp_action::ServerGoalHandle<MtcPick>;
using ApplyPlanningScene = moveit_msgs::srv::ApplyPlanningScene;

namespace
{
struct FilteredPose
{
  int original_index;
  geometry_msgs::msg::PoseStamped pose;
};
}  // namespace

class MtcPickServer
{
public:
  explicit MtcPickServer(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
    // MTC / motion parameters
    mtc_params_.planning_group = node_->declare_parameter<std::string>("planning_group", "fp3_arm");
    mtc_params_.eef_name       = node_->declare_parameter<std::string>("eef_name", "fp3_hand");
    mtc_params_.tcp_frame      = node_->declare_parameter<std::string>("tcp_frame", "fp3_hand_tcp");
    mtc_params_.hand_touch_links = node_->declare_parameter<std::vector<std::string>>(
      "hand_touch_links", {"fp3_hand", "fp3_leftfinger", "fp3_rightfinger"});
    mtc_params_.object_id = node_->declare_parameter<std::string>("object.id", "picked_object");
    mtc_params_.velocity_scaling_factor =
      node_->declare_parameter<double>("velocity_scaling_factor", 0.1);
    mtc_params_.acceleration_scaling_factor =
      node_->declare_parameter<double>("acceleration_scaling_factor", 0.1);

    // Approach / lift distances
    approach_params_.min_distance = node_->declare_parameter<double>("approach.min_distance", 0.02);
    approach_params_.max_distance = node_->declare_parameter<double>("approach.max_distance", 0.10);
    lift_params_.min_distance     = node_->declare_parameter<double>("lift.min_distance", 0.05);
    lift_params_.max_distance     = node_->declare_parameter<double>("lift.max_distance", 0.15);

    // Geometric prefilter thresholds
    min_height_above_table_ =
      node_->declare_parameter<double>("filter.min_height_above_table", 0.02);
    max_approach_tilt_deg_ =
      node_->declare_parameter<double>("filter.max_approach_tilt_deg", 45.0);
    max_reach_ = node_->declare_parameter<double>("filter.max_reach", 0.85);

    // Table top Z (used by the height filter) — derived from scene params.
    const std::vector<double> table_position =
      node_->declare_parameter<std::vector<double>>("table.position", {0.10, -0.50, -0.05});
    const std::vector<double> table_dimensions =
      node_->declare_parameter<std::vector<double>>("table.dimensions", {0.60, 1.40, 0.10});
    node_->declare_parameter<std::string>("table.frame_id", "fp3_link0");
    table_top_z_ = table_position.at(2) + table_dimensions.at(2) / 2.0;

    // Object dimensions for planning-scene attach
    object_dimensions_ = node_->declare_parameter<std::vector<double>>(
      "object.dimensions", {0.03, 0.03, 0.03});

    // Gripper parameters
    const std::string grasp_action = node_->declare_parameter<std::string>(
      "gripper_action_name", "/franka_gripper/grasp");
    const std::string move_action = node_->declare_parameter<std::string>(
      "gripper_move_action_name", "/franka_gripper/move");
    const bool simulate = node_->declare_parameter<bool>("simulate_gripper", false);
    open_width_  = node_->declare_parameter<double>("open.width", 0.08);
    open_speed_  = node_->declare_parameter<double>("open.speed", 0.1);
    grasp_width_ = node_->declare_parameter<double>("grasp.width", 0.0);
    grasp_speed_ = node_->declare_parameter<double>("grasp.speed", 0.05);
    grasp_force_ = node_->declare_parameter<double>("grasp.force", 20.0);
    grasp_epsilon_inner_ = node_->declare_parameter<double>("grasp.epsilon_inner", 0.005);
    grasp_epsilon_outer_ = node_->declare_parameter<double>("grasp.epsilon_outer", 0.005);

    gripper_ = std::make_unique<GripperController>(node_, grasp_action, move_action, simulate);

    scene_client_ = node_->create_client<ApplyPlanningScene>("apply_planning_scene");

    action_server_ = rclcpp_action::create_server<MtcPick>(
      node_,
      "mtc_pick",
      std::bind(&MtcPickServer::handle_goal, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(&MtcPickServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&MtcPickServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(
      node_->get_logger(), "pick_place_node ready (group '%s', object '%s')",
      mtc_params_.planning_group.c_str(), mtc_params_.object_id.c_str());
  }

  ~MtcPickServer()
  {
    if (execution_thread_.joinable()) execution_thread_.join();
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<MtcPick>::SharedPtr action_server_;
  rclcpp::Client<ApplyPlanningScene>::SharedPtr scene_client_;
  std::unique_ptr<GripperController> gripper_;
  std::thread execution_thread_;
  std::atomic<bool> busy_{false};

  MtcParams     mtc_params_;
  ApproachParams approach_params_;
  LiftParams     lift_params_;

  double min_height_above_table_;
  double max_approach_tilt_deg_;
  double max_reach_;
  double table_top_z_;

  double open_width_;
  double open_speed_;
  double grasp_width_;
  double grasp_speed_;
  double grasp_force_;
  double grasp_epsilon_inner_;
  double grasp_epsilon_outer_;

  std::vector<double> object_dimensions_;

  struct BusyGuard
  {
    std::atomic<bool> & flag;
    ~BusyGuard() {flag = false;}
  };

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const MtcPick::Goal> goal)
  {
    if (busy_.load()) {
      RCLCPP_WARN(node_->get_logger(), "Goal rejected: pick already in progress");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->grasp_poses.empty()) {
      RCLCPP_WARN(node_->get_logger(), "Goal rejected: no grasp poses provided");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(node_->get_logger(), "Goal received: %zu pose(s)", goal->grasp_poses.size());
    busy_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMtcPick>)
  {
    RCLCPP_WARN(node_->get_logger(), "Cancel requested; will stop at next safe checkpoint");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMtcPick> goal_handle)
  {
    if (execution_thread_.joinable()) execution_thread_.join();
    execution_thread_ = std::thread(&MtcPickServer::execute, this, goal_handle);
  }

  void publish_status(const std::shared_ptr<GoalHandleMtcPick> & gh, const std::string & s)
  {
    auto fb = std::make_shared<MtcPick::Feedback>();
    fb->status = s;
    gh->publish_feedback(fb);
  }

  // Geometric prefilter: removes poses that are below the table, out of reach,
  // or whose approach axis is too tilted. Preserves original indices.
  // Assumes poses are in fp3_link0 frame and that local +Z is the approach direction.
  std::vector<FilteredPose> filterPoses(
    const std::vector<geometry_msgs::msg::PoseStamped> & poses)
  {
    std::vector<FilteredPose> kept;
    const double max_tilt_rad = max_approach_tilt_deg_ * M_PI / 180.0;

    for (size_t i = 0; i < poses.size(); ++i) {
      const auto & p = poses[i].pose;

      if (p.position.z < table_top_z_ + min_height_above_table_) {
        RCLCPP_DEBUG(
          node_->get_logger(),
          "Pose %zu filtered: too close to table (z=%.3f)", i, p.position.z);
        continue;
      }

      const double reach = std::hypot(p.position.x, p.position.y, p.position.z);
      if (reach > max_reach_) {
        RCLCPP_DEBUG(
          node_->get_logger(),
          "Pose %zu filtered: out of reach (%.2fm > %.2fm)", i, reach, max_reach_);
        continue;
      }

      Eigen::Quaterniond q(
        p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
      Eigen::Vector3d approach_axis = q * Eigen::Vector3d::UnitZ();
      const double tilt = std::acos(std::clamp(-approach_axis.z(), -1.0, 1.0));
      // Temporary INFO-level logging to diagnose GraspGen's approach-axis
      // convention against fp3_hand_tcp (+Z expected) -- see CLAUDE.md.
      RCLCPP_INFO(
        node_->get_logger(),
        "Pose %zu: pos=(%.3f, %.3f, %.3f) reach=%.2fm tilt=%.1fdeg", i,
        p.position.x, p.position.y, p.position.z, reach, tilt * 180.0 / M_PI);
      if (tilt > max_tilt_rad) {
        RCLCPP_WARN(
          node_->get_logger(),
          "Pose %zu filtered: approach too tilted (%.1f deg > %.1f deg)", i,
          tilt * 180.0 / M_PI, max_approach_tilt_deg_);
        continue;
      }

      kept.push_back({static_cast<int>(i), poses[i]});
    }

    RCLCPP_INFO(
      node_->get_logger(), "Filter: %zu/%zu poses kept", kept.size(), poses.size());
    return kept;
  }

  // Attaches object_id to tcp_frame in the planning scene so that lift is
  // planned with the object's collision geometry included.
  bool attachObject()
  {
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name = mtc_params_.tcp_frame;
    attached.object.header.frame_id = mtc_params_.tcp_frame;
    attached.object.id = mtc_params_.object_id;
    attached.object.operation = attached.object.ADD;
    attached.touch_links = mtc_params_.hand_touch_links;

    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = {
      object_dimensions_.at(0),
      object_dimensions_.at(1),
      object_dimensions_.at(2)};
    attached.object.primitives.push_back(box);

    geometry_msgs::msg::Pose identity;
    identity.orientation.w = 1.0;
    attached.object.primitive_poses.push_back(identity);

    moveit_msgs::msg::PlanningScene diff;
    diff.is_diff = true;
    diff.robot_state.is_diff = true;
    diff.robot_state.attached_collision_objects.push_back(attached);

    if (!scene_client_->wait_for_service(std::chrono::seconds(10))) {
      RCLCPP_ERROR(node_->get_logger(), "/apply_planning_scene unavailable");
      return false;
    }
    auto request = std::make_shared<ApplyPlanningScene::Request>();
    request->scene = diff;
    return scene_client_->async_send_request(request).get()->success;
  }

  void execute(const std::shared_ptr<GoalHandleMtcPick> goal_handle)
  {
    BusyGuard guard{busy_};
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<MtcPick::Result>();
    result->used_pose_index = -1;

    // 1. Geometric prefilter
    publish_status(goal_handle, "filtering");
    std::vector<FilteredPose> filtered = filterPoses(goal->grasp_poses);
    if (filtered.empty()) {
      result->success = false;
      result->message = "0/" + std::to_string(goal->grasp_poses.size()) +
        " candidate(s) passed the geometric filter (table height / reach / approach tilt)";
      RCLCPP_WARN(node_->get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }

    // 2. Open gripper once before the approach loop
    publish_status(goal_handle, "opening");
    if (!gripper_->open(open_width_, open_speed_)) {
      result->success = false;
      result->message = "Gripper failed to open";
      RCLCPP_ERROR(node_->get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }

    // 3. Try each candidate in order
    bool approach_ok = false;
    int used_index = -1;
    for (const auto & candidate : filtered) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        result->message = "Canceled";
        goal_handle->canceled(result);
        return;
      }
      publish_status(goal_handle, "approaching");
      RCLCPP_INFO(node_->get_logger(), "Trying pose %d...", candidate.original_index);
      if (planAndExecuteApproach(node_, mtc_params_, approach_params_, candidate.pose)) {
        approach_ok = true;
        used_index = candidate.original_index;
        break;
      }
      RCLCPP_WARN(
        node_->get_logger(), "Pose %d failed, trying next", candidate.original_index);
    }

    if (!approach_ok) {
      result->success = false;
      result->message = "No pose was reachable/collision-free";
      RCLCPP_WARN(node_->get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }
    result->used_pose_index = used_index;

    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Canceled";
      goal_handle->canceled(result);
      return;
    }

    // 4. Close gripper
    publish_status(goal_handle, "grasping");
    if (!gripper_->close(
        grasp_width_, grasp_speed_, grasp_force_,
        grasp_epsilon_inner_, grasp_epsilon_outer_))
    {
      result->success = false;
      result->message = "Gripper failed to close";
      RCLCPP_ERROR(node_->get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }

    // 5. Attach object to planning scene for collision-aware lift
    publish_status(goal_handle, "attaching");
    if (!attachObject()) {
      result->success = false;
      result->message = "Failed to attach object to planning scene";
      RCLCPP_ERROR(node_->get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }

    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Canceled";
      goal_handle->canceled(result);
      return;
    }

    // 6. Lift along world Z+
    publish_status(goal_handle, "lifting");
    if (!planAndExecuteLift(node_, mtc_params_, lift_params_)) {
      result->success = false;
      result->message = "Lift failed (object remains attached)";
      RCLCPP_ERROR(node_->get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }

    result->success = true;
    result->message = "Pick executed successfully";
    RCLCPP_INFO(
      node_->get_logger(), "%s (pose index %d)",
      result->message.c_str(), used_index);
    goal_handle->succeed(result);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("pick_place_node");

  // MoveGroupInterface makes synchronous calls at construction: spin first.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  // Fetch robot_description/robot_description_semantic from /move_group so
  // MTC's Task::loadRobotModel() uses the same model move_group has.
  auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(node, "move_group");
  RCLCPP_INFO(node->get_logger(), "Waiting for move_group parameter server...");
  if (!param_client->wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_FATAL(node->get_logger(), "move_group unavailable after 10s, aborting");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  auto results =
    param_client->get_parameters({"robot_description", "robot_description_semantic"}).get();
  for (const auto & param : results) {
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      RCLCPP_FATAL(
        node->get_logger(),
        "Parameter '%s' not set on /move_group", param.get_name().c_str());
      executor.cancel();
      spin_thread.join();
      rclcpp::shutdown();
      return 1;
    }
  }
  for (const auto & param : results) {
    node->declare_parameter<std::string>(param.get_name(), param.as_string());
  }
  RCLCPP_INFO(node->get_logger(), "Robot model retrieved from move_group");

  // Gate the action server on a valid /joint_states: MTC's CurrentState stage
  // needs move_group's current_state_monitor to have received at least one message.
  std::atomic<bool> joint_state_received{false};
  auto joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10,
    [&joint_state_received](const sensor_msgs::msg::JointState::SharedPtr msg) {
      if (!msg->name.empty()) joint_state_received = true;
    });
  RCLCPP_INFO(node->get_logger(), "Waiting for a valid /joint_states message...");
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(15);
  while (!joint_state_received.load() &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (!joint_state_received.load()) {
    RCLCPP_FATAL(node->get_logger(), "No valid /joint_states received after 15s, aborting");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }
  joint_state_sub.reset();
  RCLCPP_INFO(node->get_logger(), "Current robot state is valid");

  MtcPickServer server(node);

  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
