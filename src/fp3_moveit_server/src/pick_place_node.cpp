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

#include "moveit/task_constructor/task.h"
#include "moveit/task_constructor/stages/current_state.h"
#include "moveit/task_constructor/stages/connect.h"
#include "moveit/task_constructor/stages/move_relative.h"
#include "moveit/task_constructor/stages/fixed_cartesian_poses.h"
#include "moveit/task_constructor/stages/compute_ik.h"
#include "moveit/task_constructor/stages/modify_planning_scene.h"
#include "moveit/task_constructor/solvers/pipeline_planner.h"
#include "moveit/task_constructor/solvers/cartesian_path.h"
#include "moveit/trajectory_processing/time_optimal_trajectory_generation.hpp"

#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"
#include "moveit_msgs/srv/apply_planning_scene.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

#include "franka_msgs/action/grasp.hpp"
#include "franka_msgs/action/move.hpp"
#include "franka_demo_interfaces/action/mtc_pick.hpp"

namespace mtc = moveit::task_constructor;
using MtcPick = franka_demo_interfaces::action::MtcPick;
using GoalHandleMtcPick = rclcpp_action::ServerGoalHandle<MtcPick>;
using Grasp = franka_msgs::action::Grasp;
using Move = franka_msgs::action::Move;
using ApplyPlanningScene = moveit_msgs::srv::ApplyPlanningScene;

namespace
{
struct FilteredPose
{
  int original_index;
  geometry_msgs::msg::PoseStamped pose;
};
}  // namespace

// MTC-based pick node: given an ordered list of grasp poses, runs a geometric
// prefilter, opens the gripper once, then tries each surviving pose in order:
// MTC approach (Connect + cartesian approach + IK to grasp pose), gripper
// close via franka_gripper/Grasp, planning scene attach, MTC cartesian lift
// (world Z+). Stops at the first pose that completes the full sequence.
//
// Gripper actions (open/close) are plain rclcpp_action calls run BETWEEN MTC
// Tasks -- never inside a Task stage, which would fire during task.plan()
// (candidate exploration) rather than task.execute() (actual execution).
class MtcPickServer
{
public:
  explicit MtcPickServer(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
    planning_group_ = node_->declare_parameter<std::string>("planning_group", "fp3_arm");
    eef_name_ = node_->declare_parameter<std::string>("eef_name", "fp3_hand");
    tcp_frame_ = node_->declare_parameter<std::string>("tcp_frame", "fp3_hand_tcp");
    hand_touch_links_ = node_->declare_parameter<std::vector<std::string>>(
      "hand_touch_links", {"fp3_hand", "fp3_leftfinger", "fp3_rightfinger"});
    gripper_action_name_ = node_->declare_parameter<std::string>(
      "gripper_action_name", "/franka_gripper/grasp");
    simulate_gripper_ = node_->declare_parameter<bool>("simulate_gripper", false);

    min_height_above_table_ = node_->declare_parameter<double>("filter.min_height_above_table", 0.02);
    max_approach_tilt_deg_ = node_->declare_parameter<double>("filter.max_approach_tilt_deg", 45.0);
    max_reach_ = node_->declare_parameter<double>("filter.max_reach", 0.85);

    approach_min_distance_ = node_->declare_parameter<double>("approach.min_distance", 0.02);
    approach_max_distance_ = node_->declare_parameter<double>("approach.max_distance", 0.10);

    lift_min_distance_ = node_->declare_parameter<double>("lift.min_distance", 0.05);
    lift_max_distance_ = node_->declare_parameter<double>("lift.max_distance", 0.15);

    grasp_width_ = node_->declare_parameter<double>("grasp.width", 0.0);
    grasp_epsilon_inner_ = node_->declare_parameter<double>("grasp.epsilon_inner", 0.005);
    grasp_epsilon_outer_ = node_->declare_parameter<double>("grasp.epsilon_outer", 0.005);
    grasp_speed_ = node_->declare_parameter<double>("grasp.speed", 0.05);
    grasp_force_ = node_->declare_parameter<double>("grasp.force", 20.0);

    move_action_name_ = node_->declare_parameter<std::string>(
      "gripper_move_action_name", "/franka_gripper/move");
    open_width_ = node_->declare_parameter<double>("open.width", 0.08);
    open_speed_ = node_->declare_parameter<double>("open.speed", 0.1);

    object_id_ = node_->declare_parameter<std::string>("object.id", "picked_object");
    object_dimensions_ = node_->declare_parameter<std::vector<double>>(
      "object.dimensions", {0.03, 0.03, 0.03});

    velocity_scaling_factor_ = node_->declare_parameter<double>("velocity_scaling_factor", 0.1);
    acceleration_scaling_factor_ =
      node_->declare_parameter<double>("acceleration_scaling_factor", 0.1);

    const std::vector<double> table_position =
      node_->declare_parameter<std::vector<double>>("table.position", {0.10, -0.50, -0.05});
    const std::vector<double> table_dimensions =
      node_->declare_parameter<std::vector<double>>("table.dimensions", {0.60, 1.40, 0.10});
    node_->declare_parameter<std::string>("table.frame_id", "fp3_link0");
    table_top_z_ = table_position.at(2) + table_dimensions.at(2) / 2.0;

    grasp_client_ = rclcpp_action::create_client<Grasp>(node_, gripper_action_name_);
    move_client_ = rclcpp_action::create_client<Move>(node_, move_action_name_);
    scene_client_ = node_->create_client<ApplyPlanningScene>("apply_planning_scene");

    action_server_ = rclcpp_action::create_server<MtcPick>(
      node_,
      "mtc_pick",
      std::bind(&MtcPickServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MtcPickServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&MtcPickServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(
      node_->get_logger(), "pick_place_node ready (group '%s', object '%s')",
      planning_group_.c_str(), object_id_.c_str());
  }

  ~MtcPickServer()
  {
    if (execution_thread_.joinable()) execution_thread_.join();
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<MtcPick>::SharedPtr action_server_;
  rclcpp_action::Client<Grasp>::SharedPtr grasp_client_;
  rclcpp_action::Client<Move>::SharedPtr move_client_;
  rclcpp::Client<ApplyPlanningScene>::SharedPtr scene_client_;
  std::thread execution_thread_;
  std::atomic<bool> busy_{false};

  std::string planning_group_;
  std::string eef_name_;
  std::string tcp_frame_;
  std::vector<std::string> hand_touch_links_;
  std::string gripper_action_name_;
  bool simulate_gripper_;

  double min_height_above_table_;
  double max_approach_tilt_deg_;
  double max_reach_;
  double table_top_z_;

  double approach_min_distance_;
  double approach_max_distance_;
  double lift_min_distance_;
  double lift_max_distance_;

  double grasp_width_;
  double grasp_epsilon_inner_;
  double grasp_epsilon_outer_;
  double grasp_speed_;
  double grasp_force_;

  std::string move_action_name_;
  double open_width_;
  double open_speed_;

  std::string object_id_;
  std::vector<double> object_dimensions_;

  double velocity_scaling_factor_;
  double acceleration_scaling_factor_;

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
  // or whose approach axis is too tilted. Preserves original indices so that
  // used_pose_index in the result refers to the caller's original list.
  // Assumes grasp_poses are already expressed in fp3_link0 frame and that the
  // pose's local +Z is the gripper approach direction (TCP convention).
  std::vector<FilteredPose> filterPoses(
    const std::vector<geometry_msgs::msg::PoseStamped> & poses)
  {
    std::vector<FilteredPose> kept;
    const double max_tilt_rad = max_approach_tilt_deg_ * M_PI / 180.0;

    for (size_t i = 0; i < poses.size(); ++i) {
      const auto & p = poses[i].pose;

      if (p.position.z < table_top_z_ + min_height_above_table_) {
        RCLCPP_DEBUG(node_->get_logger(), "Pose %zu filtered: too close to table (z=%.3f)", i, p.position.z);
        continue;
      }

      const double reach = std::hypot(p.position.x, p.position.y, p.position.z);
      if (reach > max_reach_) {
        RCLCPP_DEBUG(node_->get_logger(), "Pose %zu filtered: out of reach (%.2fm > %.2fm)", i, reach, max_reach_);
        continue;
      }

      Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
      Eigen::Vector3d approach_axis = q * Eigen::Vector3d::UnitZ();
      const double tilt = std::acos(std::clamp(-approach_axis.z(), -1.0, 1.0));
      // Temporary INFO-level logging (normally DEBUG) to diagnose whether
      // GraspGen's raw orientation convention matches fp3_hand_tcp's
      // expected approach axis (+Z) -- see CLAUDE.md, "Approach-axis
      // convention assumption", never verified against real GraspGen output.
      RCLCPP_INFO(
        node_->get_logger(), "Pose %zu: pos=(%.3f, %.3f, %.3f) reach=%.2fm tilt=%.1fdeg", i,
        p.position.x, p.position.y, p.position.z, reach, tilt * 180.0 / M_PI);
      if (tilt > max_tilt_rad) {
        RCLCPP_WARN(
          node_->get_logger(), "Pose %zu filtered: approach too tilted (%.1f deg > %.1f deg)", i,
          tilt * 180.0 / M_PI, max_approach_tilt_deg_);
        continue;
      }

      kept.push_back({static_cast<int>(i), poses[i]});
    }

    RCLCPP_INFO(
      node_->get_logger(), "Filter: %zu/%zu poses kept", kept.size(), poses.size());
    return kept;
  }

  // MTC Task: free-space motion to pre-grasp position (Connect via OMPL),
  // then cartesian approach along TCP Z+, arriving exactly at the grasp pose.
  // After successful execute(), the TCP is at the grasp pose.
  bool planAndExecuteApproach(const geometry_msgs::msg::PoseStamped & pose)
  {
    mtc::Task task;
    task.setName("mtc_pick approach");
    task.loadRobotModel(node_);

    task.add(std::make_unique<mtc::stages::CurrentState>("current state"));
    auto * current_state_ptr = task.stages()->findChild("current state");

    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "move_group");
    sampling_planner->setMaxVelocityScalingFactor(velocity_scaling_factor_);
    sampling_planner->setMaxAccelerationScalingFactor(acceleration_scaling_factor_);
    mtc::stages::Connect::GroupPlannerVector planners = {{planning_group_, sampling_planner}};
    task.add(std::make_unique<mtc::stages::Connect>("connect", planners));

    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(velocity_scaling_factor_);
    cartesian_planner->setMaxAccelerationScalingFactor(acceleration_scaling_factor_);
    // TOTG ensures the cartesian segment ends at zero velocity, required by
    // fp3_arm_controller (JointTrajectoryController rejects nonzero terminal
    // velocity by default -- see config/controller_overrides.yaml).
    cartesian_planner->setTimeParameterization(
      std::make_shared<trajectory_processing::TimeOptimalTrajectoryGeneration>());

    auto approach = std::make_unique<mtc::stages::MoveRelative>("approach", cartesian_planner);
    approach->setGroup(planning_group_);
    approach->setIKFrame(tcp_frame_);
    approach->setMinMaxDistance(approach_min_distance_, approach_max_distance_);
    geometry_msgs::msg::Vector3Stamped approach_dir;
    approach_dir.header.frame_id = tcp_frame_;
    approach_dir.vector.z = 1.0;
    approach->setDirection(approach_dir);
    task.add(std::move(approach));

    auto pose_generator = std::make_unique<mtc::stages::FixedCartesianPoses>("grasp pose");
    pose_generator->addPose(pose);
    pose_generator->setMonitoredStage(current_state_ptr);

    auto ik = std::make_unique<mtc::stages::ComputeIK>("grasp IK", std::move(pose_generator));
    ik->setGroup(planning_group_);
    ik->setEndEffector(eef_name_);
    ik->setIKFrame(tcp_frame_);
    ik->setMaxIKSolutions(4);
    // FixedCartesianPoses sets "target_pose" on its InterfaceState; ComputeIK
    // only reads it from there if told to pull from the interface.
    ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
    task.add(std::move(ik));

    auto allow_collision =
      std::make_unique<mtc::stages::ModifyPlanningScene>("allow hand/object collision");
    allow_collision->allowCollisions(object_id_, hand_touch_links_, true);
    task.add(std::move(allow_collision));

    if (task.plan(1) != moveit::core::MoveItErrorCode::SUCCESS || task.numSolutions() == 0) {
      return false;
    }
    return task.execute(*task.solutions().front()) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  // MTC Task: cartesian lift along world Z+ (fp3_link0 is fixed = world).
  bool planAndExecuteLift()
  {
    mtc::Task task;
    task.setName("mtc_pick lift");
    task.loadRobotModel(node_);

    task.add(std::make_unique<mtc::stages::CurrentState>("current state"));

    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(velocity_scaling_factor_);
    cartesian_planner->setMaxAccelerationScalingFactor(acceleration_scaling_factor_);
    cartesian_planner->setTimeParameterization(
      std::make_shared<trajectory_processing::TimeOptimalTrajectoryGeneration>());

    auto lift = std::make_unique<mtc::stages::MoveRelative>("lift", cartesian_planner);
    lift->setGroup(planning_group_);
    lift->setIKFrame(tcp_frame_);
    lift->setMinMaxDistance(lift_min_distance_, lift_max_distance_);
    geometry_msgs::msg::Vector3Stamped lift_dir;
    lift_dir.header.frame_id = "fp3_link0";  // fixed base frame = world Z+
    lift_dir.vector.z = 1.0;
    lift->setDirection(lift_dir);
    task.add(std::move(lift));

    if (task.plan(1) != moveit::core::MoveItErrorCode::SUCCESS || task.numSolutions() == 0) {
      return false;
    }
    return task.execute(*task.solutions().front()) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool openGripper()
  {
    if (simulate_gripper_) {
      RCLCPP_INFO(node_->get_logger(), "Opening gripper (simulated): width=%.3f", open_width_);
      return true;
    }
    if (!move_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(node_->get_logger(), "Gripper move action '%s' unavailable", move_action_name_.c_str());
      return false;
    }
    auto goal = Move::Goal();
    goal.width = open_width_;
    goal.speed = open_speed_;
    auto gh = move_client_->async_send_goal(goal).get();
    if (!gh) {
      RCLCPP_ERROR(node_->get_logger(), "Gripper open goal rejected");
      return false;
    }
    auto result = move_client_->async_get_result(gh).get();
    return result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success;
  }

  bool closeGripper()
  {
    if (simulate_gripper_) {
      RCLCPP_INFO(node_->get_logger(), "Closing gripper (simulated): width=%.3f", grasp_width_);
      return true;
    }
    if (!grasp_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(
        node_->get_logger(), "Gripper grasp action '%s' unavailable", gripper_action_name_.c_str());
      return false;
    }
    auto goal = Grasp::Goal();
    goal.width = grasp_width_;
    goal.epsilon.inner = grasp_epsilon_inner_;
    goal.epsilon.outer = grasp_epsilon_outer_;
    goal.speed = grasp_speed_;
    goal.force = grasp_force_;
    auto gh = grasp_client_->async_send_goal(goal).get();
    if (!gh) {
      RCLCPP_ERROR(node_->get_logger(), "Grasp goal rejected");
      return false;
    }
    auto result = grasp_client_->async_get_result(gh).get();
    return result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success;
  }

  // Attaches object_id_ to tcp_frame_ in the planning scene so that the lift
  // motion is planned with the object's collision geometry included.
  bool attachObject()
  {
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name = tcp_frame_;
    attached.object.header.frame_id = tcp_frame_;
    attached.object.id = object_id_;
    attached.object.operation = attached.object.ADD;
    attached.touch_links = hand_touch_links_;

    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = {object_dimensions_.at(0), object_dimensions_.at(1), object_dimensions_.at(2)};
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

    // 1. Geometric prefilter: drop poses below the table, out of reach, or
    // whose approach axis is too tilted (cf. filterPoses()).
    std::vector<FilteredPose> filtered = filterPoses(goal->grasp_poses);
    publish_status(goal_handle, "filtering");
    if (filtered.empty()) {
      result->success = false;
      result->message = "0/" + std::to_string(goal->grasp_poses.size()) +
        " candidate(s) passed the geometric filter (table height / reach / approach tilt)";
      RCLCPP_WARN(node_->get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }

    // 2. Open gripper once before the approach loop.
    publish_status(goal_handle, "opening");
    if (!openGripper()) {
      result->success = false;
      result->message = "Gripper failed to open";
      RCLCPP_ERROR(node_->get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }

    // 3. Try each candidate in order; gripper is still open if approach fails.
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
      if (planAndExecuteApproach(candidate.pose)) {
        approach_ok = true;
        used_index = candidate.original_index;
        break;
      }
      RCLCPP_WARN(node_->get_logger(), "Pose %d failed, trying next", candidate.original_index);
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

    // 4. Close gripper (only reached after a successful approach).
    publish_status(goal_handle, "grasping");
    if (!closeGripper()) {
      result->success = false;
      result->message = "Gripper failed to close";
      RCLCPP_ERROR(node_->get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }

    // 5. Attach object to planning scene for collision-aware lift.
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

    // 6. Lift along world Z+.
    publish_status(goal_handle, "lifting");
    if (!planAndExecuteLift()) {
      result->success = false;
      result->message = "Lift failed (object remains attached)";
      RCLCPP_ERROR(node_->get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }

    result->success = true;
    result->message = "Pick executed successfully";
    RCLCPP_INFO(
      node_->get_logger(), "%s (pose index %d)", result->message.c_str(), used_index);
    goal_handle->succeed(result);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("pick_place_node");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  // Fetch robot_description/robot_description_semantic from /move_group so
  // MTC's Task::loadRobotModel() uses the same model the real move_group has.
  auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(node, "move_group");
  RCLCPP_INFO(node->get_logger(), "Waiting for move_group parameter server...");
  if (!param_client->wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_FATAL(node->get_logger(), "move_group unavailable after 10s, aborting");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  auto results = param_client->get_parameters(
    {"robot_description", "robot_description_semantic"}).get();
  for (const auto & param : results) {
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      RCLCPP_FATAL(
        node->get_logger(), "Parameter '%s' not set on /move_group", param.get_name().c_str());
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

  // Gate action server on valid /joint_states: MTC's CurrentState stage needs
  // move_group's current_state_monitor to have received at least one message.
  std::atomic<bool> joint_state_received{false};
  auto joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10,
    [&joint_state_received](const sensor_msgs::msg::JointState::SharedPtr msg) {
      if (!msg->name.empty()) joint_state_received = true;
    });
  RCLCPP_INFO(node->get_logger(), "Waiting for a valid /joint_states message...");
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
  while (!joint_state_received.load() && std::chrono::steady_clock::now() < deadline) {
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
