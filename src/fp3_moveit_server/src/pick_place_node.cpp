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
#include "franka_demo_interfaces/action/pick_object.hpp"

namespace mtc = moveit::task_constructor;
using PickObject = franka_demo_interfaces::action::PickObject;
using GoalHandlePickObject = rclcpp_action::ServerGoalHandle<PickObject>;
using Grasp = franka_msgs::action::Grasp;
using Move = franka_msgs::action::Move;
using ApplyPlanningScene = moveit_msgs::srv::ApplyPlanningScene;

namespace
{
struct FilteredCandidate
{
  int original_index;
  geometry_msgs::msg::PoseStamped pose;
};
}  // namespace

// MTC-based pick AND place: given a list of grasp pose candidates, opens the
// gripper, approaches and grasps the first reachable/collision-free
// candidate, attaches the object, retreats, moves to a fixed place_pose_xyz_
// (the "ready" position), detaches the object there, and opens the gripper
// to release it. Multi-phase execution, deliberately NOT done as a single
// MTC Task with custom gripper stages: MTC stages compute() during
// task.plan(), before task.execute() -- a custom stage that called
// franka_gripper/Grasp or Move directly from compute() would physically
// move the gripper while MTC is merely exploring/pruning candidate
// branches, not when the chosen solution is actually executed. So every
// gripper action (open before approach, close to grasp, open to release) is
// a plain rclcpp_action call in this node's own code, run BETWEEN separate
// MTC Tasks (approach, retreat, place), at the correct real-world moment --
// never inside a Task's planning phase.
class PickPlaceServer
{
public:
  PickPlaceServer(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
    planning_group_ = node_->declare_parameter<std::string>("planning_group", "fp3_arm");
    eef_name_ = node_->declare_parameter<std::string>("eef_name", "fp3_hand");
    tcp_frame_ = node_->declare_parameter<std::string>("tcp_frame", "fp3_hand_tcp");
    hand_touch_links_ = node_->declare_parameter<std::vector<std::string>>(
      "hand_touch_links", {"fp3_hand", "fp3_leftfinger", "fp3_rightfinger"});
    gripper_action_name_ =
      node_->declare_parameter<std::string>("gripper_action_name", "/franka_gripper/grasp");
    // franka_gripper_node (which hosts the real Grasp action) isn't even
    // launched in fake hardware mode -- only fake_gripper_state_publisher.py
    // runs there, which isn't an action server. Rather than always failing
    // at the grasp step in fake mode, bringup.launch.py wires this to
    // use_fake_hardware so the gripper phase is simulated with a log line.
    simulate_gripper_ = node_->declare_parameter<bool>("simulate_gripper", false);

    min_height_above_table_ = node_->declare_parameter<double>("filter.min_height_above_table", 0.02);
    max_approach_tilt_deg_ = node_->declare_parameter<double>("filter.max_approach_tilt_deg", 45.0);
    max_reach_ = node_->declare_parameter<double>("filter.max_reach", 0.85);

    approach_min_distance_ = node_->declare_parameter<double>("approach.min_distance", 0.02);
    approach_max_distance_ = node_->declare_parameter<double>("approach.max_distance", 0.10);
    retreat_min_distance_ = node_->declare_parameter<double>("retreat.min_distance", 0.05);
    retreat_max_distance_ = node_->declare_parameter<double>("retreat.max_distance", 0.15);

    grasp_width_ = node_->declare_parameter<double>("grasp.width", 0.0);
    grasp_epsilon_inner_ = node_->declare_parameter<double>("grasp.epsilon_inner", 0.005);
    grasp_epsilon_outer_ = node_->declare_parameter<double>("grasp.epsilon_outer", 0.005);
    grasp_speed_ = node_->declare_parameter<double>("grasp.speed", 0.05);
    grasp_force_ = node_->declare_parameter<double>("grasp.force", 20.0);

    move_action_name_ =
      node_->declare_parameter<std::string>("gripper_move_action_name", "/franka_gripper/move");
    open_width_ = node_->declare_parameter<double>("open.width", 0.08);
    open_speed_ = node_->declare_parameter<double>("open.speed", 0.1);

    // TO ADJUST: placeholder place location -- no guarantee it's
    // reachable/collision-free in your actual scene. Same straight-down
    // orientation convention as the grasp filter (local +Z pointing world
    // -Z) is used here too, not the object's own orientation.
    place_pose_xyz_ = node_->declare_parameter<std::vector<double>>(
      "place.pose_xyz", {0.4, 0.0, 0.4});

    // Safety default: cap speed at 10% of the robot's max, everywhere this
    // node plans a motion (both the PipelinePlanner and CartesianPath
    // solvers below). Override via parameters if you actually need more.
    velocity_scaling_factor_ = node_->declare_parameter<double>("velocity_scaling_factor", 0.1);
    acceleration_scaling_factor_ =
      node_->declare_parameter<double>("acceleration_scaling_factor", 0.1);

    table_frame_ = node_->declare_parameter<std::string>("table.frame_id", "fp3_link0");
    const std::vector<double> table_position =
      node_->declare_parameter<std::vector<double>>("table.position", {0.5, 0.0, -0.05});
    const std::vector<double> table_dimensions =
      node_->declare_parameter<std::vector<double>>("table.dimensions", {1.2, 0.8, 0.1});
    table_top_z_ = table_position.at(2) + table_dimensions.at(2) / 2.0;

    grasp_client_ = rclcpp_action::create_client<Grasp>(node_, gripper_action_name_);
    move_client_ = rclcpp_action::create_client<Move>(node_, move_action_name_);
    scene_client_ = node_->create_client<ApplyPlanningScene>("apply_planning_scene");

    action_server_ = rclcpp_action::create_server<PickObject>(
      node_,
      "pick_object",
      std::bind(&PickPlaceServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PickPlaceServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&PickPlaceServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(
      node_->get_logger(), "pick_place_node ready (planning group '%s', eef '%s')",
      planning_group_.c_str(), eef_name_.c_str());
  }

  ~PickPlaceServer()
  {
    if (execution_thread_.joinable()) {
      execution_thread_.join();
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<PickObject>::SharedPtr action_server_;
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
  double approach_min_distance_;
  double approach_max_distance_;
  double retreat_min_distance_;
  double retreat_max_distance_;
  double grasp_width_;
  double grasp_epsilon_inner_;
  double grasp_epsilon_outer_;
  double grasp_speed_;
  double grasp_force_;

  std::string move_action_name_;
  double open_width_;
  double open_speed_;
  std::vector<double> place_pose_xyz_;

  double velocity_scaling_factor_;
  double acceleration_scaling_factor_;

  std::string table_frame_;
  double table_top_z_;

  struct BusyGuard
  {
    std::atomic<bool> & flag;
    ~BusyGuard() {flag = false;}
  };

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const PickObject::Goal> goal)
  {
    if (busy_.load()) {
      RCLCPP_WARN(node_->get_logger(), "Goal rejected: another pick is already in progress");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->grasp_candidates.empty()) {
      RCLCPP_WARN(node_->get_logger(), "Goal rejected: no grasp candidates provided");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(
      node_->get_logger(), "Goal received: object '%s', %zu candidate(s)",
      goal->object_id.c_str(), goal->grasp_candidates.size());
    busy_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandlePickObject>)
  {
    // Best-effort: checked between phases (filter/plan/grasp/attach/retreat)
    // in execute(). A phase already in flight (an MTC plan/execute call, or
    // an in-progress gripper close) is NOT interrupted -- stopping a grasp
    // mid-close is its own hazard, and MTC's synchronous plan()/execute()
    // calls have no interrupt hook exposed here. Same class of limitation
    // as motion_server_node's un-timed execute().
    RCLCPP_WARN(node_->get_logger(), "Cancel requested; will stop at the next safe checkpoint");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandlePickObject> goal_handle)
  {
    if (execution_thread_.joinable()) {
      execution_thread_.join();
    }
    execution_thread_ = std::thread(&PickPlaceServer::execute, this, goal_handle);
  }

  void publish_status(const std::shared_ptr<GoalHandlePickObject> & goal_handle, const std::string & stage)
  {
    auto feedback = std::make_shared<PickObject::Feedback>();
    feedback->current_stage = stage;
    goal_handle->publish_feedback(feedback);
  }

  // Cheap geometric prefilter, run before any MTC planning. Assumes
  // grasp_candidates are expressed in the same frame the planner works in
  // (fp3_link0, matching move_to_pose's convention elsewhere in this
  // codebase) -- if GraspGen actually emits poses in a camera frame, a TF
  // transform step needs to be added here first. Also assumes each
  // candidate's local +Z axis is the gripper's approach/insertion
  // direction -- verify against GraspGen's actual convention before relying
  // on this filter.
  std::vector<FilteredCandidate> filterCandidates(
    const std::vector<geometry_msgs::msg::PoseStamped> & candidates)
  {
    std::vector<FilteredCandidate> kept;
    const double max_tilt_rad = max_approach_tilt_deg_ * M_PI / 180.0;

    for (size_t i = 0; i < candidates.size(); ++i) {
      const auto & pose = candidates[i].pose;

      if (pose.position.z < table_top_z_ + min_height_above_table_) {
        continue;
      }

      const double reach = std::hypot(pose.position.x, pose.position.y, pose.position.z);
      if (reach > max_reach_) {
        continue;
      }

      Eigen::Quaterniond q(
        pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
      Eigen::Vector3d approach_axis = q * Eigen::Vector3d::UnitZ();
      const double tilt_from_down_rad = std::acos(
        std::clamp(-approach_axis.z(), -1.0, 1.0));
      if (tilt_from_down_rad > max_tilt_rad) {
        continue;
      }

      kept.push_back({static_cast<int>(i), candidates[i]});
    }
    return kept;
  }

  // Builds and plans a fresh MTC task for a single grasp candidate: connect
  // from current state, cartesian approach, IK for that one pose, allow
  // hand/object collision. Returns true and executes it if planning
  // succeeds; false (task discarded) otherwise so the caller can try the
  // next candidate.
  bool planAndExecuteApproach(
    const geometry_msgs::msg::PoseStamped & candidate, const std::string & object_id)
  {
    mtc::Task task;
    task.setName("pick approach");
    task.loadRobotModel(node_);

    task.add(std::make_unique<mtc::stages::CurrentState>("current state"));
    auto * current_state_ptr = task.stages()->findChild("current state");

    // "move_group" here is a parameter namespace, not a node reference: the
    // ompl_planning_pipeline_config dict loaded onto this node from
    // bringup.launch.py is nested under a 'move_group' key (copied as-is
    // from franka_fp3_moveit_config/launch/moveit.launch.py, which loads
    // the identical dict onto the actual /move_group node under that same
    // namespace) -- PipelinePlanner must be given the matching namespace to
    // find planning_plugins/request_adapters/etc, not the "ompl" pipeline
    // label.
    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "move_group");
    sampling_planner->setMaxVelocityScalingFactor(velocity_scaling_factor_);
    sampling_planner->setMaxAccelerationScalingFactor(acceleration_scaling_factor_);
    mtc::stages::Connect::GroupPlannerVector planners = {{planning_group_, sampling_planner}};
    task.add(std::make_unique<mtc::stages::Connect>("move to pick", planners));

    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(velocity_scaling_factor_);
    cartesian_planner->setMaxAccelerationScalingFactor(acceleration_scaling_factor_);
    // Without this, CartesianPath's raw interpolated trajectory can leave a
    // tiny nonzero terminal velocity (observed: ~0.0019 rad/s on one
    // joint), which fp3_arm_controller (a standard JointTrajectoryController)
    // rejects outright ("Velocity of last trajectory point ... is not
    // zero"). TOTG is the same algorithm move_group's own
    // AddTimeOptimalParameterization response adapter uses, and guarantees
    // the trajectory starts/ends at rest.
    cartesian_planner->setTimeParameterization(
      std::make_shared<trajectory_processing::TimeOptimalTrajectoryGeneration>());

    auto approach = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
    approach->setGroup(planning_group_);
    approach->setIKFrame(tcp_frame_);
    approach->setMinMaxDistance(approach_min_distance_, approach_max_distance_);
    geometry_msgs::msg::Vector3Stamped approach_direction;
    approach_direction.header.frame_id = tcp_frame_;
    approach_direction.vector.z = 1.0;
    approach->setDirection(approach_direction);
    task.add(std::move(approach));

    auto grasp_pose_generator = std::make_unique<mtc::stages::FixedCartesianPoses>("grasp candidate");
    grasp_pose_generator->addPose(candidate);
    grasp_pose_generator->setMonitoredStage(current_state_ptr);

    auto compute_ik =
      std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(grasp_pose_generator));
    compute_ik->setGroup(planning_group_);
    compute_ik->setEndEffector(eef_name_);
    compute_ik->setIKFrame(tcp_frame_);
    compute_ik->setMaxIKSolutions(4);
    // FixedCartesianPoses (the wrapped child) sets "target_pose" on the
    // InterfaceState it spawns; ComputeIK only reads it from there if told
    // to pull it from the interface -- without this, the property is
    // "declared but undefined" and MTC throws at plan() time.
    compute_ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
    task.add(std::move(compute_ik));

    auto allow_collision = std::make_unique<mtc::stages::ModifyPlanningScene>("allow hand/object collision");
    allow_collision->allowCollisions(object_id, hand_touch_links_, true);
    task.add(std::move(allow_collision));

    if (task.plan(1) != moveit::core::MoveItErrorCode::SUCCESS || task.numSolutions() == 0) {
      return false;
    }

    return task.execute(*task.solutions().front()) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool closeGripper()
  {
    if (simulate_gripper_) {
      RCLCPP_INFO(
        node_->get_logger(),
        "Closing gripper (simulated, use_fake_hardware): width=%.3f force=%.1f",
        grasp_width_, grasp_force_);
      return true;
    }

    if (!grasp_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(node_->get_logger(), "Gripper action '%s' unavailable", gripper_action_name_.c_str());
      return false;
    }

    auto goal = Grasp::Goal();
    goal.width = grasp_width_;
    goal.epsilon.inner = grasp_epsilon_inner_;
    goal.epsilon.outer = grasp_epsilon_outer_;
    goal.speed = grasp_speed_;
    goal.force = grasp_force_;

    // Capture the last feedback's current_width so we can double-check the
    // grasp actually closed on something roughly the expected size, not
    // just trust the driver's own success flag blindly.
    std::atomic<double> last_width{-1.0};
    rclcpp_action::Client<Grasp>::SendGoalOptions options;
    options.feedback_callback =
      [&last_width](
      rclcpp_action::ClientGoalHandle<Grasp>::SharedPtr,
      const std::shared_ptr<const Grasp::Feedback> feedback) {
        last_width.store(feedback->current_width);
      };

    auto goal_handle_future = grasp_client_->async_send_goal(goal, options);
    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "Gripper rejected the grasp goal");
      return false;
    }

    auto result = grasp_client_->async_get_result(goal_handle).get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result->success) {
      RCLCPP_WARN(
        node_->get_logger(), "Grasp action reported failure (final width=%.4f, expected=%.4f)",
        last_width.load(), grasp_width_);
      return false;
    }

    // Redundant on top of the driver's own epsilon check (which already
    // requires the final width to land within
    // [width-epsilon_inner, width+epsilon_outer] to report success), but
    // makes the "did we actually grasp something close to the expected
    // object" verification explicit and logged, not just implicit in a
    // boolean.
    const double width = last_width.load();
    const double tolerance = std::max(grasp_epsilon_inner_, grasp_epsilon_outer_);
    if (width >= 0.0 && std::abs(width - grasp_width_) > tolerance) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Grasp reported success but final width %.4f is not close to expected %.4f "
        "(tolerance %.4f) -- treating as a failed grasp",
        width, grasp_width_, tolerance);
      return false;
    }

    RCLCPP_INFO(
      node_->get_logger(), "Grasp succeeded: final width=%.4f (expected %.4f)",
      width, grasp_width_);
    return true;
  }

  // franka_gripper/Move, not Grasp: no force/epsilon control, just "go to
  // this width" -- used both before the approach (clear the fingers) and
  // after placing (release the object).
  bool openGripper()
  {
    if (simulate_gripper_) {
      RCLCPP_INFO(
        node_->get_logger(),
        "Opening gripper (simulated, use_fake_hardware): width=%.3f",
        open_width_);
      return true;
    }

    if (!move_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(node_->get_logger(), "Gripper action '%s' unavailable", move_action_name_.c_str());
      return false;
    }

    auto goal = Move::Goal();
    goal.width = open_width_;
    goal.speed = open_speed_;

    auto goal_handle_future = move_client_->async_send_goal(goal);
    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "Gripper rejected the open goal");
      return false;
    }

    auto result = move_client_->async_get_result(goal_handle).get();
    return result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success;
  }

  // Attaches the object to the TCP link via a direct planning scene diff
  // (no MTC task needed for a single scene edit). v1 simplification: since
  // PickObject only carries the object's bounding box, not its pre-grasp
  // pose in the scene, the attached geometry is centered on the TCP frame
  // rather than reflecting exactly where it sat before being grasped.
  bool attachObject(const std::string & object_id, const geometry_msgs::msg::Vector3 & dimensions)
  {
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name = tcp_frame_;
    attached.object.header.frame_id = tcp_frame_;
    attached.object.id = object_id;
    attached.object.operation = attached.object.ADD;
    attached.touch_links = hand_touch_links_;

    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = {dimensions.x, dimensions.y, dimensions.z};
    attached.object.primitives.push_back(box);

    geometry_msgs::msg::Pose identity_pose;
    identity_pose.orientation.w = 1.0;
    attached.object.primitive_poses.push_back(identity_pose);

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
    auto future = scene_client_->async_send_request(request);
    return future.get()->success;
  }

  // Reverse of attachObject(): removes the object from the robot state's
  // attached collision objects (it drops back into the world at the TCP's
  // current pose, then is removed from the world too -- v1 doesn't attempt
  // to leave a world-frame collision object behind at the release point).
  bool detachObject(const std::string & object_id)
  {
    moveit_msgs::msg::AttachedCollisionObject detached;
    detached.link_name = tcp_frame_;
    detached.object.id = object_id;
    detached.object.operation = detached.object.REMOVE;

    moveit_msgs::msg::CollisionObject world_remove;
    world_remove.id = object_id;
    world_remove.operation = world_remove.REMOVE;

    moveit_msgs::msg::PlanningScene diff;
    diff.is_diff = true;
    diff.robot_state.is_diff = true;
    diff.robot_state.attached_collision_objects.push_back(detached);
    diff.world.collision_objects.push_back(world_remove);

    if (!scene_client_->wait_for_service(std::chrono::seconds(10))) {
      RCLCPP_ERROR(node_->get_logger(), "/apply_planning_scene unavailable");
      return false;
    }
    auto request = std::make_shared<ApplyPlanningScene::Request>();
    request->scene = diff;
    auto future = scene_client_->async_send_request(request);
    return future.get()->success;
  }

  // Plans/executes straight to the fixed place_pose_xyz_ (an absolute
  // target, not a relative move like approach/retreat), via the same
  // Connect+FixedCartesianPoses+ComputeIK pattern planAndExecuteApproach()
  // uses for its grasp candidate.
  bool planAndExecutePlace()
  {
    mtc::Task task;
    task.setName("pick place");
    task.loadRobotModel(node_);

    task.add(std::make_unique<mtc::stages::CurrentState>("current state"));
    auto * current_state_ptr = task.stages()->findChild("current state");

    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "move_group");
    sampling_planner->setMaxVelocityScalingFactor(velocity_scaling_factor_);
    sampling_planner->setMaxAccelerationScalingFactor(acceleration_scaling_factor_);
    mtc::stages::Connect::GroupPlannerVector planners = {{planning_group_, sampling_planner}};
    task.add(std::make_unique<mtc::stages::Connect>("move to place", planners));

    geometry_msgs::msg::PoseStamped place_pose;
    place_pose.header.frame_id = table_frame_;
    place_pose.pose.position.x = place_pose_xyz_.at(0);
    place_pose.pose.position.y = place_pose_xyz_.at(1);
    place_pose.pose.position.z = place_pose_xyz_.at(2);
    // Same straight-down convention as the grasp filter (local +Z pointing
    // world -Z), not any particular object orientation.
    place_pose.pose.orientation.x = 1.0;
    place_pose.pose.orientation.w = 0.0;

    auto place_pose_generator = std::make_unique<mtc::stages::FixedCartesianPoses>("place pose");
    place_pose_generator->addPose(place_pose);
    place_pose_generator->setMonitoredStage(current_state_ptr);

    auto compute_ik =
      std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(place_pose_generator));
    compute_ik->setGroup(planning_group_);
    compute_ik->setEndEffector(eef_name_);
    compute_ik->setIKFrame(tcp_frame_);
    compute_ik->setMaxIKSolutions(4);
    compute_ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
    task.add(std::move(compute_ik));

    if (task.plan(1) != moveit::core::MoveItErrorCode::SUCCESS || task.numSolutions() == 0) {
      return false;
    }
    return task.execute(*task.solutions().front()) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool planAndExecuteRetreat()
  {
    mtc::Task task;
    task.setName("pick retreat");
    task.loadRobotModel(node_);

    task.add(std::make_unique<mtc::stages::CurrentState>("current state"));

    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(velocity_scaling_factor_);
    cartesian_planner->setMaxAccelerationScalingFactor(acceleration_scaling_factor_);
    // Without this, CartesianPath's raw interpolated trajectory can leave a
    // tiny nonzero terminal velocity (observed: ~0.0019 rad/s on one
    // joint), which fp3_arm_controller (a standard JointTrajectoryController)
    // rejects outright ("Velocity of last trajectory point ... is not
    // zero"). TOTG is the same algorithm move_group's own
    // AddTimeOptimalParameterization response adapter uses, and guarantees
    // the trajectory starts/ends at rest.
    cartesian_planner->setTimeParameterization(
      std::make_shared<trajectory_processing::TimeOptimalTrajectoryGeneration>());
    auto retreat = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
    retreat->setGroup(planning_group_);
    retreat->setIKFrame(tcp_frame_);
    retreat->setMinMaxDistance(retreat_min_distance_, retreat_max_distance_);
    geometry_msgs::msg::Vector3Stamped retreat_direction;
    retreat_direction.header.frame_id = tcp_frame_;
    retreat_direction.vector.z = -1.0;
    retreat->setDirection(retreat_direction);
    task.add(std::move(retreat));

    if (task.plan(1) != moveit::core::MoveItErrorCode::SUCCESS || task.numSolutions() == 0) {
      return false;
    }
    return task.execute(*task.solutions().front()) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  void execute(const std::shared_ptr<GoalHandlePickObject> goal_handle)
  {
    BusyGuard guard{busy_};
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<PickObject::Result>();
    result->used_pose_index = -1;

    publish_status(goal_handle, "filtering");
    auto filtered = filterCandidates(goal->grasp_candidates);
    if (filtered.empty()) {
      result->success = false;
      result->message = "No candidate survived the geometric prefilter";
      RCLCPP_WARN(node_->get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }

    publish_status(goal_handle, "opening");
    if (!openGripper()) {
      result->success = false;
      result->message = "Gripper failed to open before approach";
      RCLCPP_ERROR(node_->get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }

    publish_status(goal_handle, "planning");
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
      if (planAndExecuteApproach(candidate.pose, goal->object_id)) {
        approach_ok = true;
        used_index = candidate.original_index;
        break;
      }
    }

    if (!approach_ok) {
      result->success = false;
      result->message = "No candidate pose was reachable/collision-free after IK and planning";
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

    publish_status(goal_handle, "grasping");
    if (!closeGripper()) {
      result->success = false;
      result->message = "Gripper failed to grasp the object";
      RCLCPP_ERROR(node_->get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }

    publish_status(goal_handle, "attaching");
    if (!attachObject(goal->object_id, goal->object_dimensions)) {
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

    publish_status(goal_handle, "retreating");
    if (!planAndExecuteRetreat()) {
      result->success = false;
      result->message = "Retreat planning/execution failed (object remains attached)";
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

    publish_status(goal_handle, "placing");
    if (!planAndExecutePlace()) {
      result->success = false;
      result->message = "Place planning/execution failed (object remains attached)";
      RCLCPP_ERROR(node_->get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }

    publish_status(goal_handle, "detaching");
    if (!detachObject(goal->object_id)) {
      result->success = false;
      result->message = "Failed to detach object from planning scene";
      RCLCPP_ERROR(node_->get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }

    publish_status(goal_handle, "releasing");
    if (!openGripper()) {
      result->success = false;
      result->message = "Gripper failed to open to release the object";
      RCLCPP_ERROR(node_->get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }

    result->success = true;
    result->message = "Pick and place executed successfully";
    RCLCPP_INFO(node_->get_logger(), "%s", result->message.c_str());
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

  // Fetch robot_description/robot_description_semantic from /move_group,
  // same rationale and pattern as motion_server_node: MTC's PipelinePlanner
  // and Task::loadRobotModel() build a robot model LOCALLY in this process,
  // so it must always match the model the real move_group is using.
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
        node->get_logger(), "Parameter '%s' is not set on /move_group", param.get_name().c_str());
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

  // Same rationale as motion_server_node.cpp: gate the action server's
  // existence on a valid current robot state, since MTC's CurrentState
  // stage needs move_group's internal current_state_monitor to have
  // actually received a /joint_states message. Without this, a fast client
  // could send a goal before that's true.
  std::atomic<bool> joint_state_received{false};
  auto joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10,
    [&joint_state_received](const sensor_msgs::msg::JointState::SharedPtr msg) {
      if (!msg->name.empty()) {
        joint_state_received = true;
      }
    });
  RCLCPP_INFO(node->get_logger(), "Waiting for a valid /joint_states message...");
  const auto joint_state_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
  while (!joint_state_received.load() && std::chrono::steady_clock::now() < joint_state_deadline) {
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

  PickPlaceServer server(node);

  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
