#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/srv/get_position_ik.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "franka_demo_interfaces/action/move_to_pose.hpp"

using MoveToPose = franka_demo_interfaces::action::MoveToPose;
using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPose>;
using GetPositionIK = moveit_msgs::srv::GetPositionIK;

// Simple, single-stage motion primitive. Owns no scene setup (that's
// scene_setup_node's job) and is never called by clients directly -- its
// "move_to_pose" action is remapped to an internal name in
// bringup.launch.py, and only command_router_node talks to it, so that a
// single busy flag (in the router) can arbitrate against pick_place_node
// too.
class MotionServer
{
public:
  MotionServer(
    const rclcpp::Node::SharedPtr & node,
    const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> & move_group,
    std::string planning_group)
  : node_(node), move_group_(move_group), planning_group_(std::move(planning_group))
  {
    ik_client_ = node_->create_client<GetPositionIK>("compute_ik");

    action_server_ = rclcpp_action::create_server<MoveToPose>(
      node_,
      "move_to_pose",
      std::bind(&MotionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MotionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&MotionServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(
      node_->get_logger(), "motion_server_node ready (planning group '%s')", planning_group_.c_str());
  }

  ~MotionServer()
  {
    if (execution_thread_.joinable()) {
      execution_thread_.join();
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::string planning_group_;
  rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;
  rclcpp::Client<GetPositionIK>::SharedPtr ik_client_;
  std::thread execution_thread_;
  std::atomic<bool> busy_{false};

  struct BusyGuard
  {
    std::atomic<bool> & flag;
    ~BusyGuard() {flag = false;}
  };

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const MoveToPose::Goal> goal)
  {
    if (busy_.load()) {
      RCLCPP_WARN(node_->get_logger(), "Goal rejected: another goal is already in progress");
      return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "Goal received: pose (%.3f, %.3f, %.3f) frame='%s'",
      goal->target_pose.pose.position.x,
      goal->target_pose.pose.position.y,
      goal->target_pose.pose.position.z,
      goal->target_pose.header.frame_id.c_str());

    busy_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveToPose>)
  {
    RCLCPP_WARN(node_->get_logger(), "Cancel requested, stopping MoveGroup");
    move_group_->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
  {
    if (execution_thread_.joinable()) {
      execution_thread_.join();
    }
    execution_thread_ = std::thread(&MotionServer::execute, this, goal_handle);
  }

  void publish_status(const std::shared_ptr<GoalHandleMoveToPose> & goal_handle, const std::string & status)
  {
    auto feedback = std::make_shared<MoveToPose::Feedback>();
    feedback->status = status;
    goal_handle->publish_feedback(feedback);
  }

  void execute(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
  {
    BusyGuard guard{busy_};
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<MoveToPose::Result>();

    // 1. Fast IK/collision precheck (no motion). The table is already in
    // the planning scene (added once by scene_setup_node before this node
    // ever receives a goal).
    publish_status(goal_handle, "checking_ik");

    if (!ik_client_->wait_for_service(std::chrono::seconds(5))) {
      result->success = false;
      result->message = "compute_ik service unavailable";
      RCLCPP_ERROR(node_->get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }

    auto ik_request = std::make_shared<GetPositionIK::Request>();
    ik_request->ik_request.group_name = planning_group_;
    ik_request->ik_request.pose_stamped = goal->target_pose;
    ik_request->ik_request.avoid_collisions = true;
    ik_request->ik_request.timeout = rclcpp::Duration::from_seconds(1.0);

    auto ik_response = ik_client_->async_send_request(ik_request).get();
    if (ik_response->error_code.val != ik_response->error_code.SUCCESS) {
      result->success = false;
      result->message = "Target pose unreachable or in collision (IK check failed)";
      RCLCPP_WARN(node_->get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }

    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Canceled";
      goal_handle->canceled(result);
      return;
    }

    // 2. Planning.
    publish_status(goal_handle, "planning");
    move_group_->setPoseTarget(goal->target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    RCLCPP_INFO(node_->get_logger(), "Planning to target pose...");
    bool plan_ok = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Canceled";
      goal_handle->canceled(result);
      return;
    }

    if (!plan_ok) {
      result->success = false;
      result->message = "Planning failed";
      RCLCPP_ERROR(node_->get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }

    // 3. Execution. NOTE (v1): execute() can block indefinitely if the
    // controller never reports completion. No timeout here yet -- v2.
    publish_status(goal_handle, "executing");
    RCLCPP_INFO(node_->get_logger(), "Executing plan...");
    bool exec_ok = (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Canceled";
      goal_handle->canceled(result);
      return;
    }

    result->success = exec_ok;
    result->message = exec_ok ? "Movement executed successfully" : "Execution failed";

    if (exec_ok) {
      RCLCPP_INFO(node_->get_logger(), "%s", result->message.c_str());
      publish_status(goal_handle, "done");
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

  auto node = rclcpp::Node::make_shared("motion_server_node");

  // MoveGroupInterface makes synchronous calls at construction: the node
  // must already be spinning on a separate thread.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  // Fetch robot_description/robot_description_semantic from /move_group
  // rather than reprocessing xacro locally, so this node can never drift
  // from the model the real move_group is actually using.
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
        node->get_logger(),
        "Parameter '%s' is not set on /move_group (is bringup.launch.py fully launched?)",
        param.get_name().c_str());
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

  const std::string planning_group =
    node->declare_parameter<std::string>("planning_group", "fp3_arm");

  auto move_group =
    std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, planning_group);

  // Safety default: cap speed at 10% of the robot's max, everywhere this
  // node plans a motion. Override via parameters if you actually need more.
  const double velocity_scaling_factor =
    node->declare_parameter<double>("velocity_scaling_factor", 0.1);
  const double acceleration_scaling_factor =
    node->declare_parameter<double>("acceleration_scaling_factor", 0.1);
  move_group->setMaxVelocityScalingFactor(velocity_scaling_factor);
  move_group->setMaxAccelerationScalingFactor(acceleration_scaling_factor);

  // Gate the action server's existence on a valid current robot state.
  // Without this, the action server (created inside MotionServer's
  // constructor) becomes discoverable the instant this node starts, before
  // move_group's own current_state_monitor has received a single
  // /joint_states message -- a client fast enough to send a goal in that
  // window makes compute_ik run against move_group's still-empty internal
  // state ("Found empty JointState message"), failing IK for a pose that
  // was actually fine. Observed live with apriltag_move_once_node, which
  // fires as soon as it sees a tag, well inside this window.
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

  MotionServer server(node, move_group, planning_group);

  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
