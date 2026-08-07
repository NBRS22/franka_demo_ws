#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "franka_demo_interfaces/action/move_to_pose.hpp"
#include "franka_demo_interfaces/action/go_home.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using MoveToPose = franka_demo_interfaces::action::MoveToPose;
using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPose>;
using GoHome = franka_demo_interfaces::action::GoHome;
using GoalHandleGoHome = rclcpp_action::ServerGoalHandle<GoHome>;

// Nom de l'état articulaire nommé du groupe 'panda_arm' dans panda.srdf,
// utilisé pour un vrai retour "home" en espace articulaire (plutôt qu'une
// pose cartésienne arbitraire qui peut aboutir à une configuration du bras
// bizarre malgré une position d'effecteur correcte).
static const char * const HOME_NAMED_TARGET = "ready";

// v1: symmetric to fp3_motion_server, for the simulated Panda (Isaac Sim).
// Receives a target pose via the MoveToPose action and directly does
// setPoseTarget() + plan() + execute() (no pre-grasp, no cartesian
// approach). Network client of move_group (group 'panda_arm', launched via
// moveit_resources_panda_moveit_config) via MoveGroupInterface, loads no
// robot model here.
class PandaMotionServer
{
public:
  PandaMotionServer(
    const rclcpp::Node::SharedPtr & node,
    const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> & move_group)
  : node_(node), move_group_(move_group)
  {
    // Diagnostic uniquement : sert à logguer la pose de grasp une fois
    // résolue dans le planning frame, pour vérifier "à la main" qu'elle est
    // bien dans l'enveloppe de portée du Panda avant que setPoseTarget()
    // ne délègue la même résolution TF en interne à move_group.
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    action_server_ = rclcpp_action::create_server<MoveToPose>(
      node_,
      "move_to_pose",
      std::bind(&PandaMotionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PandaMotionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&PandaMotionServer::handle_accepted, this, std::placeholders::_1));

    go_home_server_ = rclcpp_action::create_server<GoHome>(
      node_,
      "go_home",
      std::bind(&PandaMotionServer::handle_go_home_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PandaMotionServer::handle_go_home_cancel, this, std::placeholders::_1),
      std::bind(&PandaMotionServer::handle_go_home_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "panda_motion_server ready (planning group 'panda_arm')");
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;
  rclcpp_action::Server<GoHome>::SharedPtr go_home_server_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Transforme la pose cible vers le planning frame de move_group (ex:
  // 'world') nous-mêmes, plutôt que de compter sur la résolution TF
  // interne de setPoseTarget()/move_group : celle-ci utilise le stamp du
  // message pour la recherche TF, or ce stamp vient de grasp_selector_node
  // (horloge murale, epoch Unix) alors que la TF de la caméra publiée par
  // Isaac Sim est en temps sim (petits nombres) — deux domaines temporels
  // incompatibles, qui font échouer la recherche par "extrapolation into
  // the future" et retombent silencieusement sur la pose non transformée.
  // On utilise ici tf2::TimePointZero ("dernière transformation connue")
  // au lieu du stamp du message : sans risque puisque la caméra est fixe
  // (confirmé : elle ne bouge pas avec le bras), donc la transformation
  // caméra->world ne varie pas dans le temps.
  geometry_msgs::msg::PoseStamped transform_to_planning_frame(
    const geometry_msgs::msg::PoseStamped & pose)
  {
    const std::string & planning_frame = move_group_->getPlanningFrame();
    geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer_->lookupTransform(
      planning_frame, pose.header.frame_id, tf2::TimePointZero, tf2::durationFromSec(1.0));

    geometry_msgs::msg::PoseStamped transformed;
    tf2::doTransform(pose, transformed, tf_stamped);
    transformed.header.frame_id = planning_frame;

    RCLCPP_INFO(
      node_->get_logger(),
      "Grasp pose dans le planning frame '%s': pos=(%.3f, %.3f, %.3f) "
      "quat=(%.3f, %.3f, %.3f, %.3f)",
      planning_frame.c_str(),
      transformed.pose.position.x,
      transformed.pose.position.y,
      transformed.pose.position.z,
      transformed.pose.orientation.x,
      transformed.pose.orientation.y,
      transformed.pose.orientation.z,
      transformed.pose.orientation.w);
    return transformed;
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const MoveToPose::Goal> goal)
  {
    RCLCPP_INFO(
      node_->get_logger(),
      "Goal received: pose (%.3f, %.3f, %.3f) frame='%s'",
      goal->target_pose.pose.position.x,
      goal->target_pose.pose.position.y,
      goal->target_pose.pose.position.z,
      goal->target_pose.header.frame_id.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMoveToPose>)
  {
    // v1: canceling an in-progress MoveIt execution is not implemented.
    RCLCPP_WARN(node_->get_logger(), "Cancel requested but not supported in v1");
    return rclcpp_action::CancelResponse::REJECT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
  {
    // plan()/execute() are blocking: we run them off the executor thread
    // so as not to block the rest of the node (including MoveGroupInterface's
    // internal calls).
    std::thread{std::bind(&PandaMotionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<MoveToPose::Result>();

    geometry_msgs::msg::PoseStamped target_pose;
    try {
      target_pose = transform_to_planning_frame(goal->target_pose);
    } catch (const tf2::TransformException & ex) {
      result->success = false;
      result->message = std::string("TF transform failed: ") + ex.what();
      RCLCPP_ERROR(node_->get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }

    move_group_->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    RCLCPP_INFO(node_->get_logger(), "Planning to target pose...");
    bool plan_ok = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!plan_ok) {
      result->success = false;
      result->message = "Planning failed";
      RCLCPP_ERROR(node_->get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }

    RCLCPP_INFO(node_->get_logger(), "Executing plan...");
    bool exec_ok = (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    result->success = exec_ok;
    result->message = exec_ok ? "Movement executed successfully" : "Execution failed";

    if (exec_ok) {
      RCLCPP_INFO(node_->get_logger(), "%s", result->message.c_str());
      goal_handle->succeed(result);
    } else {
      RCLCPP_ERROR(node_->get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
    }
  }

  // --- GoHome ---

  rclcpp_action::GoalResponse handle_go_home_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const GoHome::Goal>)
  {
    RCLCPP_INFO(node_->get_logger(), "Go home goal received");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_go_home_cancel(
    const std::shared_ptr<GoalHandleGoHome>)
  {
    return rclcpp_action::CancelResponse::REJECT;
  }

  void handle_go_home_accepted(const std::shared_ptr<GoalHandleGoHome> goal_handle)
  {
    std::thread{
      std::bind(&PandaMotionServer::execute_go_home, this, std::placeholders::_1),
      goal_handle}.detach();
  }

  void execute_go_home(const std::shared_ptr<GoalHandleGoHome> goal_handle)
  {
    auto result = std::make_shared<GoHome::Result>();

    move_group_->setNamedTarget(HOME_NAMED_TARGET);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    RCLCPP_INFO(node_->get_logger(), "Planning to named target '%s'...", HOME_NAMED_TARGET);
    bool plan_ok = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!plan_ok) {
      result->success = false;
      result->message = "Planning to home failed";
      RCLCPP_ERROR(node_->get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }

    RCLCPP_INFO(node_->get_logger(), "Executing plan to home...");
    bool exec_ok = (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    result->success = exec_ok;
    result->message = exec_ok ? "Returned home successfully" : "Execution to home failed";

    if (exec_ok) {
      RCLCPP_INFO(node_->get_logger(), "%s", result->message.c_str());
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

  auto node = rclcpp::Node::make_shared("panda_motion_server");

  // MoveGroupInterface makes synchronous calls (parameters, action
  // clients...) at construction: the node must already be spinning on a
  // separate thread, otherwise construction blocks indefinitely.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  // MoveGroupInterface builds its robot model LOCALLY and needs
  // robot_description/robot_description_semantic as parameters on this
  // node. Rather than duplicating the launch's MoveItConfigsBuilder config
  // here, we fetch these two values directly from /move_group via a ROS2
  // parameter client (same pattern as fp3_motion_server).
  auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(node, "move_group");
  RCLCPP_INFO(node->get_logger(), "Waiting for move_group parameter server...");
  if (!param_client->wait_for_service(std::chrono::seconds(30))) {
    RCLCPP_FATAL(node->get_logger(), "move_group unavailable after 30s, aborting");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  auto results = param_client->get_parameters(
    {"robot_description", "robot_description_semantic"}).get();
  for (const auto & param : results) {
    node->declare_parameter<std::string>(param.get_name(), param.as_string());
  }
  RCLCPP_INFO(node->get_logger(), "Robot model retrieved from move_group");

  auto move_group =
    std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "panda_arm");

  PandaMotionServer server(node, move_group);

  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
