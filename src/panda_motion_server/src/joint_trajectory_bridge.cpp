#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/action/gripper_command.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;
using GripperCommand = control_msgs::action::GripperCommand;
using GoalHandleGripperCommand = rclcpp_action::ServerGoalHandle<GripperCommand>;

namespace
{
// Même tolérance que allowed_start_tolerance (gripper_moveit_controllers.yaml) :
// pas la peine d'être plus strict ici que ce que move_group exige déjà pour
// accepter le prochain mouvement.
constexpr double kConvergenceTolerance = 0.01;  // rad (bras) ou m (pince)
constexpr auto kConvergenceTimeout = std::chrono::seconds(10);
constexpr auto kConvergencePollInterval = std::chrono::milliseconds(20);
}  // namespace

// Relaie vers Isaac Sim ce que move_group croit envoyer à de vrais
// contrôleurs ros2_control : trajectoire du bras (panda_arm_controller) et
// commande de pince (panda_hand_controller), toutes deux republiées sur
// /joint_command. Aucun vrai contrôleur ros2_control ici, Isaac Sim n'en
// utilise pas. Contrairement à un vrai contrôleur, on ne sait pas nativement
// quand Isaac Sim a fini de bouger : on le déduit en comparant /joint_states
// à la cible, avec un jeu de tolérance (pas d'égalité stricte).
class JointTrajectoryBridge : public rclcpp::Node
{
public:
  JointTrajectoryBridge()
  : Node("joint_trajectory_bridge")
  {
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_command", 10);
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&JointTrajectoryBridge::joint_state_callback, this, std::placeholders::_1));

    trajectory_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      this,
      "panda_arm_controller/follow_joint_trajectory",
      std::bind(&JointTrajectoryBridge::handle_trajectory_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&JointTrajectoryBridge::handle_trajectory_cancel, this, std::placeholders::_1),
      std::bind(&JointTrajectoryBridge::handle_trajectory_accepted, this, std::placeholders::_1));

    gripper_server_ = rclcpp_action::create_server<GripperCommand>(
      this,
      "panda_hand_controller/gripper_cmd",
      std::bind(&JointTrajectoryBridge::handle_gripper_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&JointTrajectoryBridge::handle_gripper_cancel, this, std::placeholders::_1),
      std::bind(&JointTrajectoryBridge::handle_gripper_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "JointTrajectoryBridge ready! (bras + pince)");
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr trajectory_server_;
  rclcpp_action::Server<GripperCommand>::SharedPtr gripper_server_;

  std::mutex joint_state_mutex_;
  std::unordered_map<std::string, double> latest_joint_positions_;

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
      latest_joint_positions_[msg->name[i]] = msg->position[i];
    }
  }

  // Attend que les joints demandés soient à kConvergenceTolerance près de
  // leur cible, d'après le dernier /joint_states reçu (pas d'égalité
  // stricte). Renvoie false si kConvergenceTimeout est atteint sans
  // convergence (joint manquant dans /joint_states, Isaac Sim à l'arrêt...).
  bool wait_for_convergence(
    const std::vector<std::string> & names,
    const std::vector<double> & targets)
  {
    const auto deadline = std::chrono::steady_clock::now() + kConvergenceTimeout;

    while (std::chrono::steady_clock::now() < deadline) {
      bool all_converged = true;
      {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        for (size_t i = 0; i < names.size(); ++i) {
          auto it = latest_joint_positions_.find(names[i]);
          if (it == latest_joint_positions_.end() ||
            std::abs(it->second - targets[i]) > kConvergenceTolerance)
          {
            all_converged = false;
            break;
          }
        }
      }
      if (all_converged) {
        return true;
      }
      std::this_thread::sleep_for(kConvergencePollInterval);
    }
    return false;
  }

  // --- FollowJointTrajectory (bras) ---

  rclcpp_action::GoalResponse handle_trajectory_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const FollowJointTrajectory::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_trajectory_cancel(
    const std::shared_ptr<GoalHandleFollowJointTrajectory>)
  {
    return rclcpp_action::CancelResponse::REJECT;
  }

  void handle_trajectory_accepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
  {
    std::thread{
      std::bind(&JointTrajectoryBridge::execute_trajectory, this, std::placeholders::_1),
      goal_handle}.detach();
  }

  void execute_trajectory(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Executing trajectory...");
    const auto & trajectory = goal_handle->get_goal()->trajectory;

    for (const auto & point : trajectory.points) {
      sensor_msgs::msg::JointState msg;
      msg.header.stamp = get_clock()->now();
      msg.name = trajectory.joint_names;
      msg.position = point.positions;
      joint_pub_->publish(msg);

      // Petit délai entre chaque point pour qu'Isaac Sim ait le temps de
      // suivre la trajectoire plutôt que de recevoir toutes les cibles
      // d'un coup (ne respecte pas le timing exact de time_from_start).
      if (point.time_from_start.sec > 0 || point.time_from_start.nanosec > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }

    auto result = std::make_shared<FollowJointTrajectory::Result>();

    if (trajectory.points.empty()) {
      goal_handle->succeed(result);
      return;
    }

    const auto & final_point = trajectory.points.back();
    bool converged = wait_for_convergence(trajectory.joint_names, final_point.positions);

    if (!converged) {
      result->error_code = FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED;
      result->error_string = "Timeout en attendant la convergence de /joint_states";
      RCLCPP_ERROR(get_logger(), "%s", result->error_string.c_str());
      goal_handle->abort(result);
      return;
    }

    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Trajectory executed!");
  }

  // --- GripperCommand (pince) ---

  rclcpp_action::GoalResponse handle_gripper_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const GripperCommand::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Gripper goal reçu: position=%.4f", goal->command.position);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_gripper_cancel(
    const std::shared_ptr<GoalHandleGripperCommand>)
  {
    return rclcpp_action::CancelResponse::REJECT;
  }

  void handle_gripper_accepted(const std::shared_ptr<GoalHandleGripperCommand> goal_handle)
  {
    std::thread{
      std::bind(&JointTrajectoryBridge::execute_gripper, this, std::placeholders::_1),
      goal_handle}.detach();
  }

  void execute_gripper(const std::shared_ptr<GoalHandleGripperCommand> goal_handle)
  {
    const double position = goal_handle->get_goal()->command.position;

    // panda_finger_joint2 mimique panda_finger_joint1 (cf. SRDF) : on publie
    // explicitement les deux avec la même valeur, comme le font les
    // group_state "open"/"close" du panda.srdf officiel.
    const std::vector<std::string> finger_names = {"panda_finger_joint1", "panda_finger_joint2"};
    const std::vector<double> finger_targets = {position, position};

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = get_clock()->now();
    msg.name = finger_names;
    msg.position = finger_targets;
    joint_pub_->publish(msg);

    bool converged = wait_for_convergence(finger_names, finger_targets);

    auto result = std::make_shared<GripperCommand::Result>();
    result->position = position;
    result->effort = 0.0;
    result->stalled = false;
    result->reached_goal = converged;

    if (!converged) {
      RCLCPP_ERROR(get_logger(), "Timeout en attendant la convergence de la pince");
      goal_handle->abort(result);
      return;
    }

    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Gripper command exécutée (position=%.4f)", position);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointTrajectoryBridge>());
  rclcpp::shutdown();
  return 0;
}
