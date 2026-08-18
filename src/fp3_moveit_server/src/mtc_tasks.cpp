#include "fp3_moveit_server/mtc_tasks.hpp"

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

namespace mtc = moveit::task_constructor;

bool planAndExecuteApproach(
  rclcpp::Node::SharedPtr node,
  const MtcParams & mtc_params,
  const ApproachParams & approach,
  const geometry_msgs::msg::PoseStamped & pose)
{
  mtc::Task task;
  task.setName("mtc_pick approach");
  task.loadRobotModel(node);

  task.add(std::make_unique<mtc::stages::CurrentState>("current state"));
  auto * current_state_ptr = task.stages()->findChild("current state");

  auto sampling_planner =
    std::make_shared<mtc::solvers::PipelinePlanner>(node, "move_group");
  sampling_planner->setMaxVelocityScalingFactor(mtc_params.velocity_scaling_factor);
  sampling_planner->setMaxAccelerationScalingFactor(mtc_params.acceleration_scaling_factor);

  mtc::stages::Connect::GroupPlannerVector planners = {
    {mtc_params.planning_group, sampling_planner}};
  task.add(std::make_unique<mtc::stages::Connect>("connect", planners));

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(mtc_params.velocity_scaling_factor);
  cartesian_planner->setMaxAccelerationScalingFactor(mtc_params.acceleration_scaling_factor);
  // TOTG ensures zero terminal velocity, required by fp3_arm_controller
  // (see config/controller_overrides.yaml).
  cartesian_planner->setTimeParameterization(
    std::make_shared<trajectory_processing::TimeOptimalTrajectoryGeneration>());

  auto approach_stage =
    std::make_unique<mtc::stages::MoveRelative>("approach", cartesian_planner);
  approach_stage->setGroup(mtc_params.planning_group);
  approach_stage->setIKFrame(mtc_params.tcp_frame);
  approach_stage->setMinMaxDistance(approach.min_distance, approach.max_distance);
  geometry_msgs::msg::Vector3Stamped approach_dir;
  approach_dir.header.frame_id = mtc_params.tcp_frame;
  approach_dir.vector.z = 1.0;
  approach_stage->setDirection(approach_dir);
  task.add(std::move(approach_stage));

  auto pose_generator =
    std::make_unique<mtc::stages::FixedCartesianPoses>("grasp pose");
  pose_generator->addPose(pose);
  pose_generator->setMonitoredStage(current_state_ptr);

  auto ik = std::make_unique<mtc::stages::ComputeIK>(
    "grasp IK", std::move(pose_generator));
  ik->setGroup(mtc_params.planning_group);
  ik->setEndEffector(mtc_params.eef_name);
  ik->setIKFrame(mtc_params.tcp_frame);
  ik->setMaxIKSolutions(4);
  // FixedCartesianPoses sets "target_pose" on its InterfaceState; ComputeIK
  // only reads it from there if told to pull from the interface.
  ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
  task.add(std::move(ik));

  auto allow_collision =
    std::make_unique<mtc::stages::ModifyPlanningScene>("allow hand/object collision");
  allow_collision->allowCollisions(
    mtc_params.object_id, mtc_params.hand_touch_links, true);
  task.add(std::move(allow_collision));

  if (task.plan(1) != moveit::core::MoveItErrorCode::SUCCESS ||
    task.numSolutions() == 0)
  {
    return false;
  }
  return task.execute(*task.solutions().front()) ==
    moveit::core::MoveItErrorCode::SUCCESS;
}

bool planAndExecuteLift(
  rclcpp::Node::SharedPtr node,
  const MtcParams & mtc_params,
  const LiftParams & lift)
{
  mtc::Task task;
  task.setName("mtc_pick lift");
  task.loadRobotModel(node);

  task.add(std::make_unique<mtc::stages::CurrentState>("current state"));

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(mtc_params.velocity_scaling_factor);
  cartesian_planner->setMaxAccelerationScalingFactor(mtc_params.acceleration_scaling_factor);
  cartesian_planner->setTimeParameterization(
    std::make_shared<trajectory_processing::TimeOptimalTrajectoryGeneration>());

  auto lift_stage =
    std::make_unique<mtc::stages::MoveRelative>("lift", cartesian_planner);
  lift_stage->setGroup(mtc_params.planning_group);
  lift_stage->setIKFrame(mtc_params.tcp_frame);
  lift_stage->setMinMaxDistance(lift.min_distance, lift.max_distance);
  geometry_msgs::msg::Vector3Stamped lift_dir;
  lift_dir.header.frame_id = "fp3_link0";  // fixed base frame = world Z+
  lift_dir.vector.z = 1.0;
  lift_stage->setDirection(lift_dir);
  task.add(std::move(lift_stage));

  if (task.plan(1) != moveit::core::MoveItErrorCode::SUCCESS ||
    task.numSolutions() == 0)
  {
    return false;
  }
  return task.execute(*task.solutions().front()) ==
    moveit::core::MoveItErrorCode::SUCCESS;
}
