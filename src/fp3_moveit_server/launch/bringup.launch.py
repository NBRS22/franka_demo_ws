import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, Shutdown, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')
    load_gripper = LaunchConfiguration('load_gripper')
    ee_id = LaunchConfiguration('ee_id')
    use_rviz = LaunchConfiguration('use_rviz')

    # --- move_group assembly: adapted copy of
    # franka_fp3_moveit_config/launch/moveit.launch.py (franka_ros2_ws,
    # read-only, never modified). It is copied rather than included because
    # its move_group Node() has no hook to add the extra `capabilities`
    # parameter MTC needs (see move_group_capabilities below) -- everything
    # it loads still comes from franka_fp3_moveit_config/franka_bringup/
    # franka_description's own files, nothing is forked into this package.

    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_bringup'), 'urdf', 'franka_arm.urdf.xacro')
    robot_description_config = Command([
        FindExecutable(name='xacro'), ' ', franka_xacro_file, ' hand:=', load_gripper,
        ' robot_type:=fp3', ' robot_ip:=', robot_ip, ' ee_id:=', ee_id,
        ' use_fake_hardware:=', use_fake_hardware,
        ' fake_sensor_commands:=', fake_sensor_commands])
    robot_description = {'robot_description': ParameterValue(robot_description_config, value_type=str)}

    franka_semantic_xacro_file = os.path.join(
        get_package_share_directory('franka_description'), 'robots', 'fp3', 'fp3.srdf.xacro')
    robot_description_semantic_config = Command([
        FindExecutable(name='xacro'), ' ', franka_semantic_xacro_file,
        ' hand:=', load_gripper, ' ee_id:=', ee_id])
    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(robot_description_semantic_config, value_type=str)}

    kinematics_config = {
        'robot_description_kinematics': load_yaml('franka_fp3_moveit_config', 'config/kinematics.yaml')}
    joint_limits_config = {
        'robot_description_planning': load_yaml('franka_fp3_moveit_config', 'config/fp3_joint_limits.yaml')}

    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugins': ['ompl_interface/OMPLPlanner'],
            'request_adapters': [
                'default_planning_request_adapters/ResolveConstraintFrames',
                'default_planning_request_adapters/ValidateWorkspaceBounds',
                'default_planning_request_adapters/CheckStartStateBounds',
                'default_planning_request_adapters/CheckStartStateCollision',
            ],
            'response_adapters': [
                'default_planning_response_adapters/AddTimeOptimalParameterization',
                'default_planning_response_adapters/ValidateSolution',
                'default_planning_response_adapters/DisplayMotionPath',
            ],
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_pipeline_config['move_group'].update(
        load_yaml('franka_fp3_moveit_config', 'config/ompl_planning.yaml'))

    moveit_simple_controllers_yaml = load_yaml('franka_fp3_moveit_config', 'config/fp3_controllers.yaml')
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # Loads MTC's move_group capability (ros-jazzy-moveit-task-constructor-*,
    # confirmed installed) so pick_place_node's task.execute() has an
    # /execute_task_solution server to talk to. This is the one parameter
    # franka_fp3_moveit_config/launch/moveit.launch.py has no hook to inject,
    # which is why this file assembles move_group itself instead of
    # including that one -- see CLAUDE.md.
    move_group_capabilities = {'capabilities': 'move_group/ExecuteTaskSolutionCapability'}

    move_group_parameters = [
        robot_description,
        robot_description_semantic,
        kinematics_config,
        joint_limits_config,
        ompl_planning_pipeline_config,
        trajectory_execution,
        moveit_controllers,
        planning_scene_monitor_parameters,
        move_group_capabilities,
    ]

    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=move_group_parameters,
    )

    rviz_full_config = os.path.join(
        get_package_share_directory('franka_fp3_moveit_config'), 'rviz', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_full_config],
        parameters=[
            robot_description, robot_description_semantic, ompl_planning_pipeline_config, kinematics_config],
        condition=IfCondition(use_rviz),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory('franka_fp3_moveit_config'), 'config', 'fp3_ros_controllers.yaml')
    # MTC's CartesianPath-generated segments (used for approach/retreat) can
    # leave a tiny nonzero terminal velocity (observed ~0.0019 rad/s on one
    # joint) even with TOTG re-timing applied in pick_place_node.
    # fp3_arm_controller (joint_trajectory_controller) rejects any trajectory
    # that doesn't end at exactly zero velocity by default -- this override
    # relaxes that check. Added here (our own config/launch file) rather
    # than editing fp3_ros_controllers.yaml inside franka_ros2_ws. Must be a
    # separate yaml FILE, not an inline parameter dict: controller_manager
    # loads per-controller parameters through its own yaml-file mechanism,
    # which doesn't see plain launch parameter dicts.
    controller_overrides_path = os.path.join(
        get_package_share_directory('fp3_moveit_server'), 'config', 'controller_overrides.yaml')
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros2_controllers_path, controller_overrides_path],
        remappings=[('joint_states', 'franka/joint_states')],
        output={'stdout': 'screen', 'stderr': 'screen'},
        on_exit=Shutdown(),
    )

    load_controllers = [
        ExecuteProcess(
            cmd=['ros2', 'run', 'controller_manager', 'spawner', controller,
                 '--controller-manager-timeout', '60'],
            output='screen')
        for controller in ['fp3_arm_controller', 'joint_state_broadcaster']
    ]

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'source_list': ['franka/joint_states', 'franka_gripper/joint_states'], 'rate': 30}],
    )

    franka_robot_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['franka_robot_state_broadcaster'],
        output='screen',
        condition=UnlessCondition(use_fake_hardware),
    )

    gripper_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
        launch_arguments={
            'robot_ip': robot_ip, 'use_fake_hardware': use_fake_hardware, 'namespace': ''}.items(),
    )

    # --- fp3_moveit_server's own nodes ---

    scene_setup_node = Node(
        package='fp3_moveit_server',
        executable='scene_setup_node',
        output='screen',
        parameters=[LaunchConfiguration('scene_config')],
    )

    # pick_place_node's MTC PipelinePlanner plans locally (like move_group
    # itself), so it needs the same kinematics/joint_limits/ompl parameters
    # -- not just robot_description, which it fetches at runtime from
    # /move_group. table.* is reused from scene_config for its own geometric
    # prefilter (same table geometry scene_setup_node applied, no separate copy).
    pick_place_node = Node(
        package='fp3_moveit_server',
        executable='pick_place_node',
        output='screen',
        parameters=[
            kinematics_config,
            joint_limits_config,
            ompl_planning_pipeline_config,
            LaunchConfiguration('scene_config'),
            LaunchConfiguration('pick_place_config'),
            # ParameterValue(..., value_type=bool) is required here: a bare
            # LaunchConfiguration resolves to the string "true"/"false" at
            # launch time, which would be declared as a string ROS
            # parameter and throw a type-mismatch when pick_place_node
            # calls declare_parameter<bool>("simulate_gripper", ...).
            {'simulate_gripper': ParameterValue(use_fake_hardware, value_type=bool)},
        ],
        remappings=[('mtc_pick', '/internal/pick_place/mtc_pick')],
    )

    command_router_node = Node(
        package='fp3_moveit_server',
        executable='command_router_node',
        output='screen',
    )

    # move_group + controller spawners need a moment before our nodes
    # connect (same rationale as the old fp3_grasp_demo launch delay: a
    # ~500ms miss was observed without it).
    delayed_server_nodes = TimerAction(
        period=5.0,
        actions=[scene_setup_node, pick_place_node, command_router_node],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ip', default_value='192.168.1.1',
            description='FP3 controller IP (ignored if use_fake_hardware:=true)'),
        DeclareLaunchArgument(
            'use_fake_hardware', default_value='true',
            description=(
                'Use simulated hardware. SAFE BY DEFAULT (true). '
                'For the real arm: use_fake_hardware:=false + FCI enabled on the Desk.')),
        DeclareLaunchArgument('fake_sensor_commands', default_value='false'),
        DeclareLaunchArgument('load_gripper', default_value='true'),
        DeclareLaunchArgument('ee_id', default_value='franka_hand'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'scene_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('fp3_moveit_server'), 'config', 'scene.yaml']),
            description='Path to the planning scene config (table collision object)'),
        DeclareLaunchArgument(
            'pick_place_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('fp3_moveit_server'), 'config', 'pick_place.yaml']),
            description='Path to pick_place_node config (groups, filter thresholds, grasp params)'),
        rviz_node,
        robot_state_publisher,
        run_move_group_node,
        ros2_control_node,
        joint_state_publisher,
        franka_robot_state_broadcaster,
        gripper_launch_file,
    ] + load_controllers + [delayed_server_nodes])
