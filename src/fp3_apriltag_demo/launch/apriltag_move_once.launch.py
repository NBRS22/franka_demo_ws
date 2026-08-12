from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# Brings up the full arm stack (fp3_moveit_server/bringup.launch.py: move_group,
# ros2_control, scene_setup_node, motion_server_node, pick_place_node,
# command_router_node) plus apriltag_move_once_node. Does NOT launch the
# camera driver, the AprilTag detector, or handeye_tf_publisher -- those are
# assumed to already be running (they own their own calibration/camera
# lifecycle, independent of the arm). apriltag_move_once_node itself blocks
# on ActionClient.wait_for_server() until command_router_node's move_to_pose
# is up, so no extra startup delay is added here for it.
def generate_launch_description():
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    robot_ip = LaunchConfiguration('robot_ip')
    use_rviz = LaunchConfiguration('use_rviz')
    tag_size = LaunchConfiguration('tag_size')
    target_tag_id = LaunchConfiguration('target_tag_id')
    force_gripper_down = LaunchConfiguration('force_gripper_down')

    moveit_server_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('fp3_moveit_server'), 'launch', 'bringup.launch.py'])),
        launch_arguments={
            'use_fake_hardware': use_fake_hardware,
            'robot_ip': robot_ip,
            'use_rviz': use_rviz,
        }.items(),
    )

    apriltag_move_once_node = Node(
        package='fp3_apriltag_demo',
        executable='apriltag_move_once_node',
        output='screen',
        parameters=[{
            # ParameterValue(..., value_type=...) required: a bare
            # LaunchConfiguration resolves to a string at launch time, which
            # would mismatch the int/float types declare_parameter() expects
            # in apriltag_move_once_node.py (same class of bug already fixed
            # once for fp3_moveit_server's simulate_gripper parameter).
            'tag_size': ParameterValue(tag_size, value_type=float),
            'target_tag_id': ParameterValue(target_tag_id, value_type=int),
            'force_gripper_down': ParameterValue(force_gripper_down, value_type=bool),
            # franka_gripper_node (the real Grasp action server) isn't
            # launched in fake hardware mode -- same fix as pick_place_node.
            'simulate_gripper': ParameterValue(use_fake_hardware, value_type=bool),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_fake_hardware', default_value='true',
            description=(
                'Use simulated hardware. SAFE BY DEFAULT (true). '
                'For the real arm: use_fake_hardware:=false + FCI enabled on the Desk.')),
        DeclareLaunchArgument(
            'robot_ip', default_value='192.168.1.1',
            description='FP3 controller IP (ignored if use_fake_hardware:=true)'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'tag_size', default_value='0.04',
            description='AprilTag physical size: outer black square side, in meters'),
        DeclareLaunchArgument(
            'target_tag_id', default_value='0',
            description='Which AprilTag id to wait for on /detections'),
        DeclareLaunchArgument(
            'force_gripper_down', default_value='true',
            description=(
                'Keep the tag\'s computed position but replace its orientation with a '
                'fixed straight-down TCP orientation (recommended: the tag\'s own '
                'orientation is often kinematically unreachable).')),
        moveit_server_bringup,
        apriltag_move_once_node,
    ])
