from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    motion_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('panda_motion_server'),
                'launch',
                'panda_motion_server.launch.py'
            ])
        ])
    )

    # No delay before this node, unlike the FP3 side: there the delay
    # compensates for waiting on the ros2_control spawner (fp3_arm_controller).
    # Here joint_trajectory_bridge is ready almost instantly (no
    # controller_manager), and wait_for_server() below already waits for
    # panda_motion_server (hence MoveGroupInterface) to be ready before
    # sending the goal.
    grasp_demo_node = Node(
        package='panda_grasp_demo',
        executable='hardcoded_grasp_node',
        output='screen'
    )

    return LaunchDescription([
        motion_server_launch,
        grasp_demo_node,
    ])
