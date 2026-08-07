from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Couche 1 (MoveIt + bridge Isaac Sim). Pas de client hardcodé ici,
    # contrairement à panda_full_demo.launch.py.
    motion_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('panda_motion_server'),
                'launch',
                'panda_motion_server.launch.py'
            ])
        ])
    )

    # Couche 2 : consomme /best_grasp_pose, publié par
    # flow_manager/grasp_selector_node (à lancer séparément, avec
    # `ros2 launch flow_manager flow_manager.launch.py use_sim:=true`).
    grasp_pose_subscriber_node = Node(
        package='panda_grasp_demo',
        executable='grasp_pose_subscriber_node',
        output='screen'
    )

    return LaunchDescription([
        motion_server_launch,
        grasp_pose_subscriber_node,
    ])
