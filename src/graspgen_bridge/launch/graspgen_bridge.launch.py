from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='graspgen_bridge',
            executable='graspgen_bridge_node',
            name='graspgen_bridge',
            output='screen',
        ),

        Node(
            package='graspgen_bridge',
            executable='visualize_grasps_node',
            name='visualize_grasps_node',
            output='screen',
        ),

    ])
