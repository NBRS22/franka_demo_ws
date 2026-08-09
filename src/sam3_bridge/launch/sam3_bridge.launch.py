from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='sam3_bridge',
            executable='sam3_bridge_node',
            name='sam3_bridge',
            output='screen',
        ),

        Node(
            package='sam3_bridge',
            executable='visualize_segmentation_node',
            name='visualize_segmentation_node',
            output='screen',
        ),

    ])
