import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'execute_pick',
            default_value='false',
            description=(
                'If true, pick_task_node sends the goal to mtc_pick after '
                'grasp generation. If false (default), the pipeline stops '
                'after grasp generation/visualization -- no motion.'
            ),
        ),

        # --- bridge nodes ---
        Node(
            package='gemini_er_bridge',
            executable='camera_bridge_node',
            name='camera_bridge',
            output='screen',
        ),

        Node(
            package='gemini_er_bridge',
            executable='command_bridge_node',
            name='command_bridge',
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('sam3_bridge'),
                    'launch',
                    'sam3_bridge.launch.py',
                )
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('graspgen_bridge'),
                    'launch',
                    'graspgen_bridge.launch.py',
                )
            ),
        ),

        # --- robot_task_manager nodes ---
        Node(
            package='robot_task_manager',
            executable='camera_buffer_node',
            name='camera_buffer',
            output='screen',
        ),

        Node(
            package='robot_task_manager',
            executable='create_pointcloud_node',
            name='create_pointcloud_node',
            output='screen',
        ),

        Node(
            package='robot_task_manager',
            executable='pick_task_node',
            name='pick_task_node',
            output='screen',
            parameters=[{
                'execute_pick': ParameterValue(
                    LaunchConfiguration('execute_pick'), value_type=bool),
            }],
        ),

    ])
