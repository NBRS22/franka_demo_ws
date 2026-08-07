import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Use Isaac Sim (true) or real RealSense + robot (false)',
        ),

        # --- real RealSense (use_sim:=false only) ---
        GroupAction(
            scoped=True,
            forwarding=False,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('realsense2_camera'),
                            'launch',
                            'rs_launch.py',
                        )
                    ),
                    launch_arguments={
                        'align_depth.enable': 'true',
                        'log_level': 'warn',
                    }.items(),
                ),
            ],
            condition=UnlessCondition(use_sim),
        ),

        # --- MoveIt2 + Isaac Sim bridge (use_sim:=true only) ---
        GroupAction(
            scoped=True,
            forwarding=False,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('panda_motion_server'),
                            'launch',
                            'panda_motion_server.launch.py',
                        )
                    ),
                ),
            ],
            condition=IfCondition(use_sim),
        ),

        # --- perception & bridge nodes ---
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

        Node(
            package='sam3_bridge',
            executable='sam3_bridge_node',
            name='sam3_bridge',
            output='screen',
        ),

        Node(
            package='graspgen_bridge',
            executable='graspgen_bridge_node',
            name='graspgen_bridge',
            output='screen',
        ),

        # --- MoveIt-dependent nodes (use_sim:=true only) ---
        Node(
            package='robot_task_manager',
            executable='scene_publisher_node',
            name='scene_publisher_node',
            output='screen',
            condition=IfCondition(use_sim),
        ),

        Node(
            package='robot_task_manager',
            executable='motion_node',
            name='motion_node',
            output='screen',
            condition=IfCondition(use_sim),
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
            executable='visualize_segmentation_node',
            name='visualize_segmentation_node',
            output='screen',
        ),

        Node(
            package='robot_task_manager',
            executable='pointcloud_node',
            name='pointcloud_node',
            output='screen',
        ),

        Node(
            package='robot_task_manager',
            executable='visualize_grasps_node',
            name='visualize_grasps_node',
            output='screen',
        ),

        Node(
            package='robot_task_manager',
            executable='pick_task_node',
            name='pick_task_node',
            output='screen',
        ),

    ])
