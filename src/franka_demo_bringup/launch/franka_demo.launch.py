import os

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([

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
                        'pointcloud.enable': 'true',
                        'log_level': 'warn',
                    }.items(),
                ),
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('robot_task_manager'),
                    'launch',
                    'robot_task_manager.launch.py',
                )
            ),
        ),

    ])
