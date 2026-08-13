from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    calibration_name_arg = DeclareLaunchArgument(
        'calibration_name',
        default_value='fp3_link0_d455_camera_color_optical_frame_001',
        description='Nom du fichier .calib (sans extension)',
    )
    calib_dir_arg = DeclareLaunchArgument(
        'calib_dir',
        default_value='~/.ros2/easy_handeye2/calibrations',
        description='Dossier contenant les fichiers .calib',
    )
    publish_rate_s_arg = DeclareLaunchArgument(
        'publish_rate_s',
        default_value='2.0',
        description="Intervalle de retry (s) si le TF n'est pas encore disponible",
    )
    camera_link_frame_arg = DeclareLaunchArgument(
        'camera_link_frame',
        default_value='camera_link',
        description='Nom du frame camera_link dans le TF tree',
    )

    node = Node(
        package='handeye_tf_publisher',
        executable='handeye_tf_publisher',
        name='handeye_tf_publisher',
        output='screen',
        parameters=[{
            'calibration_name': LaunchConfiguration('calibration_name'),
            'calib_dir': LaunchConfiguration('calib_dir'),
            'publish_rate_s': LaunchConfiguration('publish_rate_s'),
            'camera_link_frame': LaunchConfiguration('camera_link_frame'),
        }],
    )

    return LaunchDescription([
        calibration_name_arg,
        calib_dir_arg,
        publish_rate_s_arg,
        camera_link_frame_arg,
        node,
    ])
