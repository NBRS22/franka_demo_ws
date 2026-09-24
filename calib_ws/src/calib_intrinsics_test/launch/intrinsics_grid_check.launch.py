from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

REALSENSE_COLOR_PROFILE = '1280x720x30'
REALSENSE_DEPTH_PROFILE = '1280x720x30'


# Self-contained static intrinsics check: no robot needed at all -- just the
# camera pointed at a printed 3x3 grid of AprilTags (cf.
# grid_intrinsics_check_node.py docstring for the physical layout). Starts
# the camera (own retry-wrapped launch, same pattern as fp3_apriltag_demo/
# calib_bringup), apriltag_node configured for all 9 grid tag ids, and the
# check node itself.
def generate_launch_description():
    apriltag_params_file = LaunchConfiguration('apriltag_params_file')

    realsense = ExecuteProcess(
        cmd=[
            PathJoinSubstitution(
                [FindPackageShare('calib_intrinsics_test'), 'scripts', 'launch_realsense_with_retry.sh']),
            'align_depth.enable:=true',
            'initial_reset:=true',
            'log_level:=warn',
            f'rgb_camera.color_profile:={REALSENSE_COLOR_PROFILE}',
            f'depth_module.depth_profile:={REALSENSE_DEPTH_PROFILE}',
        ],
        name='realsense',
        output='screen',
    )

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        output='screen',
        remappings=[
            ('image_rect', '/camera/camera/color/image_raw'),
            ('camera_info', '/camera/camera/color/camera_info'),
        ],
        parameters=[apriltag_params_file],
    )

    grid_check = Node(
        package='calib_intrinsics_test',
        executable='grid_intrinsics_check',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'apriltag_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('calib_intrinsics_test'), 'tags', '36h11_grid_3x3_0.04.yaml']),
            description='apriltag_node params file declaring all 9 grid tag ids/sizes'),
        realsense,
        apriltag_node,
        grid_check,
    ])
