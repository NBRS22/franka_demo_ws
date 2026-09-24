from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

REALSENSE_COLOR_PROFILE = '1280x720x30'
REALSENSE_DEPTH_PROFILE = '1280x720x30'


# Brings up the real arm + camera + apriltag_node (single calibration tag) +
# handeye_tf_publisher (publishes the calibration under test) and runs
# calib_axis_test: does the calibration's ROTATION correctly map camera-frame
# directions to fp3_link0 directions? A per-axis, known, pure displacement
# check -- catches an axis-swap/sign-flip bug that calib_intrinsics_test
# could NOT (it only ever compared displacement magnitudes, never direction).
# No easy_handeye2 here (same reasoning as rigidity_check.launch.py) --
# avoids the calibrate.launch.py dummy_publisher TF conflict entirely.
def generate_launch_description():
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    robot_ip = LaunchConfiguration('robot_ip')
    use_rviz = LaunchConfiguration('use_rviz')
    apriltag_params_file = LaunchConfiguration('apriltag_params_file')
    calibration_name = LaunchConfiguration('calibration_name')

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

    realsense = ExecuteProcess(
        cmd=[
            PathJoinSubstitution(
                [FindPackageShare('calib_bringup'), 'scripts', 'launch_realsense_with_retry.sh']),
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

    handeye_tf_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('handeye_tf_publisher'), 'launch', 'publish.launch.py'])),
        launch_arguments={
            'calibration_name': calibration_name,
        }.items(),
    )

    axis_test = Node(
        package='calib_axis_test',
        executable='axis_test',
        output='screen',
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
            'apriltag_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('handeye_tf_publisher'), 'tags', '36h11_0_0.04.yaml']),
            description='apriltag_node params file for the single calibration tag (id 0)'),
        DeclareLaunchArgument(
            'calibration_name',
            default_value='fp3_link0_d455_camera_color_optical_frame_001',
            description='Name of the .calib file (without extension) whose rotation is being tested'),
        moveit_server_bringup,
        realsense,
        apriltag_node,
        handeye_tf_publisher,
        axis_test,
    ])
