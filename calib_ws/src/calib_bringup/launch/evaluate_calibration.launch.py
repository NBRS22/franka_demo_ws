from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

REALSENSE_COLOR_PROFILE = '1280x720x30'
REALSENSE_DEPTH_PROFILE = '1280x720x30'

# D405 (eye-in-hand, cf. calib_eye_in_hand) -- short-range, no depth needed
# (apriltag_node only reads color + camera_info). Same rationale/values as
# calib_eye_in_hand/launch/calibrate_eye_in_hand.launch.py and
# fp3_apriltag_demo/launch/apriltag_move_once.launch.py.
D405_COLOR_PROFILE = '848x480x30'


# Quantitative check of an ALREADY-SAVED calibration (easy_handeye2's
# rqt_evaluator, "Maximum divergence") -- not a calibration session (no
# calibrate.launch.py/dummy_publisher here) and not the physical grasp
# check (that's fp3_apriltag_demo). Brings up the real arm + camera +
# apriltag_node (single calibration tag) + handeye_tf_publisher (publishes
# the calibration under test, which rqt_evaluator needs a live TF chain
# through to measure against -- cf. handeye_rqt_evaluator_widget.py) +
# easy_handeye2's evaluate.launch.py.
#
# Usage: move the arm to a few NEW poses (not the ones used for
# calibration), tag kept visible, waiting for a steady state at each one --
# the rqt window's "Maximum divergence" field is the residual error. cf.
# handeye_tf_publisher/README.md, "Step 8 -- Verify the calibration" for the
# rough-guide thresholds (<1cm good, 1-3cm usable/tight, >3cm re-calibrate).
def generate_launch_description():
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    robot_ip = LaunchConfiguration('robot_ip')
    use_rviz = LaunchConfiguration('use_rviz')
    calibration_name = LaunchConfiguration('calibration_name')
    apriltag_params_file = LaunchConfiguration('apriltag_params_file')
    camera = LaunchConfiguration('camera')

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

    realsense_d455 = ExecuteProcess(
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
        condition=LaunchConfigurationEquals('camera', 'd455'),
    )

    realsense_d405 = ExecuteProcess(
        cmd=[
            PathJoinSubstitution(
                [FindPackageShare('calib_bringup'), 'scripts', 'launch_realsense_with_retry.sh']),
            'enable_depth:=false',
            'initial_reset:=true',
            'log_level:=warn',
            f'rgb_camera.color_profile:={D405_COLOR_PROFILE}',
        ],
        name='realsense',
        output='screen',
        condition=LaunchConfigurationEquals('camera', 'd405'),
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

    easy_handeye2_evaluate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('easy_handeye2'), 'launch', 'evaluate.launch.py'])),
        launch_arguments={
            'name': calibration_name,
        }.items(),
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
            'camera', default_value='d455', choices=['d455', 'd405'],
            description=(
                'Which camera to stream: d455 (eye-on-base) or d405 (eye-in-hand, cf. '
                'calib_eye_in_hand). handeye_tf_publisher handles both calibration types '
                'transparently (reads calibration_type from the .calib file) -- this only '
                'picks the RealSense profile/args.')),
        DeclareLaunchArgument(
            'calibration_name',
            default_value='fp3_link0_d455_camera_color_optical_frame_001',
            description=(
                'Name of the .calib file (without extension) to evaluate -- must match '
                'the camera:= selected above (default here is the d455 one; pass the '
                'saved d405 name when camera:=d405).')),
        DeclareLaunchArgument(
            'apriltag_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('handeye_tf_publisher'), 'tags', '36h11_0_0.04.yaml']),
            description='apriltag_node params file (family/size/detector settings)'),
        moveit_server_bringup,
        realsense_d455,
        realsense_d405,
        apriltag_node,
        handeye_tf_publisher,
        easy_handeye2_evaluate,
    ])
