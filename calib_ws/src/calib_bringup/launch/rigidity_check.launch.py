from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

REALSENSE_COLOR_PROFILE = '1280x720x30'
REALSENSE_DEPTH_PROFILE = '1280x720x30'


# Brings up the real arm (fp3_moveit_server, from franka_demo_ws) + camera +
# apriltag_node (single calibration tag, id 0) + handeye_tf_publisher
# (publishes fp3_link0 -> camera_link from the currently-loaded calibration,
# which calib_rigidity_test looks up live via TF to know which way to face
# the arm before starting -- cf. that node's lookup_camera_position(), same
# pattern as calib_pose_tour) and runs calib_rigidity_test: is the
# calibration tag/cube rigidly fixed to fp3_hand across a large orientation
# swing? No easy_handeye2 here (unlike calib_bringup.launch.py) -- this
# doesn't need a calibration session running, so it also avoids that launch
# file's dummy_publisher TF conflict entirely.
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

    rigidity_test = Node(
        package='calib_rigidity_test',
        executable='rigidity_test',
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
            description=(
                'Name of the easy_handeye2 .calib file (without extension) that '
                'handeye_tf_publisher should load and publish -- used to orient the arm '
                'toward the camera before the rigidity test starts. An imperfect '
                'calibration is fine here, only the orientation seed depends on it.')),
        moveit_server_bringup,
        realsense,
        apriltag_node,
        handeye_tf_publisher,
        rigidity_test,
    ])
