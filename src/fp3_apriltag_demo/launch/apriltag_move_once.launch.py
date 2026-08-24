from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# RealSense D455 stream profile ('WIDTHxHEIGHTxFPS') -- color and depth kept
# identical, same convention as franka_demo_bringup/franka_demo.launch.py
# (cf. CLAUDE.md racine, "Convention de coordonnées"). Nothing in this
# node's own code hardcodes a resolution.
REALSENSE_COLOR_PROFILE = '1280x720x30'
REALSENSE_DEPTH_PROFILE = '1280x720x30'


# Brings up the full arm stack (fp3_moveit_server/bringup.launch.py:
# move_group, ros2_control, scene_setup_node, pick_place_node,
# command_router_node), this workspace's own handeye_tf_publisher (the
# eye-on-base calibration being checked), the RealSense driver, apriltag_ros'
# apriltag_node, and apriltag_move_once_node -- fully self-contained, unlike
# the older version of this demo (a separate franka_demo_ws) which assumed
# the camera and the calibration were already running elsewhere.
def generate_launch_description():
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    robot_ip = LaunchConfiguration('robot_ip')
    use_rviz = LaunchConfiguration('use_rviz')
    tag_size = LaunchConfiguration('tag_size')
    target_tag_id = LaunchConfiguration('target_tag_id')
    force_gripper_down = LaunchConfiguration('force_gripper_down')
    calibration_name = LaunchConfiguration('calibration_name')
    apriltag_params_file = LaunchConfiguration('apriltag_params_file')

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

    # Same retry-wrapped RealSense launch as franka_demo_bringup/franka_demo.launch.py
    # (cf. that file's own comment + franka_demo_bringup/CLAUDE.md "Dépannage"):
    # initial_reset:=true triggers a real USB re-enumeration of the D455, the only
    # thing confirmed to fix "Depth stream start failure" without a physical
    # unplug/replug -- but the ROS wrapper node sometimes reopens the device before
    # that finishes, crashing within a few seconds. launch_realsense_with_retry.sh
    # retries automatically when that happens. Running it as its own OS process
    # also gives it its own LaunchConfiguration namespace for free -- the old
    # GroupAction(scoped=True, forwarding=False) isolation is no longer needed.
    realsense = ExecuteProcess(
        cmd=[
            PathJoinSubstitution(
                [FindPackageShare('franka_demo_bringup'), 'scripts', 'launch_realsense_with_retry.sh']),
            'align_depth.enable:=true',
            'initial_reset:=true',
            'log_level:=warn',
            f'rgb_camera.color_profile:={REALSENSE_COLOR_PROFILE}',
            f'depth_module.depth_profile:={REALSENSE_DEPTH_PROFILE}',
        ],
        name='realsense',
        output='screen',
    )

    handeye_tf_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('handeye_tf_publisher'), 'launch', 'publish.launch.py'])),
        launch_arguments={
            'calibration_name': calibration_name,
        }.items(),
    )

    # Same node/remappings/params-file used manually throughout the
    # calibration session (cf. handeye_tf_publisher/README.md). No
    # visualize_segmentation-style separate node here -- apriltag_node
    # publishes /detections and its own TF (tag<family>:<id>) directly.
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

    apriltag_move_once_node = Node(
        package='fp3_apriltag_demo',
        executable='apriltag_move_once_node',
        output='screen',
        parameters=[{
            # ParameterValue(..., value_type=...) required: a bare
            # LaunchConfiguration resolves to a string at launch time, which
            # would mismatch the int/float types declare_parameter() expects
            # in apriltag_move_once_node.py.
            'tag_size': ParameterValue(tag_size, value_type=float),
            'target_tag_id': ParameterValue(target_tag_id, value_type=int),
            'force_gripper_down': ParameterValue(force_gripper_down, value_type=bool),
        }],
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
            'tag_size', default_value='0.04',
            description=(
                'AprilTag physical size: outer black square side, in meters. '
                'Must match the "size" declared in apriltag_params_file below.')),
        DeclareLaunchArgument(
            'target_tag_id', default_value='0',
            description='Which AprilTag id to wait for on /detections'),
        DeclareLaunchArgument(
            'force_gripper_down', default_value='true',
            description=(
                'Keep the tag\'s computed position but replace its orientation with a '
                'fixed straight-down TCP orientation (recommended: the tag\'s own '
                'orientation is often kinematically unreachable).')),
        DeclareLaunchArgument(
            'calibration_name',
            default_value='fp3_link0_d455_camera_color_optical_frame_001',
            description=(
                'Name of the easy_handeye2 .calib file (without extension) that '
                'handeye_tf_publisher should load and publish.')),
        DeclareLaunchArgument(
            'apriltag_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('handeye_tf_publisher'), 'tags', '36h11_0_0.04.yaml']),
            description=(
                'apriltag_node params file (family/size/detector settings) -- same file '
                'used during the eye-on-base calibration.')),
        moveit_server_bringup,
        realsense,
        handeye_tf_publisher,
        apriltag_node,
        apriltag_move_once_node,
    ])
