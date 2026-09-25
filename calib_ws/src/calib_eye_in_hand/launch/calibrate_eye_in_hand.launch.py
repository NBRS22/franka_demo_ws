from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# D405 is short-range (optimal ~7-50cm); no need for the D455's 1280x720
# profile here, and depth isn't used at all -- apriltag_node only needs
# color + camera_info. Kept modest to reduce USB load once a second camera
# (the D455) is also connected for the later cross-check step.
REALSENSE_COLOR_PROFILE = '848x480x30'


# Eye-in-hand calibration of the wrist-mounted D405 -- step 1 of the
# cross-validation plan (cf. CLAUDE.md): the existing eye-on-base D455
# calibration can't be verified against itself (the verification target IS
# the calibration's own output, cf. fp3_apriltag_demo's circularity finding),
# so this calibrates a SECOND, independent camera rigidly mounted on the
# wrist, against a STATIC tag placed in the world -- the mirror image of the
# D455 session (there, the tag moves with the effector in front of a fixed
# camera; here, the camera moves with the effector in front of a fixed tag).
#
# Deliberately reuses the *same* physical tag (36h11 id 0) the D455
# calibration already uses -- not a new one -- so that step 2 (not yet
# built: cross_check_node, comparing both cameras' fp3_link0-frame estimate
# of this one tag) has a single ground-truth target common to both.
#
# robot_effector_frame is fp3_hand: the Franka Hand was remounted (the D405
# bracket sits alongside/on it, not in place of it) -- load_gripper:=true is
# the correct default here, matching fp3_moveit_server's own default. This
# also matches the frame convention already used for the D455 calibration's
# own robot_effector_frame (calib_bringup.launch.py's default), so both
# calibrations share the same effector reference frame.
#
# No calib_pose_tour equivalent yet for this geometry (that node drives the
# EFFECTOR to face a FIXED camera -- the opposite problem here, where the
# camera is on the effector and the tag is fixed). Take samples by jogging
# the arm manually (Desk, or MoveIt Servo/rqt motion planning via the
# fp3_moveit_server bringup this file already includes) through varied
# positions/orientations while keeping the tag in the D405's view -- same
# sampling guidance as the D455 session (README: >=3 non-parallel rotation
# axes, avoid tilts >60deg, avoid near-head-on views).
def generate_launch_description():
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    robot_ip = LaunchConfiguration('robot_ip')
    use_rviz = LaunchConfiguration('use_rviz')
    start_arm_stack = LaunchConfiguration('start_arm_stack')
    load_gripper = LaunchConfiguration('load_gripper')
    robot_effector_frame = LaunchConfiguration('robot_effector_frame')
    tracking_marker_frame = LaunchConfiguration('tracking_marker_frame')
    calibration_name = LaunchConfiguration('calibration_name')
    apriltag_params_file = LaunchConfiguration('apriltag_params_file')
    camera_serial_no = LaunchConfiguration('camera_serial_no')

    moveit_server_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('fp3_moveit_server'), 'launch', 'bringup.launch.py'])),
        launch_arguments={
            'use_fake_hardware': use_fake_hardware,
            'robot_ip': robot_ip,
            'use_rviz': use_rviz,
            'load_gripper': load_gripper,
        }.items(),
        condition=IfCondition(start_arm_stack),
    )

    # Unnamespaced (plain /camera/camera/... topics) -- same convention as
    # calib_bringup's D455 session. Deliberately NOT run alongside the D455
    # here: this step calibrates the D405 alone, so there's no topic
    # collision to worry about yet. The later cross-check step (both
    # cameras live at once) will need camera_name:=/camera_namespace:=
    # namespacing + distinct serial_no per device -- not needed here.
    # ros2 launch's own CLI arg parser rejects a trailing-empty 'name:='
    # token as malformed (raises even though rs_launch.py's own serial_no
    # default is '' / "any device") -- so the arg can't just be passed
    # through with an empty LaunchConfiguration value like the other args
    # above. Split into two conditioned ExecuteProcess actions instead: only
    # append serial_no:=<value> at all when camera_serial_no is non-empty.
    _realsense_base_cmd = [
        PathJoinSubstitution(
            [FindPackageShare('calib_eye_in_hand'), 'scripts', 'launch_realsense_with_retry.sh']),
        'enable_depth:=false',
        'initial_reset:=true',
        'log_level:=warn',
        f'rgb_camera.color_profile:={REALSENSE_COLOR_PROFILE}',
    ]

    realsense_default = ExecuteProcess(
        cmd=_realsense_base_cmd,
        name='realsense',
        output='screen',
        condition=LaunchConfigurationEquals('camera_serial_no', ''),
    )

    realsense_with_serial = ExecuteProcess(
        cmd=_realsense_base_cmd + [['serial_no:=', camera_serial_no]],
        name='realsense',
        output='screen',
        condition=LaunchConfigurationNotEquals('camera_serial_no', ''),
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

    # handeye_server + rqt_calibrator -- same manual sample-taking UI as the
    # D455 session (cf. calib_bringup.launch.py). tracking_base_frame is the
    # D405's own color optical frame (default RealSense frame naming,
    # camera_name left at its default 'camera' since this camera is
    # unnamespaced here).
    easy_handeye2_calibrate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('easy_handeye2'), 'launch', 'calibrate.launch.py'])),
        launch_arguments={
            'calibration_type': 'eye_in_hand',
            'name': calibration_name,
            'robot_base_frame': 'fp3_link0',
            'robot_effector_frame': robot_effector_frame,
            'tracking_base_frame': 'camera_color_optical_frame',
            'tracking_marker_frame': tracking_marker_frame,
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
            'start_arm_stack', default_value='true',
            description=(
                'Start fp3_moveit_server (MoveIt + ros2_control, the arm is then held '
                'stiff by fp3_arm_controller). Set false to hand-guide the arm: start '
                'franka_bringup\'s gravity_compensation_example_controller yourself '
                'instead (only ONE process may hold the robot connection), which also '
                'provides robot_state_publisher / joint states.')),
        DeclareLaunchArgument(
            'load_gripper', default_value='true',
            description=(
                'Franka Hand is remounted -- default true. Set false only if it gets '
                'removed again for some other mounting attempt.')),
        DeclareLaunchArgument(
            'robot_effector_frame', default_value='fp3_hand',
            description='Robot link the D405 bracket is rigidly mounted on'),
        DeclareLaunchArgument(
            'tracking_marker_frame', default_value='tag36h11:0',
            description=(
                'TF frame published by apriltag_node for the (now static, world-fixed) '
                'calibration tag -- same physical tag as the D455 eye-on-base session, '
                'kept identical on purpose for the future cross-check.')),
        DeclareLaunchArgument(
            'calibration_name',
            default_value='fp3_hand_d405_camera_color_optical_frame_001',
            description='Name of the .calib file (without extension) to create/overwrite'),
        DeclareLaunchArgument(
            'apriltag_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('handeye_tf_publisher'), 'tags', '36h11_0_0.04.yaml']),
            description=(
                'apriltag_node params file (family/size/detector settings). Default is the '
                'same 4cm tag used for the D455 session -- override if the D405\'s working '
                'distance to the fixed tag needs a smaller/larger physical tag.')),
        DeclareLaunchArgument(
            'camera_serial_no', default_value='',
            description=(
                'D405 serial number. Empty = auto-pick whichever RealSense is connected -- '
                'only safe while the D455 is unplugged/not running.')),
        moveit_server_bringup,
        realsense_default,
        realsense_with_serial,
        apriltag_node,
        easy_handeye2_calibrate,
    ])
