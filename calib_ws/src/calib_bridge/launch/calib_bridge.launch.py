from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# Neither camera needs depth here -- both apriltag_node instances only read
# color + camera_info (cf. bridge_calibration_node.py's own docstring on why
# apriltag_msgs never carries a 3D pose, so this whole pipeline only ever
# consumes color images + 2D corners).
D455_COLOR_PROFILE = '1280x720x30'
D405_COLOR_PROFILE = '848x480x30'


# Brings up BOTH cameras simultaneously (unlike calib_eye_in_hand and
# fp3_apriltag_demo, which only ever run one camera at a time) -- required
# for calib_bridge's whole premise: seeing the same static tag from both
# cameras at once. Both are namespaced under a common 'franka' ROS namespace
# with distinct camera_name (camera_namespace:=franka camera_name:=d455/d405
# -> topics under /franka/d455/... and /franka/d405/...) so nothing collides
# and both cameras read symmetrically-named topics -- unlike
# calib_eye_in_hand/fp3_apriltag_demo's D455, which stays on the plain
# /camera/camera/... default there (single camera at a time, nothing to
# namespace against). apriltag_node's own topics (/detections included) are
# namespaced the same way (Node(namespace='franka/d455'/'franka/d405', ...)),
# and the D405 instance also gets its own tag.frames override so the two
# apriltag_node instances don't both broadcast TF for the same
# tag<family>:<id> frame with two different parents (cf.
# bridge_calibration_node.py's module docstring -- moot for this node's own
# math since it never uses that TF broadcast, but a real conflict for anyone
# watching RViz).
#
# Serial numbers matter here in a way they didn't for calib_eye_in_hand/
# fp3_apriltag_demo (which only ever ran one camera, so "any device" was
# unambiguous): with both RealSense units connected at once, an empty
# serial_no is genuinely ambiguous -- find them with `rs-enumerate-devices -s`
# and pass both d455_serial_no/d405_serial_no explicitly.
def generate_launch_description():
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    robot_ip = LaunchConfiguration('robot_ip')
    use_rviz = LaunchConfiguration('use_rviz')
    target_tag_id = LaunchConfiguration('target_tag_id')
    tag_size = LaunchConfiguration('tag_size')
    apriltag_params_file = LaunchConfiguration('apriltag_params_file')
    d405_calibration_name = LaunchConfiguration('d405_calibration_name')
    output_calibration_name = LaunchConfiguration('output_calibration_name')
    d455_serial_no = LaunchConfiguration('d455_serial_no')
    d405_serial_no = LaunchConfiguration('d405_serial_no')

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

    _d455_base_cmd = [
        PathJoinSubstitution(
            [FindPackageShare('calib_bridge'), 'scripts', 'launch_realsense_with_retry.sh']),
        'camera_name:=d455',
        'camera_namespace:=franka',
        # tf_prefix decouples topic naming from TF frame naming: without it,
        # camera_name:=d455 would ALSO rename every frame_id to
        # d455_color_optical_frame etc, breaking the existing D455 .calib
        # (its tracking_base_frame is 'camera_color_optical_frame', saved
        # back when this camera was calibrated unnamespaced) and every tool
        # that still runs the D455 unnamespaced afterward (handeye_tf_
        # publisher, evaluate_calibration.launch.py camera:=d455,
        # fp3_apriltag_demo camera:=d455). Forcing tf_prefix:=camera keeps
        # frame_ids exactly as before -- only topics move under /franka/d455.
        'tf_prefix:=camera',
        'enable_depth:=false',
        'initial_reset:=true',
        'log_level:=warn',
        f'rgb_camera.color_profile:={D455_COLOR_PROFILE}',
    ]
    realsense_d455_default = ExecuteProcess(
        cmd=_d455_base_cmd, name='realsense_d455', output='screen',
        condition=LaunchConfigurationEquals('d455_serial_no', ''))
    # Single-quoted, not just 'serial_no:=<value>': rs_launch.py forwards
    # this straight into a Node parameter via a bare LaunchConfiguration
    # (no ParameterValue(..., value_type=str)), so launch_ros YAML-parses
    # the resolved string to infer its type -- an all-digit serial number
    # like '234322305396' would come out as an int, which the driver then
    # rejects ("parameter 'serial_no' ... is of type {string}, setting it
    # to {integer} is not allowed"). Wrapping it in literal single quotes
    # makes it parse as a YAML string instead, matching this launch file's
    # own default for this same argument (default_value="''").
    realsense_d455_with_serial = ExecuteProcess(
        cmd=_d455_base_cmd + [["serial_no:='", d455_serial_no, "'"]], name='realsense_d455', output='screen',
        condition=LaunchConfigurationNotEquals('d455_serial_no', ''))

    # No tf_prefix override needed here (unlike the D455 above): this node's
    # own solvePnP-based math never looks up the D405's frame_id through TF
    # or reads it from the saved D405 .calib for anything but the effector
    # frame, so leaving it at the camera_name-derived default
    # ('d405_color_optical_frame') is harmless -- and it must NOT be forced
    # to 'camera_color_optical_frame' like the D455's, since that would
    # collide with the D455's own live frame_id in the same TF tree.
    _d405_base_cmd = [
        PathJoinSubstitution(
            [FindPackageShare('calib_bridge'), 'scripts', 'launch_realsense_with_retry.sh']),
        'camera_name:=d405',
        'camera_namespace:=franka',
        'enable_depth:=false',
        'initial_reset:=true',
        'log_level:=warn',
        f'rgb_camera.color_profile:={D405_COLOR_PROFILE}',
    ]
    realsense_d405_default = ExecuteProcess(
        cmd=_d405_base_cmd, name='realsense_d405', output='screen',
        condition=LaunchConfigurationEquals('d405_serial_no', ''))
    # Same single-quoting as the D455's serial_no above, same reason.
    realsense_d405_with_serial = ExecuteProcess(
        cmd=_d405_base_cmd + [["serial_no:='", d405_serial_no, "'"]], name='realsense_d405', output='screen',
        condition=LaunchConfigurationNotEquals('d405_serial_no', ''))

    apriltag_node_d455 = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node_d455',
        namespace='franka/d455',
        output='screen',
        remappings=[
            ('image_rect', '/franka/d455/color/image_raw'),
            ('camera_info', '/franka/d455/color/camera_info'),
        ],
        parameters=[apriltag_params_file],
    )

    apriltag_node_d405 = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node_d405',
        namespace='franka/d405',
        output='screen',
        remappings=[
            ('image_rect', '/franka/d405/color/image_raw'),
            ('camera_info', '/franka/d405/color/camera_info'),
        ],
        parameters=[
            apriltag_params_file,
            # Overrides just this one field from apriltag_params_file above
            # (later entries in this list win) -- gives this instance's tag
            # TF broadcast a distinct frame name so it doesn't conflict with
            # apriltag_node_d455's broadcast for the same physical tag id.
            {'tag.frames': ['tag36h11_d405:0']},
        ],
    )

    bridge_calibration_node = Node(
        package='calib_bridge',
        executable='bridge_calibration_node',
        output='screen',
        parameters=[{
            'target_tag_id': ParameterValue(target_tag_id, value_type=int),
            'tag_size': ParameterValue(tag_size, value_type=float),
            'd405_calibration_name': d405_calibration_name,
            'output_calibration_name': output_calibration_name,
            # Must match the apriltag_node_d455/apriltag_node_d405
            # namespaces + remappings above -- not this node's own built-in
            # defaults (those are the plain /camera/... /d405/... scheme
            # from before this was switched to the franka/d455, franka/d405
            # convention).
            'd455_camera_info_topic': '/franka/d455/color/camera_info',
            'd455_detections_topic': '/franka/d455/detections',
            'd405_camera_info_topic': '/franka/d405/color/camera_info',
            'd405_detections_topic': '/franka/d405/detections',
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
            'target_tag_id', default_value='0',
            description='AprilTag id both cameras must see (the static, world-fixed tag)'),
        DeclareLaunchArgument(
            'tag_size', default_value='0.04',
            description='Tag physical size in meters -- same tag for both cameras'),
        DeclareLaunchArgument(
            'apriltag_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('handeye_tf_publisher'), 'tags', '36h11_0_0.04.yaml']),
            description='apriltag_node params file, shared by both camera instances'),
        DeclareLaunchArgument(
            'd405_calibration_name',
            default_value='fp3_hand_d405_camera_color_optical_frame_001',
            description='Name of the (trusted) D405 eye-in-hand .calib file to bridge through'),
        DeclareLaunchArgument(
            'output_calibration_name',
            default_value='fp3_link0_d455_camera_color_optical_frame_derived_001',
            description=(
                'Name of the .calib file this node writes -- deliberately distinct from the '
                'existing D455 eye-on-base one so you can compare/evaluate both before '
                'deciding to replace it.')),
        DeclareLaunchArgument(
            'd455_serial_no', default_value='',
            description='D455 serial number -- required (non-empty) once both cameras are connected at once'),
        DeclareLaunchArgument(
            'd405_serial_no', default_value='',
            description='D405 serial number -- required (non-empty) once both cameras are connected at once'),
        moveit_server_bringup,
        realsense_d455_default,
        realsense_d455_with_serial,
        realsense_d405_default,
        realsense_d405_with_serial,
        apriltag_node_d455,
        apriltag_node_d405,
        bridge_calibration_node,
    ])
