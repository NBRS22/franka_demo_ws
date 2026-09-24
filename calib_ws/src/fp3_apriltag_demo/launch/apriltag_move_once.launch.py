from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
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

# D405 (eye-in-hand, cf. calib_eye_in_hand) -- short-range, no depth needed
# here either (apriltag_node only reads color + camera_info). Kept distinct
# from the D455's profile above rather than reused, same rationale as
# calib_eye_in_hand/launch/calibrate_eye_in_hand.launch.py.
D405_COLOR_PROFILE = '848x480x30'


# Brings up the full arm stack (fp3_moveit_server/bringup.launch.py:
# move_group, ros2_control, scene_setup_node, pick_place_node,
# command_router_node -- the last two aren't actually used by this node
# anymore, cf. below, but scene_setup_node/move_group are), this workspace's
# own handeye_tf_publisher (the eye-on-base calibration being checked), the
# RealSense driver, apriltag_ros' apriltag_node, and apriltag_move_once_node
# -- fully self-contained. apriltag_move_once_node no longer calls mtc_pick
# at all (calib_ws's own simplification, cf. that node's module docstring):
# it moves fp3_hand_tcp straight to the tag's pose via /move_action directly
# and stops there, gripper open, no grasp/lift -- so you can compare
# fp3_hand_tcp against the tag's own live TF frame afterward instead of
# relying on a grasp succeeding/failing as a proxy.
def generate_launch_description():
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    robot_ip = LaunchConfiguration('robot_ip')
    use_rviz = LaunchConfiguration('use_rviz')
    tag_size = LaunchConfiguration('tag_size')
    target_tag_id = LaunchConfiguration('target_tag_id')
    flip_tag_orientation = LaunchConfiguration('flip_tag_orientation')
    force_top_down = LaunchConfiguration('force_top_down')
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

    # scripts/launch_realsense_with_retry.sh (own copy in this package -- calib_ws is
    # self-contained, not cross-referencing franka_demo_ws's franka_demo_bringup for
    # this): initial_reset:=true triggers a real USB re-enumeration of the D455, the
    # only thing confirmed to fix "Depth stream start failure" without a physical
    # unplug/replug -- but the ROS wrapper node sometimes reopens the device before
    # that finishes, crashing within a few seconds. The script retries automatically
    # when that happens. Running it as its own OS process also gives it its own
    # LaunchConfiguration namespace for free.
    # pointcloud.enable:=true -- for visual sanity-checking in RViz only (cf.
    # handeye_tf_publisher/README.md, "Recommended extra check": confirm the
    # tag's TF frame visually sits on the physical tag/cube in the point
    # cloud). NOT used for anything quantitative here or anywhere else in
    # this pipeline: align_depth.enable + pointcloud.enable combined is a
    # known unresolved realsense-ros bug (native cloud spatially offset by a
    # few cm from the true aligned-depth geometry -- cf. root CLAUDE.md,
    # dette technique "Important", issues #2595/#3050) -- this is exactly why
    # create_pointcloud_node deprojects manually instead of consuming this
    # topic. Eyeball only, never trust it for a real measurement.
    realsense_d455 = ExecuteProcess(
        cmd=[
            PathJoinSubstitution(
                [FindPackageShare('fp3_apriltag_demo'), 'scripts', 'launch_realsense_with_retry.sh']),
            'align_depth.enable:=true',
            'pointcloud.enable:=true',
            'initial_reset:=true',
            'log_level:=warn',
            f'rgb_camera.color_profile:={REALSENSE_COLOR_PROFILE}',
            f'depth_module.depth_profile:={REALSENSE_DEPTH_PROFILE}',
        ],
        name='realsense',
        output='screen',
        condition=LaunchConfigurationEquals('camera', 'd455'),
    )

    # D405: no depth/pointcloud (unused by this test either way, cf. root
    # CLAUDE.md on the pointcloud.enable+align_depth bug -- moot here since
    # apriltag_node never reads depth), unnamespaced -- same convention as
    # calib_eye_in_hand (single camera at a time, no topic collision to
    # avoid yet).
    realsense_d405 = ExecuteProcess(
        cmd=[
            PathJoinSubstitution(
                [FindPackageShare('fp3_apriltag_demo'), 'scripts', 'launch_realsense_with_retry.sh']),
            'enable_depth:=false',
            'initial_reset:=true',
            'log_level:=warn',
            f'rgb_camera.color_profile:={D405_COLOR_PROFILE}',
        ],
        name='realsense',
        output='screen',
        condition=LaunchConfigurationEquals('camera', 'd405'),
    )

    # handeye_tf_publisher for BOTH camera types now (not easy_handeye2's
    # own handeye_publisher for d405): publishing straight to
    # tracking_base_frame (camera_color_optical_frame) -- what
    # easy_handeye2's publish.launch.py does -- gives that frame a SECOND,
    # conflicting parent on top of the one the camera driver's own static TF
    # chain already publishes for it, which is exactly the "two or more
    # unconnected trees" TF error hit when this used easy_handeye2's
    # publisher for d405. handeye_tf_publisher always composes through
    # camera_link_frame instead (that chain's actual root, no parent of its
    # own) -- now generalized to read calibration_type from the .calib file
    # and pick robot_effector_frame (eye_in_hand) vs robot_base_frame
    # (eye_on_base) as the frame it publishes FROM, cf.
    # handeye_tf_publisher/publisher_node.py.
    handeye_tf_publisher_include = IncludeLaunchDescription(
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
            'flip_tag_orientation': ParameterValue(flip_tag_orientation, value_type=bool),
            'force_top_down': ParameterValue(force_top_down, value_type=bool),
            'offset_x': ParameterValue(LaunchConfiguration('offset_x'), value_type=float),
            'offset_y': ParameterValue(LaunchConfiguration('offset_y'), value_type=float),
            'offset_z': ParameterValue(LaunchConfiguration('offset_z'), value_type=float),
            'gripper_width_m': ParameterValue(LaunchConfiguration('gripper_width_m'), value_type=float),
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
            'flip_tag_orientation', default_value='true',
            description=(
                'Default true: compose the tag\'s own orientation with a 180deg flip about '
                'its local X, instead of using it as-is -- a tag facing the camera has its '
                'normal pointing the way the gripper approaches FROM, not a reachable '
                'direction, so fp3_hand_tcp can never literally match the tag\'s raw '
                'orientation. The flip keeps the tag\'s real tilt (unlike the old fixed '
                'straight-down constant) while making the target orientation reachable. Set '
                'false only if the tag is mounted such that its raw orientation is already a '
                'valid approach direction (e.g. a vertical face, not lying flat).')),
        DeclareLaunchArgument(
            'force_top_down', default_value='false',
            description=(
                'Default false: keep flip_tag_orientation\'s behaviour (tilt-preserving flip). '
                'Set true to ignore the tag\'s orientation entirely and force a pure vertical '
                'descent instead (fp3_hand_tcp local +Z straight along world -Z) -- only the '
                'tag\'s position is used, not its orientation. Takes priority over '
                'flip_tag_orientation when both are set.')),
        DeclareLaunchArgument(
            'offset_x', default_value='0.0',
            description='Manual correction added to the target, meters, along fp3_link0 X (test only).'),
        DeclareLaunchArgument(
            'offset_y', default_value='0.0',
            description='Same, along fp3_link0 Y.'),
        DeclareLaunchArgument(
            'offset_z', default_value='0.0',
            description='Same, along fp3_link0 Z.'),
        DeclareLaunchArgument(
            'gripper_width_m', default_value='0.08',
            description=(
                'Target gripper width before the move, meters. Default 0.08 = fully open. '
                'Set near 0.0 to close the gripper instead (e.g. to align its closed tip '
                'against a tag bigger than the gripper opening).')),
        DeclareLaunchArgument(
            'camera', default_value='d455', choices=['d455', 'd405'],
            description=(
                'Which camera to stream: d455 (eye-on-base) or d405 (eye-in-hand, cf. '
                'calib_eye_in_hand). handeye_tf_publisher itself handles both calibration '
                'types transparently (reads calibration_type from the .calib file) -- this '
                'only picks the RealSense profile/args.')),
        DeclareLaunchArgument(
            'calibration_name',
            default_value='fp3_link0_d455_camera_color_optical_frame_001',
            description=(
                'Name of the easy_handeye2 .calib file (without extension) to load and '
                'publish -- must match the camera:= selected above (its own default '
                'here is the d455 one; pass the saved d405 name when camera:=d405).')),
        DeclareLaunchArgument(
            'apriltag_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('handeye_tf_publisher'), 'tags', '36h11_0_0.04.yaml']),
            description=(
                'apriltag_node params file (family/size/detector settings) -- same file '
                'used during the eye-on-base calibration.')),
        moveit_server_bringup,
        realsense_d455,
        realsense_d405,
        handeye_tf_publisher_include,
        apriltag_node,
        apriltag_move_once_node,
    ])
