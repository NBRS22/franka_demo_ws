from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# RealSense D455 stream profile ('WIDTHxHEIGHTxFPS') -- same convention as
# franka_demo_bringup/franka_demo.launch.py (cf. root CLAUDE.md, "Convention
# de coordonnées"). Nothing downstream hardcodes a resolution.
REALSENSE_COLOR_PROFILE = '1280x720x30'
REALSENSE_DEPTH_PROFILE = '1280x720x30'

# Seconds to wait after starting the camera/apriltag/move_group stack before
# starting calib_pose_tour -- gives fp3_moveit_server (move_group,
# ros2_control, robot_state_publisher) and handeye_tf_publisher a head start
# on a cold real-hardware boot, which can easily take longer than the tour
# node's own internal TF-wait timeout on its own.
POSE_TOUR_START_DELAY_S = 15.0


# Root entry point for calib_ws: one command brings up everything needed for
# a full hand-eye calibration session end-to-end -- camera, apriltag
# detection, easy_handeye2's calibrator UI, calib_sample_guard's live
# reprojection-error/tilt guardrail, the real arm stack (fp3_moveit_server,
# from franka_demo_ws -- not duplicated here, source that workspace alongside
# this one), and the pose tour that drives the arm through the real pick
# working volume so samples can be taken close to the camera at varied
# orientations (cf. calib_pose_tour). run_pose_tour:=false to skip that last
# step (e.g. re-verifying an existing calibration with fp3_apriltag_demo
# instead of recalibrating, or taking samples manually by hand).
def generate_launch_description():
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    robot_ip = LaunchConfiguration('robot_ip')
    use_rviz = LaunchConfiguration('use_rviz')
    calibration_type = LaunchConfiguration('calibration_type')
    robot_effector_frame = LaunchConfiguration('robot_effector_frame')
    tracking_marker_frame = LaunchConfiguration('tracking_marker_frame')
    calibration_name = LaunchConfiguration('calibration_name')
    apriltag_params_file = LaunchConfiguration('apriltag_params_file')
    run_pose_tour = LaunchConfiguration('run_pose_tour')

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

    # scripts/launch_realsense_with_retry.sh (own copy in this package, cf.
    # fp3_apriltag_demo's identical copy -- calib_ws is self-contained, not
    # cross-referencing franka_demo_ws's franka_demo_bringup for this):
    # initial_reset:=true triggers a real USB re-enumeration of the D455, the
    # only thing confirmed to fix "Depth stream start failure" without a
    # physical unplug/replug -- but the ROS wrapper node sometimes reopens
    # the device before that finishes, crashing within a few seconds. The
    # script retries automatically when that happens.
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

    # handeye_server + rqt_calibrator -- the manual sample-taking UI (Step 4
    # of the old franka_demo_ws README, cf. handeye_tf_publisher/README.md
    # ported here). Note the calibrate.launch.py's own dummy_publisher (a
    # placeholder fp3_link0 -> camera_color_optical_frame static TF) conflicts
    # with the real RealSense TF tree during a session -- safe to kill
    # manually if RViz/TF looks wrong, cf. that README's own note; not used
    # by the actual sample-taking or solver math.
    easy_handeye2_calibrate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('easy_handeye2'), 'launch', 'calibrate.launch.py'])),
        launch_arguments={
            'calibration_type': calibration_type,
            'name': calibration_name,
            'robot_base_frame': 'fp3_link0',
            'robot_effector_frame': robot_effector_frame,
            'tracking_base_frame': 'camera_color_optical_frame',
            'tracking_marker_frame': tracking_marker_frame,
        }.items(),
    )

    # Live reprojection-error/tilt guardrail (cf. calib_sample_guard) --
    # watch its output and only click "Take sample" in rqt when it reports
    # OK, not a warning (noisy detection, or a near-head-on view at risk of
    # the planar-PnP pose-ambiguity problem).
    sample_guard = Node(
        package='calib_sample_guard',
        executable='sample_guard',
        output='screen',
        parameters=[{
            'target_tag_id': ParameterValue(LaunchConfiguration('target_tag_id'), value_type=int),
        }],
    )

    # Drives the arm through the real pick working volume for sample-taking
    # (cf. calib_pose_tour/CLAUDE.md) -- delayed to give the rest of the
    # stack (move_group, handeye_tf_publisher's TF composition) a head start
    # on a cold boot, cf. POSE_TOUR_START_DELAY_S above.
    pose_tour = TimerAction(
        period=POSE_TOUR_START_DELAY_S,
        actions=[
            Node(
                package='calib_pose_tour',
                executable='calib_pose_tour',
                output='screen',
                # Bootstraps anchor placement from the PREVIOUS saved
                # calibration under this same name (read before this
                # session's own "Save" overwrites it) -- cf.
                # calibration_pose_tour_node.py's _read_calibration_pose
                # docstring for why this can't be live TF (there is no real
                # fp3_link0 -> camera_color_optical_frame transform yet
                # during an active calibration session; only
                # easy_handeye2_calibrate's own dummy_publisher placeholder
                # exists at that point).
                parameters=[{'calibration_name': calibration_name}],
            ),
        ],
        condition=IfCondition(run_pose_tour),
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
            'calibration_type', default_value='eye_on_base',
            description='easy_handeye2 calibration type (camera fixed, tag on the effector)'),
        DeclareLaunchArgument(
            'robot_effector_frame', default_value='fp3_hand',
            description='Robot link the calibration tag is rigidly mounted on'),
        DeclareLaunchArgument(
            'tracking_marker_frame', default_value='tag36h11:0',
            description='TF frame published by apriltag_node for the calibration tag'),
        DeclareLaunchArgument(
            'calibration_name',
            default_value='fp3_link0_d455_camera_color_optical_frame_001',
            description='Name of the .calib file (without extension) to create/overwrite'),
        DeclareLaunchArgument(
            'apriltag_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('handeye_tf_publisher'), 'tags', '36h11_0_0.04.yaml']),
            description='apriltag_node params file (family/size/detector settings)'),
        DeclareLaunchArgument(
            'run_pose_tour', default_value='true',
            description=(
                'Also drive the arm through calib_pose_tour\'s pose sequence for sample-'
                'taking. Set false to bring up a plain manual calibration session '
                '(e.g. to only re-verify an existing calibration with fp3_apriltag_demo).')),
        DeclareLaunchArgument(
            'target_tag_id', default_value='0',
            description='Tag id calib_sample_guard watches (must match the calibration tag)'),
        moveit_server_bringup,
        realsense,
        apriltag_node,
        easy_handeye2_calibrate,
        sample_guard,
        pose_tour,
    ])
