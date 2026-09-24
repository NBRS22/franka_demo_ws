from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

REALSENSE_COLOR_PROFILE = '1280x720x30'
REALSENSE_DEPTH_PROFILE = '1280x720x30'


# Self-contained interactive calibration check: click any point in the point
# cloud in RViz (the "Publish Point" tool) and the arm moves fp3_hand_tcp
# there (position only, no gripper action) -- cf. click_to_point_node.py.
# Brings up the real arm + camera (WITH pointcloud.enable:=true this time --
# unlike every other launch file in this workspace, the point cloud is the
# whole point here, not just an eyeball sanity check; still subject to the
# same known realsense-ros align_depth+pointcloud spatial-offset bug, cf.
# root CLAUDE.md dette technique "Important" -- keep that in mind when
# judging how close the arm lands) + handeye_tf_publisher (publishes the
# calibration under test, positions the cloud in fp3_link0 for RViz).
def generate_launch_description():
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    robot_ip = LaunchConfiguration('robot_ip')
    use_rviz = LaunchConfiguration('use_rviz')
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
                [FindPackageShare('calib_click_test'), 'scripts', 'launch_realsense_with_retry.sh']),
            'align_depth.enable:=true',
            'pointcloud.enable:=true',
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

    click_test = Node(
        package='calib_click_test',
        executable='click_test',
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
            'calibration_name',
            default_value='fp3_link0_d455_camera_color_optical_frame_001',
            description='Name of the .calib file (without extension) whose accuracy is being probed'),
        moveit_server_bringup,
        realsense,
        handeye_tf_publisher,
        click_test,
    ])
