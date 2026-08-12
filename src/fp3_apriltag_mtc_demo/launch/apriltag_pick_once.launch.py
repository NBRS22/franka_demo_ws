from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# Same shape as fp3_apriltag_demo's launch file: brings up the full arm
# stack (fp3_moveit_server/bringup.launch.py) plus one demo node. Does NOT
# launch the camera driver, the AprilTag detector, or handeye_tf_publisher --
# those are assumed to already be running. pick_place_config is overridden
# to this package's own config/pick_place.yaml (grasp/open/place parameters
# tuned for this demo) -- gripper open/close and the place-at-ready step are
# entirely owned by pick_place_node now, this launch file only needs to pass
# use_fake_hardware through to it (via bringup.launch.py), not to the demo
# node itself.
def generate_launch_description():
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    robot_ip = LaunchConfiguration('robot_ip')
    use_rviz = LaunchConfiguration('use_rviz')
    tag_size = LaunchConfiguration('tag_size')
    target_tag_id = LaunchConfiguration('target_tag_id')
    object_id = LaunchConfiguration('object_id')

    pick_place_config = PathJoinSubstitution(
        [FindPackageShare('fp3_apriltag_mtc_demo'), 'config', 'pick_place.yaml'])

    moveit_server_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('fp3_moveit_server'), 'launch', 'bringup.launch.py'])),
        launch_arguments={
            'use_fake_hardware': use_fake_hardware,
            'robot_ip': robot_ip,
            'use_rviz': use_rviz,
            'pick_place_config': pick_place_config,
        }.items(),
    )

    apriltag_pick_once_node = Node(
        package='fp3_apriltag_mtc_demo',
        executable='apriltag_pick_once_node',
        output='screen',
        parameters=[{
            # ParameterValue(..., value_type=...) required: a bare
            # LaunchConfiguration resolves to a string at launch time, which
            # would mismatch the int/float types declare_parameter() expects
            # in apriltag_pick_once_node.py.
            'tag_size': ParameterValue(tag_size, value_type=float),
            'target_tag_id': ParameterValue(target_tag_id, value_type=int),
            'object_id': object_id,
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
            description='AprilTag physical size: outer black square side, in meters'),
        DeclareLaunchArgument(
            'target_tag_id', default_value='0',
            description='Which AprilTag id to wait for on /detections'),
        DeclareLaunchArgument(
            'object_id', default_value='apriltag_object',
            description='Planning-scene id used for the attached collision object'),
        moveit_server_bringup,
        apriltag_pick_once_node,
    ])
