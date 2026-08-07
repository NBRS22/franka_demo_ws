from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.1',
        description='FP3 controller IP (ignored if use_fake_hardware:=true)'
    )
    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='true',
        description=(
            'Use simulated hardware. SAFE BY DEFAULT (true). '
            'For the real arm: use_fake_hardware:=false + FCI enabled on the Desk.'
        )
    )
    load_gripper_arg = DeclareLaunchArgument(
        'load_gripper',
        default_value='true',
        description='Load the FP3 gripper model and control'
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('franka_fp3_moveit_config'),
                'launch',
                'moveit.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_ip': LaunchConfiguration('robot_ip'),
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
            'load_gripper': LaunchConfiguration('load_gripper'),
        }.items()
    )

    # fp3_motion_server retrieves robot_description/
    # robot_description_semantic itself at runtime via a ROS2 parameter
    # client on /move_group: no need to provide them here.
    motion_server_node = Node(
        package='fp3_motion_server',
        executable='fp3_motion_server_node',
        output='screen',
    )

    grasp_demo_node = Node(
        package='fp3_grasp_demo',
        executable='hardcoded_grasp_node',
        output='screen'
    )

    # fp3_arm_controller is loaded/activated by the ros2_control spawners after
    # move_group, in parallel with the rest of the launch: without a delay,
    # execute() can be called before the controller is ready (a miss was
    # observed with a ~500ms shortfall). Crude but standard delay for this
    # kind of launch; a cleaner v2 would explicitly wait for the controller
    # to become active.
    delayed_grasp_demo_node = TimerAction(period=5.0, actions=[grasp_demo_node])

    return LaunchDescription([
        robot_ip_arg,
        use_fake_hardware_arg,
        load_gripper_arg,
        moveit_launch,
        motion_server_node,
        delayed_grasp_demo_node,
    ])
