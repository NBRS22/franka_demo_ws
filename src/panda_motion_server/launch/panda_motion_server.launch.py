from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # moveit_resources_panda_moveit_config is a standard ROS2 package
    # (installed with MoveIt, no need for panda_sim_ws for this part).
    moveit_config = (
        MoveItConfigsBuilder(
            "panda",
            package_name="moveit_resources_panda_moveit_config"
        )
        .robot_description()
        .robot_description_semantic()
        .robot_description_kinematics()
        .joint_limits()
        # The default controllers file only declares panda_arm_controller.
        # gripper_moveit_controllers.yaml adds panda_hand_controller
        # (GripperCommand action on panda_hand_controller/gripper_cmd).
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # use_sim_time=True partout ci-dessous car Isaac Sim horodate ses
    # messages (/joint_states, /tf) avec une horloge de simulation interne
    # (petits nombres type "415.95", pas un epoch Unix). Ça ne fonctionne
    # QUE si Isaac Sim publie aussi /clock (nœud OmniGraph "ROS2 Publish
    # Clock" dans le ROS2 bridge) — sinon l'horloge ROS de ces nodes reste
    # gelée à t=0 et tout ce qui dépend du temps (timeouts TF, fraîcheur du
    # current_state_monitor) bloque indéfiniment. Vérifier avec
    # `ros2 topic hz /clock` avant de lancer.
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"planning_pipelines": ["ompl"]},
            {"use_sim_time": True},
        ],
    )

    # world est le planning frame de MoveIt (virtual_joint du SRDF,
    # parent_frame="world"), mais rien ne le publiait jusqu'ici. Isaac Sim
    # publie panda_link0 -> Camera_OmniVision_OV9782_Color (confirmé même
    # origine 0,0,0 pour world et panda_link0), donc panda_link0 ne peut
    # avoir qu'un seul parent TF : world en identité ici.
    world_to_robot_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_panda_link0",
        arguments=["0", "0", "0", "0", "0", "0", "1", "world", "panda_link0"],
        parameters=[{"use_sim_time": True}],
    )

    # Bridge to Isaac Sim (relays the planned trajectory on
    # /joint_command).
    bridge_node = Node(
        package="panda_motion_server",
        executable="joint_trajectory_bridge",
        name="joint_trajectory_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # panda_motion_server_node fetches robot_description/*_semantic directly
    # from /move_group (cf. comment in panda_motion_server_node.cpp): no need
    # to pass them here.
    motion_server_node = Node(
        package="panda_motion_server",
        executable="panda_motion_server_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        move_group_node,
        world_to_robot_tf,
        bridge_node,
        motion_server_node,
    ])
