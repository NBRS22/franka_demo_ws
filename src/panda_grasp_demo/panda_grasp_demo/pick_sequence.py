import threading

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from franka_demo_interfaces.action import MoveToPose, GoHome
from control_msgs.action import GripperCommand
from scipy.spatial.transform import Rotation

PRE_GRASP_OFFSET = 0.10  # meters, retreat along the grasp's local Z axis — TO ADJUST
GRIPPER_OPEN_POSITION = 0.035  # panda_finger_joint1/2 open (cf. panda.srdf "open" state)
GRIPPER_CLOSE_POSITION = 0.0  # panda_finger_joint1/2 closed — TO ADJUST
GRIPPER_MAX_EFFORT = 20.0
ACTION_TIMEOUT = 90.0  # secondes, large marge pour un pick complet (plan+exec)


def offset_pose_along_local_z(pose: PoseStamped, distance: float) -> PoseStamped:
    """Pose offset by `distance` (m) along the local Z axis of `pose`
    (negative distance = retreat, opposite the approach axis). Fonctionne
    quel que soit le frame de `pose` (offset purement local, aucune
    transformation TF requise ici)."""
    q = pose.pose.orientation
    rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
    world_offset = rotation.apply([0.0, 0.0, distance])

    offset_pose = PoseStamped()
    offset_pose.header = pose.header
    offset_pose.pose.orientation = pose.pose.orientation
    offset_pose.pose.position.x = pose.pose.position.x + world_offset[0]
    offset_pose.pose.position.y = pose.pose.position.y + world_offset[1]
    offset_pose.pose.position.z = pose.pose.position.z + world_offset[2]
    return offset_pose


class PickSequenceNode(Node):
    # Séquence commune : gripper open -> pre-grasp -> grasp -> gripper close
    # -> home. Deux plans MoveIt distincts pour pre-grasp/grasp, pas de
    # trajectoire cartésienne pour cette v1. Home = named joint target
    # 'ready' via panda_motion_server (pas une pose cartésienne bricolée).
    #
    # Pas de délai artificiel entre les mouvements : joint_trajectory_bridge
    # attend la convergence réelle de /joint_states (à une tolérance près)
    # avant de répondre succès, donc le but suivant n'est envoyé qu'une fois
    # le robot effectivement arrivé.
    #
    # La pose de grasp (frame_id quelconque, ex: Camera_OmniVision_OV9782_Color)
    # est transmise telle quelle à panda_motion_server : MoveGroupInterface
    # la résout vers le planning frame via TF2, pas de transform manuel ici.
    def __init__(self, node_name: str):
        super().__init__(node_name)
        # pick() peut être appelé depuis un callback déjà en cours de
        # dispatch (ex: grasp_pose_subscriber_node, à la réception d'une
        # pose sur /best_grasp_pose) — il bloque son thread en attendant
        # chaque résultat d'action (cf. _send_goal/_wait_result), donc les
        # clients d'action doivent être sur un callback group DIFFÉRENT,
        # sinon leurs réponses ne peuvent jamais être traitées pendant
        # l'attente (deadlock même avec un MultiThreadedExecutor). Même
        # cause, même fix que "Nested spinning" dans flow_manager_node —
        # cf. dette technique dans CLAUDE.md.
        action_cb_group = ReentrantCallbackGroup()
        self._move_client = ActionClient(
            self, MoveToPose, 'move_to_pose', callback_group=action_cb_group
        )
        self._go_home_client = ActionClient(
            self, GoHome, 'go_home', callback_group=action_cb_group
        )
        self._gripper_client = ActionClient(
            self, GripperCommand, 'panda_hand_controller/gripper_cmd',
            callback_group=action_cb_group
        )

    def _send_goal(self, client, goal, label):
        """Envoie un but, bloque CE thread jusqu'à acceptation/rejet (pas le
        résultat final) — pas de spin imbriqué, cf. commentaire __init__."""
        client.wait_for_server()

        event = threading.Event()
        holder = {}

        def _on_response(future):
            holder['goal_handle'] = future.result()
            event.set()

        client.send_goal_async(goal).add_done_callback(_on_response)

        if not event.wait(timeout=ACTION_TIMEOUT):
            self.get_logger().error(f"[{label}] Timeout en attendant l'acceptation du but")
            return None
        return holder['goal_handle']

    def _wait_result(self, goal_handle, label):
        """Bloque CE thread jusqu'au résultat final d'un but déjà accepté."""
        event = threading.Event()
        holder = {}

        def _on_result(future):
            holder['result'] = future.result().result
            event.set()

        goal_handle.get_result_async().add_done_callback(_on_result)

        if not event.wait(timeout=ACTION_TIMEOUT):
            self.get_logger().error(f"[{label}] Timeout en attendant le résultat")
            return None
        return holder['result']

    def _move_to(self, pose: PoseStamped, label: str) -> bool:
        self.get_logger().info(f"[{label}] Sending move_to_pose goal...")
        goal = MoveToPose.Goal()
        goal.target_pose = pose

        goal_handle = self._send_goal(self._move_client, goal, label)
        if goal_handle is None:
            return False
        if not goal_handle.accepted:
            self.get_logger().error(f"[{label}] Goal rejected by panda_motion_server")
            return False

        result = self._wait_result(goal_handle, label)
        if result is None:
            return False

        if result.success:
            self.get_logger().info(f"[{label}] OK - {result.message}")
        else:
            self.get_logger().error(f"[{label}] FAILED - {result.message}")
        return result.success

    def _go_home(self) -> bool:
        self.get_logger().info("[home] Sending go_home goal...")
        goal_handle = self._send_goal(self._go_home_client, GoHome.Goal(), "home")
        if goal_handle is None:
            return False
        if not goal_handle.accepted:
            self.get_logger().error("[home] Goal rejected by panda_motion_server")
            return False

        result = self._wait_result(goal_handle, "home")
        if result is None:
            return False

        if result.success:
            self.get_logger().info(f"[home] OK - {result.message}")
        else:
            self.get_logger().error(f"[home] FAILED - {result.message}")
        return result.success

    def _set_gripper(self, position: float, label: str) -> bool:
        self.get_logger().info(f"[{label}] Sending gripper command (position={position})...")
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = GRIPPER_MAX_EFFORT

        goal_handle = self._send_goal(self._gripper_client, goal, label)
        if goal_handle is None:
            return False
        if not goal_handle.accepted:
            self.get_logger().error(f"[{label}] Gripper command rejected")
            return False

        result = self._wait_result(goal_handle, label)
        if result is None:
            return False

        self.get_logger().info(
            f"[{label}] Gripper: position={result.position:.4f} "
            f"reached_goal={result.reached_goal}"
        )
        return True

    def pick(self, grasp_pose: PoseStamped) -> bool:
        pre_grasp_pose = offset_pose_along_local_z(grasp_pose, -PRE_GRASP_OFFSET)

        # Ouverture avant l'approche : sans ça la pince ne peut rien
        # enclore au moment du grasp. Sert aussi à vérifier visuellement
        # qu'une transition ouvert->fermé a bien lieu (sinon impossible de
        # distinguer "commande ignorée" de "déjà fermé par défaut").
        if not self._set_gripper(GRIPPER_OPEN_POSITION, "gripper open"):
            return False
        if not self._move_to(pre_grasp_pose, "pre-grasp"):
            return False
        if not self._move_to(grasp_pose, "grasp"):
            return False
        if not self._set_gripper(GRIPPER_CLOSE_POSITION, "gripper close"):
            return False
        return self._go_home()
