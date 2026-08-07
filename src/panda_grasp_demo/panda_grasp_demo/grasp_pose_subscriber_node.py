import threading
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from panda_grasp_demo.pick_sequence import PickSequenceNode


class GraspPoseSubscriberNode(PickSequenceNode):
    # Branché sur le pipeline de perception réel (flow_manager -> SAM3 ->
    # GraspGen -> grasp_selector), pas de pose hardcodée. Une seule pose
    # traitée à la fois : toute pose reçue pendant qu'un pick est déjà en
    # cours est ignorée (pas de file d'attente pour cette v1).
    def __init__(self):
        super().__init__('grasp_pose_subscriber_node')
        self._executing = False
        self._lock = threading.Lock()          # ← protège _executing
        self._sub = self.create_subscription(
            PoseStamped,
            '/best_grasp_pose',
            self._on_grasp_pose,
            10
        )
        self.get_logger().info("En attente d'une pose sur /best_grasp_pose...")

    def _on_grasp_pose(self, msg: PoseStamped):
        with self._lock:
            if self._executing:
                self.get_logger().warn(
                    "Pick déjà en cours, nouvelle pose ignorée"
                )
                return
            self._executing = True

        self.get_logger().info(
            f"Pose reçue: ({msg.pose.position.x:.3f}, "
            f"{msg.pose.position.y:.3f}, {msg.pose.position.z:.3f}) "
            f"frame='{msg.header.frame_id}'"
        )

        # Lance le pick dans un thread séparé → libère immédiatement l'executor
        thread = threading.Thread(
            target=self._pick_thread,
            args=(msg,),
            daemon=True
        )
        thread.start()

    def _pick_thread(self, msg: PoseStamped):
        """Exécuté dans un thread worker — bloque sans gêner l'executor."""
        try:
            self.pick(msg)
        except Exception as e:
            self.get_logger().error(f"Erreur pendant le pick : {e}")
        finally:
            with self._lock:
                self._executing = False
            self.get_logger().info("En attente d'une pose sur /best_grasp_pose...")


def main(args=None):
    rclpy.init(args=args)
    node = GraspPoseSubscriberNode()
    # MultiThreadedExecutor obligatoire : pick_sequence utilise un
    # ReentrantCallbackGroup pour que les réponses des actions (gripper,
    # move_to_pose, go_home) soient traitées pendant que le thread
    # worker attend sur threading.Event.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()