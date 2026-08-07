import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped

from panda_grasp_demo.pick_sequence import PickSequenceNode


def _hardcoded_grasp_pose(node: PickSequenceNode) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'panda_link0'
    pose.header.stamp = node.get_clock().now().to_msg()

    # TO ADJUST: placeholder grasp pose, not coming from GraspGen (test
    # isolé de la séquence, indépendant du pipeline de perception).
    pose.pose.position.x = 0.4
    pose.pose.position.y = 0.0
    pose.pose.position.z = 0.4
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 1.0
    return pose


def main(args=None):
    rclpy.init(args=args)
    node = PickSequenceNode('hardcoded_grasp_node')

    # pick() bloque son thread en attendant chaque résultat d'action (cf.
    # pick_sequence.py) — il faut que le node spinne déjà sur un thread
    # séparé, sinon rien ne traite jamais les réponses des actions.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    node.pick(_hardcoded_grasp_pose(node))

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
