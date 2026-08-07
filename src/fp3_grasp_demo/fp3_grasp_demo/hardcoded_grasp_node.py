import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from franka_demo_interfaces.action import MoveToPose


class HardcodedGraspNode(Node):
    def __init__(self):
        super().__init__('hardcoded_grasp_node')
        self._client = ActionClient(self, MoveToPose, 'move_to_pose')

    def send_goal(self):
        pose = PoseStamped()
        pose.header.frame_id = 'fp3_link0'
        pose.header.stamp = self.get_clock().now().to_msg()

        # TO ADJUST: placeholder pose for this v1 test. No hand-eye
        # calibration nor graspgen_bridge wired up here yet (cf. v2). Neutral
        # orientation (identity quaternion): to be replaced with a real grasp
        # orientation before testing on a real pose.
        pose.pose.position.x = 0.4
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.4
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        self.get_logger().info("Waiting for the move_to_pose action server...")
        self._client.wait_for_server()

        goal = MoveToPose.Goal()
        goal.target_pose = pose

        self.get_logger().info(
            f"Sending goal: pose=({pose.pose.position.x}, "
            f"{pose.pose.position.y}, {pose.pose.position.z}) "
            f"frame='{pose.header.frame_id}'"
        )
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by fp3_motion_server")
            return

        self.get_logger().info("Goal accepted, executing...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result = future.result().result
        status = "success" if result.success else "failure"
        self.get_logger().info(f"Result: {status} - {result.message}")


def main(args=None):
    rclpy.init(args=args)
    node = HardcodedGraspNode()
    node.send_goal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
