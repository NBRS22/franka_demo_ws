import rclpy
from rclpy.node import Node
from franka_demo_interfaces.srv import SelectBestGrasp
from geometry_msgs.msg import PoseStamped, PoseArray
import numpy as np


class GraspSelectorNode(Node):
    def __init__(self):
        super().__init__('grasp_selector')

        # RViz publishers
        self.grasp_poses_pub = self.create_publisher(
            PoseArray,
            '/grasp_poses',
            10
        )
        self.best_grasp_pub = self.create_publisher(
            PoseStamped,
            '/best_grasp_pose',
            10
        )

        self.srv = self.create_service(
            SelectBestGrasp,
            'select_best_grasp',
            self.handle_select_best_grasp
        )

        self.get_logger().info("Grasp Selector Node started")

    def handle_select_best_grasp(self, request, response):
        try:
            if len(request.grasps.poses) == 0:
                response.success = False
                response.message = "empty grasps"
                return response

            # best pose
            best_idx = int(np.argmax(request.scores))
            best_pose = request.grasps.poses[best_idx]
            best_score = float(request.scores[best_idx])

            stamp = self.get_clock().now().to_msg()
            frame_id = request.frame_id

            # publish all poses
            pose_array = request.grasps
            pose_array.header.frame_id = frame_id
            pose_array.header.stamp = stamp
            self.grasp_poses_pub.publish(pose_array)

            # publish best pose
            best_pose_stamped = PoseStamped()
            best_pose_stamped.header.frame_id = frame_id
            best_pose_stamped.header.stamp = stamp
            best_pose_stamped.pose = best_pose
            self.best_grasp_pub.publish(best_pose_stamped)

            self.get_logger().info(
                f"Best pose idx={best_idx} score={best_score:.3f}"
            )

            response.best_pose = best_pose_stamped
            response.best_idx = best_idx
            response.best_score = best_score
            response.success = True
            response.message = ""

        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            response.success = False
            response.message = str(e)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = GraspSelectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()