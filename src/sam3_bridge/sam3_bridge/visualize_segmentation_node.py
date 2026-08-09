import numpy as np
import rclpy
import cv2

from franka_demo_interfaces.srv import VisualizeSegmentation
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.node import Node


class VisualizeSegmentationNode(Node):
    def __init__(self):
        super().__init__('visualize_segmentation_node')

        # CV Bridge
        self.bridge = CvBridge()

        # ROS Publisher Topics
        self.viz_pub = self.create_publisher(Image, '/pick/segmentation_visualization', 10)

        # ROS Service Servers
        self.create_service(VisualizeSegmentation,'visualize_segmentation',self.handle_visualize)

        self.get_logger().info("Visualize Segmentation Node started")

    def handle_visualize(self, request, response):
        try:
            rgb = self.bridge.imgmsg_to_cv2(request.rgb, desired_encoding='bgr8')
            mask_raw = self.bridge.imgmsg_to_cv2(request.mask, desired_encoding='mono8')
            mask = mask_raw > 0

            out = rgb.copy()

            gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
            gray_bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            out[~mask] = (gray_bgr[~mask] * 0.5).astype(np.uint8)

            px = int(round(request.point_x))
            py = int(round(request.point_y))
            arm = 5
            cv2.line(out, (px - arm, py - arm), (px + arm, py + arm), (0, 0, 220), 2)
            cv2.line(out, (px + arm, py - arm), (px - arm, py + arm), (0, 0, 220), 2)

            msg = self.bridge.cv2_to_imgmsg(out, encoding='bgr8')
            msg.header = request.rgb.header
            self.viz_pub.publish(msg)

            self.get_logger().info(f"Segmentation visualization published")
            response.success = True
            response.message = ""

        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            response.success = False
            response.message = str(e)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = VisualizeSegmentationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
