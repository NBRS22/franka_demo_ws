import numpy as np
import rclpy

from franka_demo_interfaces.srv import FilterPointcloud
from sensor_msgs_py import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge
from rclpy.node import Node


class FilterPointcloudNode(Node):
    def __init__(self):
        super().__init__('filter_pointcloud_node')

        # CV Bridge
        self.bridge = CvBridge()

        # ROS Publisher Topics
        self.cloud_pub = self.create_publisher(PointCloud2, '/pick/pointcloud', 10)

        # ROS Service Servers
        self.create_service(FilterPointcloud, 'filter_pointcloud', self.handle_filter_pointcloud)

        self.get_logger().info("Filter Pointcloud Node started")

    def handle_filter_pointcloud(self, request, response):
        try:
            cloud_msg = request.cloud
            height, width = cloud_msg.height, cloud_msg.width

            mask_raw = self.bridge.imgmsg_to_cv2(request.mask, desired_encoding='mono8')
            if mask_raw.shape != (height, width):
                response.success = False
                response.message = (
                    f"mask shape {mask_raw.shape} does not match camera pointcloud "
                    f"shape ({height}, {width}) — RealSense pointcloud must be organized "
                    "and aligned to the color frame (align_depth + pointcloud both enabled)"
                )
                return response

            organized = pc2.read_points(
                cloud_msg, field_names=('x', 'y', 'z'), skip_nans=False,
                reshape_organized_cloud=True,
            )
            xyz = np.stack(
                [organized['x'], organized['y'], organized['z']], axis=-1
            )

            points = xyz[mask_raw > 0]
            points = points[np.isfinite(points).all(axis=1)]

            if points.shape[0] == 0:
                response.success = False
                response.message = "no valid points in mask"
                return response

            out_msg = pc2.create_cloud_xyz32(cloud_msg.header, points.astype(np.float32))
            self.cloud_pub.publish(out_msg)

            self.get_logger().info(f"Pointcloud published — {points.shape[0]} points")

            response.cloud = out_msg
            response.success = True
            response.message = ""

        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            response.success = False
            response.message = str(e)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = FilterPointcloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
