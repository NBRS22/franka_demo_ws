import numpy as np
import rclpy
from cv_bridge import CvBridge
from franka_demo_interfaces.srv import CreatePointcloud
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


class PointcloudNode(Node):
    def __init__(self):
        super().__init__('pointcloud_node')

        self.bridge = CvBridge()

        self.cloud_pub = self.create_publisher(PointCloud2, '/pick/pointcloud', 10)

        self.create_service(
            CreatePointcloud,
            'create_pointcloud',
            self.handle_create_pointcloud,
        )

        self.get_logger().info("Pointcloud Node started")

    def handle_create_pointcloud(self, request, response):
        try:
            depth_raw = self.bridge.imgmsg_to_cv2(
                request.depth, desired_encoding='passthrough'
            )
            mask_raw = self.bridge.imgmsg_to_cv2(request.mask, desired_encoding='mono8')
            mask = mask_raw > 0

            # convert depth to metres
            if depth_raw.dtype == np.uint16:
                depth_m = depth_raw.astype(np.float32) / 1000.0
            else:
                depth_m = depth_raw.astype(np.float32)

            K = request.camera_info.k  # [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            fx, fy = K[0], K[4]
            cx, cy = K[2], K[5]

            rows, cols = np.where(mask)
            z = depth_m[rows, cols]

            valid = z > 0.0
            rows, cols, z = rows[valid], cols[valid], z[valid]

            if z.size == 0:
                response.success = False
                response.message = "no valid depth points in mask"
                return response

            x = (cols - cx) * z / fx
            y = (rows - cy) * z / fy

            points = np.stack([x, y, z], axis=-1).astype(np.float32)

            cloud_msg = pc2.create_cloud_xyz32(request.depth.header, points)
            self.cloud_pub.publish(cloud_msg)

            self.get_logger().info(f"Pointcloud published — {len(points)} points")

            response.cloud = cloud_msg
            response.success = True
            response.message = ""

        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            response.success = False
            response.message = str(e)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = PointcloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
