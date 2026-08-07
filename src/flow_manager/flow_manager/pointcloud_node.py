import rclpy
from rclpy.node import Node
from franka_demo_interfaces.srv import FuseMaskDepth
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np


class PointcloudNode(Node):
    def __init__(self):
        super().__init__('pointcloud_node')

        # Vraie D455 (mode réel) : depth en 16UC1, millimètres entiers.
        # Isaac Sim (mode sim) : depth en 32FC1, mètres déjà en flottant.
        self.declare_parameter('use_sim', False)
        self.use_sim = self.get_parameter('use_sim').value

        self.bridge = CvBridge()

        self.srv = self.create_service(
            FuseMaskDepth,
            'fuse_mask_depth',
            self.handle_fuse_mask_depth
        )

        self.get_logger().info(
            f"Pointcloud Node started (use_sim={self.use_sim})"
        )

    def handle_fuse_mask_depth(self, request, response):
        try:
            # binary mask
            mask_cv = self.bridge.imgmsg_to_cv2(
                request.mask,
                desired_encoding='mono8'
            )
            mask_bool = mask_cv > 0

            # depth
            depth_cv = self.bridge.imgmsg_to_cv2(
                request.depth,
                desired_encoding='passthrough'
            ).astype(np.float32)

            # intrinsics
            K = request.camera_info.k
            fx, fy = K[0], K[4]
            cx, cy = K[2], K[5]

            # deprojection
            rows, cols = np.where(mask_bool)
            if self.use_sim:
                z = depth_cv[rows, cols]  # 32FC1, déjà en mètres (Isaac Sim)
            else:
                z = depth_cv[rows, cols] / 1000.0  # 16UC1, mm → mètres (D455 réelle)

            # filter invalid points
            valid = z > 0
            rows, cols, z = rows[valid], cols[valid], z[valid]

            if len(z) == 0:
                response.success = False
                response.message = "no valid depth points in mask"
                return response

            x = (cols - cx) * z / fx
            y = (rows - cy) * z / fy

            xyz = np.stack([x, y, z], axis=1).astype(np.float32)
            self.get_logger().info(f"Cloud: {xyz.shape[0]} points")

            response.cloud = self._numpy_to_pointcloud2(
                xyz,
                request.depth.header.frame_id
            )
            response.success = True
            response.message = ""

        except Exception as e:
            self.get_logger().error(f"Fusion error: {e}")
            response.success = False
            response.message = str(e)

        return response

    def _numpy_to_pointcloud2(self, xyz, frame_id):
        cloud = PointCloud2()
        cloud.header = Header()
        cloud.header.frame_id = frame_id
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.height = 1
        cloud.width = xyz.shape[0]
        cloud.fields = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,
                       datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.data = xyz.tobytes()
        cloud.is_dense = True
        return cloud


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
