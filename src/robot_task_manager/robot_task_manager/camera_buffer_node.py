import rclpy

from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from franka_demo_interfaces.srv import GetFrames
from rclpy.node import Node

class CameraBufferNode(Node):
    def __init__(self):
        super().__init__('camera_buffer')

        # Params
        self._last_rgb = None
        self._last_depth = None
        self._last_camera_info = None
        self._last_cloud = None

        # ROS Subscription Topics
        self.create_subscription(Image, '/camera/camera/color/image_raw', self._rgb_callback, 10)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self._depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self._camera_info_callback, 10)
        # Published continuously by pointcloud_publisher_node (full,
        # unmasked, organized deprojection of aligned_depth_to_color -- not
        # the native RealSense cloud, cf. CLAUDE.md racine "Pourquoi plus de
        # nuage natif RealSense"). Buffered the same way as rgb/depth/
        # camera_info so a single get_frames call returns all four from the
        # same camera instant -- the mask SAM3 computes from this same rgb
        # stays spatially consistent with this same cloud, even after SAM3's
        # round-trip latency, instead of picking up whatever the topic's
        # latest sample happens to be by the time create_pointcloud is
        # actually called.
        self.create_subscription(PointCloud2, '/pick/raw_pointcloud', self._cloud_callback, 10)

        # ROS Service Servers
        self.create_service(GetFrames, 'get_frames', self.handle_get_frames)

        self.get_logger().info('Camera Buffer Node started')

    def _rgb_callback(self, msg):
        self._last_rgb = msg

    def _depth_callback(self, msg):
        self._last_depth = msg

    def _camera_info_callback(self, msg):
        self._last_camera_info = msg

    def _cloud_callback(self, msg):
        self._last_cloud = msg

    def handle_get_frames(self, request, response):
        if self._last_rgb is None:
            response.success = False
            response.message = 'no RGB frame available'
            return response

        if self._last_depth is None:
            response.success = False
            response.message = 'no depth frame available'
            return response

        if self._last_camera_info is None:
            response.success = False
            response.message = 'no camera_info available'
            return response

        if self._last_cloud is None:
            response.success = False
            response.message = 'no pointcloud available (pointcloud_publisher_node not running yet?)'
            return response

        response.rgb = self._last_rgb
        response.depth = self._last_depth
        response.camera_info = self._last_camera_info
        response.cloud = self._last_cloud
        response.success = True
        response.message = ''
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CameraBufferNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
