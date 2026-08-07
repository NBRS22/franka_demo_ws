import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from franka_demo_interfaces.srv import GetFrames


class CameraBufferNode(Node):
    def __init__(self):
        super().__init__('camera_buffer')

        self.last_rgb = None
        self.last_depth = None
        self.last_camera_info = None

        self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self._rgb_callback,
            10
        )
        self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self._depth_callback,
            10
        )
        self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self._camera_info_callback,
            10
        )

        self.srv = self.create_service(
            GetFrames,
            'get_frames',
            self.handle_get_frames
        )

        self.get_logger().info("Camera Buffer Node started")

    def _rgb_callback(self, msg):
        self.last_rgb = msg

    def _depth_callback(self, msg):
        self.last_depth = msg

    def _camera_info_callback(self, msg):
        self.last_camera_info = msg

    def handle_get_frames(self, request, response):
        if self.last_rgb is None:
            response.success = False
            response.message = "no RGB frame available"
            return response

        if self.last_depth is None:
            response.success = False
            response.message = "no depth frame available"
            return response

        if self.last_camera_info is None:
            response.success = False
            response.message = "no camera_info available"
            return response

        response.rgb = self.last_rgb
        response.depth = self.last_depth
        response.camera_info = self.last_camera_info
        response.success = True
        response.message = ""
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