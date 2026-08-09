import rclpy
import time
import zmq
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.node import Node


class CameraBridgeNode(Node):
    def __init__(self):
        super().__init__('camera_bridge')

        # Params
        self.declare_parameter('camera_bridge_host', '0.0.0.0')
        self.declare_parameter('camera_bridge_port', 5555)
        self.declare_parameter('camera_bridge_jpeg_quality', 80)

        self.host = self.get_parameter('camera_bridge_host').value
        self.port = self.get_parameter('camera_bridge_port').value
        self.jpeg_quality = self.get_parameter('camera_bridge_jpeg_quality').value

        self._last_frame_time = None
        
        self._WATCHDOG_INTERVAL_S = 10.0
        self._WATCHDOG_STALE_S = 10.0

        # ZMQ PUB socket
        self.zmq_context = zmq.Context()
        self.socket = self._make_socket()

        # CV Bridge
        self.bridge = CvBridge()

        # ROS Subscription Topics
        self.sub_rgb = self.create_subscription(Image, '/camera/camera/color/image_raw', self.rgb_callback, 10)

        # Watchdog Timer
        self.create_timer(self._WATCHDOG_INTERVAL_S, self._watchdog)

        self.get_logger().info('Camera Bridge started')

    def _make_socket(self):
        sock = self.zmq_context.socket(zmq.PUB)
        sock.setsockopt(zmq.SNDHWM, 1)
        sock.bind(f'tcp://{self.host}:{self.port}')
        self.get_logger().info(f'ZMQ PUB bound on {self.host}:{self.port}')
        return sock

    def rgb_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            _, jpeg = cv2.imencode(
                '.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
            )
            self.socket.send_multipart([b'rgb', jpeg.tobytes()])
            self._last_frame_time = time.monotonic()
        except Exception as e:
            self.get_logger().error(f'RGB callback error: {e}')

    def _watchdog(self):
        if self._last_frame_time is None:
            self.get_logger().warn('No camera frame received yet')
        elif time.monotonic() - self._last_frame_time > self._WATCHDOG_STALE_S:
            elapsed = time.monotonic() - self._last_frame_time
            self.get_logger().warn(f'No camera frame for {elapsed:.1f}s — camera disconnected?')

    def destroy_node(self):
        self.socket.close()
        self.zmq_context.term()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
