import rclpy
import cv2
import zmq

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraBridgeNode(Node):
    def __init__(self):
        super().__init__('camera_bridge')

        # params
        self.declare_parameter('camera_host', '0.0.0.0')
        self.declare_parameter('camera_port', 5555)
        self.declare_parameter('jpeg_quality', 80)

        self.host = self.get_parameter('camera_host').value
        self.port = self.get_parameter('camera_port').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value

        # ZMQ PUB socket
        self.zmq_context = zmq.Context()
        self.socket = self.zmq_context.socket(zmq.PUB)
        self.socket.bind(f"tcp://{self.host}:{self.port}")
        self.get_logger().info(f"ZMQ PUB bound on {self.host}:{self.port}")

        # CV bridge
        self.bridge = CvBridge()

        # RGB subscriber
        self.sub_rgb = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.rgb_callback,
            10
        )

        self.get_logger().info("Camera ZMQ Bridge started")

    def rgb_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            _, jpeg = cv2.imencode(
                '.jpg',
                cv_image,
                [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
            )
            self.socket.send_multipart([b"rgb", jpeg.tobytes()])
        except Exception as e:
            self.get_logger().error(f"RGB callback error: {e}")

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