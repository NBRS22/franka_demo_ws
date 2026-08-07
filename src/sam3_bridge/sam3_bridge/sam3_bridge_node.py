import numpy as np
import msgpack
import rclpy
import zmq
import cv2

from franka_demo_interfaces.srv import SegmentObject
from cv_bridge import CvBridge
from rclpy.node import Node


class Sam3BridgeNode(Node):
    def __init__(self):
        super().__init__('sam3_bridge')

        # params
        self.declare_parameter('sam3_host', '127.0.0.1')
        self.declare_parameter('sam3_port', 5557)

        self.host = self.get_parameter('sam3_host').value
        self.port = self.get_parameter('sam3_port').value

        # cv bridge
        self.bridge = CvBridge()

        # ZMQ REQ socket
        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.REQ)
        self.zmq_socket.connect(f"tcp://{self.host}:{self.port}")
        self.get_logger().info(f"ZMQ REQ connected to {self.host}:{self.port}")

        # service ROS
        self.srv = self.create_service(SegmentObject, 'segment_object', self.handle_segment_object)

        self.get_logger().info("SAM3 Bridge started")

    def _image_to_jpeg_bytes(self, image_msg):
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        _, jpeg = cv2.imencode(
            '.jpg',
            cv_image,
            [cv2.IMWRITE_JPEG_QUALITY, 95]
        )
        return jpeg.tobytes()

    def _call_sam3(self, jpeg_bytes, label, point_x, point_y, use_pointing, threshold):
        request_msg = {
            "image": jpeg_bytes,
            "text": label if use_pointing or label else "visual",
            "point_x": float(point_x),
            "point_y": float(point_y),
            "threshold": float(threshold) if threshold > 0 else 0.05
        }
        self.zmq_socket.send(msgpack.packb(request_msg))
        self.get_logger().info("Waiting for SAM3 response...")
        raw_response = self.zmq_socket.recv()
        return msgpack.unpackb(raw_response, raw=False)

    def _numpy_mask_to_image_msg(self, mask_bytes, mask_shape):
        mask_np = np.frombuffer(mask_bytes, dtype=bool).reshape(mask_shape)
        mask_uint8 = mask_np.astype(np.uint8) * 255
        return self.bridge.cv2_to_imgmsg(mask_uint8, encoding='mono8')

    def handle_segment_object(self, request, response):
        self.get_logger().info(
            f"Request received - label: '{request.label}' "
            f"point: ({request.point_x}, {request.point_y})"
        )

        try:
            # convert ROS image → JPEG bytes
            jpeg_bytes = self._image_to_jpeg_bytes(request.image)
            self.get_logger().info(f"Image encoded: {len(jpeg_bytes)} bytes")

            # call SAM3
            result = self._call_sam3(
                jpeg_bytes,
                request.label,
                request.point_x,
                request.point_y,
                request.use_pointing,
                request.threshold
            )

            # process response
            status = result.get("status")

            if status == "error":
                response.success = False
                response.error_msg = result.get("error_msg", "Unknown error")
                response.has_mask = False
                return response

            if not result.get("has_mask", False):
                self.get_logger().warn("No mask found")
                response.success = True
                response.has_mask = False
                response.score = 0.0
                response.error_msg = ""
                return response

            # mask found
            mask_bytes = result["mask"]
            mask_shape = result["mask_shape"]
            score = result["score"]

            self.get_logger().info(
                f"Mask received: shape={mask_shape} score={score:.3f}"
            )

            response.mask = self._numpy_mask_to_image_msg(mask_bytes, mask_shape)
            response.score = float(score)
            response.has_mask = True
            response.success = True
            response.error_msg = ""

        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            response.success = False
            response.has_mask = False
            response.error_msg = str(e)

        return response

    def destroy_node(self):
        self.zmq_socket.close()
        self.zmq_context.term()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Sam3BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()