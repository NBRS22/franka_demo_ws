import numpy as np
import msgpack
import rclpy
import zmq
import cv2

from franka_demo_interfaces.srv import SegmentObject, VisualizeSegmentation
from cv_bridge import CvBridge
from rclpy.node import Node


class Sam3BridgeNode(Node):
    def __init__(self):
        super().__init__('sam3_bridge')

        # Params
        self.declare_parameter('sam3_bridge_host', '127.0.0.1')
        self.declare_parameter('sam3_bridge_port', 5557)

        self._host = self.get_parameter('sam3_bridge_host').value
        self._port = self.get_parameter('sam3_bridge_port').value

        self._RECV_TIMEOUT_MS = 30_000
        self._HEALTH_TIMEOUT_MS = 3_000

        # CV Bridge
        self.bridge = CvBridge()

        # ZMQ REQ socket
        self.zmq_context = zmq.Context()
        self.zmq_socket = self._make_socket()

        # Health Check
        self._check_sam3_server()

        # ROS Service Servers
        self.create_service(SegmentObject, 'segment_object', self.handle_segment_object)

        # ROS Service Clients
        self.viz_seg_client = self.create_client(VisualizeSegmentation, 'visualize_segmentation')

        self.get_logger().info('SAM3 Bridge started')

    def _make_socket(self):
        sock = self.zmq_context.socket(zmq.REQ)
        sock.connect(f'tcp://{self._host}:{self._port}')
        self.get_logger().info(f'ZMQ REQ connected to {self._host}:{self._port}')
        return sock

    def _recreate_socket(self):
        self.zmq_socket.close(linger=0)
        self.zmq_socket = self._make_socket()

    def _check_sam3_server(self):
        try:
            self.zmq_socket.setsockopt(zmq.RCVTIMEO, self._HEALTH_TIMEOUT_MS)
            self.zmq_socket.send(msgpack.packb({'action': 'health'}))
            raw = self.zmq_socket.recv()
            result = msgpack.unpackb(raw, raw=False)
            if result.get('status') == 'ok':
                self.get_logger().info('SAM3 server available')
            else:
                self.get_logger().warn(f'SAM3 server unexpected status: {result}')
        except zmq.Again:
            self._recreate_socket()
            self.get_logger().warn('SAM3 server unavailable at startup')
        finally:
            self.zmq_socket.setsockopt(zmq.RCVTIMEO, -1)

    def _image_to_jpeg_bytes(self, image_msg):
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        _, jpeg = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 95])
        return jpeg.tobytes()

    def _call_sam3(self, jpeg_bytes, label, point_x, point_y, use_pointing, threshold):
        request_msg = {
            'image': jpeg_bytes,
            'text': label if use_pointing or label else 'visual',
            'point_x': float(point_x),
            'point_y': float(point_y),
            'threshold': float(threshold) if threshold > 0 else 0.05,
        }

        self.zmq_socket.setsockopt(zmq.RCVTIMEO, self._RECV_TIMEOUT_MS)

        try:
            self.zmq_socket.send(msgpack.packb(request_msg))
            self.get_logger().info('Waiting for SAM3 response...')
            raw_response = self.zmq_socket.recv()
        except zmq.Again:
            self._recreate_socket()
            raise TimeoutError(f'SAM3 did not respond within {self._RECV_TIMEOUT_MS // 1000}s')
        finally:
            self.zmq_socket.setsockopt(zmq.RCVTIMEO, -1)

        return msgpack.unpackb(raw_response, raw=False)

    def _numpy_mask_to_image_msg(self, mask_bytes, mask_shape):
        mask_np = np.frombuffer(mask_bytes, dtype=bool).reshape(mask_shape)
        mask_uint8 = mask_np.astype(np.uint8) * 255
        return self.bridge.cv2_to_imgmsg(mask_uint8, encoding='mono8')

    def _trigger_visualization(self, image_msg, mask_msg, point_x, point_y):
        if not self.viz_seg_client.service_is_ready():
            self.get_logger().error(f'Service {self.viz_seg_client.srv_name} not available')
            return None

        req = VisualizeSegmentation.Request()
        req.rgb = image_msg
        req.mask = mask_msg
        req.point_x = point_x
        req.point_y = point_y

        future = self.viz_seg_client.call_async(req)
        future.add_done_callback(self._on_visualization_done)

    def _on_visualization_done(self, future):
        try:
            result = future.result()
            if result is None or not result.success:
                self.get_logger().warn(f"Segmentation visualization failed : {result.message if result else 'no response'}")
            else:
                self.get_logger().info('Segmentation visualization published')
        except Exception as e:
            self.get_logger().warn(f'Segmentation visualization call failed : {e}')

    def handle_segment_object(self, request, response):
        self.get_logger().info(f"Request received — label : '{request.label}' " f"point : ({request.point_x}, {request.point_y})")

        try:
            jpeg_bytes = self._image_to_jpeg_bytes(request.image)
            self.get_logger().info(f'Image encoded : {len(jpeg_bytes)} bytes')

            result = self._call_sam3(
                jpeg_bytes,
                request.label,
                request.point_x,
                request.point_y,
                request.use_pointing,
                request.threshold,
            )

            status = result.get('status')

            if status == 'error':
                response.success = False
                response.message = result.get('error_msg', 'Unknown error')
                response.has_mask = False
                return response

            if not result.get('has_mask', False):
                self.get_logger().warn('No mask found')
                response.success = True
                response.has_mask = False
                response.score = 0.0
                response.message = ''
                return response

            mask_bytes = result['mask']
            mask_shape = result['mask_shape']
            score = result['score']

            self.get_logger().info(f'Mask received : shape = {mask_shape} score = {score:.3f}')

            response.mask = self._numpy_mask_to_image_msg(mask_bytes, mask_shape)
            response.score = float(score)
            response.has_mask = True
            response.success = True
            response.message = ''

            self._trigger_visualization(request.image, response.mask, request.point_x, request.point_y)

        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            response.success = False
            response.has_mask = False
            response.message = str(e)

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
