import msgpack
import msgpack_numpy as m
import numpy as np
import rclpy
import zmq
from franka_demo_interfaces.srv import GenerateGraspPose
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs_py import point_cloud2 as pc2

m.patch()

# GraspGen inference can take tens of seconds for large clouds
_RECV_TIMEOUT_MS = 60_000
_HEALTH_TIMEOUT_MS = 3_000


class GraspGenBridgeNode(Node):
    def __init__(self):
        super().__init__('graspgen_bridge')

        self.declare_parameter('graspgen_bridge_host', '172.22.62.94')
        self.declare_parameter('graspgen_bridge_port', 5556)
        self.declare_parameter('num_grasps', 200)
        self.declare_parameter('topk_num_grasps', 10)

        self._host = self.get_parameter('graspgen_bridge_host').value
        self._port = self.get_parameter('graspgen_bridge_port').value
        self.num_grasps = self.get_parameter('num_grasps').value
        self.topk_num_grasps = self.get_parameter('topk_num_grasps').value

        self.zmq_context = zmq.Context()
        self.zmq_socket = self._make_socket()

        self._check_graspgen_server()

        self.srv = self.create_service(
            GenerateGraspPose,
            'generate_grasp_pose',
            self.handle_generate_grasp_pose,
        )

        self.get_logger().info('GraspGen Bridge started')

    # -----------------------------------------------------------------------

    def _make_socket(self):
        """Create and connect a fresh ZMQ REQ socket."""
        sock = self.zmq_context.socket(zmq.REQ)
        sock.connect(f'tcp://{self._host}:{self._port}')
        self.get_logger().info(f'ZMQ REQ connected to {self._host}:{self._port}')
        return sock

    def _recreate_socket(self):
        """Reset ZMQ REQ state machine after a timeout (EFSM-safe).

        After zmq.Again the socket is stuck mid-transaction; close with
        linger=0 to discard pending state and open a fresh connection.
        """
        self.zmq_socket.close(linger=0)
        self.zmq_socket = self._make_socket()

    # -----------------------------------------------------------------------

    def _check_graspgen_server(self):
        """Non-blocking startup health-check; logs availability but never blocks."""
        try:
            self.zmq_socket.setsockopt(zmq.RCVTIMEO, _HEALTH_TIMEOUT_MS)
            self.zmq_socket.send(msgpack.packb({'action': 'health'}))
            raw = self.zmq_socket.recv()
            result = msgpack.unpackb(raw)
            if result.get('status') == 'ok':
                self.get_logger().info('GraspGen server available')
            else:
                self.get_logger().warn(f'GraspGen server unexpected status: {result}')
        except zmq.Again:
            # Socket is now in EFSM state — must recreate before any next send
            self._recreate_socket()
            self.get_logger().warn('GraspGen server unavailable at startup')
        finally:
            self.zmq_socket.setsockopt(zmq.RCVTIMEO, -1)

    def _pointcloud2_to_numpy(self, cloud_msg):
        points = pc2.read_points(cloud_msg, field_names=('x', 'y', 'z'), skip_nans=True)
        return np.stack([points['x'], points['y'], points['z']], axis=-1).astype(np.float32)

    def _call_graspgen(self, xyz):
        request_msg = {
            'action': 'infer',
            'point_cloud': xyz,
            'num_grasps': self.num_grasps,
            'topk_num_grasps': self.topk_num_grasps,
        }

        self.zmq_socket.setsockopt(zmq.RCVTIMEO, _RECV_TIMEOUT_MS)
        try:
            self.zmq_socket.send(msgpack.packb(request_msg))
            self.get_logger().info('Waiting for GraspGen response...')
            raw_response = self.zmq_socket.recv()
        except zmq.Again:
            self._recreate_socket()
            raise TimeoutError(
                f'GraspGen did not respond within {_RECV_TIMEOUT_MS // 1000}s'
            )
        finally:
            self.zmq_socket.setsockopt(zmq.RCVTIMEO, -1)

        return msgpack.unpackb(raw_response)

    def _matrix_to_pose_array(self, grasps, frame_id):
        pose_array = PoseArray()
        pose_array.header.frame_id = frame_id
        pose_array.header.stamp = self.get_clock().now().to_msg()
        for grasp_matrix in grasps:
            pose = Pose()
            pose.position.x = float(grasp_matrix[0, 3])
            pose.position.y = float(grasp_matrix[1, 3])
            pose.position.z = float(grasp_matrix[2, 3])
            q = Rotation.from_matrix(grasp_matrix[:3, :3]).as_quat()
            pose.orientation.x = float(q[0])
            pose.orientation.y = float(q[1])
            pose.orientation.z = float(q[2])
            pose.orientation.w = float(q[3])
            pose_array.poses.append(pose)
        return pose_array

    # -----------------------------------------------------------------------

    def handle_generate_grasp_pose(self, request, response):
        self.get_logger().info('Grasp generation requested')
        try:
            xyz = self._pointcloud2_to_numpy(request.object_cloud)
            self.get_logger().info(f'Point cloud: {xyz.shape[0]} points')

            if xyz.shape[0] == 0:
                response.success = False
                response.error_msg = 'empty point cloud'
                return response

            result = self._call_graspgen(xyz)
            grasps = result.get('grasps')
            scores = result.get('confidences')

            if grasps is None or scores is None:
                response.success = False
                response.error_msg = f'unexpected server response keys: {list(result.keys())}'
                return response

            self.get_logger().info(f'{len(scores)} grasps received')

            response.grasps = self._matrix_to_pose_array(
                grasps, request.object_cloud.header.frame_id
            )
            response.scores = [float(s) for s in scores]
            response.success = True
            response.error_msg = ''

        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            response.success = False
            response.error_msg = str(e)

        return response

    def destroy_node(self):
        self.zmq_socket.close()
        self.zmq_context.term()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GraspGenBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
