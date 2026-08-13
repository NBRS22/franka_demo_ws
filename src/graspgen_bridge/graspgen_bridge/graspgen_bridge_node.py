import msgpack_numpy as m
import numpy as np
import msgpack
import rclpy
import zmq

from franka_demo_interfaces.srv import GenerateGraspPose, VisualizeGrasps
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs_py import point_cloud2 as pc2


class GraspGenBridgeNode(Node):
    def __init__(self):
        super().__init__('graspgen_bridge')

        m.patch()

        # Params
        self.declare_parameter('graspgen_bridge_host', '127.0.0.1')
        self.declare_parameter('graspgen_bridge_port', 5558)
        self.declare_parameter('num_grasps', 200)
        self.declare_parameter('topk_num_grasps', 10)
        self.declare_parameter('collision_threshold', 0.01)
        self.declare_parameter('max_scene_points', 8192)
        # GraspGen's raw grasp matrices are expressed at the gripper's base
        # link (panda_hand/fp3_hand), not the TCP (fingertip) frame -- cf.
        # GraspGen's docs/GRIPPER_DESCRIPTION.md ("depth" = extent from base
        # link to TCP along local +Z, approach axis). Franka's own
        # fp3_hand -> fp3_hand_tcp offset (franka_description,
        # end_effectors/franka_hand/franka_hand_arguments.xacro, tcp_xyz
        # default "0 0 0.1034") is used here rather than GraspGen's own
        # depth (0.10527314 for franka_panda.yaml, a mesh-based approximation)
        # since it's the exact value this robot's URDF actually uses.
        self.declare_parameter('hand_to_tcp_offset_z', 0.1034)

        self._host = self.get_parameter('graspgen_bridge_host').value
        self._port = self.get_parameter('graspgen_bridge_port').value
        self.num_grasps = self.get_parameter('num_grasps').value
        self.topk_num_grasps = self.get_parameter('topk_num_grasps').value
        self.collision_threshold = self.get_parameter('collision_threshold').value
        self.max_scene_points = self.get_parameter('max_scene_points').value
        self.hand_to_tcp_offset_z = self.get_parameter('hand_to_tcp_offset_z').value

        self._RECV_TIMEOUT_MS = 60_000
        self._HEALTH_TIMEOUT_MS = 3_000

        # ZMQ REQ Socket
        self.zmq_context = zmq.Context()
        self.zmq_socket = self._make_socket()

        # Health Check
        self._check_graspgen_server()

        # ROS Service Servers
        self.srv = self.create_service(GenerateGraspPose, 'generate_grasp_pose', self.handle_generate_grasp_pose)

        # ROS Service Clients
        self.viz_grasps_client = self.create_client(VisualizeGrasps, 'visualize_grasps')

        self.get_logger().info('GraspGen Bridge started')

    def _make_socket(self):
        sock = self.zmq_context.socket(zmq.REQ)
        sock.connect(f'tcp://{self._host}:{self._port}')
        self.get_logger().info(f'ZMQ REQ connected to {self._host}:{self._port}')
        return sock

    def _recreate_socket(self):
        self.zmq_socket.close(linger=0)
        self.zmq_socket = self._make_socket()

    def _check_graspgen_server(self):
        try:
            self.zmq_socket.setsockopt(zmq.RCVTIMEO, self._HEALTH_TIMEOUT_MS)
            self.zmq_socket.send(msgpack.packb({'action': 'health'}))
            raw = self.zmq_socket.recv()
            result = msgpack.unpackb(raw)
            if result.get('status') == 'ok':
                self.get_logger().info('GraspGen server available')
            else:
                self.get_logger().warn(f'GraspGen server unexpected status: {result}')
        except zmq.Again:
            self._recreate_socket()
            self.get_logger().warn('GraspGen server unavailable at startup')
        finally:
            self.zmq_socket.setsockopt(zmq.RCVTIMEO, -1)

    def _pointcloud2_to_numpy(self, cloud_msg):
        points = pc2.read_points(cloud_msg, field_names=('x', 'y', 'z'), skip_nans=True)
        return np.stack([points['x'], points['y'], points['z']], axis=-1).astype(np.float32)

    def _call_graspgen(self, xyz, scene_xyz=None):
        request_msg = {
            'action': 'infer',
            'point_cloud': xyz,
            'num_grasps': self.num_grasps,
            'topk_num_grasps': self.topk_num_grasps,
        }
        # scene_point_cloud is opt-in on the server: only sent when we actually
        # have collision context, so a request without it behaves exactly as
        # before (no collision filtering) — cf. GraspGen zmq_server.py.
        if scene_xyz is not None and scene_xyz.shape[0] > 0:
            request_msg['scene_point_cloud'] = scene_xyz
            request_msg['collision_threshold'] = self.collision_threshold
            request_msg['max_scene_points'] = self.max_scene_points

        self.zmq_socket.setsockopt(zmq.RCVTIMEO, self._RECV_TIMEOUT_MS)
        try:
            self.zmq_socket.send(msgpack.packb(request_msg))
            self.get_logger().info('Waiting for GraspGen response...')
            raw_response = self.zmq_socket.recv()
        except zmq.Again:
            self._recreate_socket()
            raise TimeoutError(
                f'GraspGen did not respond within {self._RECV_TIMEOUT_MS // 1000}s'
            )
        finally:
            self.zmq_socket.setsockopt(zmq.RCVTIMEO, -1)

        return msgpack.unpackb(raw_response)

    def _matrix_to_pose_array(self, grasps, frame_id):
        pose_array = PoseArray()
        pose_array.header.frame_id = frame_id
        pose_array.header.stamp = self.get_clock().now().to_msg()
        tcp_offset = np.array([0.0, 0.0, self.hand_to_tcp_offset_z])
        for grasp_matrix in grasps:
            # grasp_matrix is GraspGen's raw hand/base-link pose; translate
            # along its own local +Z (approach axis) by the hand->TCP offset
            # so the resulting pose matches fp3_hand_tcp, the frame
            # pick_place_node actually drives IK against.
            R = grasp_matrix[:3, :3]
            tcp_position = grasp_matrix[:3, 3] + R @ tcp_offset
            pose = Pose()
            pose.position.x = float(tcp_position[0])
            pose.position.y = float(tcp_position[1])
            pose.position.z = float(tcp_position[2])
            q = Rotation.from_matrix(grasp_matrix[:3, :3]).as_quat()
            pose.orientation.x = float(q[0])
            pose.orientation.y = float(q[1])
            pose.orientation.z = float(q[2])
            pose.orientation.w = float(q[3])
            pose_array.poses.append(pose)
        return pose_array

    def _trigger_visualization(self, grasps, scores):
        if not self.viz_grasps_client.service_is_ready():
            self.get_logger().error(f'Service {self.viz_grasps_client.srv_name} not available')
            return None

        req = VisualizeGrasps.Request()
        req.grasps = grasps
        req.scores = scores

        future = self.viz_grasps_client.call_async(req)
        future.add_done_callback(self._on_visualization_done)

    def _on_visualization_done(self, future):
        try:
            result = future.result()
            if result is None or not result.success:
                self.get_logger().warn(f"Grasp markers failed : {result.message if result else 'no response'}")
            else:
                self.get_logger().info('Grasp markers published')
        except Exception as e:
            self.get_logger().warn(f'Grasp markers call failed : {e}')

    def handle_generate_grasp_pose(self, request, response):
        self.get_logger().info('Grasp generation requested')
        try:
            xyz = self._pointcloud2_to_numpy(request.object_cloud)
            self.get_logger().info(f'Point cloud : {xyz.shape[0]} points')

            if xyz.shape[0] == 0:
                response.success = False
                response.message = 'empty point cloud'
                return response

            scene_xyz = None
            if request.scene_cloud.data:
                scene_xyz = self._pointcloud2_to_numpy(request.scene_cloud)
                self.get_logger().info(
                    f'Scene point cloud (collision context) : {scene_xyz.shape[0]} points'
                )

            result = self._call_graspgen(xyz, scene_xyz)

            if 'error' in result:
                response.success = False
                response.message = f"GraspGen server error : {result['error']}"
                return response

            grasps = result.get('grasps')
            scores = result.get('confidences')

            if grasps is None or scores is None:
                response.success = False
                response.message = f'unexpected server response keys : {list(result.keys())}'
                return response

            if len(grasps) != len(scores):
                response.success = False
                response.message = (
                    f'grasps/scores length mismatch : {len(grasps)} grasps vs {len(scores)} scores'
                )
                return response

            self.get_logger().info(f'{len(scores)} grasps received')

            n_before_collision = result.get('num_grasps_before_collision_filter')
            if n_before_collision is not None:
                collision_ms = result.get('timing', {}).get('collision_filter_ms', 0.0)
                self.get_logger().info(
                    f'Collision filter : {len(scores)}/{n_before_collision} grasps '
                    f'collision-free ({collision_ms:.1f}ms)'
                )

            response.grasps = self._matrix_to_pose_array(
                grasps, request.object_cloud.header.frame_id
            )
            response.scores = [float(s) for s in scores]
            response.success = True
            response.message = ''

            self._trigger_visualization(response.grasps, response.scores)

        except Exception as e:
            self.get_logger().error(f'Error : {e}')
            response.success = False
            response.message = str(e)

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
