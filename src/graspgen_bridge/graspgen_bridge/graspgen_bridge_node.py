import rclpy
from rclpy.node import Node
from franka_demo_interfaces.srv import GenerateGraspPose
from sensor_msgs_py import point_cloud2 as pc2
from geometry_msgs.msg import Pose, PoseArray
import zmq
import msgpack
import msgpack_numpy as m
import numpy as np

m.patch()

# Clés str (pas bytes) partout ci-dessous : avec msgpack>=1.0 (installé ici :
# 1.2.1), raw=False est déjà le défaut de unpackb(), et msgpack_numpy
# reconstruit correctement les ndarray même avec des clés str (vérifié). Le
# contournement "clés bytes" documenté historiquement dans CLAUDE.md ne
# s'applique qu'à une version antérieure de msgpack (raw=True par défaut) —
# obsolète sur ce système, corrigé après un vrai KeyError b'grasps' en test.


class GraspGenBridgeNode(Node):
    def __init__(self):
        super().__init__('graspgen_bridge')

        # params
        self.declare_parameter('graspgen_host', '127.0.0.1')
        self.declare_parameter('graspgen_port', 5556)
        self.declare_parameter('num_grasps', 200)
        self.declare_parameter('topk_num_grasps', 10)

        host = self.get_parameter('graspgen_host').value
        port = self.get_parameter('graspgen_port').value
        self.num_grasps = self.get_parameter('num_grasps').value
        self.topk_num_grasps = self.get_parameter('topk_num_grasps').value

        # zmq REQ socket
        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.REQ)
        self.zmq_socket.connect(f"tcp://{host}:{port}")
        self.get_logger().info(f"ZMQ REQ connected to {host}:{port}")

        # health check
        self._check_graspgen_server()

        # service ROS
        self.srv = self.create_service(
            GenerateGraspPose,
            'generate_grasp_pose',
            self.handle_generate_grasp_pose
        )

        self.get_logger().info("GraspGen Bridge started")

    def _check_graspgen_server(self):
        try:
            self.zmq_socket.setsockopt(zmq.RCVTIMEO, 3000)
            request = {"action": "health"}
            self.zmq_socket.send(msgpack.packb(request))
            raw = self.zmq_socket.recv()
            result = msgpack.unpackb(raw)

            if result.get("status") == "ok":
                self.get_logger().info("✅ GraspGen server available")
            else:
                self.get_logger().warn(f"⚠️ GraspGen server responded but with unexpected status: {result}")

            self.zmq_socket.setsockopt(zmq.RCVTIMEO, -1)

        except zmq.Again:
            self.get_logger().warn("⚠️ GraspGen server unavailable at startup")
            self.zmq_socket.setsockopt(zmq.RCVTIMEO, -1)

    def _pointcloud2_to_numpy(self, cloud_msg):
        # read_points() renvoie un tableau structuré (champs nommés x/y/z),
        # pas un tableau plat (N,3) — un cast direct dtype=float32 échoue
        # avec "Cannot cast array data from dtype([...]) to dtype('float32')".
        points = pc2.read_points(
            cloud_msg,
            field_names=("x", "y", "z"),
            skip_nans=True
        )
        return np.stack(
            [points["x"], points["y"], points["z"]], axis=-1
        ).astype(np.float32)

    def _call_graspgen(self, xyz):
        request_msg = {
            "action": "infer",
            "point_cloud": xyz,
            "num_grasps": self.num_grasps,
            "topk_num_grasps": self.topk_num_grasps,
        }
        self.zmq_socket.send(msgpack.packb(request_msg))
        self.get_logger().info("Waiting for GraspGen response...")
        raw_response = self.zmq_socket.recv()
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

            qx, qy, qz, qw = self._rotation_matrix_to_quaternion(
                grasp_matrix[:3, :3]
            )
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw

            pose_array.poses.append(pose)

        return pose_array

    def _rotation_matrix_to_quaternion(self, R):
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        return x, y, z, w

    def handle_generate_grasp_pose(self, request, response):
        self.get_logger().info("Request received from flow_manager")

        try:
            # convert PointCloud2 → numpy
            xyz = self._pointcloud2_to_numpy(request.object_cloud)
            self.get_logger().info(f"Point cloud: {xyz.shape}")

            if xyz.shape[0] == 0:
                response.success = False
                response.error_msg = "Empty point cloud"
                return response

            # call GraspGen
            result = self._call_graspgen(xyz)
            grasps = result["grasps"]
            scores = result["confidences"]
            self.get_logger().info(f"{len(scores)} grasps received")

            # convert → PoseArray
            response.grasps = self._matrix_to_pose_array(
                grasps,
                request.object_cloud.header.frame_id
            )
            response.scores = [float(s) for s in scores]
            response.success = True
            response.error_msg = ""

        except Exception as e:
            self.get_logger().error(f"Error: {e}")
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