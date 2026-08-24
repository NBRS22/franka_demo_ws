import cv2
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs  # noqa: F401  (registers PoseStamped support on Buffer.transform)

from franka_demo_interfaces.action import MtcPick


def rotation_matrix_to_quaternion(r):
    # Standard Shepperd's method. Avoids pulling in tf_transformations/scipy
    # for a single conversion.
    trace = r[0, 0] + r[1, 1] + r[2, 2]
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (r[2, 1] - r[1, 2]) * s
        y = (r[0, 2] - r[2, 0]) * s
        z = (r[1, 0] - r[0, 1]) * s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = 2.0 * np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = 2.0 * np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


def _quaternion_to_euler_deg(x, y, z, w):
    # Standard intrinsic ZYX (yaw-pitch-roll) extraction, degrees. Diagnostic
    # display only -- not used for any control/planning decision.
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.degrees(np.arctan2(sinr_cosp, cosr_cosp))

    sinp = 2.0 * (w * y - z * x)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.degrees(np.arcsin(sinp))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.degrees(np.arctan2(siny_cosp, cosy_cosp))

    return roll, pitch, yaw


# Real-hardware check that the eye-on-base calibration (handeye_tf_publisher)
# is accurate: reads one AprilTag detection, computes its 3D pose via
# solvePnP + TF (through fp3_link0 -> camera_link -> ... published by
# handeye_tf_publisher, no calibration done here), and sends it as a
# single-candidate goal to mtc_pick (command_router_node). If the pick
# succeeds -- approach, close the gripper, lift -- the arm physically
# reached and grasped exactly where the camera says the tag is, which is a
# stronger confirmation than eyeballing the point cloud in RViz (position
# AND depth both have to be right for a real grasp to succeed).
#
# /detections (apriltag_msgs/AprilTagDetectionArray) only carries 2D pixel
# corners + a homography, no 3D pose -- apriltag_msgs deliberately leaves
# pose estimation to the consumer, since it needs a physical tag size the
# detector itself doesn't know. Unlike the older version of this demo (a
# separate franka_demo_ws, since ported here), there is no direct
# move_to_pose/motion_server_node action anymore -- mtc_pick already does
# open -> approach -> close -> attach -> lift -> detach as a single action
# call (cf. fp3_moveit_server/CLAUDE.md), so this node no longer needs its
# own gripper action clients or a separate "return to ready" step.
class AprilTagMoveOnceNode(Node):

    def __init__(self):
        super().__init__('apriltag_move_once_node')

        self.tag_size = self.declare_parameter('tag_size', 0.04).value
        self.target_tag_id = self.declare_parameter('target_tag_id', 0).value
        self.robot_frame = self.declare_parameter('robot_frame', 'fp3_link0').value
        self.detections_topic = self.declare_parameter('detections_topic', '/detections').value
        self.camera_info_topic = self.declare_parameter(
            'camera_info_topic', '/camera/camera/color/camera_info').value
        self.tf_timeout_sec = self.declare_parameter('tf_timeout_sec', 2.0).value
        self.search_timeout_sec = self.declare_parameter('search_timeout_sec', 30.0).value
        # The tag's own orientation is often kinematically hard/impossible to
        # match exactly. Default is to keep the tag's computed 3D position
        # but replace the orientation with a fixed straight-down TCP
        # orientation instead -- same convention as pick_place_node's
        # approach axis (local +Z), cf. rotation below.
        self.force_gripper_down = self.declare_parameter('force_gripper_down', True).value

        self._camera_info = None
        self._done = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, self.camera_info_topic, self._camera_info_cb, 10)
        self.create_subscription(
            AprilTagDetectionArray, self.detections_topic, self._detections_cb, 10)

        self._mtc_pick_client = ActionClient(self, MtcPick, 'mtc_pick')

        self._search_timer = self.create_timer(self.search_timeout_sec, self._on_search_timeout)

        self.get_logger().info(
            f"Waiting for tag id={self.target_tag_id} on '{self.detections_topic}' "
            f"(tag_size={self.tag_size} m, target frame='{self.robot_frame}')")

    def _camera_info_cb(self, msg):
        self._camera_info = msg

    def _on_search_timeout(self):
        if self._done:
            return
        self._done = True
        self.get_logger().error(
            f"No detection for tag id={self.target_tag_id} within "
            f"{self.search_timeout_sec:.1f}s, giving up")
        rclpy.shutdown()

    def _detections_cb(self, msg):
        if self._done:
            return
        if self._camera_info is None:
            self.get_logger().warn('Detection received but no camera_info yet, skipping')
            return

        detection = next(
            (d for d in msg.detections if d.id == self.target_tag_id), None)
        if detection is None:
            return

        # Deliberately not claiming self._done here: the camera/apriltag
        # pipeline is a separate, already-running process, so the first
        # matching detection can arrive well before the arm stack's own TF
        # tree (fp3_link0, published by robot_state_publisher) exists on a
        # fresh launch. _process() only claims _done once it has actually
        # sent the goal; a TF miss just waits for the next detection message
        # instead of killing the node, bounded by _search_timer's watchdog.
        self._process(detection, msg.header)

    def _process(self, detection, header):
        pose_camera = self._estimate_pose(detection, header)
        if pose_camera is None:
            rclpy.shutdown()
            return

        try:
            pose_robot = self.tf_buffer.transform(
                pose_camera, self.robot_frame, timeout=Duration(seconds=self.tf_timeout_sec))
        except TransformException as ex:
            self.get_logger().warn(
                f"Could not transform tag pose from '{pose_camera.header.frame_id}' to "
                f"'{self.robot_frame}' yet, will retry on the next detection: {ex}")
            return

        # Logged unconditionally (even when force_gripper_down discards it below) --
        # a calibration-induced rotation bias would show up here numerically, without
        # having to risk a real grasp attempt at an orientation mtc_pick's own tilt
        # filter might reject outright for unrelated (kinematic) reasons.
        native_q = pose_robot.pose.orientation
        roll_deg, pitch_deg, yaw_deg = _quaternion_to_euler_deg(
            native_q.x, native_q.y, native_q.z, native_q.w)
        self.get_logger().info(
            f"Tag {self.target_tag_id} native orientation in '{self.robot_frame}': "
            f"roll={roll_deg:.1f}deg pitch={pitch_deg:.1f}deg yaw={yaw_deg:.1f}deg "
            f"(quaternion x={native_q.x:.3f} y={native_q.y:.3f} z={native_q.z:.3f} w={native_q.w:.3f})")

        if self.force_gripper_down:
            # 180-degree rotation about X: maps the TCP's local +Z (the
            # approach axis, per this codebase's convention -- see
            # pick_place_node.cpp) to world -Z, i.e. straight down. Ignores
            # the tag's own orientation/yaw entirely, only its position is
            # kept.
            pose_robot.pose.orientation.x = 1.0
            pose_robot.pose.orientation.y = 0.0
            pose_robot.pose.orientation.z = 0.0
            pose_robot.pose.orientation.w = 0.0

        self._done = True
        self._search_timer.cancel()
        self.get_logger().info(
            f"Tag {self.target_tag_id} pose in '{self.robot_frame}': "
            f"({pose_robot.pose.position.x:.3f}, {pose_robot.pose.position.y:.3f}, "
            f"{pose_robot.pose.position.z:.3f}) "
            f"orientation={'forced straight-down' if self.force_gripper_down else 'tag-native'}")
        self._send_mtc_pick_goal(pose_robot)

    def _estimate_pose(self, detection, header):
        k = np.array(self._camera_info.k, dtype=np.float64).reshape(3, 3)
        dist = np.array(self._camera_info.d, dtype=np.float64)

        half = self.tag_size / 2.0
        # Matches the standard AprilTag corner order (bottom-left,
        # bottom-right, top-right, top-left) in the tag's own frame: x right,
        # y up, z out of the tag toward the camera.
        object_points = np.array([
            [-half, -half, 0.0],
            [half, -half, 0.0],
            [half, half, 0.0],
            [-half, half, 0.0],
        ], dtype=np.float64)
        image_points = np.array(
            [[c.x, c.y] for c in detection.corners], dtype=np.float64)

        # NOT SOLVEPNP_IPPE_SQUARE: despite being the "textbook" choice for a
        # planar square target, it hardcodes an internal assumption about
        # corner order (OpenCV's own clockwise TL,TR,BR,BL) and silently
        # returns a wrong closed-form solution if the correspondence isn't in
        # that exact order -- which it isn't here, since apriltag_msgs uses
        # AprilTag's native order (BL,BR,TR,TL, counterclockwise). ITERATIVE
        # has no such ordering assumption, only consistent correspondence.
        ok, rvec, tvec = cv2.solvePnP(
            object_points, image_points, k, dist, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            self.get_logger().error('solvePnP failed to estimate tag pose')
            return None

        rotation_matrix, _ = cv2.Rodrigues(rvec)
        qx, qy, qz, qw = rotation_matrix_to_quaternion(rotation_matrix)

        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x = float(tvec[0])
        pose.pose.position.y = float(tvec[1])
        pose.pose.position.z = float(tvec[2])
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    # ---- mtc_pick (open -> approach -> close -> attach -> lift -> detach,
    # all handled by pick_place_node -- this node only sends the one
    # candidate pose it computed) ----

    def _send_mtc_pick_goal(self, pose_robot):
        self.get_logger().info("Waiting for the mtc_pick action server...")
        self._mtc_pick_client.wait_for_server()

        goal = MtcPick.Goal()
        goal.grasp_poses = [pose_robot]
        goal.scores = [1.0]

        self.get_logger().info('Sending mtc_pick goal...')
        future = self._mtc_pick_client.send_goal_async(
            goal, feedback_callback=self._mtc_pick_feedback_cb)
        future.add_done_callback(self._mtc_pick_goal_response_cb)

    def _mtc_pick_feedback_cb(self, feedback_msg):
        self.get_logger().info(f'status: {feedback_msg.feedback.status}')

    def _mtc_pick_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('mtc_pick goal rejected by command_router_node')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted, executing...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._mtc_pick_result_cb)

    def _mtc_pick_result_cb(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(
                f'CALIBRATION CHECK PASSED: mtc_pick succeeded on the AprilTag pose '
                f'(used_pose_index={result.used_pose_index}). The arm reached and '
                f'grasped exactly where the camera says the tag is.')
        else:
            self.get_logger().error(
                f'CALIBRATION CHECK FAILED: mtc_pick did not succeed on the AprilTag '
                f'pose -- {result.message}. Either the calibration is off, or the '
                f'candidate pose was filtered/unreachable for an unrelated reason '
                f'(check the message above and pick_place_node\'s own logs).')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagMoveOnceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
