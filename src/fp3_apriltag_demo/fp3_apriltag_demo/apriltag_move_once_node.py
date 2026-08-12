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

from franka_demo_interfaces.action import MoveToPose
from franka_msgs.action import Grasp, Move


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


# Reads AprilTag detections, moves to the requested tag's pose once, closes
# the gripper, then returns to a fixed "ready" pose and stops. /detections
# (apriltag_msgs/AprilTagDetectionArray) only carries 2D pixel corners + a
# homography, no 3D pose -- apriltag_msgs deliberately leaves pose estimation
# to the consumer, since it needs a physical tag size the detector itself
# doesn't know. So this node does its own solvePnP (corners + camera_info
# intrinsics + tag_size) to get the tag's pose in the camera optical frame,
# then transforms it into fp3_link0 via TF (a handeye_tf_publisher node is
# expected to already publish that transform -- this node does not calibrate
# anything itself).
#
# Sequence: open gripper (franka_gripper/move) -> move to tag pose -> close
# gripper (franka_gripper/grasp) -> move to ready_pose -> stop. Both gripper
# actions are called directly, NOT routed through fp3_moveit_server -- same
# rationale as pick_place_node's two-phase design: closing/opening the
# gripper is a physical action with real-world timing, not something to hide
# inside a planning step.
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
        # match exactly (observed live: IK rejects reachable, comfortably-
        # positioned poses because of it). Default is to keep the tag's
        # computed 3D position but replace the orientation with a fixed
        # straight-down TCP orientation instead, which is what actually gets
        # driven to move_to_pose.
        self.force_gripper_down = self.declare_parameter('force_gripper_down', True).value

        # TO ADJUST: placeholder "ready" pose, same convention as
        # fp3_grasp_demo's hardcoded test pose -- no attempt made here to
        # pick a pose that's guaranteed reachable or collision-free for your
        # actual scene, just a reasonable default in front of the base.
        self.ready_pose_xyz = self.declare_parameter(
            'ready_pose_xyz', [0.4, 0.0, 0.4]).value

        self.gripper_action_name = self.declare_parameter(
            'gripper_action_name', '/franka_gripper/grasp').value
        # franka_gripper_node (the real Grasp action server) isn't launched
        # in fake hardware mode -- same issue and same fix as
        # pick_place_node's simulate_gripper.
        self.simulate_gripper = self.declare_parameter('simulate_gripper', False).value
        self.grasp_width = self.declare_parameter('grasp.width', 0.06).value
        self.grasp_speed = self.declare_parameter('grasp.speed', 0.05).value
        self.grasp_force = self.declare_parameter('grasp.force', 70.0).value
        self.grasp_epsilon_inner = self.declare_parameter('grasp.epsilon_inner', 0.06).value
        self.grasp_epsilon_outer = self.declare_parameter('grasp.epsilon_outer', 0.08).value

        self.gripper_move_action_name = self.declare_parameter(
            'gripper_move_action_name', '/franka_gripper/move').value
        self.open_width = self.declare_parameter('open.width', 0.08).value
        self.open_speed = self.declare_parameter('open.speed', 0.1).value

        self._camera_info = None
        self._done = False
        # Guards _detections_cb: the opening move (below, blocking, run once
        # at startup before rclpy.spin(node)) still spins the executor
        # internally to wait for its own result, which would otherwise let a
        # detection race in and start the pick move before the gripper has
        # actually finished opening.
        self._gripper_opened = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, self.camera_info_topic, self._camera_info_cb, 10)
        self.create_subscription(
            AprilTagDetectionArray, self.detections_topic, self._detections_cb, 10)

        self._move_client = ActionClient(self, MoveToPose, 'move_to_pose')
        self._grasp_client = ActionClient(self, Grasp, self.gripper_action_name)
        self._gripper_move_client = ActionClient(self, Move, self.gripper_move_action_name)

        self._search_timer = self.create_timer(self.search_timeout_sec, self._on_search_timeout)

        self.get_logger().info(
            f"Waiting for tag id={self.target_tag_id} on '{self.detections_topic}' "
            f"(tag_size={self.tag_size} m, target frame='{self.robot_frame}')")

    # Blocking on purpose: called once from main(), before rclpy.spin(node),
    # so the gripper is confirmed open before anything reacts to a tag
    # detection. Uses spin_until_future_complete on this same node (valid
    # before it's been handed to a real executor).
    def open_gripper_blocking(self):
        if self.simulate_gripper:
            self.get_logger().info(
                f"Opening gripper (simulated, use_fake_hardware): width={self.open_width:.3f}")
            self._gripper_opened = True
            return

        self.get_logger().info(f"Waiting for gripper action '{self.gripper_move_action_name}'...")
        if not self._gripper_move_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                f"Gripper action '{self.gripper_move_action_name}' unavailable, "
                f"skipping open and proceeding anyway")
            self._gripper_opened = True
            return

        goal = Move.Goal()
        goal.width = self.open_width
        goal.speed = self.open_speed
        self.get_logger().info(
            f"Sending gripper open goal: width={goal.width:.3f} speed={goal.speed:.3f}")

        send_future = self._gripper_move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper open goal rejected, proceeding anyway')
            self._gripper_opened = True
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(
            f"Gripper open result: {'success' if result.success else 'failure'}")
        self._gripper_opened = True

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
        if self._done or not self._gripper_opened:
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
        # tree (fp3_link0, published by robot_state_publisher once it has
        # processed robot_description) exists on a fresh launch -- observed
        # live, a few seconds of gap. _process() only claims _done once it
        # has actually sent the goal; a TF miss just waits for the next
        # detection message (apriltag publishes continuously) instead of
        # killing the node, bounded by _search_timer's overall watchdog.
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
        self._send_move_goal(pose_robot, self._on_pick_move_done)

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
        # AprilTag's native order (BL,BR,TR,TL, counterclockwise). Verified
        # live: IPPE_SQUARE gave a ~0.085 m tag distance with a 135 px mean
        # reprojection error (garbage); ITERATIVE on the same input gives
        # ~0.33 m with a 0.24 px mean reprojection error. ITERATIVE has no
        # such ordering assumption, only consistent correspondence.
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

    # ---- move_to_pose (used for both the pick pose and the ready pose) ----

    def _send_move_goal(self, pose, on_done):
        self.get_logger().info("Waiting for the move_to_pose action server...")
        self._move_client.wait_for_server()

        goal = MoveToPose.Goal()
        goal.target_pose = pose

        self.get_logger().info('Sending move_to_pose goal...')
        future = self._move_client.send_goal_async(goal, feedback_callback=self._move_feedback_cb)
        future.add_done_callback(lambda f: self._move_goal_response_cb(f, on_done))

    def _move_feedback_cb(self, feedback_msg):
        self.get_logger().info(f'status: {feedback_msg.feedback.status}')

    def _move_goal_response_cb(self, future, on_done):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('move_to_pose goal rejected by fp3_moveit_server')
            on_done(False)
            return

        self.get_logger().info('Goal accepted, executing...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._move_result_cb(f, on_done))

    def _move_result_cb(self, future, on_done):
        result = future.result().result
        status = 'success' if result.success else 'failure'
        self.get_logger().info(f'move_to_pose result: {status} - {result.message}')
        on_done(result.success)

    # ---- sequence: pick move -> gripper close -> ready move -> stop ----

    def _on_pick_move_done(self, success):
        if not success:
            self.get_logger().error('Move to tag pose failed, stopping (not attempting grasp)')
            rclpy.shutdown()
            return
        self._close_gripper()

    def _close_gripper(self):
        if self.simulate_gripper:
            self.get_logger().info(
                f"Closing gripper (simulated, use_fake_hardware): "
                f"width={self.grasp_width:.3f} force={self.grasp_force:.1f}")
            self._on_grasp_done(True)
            return

        self.get_logger().info(f"Waiting for gripper action '{self.gripper_action_name}'...")
        if not self._grasp_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                f"Gripper action '{self.gripper_action_name}' unavailable, "
                f"skipping grasp and returning to ready")
            self._on_grasp_done(False)
            return

        goal = Grasp.Goal()
        goal.width = self.grasp_width
        goal.speed = self.grasp_speed
        goal.force = self.grasp_force
        goal.epsilon.inner = self.grasp_epsilon_inner
        goal.epsilon.outer = self.grasp_epsilon_outer

        self.get_logger().info(
            f"Sending grasp goal: width={goal.width:.3f} speed={goal.speed:.3f} "
            f"force={goal.force:.1f} epsilon=({goal.epsilon.inner:.3f}, {goal.epsilon.outer:.3f})")
        future = self._grasp_client.send_goal_async(goal)
        future.add_done_callback(self._grasp_goal_response_cb)

    def _grasp_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Grasp goal rejected')
            self._on_grasp_done(False)
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._grasp_result_cb)

    def _grasp_result_cb(self, future):
        result = future.result().result
        status = 'success' if result.success else 'failure'
        self.get_logger().info(f'Grasp result: {status}')
        self._on_grasp_done(result.success)

    def _on_grasp_done(self, success):
        # Return to the ready pose regardless of grasp outcome, to leave the
        # arm in a known/safe position rather than parked at the tag.
        if not success:
            self.get_logger().warn('Grasp did not succeed, returning to ready pose anyway')

        ready_pose = PoseStamped()
        ready_pose.header.frame_id = self.robot_frame
        ready_pose.header.stamp = self.get_clock().now().to_msg()
        ready_pose.pose.position.x = float(self.ready_pose_xyz[0])
        ready_pose.pose.position.y = float(self.ready_pose_xyz[1])
        ready_pose.pose.position.z = float(self.ready_pose_xyz[2])
        if self.force_gripper_down:
            ready_pose.pose.orientation.x = 1.0
            ready_pose.pose.orientation.y = 0.0
            ready_pose.pose.orientation.z = 0.0
            ready_pose.pose.orientation.w = 0.0
        else:
            ready_pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f"Returning to ready pose ({ready_pose.pose.position.x:.3f}, "
            f"{ready_pose.pose.position.y:.3f}, {ready_pose.pose.position.z:.3f})")
        self._send_move_goal(ready_pose, self._on_ready_move_done)

    def _on_ready_move_done(self, success):
        status = 'success' if success else 'failure'
        self.get_logger().info(f'Return-to-ready: {status}. Done.')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagMoveOnceNode()
    node.open_gripper_blocking()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
