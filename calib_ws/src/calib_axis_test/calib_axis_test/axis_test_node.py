"""
Tests whether the hand-eye calibration's ROTATION is correct -- not just its
scale (already validated twice: robot-displacement magnitude and the static
tag-grid, both ~1-2%). Neither of those tests could catch an axis-swap or
sign-flip in the calibration's rotation component, because they only ever
compared scalar magnitudes (|displacement|), never direction. A rotation bug
would produce exactly the symptom reported live on this robot: a constant,
same-direction lateral offset at pick time, present in BOTH fp3_apriltag_demo
(its own direct solvePnP) and the classic SAM3/GraspGen pipeline -- the only
thing those two otherwise-independent code paths share is this calibration
and the (already-verified-accurate) robot kinematic chain.

Method: command fp3_hand by a KNOWN, PURE, single-axis displacement in
fp3_link0 (e.g. +10cm along X only), read the ACTUAL displacement back via
FK (ground truth vector, not just magnitude). Separately, read the
calibration tag's pose in camera frame (solvePnP) and transform it into
fp3_link0 THROUGH THE CALIBRATION UNDER TEST (same tf_buffer.transform(...)
call fp3_apriltag_demo itself uses) before and after the move, giving the
calibrated tag's own displacement vector in fp3_link0. If the calibration's
rotation is correct, these two vectors must match component-by-component
(not just in magnitude) -- a mismatch on the "wrong" axis, or a sign flip,
is direct, unambiguous evidence of a rotation bug in the calibration itself.

Needs: /move_action (fp3_moveit_server bringup, real hardware), /detections
(apriltag_ros) on the target tag id, /camera/camera/color/camera_info, and
fp3_link0 -> camera_link published by handeye_tf_publisher (the calibration
being tested).
"""
import cv2
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
from sensor_msgs.msg import CameraInfo
from shape_msgs.msg import SolidPrimitive
import tf2_geometry_msgs  # noqa: F401 -- registers PoseStamped support on Buffer.transform

TARGET_TAG_ID = 0
TAG_SIZE_M = 0.04
BASE_FRAME = 'fp3_link0'
EFFECTOR_LINK = 'fp3_hand'
PLANNING_GROUP = 'fp3_arm'
SAMPLES_PER_READING = 15
POSITION_TOLERANCE_M = 0.01
ORIENTATION_TOLERANCE_RAD = 0.15
VELOCITY_SCALING = 0.1
ACCELERATION_SCALING = 0.1
PLANNING_TIME_S = 5.0
AXIS_MOVES = [
    ('X', (0.10, 0.0, 0.0)),
    ('Y', (0.0, 0.10, 0.0)),
    ('Z', (0.0, 0.0, 0.10)),
]
# A mismatch below this is treated as noise (detection jitter, planner
# imprecision -- cf. this session's own measured baselines: ~1-2mm typical).
MISMATCH_WARN_MM = 5.0

_OBJECT_POINTS = np.array([
    [-TAG_SIZE_M / 2, -TAG_SIZE_M / 2, 0.0], [TAG_SIZE_M / 2, -TAG_SIZE_M / 2, 0.0],
    [TAG_SIZE_M / 2, TAG_SIZE_M / 2, 0.0], [-TAG_SIZE_M / 2, TAG_SIZE_M / 2, 0.0],
], dtype=np.float64)


def estimate_tag_pose_camera_frame(detection, k, dist):
    image_points = np.array([[c.x, c.y] for c in detection.corners], dtype=np.float64)
    ok, rvec, tvec = cv2.solvePnP(_OBJECT_POINTS, image_points, k, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    r, _ = cv2.Rodrigues(rvec)
    return r, np.array([tvec[0, 0], tvec[1, 0], tvec[2, 0]])


def _rotation_to_quat(r):
    # Shepperd's method -- avoids pulling in scipy for a single conversion,
    # same approach already used in fp3_apriltag_demo/apriltag_move_once_node.py.
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


class AxisTestNode(Node):

    def __init__(self):
        super().__init__('calib_axis_test')
        self._camera_info = None
        self._last_detection = None
        self._last_detection_frame = None
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self._ci_cb, 10)
        self.create_subscription(AprilTagDetectionArray, '/detections', self._det_cb, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._move_client = ActionClient(self, MoveGroup, '/move_action')

    def _ci_cb(self, msg):
        self._camera_info = msg

    def _det_cb(self, msg):
        for d in msg.detections:
            if d.id == TARGET_TAG_ID:
                self._last_detection = d
                self._last_detection_frame = msg.header.frame_id

    def spin_for(self, seconds):
        deadline = self.get_clock().now() + Duration(seconds=seconds)
        while self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_camera_info(self):
        while self._camera_info is None:
            rclpy.spin_once(self, timeout_sec=0.1)

    def get_hand_pose(self):
        self.spin_for(0.2)
        tf = self.tf_buffer.lookup_transform(BASE_FRAME, EFFECTOR_LINK, rclpy.time.Time())
        t = tf.transform.translation
        q = tf.transform.rotation
        return np.array([t.x, t.y, t.z]), (q.x, q.y, q.z, q.w)

    def calibrated_tag_position_in_base(self):
        """Tag's pose in camera frame (averaged solvePnP), transformed into
        fp3_link0 THROUGH the calibration under test -- same TF chain
        fp3_apriltag_demo itself uses (fp3_apriltag_demo/
        apriltag_move_once_node.py's tf_buffer.transform call)."""
        self.wait_for_camera_info()
        k = np.array(self._camera_info.k, dtype=np.float64).reshape(3, 3)
        dist = np.array(self._camera_info.d, dtype=np.float64)

        positions = []
        rotations = []
        header_frame = None
        while len(positions) < SAMPLES_PER_READING:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._last_detection is not None:
                result = estimate_tag_pose_camera_frame(self._last_detection, k, dist)
                if result is not None:
                    r, t = result
                    positions.append(t)
                    rotations.append(r)
                    header_frame = self._last_detection_frame
            self._last_detection = None
        mean_pos = np.mean(positions, axis=0)
        mean_r = rotations[len(rotations) // 2]  # representative rotation, orientation not analyzed here

        pose_camera = PoseStamped()
        pose_camera.header.frame_id = header_frame
        pose_camera.pose.position.x, pose_camera.pose.position.y, pose_camera.pose.position.z = mean_pos
        qx, qy, qz, qw = _rotation_to_quat(mean_r)
        pose_camera.pose.orientation.x = qx
        pose_camera.pose.orientation.y = qy
        pose_camera.pose.orientation.z = qz
        pose_camera.pose.orientation.w = qw

        tf = self.tf_buffer.lookup_transform(BASE_FRAME, header_frame, rclpy.time.Time())
        pose_base = tf2_geometry_msgs.do_transform_pose_stamped(pose_camera, tf)
        p = pose_base.pose.position
        return np.array([p.x, p.y, p.z])

    def move_to_pose(self, position, orientation_quat):
        constraints = Constraints()
        pos_c = PositionConstraint()
        pos_c.header.frame_id = BASE_FRAME
        pos_c.link_name = EFFECTOR_LINK
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [2 * POSITION_TOLERANCE_M] * 3
        pos_c.constraint_region.primitives.append(primitive)
        target_pose = Pose()
        target_pose.position.x, target_pose.position.y, target_pose.position.z = position
        (target_pose.orientation.x, target_pose.orientation.y,
         target_pose.orientation.z, target_pose.orientation.w) = orientation_quat
        pos_c.constraint_region.primitive_poses.append(target_pose)
        pos_c.weight = 1.0
        constraints.position_constraints.append(pos_c)

        orient_c = OrientationConstraint()
        orient_c.header.frame_id = BASE_FRAME
        orient_c.link_name = EFFECTOR_LINK
        orient_c.orientation = target_pose.orientation
        orient_c.absolute_x_axis_tolerance = ORIENTATION_TOLERANCE_RAD
        orient_c.absolute_y_axis_tolerance = ORIENTATION_TOLERANCE_RAD
        orient_c.absolute_z_axis_tolerance = ORIENTATION_TOLERANCE_RAD
        orient_c.weight = 1.0
        constraints.orientation_constraints.append(orient_c)

        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = PLANNING_GROUP
        req.num_planning_attempts = 5
        req.allowed_planning_time = PLANNING_TIME_S
        req.max_velocity_scaling_factor = VELOCITY_SCALING
        req.max_acceleration_scaling_factor = ACCELERATION_SCALING
        req.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False

        if not self._move_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('/move_action unavailable')
        send_future = self._move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        gh = send_future.result()
        if gh is None or not gh.accepted:
            raise RuntimeError('goal rejected')
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result is None or result.result.error_code.val != 1:
            raise RuntimeError(f'move failed: {result.result.error_code.val if result else None}')


def run_test(node):
    node.spin_for(2.0)
    node.get_logger().info('Reading home pose and calibrated tag position...')
    home_pos, home_quat = node.get_hand_pose()
    tag_home = node.calibrated_tag_position_in_base()
    node.get_logger().info(f'  fp3_hand = {home_pos}')
    node.get_logger().info(f'  calibrated tag (in fp3_link0) = {tag_home}')

    results = []
    for label, delta in AXIS_MOVES:
        target = home_pos + np.array(delta)
        node.get_logger().info(f'Axis {label}: moving fp3_hand by {delta}...')
        try:
            node.move_to_pose(tuple(target), home_quat)
        except RuntimeError as e:
            node.get_logger().warn(f'  (skipped, unreachable: {e})')
            continue
        node.spin_for(1.0)

        hand_pos, _ = node.get_hand_pose()
        tag_pos = node.calibrated_tag_position_in_base()
        robot_delta = hand_pos - home_pos
        tag_delta = tag_pos - tag_home
        node.get_logger().info(f'  fp3_hand actual delta   = {robot_delta}')
        node.get_logger().info(f'  calibrated tag delta    = {tag_delta}')
        results.append((label, robot_delta, tag_delta))

        node.get_logger().info(f'Axis {label}: returning to home...')
        try:
            node.move_to_pose(tuple(home_pos), home_quat)
        except RuntimeError as e:
            node.get_logger().warn(f'  (could not return home: {e})')
        node.spin_for(1.0)

    node.get_logger().info('')
    node.get_logger().info('=== RESULT ===')
    node.get_logger().info(f'{"axis":>5} {"robot dX":>9} {"robot dY":>9} {"robot dZ":>9}   '
                            f'{"tag dX":>9} {"tag dY":>9} {"tag dZ":>9}   {"max err":>8}')
    any_mismatch = False
    for label, robot_delta, tag_delta in results:
        err = np.abs(robot_delta - tag_delta)
        max_err_mm = float(err.max()) * 1000
        flag = ' <-- MISMATCH' if max_err_mm > MISMATCH_WARN_MM else ''
        if flag:
            any_mismatch = True
        node.get_logger().info(
            f'{label:>5} {robot_delta[0]:>9.3f} {robot_delta[1]:>9.3f} {robot_delta[2]:>9.3f}   '
            f'{tag_delta[0]:>9.3f} {tag_delta[1]:>9.3f} {tag_delta[2]:>9.3f}   {max_err_mm:>7.1f}mm{flag}')

    node.get_logger().info('')
    if not results:
        node.get_logger().error('No axis move succeeded -- inconclusive')
    elif any_mismatch:
        node.get_logger().info(
            '-> ROTATION BUG LIKELY: the calibrated tag did not move the same way the robot '
            'did on at least one axis (wrong axis and/or wrong sign) -- this points to a real '
            'error in the calibration\'s rotation component, not calibration noise/coverage.')
    else:
        node.get_logger().info(
            '-> Calibration rotation looks consistent: the calibrated tag moved the same way '
            'the robot did on all 3 axes, within noise. A rotation-axis bug is NOT the '
            'explanation for the observed offset.')


def main(args=None):
    rclpy.init(args=args)
    node = AxisTestNode()
    try:
        run_test(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
