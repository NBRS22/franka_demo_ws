"""
Tests whether the calibration tag/cube is rigidly fixed to fp3_hand across a
large orientation swing, WITHOUT using the hand-eye calibration at all.

If fp3_hand returns to the EXACT SAME physical configuration (commanded via
the exact recorded JOINT values, not a Cartesian pose-tolerance region --
a Cartesian-tolerance return was tried first and rejected: it lets the
planner pick a different IK solution within the tolerance box, which alone
produced several mm of apparent "drift" with a perfectly rigid mount) after a
large excursion, and the camera hasn't moved, then a rigid mount MUST give an
identical tag pose in the camera's own frame both times -- no coordinate
transform to fp3_link0 needed, so no dependency on the calibration being
diagnosed elsewhere. The tag's camera-frame drift is compared against
fp3_hand's own FK drift (never exactly zero, bounded by controller
repeatability): if the tag drifts by significantly MORE than the hand itself
did, that excess is a real sign of slip; if it's about the same or less, the
mount is rigid.

Needs: /move_action (fp3_moveit_server bringup, real hardware, real
move_group), /joint_states, /detections (apriltag_ros) on the target tag id,
/camera/camera/color/camera_info.
"""
import cv2
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from sensor_msgs.msg import CameraInfo, JointState

TARGET_TAG_ID = 0
TAG_SIZE_M = 0.04
BASE_FRAME = 'fp3_link0'
EFFECTOR_LINK = 'fp3_hand'
PLANNING_GROUP = 'fp3_arm'
ARM_JOINT_NAMES = [f'fp3_joint{i}' for i in range(1, 8)]
SAMPLES_PER_READING = 15
# Tight: this is what makes the comparison valid, cf. module docstring.
JOINT_TOLERANCE_RAD = 0.002
VELOCITY_SCALING = 0.1
ACCELERATION_SCALING = 0.1
PLANNING_TIME_S = 5.0
EXCURSION_DEGREES = [
    ('roll+40', ('z', 40.0)),
    ('pitch-35', ('x', -35.0)),
    ('yaw+45', ('y', 45.0)),
]
CAMERA_TF_FRAME = 'camera_link'
BASE_ORIGIN = (0.0, 0.0, 0.0)
# Distance from the camera to start the test at -- close enough for reliable
# AprilTag detection of a 4cm tag (cf. calib_pose_tour's anchors, same
# 0.3-0.55m range). Keeping the arm's current XYZ unchanged and only
# reorienting (tried first) wasn't enough: the arm can easily be resting far
# from the camera at startup, too far/small for detection regardless of
# which way it's pointed.
START_DISTANCE_FROM_CAMERA_M = 0.4


def look_at_quaternion(from_xyz, to_xyz):
    direction = [to_xyz[i] - from_xyz[i] for i in range(3)]
    norm = sum(c * c for c in direction) ** 0.5
    direction = [c / norm for c in direction]
    rotation, _ = Rotation.align_vectors([direction], [[0.0, 0.0, 1.0]])
    return tuple(rotation.as_quat())


def position_near_camera(camera_position, distance_m=START_DISTANCE_FROM_CAMERA_M,
                          base_position=BASE_ORIGIN):
    """A point `distance_m` from the camera, along the camera->base ray."""
    toward_base = [base_position[i] - camera_position[i] for i in range(3)]
    norm = sum(c * c for c in toward_base) ** 0.5
    toward_base = [c / norm for c in toward_base]
    return tuple(camera_position[i] + toward_base[i] * distance_m for i in range(3))


def estimate_tag_position(detection, k, dist):
    half = TAG_SIZE_M / 2.0
    object_points = np.array([
        [-half, -half, 0.0], [half, -half, 0.0],
        [half, half, 0.0], [-half, half, 0.0],
    ], dtype=np.float64)
    image_points = np.array([[c.x, c.y] for c in detection.corners], dtype=np.float64)
    ok, rvec, tvec = cv2.solvePnP(object_points, image_points, k, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    return np.array([tvec[0, 0], tvec[1, 0], tvec[2, 0]])


class RigidityTestNode(Node):

    def __init__(self):
        super().__init__('calib_rigidity_test')
        self._camera_info = None
        self._last_detection = None
        self._last_joint_state = None
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self._ci_cb, 10)
        self.create_subscription(AprilTagDetectionArray, '/detections', self._det_cb, 10)
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._move_client = ActionClient(self, MoveGroup, '/move_action')

    def _ci_cb(self, msg):
        self._camera_info = msg

    def _det_cb(self, msg):
        for d in msg.detections:
            if d.id == TARGET_TAG_ID:
                self._last_detection = d

    def _js_cb(self, msg):
        if all(name in msg.name for name in ARM_JOINT_NAMES):
            self._last_joint_state = dict(zip(msg.name, msg.position))

    def spin_for(self, seconds):
        deadline = self.get_clock().now() + Duration(seconds=seconds)
        while self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_camera_info(self):
        while self._camera_info is None:
            rclpy.spin_once(self, timeout_sec=0.1)

    def lookup_camera_position(self, timeout_s=45.0):
        # Manual spin loop, not tf_buffer.lookup_transform(..., timeout=...):
        # that variant needs a concurrently-spinning executor to ever see new
        # /tf_static messages, which nothing provides here (no
        # rclpy.spin(node) running, no spin_thread=True on the listener --
        # the latter would add this same node to a second executor, which
        # conflicts with rclpy.spin_until_future_complete(self, ...) used
        # elsewhere for MoveGroup action results). Same pattern as
        # calib_pose_tour's _lookup_camera_position.
        self.get_logger().info(
            f"Looking up TF '{BASE_FRAME}' -> '{CAMERA_TF_FRAME}' (up to {timeout_s:.0f}s, "
            f"published by handeye_tf_publisher from the currently-loaded calibration)...")
        deadline = self.get_clock().now() + Duration(seconds=timeout_s)
        last_exc = None
        while self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            try:
                tf = self.tf_buffer.lookup_transform(BASE_FRAME, CAMERA_TF_FRAME, rclpy.time.Time())
                t = tf.transform.translation
                return (t.x, t.y, t.z)
            except TransformException as ex:
                last_exc = ex
        raise RuntimeError(f"Could not look up camera position: {last_exc}")

    def average_tag_position(self):
        self.wait_for_camera_info()
        k = np.array(self._camera_info.k, dtype=np.float64).reshape(3, 3)
        dist = np.array(self._camera_info.d, dtype=np.float64)
        positions = []
        while len(positions) < SAMPLES_PER_READING:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._last_detection is not None:
                pos = estimate_tag_position(self._last_detection, k, dist)
                if pos is not None:
                    positions.append(pos)
            self._last_detection = None
        arr = np.array(positions)
        return arr.mean(axis=0), arr.std(axis=0)

    def get_hand_pose(self):
        self.spin_for(0.2)
        tf = self.tf_buffer.lookup_transform(BASE_FRAME, EFFECTOR_LINK, rclpy.time.Time())
        t = tf.transform.translation
        q = tf.transform.rotation
        return np.array([t.x, t.y, t.z]), (q.x, q.y, q.z, q.w)

    def get_joint_positions(self):
        self.spin_for(0.2)
        while self._last_joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return {name: self._last_joint_state[name] for name in ARM_JOINT_NAMES}

    def _send_goal(self, constraints):
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

    def move_to_pose(self, position, orientation_quat):
        from moveit_msgs.msg import OrientationConstraint, PositionConstraint
        from shape_msgs.msg import SolidPrimitive
        constraints = Constraints()
        pos_c = PositionConstraint()
        pos_c.header.frame_id = BASE_FRAME
        pos_c.link_name = EFFECTOR_LINK
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.02, 0.02, 0.02]
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
        orient_c.absolute_x_axis_tolerance = 0.15
        orient_c.absolute_y_axis_tolerance = 0.15
        orient_c.absolute_z_axis_tolerance = 0.15
        orient_c.weight = 1.0
        constraints.orientation_constraints.append(orient_c)
        self._send_goal(constraints)

    def move_to_joints(self, joint_positions):
        constraints = Constraints()
        for name, pos in joint_positions.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = JOINT_TOLERANCE_RAD
            jc.tolerance_below = JOINT_TOLERANCE_RAD
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        self._send_goal(constraints)


def run_test(node):
    node.spin_for(2.0)
    camera_position = node.lookup_camera_position()

    start_pos = position_near_camera(camera_position)
    face_camera_quat = look_at_quaternion(start_pos, camera_position)
    node.get_logger().info(
        f'Moving to start_pos={start_pos} ({START_DISTANCE_FROM_CAMERA_M:.2f}m from '
        f'camera_position={camera_position}, live TF) facing the camera, so the tag is '
        f'close enough to be reliably detected for the START reading...')
    try:
        node.move_to_pose(start_pos, face_camera_quat)
    except RuntimeError as e:
        node.get_logger().warn(
            f'Could not move near the camera ({e}) -- proceeding with whatever '
            'orientation the arm is already at; the tag may not be visible.')
    node.spin_for(1.0)

    node.get_logger().info('Reading tag pose at START pose...')
    home_pos, home_quat = node.get_hand_pose()
    home_joints = node.get_joint_positions()
    tag_start, tag_start_std = node.average_tag_position()
    node.get_logger().info(f'  fp3_hand = {home_pos}')
    node.get_logger().info(f'  tag (camera frame) = {tag_start}, std={tag_start_std}')

    base_rot = Rotation.from_quat(home_quat)
    for label, (axis, deg) in EXCURSION_DEGREES:
        q = (base_rot * Rotation.from_euler(axis, deg, degrees=True)).as_quat()
        node.get_logger().info(f'Excursion: {label}...')
        try:
            node.move_to_pose(home_pos, tuple(q))
        except RuntimeError as e:
            node.get_logger().warn(f'  (skipped, unreachable: {e})')
        node.spin_for(1.0)

    node.get_logger().info('Returning to START pose (exact recorded joint values, tight tolerance)...')
    node.move_to_joints(home_joints)
    node.spin_for(2.0)

    hand_end, _ = node.get_hand_pose()
    end_joints = node.get_joint_positions()
    fk_drift_mm = float(np.linalg.norm(hand_end - home_pos) * 1000)
    joint_drift_deg = max(abs(np.degrees(end_joints[n] - home_joints[n])) for n in ARM_JOINT_NAMES)
    node.get_logger().info(f'  fp3_hand back at {hand_end} (FK drift from start: {fk_drift_mm:.3f} mm)')
    node.get_logger().info(f'  max joint drift: {joint_drift_deg:.3f} deg')

    tag_end, tag_end_std = node.average_tag_position()
    node.get_logger().info(f'  tag (camera frame) = {tag_end}, std={tag_end_std}')

    tag_drift_mm = float(np.linalg.norm(tag_end - tag_start) * 1000)

    node.get_logger().info('')
    node.get_logger().info('=== RESULT ===')
    node.get_logger().info(f'fp3_hand FK drift (start vs end):     {fk_drift_mm:.3f} mm')
    node.get_logger().info(f'Tag drift in camera frame:            {tag_drift_mm:.3f} mm')
    excess_mm = tag_drift_mm - fk_drift_mm
    node.get_logger().info(f'Excess beyond the hand\'s own drift:   {excess_mm:+.3f} mm')
    if excess_mm < 1.0:
        node.get_logger().info(
            '-> RIGID: the tag moved by about the same amount as fp3_hand itself did '
            '(no drift left unexplained). Slip is NOT a contributor.')
    else:
        node.get_logger().info(
            '-> NOT RIGID: the tag moved MORE than fp3_hand itself did -- excess is a '
            'real sign of slip, amplified by the tag\'s distance from fp3_hand.')


def main(args=None):
    rclpy.init(args=args)
    node = RigidityTestNode()
    try:
        run_test(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
