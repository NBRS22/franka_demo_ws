"""
Tests whether the D455's color camera intrinsics (Fx/Fy/Cx/Cy/distortion, as
reported on /camera/camera/color/camera_info) correctly measure a 3D
displacement, WITHOUT relying on the hand-eye (fp3_link0 -> camera_link)
calibration at all.

The trick: rigid-body Euclidean distance is invariant under any unknown
rotation/translation between two frames. So:
  1. Read the tag's pose in the camera's own frame (solvePnP) before a move.
  2. Move the arm by a Cartesian delta in fp3_link0, read back from TF after
     execution (the robot's own FK, sub-mm accurate) as ground truth.
  3. Read the tag's pose in the camera's own frame again after the move.
  4. |pose_after - pose_before| in the CAMERA's own frame must equal the
     |fp3_hand displacement| in fp3_link0, regardless of any unknown rotation
     between the two frames -- if it doesn't, that's a real intrinsics-driven
     scale/measurement error, not a hand-eye calibration error (no hand-eye
     transform is used anywhere in this computation).

Needs: /move_action (fp3_moveit_server bringup, real hardware, real
move_group), /detections (apriltag_ros) on the target tag id,
/camera/camera/color/camera_info.
"""
import cv2
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
from sensor_msgs.msg import CameraInfo
from shape_msgs.msg import SolidPrimitive

TARGET_TAG_ID = 0
TAG_SIZE_M = 0.04
BASE_FRAME = 'fp3_link0'
EFFECTOR_LINK = 'fp3_hand'
PLANNING_GROUP = 'fp3_arm'
DELTA_Y_M = 0.12  # lateral move, chosen for sensitivity to Fx/Fy
SAMPLES_PER_READING = 15
POSITION_TOLERANCE_M = 0.01
ORIENTATION_TOLERANCE_RAD = 0.15
VELOCITY_SCALING = 0.1
ACCELERATION_SCALING = 0.1
PLANNING_TIME_S = 5.0
# A discrepancy below this is treated as sensor/detection noise, not a real
# intrinsics error -- calibrated against this test's own observed std
# (sub-0.1mm per single reading) plus solvePnP/PnP jitter margin.
NOISE_THRESHOLD_MM = 3.0


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


class IntrinsicsTestNode(Node):

    def __init__(self):
        super().__init__('calib_intrinsics_test')
        self._camera_info = None
        self._last_detection = None
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

    def spin_for(self, seconds):
        deadline = self.get_clock().now() + Duration(seconds=seconds)
        while self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def average_tag_position(self):
        self.spin_for(0.3)
        if self._camera_info is None:
            raise RuntimeError('no camera_info received')
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

    def get_hand_position(self):
        self.spin_for(0.2)
        tf = self.tf_buffer.lookup_transform(BASE_FRAME, EFFECTOR_LINK, rclpy.time.Time())
        t = tf.transform.translation
        q = tf.transform.rotation
        return np.array([t.x, t.y, t.z]), (q.x, q.y, q.z, q.w)

    def move_to(self, position, orientation_quat):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = PLANNING_GROUP
        req.num_planning_attempts = 5
        req.allowed_planning_time = PLANNING_TIME_S
        req.max_velocity_scaling_factor = VELOCITY_SCALING
        req.max_acceleration_scaling_factor = ACCELERATION_SCALING

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

    node.get_logger().info('Reading BEFORE tag position (camera frame)...')
    tag_before, tag_before_std = node.average_tag_position()
    node.get_logger().info(f'  tag_before = {tag_before}, std={tag_before_std}')

    hand_before, quat = node.get_hand_position()
    node.get_logger().info(f'  hand_before (fp3_link0) = {hand_before}')

    target = hand_before.copy()
    target[1] += DELTA_Y_M
    node.get_logger().info(f'Moving fp3_hand by +{DELTA_Y_M}m along fp3_link0 Y (same orientation)...')
    node.move_to(target, quat)
    node.spin_for(2.0)

    hand_after, _ = node.get_hand_position()
    node.get_logger().info(f'  hand_after (fp3_link0) = {hand_after}')
    true_robot_delta = float(np.linalg.norm(hand_after - hand_before))
    node.get_logger().info(f'  ACTUAL robot displacement (FK, ground truth) = {true_robot_delta * 1000:.2f} mm')

    node.get_logger().info('Reading AFTER tag position (camera frame)...')
    tag_after, tag_after_std = node.average_tag_position()
    node.get_logger().info(f'  tag_after = {tag_after}, std={tag_after_std}')

    camera_delta = float(np.linalg.norm(tag_after - tag_before))
    node.get_logger().info(f'  Camera-measured tag displacement = {camera_delta * 1000:.2f} mm')

    error_mm = (camera_delta - true_robot_delta) * 1000
    ratio = camera_delta / true_robot_delta if true_robot_delta > 0 else float('nan')
    node.get_logger().info('')
    node.get_logger().info('=== RESULT ===')
    node.get_logger().info(f'Robot FK displacement (ground truth): {true_robot_delta * 1000:.2f} mm')
    node.get_logger().info(f'Camera-measured displacement:          {camera_delta * 1000:.2f} mm')
    node.get_logger().info(f'Difference:                            {error_mm:+.2f} mm')
    node.get_logger().info(f'Ratio (camera/robot):                  {ratio:.4f}')
    if abs(error_mm) < NOISE_THRESHOLD_MM:
        node.get_logger().info(
            f'-> Camera intrinsics measure 3D displacement accurately (within {NOISE_THRESHOLD_MM:.0f}mm). '
            'Does not rule out intrinsics entirely, but points AWAY from them as the '
            'dominant cause of a multi-cm pick error -- the hand-eye (extrinsic) '
            'calibration remains the more likely culprit.')
    else:
        node.get_logger().info(
            '-> Meaningful discrepancy: camera intrinsics may be contributing to the '
            'observed pick error. Worth a full checkerboard recalibration '
            '(ros2 run camera_calibration cameracalibrator).')


def main(args=None):
    rclpy.init(args=args)
    node = IntrinsicsTestNode()
    try:
        run_test(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
