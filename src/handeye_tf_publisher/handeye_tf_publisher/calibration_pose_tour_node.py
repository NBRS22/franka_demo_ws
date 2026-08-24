import time

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
from shape_msgs.msg import SolidPrimitive


# Anchors are computed at RUNTIME from the live camera position (cf.
# _build_anchor_positions below) -- NOT as fixed fp3_link0-frame points.
# First cut used fixed points near fp3_link0's own reach envelope
# (0.35-0.55m from the robot base) without checking actual distance to the
# camera -- turned out to put every anchor ~1.0-1.1m from the camera (given
# camera_pos ~= (0.2, 0.99, 0.49) in fp3_link0, far from the anchors' y~=0),
# well past AprilTag's reliable detection range and the user's own working
# distance during manual sampling. Anchors are now placed along the
# camera->base ray at controlled DISTANCES FROM THE CAMERA instead, matching
# the README's own manual pose plan convention ("Close ~40cm / Center ~60cm
# / Far ~80cm from camera") but biased closer per explicit feedback -- close
# to camera unavoidably means a larger reach from fp3_link0 (the camera
# itself sits ~1.05-1.1m from the base), so these distances are chosen to
# stay under fp3's ~0.85m max reach with some margin.
_DISTANCES_FROM_CAMERA_M = [0.30, 0.35, 0.40, 0.45, 0.55]
_LATERAL_OFFSET_M = 0.12


def _normalize(v):
    norm = sum(c * c for c in v) ** 0.5
    return tuple(c / norm for c in v)


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _build_anchor_positions(camera_pos, base_pos=(0.0, 0.0, 0.0)):
    toward_base = _normalize(tuple(base_pos[i] - camera_pos[i] for i in range(3)))
    # Any helper not parallel to toward_base gives a valid perpendicular basis.
    helper = (0.0, 0.0, 1.0) if abs(toward_base[2]) < 0.9 else (1.0, 0.0, 0.0)
    right = _normalize(_cross(toward_base, helper))
    up = _normalize(_cross(right, toward_base))

    anchors = []
    for i, dist in enumerate(_DISTANCES_FROM_CAMERA_M):
        center = tuple(camera_pos[k] + toward_base[k] * dist for k in range(3))
        lateral = right if i % 2 == 0 else up
        sign = 1.0 if (i // 2) % 2 == 0 else -1.0
        offset = tuple(lateral[k] * _LATERAL_OFFSET_M * sign for k in range(3))
        anchors.append(tuple(center[k] + offset[k] for k in range(3)))
    return anchors

# Orientation variants per anchor, applied on top of a "look at the camera"
# baseline (local +Z, this codebase's approach-axis convention -- cf.
# pick_place_node.cpp -- pointed from the anchor toward the camera). Confirmed
# by the user: this "facing the camera" orientation is what worked during
# manual sample-taking. Perturbations are intrinsic (local-frame) rotations on
# top of that baseline, kept moderate (25deg) to stay within AprilTag's
# reliable detection tilt range (cf. handeye_tf_publisher/README.md: "avoid
# tilt angles > 60deg") while still hitting >= 3 non-parallel rotation axes
# per the README's own sampling guidance.
_ORIENTATION_VARIANTS = [
    ('baseline', None),
    ('roll+25', ('z', 25.0)),   # about the local approach axis
    ('pitch+25', ('x', 25.0)),
    ('yaw+25', ('y', 25.0)),
]

_PLANNING_GROUP = 'fp3_arm'
_EFFECTOR_LINK = 'fp3_hand'  # matches robot_effector_frame used for calibration
_BASE_FRAME = 'fp3_link0'
_CAMERA_FRAME = 'camera_link'

_POSITION_TOLERANCE_M = 0.01
_ORIENTATION_TOLERANCE_RAD = 0.15
_VELOCITY_SCALING = 0.1
_ACCELERATION_SCALING = 0.1
_PLANNING_TIME_S = 5.0
_DWELL_S = 4.0


def _look_at_quaternion(from_xyz, to_xyz, base_rotation_axis_deg=None):
    direction = [to_xyz[i] - from_xyz[i] for i in range(3)]
    norm = sum(c * c for c in direction) ** 0.5
    direction = [c / norm for c in direction]

    baseline, _ = Rotation.align_vectors([direction], [[0.0, 0.0, 1.0]])

    if base_rotation_axis_deg is None:
        rotation = baseline
    else:
        axis, deg = base_rotation_axis_deg
        rotation = baseline * Rotation.from_euler(axis, deg, degrees=True)

    x, y, z, w = rotation.as_quat()
    return x, y, z, w


def _build_poses(camera_pos):
    anchors = _build_anchor_positions(camera_pos)
    poses = []
    for anchor in anchors:
        for label, variant in _ORIENTATION_VARIANTS:
            poses.append((anchor, variant, label))
    return poses


class CalibrationPoseTourNode(Node):

    def __init__(self):
        super().__init__('calibration_pose_tour_node')

        self.tf_buffer = Buffer()
        # No spin_thread=True here: it would add this same node to its own
        # dedicated executor, which then conflicts with
        # rclpy.spin_until_future_complete(self, ...) used later for MoveGroup
        # action results (a node can only belong to one executor at a time in
        # rclpy). Instead, _lookup_camera_position() below manually pumps
        # rclpy.spin_once(self, ...) itself so this listener's /tf and
        # /tf_static subscription callbacks actually get a chance to run.
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._move_client = ActionClient(self, MoveGroup, '/move_action')

    def _lookup_camera_position(self, timeout_s=10.0):
        self.get_logger().info(
            f"Waiting for TF '{_BASE_FRAME}' -> '{_CAMERA_FRAME}' (up to {timeout_s:.0f}s)...")
        deadline = self.get_clock().now() + Duration(seconds=timeout_s)
        last_exc = None
        while self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            try:
                tf = self.tf_buffer.lookup_transform(
                    _BASE_FRAME, _CAMERA_FRAME, rclpy.time.Time())
                t = tf.transform.translation
                return (t.x, t.y, t.z)
            except TransformException as ex:
                last_exc = ex
        self.get_logger().error(f"Could not look up camera position: {last_exc}")
        return None

    def _send_pose_goal(self, position, orientation_quat):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = _PLANNING_GROUP
        req.num_planning_attempts = 5
        req.allowed_planning_time = _PLANNING_TIME_S
        req.max_velocity_scaling_factor = _VELOCITY_SCALING
        req.max_acceleration_scaling_factor = _ACCELERATION_SCALING

        constraints = Constraints()

        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = _BASE_FRAME
        pos_constraint.link_name = _EFFECTOR_LINK
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [2 * _POSITION_TOLERANCE_M] * 3
        pos_constraint.constraint_region.primitives.append(primitive)
        target_pose = _pose_from(position, orientation_quat)
        pos_constraint.constraint_region.primitive_poses.append(target_pose)
        pos_constraint.weight = 1.0
        constraints.position_constraints.append(pos_constraint)

        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = _BASE_FRAME
        orient_constraint.link_name = _EFFECTOR_LINK
        orient_constraint.orientation = target_pose.orientation
        orient_constraint.absolute_x_axis_tolerance = _ORIENTATION_TOLERANCE_RAD
        orient_constraint.absolute_y_axis_tolerance = _ORIENTATION_TOLERANCE_RAD
        orient_constraint.absolute_z_axis_tolerance = _ORIENTATION_TOLERANCE_RAD
        orient_constraint.weight = 1.0
        constraints.orientation_constraints.append(orient_constraint)

        req.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False

        if not self._move_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('/move_action server unavailable')
            return False

        send_future = self._move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by move_group')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result is None:
            self.get_logger().warn('No result from move_group')
            return False

        error_code = result.result.error_code.val
        if error_code != 1:  # moveit_msgs/MoveItErrorCodes.SUCCESS
            self.get_logger().warn(f'Motion failed (error_code={error_code}) -- skipping this pose')
            return False
        return True

    def run_tour(self):
        camera_pos = self._lookup_camera_position()
        if camera_pos is None:
            self.get_logger().error('Aborting: no camera position available')
            return

        poses = _build_poses(camera_pos)
        self.get_logger().info(
            f'Starting calibration pose tour: {len(poses)} poses, '
            f'{_DWELL_S:.0f}s dwell each -- take your sample in rqt during each dwell')

        for i, (anchor, variant, label) in enumerate(poses, start=1):
            quat = _look_at_quaternion(anchor, camera_pos, variant)
            self.get_logger().info(
                f'[{i}/{len(poses)}] Moving to anchor={anchor} orientation={label}...')
            ok = self._send_pose_goal(anchor, quat)
            if not ok:
                self.get_logger().warn(f'[{i}/{len(poses)}] Skipped (unreachable/filtered)')
                continue

            self.get_logger().info(
                f'[{i}/{len(poses)}] Pose reached -- take your sample now '
                f'({_DWELL_S:.0f}s)')
            time.sleep(_DWELL_S)

        self.get_logger().info('Tour complete.')


def _pose_from(position, orientation_quat):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = position
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = orientation_quat
    return pose


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationPoseTourNode()
    try:
        node.run_tour()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
