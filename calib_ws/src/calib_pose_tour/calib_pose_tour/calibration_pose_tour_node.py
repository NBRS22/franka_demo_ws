import math
import os
import time

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from scipy.spatial.transform import Rotation

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
from shape_msgs.msg import SolidPrimitive

# easy_handeye2's own convention -- same path this project's other tools
# (evaluate_calibration.launch.py's handeye_tf_publisher, the manual
# "cp ... _good.calib" backups) already read/write.
_CALIBRATIONS_DIR = os.path.expanduser('~/.ros2/easy_handeye2/calibrations')


# Anchors are computed at STARTUP from a PREVIOUSLY SAVED calibration file,
# NOT from live TF. This replaces an earlier version that read
# fp3_link0 -> camera_color_optical_frame live via TF -- which sounds more
# "correct" but is actually broken for how this node is ever used: it only
# ever runs as part of calib_bringup.launch.py, i.e. during an ACTIVE
# calibration-taking session, where fp3_link0 -> camera_color_optical_frame
# has NO real transform yet (that's the unknown being solved). What was
# actually being published there the whole time is easy_handeye2's own
# built-in placeholder for eye-on-base mode -- literally hardcoded as
# `--x 1 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1` in its calibrate.launch.py
# (node name 'dummy_publisher') -- not this project's handeye_tf_publisher,
# which calib_bringup.launch.py never launches at all. So every anchor this
# node has ever computed via TF was built relative to a fake camera at
# (1,0,0) with identity rotation, not the real one -- almost certainly the
# actual root cause behind the ~1-3cm pick offset on the 12cm-tag
# calibration this tour produced, not (only) the rotation-diversity or
# reach/FOV issues fixed earlier the same session.
#
# Using the PREVIOUS calibration as a rough bootstrap for where to explore
# is the standard, correct approach here (same idea as camera_calibration
# tools reusing a prior intrinsics estimate to guide new sample poses) --
# it only affects which poses get visited, not the math of the new
# calibration being computed from the fresh samples taken at those poses.
def _read_calibration_pose(calibration_name):
    path = os.path.join(_CALIBRATIONS_DIR, f'{calibration_name}.calib')
    with open(path) as f:
        data = yaml.safe_load(f)
    t = data['transform']['translation']
    r = data['transform']['rotation']
    camera_pos = (t['x'], t['y'], t['z'])
    camera_rot = Rotation.from_quat([r['x'], r['y'], r['z'], r['w']]).as_matrix()
    return camera_pos, camera_rot


# Anchors are computed at RUNTIME from the live camera pose (cf.
# _build_anchor_positions below) -- NOT as fixed fp3_link0-frame points.
#
# History of two failed approaches, kept here so the next attempt doesn't
# repeat either mistake:
#
# 1) Fixed DISTANCES-FROM-CAMERA along the camera->fp3_link0-ORIGIN ray
#    (0.30-0.55m). Silently assumed a specific camera->base distance
#    (~1.05-1.1m at the time); after the camera mount was nudged, the real
#    distance grew to ~1.28m and 4 of 5 anchors ended up AT OR BEYOND fp3's
#    ~0.85m max reach -- most tour poses were silently skipped
#    ("unreachable/filtered"), so the calibration this fed came from far
#    fewer, far less diverse samples than intended.
#
# 2) Retuned to target REACH FROM THE BASE instead (fixing #1), still along
#    the camera->origin ray. This exposed a second, separate problem:
#    measured live, this camera's real optical axis points ~34deg away
#    from "toward fp3_link0's origin" (it looks down at ~62deg from
#    horizontal, well past the robot base). Pushing anchors further out
#    along the WRONG ray (to reach fp3's workspace) pushed them well
#    outside the camera's actual field of view -- "most poses not visible"
#    reported live, confirming it wasn't a fluke.
#
# Root fix: build anchors in the CAMERA'S OWN frame (depth + horizontal/
# vertical angular offset from its real orientation, not the vector toward
# fp3_link0's origin) -- guarantees anchors stay inside the camera's field
# of view by construction, AS LONG AS the (camera_pos, camera_rot) fed in
# are actually correct (cf. _read_calibration_pose -- an earlier version of
# this same fix used live TF instead, which turned out to read
# easy_handeye2's own dummy placeholder during an active session, not the
# real camera; every offline-scanned candidate below was invalidated by
# that bug until it was found and fixed the same session).
#
# _ANCHOR_CAMERA_OFFSETS below are NOT an offline geometric estimate --
# each one was moved to LIVE on the real arm (fp3_hand, gripper holding the
# calibration tag) and confirmed via /detections that the tag was actually
# seen, using the corrected (camera_pos, camera_rot) read from the
# fp3_link0_d455_camera_color_optical_frame_001_tag12cm_good.calib
# bootstrap. 11 of 12 probed candidates were visible (the 1 failure was an
# out-of-reach motion goal, not a field-of-view miss) -- the field of view
# at this camera mount is comfortably wide once given the right pose to
# begin with. Picked for spread across horizontal/vertical/depth from
# that confirmed-visible set, not just the first ones that worked.
# run_tour() ALSO still re-verifies live detection per pose at runtime (cf.
# _wait_for_tag_visible) as a second, independent safety net -- e.g. if the
# camera or tag mount shifts again before the next full tour.
_ANCHOR_CAMERA_OFFSETS = [
    # (depth_from_camera_m, horizontal_deg, vertical_deg) -- ROS optical
    # convention: local +X = image right, local -Y = image up.
    (0.75, -15.0, 15.0),
    (0.65, -25.0, 5.0),
    (0.65, 5.0, 15.0),
    (0.55, -15.0, 15.0),
    (0.65, -25.0, 15.0),
    (0.65, -5.0, 10.0),
    (0.70, -20.0, 10.0),
]


def _normalize(v):
    norm = sum(c * c for c in v) ** 0.5
    return tuple(c / norm for c in v)


def _build_anchor_positions(camera_pos, camera_rot_matrix):
    # ROS optical-frame convention (camera_color_optical_frame, the frame
    # this is looked up in -- cf. _CAMERA_FRAME): column 0 = local X =
    # image right, column 1 = local Y = image down, column 2 = local Z =
    # forward (viewing direction).
    cam_right = tuple(camera_rot_matrix[i][0] for i in range(3))
    cam_up = tuple(-camera_rot_matrix[i][1] for i in range(3))
    cam_fwd = tuple(camera_rot_matrix[i][2] for i in range(3))

    anchors = []
    for depth, h_deg, v_deg in _ANCHOR_CAMERA_OFFSETS:
        th = math.tan(math.radians(h_deg))
        tv = math.tan(math.radians(v_deg))
        direction = _normalize(tuple(
            cam_fwd[k] + cam_right[k] * th + cam_up[k] * tv for k in range(3)))
        anchors.append(tuple(camera_pos[k] + direction[k] * depth for k in range(3)))
    return anchors


# Orientation variants per anchor, applied on top of a "look at the camera"
# baseline (local +Z, this codebase's approach-axis convention -- cf.
# pick_place_node.cpp -- pointed from the anchor toward the camera).
# Perturbations are intrinsic (local-frame) rotations on top of that
# baseline.
#
# Previous set (baseline + roll/pitch/yaw at a fixed 25deg, one axis at a
# time) under-covered the AX=XB hand-eye math's real requirement -- cf.
# https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-calibration-extrinsic-eye-to-hand.html,
# which calls for orientations spanning "the surface of a half-sphere", not
# a narrow cluster around one viewing direction. Two compounding issues with
# the old set: (1) 'roll' rotates about the approach axis itself (the
# direction pointing at the camera) -- it changes in-image rotation only,
# NOT the actual viewing angle/tilt, so only 'pitch'/'yaw' were doing any
# real tilt-diversity work; (2) even those two topped out at 25deg, each
# applied alone -- never combined, never larger. All ~20 samples from a
# tour ended up clustered within a narrow cone around "facing the camera",
# which is the classic ill-conditioned case for solving the translation
# part of X in Tsai/Park-style hand-eye calibration (rotation looks fine,
# translation/camera-position accuracy suffers) -- matches the ~1-3cm
# horizontal pick offset measured on the last calibration built from this
# tour despite an OK-looking "Maximum divergence" reading.
#
# Widened to 45deg (kept under the 60deg AprilTag-detection-reliability
# ceiling, cf. handeye_tf_publisher/README.md) on pitch/yaw individually,
# added negative-direction variants (was one-sided before), and added two
# combined pitch+yaw ("diagonal") variants -- these are what actually start
# approximating a half-sphere spread rather than a single cone. 'roll'
# kept at one variant: still useful for the rotation-only part of the
# solve, just not relied on for tilt diversity.
_ORIENTATION_VARIANTS = [
    ('baseline', None),
    ('pitch+45', [('x', 45.0)]),
    ('pitch-45', [('x', -45.0)]),
    ('yaw+45', [('y', 45.0)]),
    ('yaw-45', [('y', -45.0)]),
    # 35+35 (not 45+45) here specifically: two combined 45deg axis rotations
    # compound to a 60deg REAL tilt (cos(tilt) = cos(45)*cos(45) = 0.5) --
    # exactly the documented reliability ceiling, no margin. 35+35 lands at
    # ~48deg real tilt instead, verified numerically rather than assumed
    # from the per-axis angle alone.
    ('pitch+35_yaw+35', [('x', 35.0), ('y', 35.0)]),
    ('pitch-35_yaw-35', [('x', -35.0), ('y', -35.0)]),
    ('roll+45', [('z', 45.0)]),   # about the local approach axis -- in-image rotation only
]

_PLANNING_GROUP = 'fp3_arm'
_EFFECTOR_LINK = 'fp3_hand'  # matches robot_effector_frame used for calibration
_BASE_FRAME = 'fp3_link0'
# camera_color_optical_frame, not camera_link: need the camera's real
# ORIENTATION for _build_anchor_positions' camera-relative basis, and only
# the *_optical_frame follows the ROS optical convention (Z=forward) this
# codebase already relies on elsewhere (cf. pick_place_node.cpp's approach
# axis, root CLAUDE.md "Convention de coordonnées"). camera_link uses the
# physical-mount convention instead (X=forward for RealSense), which would
# silently misinterpret _ANCHOR_CAMERA_OFFSETS. Position differs from
# camera_link by only a few cm -- negligible at these anchor distances.
# Reading (not publishing) this frame has none of the TF-parent-conflict
# risk that made handeye_tf_publisher avoid it for PUBLISHING (cf. that
# package's own README) -- that concern is about adding a second parent to
# a frame, not about looking one up.
_CAMERA_FRAME = 'camera_color_optical_frame'

_POSITION_TOLERANCE_M = 0.01
_ORIENTATION_TOLERANCE_RAD = 0.15
_VELOCITY_SCALING = 0.1
_ACCELERATION_SCALING = 0.1
_PLANNING_TIME_S = 5.0
_DWELL_S = 4.0

# Tag id calib_pose_tour verifies is actually visible before dwelling --
# matches the fixed convention used throughout this session (tag36h11:0,
# same id calib_sample_guard defaults to). Not a declared parameter: this
# node has none so far, kept consistent.
_TARGET_TAG_ID = 0
# How long to wait for a detection of _TARGET_TAG_ID after reaching a pose
# before giving up and skipping it -- cf. run_tour/_wait_for_tag_visible.
# Short on purpose: this is what makes a geometry mistake in
# _ANCHOR_CAMERA_OFFSETS cheap (a few wasted seconds, not a wasted
# _DWELL_S-long stop with nothing to sample) instead of expensive.
_VISIBILITY_TIMEOUT_S = 3.0


def _look_at_quaternion(from_xyz, to_xyz, axis_deg_list=None):
    """axis_deg_list: None, or a list of (axis, deg) intrinsic rotations
    applied in order on top of the look-at baseline -- a list (not just a
    single pair) so orientation variants can combine axes (cf.
    _ORIENTATION_VARIANTS' 'pitch+45_yaw+45' entries) instead of only ever
    rotating about one axis at a time.
    """
    direction = [to_xyz[i] - from_xyz[i] for i in range(3)]
    norm = sum(c * c for c in direction) ** 0.5
    direction = [c / norm for c in direction]

    rotation, _ = Rotation.align_vectors([direction], [[0.0, 0.0, 1.0]])

    for axis, deg in (axis_deg_list or []):
        rotation = rotation * Rotation.from_euler(axis, deg, degrees=True)

    x, y, z, w = rotation.as_quat()
    return x, y, z, w


def _build_poses(camera_pos, camera_rot_matrix):
    anchors = _build_anchor_positions(camera_pos, camera_rot_matrix)
    poses = []
    for anchor in anchors:
        for label, variant in _ORIENTATION_VARIANTS:
            poses.append((anchor, variant, label))
    return poses


class CalibrationPoseTourNode(Node):

    def __init__(self):
        super().__init__('calibration_pose_tour_node')

        # Name of the PREVIOUS saved calibration to bootstrap anchor
        # placement from (cf. _read_calibration_pose) -- calib_bringup.launch.py
        # passes the same 'calibration_name' it also gives
        # easy_handeye2_calibrate, i.e. this reads that file BEFORE the
        # session's own "Save" overwrites it.
        self.declare_parameter('calibration_name', '')
        self._calibration_name = self.get_parameter('calibration_name').value

        # No spin_thread=True on the tf listener setup needed here since this
        # node no longer reads any live TF -- kept out entirely (cf.
        # _read_calibration_pose). MoveGroup action results and /detections
        # are pumped via rclpy.spin_once(self, ...) in the wait loops below.
        self._move_client = ActionClient(self, MoveGroup, '/move_action')

        self._latest_detection_ids = set()
        self.create_subscription(
            AprilTagDetectionArray, '/detections', self._on_detections, 10)

    def _on_detections(self, msg):
        self._latest_detection_ids = {d.id for d in msg.detections}

    def _wait_for_tag_visible(self, timeout_s):
        deadline = self.get_clock().now() + Duration(seconds=timeout_s)
        while self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if _TARGET_TAG_ID in self._latest_detection_ids:
                return True
        return False

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
        if not self._calibration_name:
            self.get_logger().error(
                "Aborting: 'calibration_name' parameter is empty -- can't find a "
                'previous calibration file to bootstrap anchor placement from '
                '(cf. _read_calibration_pose docstring for why this replaced a '
                'live TF lookup).')
            return
        try:
            camera_pos, camera_rot = _read_calibration_pose(self._calibration_name)
        except (OSError, KeyError, yaml.YAMLError) as exc:
            self.get_logger().error(
                f"Aborting: could not read previous calibration "
                f"'{self._calibration_name}' from {_CALIBRATIONS_DIR}: {exc}")
            return
        self.get_logger().info(
            f"Bootstrapped from previous calibration '{self._calibration_name}': "
            f'camera_pos={tuple(round(c, 3) for c in camera_pos)}')

        poses = _build_poses(camera_pos, camera_rot)
        self.get_logger().info(
            f'Starting calibration pose tour: {len(poses)} poses -- each pose is '
            f'verified visible (tag {_TARGET_TAG_ID}, up to {_VISIBILITY_TIMEOUT_S:.0f}s) '
            f'before a {_DWELL_S:.0f}s dwell to take your sample in rqt')

        n_visible = 0
        for i, (anchor, variant, label) in enumerate(poses, start=1):
            quat = _look_at_quaternion(anchor, camera_pos, variant)
            self.get_logger().info(
                f'[{i}/{len(poses)}] Moving to anchor={anchor} orientation={label}...')
            ok = self._send_pose_goal(anchor, quat)
            if not ok:
                self.get_logger().warn(f'[{i}/{len(poses)}] Skipped (unreachable/filtered)')
                continue

            if not self._wait_for_tag_visible(_VISIBILITY_TIMEOUT_S):
                self.get_logger().warn(
                    f'[{i}/{len(poses)}] Tag {_TARGET_TAG_ID} not visible after '
                    f'{_VISIBILITY_TIMEOUT_S:.0f}s -- skipping, no dwell')
                continue

            n_visible += 1
            self.get_logger().info(
                f'[{i}/{len(poses)}] Pose reached, tag visible -- take your sample now '
                f'({_DWELL_S:.0f}s)')
            time.sleep(_DWELL_S)

        self.get_logger().info(
            f'Tour complete: tag was visible at {n_visible}/{len(poses)} attempted poses.')


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
