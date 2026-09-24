import cv2
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import CameraInfo
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs  # noqa: F401  (registers PoseStamped support on Buffer.transform)

from franka_msgs.action import Move
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
from shape_msgs.msg import SolidPrimitive

PLANNING_GROUP = 'fp3_arm'
EFFECTOR_LINK = 'fp3_hand_tcp'
POSITION_TOLERANCE_M = 0.005
ORIENTATION_TOLERANCE_RAD = 0.15
VELOCITY_SCALING = 0.15
ACCELERATION_SCALING = 0.15
PLANNING_TIME_S = 5.0
GRIPPER_OPEN_SPEED = 0.1


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
# handeye_tf_publisher, no calibration done here), and moves fp3_hand_tcp
# directly to that pose -- position AND orientation, gripper kept OPEN the
# whole time, no grasp/lift/attach at all.
#
# Deliberately NOT a grasp test (that used to be this node's design, via
# mtc_pick -- calib_ws's copy dropped that entirely, franka_demo_ws's
# original is untouched and still does the full grasp+lift). A grasp
# conflates two independent questions -- "did the calibration place the
# target correctly" and "did the mechanical grasp succeed" (which can fail
# for reasons having nothing to do with calibration: cube slipping, grasp
# width, etc). Stopping with the gripper open instead lets you compare
# fp3_hand_tcp against the tag's own live TF frame (tag36h11:0, published
# continuously by apriltag_node for as long as the tag stays visible)
# directly in RViz, or numerically:
#   ros2 run tf2_ros tf2_echo tag36h11:0 fp3_hand_tcp
# A well-calibrated setup should show translation ~= [0,0,0] once the arm
# has arrived and settled.
#
# /detections (apriltag_msgs/AprilTagDetectionArray) only carries 2D pixel
# corners + a homography, no 3D pose -- apriltag_msgs deliberately leaves
# pose estimation to the consumer, since it needs a physical tag size the
# detector itself doesn't know.
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
        # Default true: flip the tag's own orientation 180deg about its local
        # X (cf. _process below) rather than using it as-is -- a tag facing
        # the camera has its normal pointing the way the gripper is coming
        # FROM, not a reachable approach direction. Set false only if the
        # tag is mounted such that its raw orientation IS already a valid
        # approach direction (e.g. on a vertical face, not lying flat).
        self.flip_tag_orientation = self.declare_parameter('flip_tag_orientation', True).value
        # Default false: keeps the existing flip_tag_orientation behaviour
        # (approach direction still derived from the tag's own real tilt).
        # Set true to ignore the tag's orientation entirely and force a pure
        # vertical descent instead -- fp3_hand_tcp's local +Z (this
        # codebase's approach axis, cf. pick_place_node.cpp) pointed straight
        # along world -Z, same fixed quaternion (1,0,0,0) the old
        # grasp-based version of this node used unconditionally before
        # flip_tag_orientation replaced it with a tilt-preserving flip (cf.
        # module docstring above). Takes priority over flip_tag_orientation
        # when both are set.
        self.force_top_down = self.declare_parameter('force_top_down', False).value
        # Test-only manual correction added to the target position, in
        # robot_frame (fp3_link0) axes, meters. Default 0 = no correction.
        self.offset_x = self.declare_parameter('offset_x', 0.0).value
        self.offset_y = self.declare_parameter('offset_y', 0.0).value
        self.offset_z = self.declare_parameter('offset_z', 0.0).value
        # Target gripper width before the move (meters). Default 0.08 = fully
        # open (original behaviour). Set near 0 to close the gripper instead --
        # e.g. to align its closed tip against a tag larger than the gripper's
        # own opening, combined with offset_z to hover above it rather than
        # descending onto the tag's own plane.
        self.gripper_width_m = self.declare_parameter('gripper_width_m', 0.08).value

        self._camera_info = None
        self._done = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, self.camera_info_topic, self._camera_info_cb, 10)
        self.create_subscription(
            AprilTagDetectionArray, self.detections_topic, self._detections_cb, 10)

        self._move_client = ActionClient(self, MoveGroup, '/move_action')
        self._gripper_client = ActionClient(self, Move, '/franka_gripper/move')

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

        # Request the LATEST available transform rather than the one at the
        # detection's own capture stamp. Fine for eye-on-base (that chain is
        # entirely static, cf. handeye_tf_publisher), but load-bearing for
        # eye-in-hand: there, fp3_link0 -> fp3_hand is a live, joint-states-
        # driven transform, and its publish rate can lag behind the
        # detection timestamp enough to trip "Lookup would require
        # extrapolation into the future" -- a stamp of exactly zero is tf2's
        # documented meaning for "latest available", sidestepping that.
        pose_camera.header.stamp = Time().to_msg()

        try:
            pose_robot = self.tf_buffer.transform(
                pose_camera, self.robot_frame, timeout=Duration(seconds=self.tf_timeout_sec))
        except TransformException as ex:
            self.get_logger().warn(
                f"Could not transform tag pose from '{pose_camera.header.frame_id}' to "
                f"'{self.robot_frame}' yet, will retry on the next detection: {ex}")
            return

        # Logged unconditionally (even when flip_tag_orientation modifies it below) --
        # a calibration-induced rotation bias would show up here numerically.
        native_q = pose_robot.pose.orientation
        roll_deg, pitch_deg, yaw_deg = _quaternion_to_euler_deg(
            native_q.x, native_q.y, native_q.z, native_q.w)
        self.get_logger().info(
            f"Tag {self.target_tag_id} native orientation in '{self.robot_frame}': "
            f"roll={roll_deg:.1f}deg pitch={pitch_deg:.1f}deg yaw={yaw_deg:.1f}deg "
            f"(quaternion x={native_q.x:.3f} y={native_q.y:.3f} z={native_q.z:.3f} w={native_q.w:.3f})")

        if self.force_top_down:
            # Ignore the tag's own orientation entirely -- fp3_hand_tcp's
            # local +Z pointed straight along world -Z, position still taken
            # from the tag (translation only). Same fixed quaternion as the
            # flip below (1,0,0,0), but standing in for the orientation
            # outright instead of composing with the tag's real tilt --
            # exactly the old grasp-based node's unconditional behaviour,
            # cf. flip_tag_orientation's own comment above.
            pose_robot.pose.orientation.x = 1.0
            pose_robot.pose.orientation.y = 0.0
            pose_robot.pose.orientation.z = 0.0
            pose_robot.pose.orientation.w = 0.0
        elif self.flip_tag_orientation:
            # A tag lying flat facing the camera has its own local +Z
            # (surface normal) pointing UP, toward the camera -- but a
            # gripper approaching it can only ever come from ABOVE, pointing
            # DOWN, never from below. Comparing fp3_hand_tcp's orientation
            # directly against that raw tag orientation is therefore
            # comparing against a target that's often kinematically
            # unreachable (confirmed live: OMPL "Unable to sample any valid
            # states for goal tree" with the tag's raw orientation).
            #
            # Fix: compose the tag's own orientation with a 180-degree
            # rotation about ITS OWN local X axis (post-multiply, i.e.
            # expressed in the tag's already-rotated frame) instead of
            # replacing it outright with a fixed world-frame constant. This
            # keeps whatever real tilt the tag has (roll/pitch don't get
            # thrown away) while flipping its normal to point the only way a
            # real approach can: opposite the tag's face, same side the
            # gripper is coming from. Matches this codebase's approach-axis
            # convention (TCP local +Z, cf. pick_place_node.cpp) -- the flip
            # itself is exactly the same 180-about-X quaternion (1,0,0,0)
            # the old fixed-constant version used, just now composed with
            # the tag's real orientation instead of standing in for it.
            nx, ny, nz, nw = native_q.x, native_q.y, native_q.z, native_q.w
            # Hamilton product q_tag (x,y,z,w) * q_flip (1,0,0,0), simplified:
            fx, fy, fz, fw = nw, nz, -ny, -nx
            pose_robot.pose.orientation.x = fx
            pose_robot.pose.orientation.y = fy
            pose_robot.pose.orientation.z = fz
            pose_robot.pose.orientation.w = fw

        if self.offset_x or self.offset_y or self.offset_z:
            pose_robot.pose.position.x += self.offset_x
            pose_robot.pose.position.y += self.offset_y
            pose_robot.pose.position.z += self.offset_z
            self.get_logger().warn(
                f"Manual offset applied in '{self.robot_frame}': "
                f"dx={self.offset_x*1000:+.0f}mm dy={self.offset_y*1000:+.0f}mm "
                f"dz={self.offset_z*1000:+.0f}mm")

        self._done = True
        self._search_timer.cancel()
        self.get_logger().info(
            f"Tag {self.target_tag_id} pose in '{self.robot_frame}': "
            f"({pose_robot.pose.position.x:.3f}, {pose_robot.pose.position.y:.3f}, "
            f"{pose_robot.pose.position.z:.3f}) "
            f"orientation={'forced top-down (tag tilt ignored)' if self.force_top_down else 'tag-native flipped 180deg about local X' if self.flip_tag_orientation else 'tag-native as-is'}")
        self._open_gripper_then_move(pose_robot)

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

    # ---- open gripper, then move fp3_hand_tcp directly to the tag pose
    # (no grasp/lift/attach) ----

    def _open_gripper_then_move(self, pose_robot):
        self.get_logger().info(
            f'Setting gripper width to {self.gripper_width_m}m before moving...')
        if not self._gripper_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn('/franka_gripper/move unavailable -- moving without adjusting gripper first')
            self._move_to_pose(pose_robot)
            return

        goal = Move.Goal()
        goal.width = self.gripper_width_m
        goal.speed = GRIPPER_OPEN_SPEED
        future = self._gripper_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._on_gripper_open_response(f, pose_robot))

    def _on_gripper_open_response(self, future, pose_robot):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('Gripper open goal rejected -- moving anyway')
            self._move_to_pose(pose_robot)
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._move_to_pose(pose_robot))

    def _move_to_pose(self, pose_robot):
        constraints = Constraints()

        pos_c = PositionConstraint()
        pos_c.header.frame_id = self.robot_frame
        pos_c.link_name = EFFECTOR_LINK
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [POSITION_TOLERANCE_M]
        pos_c.constraint_region.primitives.append(primitive)
        target_pose = Pose()
        target_pose.position = pose_robot.pose.position
        target_pose.orientation = pose_robot.pose.orientation
        pos_c.constraint_region.primitive_poses.append(target_pose)
        pos_c.weight = 1.0
        constraints.position_constraints.append(pos_c)

        orient_c = OrientationConstraint()
        orient_c.header.frame_id = self.robot_frame
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
        req.num_planning_attempts = 10
        req.allowed_planning_time = PLANNING_TIME_S
        req.max_velocity_scaling_factor = VELOCITY_SCALING
        req.max_acceleration_scaling_factor = ACCELERATION_SCALING
        req.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False

        self.get_logger().info("Waiting for the /move_action server...")
        if not self._move_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('/move_action unavailable')
            rclpy.shutdown()
            return

        self.get_logger().info(f'Moving {EFFECTOR_LINK} to the tag pose (gripper stays open)...')
        future = self._move_client.send_goal_async(goal)
        future.add_done_callback(self._on_move_goal_response)

    def _on_move_goal_response(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Move goal rejected (likely unreachable)')
            rclpy.shutdown()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_move_result)

    def _on_move_result(self, future):
        result = future.result()
        error_code = result.result.error_code.val if result else None
        if error_code == 1:  # moveit_msgs/MoveItErrorCodes.SUCCESS
            self.get_logger().info(
                f'CALIBRATION CHECK: arrived. Compare "{EFFECTOR_LINK}" against '
                f'"tag36h11:{self.target_tag_id}" (published live by apriltag_node as long as '
                f'the tag stays visible) in RViz, or numerically:\n'
                f'  ros2 run tf2_ros tf2_echo tag36h11:{self.target_tag_id} {EFFECTOR_LINK}\n'
                f'Translation close to [0,0,0] once settled = calibration checks out at this point.')
        else:
            self.get_logger().error(
                f'CALIBRATION CHECK: move failed (error_code={error_code}) -- either the '
                f'calibration puts the target somewhere unreachable, or the pose is genuinely '
                f'kinematically infeasible for an unrelated reason.')
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
