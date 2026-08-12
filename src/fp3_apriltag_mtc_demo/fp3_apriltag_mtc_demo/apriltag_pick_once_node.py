import cv2
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, Vector3
from sensor_msgs.msg import CameraInfo
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs  # noqa: F401  (registers PoseStamped support on Buffer.transform)

from franka_demo_interfaces.action import PickObject


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


# Minimal client: detect an AprilTag, solvePnP its 3D pose, transform it into
# fp3_link0, and hand TWO grasp candidates to fp3_moveit_server's MTC-based
# pick_object action -- candidate 0 with a forced straight-down orientation
# (usually the one that's actually reachable, see pick_place_node.cpp's
# ComputeIK), candidate 1 with the tag's own raw orientation (kept as a
# fallback candidate, in case straight-down isn't actually right for this
# tag/object). pick_object itself now owns the ENTIRE rest of the job: open
# the gripper, approach + grasp + attach + retreat, then move to its
# configured place pose, detach, and open the gripper again to release --
# all via the real franka_gripper actions, never through MoveIt. This node
# does nothing but detect, transform, and hand off a single goal.
class AprilTagPickOnceNode(Node):

    # Retry-with-rescan triggers on exactly one condition: the gripper
    # closed but pick_place_node's width check (closeGripper() in
    # pick_place_node.cpp) found the final width didn't match the expected
    # object -- i.e. the gripper closed on nothing or missed. That's the one
    # failure a fresh scan can plausibly fix (the object may have shifted
    # slightly, or the first read was a little off). It's identified as the
    # last feedback stage still being 'grasping' when the result comes back
    # (pick_place_node aborts immediately on grasp failure, before ever
    # publishing 'attaching'). Every earlier failure (filtering, approach/IK)
    # means the candidate pose itself was unreachable/rejected -- rescanning
    # gives essentially the same pose again and just repeats the same
    # failure (observed live: a tag sitting just past filter.max_reach kept
    # failing 'filtering' 3 times in a row for no benefit). Those are
    # reported once and NOT retried, same as any post-grasp failure.
    RETRY_STAGE = 'grasping'

    def __init__(self):
        super().__init__('apriltag_pick_once_node')

        self.tag_size = self.declare_parameter('tag_size', 0.04).value
        self.target_tag_id = self.declare_parameter('target_tag_id', 0).value
        self.robot_frame = self.declare_parameter('robot_frame', 'fp3_link0').value
        self.detections_topic = self.declare_parameter('detections_topic', '/detections').value
        self.camera_info_topic = self.declare_parameter(
            'camera_info_topic', '/camera/camera/color/camera_info').value
        self.tf_timeout_sec = self.declare_parameter('tf_timeout_sec', 2.0).value
        self.search_timeout_sec = self.declare_parameter('search_timeout_sec', 30.0).value

        # PickObject.Goal needs an object id and a bounding box for the
        # attach-to-planning-scene step; AprilTag detection alone gives
        # neither. TO ADJUST for your actual object.
        self.object_id = self.declare_parameter('object_id', 'apriltag_object').value
        self.object_dimensions_xyz = self.declare_parameter(
            'object_dimensions_xyz', [0.03, 0.03, 0.03]).value

        # pick_place_node already verifies the grasp against the expected
        # object width (franka_gripper's own epsilon check, plus an
        # explicit width comparison on top -- see pick_place_node.cpp's
        # closeGripper()) and reports success/failure via pick_object's
        # result. On failure, re-scan for the tag and retry the whole
        # sequence from scratch (position may have shifted, or the first
        # read was simply off), bounded by max_attempts so a persistently
        # failing grasp doesn't loop forever.
        self.max_attempts = self.declare_parameter('max_attempts', 3).value
        self._attempt = 0
        self._last_stage = None

        self._camera_info = None
        self._done = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, self.camera_info_topic, self._camera_info_cb, 10)
        self.create_subscription(
            AprilTagDetectionArray, self.detections_topic, self._detections_cb, 10)

        self._pick_client = ActionClient(self, PickObject, 'pick_object')

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

        # Deliberately not claiming self._done here -- see
        # fp3_apriltag_demo's apriltag_move_once_node for the full rationale
        # (camera/apriltag pipeline runs independently of the arm stack, so
        # a detection can arrive before fp3_link0's TF exists on a fresh
        # launch). _process() only claims _done once it has actually sent
        # the pick goal.
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

        # Candidate 0: forced straight-down orientation. 180-degree rotation
        # about X maps the TCP's local +Z (the approach axis, per
        # pick_place_node.cpp's convention) to world -Z. Usually the one
        # that's actually IK-reachable -- see pick_place_node.cpp.
        pose_down = PoseStamped()
        pose_down.header = pose_robot.header
        pose_down.pose.position = pose_robot.pose.position
        pose_down.pose.orientation.x = 1.0
        pose_down.pose.orientation.y = 0.0
        pose_down.pose.orientation.z = 0.0
        pose_down.pose.orientation.w = 0.0

        # Candidate 1: the tag's own raw orientation, kept as a fallback in
        # case straight-down isn't actually right for this tag/object.
        pose_native = pose_robot

        self._done = True
        self._search_timer.cancel()
        self._attempt += 1
        self.get_logger().info(
            f"Attempt {self._attempt}/{self.max_attempts}: tag {self.target_tag_id} pose in "
            f"'{self.robot_frame}': ({pose_robot.pose.position.x:.3f}, "
            f"{pose_robot.pose.position.y:.3f}, {pose_robot.pose.position.z:.3f}), "
            f"sending 2 candidates (forced straight-down, tag-native orientation)")
        self._send_pick_goal([pose_down, pose_native])

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

        # NOT SOLVEPNP_IPPE_SQUARE -- see fp3_apriltag_demo's
        # apriltag_move_once_node for the full writeup: it hardcodes an
        # internal corner-order assumption that doesn't match AprilTag's
        # native order and silently returns a garbage pose. ITERATIVE
        # verified live: 0.24 px mean reprojection error vs IPPE's 135 px.
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

    # ---- pick_object: MTC-based open + approach + grasp + attach +
    # retreat + place + detach + release, entirely on the server side ----

    def _send_pick_goal(self, grasp_candidates):
        self.get_logger().info("Waiting for the pick_object action server...")
        self._pick_client.wait_for_server()

        goal = PickObject.Goal()
        goal.grasp_candidates = grasp_candidates
        goal.object_id = self.object_id
        goal.object_dimensions = Vector3(
            x=float(self.object_dimensions_xyz[0]),
            y=float(self.object_dimensions_xyz[1]),
            z=float(self.object_dimensions_xyz[2]))

        self.get_logger().info(f"Sending pick_object goal (object_id='{self.object_id}')...")
        future = self._pick_client.send_goal_async(goal, feedback_callback=self._pick_feedback_cb)
        future.add_done_callback(self._pick_goal_response_cb)

    def _pick_feedback_cb(self, feedback_msg):
        self._last_stage = feedback_msg.feedback.current_stage
        self.get_logger().info(f'stage: {self._last_stage}')

    def _pick_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('pick_object goal rejected by fp3_moveit_server')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted, executing...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._pick_result_cb)

    def _pick_result_cb(self, future):
        result = future.result().result
        status = 'success' if result.success else 'failure'
        self.get_logger().info(
            f'pick_object result: {status} - {result.message} '
            f'(used_pose_index={result.used_pose_index})')

        if result.success:
            rclpy.shutdown()
            return

        if self._last_stage != self.RETRY_STAGE:
            if self._last_stage in ('attaching', 'retreating', 'placing', 'detaching', 'releasing'):
                self.get_logger().error(
                    f"Failure happened after a successful grasp (last stage: "
                    f"'{self._last_stage}') -- the object may still be attached/in the gripper. "
                    f"NOT retrying: re-scanning and re-approaching now would mean grasping again "
                    f"with an object already held.")
            else:
                self.get_logger().error(
                    f"Failure at stage '{self._last_stage}', before any grasp was attempted -- "
                    f"the candidate pose itself was unreachable/rejected. NOT retrying: "
                    f"re-scanning would give essentially the same pose and just repeat the same "
                    f"failure.")
            rclpy.shutdown()
            return

        if self._attempt >= self.max_attempts:
            self.get_logger().error(
                f'Giving up after {self._attempt} attempt(s) (max_attempts={self.max_attempts})')
            rclpy.shutdown()
            return

        self.get_logger().warn(
            f'Grasp closed but width check failed (attempt {self._attempt}/'
            f'{self.max_attempts}), re-scanning for the tag and retrying...')
        self._done = False
        self._search_timer.reset()


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagPickOnceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
