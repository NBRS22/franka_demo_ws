"""
Derives the D455 eye-on-base calibration (fp3_link0 -> D455's tracking_base_frame)
from the D405 eye-in-hand calibration (already trusted, cf. calib_eye_in_hand)
instead of running easy_handeye2's own AX=XB solver on the D455 at all --
motivation: ~10 direct eye-on-base recalibration attempts on the D455 never
converged to a physically correct result (cf. fp3_apriltag_demo's circularity
finding, which invalidated the earlier "looks accurate" read on that
calibration).

Method: both cameras observe the SAME static tag at once. The D405 + live FK
gives an independent measurement of the tag's pose in fp3_link0 (independent
because it never depends on the D455 calibration under derivation):

    fp3_link0 -> tag  =  (fp3_link0 -> fp3_hand)         [FK, live]
                       x  (fp3_hand -> D405 optical)      [calib_eye_in_hand's saved .calib]
                       x  (D405 optical -> tag)            [D405 detection, live]

Inverting the D455's own simultaneous detection of the same tag then gives
the D455 extrinsic directly, no solver needed:

    fp3_link0 -> D455 optical  =  (fp3_link0 -> tag) x (D455 optical -> tag)^-1

One combined measurement is noisy (detection jitter on both cameras, at a
single instant) -- take_sample/save_calibration below collect several
samples (arm held still, tag visible to both cameras, ideally at different
robot poses so detection noise doesn't correlate) and average, mirroring the
role easy_handeye2's own multi-sample AX=XB solve plays for a normal
calibration.

apriltag_node's own /tf broadcast (tag<family>:<id>) is unused here on
purpose: with both cameras' apriltag_node instances running at once, that
frame would get two different parents (one per camera) -- a genuine TF
conflict, not just cosmetic (cf. calib_eye_in_hand/CLAUDE.md's plan, and the
"two or more unconnected trees" bug hit testing camera<->robot static TFs the
same way). This node does its own solvePnP directly from /detections (2D
corners only, cf. fp3_apriltag_demo/apriltag_move_once_node.py's own
docstring on why apriltag_msgs never carries a 3D pose) and never looks up
either tag frame via TF.
"""
import os

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation

from apriltag_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import CameraInfo
from std_srvs.srv import Trigger
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from easy_handeye2.handeye_calibration import save_calibration
from easy_handeye2_msgs.msg import HandeyeCalibration, HandeyeCalibrationParameters


def _estimate_tag_pose_camera_frame(detection, tag_size_m, k, dist):
    half = tag_size_m / 2.0
    object_points = np.array([
        [-half, -half, 0.0], [half, -half, 0.0], [half, half, 0.0], [-half, half, 0.0],
    ], dtype=np.float64)
    image_points = np.array([[c.x, c.y] for c in detection.corners], dtype=np.float64)
    ok, rvec, tvec = cv2.solvePnP(object_points, image_points, k, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    r, _ = cv2.Rodrigues(rvec)
    return r, np.array([tvec[0, 0], tvec[1, 0], tvec[2, 0]])


def _to_matrix(translation, quat_xyzw):
    m = np.eye(4)
    m[:3, :3] = Rotation.from_quat(quat_xyzw).as_matrix()
    m[:3, 3] = translation
    return m


def _rt_to_matrix(r, t):
    m = np.eye(4)
    m[:3, :3] = r
    m[:3, 3] = t
    return m


class _CameraBuffer:
    """Latest CameraInfo + latest detection of one tag id, for one camera."""

    def __init__(self, node, camera_info_topic, detections_topic, target_tag_id):
        self.camera_info = None
        self.detection = None
        self.detection_frame_id = None
        self.detection_stamp = None
        self._target_tag_id = target_tag_id
        node.create_subscription(CameraInfo, camera_info_topic, self._ci_cb, 10)
        node.create_subscription(AprilTagDetectionArray, detections_topic, self._det_cb, 10)

    def _ci_cb(self, msg):
        self.camera_info = msg

    def _det_cb(self, msg):
        for d in msg.detections:
            if d.id == self._target_tag_id:
                self.detection = d
                self.detection_frame_id = msg.header.frame_id
                self.detection_stamp = Time.from_msg(msg.header.stamp)

    def estimate_pose(self, tag_size_m):
        if self.camera_info is None or self.detection is None:
            return None
        k = np.array(self.camera_info.k, dtype=np.float64).reshape(3, 3)
        dist = np.array(self.camera_info.d, dtype=np.float64)
        return _estimate_tag_pose_camera_frame(self.detection, tag_size_m, k, dist)

    def staleness_s(self, now):
        if self.detection_stamp is None:
            return float('inf')
        return (now - self.detection_stamp).nanoseconds / 1e9


class BridgeCalibrationNode(Node):

    def __init__(self):
        super().__init__('calib_bridge')

        self.target_tag_id = self.declare_parameter('target_tag_id', 0).value
        self.tag_size_m = self.declare_parameter('tag_size', 0.04).value
        self.robot_base_frame = self.declare_parameter('robot_base_frame', 'fp3_link0').value
        self.max_staleness_s = self.declare_parameter('max_detection_staleness_s', 0.5).value
        self.tf_timeout_s = self.declare_parameter('tf_timeout_sec', 2.0).value

        # Defaults match calib_bridge.launch.py's 'franka' namespace
        # convention (camera_namespace:=franka, camera_name:=d455/d405 ->
        # /franka/d455/..., /franka/d405/...) -- the launch file passes
        # these explicitly too, but keeping the defaults in sync means
        # `ros2 run calib_bridge bridge_calibration_node` alongside that same
        # launch file's cameras works without extra flags.
        self.d455_camera_info_topic = self.declare_parameter(
            'd455_camera_info_topic', '/franka/d455/color/camera_info').value
        self.d455_detections_topic = self.declare_parameter(
            'd455_detections_topic', '/franka/d455/detections').value
        self.d405_camera_info_topic = self.declare_parameter(
            'd405_camera_info_topic', '/franka/d405/color/camera_info').value
        self.d405_detections_topic = self.declare_parameter(
            'd405_detections_topic', '/franka/d405/detections').value

        d405_calibration_name = self.declare_parameter(
            'd405_calibration_name', 'fp3_hand_d405_camera_color_optical_frame_001').value
        d405_calib_dir = os.path.expanduser(self.declare_parameter(
            'd405_calib_dir', '~/.ros2/easy_handeye2/calibrations').value)
        self.output_calibration_name = self.declare_parameter(
            'output_calibration_name', 'fp3_link0_d455_camera_color_optical_frame_derived_001').value

        (self.d405_effector_frame, self.d405_tracking_base_frame,
         self.t_hand_d405) = self._load_d405_calibration(
            os.path.join(d405_calib_dir, f'{d405_calibration_name}.calib'))

        self.d455 = _CameraBuffer(
            self, self.d455_camera_info_topic, self.d455_detections_topic, self.target_tag_id)
        self.d405 = _CameraBuffer(
            self, self.d405_camera_info_topic, self.d405_detections_topic, self.target_tag_id)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._samples = []  # list of (R 3x3, t 3,) for fp3_link0 -> D455 tracking_base_frame

        self.create_service(Trigger, 'calib_bridge/take_sample', self._on_take_sample)
        self.create_service(Trigger, 'calib_bridge/save_calibration', self._on_save_calibration)

        self.get_logger().info(
            f"calib_bridge ready. D405 calib '{d405_calibration_name}' loaded "
            f"({self.d405_effector_frame} -> {self.d405_tracking_base_frame}). "
            f"Deriving '{self.output_calibration_name}' ({self.robot_base_frame} -> <D455 tracking frame>). "
            f"Call 'ros2 service call /calib_bridge/take_sample std_srvs/srv/Trigger {{}}' at each pose, "
            f"then 'ros2 service call /calib_bridge/save_calibration std_srvs/srv/Trigger {{}}' once done.")

    def _load_d405_calibration(self, calib_path):
        if not os.path.isfile(calib_path):
            self.get_logger().fatal(f'D405 calibration file not found: {calib_path}')
            raise SystemExit(1)
        with open(calib_path, 'r') as f:
            data = yaml.safe_load(f)
        try:
            parameters = data['parameters']
            if parameters.get('calibration_type') != 'eye_in_hand':
                self.get_logger().warn(
                    f"D405 calibration '{parameters.get('name')}' has calibration_type="
                    f"'{parameters.get('calibration_type')}', expected 'eye_in_hand' -- "
                    f"the frame this node reads (robot_effector_frame) may not mean what "
                    f"it expects for a different type.")
            effector_frame = parameters['robot_effector_frame']
            tracking_base_frame = parameters['tracking_base_frame']
            translation = data['transform']['translation']
            rotation = data['transform']['rotation']
            t = [translation['x'], translation['y'], translation['z']]
            q = [rotation['x'], rotation['y'], rotation['z'], rotation['w']]
        except (KeyError, TypeError) as exc:
            self.get_logger().fatal(f'Missing field in {calib_path}: {exc}')
            raise SystemExit(1)
        return effector_frame, tracking_base_frame, _to_matrix(t, q)

    def _on_take_sample(self, request, response):
        now = self.get_clock().now()

        d455_staleness = self.d455.staleness_s(now)
        d405_staleness = self.d405.staleness_s(now)
        if d455_staleness > self.max_staleness_s:
            response.success = False
            response.message = (
                f'D455: no recent detection of tag {self.target_tag_id} '
                f'(staleness={d455_staleness:.2f}s > {self.max_staleness_s}s) -- keep the tag in view')
            self.get_logger().warn(response.message)
            return response
        if d405_staleness > self.max_staleness_s:
            response.success = False
            response.message = (
                f'D405: no recent detection of tag {self.target_tag_id} '
                f'(staleness={d405_staleness:.2f}s > {self.max_staleness_s}s) -- keep the tag in view')
            self.get_logger().warn(response.message)
            return response

        d455_pose = self.d455.estimate_pose(self.tag_size_m)
        d405_pose = self.d405.estimate_pose(self.tag_size_m)
        if d455_pose is None or d405_pose is None:
            response.success = False
            response.message = 'solvePnP failed on one of the two cameras -- retry'
            self.get_logger().warn(response.message)
            return response

        try:
            # Latest available, not the detections' own stamps: fp3_link0 ->
            # fp3_hand is a live, joint-states-driven transform whose publish
            # rate can lag behind a detection's capture stamp enough to trip
            # "extrapolation into the future" (cf. fp3_apriltag_demo's same
            # fix) -- fine here as long as the arm is held STILL while
            # sampling, which is the whole point of a discrete take_sample
            # call rather than continuous auto-collection.
            tf = self.tf_buffer.lookup_transform(
                self.robot_base_frame, self.d405_effector_frame, Time(),
                timeout=Duration(seconds=self.tf_timeout_s))
        except TransformException as ex:
            response.success = False
            response.message = f'TF {self.robot_base_frame} -> {self.d405_effector_frame} unavailable: {ex}'
            self.get_logger().warn(response.message)
            return response

        tr = tf.transform.translation
        rot = tf.transform.rotation
        t_base_hand = _to_matrix([tr.x, tr.y, tr.z], [rot.x, rot.y, rot.z, rot.w])

        r_d405_tag, t_d405_tag = d405_pose
        r_d455_tag, t_d455_tag = d455_pose
        t_d405_tag_m = _rt_to_matrix(r_d405_tag, t_d405_tag)
        t_d455_tag_m = _rt_to_matrix(r_d455_tag, t_d455_tag)

        t_base_tag = t_base_hand @ self.t_hand_d405 @ t_d405_tag_m
        t_base_d455 = t_base_tag @ np.linalg.inv(t_d455_tag_m)

        self._samples.append((t_base_d455[:3, :3].copy(), t_base_d455[:3, 3].copy()))
        self._d455_tracking_base_frame = self.d455.detection_frame_id

        n = len(self._samples)
        response.success = True
        response.message = f'Sample {n} recorded.'
        self.get_logger().info(
            f'Sample {n}: fp3_link0 -> D455 optical = t={t_base_d455[:3, 3]}')
        if n >= 2:
            self._log_spread()
        return response

    def _log_spread(self):
        translations = np.array([t for _, t in self._samples])
        mean_t = translations.mean(axis=0)
        max_dev_mm = float(np.max(np.linalg.norm(translations - mean_t, axis=1))) * 1000
        rotations = Rotation.from_matrix(np.stack([r for r, _ in self._samples]))
        mean_r = rotations.mean()
        angles_deg = (mean_r.inv() * rotations).magnitude() * 180.0 / np.pi
        max_angle_deg = float(np.max(angles_deg))
        self.get_logger().info(
            f'  spread so far: max translation deviation from mean = {max_dev_mm:.1f}mm, '
            f'max rotation deviation from mean = {max_angle_deg:.2f}deg')

    def _on_save_calibration(self, request, response):
        n = len(self._samples)
        if n == 0:
            response.success = False
            response.message = 'No samples recorded yet -- call take_sample first'
            self.get_logger().warn(response.message)
            return response
        if n < 3:
            self.get_logger().warn(
                f'Only {n} sample(s) -- averaging needs several DIFFERENT robot poses to '
                f'actually cancel out detection noise, cf. calib_bridge/CLAUDE.md. Saving anyway.')

        translations = np.array([t for _, t in self._samples])
        mean_t = translations.mean(axis=0)
        rotations = Rotation.from_matrix(np.stack([r for r, _ in self._samples]))
        mean_r = rotations.mean()
        mean_q = mean_r.as_quat()  # x, y, z, w

        tracking_base_frame = getattr(self, '_d455_tracking_base_frame', None) or 'camera_color_optical_frame'

        calibration = HandeyeCalibration()
        calibration.parameters = HandeyeCalibrationParameters(
            name=self.output_calibration_name,
            calibration_type='eye_on_base',
            robot_base_frame=self.robot_base_frame,
            # Metadata only (matches the schema of a normal eye_on_base
            # .calib -- cf. calib_bridge/CLAUDE.md) -- not used by this
            # node's own derivation math, which never runs an AX=XB solve
            # against a tag mounted on the effector.
            robot_effector_frame=self.d405_effector_frame,
            tracking_base_frame=tracking_base_frame,
            tracking_marker_frame=f'tag36h11:{self.target_tag_id}',
            freehand_robot_movement=True,
        )
        calibration.transform.translation.x = float(mean_t[0])
        calibration.transform.translation.y = float(mean_t[1])
        calibration.transform.translation.z = float(mean_t[2])
        calibration.transform.rotation.x = float(mean_q[0])
        calibration.transform.rotation.y = float(mean_q[1])
        calibration.transform.rotation.z = float(mean_q[2])
        calibration.transform.rotation.w = float(mean_q[3])

        filepath = save_calibration(calibration)
        response.success = True
        response.message = f'Saved {n}-sample average to {filepath}'
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = BridgeCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
