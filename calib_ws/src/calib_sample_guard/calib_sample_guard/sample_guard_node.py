"""
Live guardrail for hand-eye calibration sampling: watches /detections for the
tag being calibrated and continuously reports, for the CURRENT view:

  1. Reprojection error (px) -- reproject the 4 known 3D tag corners through
     the solved pose (rvec/tvec) and compare to the corners AprilTag actually
     detected. A high residual means the pose doesn't self-consistently
     explain the image -- a genuinely bad/noisy detection.
  2. Tilt angle (deg) from head-on -- how far the tag's face is from pointing
     straight back at the camera. Low reprojection error alone does NOT rule
     out a wrong pose: planar targets viewed near head-on are the classic
     case where solvePnP can converge to an "ambiguous" flipped solution that
     ALSO reprojects with low error (cf. the pose-ambiguity problem for
     planar PnP). Near-head-on views are exactly the danger zone for this,
     so a separate tilt check is needed -- reprojection error alone can't
     catch it.

This is meant to run ALONGSIDE a live easy_handeye2 calibration session
(rqt_calibrator) -- watch this terminal and only click "Take sample" when
the current view is clear of both warnings, rather than trying to audit
already-taken samples afterward (impossible: easy_handeye2's saved samples
only keep the already-computed poses, not the raw pixel corners needed to
recompute any of this).
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from apriltag_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import CameraInfo

TARGET_TAG_ID = 0
TAG_SIZE_M = 0.04
REPROJECTION_ERROR_WARN_PX = 1.5
TILT_TOO_FLAT_WARN_DEG = 20.0
TILT_TOO_OBLIQUE_WARN_DEG = 60.0
PRINT_PERIOD_S = 0.5

_OBJECT_POINTS = np.array([
    [-TAG_SIZE_M / 2, -TAG_SIZE_M / 2, 0.0],
    [TAG_SIZE_M / 2, -TAG_SIZE_M / 2, 0.0],
    [TAG_SIZE_M / 2, TAG_SIZE_M / 2, 0.0],
    [-TAG_SIZE_M / 2, TAG_SIZE_M / 2, 0.0],
], dtype=np.float64)


def solve_and_check(detection, k, dist):
    image_points = np.array([[c.x, c.y] for c in detection.corners], dtype=np.float64)
    ok, rvec, tvec = cv2.solvePnP(_OBJECT_POINTS, image_points, k, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None

    reprojected, _ = cv2.projectPoints(_OBJECT_POINTS, rvec, tvec, k, dist)
    reprojected = reprojected.reshape(-1, 2)
    reprojection_error_px = float(np.linalg.norm(reprojected - image_points, axis=1).mean())

    r, _ = cv2.Rodrigues(rvec)
    # Tag's local +Z ("out of the tag toward the camera", cf. this codebase's
    # own convention -- see apriltag_move_once_node.py) expressed in the
    # camera frame. Head-on (tag facing the camera dead-on) means this
    # points back along the camera's own -Z (the camera looks down +Z into
    # the scene) -- tilt = angle between the two, 0deg = perfectly head-on.
    normal_in_camera = r @ np.array([0.0, 0.0, 1.0])
    cos_tilt = np.clip(-normal_in_camera[2], -1.0, 1.0)
    tilt_deg = float(np.degrees(np.arccos(cos_tilt)))

    depth_m = float(tvec[2, 0])
    return reprojection_error_px, tilt_deg, depth_m


class SampleGuardNode(Node):

    def __init__(self):
        super().__init__('calib_sample_guard')
        self.declare_parameter('target_tag_id', TARGET_TAG_ID)
        self.declare_parameter('tag_size', TAG_SIZE_M)
        self.target_tag_id = self.get_parameter('target_tag_id').value
        self.tag_size = self.get_parameter('tag_size').value

        self._camera_info = None
        self._last_print = self.get_clock().now()
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self._ci_cb, 10)
        self.create_subscription(AprilTagDetectionArray, '/detections', self._det_cb, 10)
        self.get_logger().info(
            f'Watching tag id={self.target_tag_id} -- take a sample only when both '
            f'reprojection error and tilt are clear of warnings below.')

    def _ci_cb(self, msg):
        self._camera_info = msg

    def _det_cb(self, msg):
        if self._camera_info is None:
            return
        now = self.get_clock().now()
        if (now - self._last_print).nanoseconds < PRINT_PERIOD_S * 1e9:
            return

        detection = next((d for d in msg.detections if d.id == self.target_tag_id), None)
        if detection is None:
            self.get_logger().warn(f'Tag id={self.target_tag_id} not currently visible')
            self._last_print = now
            return

        k = np.array(self._camera_info.k, dtype=np.float64).reshape(3, 3)
        dist = np.array(self._camera_info.d, dtype=np.float64)
        result = solve_and_check(detection, k, dist)
        self._last_print = now
        if result is None:
            self.get_logger().warn('solvePnP failed on the current detection')
            return

        reprojection_error_px, tilt_deg, depth_m = result
        warnings = []
        if reprojection_error_px > REPROJECTION_ERROR_WARN_PX:
            warnings.append(f'HIGH REPROJECTION ERROR (>{REPROJECTION_ERROR_WARN_PX:.1f}px)')
        if tilt_deg < TILT_TOO_FLAT_WARN_DEG:
            warnings.append(f'TOO FLAT/HEAD-ON (<{TILT_TOO_FLAT_WARN_DEG:.0f}deg -- PnP pose-ambiguity risk)')
        if tilt_deg > TILT_TOO_OBLIQUE_WARN_DEG:
            warnings.append(f'TOO OBLIQUE (>{TILT_TOO_OBLIQUE_WARN_DEG:.0f}deg -- degraded corner detection)')

        status = 'OK -- safe to take sample' if not warnings else ' / '.join(warnings)
        self.get_logger().info(
            f'reprojection={reprojection_error_px:5.2f}px  tilt={tilt_deg:5.1f}deg  '
            f'depth={depth_m:4.2f}m  -- {status}')


def main(args=None):
    rclpy.init(args=args)
    node = SampleGuardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
