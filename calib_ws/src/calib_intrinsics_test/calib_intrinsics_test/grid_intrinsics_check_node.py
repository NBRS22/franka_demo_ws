"""
Static camera-intrinsics check: a printed sheet of 9 AprilTags (36h11, ids
0-8, 4cm black square each) arranged in a 3x3 grid, tags read left-to-right
top-to-bottom (id = row*3 + col):

    0  1  2
    3  4  5
    6  7  8

Center-to-center spacing is 6cm in both X and Y (2cm gap between the black
squares' edges + 2cm half-width on each side of the 4cm tag). No robot motion
involved at all: every tag's 3D position is computed via solvePnP in the
camera's OWN frame from a single view, so this validates Fx/Fy/Cx/Cy/
distortion directly against known physical geometry -- no hand-eye
calibration in the loop, unlike the robot-motion-based
calib_intrinsics_test.intrinsics_test_node.

Rather than a single blended average over adjacent pairs (which can hide an
anisotropic Fx vs Fy bias if the two errors happen to cancel out), this
checks ALL pairwise distances against their expected grid distance, and
reports results broken down by direction: horizontal-only, vertical-only,
and diagonal (which mixes both axes). A uniform scale bias shows consistently
across all three categories; a direction-specific bias only shows in one.
"""
import itertools

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from apriltag_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import CameraInfo

TAG_SIZE_M = 0.04
GRID_SPACING_M = 0.06
GRID_COLS = 3
NUM_TAGS = 9
SAMPLES_PER_TAG = 15
WAIT_FOR_ALL_TAGS_S = 20.0


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


def grid_index(tag_id):
    return tag_id // GRID_COLS, tag_id % GRID_COLS


def expected_distance(id_a, id_b):
    ra, ca = grid_index(id_a)
    rb, cb = grid_index(id_b)
    return GRID_SPACING_M * float(np.hypot(ra - rb, ca - cb))


def pair_direction(id_a, id_b):
    ra, ca = grid_index(id_a)
    rb, cb = grid_index(id_b)
    if ra == rb:
        return 'horizontal'
    if ca == cb:
        return 'vertical'
    return 'diagonal'


class GridIntrinsicsCheckNode(Node):

    def __init__(self):
        super().__init__('grid_intrinsics_check')
        self._camera_info = None
        self._latest_by_id = {}
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self._ci_cb, 10)
        self.create_subscription(AprilTagDetectionArray, '/detections', self._det_cb, 10)

    def _ci_cb(self, msg):
        self._camera_info = msg

    def _det_cb(self, msg):
        for d in msg.detections:
            if 0 <= d.id < NUM_TAGS:
                self._latest_by_id[d.id] = d

    def spin_for(self, seconds):
        deadline = self.get_clock().now() + Duration(seconds=seconds)
        while self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_camera_info(self):
        while self._camera_info is None:
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_for_all_tags(self):
        self.get_logger().info(
            f'Waiting up to {WAIT_FOR_ALL_TAGS_S:.0f}s for all {NUM_TAGS} tags to be seen at least once...')
        deadline = self.get_clock().now() + Duration(seconds=WAIT_FOR_ALL_TAGS_S)
        while self.get_clock().now() < deadline and len(self._latest_by_id) < NUM_TAGS:
            rclpy.spin_once(self, timeout_sec=0.1)
        missing = sorted(set(range(NUM_TAGS)) - set(self._latest_by_id.keys()))
        if missing:
            self.get_logger().warn(f'Never saw tag id(s) {missing} -- proceeding with what was detected')

    def average_positions(self):
        k = np.array(self._camera_info.k, dtype=np.float64).reshape(3, 3)
        dist = np.array(self._camera_info.d, dtype=np.float64)

        samples = {tag_id: [] for tag_id in self._latest_by_id}
        target_count = SAMPLES_PER_TAG * len(samples)
        collected = 0
        deadline = self.get_clock().now() + Duration(seconds=10.0)
        while collected < target_count and self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            # Consume-and-clear: a detection is only sampled once, then
            # removed, so the same message can never be resampled as if it
            # were several independent readings while waiting for the next
            # /detections callback to refresh it (that would silently inflate
            # the sample count without adding real statistical independence).
            for tag_id in list(self._latest_by_id.keys()):
                if len(samples[tag_id]) >= SAMPLES_PER_TAG:
                    continue
                detection = self._latest_by_id.pop(tag_id)
                pos = estimate_tag_position(detection, k, dist)
                if pos is not None:
                    samples[tag_id].append(pos)
                    collected += 1

        positions = {}
        for tag_id, pts in samples.items():
            if pts:
                positions[tag_id] = np.mean(pts, axis=0)
        return positions


def run_check(node):
    node.wait_for_camera_info()
    node.wait_for_all_tags()
    positions = node.average_positions()

    seen_ids = sorted(positions.keys())
    node.get_logger().info(f'Tags detected: {seen_ids}')
    if len(seen_ids) < 2:
        node.get_logger().error('Fewer than 2 tags detected -- cannot compute any distance')
        return

    rows = []
    for id_a, id_b in itertools.combinations(seen_ids, 2):
        measured = float(np.linalg.norm(positions[id_a] - positions[id_b]))
        expected = expected_distance(id_a, id_b)
        direction = pair_direction(id_a, id_b)
        error_mm = (measured - expected) * 1000
        error_pct = (error_mm / (expected * 1000)) * 100 if expected > 0 else float('nan')
        rows.append((id_a, id_b, direction, expected, measured, error_mm, error_pct))

    node.get_logger().info('')
    node.get_logger().info(f'{"pair":>7} {"dir":>10} {"expected":>10} {"measured":>10} {"error":>9} {"error%":>8}')
    for id_a, id_b, direction, expected, measured, error_mm, error_pct in rows:
        node.get_logger().info(
            f'{id_a}-{id_b:>5} {direction:>10} {expected*100:>8.2f}cm {measured*100:>8.2f}cm '
            f'{error_mm:>+7.2f}mm {error_pct:>+6.2f}%')

    node.get_logger().info('')
    node.get_logger().info('=== SUMMARY BY DIRECTION ===')
    for direction in ('horizontal', 'vertical', 'diagonal'):
        subset = [r for r in rows if r[2] == direction]
        if not subset:
            continue
        errors_mm = np.array([r[5] for r in subset])
        errors_pct = np.array([r[6] for r in subset])
        node.get_logger().info(
            f'{direction:>10}: n={len(subset):2d}  mean error={errors_mm.mean():+.2f}mm '
            f'({errors_pct.mean():+.2f}%)  std={errors_mm.std():.2f}mm')

    all_expected = np.array([r[3] for r in rows])
    all_measured = np.array([r[4] for r in rows])
    # Best-fit uniform scale factor (measured = scale * expected), least squares
    # through the origin -- separates a genuine Fx/Fy scale bias (would show up
    # as scale != 1.0 consistently) from per-pair detection noise (would show
    # up as scatter around scale ~= 1.0 instead).
    scale = float(np.sum(all_measured * all_expected) / np.sum(all_expected ** 2))
    node.get_logger().info('')
    node.get_logger().info(f'Best-fit uniform scale factor (measured/expected): {scale:.4f}')
    if abs(scale - 1.0) < 0.01:
        node.get_logger().info(
            '-> Camera intrinsics measure known geometry accurately (scale within 1%). '
            'No sign of a systematic Fx/Fy bias.')
    else:
        node.get_logger().info(
            f'-> Scale off by {abs(scale - 1.0) * 100:.1f}% -- worth a full checkerboard '
            'recalibration (ros2 run camera_calibration cameracalibrator).')


def main(args=None):
    rclpy.init(args=args)
    node = GridIntrinsicsCheckNode()
    try:
        run_check(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
