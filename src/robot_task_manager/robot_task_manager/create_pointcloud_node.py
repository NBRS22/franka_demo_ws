import cv2
import numpy as np
import rclpy

from franka_demo_interfaces.srv import CreatePointcloud
from sensor_msgs_py import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from cv_bridge import CvBridge
from rclpy.node import Node


_MAX_SCENE_POINTS = 20_000

# x, y, z, rgb (packed as a bit-reinterpreted float32 — the PCL/RViz XYZRGB
# convention: rgb_uint32 = (r<<16)|(g<<8)|b, then reinterpreted as float32 bits,
# not cast). Matches the field layout RViz's PointCloud2 "RGB8" color transformer
# expects, same convention the native RealSense colored pointcloud uses.
_XYZRGB_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
]
_XYZRGB_DTYPE = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.float32)])


class CreatePointcloudNode(Node):
    def __init__(self):
        super().__init__('create_pointcloud_node')

        # CV Bridge
        self.bridge = CvBridge()

        # Params
        self.declare_parameter('scene_exclusion_margin_px', 15)
        self.scene_exclusion_margin_px = self.get_parameter('scene_exclusion_margin_px').value
        self.declare_parameter('object_erosion_margin_px', 2)
        self.object_erosion_margin_px = self.get_parameter('object_erosion_margin_px').value

        # ROS Publisher Topics
        self.cloud_pub = self.create_publisher(PointCloud2, '/pick/pointcloud', 10)
        self.scene_cloud_pub = self.create_publisher(PointCloud2, '/pick/scene_pointcloud', 10)

        # ROS Service Servers
        self.create_service(CreatePointcloud, 'create_pointcloud', self.handle_create_pointcloud)

        self.get_logger().info("Create Pointcloud Node started")

    def _deproject_xyz(self, depth_raw, camera_info):
        """Manually deproject an aligned depth image into an (height, width, 3) xyz
        array using the pinhole intrinsics from camera_info.k (row-major 3x3: fx, 0,
        cx, 0, fy, cy, 0, 0, 1).

        Not using the native RealSense /depth/color/points topic: with
        align_depth.enable:=true, that topic is a documented, unfixed upstream bug —
        the pointcloud is computed independently of the depth-to-color alignment and
        ends up spatially shifted (translation offset, ~cm scale) — see
        https://github.com/IntelRealSense/realsense-ros/issues/2595 and
        https://github.com/realsenseai/realsense-ros/issues/3050. Deprojecting
        ourselves from aligned_depth_to_color (guaranteed pixel-aligned to the color
        image by construction — a separate, much simpler, unaffected feature) sidesteps
        the bug entirely, and also removes the whole class of PointCloud2-parsing bugs
        we hit earlier (row_step padding, reshape order) since we no longer parse any
        externally-produced organized cloud at all.
        """
        fx, cx = camera_info.k[0], camera_info.k[2]
        fy, cy = camera_info.k[4], camera_info.k[5]

        depth_m = depth_raw.astype(np.float32) * 0.001  # RealSense aligned depth is 16UC1, millimeters
        depth_m[depth_raw == 0] = np.nan  # 0 == no valid depth at this pixel

        height, width = depth_raw.shape
        u, v = np.meshgrid(np.arange(width, dtype=np.float32), np.arange(height, dtype=np.float32))

        x = (u - cx) * depth_m / fx
        y = (v - cy) * depth_m / fy
        return np.stack([x, y, depth_m], axis=-1)

    def _pack_rgb_float32(self, rgb_msg):
        """BGR8 image -> (height, width) float32 array, PCL/RViz XYZRGB packing."""
        bgr = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        b, g, r = bgr[..., 0].astype(np.uint32), bgr[..., 1].astype(np.uint32), bgr[..., 2].astype(np.uint32)
        rgb_uint32 = (r << 16) | (g << 8) | b
        return rgb_uint32.view(np.float32)

    def _dilate_mask(self, mask_raw, margin_px):
        """Grow the object mask by margin_px before it's used to exclude points from
        the scene cloud. SAM3's mask edges are only accurate to a few pixels, and
        depth noise adds a bit more slop right at the object boundary — without this
        margin, scene points that are actually just noisy edge artifacts of the
        object itself end up right next to real grasp contact points, and GraspGen's
        collision filter (filter_colliding_grasps_fast, cf. graspgen_bridge) rejects
        nearly every grasp as a false positive. The object cloud itself uses the
        eroded mask instead (cf. _erode_mask) — only what counts as "scene" gets this
        outward margin.
        """
        if margin_px <= 0:
            return mask_raw
        kernel = np.ones((margin_px * 2 + 1, margin_px * 2 + 1), np.uint8)
        return cv2.dilate(mask_raw, kernel, iterations=1)

    def _erode_mask(self, mask_raw, margin_px):
        """Shrink the object mask by margin_px before it's used to build the object
        cloud sent to GraspGen. Same root cause as _dilate_mask above (SAM3 edge
        imprecision + RealSense depth noise concentrated right at the object
        boundary), but the mirror-image symptom: noisy boundary points end up *inside*
        the object cloud instead of leaking into the scene cloud, distorting the
        surface geometry GraspGen's diffusion sampler conditions on. Trimming a small
        margin off the mask before deprojecting the object removes those points
        without eroding so much that a genuinely small object loses too much of its
        cloud.
        """
        if margin_px <= 0:
            return mask_raw
        kernel = np.ones((margin_px * 2 + 1, margin_px * 2 + 1), np.uint8)
        return cv2.erode(mask_raw, kernel, iterations=1)

    def _xyzrgb_cloud(self, header, xyz_points, rgb_points):
        structured = np.zeros(xyz_points.shape[0], dtype=_XYZRGB_DTYPE)
        structured['x'], structured['y'], structured['z'] = xyz_points[:, 0], xyz_points[:, 1], xyz_points[:, 2]
        structured['rgb'] = rgb_points
        return pc2.create_cloud(header, _XYZRGB_FIELDS, structured)

    def handle_create_pointcloud(self, request, response):
        try:
            depth_raw = self.bridge.imgmsg_to_cv2(request.depth, desired_encoding='passthrough')
            mask_raw = self.bridge.imgmsg_to_cv2(request.mask, desired_encoding='mono8')

            if mask_raw.shape != depth_raw.shape:
                response.success = False
                response.message = (
                    f"mask shape {mask_raw.shape} does not match depth shape "
                    f"{depth_raw.shape} — aligned_depth_to_color and the color image "
                    "SAM3 segmented should always be the same resolution"
                )
                return response

            xyz = self._deproject_xyz(depth_raw, request.camera_info)
            rgb = self._pack_rgb_float32(request.rgb)
            if rgb.shape != mask_raw.shape:
                response.success = False
                response.message = (
                    f"rgb shape {rgb.shape} does not match mask shape {mask_raw.shape}"
                )
                return response

            valid = np.isfinite(xyz).all(axis=2)
            eroded_mask = self._erode_mask(mask_raw, self.object_erosion_margin_px)
            if not eroded_mask.any():
                # Object too small/thin for this erosion margin — falling back to the
                # raw mask keeps the object cloud non-empty rather than silently
                # trimming a small object away to nothing.
                self.get_logger().warn(
                    f"object_erosion_margin_px={self.object_erosion_margin_px} erodes the "
                    "mask to empty — falling back to the un-eroded mask for this request"
                )
                eroded_mask = mask_raw
            object_mask = (eroded_mask > 0) & valid

            points = xyz[object_mask]
            colors = rgb[object_mask]

            if points.shape[0] == 0:
                response.success = False
                response.message = "no valid points in mask"
                return response

            out_msg = self._xyzrgb_cloud(request.depth.header, points, colors)
            self.cloud_pub.publish(out_msg)

            self.get_logger().info(f"Pointcloud published — {points.shape[0]} points")

            # Scene-minus-object cloud: collision context for GraspGen (table, other
            # objects, ...), and colored for visualization. Best-effort — an
            # empty/undersized scene cloud just means no collision filtering
            # downstream, not a failure of this request.
            dilated_mask = self._dilate_mask(mask_raw, self.scene_exclusion_margin_px)
            scene_mask = (dilated_mask == 0) & valid
            scene_points = xyz[scene_mask]
            scene_colors = rgb[scene_mask]
            if scene_points.shape[0] > _MAX_SCENE_POINTS:
                idx = np.random.choice(scene_points.shape[0], _MAX_SCENE_POINTS, replace=False)
                scene_points = scene_points[idx]
                scene_colors = scene_colors[idx]

            scene_msg = self._xyzrgb_cloud(request.depth.header, scene_points, scene_colors)
            self.scene_cloud_pub.publish(scene_msg)

            self.get_logger().info(f"Scene pointcloud published — {scene_points.shape[0]} points")

            response.cloud = out_msg
            response.scene_cloud = scene_msg
            response.success = True
            response.message = ""

        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            response.success = False
            response.message = str(e)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = CreatePointcloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
