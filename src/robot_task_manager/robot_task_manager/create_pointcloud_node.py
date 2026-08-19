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
# expects, same convention the native RealSense colored pointcloud uses. Also
# the exact dtype pointcloud_publisher_node uses to build request.raw_cloud —
# read back here with a plain np.frombuffer/reshape (never
# sensor_msgs_py.read_points(), cf. CLAUDE.md racine "Pourquoi plus de nuage
# natif RealSense" for the PointCloud2 parsing bug class that avoids).
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

    def _parse_raw_cloud(self, cloud_msg):
        """Reads back the organized (height, width) XYZRGB cloud published
        continuously by pointcloud_publisher_node. Plain numpy
        frombuffer/reshape against the fixed, padding-free _XYZRGB_DTYPE —
        deliberately not sensor_msgs_py.read_points() (cf. module docstring).
        """
        structured = np.frombuffer(cloud_msg.data, dtype=_XYZRGB_DTYPE).reshape(
            cloud_msg.height, cloud_msg.width)
        xyz = np.stack(
            [structured['x'], structured['y'], structured['z']], axis=-1)
        rgb = structured['rgb']
        return xyz, rgb

    def _xyzrgb_cloud(self, header, xyz_points, rgb_points):
        structured = np.zeros(xyz_points.shape[0], dtype=_XYZRGB_DTYPE)
        structured['x'], structured['y'], structured['z'] = xyz_points[:, 0], xyz_points[:, 1], xyz_points[:, 2]
        structured['rgb'] = rgb_points
        return pc2.create_cloud(header, _XYZRGB_FIELDS, structured)

    def handle_create_pointcloud(self, request, response):
        try:
            mask_raw = self.bridge.imgmsg_to_cv2(request.mask, desired_encoding='mono8')
            cloud_msg = request.raw_cloud

            if cloud_msg.height == 0 or cloud_msg.width == 0:
                response.success = False
                response.message = "raw_cloud has no height/width (not organized, or empty)"
                return response

            if mask_raw.shape != (cloud_msg.height, cloud_msg.width):
                response.success = False
                response.message = (
                    f"mask shape {mask_raw.shape} does not match raw_cloud shape "
                    f"({cloud_msg.height}, {cloud_msg.width})"
                )
                return response

            xyz, rgb = self._parse_raw_cloud(cloud_msg)

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

            out_msg = self._xyzrgb_cloud(cloud_msg.header, points, colors)
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

            scene_msg = self._xyzrgb_cloud(cloud_msg.header, scene_points, scene_colors)
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
