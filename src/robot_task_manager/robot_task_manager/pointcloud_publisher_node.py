import numpy as np
import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


# x, y, z, rgb (packed as a bit-reinterpreted float32 -- the PCL/RViz XYZRGB
# convention, cf. create_pointcloud_node). Same fixed, padding-free
# structured dtype produced here and consumed by create_pointcloud_node --
# both ends read/write it with plain numpy (frombuffer/tobytes), never
# sensor_msgs_py.read_points(), which is the whole class of PointCloud2
# parsing bug (row_step padding, reshape order) this project already hit
# once with the native RealSense cloud (cf. CLAUDE.md racine, "Pourquoi plus
# de nuage natif RealSense").
_XYZRGB_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
]
_XYZRGB_DTYPE = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.float32)])


# Continuously deprojects aligned_depth_to_color + camera_info into a full
# (unmasked), organized, colored point cloud and publishes it on
# /pick/raw_pointcloud -- the geometry source create_pointcloud_node used to
# compute itself, on every request, from scratch. Splitting it out here
# means: (1) the raw cloud exists independently of a pick request, so it can
# be visualized in RViz at any time (nothing else currently publishes a live
# point cloud -- pointcloud.enable is deliberately off RealSense-side, cf.
# CLAUDE.md racine), and (2) create_pointcloud_node no longer redoes the
# same deprojection math on every single pick.
#
# Deliberately organized (height/width set, row_step = width * point_step,
# no padding) so create_pointcloud_node can index it directly by SAM3's
# mask (same (height, width) grid) without re-deriving pixel<->3D geometry.
#
# Synchronization: camera_buffer_node subscribes to this topic the same way
# it already buffers rgb/depth/camera_info, and returns whichever cloud was
# latest *at the same /get_frames call* that captured the rgb sent to SAM3 --
# not "whatever is latest by the time create_pointcloud is actually called",
# which could be a different camera instant after SAM3's round-trip
# (cf. camera_buffer_node's existing rationale for bundling rgb/depth/
# camera_info together at the start of a pick).
class PointcloudPublisherNode(Node):
    def __init__(self):
        super().__init__('pointcloud_publisher_node')

        self.bridge = CvBridge()

        self.declare_parameter('publish_rate_hz', 10.0)
        publish_rate_hz = self.get_parameter('publish_rate_hz').value

        self._last_rgb = None
        self._last_depth = None
        self._last_camera_info = None

        self.create_subscription(Image, '/camera/camera/color/image_raw', self._rgb_cb, 10)
        self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', self._depth_cb, 10)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self._camera_info_cb, 10)

        self.cloud_pub = self.create_publisher(PointCloud2, '/pick/raw_pointcloud', 10)

        self.create_timer(1.0 / publish_rate_hz, self._publish_cloud)

        self.get_logger().info('Pointcloud Publisher Node started')

    def _rgb_cb(self, msg):
        self._last_rgb = msg

    def _depth_cb(self, msg):
        self._last_depth = msg

    def _camera_info_cb(self, msg):
        self._last_camera_info = msg

    def _deproject_xyz(self, depth_raw, camera_info):
        """Same pinhole deprojection as create_pointcloud_node used to do
        itself -- moved here now that it runs continuously instead of once
        per pick request."""
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

    def _publish_cloud(self):
        if self._last_rgb is None or self._last_depth is None or self._last_camera_info is None:
            return

        try:
            depth_raw = self.bridge.imgmsg_to_cv2(self._last_depth, desired_encoding='passthrough')
            rgb_msg = self._last_rgb

            if depth_raw.shape != (rgb_msg.height, rgb_msg.width):
                self.get_logger().warn(
                    f"depth shape {depth_raw.shape} does not match rgb shape "
                    f"({rgb_msg.height}, {rgb_msg.width}), skipping this frame")
                return

            xyz = self._deproject_xyz(depth_raw, self._last_camera_info)
            rgb = self._pack_rgb_float32(rgb_msg)

            height, width = depth_raw.shape
            structured = np.zeros((height, width), dtype=_XYZRGB_DTYPE)
            structured['x'] = xyz[..., 0]
            structured['y'] = xyz[..., 1]
            structured['z'] = xyz[..., 2]
            structured['rgb'] = rgb

            msg = PointCloud2()
            msg.header = self._last_depth.header
            msg.height = height
            msg.width = width
            msg.fields = _XYZRGB_FIELDS
            msg.is_bigendian = False
            msg.point_step = _XYZRGB_DTYPE.itemsize
            msg.row_step = _XYZRGB_DTYPE.itemsize * width
            msg.is_dense = False  # NaN points where depth was 0
            msg.data = structured.tobytes()

            self.cloud_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error building pointcloud: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PointcloudPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
