import math

import rclpy
from franka_demo_interfaces.srv import VisualizeGrasps
from geometry_msgs.msg import Pose
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


# RViz draws a pose-based Marker.ARROW along the marker's local +X axis
# (fixed RViz convention, scale.x = length) -- but this pipeline's grasp
# pose convention (cf. GraspGen's docs/GRIPPER_DESCRIPTION.md, and
# pick_place_node.cpp's approachTiltDeg) puts the approach direction
# (pointing from the gripper toward the object) on local +Z, with +X being
# the finger-closing axis instead. Rotating the grasp orientation by a fixed
# -90deg about local Y before handing it to the marker remaps +Z onto +X, so
# the rendered arrow actually shows the approach direction, not the
# finger-closing axis. Equivalent to composing with quaternion
# (w=cos(-45deg), x=0, y=sin(-45deg), z=0) via q_marker = q_grasp * R;
# verified by hand against known cases (identity -> arrow along +Z;
# straight-down top-down grasp -> arrow along -Z).
_SQRT2_OVER_2 = math.sqrt(2.0) / 2.0

_ARROW_LENGTH = 0.08  # meters -- also used to offset the arrow tail so its
                      # tip (not tail) lands exactly on the grasp point.


def _approach_to_arrow_orientation(q):
    """geometry_msgs/Quaternion (local +Z = approach) -> new Quaternion whose
    local +X (RViz's ARROW direction) points the same way."""
    w, x, y, z = q.w, q.x, q.y, q.z
    out = Pose().orientation
    out.w = _SQRT2_OVER_2 * (w + y)
    out.x = _SQRT2_OVER_2 * (x + z)
    out.y = _SQRT2_OVER_2 * (y - w)
    out.z = _SQRT2_OVER_2 * (z - x)
    return out


def _approach_axis_world(q):
    """geometry_msgs/Quaternion -> (x, y, z) unit vector: local +Z (approach
    axis) rotated into world frame, i.e. the direction the gripper travels
    as it approaches the object."""
    w, x, y, z = q.w, q.x, q.y, q.z
    return (
        2.0 * (x * z + w * y),
        2.0 * (y * z - w * x),
        1.0 - 2.0 * (x * x + y * y),
    )


class VisualizeGraspsNode(Node):
    def __init__(self):
        super().__init__('visualize_grasps_node')

        self.markers_pub = self.create_publisher(MarkerArray, '/pick/grasp_markers', 10)

        self.create_service(
            VisualizeGrasps,
            'visualize_grasps',
            self.handle_visualize,
        )

        self.get_logger().info("Visualize Grasps Node started")

    def handle_visualize(self, request, response):
        try:
            grasps = request.grasps.poses
            scores = list(request.scores)
            frame_id = request.grasps.header.frame_id
            stamp = request.grasps.header.stamp

            if not grasps:
                response.success = False
                response.message = "no grasps to visualize"
                return response

            if len(grasps) != len(scores):
                response.success = False
                response.message = (
                    f"grasps/scores length mismatch : {len(grasps)} grasps vs {len(scores)} scores"
                )
                return response

            best_idx = scores.index(max(scores))

            marker_array = MarkerArray()

            # clear previous markers
            clear = Marker()
            clear.action = Marker.DELETEALL
            marker_array.markers.append(clear)

            for i, pose in enumerate(grasps):
                m = Marker()
                m.header.frame_id = frame_id
                m.header.stamp = stamp
                m.ns = "grasps"
                m.id = i
                m.type = Marker.ARROW
                m.action = Marker.ADD
                # Tail offset backward along the approach direction by the
                # arrow's own length, so the tip -- not the tail -- lands on
                # the actual grasp point. The arrow then reads as "the
                # gripper came from here, along this orientation, to reach
                # this point", matching the real approach trajectory.
                ax, ay, az = _approach_axis_world(pose.orientation)
                m.pose.position.x = pose.position.x - _ARROW_LENGTH * ax
                m.pose.position.y = pose.position.y - _ARROW_LENGTH * ay
                m.pose.position.z = pose.position.z - _ARROW_LENGTH * az
                m.pose.orientation = _approach_to_arrow_orientation(pose.orientation)
                m.scale.x = _ARROW_LENGTH  # arrow length
                m.scale.y = 0.008  # shaft diameter
                m.scale.z = 0.012  # head diameter
                m.lifetime.sec = 0
                m.lifetime.nanosec = 0

                if i == best_idx:
                    # best grasp: green, fully opaque
                    m.color.r = 0.0
                    m.color.g = 1.0
                    m.color.b = 0.2
                    m.color.a = 1.0
                else:
                    # other grasps: cyan, semi-transparent
                    m.color.r = 0.0
                    m.color.g = 0.7
                    m.color.b = 1.0
                    m.color.a = 0.4

                marker_array.markers.append(m)

            self.markers_pub.publish(marker_array)

            self.get_logger().info(
                f"{len(grasps)} grasps published — best idx={best_idx} "
                f"score={scores[best_idx]:.3f}"
            )
            response.success = True
            response.message = ""

        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            response.success = False
            response.message = str(e)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = VisualizeGraspsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
