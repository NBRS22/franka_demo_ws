import rclpy
from franka_demo_interfaces.srv import VisualizeGrasps
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


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

            best_idx = scores.index(max(scores)) if scores else 0

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
                m.pose = pose
                m.scale.x = 0.08   # arrow length
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
