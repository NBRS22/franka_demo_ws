"""
Interactive calibration check: click any point in RViz's point cloud
(the "Publish Point" tool, publishes geometry_msgs/PointStamped on
/clicked_point by default) and the arm moves fp3_hand_tcp to that exact
point -- position only, no orientation constraint (MoveIt picks whatever
reachable orientation it wants) and no gripper action, just move there and
stop. If the arm visually/physically arrives at the same point you clicked,
that's a direct, intuitive confirmation the calibration places the point
cloud correctly in fp3_link0 at that specific location -- lets you probe
anywhere in the workspace interactively, not just the fixed points the other
diagnostic tools check.

The clicked point can be in ANY frame RViz happens to be using as Fixed
Frame -- this node transforms it into fp3_link0 via TF regardless.

Needs: /move_action (fp3_moveit_server bringup, real hardware, real
move_group), /clicked_point (RViz), and fp3_link0 -> camera_link published
by handeye_tf_publisher (the calibration being tested -- needed for the
point cloud itself to be positioned correctly in fp3_link0 in RViz, not by
this node directly).
"""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import tf2_geometry_msgs  # noqa: F401 -- registers PointStamped support on Buffer.transform
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import PointStamped, Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint
from shape_msgs.msg import SolidPrimitive

BASE_FRAME = 'fp3_link0'
EFFECTOR_LINK = 'fp3_hand_tcp'
PLANNING_GROUP = 'fp3_arm'
POSITION_TOLERANCE_M = 0.005
VELOCITY_SCALING = 0.15
ACCELERATION_SCALING = 0.15
PLANNING_TIME_S = 5.0


class ClickToPointNode(Node):

    def __init__(self):
        super().__init__('calib_click_test')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._move_client = ActionClient(self, MoveGroup, '/move_action')
        self._busy = False
        self.create_subscription(PointStamped, '/clicked_point', self._point_cb, 10)
        self.get_logger().info(
            f"Ready -- click a point in RViz's point cloud (Publish Point tool) and "
            f"{EFFECTOR_LINK} will move there (position only, no gripper action).")

    def _point_cb(self, msg):
        if self._busy:
            self.get_logger().warn('Still moving from a previous click -- ignoring this one')
            return
        self._busy = True
        try:
            self._handle_click(msg)
        finally:
            self._busy = False

    def _handle_click(self, msg):
        try:
            target = self.tf_buffer.transform(msg, BASE_FRAME, timeout=rclpy.duration.Duration(seconds=2.0))
        except TransformException as ex:
            self.get_logger().error(f"Could not transform clicked point into '{BASE_FRAME}': {ex}")
            return

        p = target.point
        self.get_logger().info(f"Clicked point in '{BASE_FRAME}': ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) -- moving...")

        constraints = Constraints()
        pos_c = PositionConstraint()
        pos_c.header.frame_id = BASE_FRAME
        pos_c.link_name = EFFECTOR_LINK
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [POSITION_TOLERANCE_M]
        pos_c.constraint_region.primitives.append(primitive)
        primitive_pose = Pose()
        primitive_pose.position.x, primitive_pose.position.y, primitive_pose.position.z = p.x, p.y, p.z
        primitive_pose.orientation.w = 1.0
        pos_c.constraint_region.primitive_poses.append(primitive_pose)
        pos_c.weight = 1.0
        constraints.position_constraints.append(pos_c)
        # No OrientationConstraint on purpose: MoveIt is free to pick any
        # reachable orientation, giving it the best chance to find a valid
        # IK solution for an arbitrary clicked point.

        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = PLANNING_GROUP
        req.num_planning_attempts = 10
        req.allowed_planning_time = PLANNING_TIME_S
        req.max_velocity_scaling_factor = VELOCITY_SCALING
        req.max_acceleration_scaling_factor = ACCELERATION_SCALING
        req.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False

        if not self._move_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('/move_action unavailable')
            return

        send_future = self._move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        gh = send_future.result()
        if gh is None or not gh.accepted:
            self.get_logger().warn('Goal rejected (likely unreachable) -- try a closer/different point')
            return

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result is None or result.result.error_code.val != 1:
            code = result.result.error_code.val if result else None
            self.get_logger().warn(f'Move failed (error_code={code}) -- try a closer/different point')
            return

        self.get_logger().info(
            f"Arrived. Compare {EFFECTOR_LINK}'s real physical position to the point you clicked.")


def main(args=None):
    rclpy.init(args=args)
    node = ClickToPointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
