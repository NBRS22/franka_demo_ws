"""Publishes static collision objects (table + floor) to MoveIt planning scene."""
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


class ScenePublisherNode(Node):
    def __init__(self):
        super().__init__('scene_publisher_node')

        # Ground surface just below z=0.  We keep it at -0.2 so MoveIt can
        # still find IK configs for grasps near ground level (z≈0.02 m) while
        # blocking trajectories that dive unreasonably below the actual floor.
        self.declare_parameter('ground_z', -0.25)
        self.declare_parameter('ground_thickness', 0.1)

        self._pub = self.create_publisher(PlanningScene, '/planning_scene', 10)

        # Fire once after 3 s to let move_group fully initialise
        self._timer = self.create_timer(3.0, self._publish_once)
        self.get_logger().info('Scene Publisher started')

    # -----------------------------------------------------------------------

    def _publish_once(self):
        self._timer.cancel()

        ground_z = self.get_parameter('ground_z').value
        ground_t = self.get_parameter('ground_thickness').value

        scene = PlanningScene()
        scene.is_diff = True

        scene.world.collision_objects.append(
            self._box('ground', 6.0, 6.0, ground_t, 0.0, 0.0, ground_z)
        )

        self._pub.publish(scene)
        self.get_logger().info(
            f'Planning scene published: ground collision plane at z={ground_z + ground_t/2:.2f} '
            f'(box centre z={ground_z:.3f}, frame: panda_link0)'
        )

    def _box(self, name, sx, sy, sz, px, py, pz):
        obj = CollisionObject()
        obj.id = name
        obj.header.frame_id = 'panda_link0'
        obj.operation = CollisionObject.ADD

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [sx, sy, sz]
        obj.primitives.append(box)

        pose = Pose()
        pose.position.x = px
        pose.position.y = py
        pose.position.z = pz
        pose.orientation.w = 1.0
        obj.primitive_poses.append(pose)

        return obj


def main(args=None):
    rclpy.init(args=args)
    node = ScenePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
