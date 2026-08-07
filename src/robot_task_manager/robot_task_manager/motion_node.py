import threading

import numpy as np
import rclpy
from control_msgs.action import GripperCommand
from franka_demo_interfaces.srv import ExecuteGrasp
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    CollisionObject,
    Constraints,
    MoveItErrorCodes,
    MotionPlanRequest,
    OrientationConstraint,
    PositionConstraint,
    PlanningScene,
)
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from shape_msgs.msg import SolidPrimitive

# ---------------------------------------------------------------------------
# Hardcoded transform: panda_link0 → Camera_OmniVision_OV9782_Color
# ---------------------------------------------------------------------------
_T_TRANSLATION = np.array([2.0, 0.012, 1.0])
_T_QUAT_XYZW = np.array([0.579, 0.579, -0.406, -0.406])

PRE_GRASP_OFFSET_M = 0.10
LIFT_HEIGHT_M = 0.15
OBJECT_PADDING = 0.03       # 3 cm margin added to AABB on each axis
OBJECT_COLLISION_ID = 'pick_object'

ARM_GROUP = 'panda_arm'
EEF_LINK = 'panda_hand'
ROBOT_FRAME = 'panda_link0'
POSITION_TOLERANCE = 0.02   # metres
ORIENTATION_TOLERANCE = 0.5  # radians
PANDA_MAX_REACH = 0.855     # metres from panda_link0 origin


def _build_transform(translation, quat_xyzw):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(quat_xyzw).as_matrix()
    T[:3, 3] = translation
    return T


def _pose_to_matrix(pose):
    T = np.eye(4)
    q = [pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w]
    T[:3, :3] = Rotation.from_quat(q).as_matrix()
    T[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return T


def _matrix_to_pose(T):
    pose = Pose()
    pose.position.x = float(T[0, 3])
    pose.position.y = float(T[1, 3])
    pose.position.z = float(T[2, 3])
    q = Rotation.from_matrix(T[:3, :3]).as_quat()
    pose.orientation.x = float(q[0])
    pose.orientation.y = float(q[1])
    pose.orientation.z = float(q[2])
    pose.orientation.w = float(q[3])
    return pose


def _pose_constraint(pose, frame_id, link_name):
    c = Constraints()

    pc = PositionConstraint()
    pc.header.frame_id = frame_id
    pc.link_name = link_name
    pc.weight = 1.0
    bv = BoundingVolume()
    sp = SolidPrimitive()
    sp.type = SolidPrimitive.SPHERE
    sp.dimensions = [POSITION_TOLERANCE]
    bv.primitives.append(sp)
    target = Pose()
    target.position = pose.position
    target.orientation.w = 1.0
    bv.primitive_poses.append(target)
    pc.constraint_region = bv
    c.position_constraints.append(pc)

    oc = OrientationConstraint()
    oc.header.frame_id = frame_id
    oc.link_name = link_name
    oc.orientation = pose.orientation
    oc.absolute_x_axis_tolerance = ORIENTATION_TOLERANCE
    oc.absolute_y_axis_tolerance = ORIENTATION_TOLERANCE
    oc.absolute_z_axis_tolerance = ORIENTATION_TOLERANCE
    oc.weight = 1.0
    c.orientation_constraints.append(oc)

    return c


class MotionNode(Node):
    def __init__(self):
        super().__init__('motion_node')

        cb = ReentrantCallbackGroup()

        self._T_base_cam = _build_transform(_T_TRANSLATION, _T_QUAT_XYZW)

        self._move_client = ActionClient(
            self, MoveGroup, '/move_action', callback_group=cb
        )
        self._gripper_client = ActionClient(
            self, GripperCommand, '/panda_hand_controller/gripper_cmd', callback_group=cb
        )
        self._planning_scene_pub = self.create_publisher(
            PlanningScene, '/planning_scene', 10
        )

        self.create_service(ExecuteGrasp, 'execute_grasp', self.handle_execute_grasp)
        self.get_logger().info('Motion Node started — waiting for move_action...')
        self._move_client.wait_for_server(timeout_sec=15.0)
        self.get_logger().info('Motion Node ready')

    # -----------------------------------------------------------------------
    # Action helpers
    # -----------------------------------------------------------------------

    def _call_action(self, client, goal, timeout=60.0):
        send_future = client.send_goal_async(goal)
        event = threading.Event()
        result_holder = [None]

        def _on_goal(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error(f'Goal rejected by {client._action_name}')
                event.set()
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda f: (
                result_holder.__setitem__(0, f.result()),
                event.set(),
            ))

        send_future.add_done_callback(_on_goal)

        if not event.wait(timeout=timeout):
            self.get_logger().error(f'Timeout on action {client._action_name}')
            return None
        return result_holder[0]

    def _move_to_pose(self, pose, label):
        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = ARM_GROUP
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.7
        req.max_acceleration_scaling_factor = 0.5
        req.goal_constraints.append(_pose_constraint(pose, ROBOT_FRAME, EEF_LINK))
        goal.request = req
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False

        result = self._call_action(self._move_client, goal, timeout=30.0)
        if result is None:
            return False

        ec = result.result.error_code.val
        if ec != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f'{label} failed: MoveItErrorCode={ec}')
            return False

        self.get_logger().info(f'{label} done')
        return True

    def _set_gripper(self, open_gripper):
        goal = GripperCommand.Goal()
        goal.command.position = 0.04 if open_gripper else 0.0
        goal.command.max_effort = 50.0
        result = self._call_action(self._gripper_client, goal, timeout=10.0)
        state = 'open' if open_gripper else 'close'
        if result is None:
            self.get_logger().warn(f'Gripper {state} timed out, continuing')
        else:
            self.get_logger().info(f'Gripper {state} done')

    # -----------------------------------------------------------------------
    # Planning-scene collision helpers
    # -----------------------------------------------------------------------

    @staticmethod
    def _parse_cloud_xyz(cloud_msg):
        """Return Nx3 float32 array of finite XYZ points from a PointCloud2."""
        n = cloud_msg.width * cloud_msg.height
        if n == 0:
            return np.empty((0, 3), dtype=np.float32)

        step = cloud_msg.point_step
        off = {f.name: f.offset for f in cloud_msg.fields if f.name in ('x', 'y', 'z')}

        raw = np.frombuffer(bytes(cloud_msg.data), dtype=np.uint8).reshape(n, step)

        coords = []
        for name in ('x', 'y', 'z'):
            # Each row slice of 4 uint8 bytes → 1 float32
            col = raw[:, off[name]:off[name] + 4].copy().view(np.float32).ravel()
            coords.append(col)

        pts = np.column_stack(coords)
        return pts[np.all(np.isfinite(pts), axis=1)]

    def _add_object_collision(self, pts_cam):
        """Compute AABB from camera-frame points and add it to MoveIt scene."""
        if len(pts_cam) == 0:
            self.get_logger().warn('Empty cloud — skipping collision object')
            return

        # Transform points to robot base frame
        ones = np.ones((len(pts_cam), 1), dtype=np.float64)
        pts_h = np.hstack([pts_cam.astype(np.float64), ones])
        pts_robot = (self._T_base_cam @ pts_h.T).T[:, :3]

        mn = pts_robot.min(axis=0)
        mx = pts_robot.max(axis=0)
        center = (mn + mx) / 2.0
        dims = mx - mn + OBJECT_PADDING

        obj = CollisionObject()
        obj.id = OBJECT_COLLISION_ID
        obj.header.frame_id = ROBOT_FRAME
        obj.operation = CollisionObject.ADD

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [float(dims[0]), float(dims[1]), float(dims[2])]
        obj.primitives.append(box)

        pose = Pose()
        pose.position.x = float(center[0])
        pose.position.y = float(center[1])
        pose.position.z = float(center[2])
        pose.orientation.w = 1.0
        obj.primitive_poses.append(pose)

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obj)
        self._planning_scene_pub.publish(scene)

        self.get_logger().info(
            f'Object collision added: centre=({center[0]:.3f},{center[1]:.3f},'
            f'{center[2]:.3f}) dims=({dims[0]:.3f},{dims[1]:.3f},{dims[2]:.3f})'
        )

    def _remove_object_collision(self):
        """Remove the pick object from the MoveIt planning scene."""
        obj = CollisionObject()
        obj.id = OBJECT_COLLISION_ID
        obj.header.frame_id = ROBOT_FRAME
        obj.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obj)
        self._planning_scene_pub.publish(scene)
        self.get_logger().info('Object collision removed')

    # -----------------------------------------------------------------------
    # Main service handler
    # -----------------------------------------------------------------------

    def handle_execute_grasp(self, request, response):
        try:
            G_cam = _pose_to_matrix(request.grasp_pose)
            G_base = self._T_base_cam @ G_cam

            # Always approach from directly above
            G_pre = G_base.copy()
            G_pre[2, 3] += PRE_GRASP_OFFSET_M

            G_lift = G_base.copy()
            G_lift[2, 3] += LIFT_HEIGHT_M

            p = G_base[:3, 3]
            pp = G_pre[:3, 3]
            dist_grasp = float(np.linalg.norm(p))
            dist_pre = float(np.linalg.norm(pp))
            self.get_logger().info(
                f'Grasp in robot frame: x={p[0]:.3f} y={p[1]:.3f} z={p[2]:.3f} '
                f'(dist={dist_grasp:.3f}m)'
            )
            self.get_logger().info(
                f'Pre-grasp in robot frame: x={pp[0]:.3f} y={pp[1]:.3f} z={pp[2]:.3f} '
                f'(dist={dist_pre:.3f}m)'
            )

            if dist_pre > PANDA_MAX_REACH:
                msg = (
                    f'Pre-grasp unreachable: {dist_pre:.3f}m > Panda max reach '
                    f'{PANDA_MAX_REACH}m.'
                )
                self.get_logger().error(msg)
                response.success = False
                response.message = msg
                return response

            # Add object to planning scene so the arm avoids it during approach
            has_cloud = len(request.object_cloud.data) > 0
            if has_cloud:
                pts_cam = self._parse_cloud_xyz(request.object_cloud)
                self._add_object_collision(pts_cam)

            self.get_logger().info('Opening gripper...')
            self._set_gripper(open_gripper=True)

            self.get_logger().info('Moving to pre-grasp...')
            if not self._move_to_pose(_matrix_to_pose(G_pre), 'pre-grasp'):
                if has_cloud:
                    self._remove_object_collision()
                response.success = False
                response.message = 'pre-grasp planning failed'
                return response

            # Remove object before the grasp motion: the grasp pose is inside/at
            # the object surface, so MoveIt would reject it as being in collision.
            if has_cloud:
                self._remove_object_collision()

            self.get_logger().info('Moving to grasp...')
            if not self._move_to_pose(_matrix_to_pose(G_base), 'grasp'):
                response.success = False
                response.message = 'grasp planning failed'
                return response

            self.get_logger().info('Closing gripper...')
            self._set_gripper(open_gripper=False)

            self.get_logger().info('Lifting...')
            if not self._move_to_pose(_matrix_to_pose(G_lift), 'lift'):
                response.success = False
                response.message = 'lift planning failed'
                return response

            response.success = True
            response.message = 'grasp executed'

        except Exception as e:
            self.get_logger().error(f'execute_grasp error: {e}')
            response.success = False
            response.message = str(e)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = MotionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
