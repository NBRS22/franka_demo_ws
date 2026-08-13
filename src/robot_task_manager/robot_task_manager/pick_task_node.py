import threading
import rclpy

from franka_demo_interfaces.action import MtcPick
from franka_demo_interfaces.srv import (
    CreatePointcloud,
    ExecutePickTask,
    GenerateGraspPose,
    GetFrames,
    SegmentObject,
)
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import tf2_geometry_msgs  # noqa: F401 — registers PoseStamped with tf2 do_transform_*
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


ROBOT_BASE_FRAME = 'fp3_link0'


class PickTaskNode(Node):
    def __init__(self):
        super().__init__('pick_task_node')

        # TF: used to transform grasp poses from camera frame → fp3_link0.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ROS Service Clients
        self.frame_client = self.create_client(GetFrames, 'get_frames', callback_group=ReentrantCallbackGroup())
        self.sam3_client = self.create_client(SegmentObject, 'segment_object', callback_group=ReentrantCallbackGroup())
        self.create_pointcloud_client = self.create_client(CreatePointcloud, 'create_pointcloud', callback_group=ReentrantCallbackGroup())
        self.graspgen_client = self.create_client(GenerateGraspPose, 'generate_grasp_pose', callback_group=ReentrantCallbackGroup())

        # Action Client — calls command_router_node's public mtc_pick action
        self._mtc_pick_client = ActionClient(self, MtcPick, 'mtc_pick')

        # ROS Service Servers
        self.create_service(ExecutePickTask, 'execute_pick_task', self.handle_pick_task)

        self.get_logger().info('Pick Task Node started')

    def _get_frames(self):
        result = self._call_service(self.frame_client, GetFrames.Request(), timeout=5.0)
        if result is None or not result.success:
            raise RuntimeError(result.message if result else 'get_frames failed')
        return result

    def _segment(self, frame, label, point_x, point_y):
        req = SegmentObject.Request()
        req.image = frame.rgb
        req.label = label
        req.point_x = point_x
        req.point_y = point_y
        req.use_pointing = True
        req.threshold = 0.05
        self.get_logger().info('Calling SAM3...')
        result = self._call_service(self.sam3_client, req, timeout=30.0)
        if result is None or not result.success:
            raise RuntimeError('SAM3 failed')
        if not result.has_mask:
            raise RuntimeError('no mask found')
        self.get_logger().info(f'Mask received score = {result.score:.3f}')
        return result

    def _create_pointcloud(self, frame, mask_result):
        req = CreatePointcloud.Request()
        req.mask = mask_result.mask
        req.rgb = frame.rgb
        req.depth = frame.depth
        req.camera_info = frame.camera_info
        result = self._call_service(self.create_pointcloud_client, req, timeout=10.0)
        if result is None or not result.success:
            raise RuntimeError(result.message if result else 'pointcloud failed')
        self.get_logger().info(f'Pointcloud ready — {len(result.cloud.data)} bytes')
        return result

    def _generate_grasps(self, cloud, scene_cloud):
        req = GenerateGraspPose.Request()
        req.object_cloud = cloud
        req.scene_cloud = scene_cloud
        self.get_logger().info('Calling GraspGen...')
        result = self._call_service(self.graspgen_client, req, timeout=60.0)
        if result is None or not result.success:
            raise RuntimeError(result.message if result else 'graspgen failed')
        self.get_logger().info(f'{len(result.grasps.poses)} grasps received')
        if not result.grasps.poses:
            raise RuntimeError('GraspGen found 0 grasps for the segmented object')
        return result

    def _transform_poses(self, pose_array) -> list:
        """Transform all poses from the PoseArray's frame into fp3_link0.

        GraspGen returns poses in camera_color_optical_frame (the frame_id
        of the object pointcloud built by create_pointcloud_node). The TF
        fp3_link0 → camera_link is published by handeye_tf_publisher, which
        chains through RealSense's own camera_link → camera_color_optical_frame.
        """
        src_frame = pose_array.header.frame_id
        if not src_frame:
            raise RuntimeError('GraspGen PoseArray has no frame_id')

        try:
            tf = self._tf_buffer.lookup_transform(
                ROBOT_BASE_FRAME, src_frame, rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            raise RuntimeError(
                f'TF {ROBOT_BASE_FRAME} <- {src_frame} unavailable: {exc}'
            )

        robot_poses = []
        for pose in pose_array.poses:
            stamped = PoseStamped()
            stamped.header = pose_array.header
            stamped.pose = pose
            transformed = tf2_geometry_msgs.do_transform_pose_stamped(stamped, tf)
            transformed.header.frame_id = ROBOT_BASE_FRAME
            robot_poses.append(transformed)

        self.get_logger().info(
            f'Transformed {len(robot_poses)} poses from {src_frame} → {ROBOT_BASE_FRAME}'
        )
        return robot_poses

    def _execute_pick(self, grasp_poses: list) -> tuple:
        """Send grasp poses to the mtc_pick action and wait for the result."""
        if not self._mtc_pick_client.wait_for_server(timeout_sec=10.0):
            return False, 'mtc_pick action server unavailable'

        goal = MtcPick.Goal()
        goal.grasp_poses = grasp_poses

        event = threading.Event()
        result_holder = {}

        def _on_result(future):
            result_holder['result'] = future.result()
            event.set()

        def _on_goal_response(future):
            gh = future.result()
            if gh is None:
                result_holder['error'] = 'goal rejected by mtc_pick server'
                event.set()
                return
            result_future = gh.get_result_async()
            result_future.add_done_callback(_on_result)

        send_future = self._mtc_pick_client.send_goal_async(goal)
        send_future.add_done_callback(_on_goal_response)

        if not event.wait(timeout=120.0):
            return False, 'mtc_pick timed out after 120s'

        if 'error' in result_holder:
            return False, result_holder['error']

        wrapped = result_holder['result']
        r = wrapped.result
        self.get_logger().info(
            f'mtc_pick result: success={r.success} pose_index={r.used_pose_index} msg={r.message}'
        )
        return r.success, r.message

    def handle_pick_task(self, request, response):
        self.get_logger().info(
            f"Pick task received : label = '{request.object_label}' "
            f"point = ({request.point_x}, {request.point_y})"
        )
        try:
            frame = self._get_frames()
            mask = self._segment(frame, request.object_label, request.point_x, request.point_y)
            pc = self._create_pointcloud(frame, mask)
            grasps = self._generate_grasps(pc.cloud, pc.scene_cloud)

            robot_poses = self._transform_poses(grasps.grasps)
            ok, msg = self._execute_pick(robot_poses)
            if not ok:
                raise RuntimeError(f'pick failed: {msg}')

            response.success = True
            response.message = (
                f'pick OK — seg={mask.score:.3f} '
                f'grasps={len(grasps.grasps.poses)} '
                f'pick={msg}'
            )
        except RuntimeError as e:
            response.success = False
            response.message = str(e)

        return response

    def _call_service(self, client, request, timeout=10.0):
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'Service {client.srv_name} not available')
            return None

        future = client.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda _f: event.set())

        if not event.wait(timeout=timeout):
            future.cancel()
            self.get_logger().error(f'Timeout on service {client.srv_name}')
            return None

        return future.result()


def main(args=None):
    rclpy.init(args=args)
    node = PickTaskNode()
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
