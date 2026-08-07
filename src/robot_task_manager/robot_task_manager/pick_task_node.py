import threading
import rclpy

from franka_demo_interfaces.srv import (
    CreatePointcloud,
    ExecuteGrasp,
    ExecutePickTask,
    GenerateGraspPose,
    GetFrames,
    SegmentObject,
    VisualizeGrasps,
    VisualizeSegmentation,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class PickTaskNode(Node):
    def __init__(self):
        super().__init__('pick_task_node')

        cb = ReentrantCallbackGroup()

        self.frame_client = self.create_client(GetFrames, 'get_frames', callback_group=cb)
        self.sam3_client = self.create_client(SegmentObject, 'segment_object', callback_group=cb)
        self.viz_seg_client = self.create_client(VisualizeSegmentation, 'visualize_segmentation', callback_group=cb)
        self.pointcloud_client = self.create_client(CreatePointcloud, 'create_pointcloud', callback_group=cb)
        self.graspgen_client = self.create_client(GenerateGraspPose, 'generate_grasp_pose', callback_group=cb)
        self.viz_grasps_client = self.create_client(VisualizeGrasps, 'visualize_grasps', callback_group=cb)
        self.execute_grasp_client = self.create_client(ExecuteGrasp, 'execute_grasp', callback_group=cb)

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
        self.get_logger().info(f'Mask received score={result.score:.3f}')
        return result

    def _visualize_segmentation(self, frame, mask_result, point_x, point_y):
        req = VisualizeSegmentation.Request()
        req.rgb = frame.rgb
        req.mask = mask_result.mask
        req.point_x = point_x
        req.point_y = point_y
        self._call_service(self.viz_seg_client, req, timeout=5.0)

    def _create_pointcloud(self, frame, mask_result):
        req = CreatePointcloud.Request()
        req.depth = frame.depth
        req.mask = mask_result.mask
        req.camera_info = frame.camera_info
        result = self._call_service(self.pointcloud_client, req, timeout=10.0)
        if result is None or not result.success:
            raise RuntimeError(result.message if result else 'pointcloud failed')
        self.get_logger().info(f'Pointcloud ready — {len(result.cloud.data)} bytes')
        return result

    def _generate_grasps(self, cloud):
        req = GenerateGraspPose.Request()
        req.object_cloud = cloud
        self.get_logger().info('Calling GraspGen...')
        result = self._call_service(self.graspgen_client, req, timeout=60.0)
        if result is None or not result.success:
            raise RuntimeError(result.error_msg if result else 'graspgen failed')
        self.get_logger().info(f'{len(result.grasps.poses)} grasps received')
        return result

    def _visualize_grasps(self, grasp_result):
        req = VisualizeGrasps.Request()
        req.grasps = grasp_result.grasps
        req.scores = grasp_result.scores
        self._call_service(self.viz_grasps_client, req, timeout=5.0)

    def _execute_grasp(self, cloud, grasp_result):
        scores = list(grasp_result.scores)
        best_idx = scores.index(max(scores)) if scores else 0
        best_pose = grasp_result.grasps.poses[best_idx]
        self.get_logger().info(f'Executing best grasp idx={best_idx} score={scores[best_idx]:.3f}')
        req = ExecuteGrasp.Request()
        req.object_cloud = cloud
        req.grasp_pose = best_pose
        result = self._call_service(self.execute_grasp_client, req, timeout=120.0)
        if result is None or not result.success:
            raise RuntimeError(result.message if result else 'execute_grasp failed')

    def handle_pick_task(self, request, response):
        self.get_logger().info(f"Pick task received: label='{request.object_label}' "f"point=({request.point_x}, {request.point_y})")
        try:
            frame = self._get_frames()
            mask = self._segment(frame, request.object_label, request.point_x, request.point_y)
            self._visualize_segmentation(frame, mask, request.point_x, request.point_y)
            pc = self._create_pointcloud(frame, mask)
            grasps = self._generate_grasps(pc.cloud)
            self._visualize_grasps(grasps)
            self._execute_grasp(pc.cloud, grasps)
            response.success = True
            response.message = (f'pick OK — seg={mask.score:.3f} grasps={len(grasps.grasps.poses)}')
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
