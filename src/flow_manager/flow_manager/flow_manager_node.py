import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from franka_demo_interfaces.srv import (
    ExecuteTask,
    GetFrames,
    SegmentObject,
    FuseMaskDepth,
    GenerateGraspPose,
    SelectBestGrasp
)


class FlowManagerNode(Node):
    def __init__(self):
        super().__init__('flow_manager')

        # handle_execute_task blocks its own thread waiting on the futures
        # from these clients (cf. _call_service) — they need a DIFFERENT
        # callback group than the execute_task service itself, otherwise
        # the default mutually-exclusive group would prevent their
        # responses from ever being processed while handle_execute_task is
        # blocked, deadlocking even under a MultiThreadedExecutor.
        client_cb_group = ReentrantCallbackGroup()

        self.validator_client = self.create_client(
            ExecuteTask, 'validate_task', callback_group=client_cb_group
        )
        self.camera_client = self.create_client(
            GetFrames, 'get_frames', callback_group=client_cb_group
        )
        self.sam3_client = self.create_client(
            SegmentObject, 'segment_object', callback_group=client_cb_group
        )
        self.pointcloud_client = self.create_client(
            FuseMaskDepth, 'fuse_mask_depth', callback_group=client_cb_group
        )
        self.graspgen_client = self.create_client(
            GenerateGraspPose, 'generate_grasp_pose', callback_group=client_cb_group
        )
        self.selector_client = self.create_client(
            SelectBestGrasp, 'select_best_grasp', callback_group=client_cb_group
        )

        # service execute_task
        self.srv = self.create_service(
            ExecuteTask,
            'execute_task',
            self.handle_execute_task
        )

        self.get_logger().info("Flow Manager Node started")

    def handle_execute_task(self, request, response):
        self.get_logger().info(
            f"Task received: type={request.task_type} "
            f"label={request.object_label}"
        )

        if request.task_type == "pick":
            return self._handle_pick(request, response)
        elif request.task_type == "place":
            self.get_logger().info("Place not implemented")
            response.success = False
            response.message = "place not implemented"
            return response
        elif request.task_type == "stop":
            self.get_logger().info("Stop received")
            response.success = True
            response.message = "stopped"
            return response
        else:
            self.get_logger().warn(f"Unknown task_type: {request.task_type}")
            response.success = False
            response.message = f"unknown task_type: {request.task_type}"
            return response

    def _handle_pick(self, request, response):

        # step 1: validation
        self.get_logger().info("Validating task...")
        val_result = self._call_service(
            self.validator_client,
            ExecuteTask.Request(
                task_type=request.task_type,
                object_label=request.object_label,
                point_x=request.point_x,
                point_y=request.point_y
            ),
            timeout=5.0
        )
        if val_result is None or not val_result.success:
            response.success = False
            response.message = val_result.message if val_result else "validator failed"
            return response

        # step 2: retrieve frames
        self.get_logger().info("Retrieving camera frames...")
        frames = self._call_service(
            self.camera_client,
            GetFrames.Request(),
            timeout=5.0
        )
        if frames is None or not frames.success:
            response.success = False
            response.message = frames.message if frames else "camera buffer failed"
            return response

        # step 3: SAM3
        self.get_logger().info("Calling SAM3...")
        sam3_req = SegmentObject.Request()
        sam3_req.image = frames.rgb
        sam3_req.label = request.object_label
        sam3_req.point_x = request.point_x
        sam3_req.point_y = request.point_y
        sam3_req.use_pointing = True
        sam3_req.threshold = 0.05

        mask_result = self._call_service(
            self.sam3_client,
            sam3_req,
            timeout=30.0
        )
        if mask_result is None or not mask_result.success:
            response.success = False
            response.message = "SAM3 failed"
            return response

        if not mask_result.has_mask:
            response.success = False
            response.message = "no mask found"
            return response

        self.get_logger().info(f"Mask received score={mask_result.score:.3f}")

        # step 4: fuse mask + depth
        self.get_logger().info("Fusing mask + depth...")
        fuse_req = FuseMaskDepth.Request()
        fuse_req.mask = mask_result.mask
        fuse_req.depth = frames.depth
        fuse_req.camera_info = frames.camera_info

        cloud_result = self._call_service(
            self.pointcloud_client,
            fuse_req,
            timeout=10.0
        )
        if cloud_result is None or not cloud_result.success:
            response.success = False
            response.message = cloud_result.message if cloud_result else "fusion failed"
            return response

        # step 5: GraspGen
        self.get_logger().info("Calling GraspGen...")
        grasp_req = GenerateGraspPose.Request()
        grasp_req.object_cloud = cloud_result.cloud
        grasp_req.gripper_type = "franka_panda"
        grasp_req.max_grasps = 10

        grasp_result = self._call_service(
            self.graspgen_client,
            grasp_req,
            timeout=60.0
        )
        if grasp_result is None or not grasp_result.success:
            response.success = False
            response.message = "GraspGen failed"
            return response

        if len(grasp_result.grasps.poses) == 0:
            response.success = False
            response.message = "no grasp found"
            return response

        self.get_logger().info(
            f"{len(grasp_result.grasps.poses)} grasps received"
        )

        # step 6: select best pose + publish to RViz
        self.get_logger().info("Selecting best pose...")
        sel_req = SelectBestGrasp.Request()
        sel_req.grasps = grasp_result.grasps
        sel_req.scores = grasp_result.scores
        sel_req.frame_id = grasp_result.grasps.header.frame_id

        sel_result = self._call_service(
            self.selector_client,
            sel_req,
            timeout=5.0
        )
        if sel_result is None or not sel_result.success:
            response.success = False
            response.message = "grasp selection failed"
            return response

        self.get_logger().info(
            f"Best pose selected "
            f"idx={sel_result.best_idx} "
            f"score={sel_result.best_score:.3f}"
        )

        response.success = True
        response.message = "pick successful"
        return response

    def _call_service(self, client, request, timeout=10.0):
        # wait_for_service() polls the graph directly, no spin needed.
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                f"Service {client.srv_name} not available"
            )
            return None

        # Blocks THIS thread (a MultiThreadedExecutor worker, not the
        # executor's own dispatch loop) on a plain threading.Event, woken up
        # by the future's done-callback. This replaces both
        # spin_until_future_complete (crashes: "Executor is already
        # spinning", since this method runs from within a callback the
        # executor is already dispatching — cf. dette technique "Nested
        # spinning") and a bare `await future` (silently hangs: response
        # processing for these clients needs the SAME executor thread that
        # would be blocked awaiting it). Requires client_cb_group above to
        # be different from execute_task's own callback group, otherwise
        # the response callback can't run concurrently even with
        # MultiThreadedExecutor.
        future = client.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda _f: event.set())

        if not event.wait(timeout=timeout):
            future.cancel()
            self.get_logger().error(
                f"Timeout on service {client.srv_name}"
            )
            return None

        return future.result()


def main(args=None):
    rclpy.init(args=args)
    node = FlowManagerNode()
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
