import rclpy
from rclpy.node import Node
from franka_demo_interfaces.srv import ExecuteTask

VALID_TASK_TYPES = ["pick", "place", "stop"]


class TaskValidatorNode(Node):
    def __init__(self):
        super().__init__('task_validator')

        self.srv = self.create_service(
            ExecuteTask,
            'validate_task',
            self.handle_validate_task
        )

        self.get_logger().info("Task Validator Node started")

    def handle_validate_task(self, request, response):
        # validate task_type
        if request.task_type not in VALID_TASK_TYPES:
            response.success = False
            response.message = f"invalid task_type: {request.task_type}"
            return response

        # validate label for pick
        if request.task_type == "pick" and not request.object_label:
            response.success = False
            response.message = "object_label required for pick"
            return response

        # validate point for pick
        if request.task_type == "pick":
            if request.point_x < 0 or request.point_y < 0:
                response.success = False
                response.message = "invalid pointing coordinates"
                return response

        response.success = True
        response.message = "valid"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TaskValidatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()