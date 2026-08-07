import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from franka_demo_interfaces.srv import ExecuteTask
import zmq
import msgpack


class CommandBridgeNode(Node):
    def __init__(self):
        super().__init__('command_bridge')

        # params
        self.declare_parameter('command_host', '0.0.0.0')
        self.declare_parameter('command_port', 5558)
        self.declare_parameter('flow_manager_timeout', 30.0)

        self.host = self.get_parameter('command_host').value
        self.port = self.get_parameter('command_port').value
        self.timeout = self.get_parameter('flow_manager_timeout').value

        # ZMQ REP socket
        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.REP)
        self.zmq_socket.bind(f"tcp://{self.host}:{self.port}")
        self.get_logger().info(f"ZMQ REP bound on {self.host}:{self.port}")

        # _handle_command blocks its own thread waiting on this client's
        # future (cf. _call_service) — needs a DIFFERENT callback group than
        # poll_zmq's timer, otherwise the default mutually-exclusive group
        # would prevent the response from ever being processed while
        # _handle_command is blocked, deadlocking even under a
        # MultiThreadedExecutor.
        self.client = self.create_client(
            ExecuteTask, 'execute_task', callback_group=ReentrantCallbackGroup()
        )

        # timer to poll the ZMQ socket
        self.create_timer(0.05, self.poll_zmq)

        self.get_logger().info("Command Bridge started, waiting for Gemini ER commands...")

    def poll_zmq(self):
        try:
            # non-blocking
            raw = self.zmq_socket.recv(zmq.NOBLOCK)
        except zmq.Again:
            return

        try:
            command = msgpack.unpackb(raw, raw=False)
            self.get_logger().info(
                f"Command received: task={command.get('task_type')} "
                f"label={command.get('object_label')}"
            )
            self._handle_command(command)

        except Exception as e:
            self.get_logger().error(f"Command parsing error: {e}")
            self.zmq_socket.send(msgpack.packb({
                "status": "failed",
                "message": f"Parsing error: {str(e)}"
            }))

    def _handle_command(self, command):
        # wait_for_service() polls the graph directly, no spin needed.
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("flow_manager service unavailable")
            self.zmq_socket.send(msgpack.packb({
                "status": "failed",
                "message": "flow_manager unavailable"
            }))
            return

        # build the request
        request = ExecuteTask.Request()
        request.task_type = command.get('task_type', 'pick')
        request.object_label = command.get('object_label', '')
        request.point_x = float(command.get('point_x', 0.0))
        request.point_y = float(command.get('point_y', 0.0))

        # call flow_manager service
        self.get_logger().info("Sending command to flow_manager...")

        # Blocks THIS thread (a MultiThreadedExecutor worker, not the
        # executor's own dispatch loop) on a plain threading.Event, woken up
        # by the future's done-callback. This replaces the old spin_once
        # loop, which crashed with "Executor is already spinning" — cf.
        # dette technique "Nested spinning" dans CLAUDE.md — since poll_zmq
        # is itself already running inside the executor's dispatch.
        future = self.client.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda _f: event.set())

        if not event.wait(timeout=self.timeout):
            future.cancel()
            self.get_logger().error("flow_manager timeout")
            self.zmq_socket.send(msgpack.packb({
                "status": "failed",
                "message": "flow_manager timeout"
            }))
            return

        # return the result to Gemini ER
        result = future.result()
        status = "ok" if result.success else "failed"
        self.get_logger().info(f"Result: {status} - {result.message}")

        self.zmq_socket.send(msgpack.packb({
            "status": status,
            "message": result.message
        }))

    def destroy_node(self):
        self.zmq_socket.close()
        self.zmq_context.term()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CommandBridgeNode()
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
