import threading
import msgpack
import rclpy
import zmq

from franka_demo_interfaces.srv import ExecutePickTask
from rclpy.node import Node


class CommandBridgeNode(Node):
    def __init__(self):
        super().__init__('command_bridge')

        # Params
        self.declare_parameter('command_bridge_host', '0.0.0.0')
        self.declare_parameter('command_bridge_port', 5556)
        self.declare_parameter('command_bridge_timeout', 60.0)

        self.host = self.get_parameter('command_bridge_host').value
        self.port = self.get_parameter('command_bridge_port').value
        self.timeout = self.get_parameter('command_bridge_timeout').value

        # ZMQ REP Socket
        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.REP)
        self.zmq_socket.bind(f'tcp://{self.host}:{self.port}')
        self.get_logger().info(f'ZMQ REP bound on {self.host}:{self.port}')

        # ROS Service clients
        self.client = self.create_client(ExecutePickTask, 'execute_pick_task')

        # ZMQ thread
        self._shutdown = threading.Event()
        self._zmq_thread = threading.Thread(target=self._zmq_loop, daemon=True)
        self._zmq_thread.start()

        self.get_logger().info('Command Bridge started')

    def _zmq_loop(self):
        while not self._shutdown.is_set() and rclpy.ok():
            try:
                raw = self.zmq_socket.recv()
            except zmq.ZMQError:
                break
            try:
                command = msgpack.unpackb(raw, raw=False)
                self.get_logger().info(f"Command received : task = {command.get('task_type')}")
                self._handle_command(command)
            except Exception as e:
                self.get_logger().error(f'Command parsing error : {e}')
                self._send({'status': 'failed', 'message': f'Parsing error : {e}'})

    def _send(self, payload):
        self.zmq_socket.send(msgpack.packb(payload))

    def _handle_command(self, command):
        task_type = command.get('task_type')
        if task_type == 'pick':
            self._handle_pick_command(command)
        else:
            self.get_logger().warn(f"Unsupported task_type : '{task_type}'")
            self._send({'status': 'failed', 'message': f"unsupported task_type : '{task_type}'"})

    def _handle_pick_command(self, command):

        object_label = command.get('object_label')
        point_x = command.get('point_x')
        point_y = command.get('point_y')

        if not object_label or point_x is None or point_y is None:
            self.get_logger().warn('Missing required fields : object_label, point_x, point_y')
            self._send({'status': 'failed', 'message': 'missing required fields'})
            return

        if not self.client.service_is_ready():
            self.get_logger().error('execute_pick_task service not ready')
            self._send({'status': 'failed', 'message': 'execute_pick_task service not ready'})
            return

        request = ExecutePickTask.Request()
        request.object_label = object_label
        request.point_x = float(point_x)
        request.point_y = float(point_y)

        self.get_logger().info(
            f"Sending pick task : label = '{object_label}' point = ({point_x}, {point_y})"
        )

        future = self.client.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda _f: event.set())

        completed = event.wait(timeout=self.timeout)

        if self._shutdown.is_set():
            future.cancel()
            self._send({'status': 'failed', 'message': 'node shutting down'})
            return

        if not completed:
            future.cancel()
            self.get_logger().error('execute_pick_task timeout')
            self._send({'status': 'failed', 'message': 'execute_pick_task timeout'})
            return

        result = future.result()
        status = 'ok' if result.success else 'failed'
        self.get_logger().info(f'Pick task result : {status} — {result.message}')
        self._send({'status': status, 'message': result.message})

    def destroy_node(self):
        self._shutdown.set()    
        self.zmq_socket.close()   
        self.zmq_context.term()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CommandBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
