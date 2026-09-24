import numpy as np
import rclpy
import yaml
import os

from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation
from rclpy.node import Node

from tf2_ros import ConnectivityException, ExtrapolationException, LookupException
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformListener
from tf2_ros.buffer import Buffer


class HandeyeTfPublisher(Node):

    @staticmethod
    def _transform_to_matrix(translation, quaternion):
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
        matrix[:3, 3] = translation
        return matrix

    @staticmethod
    def _matrix_to_translation_quaternion(matrix):
        translation = matrix[:3, 3]
        quaternion = Rotation.from_matrix(matrix[:3, :3]).as_quat()
        return translation, quaternion

    def __init__(self):
        super().__init__('handeye_tf_publisher')

        # Params
        self.declare_parameter('calibration_name', 'fp3_link0_d455_camera_color_optical_frame_001')
        self.declare_parameter('calib_dir', '~/.ros2/easy_handeye2/calibrations')
        self.declare_parameter('publish_rate_s', 2.0)
        self.declare_parameter('camera_link_frame', 'camera_link')

        calibration_name = self.get_parameter('calibration_name').value
        calib_dir = os.path.expanduser(self.get_parameter('calib_dir').value)
        publish_rate_s = self.get_parameter('publish_rate_s').value

        self.camera_link_frame = self.get_parameter('camera_link_frame').value

        calib_path = os.path.join(calib_dir, f'{calibration_name}.calib')
        self.robot_base_frame, self.tracking_base_frame, self.t1_matrix = (self._load_calibration(calib_path))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        self.get_logger().info(
            f"Calibration '{calibration_name}' loaded from {calib_path} "
            f'({self.robot_base_frame} -> {self.tracking_base_frame}). '
            f'Waiting for TF {self.camera_link_frame} -> {self.tracking_base_frame}...'
        )

        self.timer = self.create_timer(publish_rate_s, self._try_publish)

    def _load_calibration(self, calib_path):
        if not os.path.isfile(calib_path):
            self.get_logger().fatal(f'Calibration file not found : {calib_path}')
            raise SystemExit(1)

        with open(calib_path, 'r') as f:
            data = yaml.safe_load(f)

        try:
            parameters = data['parameters']
            robot_base_frame = parameters['robot_base_frame']
            tracking_base_frame = parameters['tracking_base_frame']

            translation = data['transform']['translation']
            rotation = data['transform']['rotation']
            t = [translation['x'], translation['y'], translation['z']]
            q = [rotation['x'], rotation['y'], rotation['z'], rotation['w']]
        except (KeyError, TypeError) as exc:
            self.get_logger().fatal(f'Missing field in {calib_path}: {exc}')
            raise SystemExit(1)

        matrix = self._transform_to_matrix(t, q)
        return robot_base_frame, tracking_base_frame, matrix

    def _try_publish(self):
        try:
            camera_tf = self.tf_buffer.lookup_transform(self.camera_link_frame, self.tracking_base_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warn(
                f'TF {self.camera_link_frame} -> {self.tracking_base_frame} unavailable, '
                f'retrying next tick ({exc})'
            )
            return

        translation = camera_tf.transform.translation
        rotation = camera_tf.transform.rotation
        t2_matrix = self._transform_to_matrix(
            [translation.x, translation.y, translation.z],
            [rotation.x, rotation.y, rotation.z, rotation.w],
        )

        result_matrix = self.t1_matrix @ np.linalg.inv(t2_matrix)
        result_translation, result_quat = self._matrix_to_translation_quaternion(result_matrix)

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.robot_base_frame
        msg.child_frame_id = self.camera_link_frame
        msg.transform.translation.x = float(result_translation[0])
        msg.transform.translation.y = float(result_translation[1])
        msg.transform.translation.z = float(result_translation[2])
        msg.transform.rotation.x = float(result_quat[0])
        msg.transform.rotation.y = float(result_quat[1])
        msg.transform.rotation.z = float(result_quat[2])
        msg.transform.rotation.w = float(result_quat[3])

        self.static_broadcaster.sendTransform(msg)
        self.get_logger().info(f'Static transform published: {self.robot_base_frame} -> {self.camera_link_frame}')

        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HandeyeTfPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
