#!/usr/bin/env python3
"""Mesure le décalage entre l'objet vu par la caméra et sa vraie position.

1. Lance la pipeline avec execute_pick:=false et clique l'objet (Gemini ER
   simulator): le nuage de l'objet est publié sur /pick/pointcloud.
2. Ce script affiche le centre et le dessus de ce nuage dans fp3_link0.
3. Guide le bras à la main (bout des doigts sur le centre du dessus de l'objet),
   appuie sur Entrée: le script affiche la position réelle du TCP et le
   décalage (TCP réel - nuage) sur X, Y, Z dans fp3_link0.
4. Avec execute_pick:=true, il affiche aussi la pose envoyée au bras et son
   écart au centre du nuage (= effet du choix du grasp, hors calibration).

Un décalage constant en Z pointe vers la profondeur/le TCP, un décalage latéral
vers la calibration main-oeil.
"""
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener

BASE = 'fp3_link0'
TCP = 'fp3_hand_tcp'


class MeasurePickOffset(Node):
    def __init__(self):
        super().__init__('measure_pick_offset')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(PointCloud2, '/pick/pointcloud', self._on_cloud, 10)
        self.create_subscription(
            PoseStamped, '/pick/executed_grasp_pose', self._on_executed, 10)
        self.obj_center = None
        self.obj_top = None
        threading.Thread(target=self._wait_enter, daemon=True).start()
        self.get_logger().info(
            "En attente d'un nuage sur /pick/pointcloud (clique l'objet)...")

    def _on_cloud(self, msg):
        if msg.width * msg.height == 0:
            return
        n = msg.width * msg.height
        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, msg.point_step)
        xyz = np.ascontiguousarray(raw[:, :12]).view(np.float32).reshape(n, 3).astype(float)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        if len(xyz) == 0:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                BASE, msg.header.frame_id, rclpy.time.Time())
        except Exception as exc:
            self.get_logger().warn(f'TF {BASE} <- {msg.header.frame_id} indisponible: {exc}')
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        rot = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        pts = xyz @ rot.T + np.array([t.x, t.y, t.z])
        self.obj_center = pts.mean(axis=0)
        self.obj_top = float(np.percentile(pts[:, 2], 95))
        c = self.obj_center
        print(f'\n[NUAGE OBJET dans {BASE}] {len(pts)} pts | centre = '
              f'({c[0]:.3f}, {c[1]:.3f}, {c[2]:.3f}) m | dessus (z 95%) = '
              f'{self.obj_top:.3f} m', flush=True)
        print("-> Guide le bout des doigts au centre du DESSUS de l'objet, "
              'puis appuie sur Entrée.', flush=True)

    def _on_executed(self, msg):
        p = msg.pose.position
        print(f'\n[POSE ENVOYEE AU BRAS ({msg.header.frame_id})] '
              f'({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) m', flush=True)
        if self.obj_center is None:
            return
        d = np.array([p.x, p.y, p.z]) - self.obj_center
        print(f'[ECART choix du grasp = pose envoyée - centre du nuage] '
              f'dx={d[0]*1000:+.0f} mm  dy={d[1]*1000:+.0f} mm  '
              f'(horizontal {np.hypot(d[0], d[1])*1000:.0f} mm)', flush=True)

    def _wait_enter(self):
        while True:
            input()
            self._report()

    def _report(self):
        try:
            tf = self.tf_buffer.lookup_transform(BASE, TCP, rclpy.time.Time())
        except Exception as exc:
            print(f'TF {BASE} <- {TCP} indisponible: {exc}', flush=True)
            return
        p = tf.transform.translation
        tcp = np.array([p.x, p.y, p.z])
        print(f'[TCP réel dans {BASE}] ({tcp[0]:.3f}, {tcp[1]:.3f}, {tcp[2]:.3f}) m',
              flush=True)
        if self.obj_center is None:
            print("Pas encore de nuage d'objet reçu.", flush=True)
            return
        d = tcp - self.obj_center
        print(f'[DECALAGE = TCP réel - centre du nuage] dx={d[0]*1000:+.0f} mm  '
              f'dy={d[1]*1000:+.0f} mm  (horizontal {np.hypot(d[0], d[1])*1000:.0f} mm)',
              flush=True)
        print(f'[DECALAGE en Z: TCP réel - dessus du nuage] '
              f'dz={(tcp[2]-self.obj_top)*1000:+.0f} mm', flush=True)


def main():
    rclpy.init()
    node = MeasurePickOffset()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
