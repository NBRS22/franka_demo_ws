# gemini_er_simulator.py
import argparse
import json
import threading

import zmq
import numpy as np
import cv2
import msgpack

TASK_TIMEOUT_MS = 180_000  # 180 secondes


class GeminiERSimulator:
    def __init__(self, label="cube", task_type="pick", robot_ip="127.0.0.1", camera_port=5555, bridge_port=5556):
        self.label = label
        self.task_type = task_type
        self.robot_ip = robot_ip
        self.camera_port = camera_port
        self.bridge_port = bridge_port
        self.clicked_point = None
        self.frame_shape = None  # (height, width) de la derniere frame recue
        self.window_name = "Gemini ER Simulator"
        self.editing_label = False
        self.label_buffer = ""
        self.busy = False
        self.status_message = ""

        self.cam_context = zmq.Context()
        self.cam_socket = self.cam_context.socket(zmq.SUB)
        self.cam_socket.connect(f"tcp://{self.robot_ip}:{self.camera_port}")
        self.cam_socket.setsockopt(zmq.SUBSCRIBE, b"rgb")
        self.cam_socket.setsockopt(zmq.RCVHWM, 1)
        print(f"Connecté au flux caméra sur {self.robot_ip}:{self.camera_port}")

        print(f"Bridge configuré sur {self.robot_ip}:{self.bridge_port}")

        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self._mouse_callback)

    def _mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and not self.editing_label and not self.busy:
            self.clicked_point = (x, y)
            print(f"Point cliqué: ({x}, {y})")

    def _receive_frame(self):
        try:
            topic, data = self.cam_socket.recv_multipart(zmq.NOBLOCK)
            return cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        except zmq.Again:
            return None

    def _draw_overlay(self, img):
        display = img.copy()

        if self.clicked_point is not None:
            cv2.circle(display, self.clicked_point, 8, (0, 255, 0), -1)
            cv2.putText(
                display,
                f"({self.clicked_point[0]}, {self.clicked_point[1]})",
                (self.clicked_point[0] + 10, self.clicked_point[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

        if self.editing_label:
            cv2.putText(
                display,
                f"Nouveau label: {self.label_buffer}_",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 165, 255),
                2
            )
            cv2.putText(
                display,
                "Entree = valider, Echap = annuler",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                1
            )
        else:
            cv2.putText(
                display,
                f"Label: {self.label} | Tache: {self.task_type}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2
            )
            if self.busy:
                cv2.putText(
                    display,
                    self.status_message,
                    (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 165, 255),
                    1
                )
            else:
                cv2.putText(
                    display,
                    "Clic gauche = envoyer, 'l' = label, 't' = pick/place, 'q' = quitter",
                    (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    1
                )
        return display

    def _to_normalized(self, point):
        """Convertit un point en pixels vers des coordonnées normalisées 0-1000 (convention Gemini)."""
        if self.frame_shape is None:
            return point
        height, width = self.frame_shape[:2]
        nx = point[0] / width * 1000.0
        ny = point[1] / height * 1000.0
        return (nx, ny)

    def _send_command(self, task_type, label, point):
        self.busy = True
        self.status_message = f"Envoi {task_type}..."

        norm_point = self._to_normalized(point)

        print(f"\nEnvoi {task_type} command...")
        print(f"  label: {label}")
        print(f"  point (pixels): {point}")
        print(f"  point (normalise 0-1000): {norm_point}")

        context = zmq.Context()
        socket = context.socket(zmq.REQ)
        try:
            socket.connect(f"tcp://{self.robot_ip}:{self.bridge_port}")

            command = {
                "task_type": task_type,
                "point_x": float(norm_point[0]),
                "point_y": float(norm_point[1]),
                "object_label": label,
            }

            socket.send(msgpack.packb(command, use_bin_type=True))
            print("Commande envoyée, en attente réponse...")

            socket.setsockopt(zmq.RCVTIMEO, TASK_TIMEOUT_MS)
            raw = socket.recv()
            try:
                result = json.loads(raw.decode("utf8"))
            except UnicodeDecodeError:
                result = msgpack.unpackb(raw, raw=False)
            status = result.get("status")
            message = result.get("message", "")

            if status == "ok":
                print(f"✅ {task_type.capitalize()} réussi: {message}")
                self.status_message = f"OK: {message}"
            else:
                print(f"❌ {task_type.capitalize()} échoué: {message}")
                self.status_message = f"Echec: {message}"

        except zmq.Again:
            print("❌ Timeout - pas de réponse du robot")
            self.status_message = "Timeout - pas de réponse"

        finally:
            socket.close()
            context.term()
            self.busy = False

    def _handle_key(self, key):
        if key == 255:  # aucune touche pressée
            return True

        if self.editing_label:
            if key in (13, 10):  # Entree : valider le nouveau label
                if self.label_buffer:
                    self.label = self.label_buffer
                    print(f"Label mis à jour: '{self.label}'")
                self.editing_label = False
                self.label_buffer = ""
            elif key == 27:  # Echap : annuler
                self.editing_label = False
                self.label_buffer = ""
            elif key in (8, 127):  # Backspace
                self.label_buffer = self.label_buffer[:-1]
            elif 32 <= key <= 126:  # caractère imprimable
                self.label_buffer += chr(key)
            return True

        if key == ord('q'):
            return False
        elif key == ord('l'):
            self.editing_label = True
            self.label_buffer = ""
        elif key == ord('t'):
            self.task_type = "place" if self.task_type == "pick" else "pick"
            print(f"Tâche: {self.task_type}")

        return True

    def run(self):
        print("Cliquez sur le point pour envoyer une commande")
        print(f"Label actuel: '{self.label}' | Tâche: {self.task_type}")
        print("'l' = changer le label, 't' = basculer pick/place, 'q' = quitter")

        while True:
            img = self._receive_frame()

            if img is not None:
                self.frame_shape = img.shape
                display = self._draw_overlay(img)
                cv2.imshow(self.window_name, display)

            if self.clicked_point is not None and not self.editing_label and not self.busy:
                point = self.clicked_point
                self.clicked_point = None
                threading.Thread(
                    target=self._send_command,
                    args=(self.task_type, self.label, point),
                    daemon=True,
                ).start()

            key = cv2.waitKey(1) & 0xFF
            if not self._handle_key(key):
                break

        self.close()

    def close(self):
        cv2.destroyAllWindows()
        self.cam_socket.close()
        self.cam_context.term()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Gemini ER simulator (click-to-pick, local dev)")
    parser.add_argument("--robot-ip", default="127.0.0.1",
                         help="Host running camera_bridge/command_bridge (default: 127.0.0.1, "
                              "same machine as the pipeline). Use the LAN IP for a real remote deployment.")
    parser.add_argument("--camera-port", type=int, default=5555)
    parser.add_argument("--bridge-port", type=int, default=5556)
    parser.add_argument("--label", default="cube")
    parser.add_argument("--task-type", default="pick", choices=["pick", "place"])
    args = parser.parse_args()

    sim = GeminiERSimulator(
        label=args.label,
        task_type=args.task_type,
        robot_ip=args.robot_ip,
        camera_port=args.camera_port,
        bridge_port=args.bridge_port,
    )
    sim.run()
