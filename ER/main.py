import argparse

from gemini_er_simulator import GeminiERSimulator


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--label", default="cube", help="Label de l'objet")
    parser.add_argument("--task", choices=["pick", "place"], default="pick", help="Tâche initiale (bascule avec 't')")
    parser.add_argument("--robot-ip", default="172.22.62.72", help="IP du robot")
    parser.add_argument("--camera-port", type=int, default=5555, help="Port du flux caméra")
    parser.add_argument("--bridge-port", type=int, default=5556, help="Port du bridge")
    args = parser.parse_args()

    GeminiERSimulator(
        label=args.label,
        task_type=args.task,
        robot_ip=args.robot_ip,
        camera_port=args.camera_port,
        bridge_port=args.bridge_port,
    ).run()


if __name__ == "__main__":
    main()
