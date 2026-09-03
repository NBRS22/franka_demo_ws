import os
from pathlib import Path


def load():
    """Load .config file into env vars without overriding existing ones."""
    for path in (Path(".config"), Path(__file__).resolve().parents[1] / ".config"):
        if not path.is_file():
            continue
        for raw_line in path.read_text(encoding="utf-8").splitlines():
            line = raw_line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", 1)
            key = key.strip()
            value = value.strip().strip('"').strip("'")
            if key:
                os.environ.setdefault(key, value)
        break


def zmq_camera_url() -> str:
    return os.environ.get("ZMQ_CAMERA_URL", "tcp://localhost:5555")


def zmq_robot_url() -> str:
    return os.environ.get("ZMQ_ROBOT_URL", "tcp://localhost:5558")


def zmq_robot_timeout_ms() -> int:
    return int(os.environ.get("ZMQ_ROBOT_TIMEOUT_MS", "5000"))


def backend_host() -> str:
    return os.environ.get("BACKEND_HOST", "0.0.0.0")


def backend_port() -> int:
    return int(os.environ.get("BACKEND_PORT", "8888"))
