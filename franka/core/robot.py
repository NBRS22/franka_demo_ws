import logging

import msgpack
import zmq

from core.config import zmq_robot_timeout_ms, zmq_robot_url

logger = logging.getLogger(__name__)


def _send_command(payload: dict) -> dict:
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.REQ)
    sock.setsockopt(zmq.RCVTIMEO, zmq_robot_timeout_ms())
    sock.setsockopt(zmq.LINGER, 0)
    sock.connect(zmq_robot_url())
    try:
        sock.send(msgpack.packb(payload, use_bin_type=True))
        raw = sock.recv()
        return msgpack.unpackb(raw, raw=False)
    except zmq.Again:
        logger.error("Robot command timed out after %d ms: %s", zmq_robot_timeout_ms(), payload)
        return {"success": False, "error": "timeout"}
    finally:
        sock.close()


def pick(x: int, y: int, label: str) -> dict:
    return _send_command({"action": "pick", "x": x, "y": y, "label": label})


def place(x: int, y: int, label: str) -> dict:
    return _send_command({"action": "place", "x": x, "y": y, "label": label})


def home() -> dict:
    return _send_command({"action": "home"})


def stop() -> dict:
    return _send_command({"action": "stop"})


def conveyor(state: str, direction: str) -> dict:
    return _send_command({"action": "conveyor", "state": state, "direction": direction})
