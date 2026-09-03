import logging

import zmq

from core.config import zmq_robot_timeout_ms, zmq_robot_url

logger = logging.getLogger(__name__)


def _send_command(payload: dict) -> dict:
    """Send a ZMQ REQ command and wait for the REP reply.

    Uses a per-call socket so that long-running VLA instructions (which can
    take tens of seconds) do not block other short commands.
    The timeout is set per the config — default 120 s for VLA execution.
    """
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.REQ)
    sock.setsockopt(zmq.RCVTIMEO, zmq_robot_timeout_ms())
    sock.setsockopt(zmq.LINGER, 0)
    sock.connect(zmq_robot_url())
    try:
        sock.send_json(payload)
        return sock.recv_json()
    except zmq.Again:
        logger.error(
            "Robot command timed out after %d ms: %s",
            zmq_robot_timeout_ms(),
            payload,
        )
        return {"success": False, "error": "timeout"}
    finally:
        sock.close()


def run_instruction(instruction: str) -> dict:
    """Send a natural language instruction to the VLA worker (blocking)."""
    return _send_command({"action": "run_instruction", "instruction": instruction})


def home() -> dict:
    return _send_command({"action": "home"})


def stop() -> dict:
    return _send_command({"action": "stop"})
