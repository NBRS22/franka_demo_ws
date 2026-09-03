"""Franka VLA robot client — talks to the franka_vla backend."""

from __future__ import annotations

import logging

import httpx

from embodiment import robot_client

logger = logging.getLogger(__name__)

_LONG_TIMEOUT = httpx.Timeout(180.0)


class FrankaVlaRobotClient(robot_client.RobotClient):
  """RobotClient configured for the Franka VLA backend.

  Expects the franka_vla backend to expose:
    GET  /camera/snapshot     -> raw JPEG bytes
    GET  /camera/stream       -> MJPEG stream
    POST /run_instruction     -> {"instruction": str}  (blocking until done)
    POST /home                -> {}
    POST /stop                -> {}
  """

  CAMERA_IDS = ["camera"]
  ENDPOINT_MAP = {"camera": "/camera/stream"}

  def __init__(
      self,
      base_url: str = "http://localhost:8889",
      timeout: float = 30.0,
  ):
    super().__init__(base_url=base_url, timeout=timeout)

  async def get_camera_snapshot(self, camera_id: str) -> bytes | None:
    try:
      resp = await self._client.get(
          "/camera/snapshot",
          timeout=self._camera_timeout,
      )
      resp.raise_for_status()
      return resp.content
    except Exception as e:  # pylint: disable=broad-except
      logger.warning("camera snapshot error: %s", e)
      return None

  async def run_instruction(self, instruction: str) -> dict:
    try:
      resp = await self._client.post(
          "/run_instruction",
          json={"instruction": instruction},
          timeout=_LONG_TIMEOUT,
      )
      resp.raise_for_status()
      return resp.json()
    except Exception as e:  # pylint: disable=broad-except
      logger.error("run_instruction error: %s", e)
      return {"error": str(e)}

  async def home(self) -> dict:
    try:
      resp = await self._client.post("/home", timeout=_LONG_TIMEOUT)
      resp.raise_for_status()
      return resp.json()
    except Exception as e:  # pylint: disable=broad-except
      logger.error("home error: %s", e)
      return {"error": str(e)}

  async def stop(self) -> dict:
    try:
      resp = await self._client.post("/stop")
      resp.raise_for_status()
      return resp.json()
    except Exception as e:  # pylint: disable=broad-except
      logger.error("stop error: %s", e)
      return {"error": str(e)}
