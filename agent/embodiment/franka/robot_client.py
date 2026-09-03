"""Franka arm robot client (Lite version)."""

from __future__ import annotations

import logging

import httpx

from embodiment import robot_client

logger = logging.getLogger(__name__)

_LONG_TIMEOUT = httpx.Timeout(120.0)


class FrankaRobotClient(robot_client.RobotClient):
  """RobotClient configured for the Franka arm platform.

  Expects the Franka backend to expose:
    GET  /camera/snapshot  -> raw JPEG bytes
    GET  /camera/stream    -> MJPEG stream (optional)
    POST /pick             -> {"x": int, "y": int, "label": str}
    POST /place            -> {"x": int, "y": int, "label": str}
    POST /home             -> {}
    POST /stop             -> {}
  """

  CAMERA_IDS = ["camera"]
  ENDPOINT_MAP = {"camera": "/camera/stream"}

  def __init__(
      self,
      base_url: str = "http://localhost:8888",
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

  async def pick(self, x: int, y: int, label: str) -> dict:
    try:
      resp = await self._client.post(
          "/pick",
          json={"x": x, "y": y, "label": label},
          timeout=_LONG_TIMEOUT,
      )
      resp.raise_for_status()
      return resp.json()
    except Exception as e:  # pylint: disable=broad-except
      logger.error("pick error: %s", e)
      return {"error": str(e)}

  async def place(self, x: int, y: int, label: str) -> dict:
    try:
      resp = await self._client.post(
          "/place",
          json={"x": x, "y": y, "label": label},
          timeout=_LONG_TIMEOUT,
      )
      resp.raise_for_status()
      return resp.json()
    except Exception as e:  # pylint: disable=broad-except
      logger.error("place error: %s", e)
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

  async def conveyor(self, state: str, direction: str) -> dict:
    try:
      resp = await self._client.post(
          "/conveyor",
          json={"state": state, "direction": direction},
      )
      resp.raise_for_status()
      return resp.json()
    except Exception as e:  # pylint: disable=broad-except
      logger.error("conveyor error: %s", e)
      return {"error": str(e)}
