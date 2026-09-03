"""Franka VLA embodiment — controls Franka FR3 via a VLA model backend."""

import asyncio
import logging

from camera import poller as camera_poller
from embodiment import base
from embodiment.franka import franka_vla_robot_client
from tools import tools as tools_lib

logger = logging.getLogger(__name__)


class FrankaVlaEmbodiment(base.Embodiment):
  """Embodiment for a Franka FR3 arm controlled through a VLA model.

  The VLA backend receives natural language instructions and translates them
  to motor commands. This embodiment exposes run_instruction (BLOCKING),
  home, and stop — no pixel-coordinate pick/place.
  """

  def __init__(
      self,
      robot_url: str = "http://localhost:8889",
      poll_hz: float = 2.0,
      push_hz: float = 1.0,
  ):
    self.audio_queue: asyncio.Queue = asyncio.Queue()
    self.video_queue: asyncio.Queue = asyncio.Queue()
    self.text_queue: asyncio.Queue = asyncio.Queue()
    self.robot = franka_vla_robot_client.FrankaVlaRobotClient(base_url=robot_url)
    self.poller = camera_poller.CameraPoller(
        robot_client=self.robot,
        video_input_queue=self.video_queue,
        camera_ids=["camera"],
        poll_hz=poll_hz,
        push_hz=push_hz,
    )
    self.poller_task = asyncio.create_task(self.poller.run())
    logger.info("FrankaVlaEmbodiment initialized with URL: %s", robot_url)

  def get_audio_queue(self) -> asyncio.Queue:
    return self.audio_queue

  def get_video_queue(self) -> asyncio.Queue:
    return self.video_queue

  def get_text_queue(self) -> asyncio.Queue:
    return self.text_queue

  def get_tools(self) -> list[dict]:
    return tools_lib.franka_vla_tools()

  def get_system_instruction(self) -> str:
    return ""

  async def execute_action(self, action_name: str, **kwargs):
    logger.info(
        "FrankaVlaEmbodiment executing action: %s with args: %s",
        action_name,
        kwargs,
    )
    if action_name == "ack":
      return {"status": "ok"}
    if action_name == "run_instruction":
      return await self.robot.run_instruction(
          instruction=kwargs["instruction"],
      )
    if action_name == "home":
      return await self.robot.home()
    if action_name == "stop":
      return await self.robot.stop()
    raise ValueError(f"Unknown action: {action_name}")

  async def close(self):
    self.poller.stop()
    await self.poller_task
    await self.robot.close()
