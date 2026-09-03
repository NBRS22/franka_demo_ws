"""Franka arm embodiment (Lite version)."""

import asyncio
import logging

from camera import poller as camera_poller
from embodiment import base
from embodiment.franka import robot_client
from tools import tools as tools_lib

logger = logging.getLogger(__name__)


class FrankaEmbodiment(base.Embodiment):
  """Embodiment for a Franka arm with a fixed overhead camera."""

  def __init__(
      self,
      robot_url: str,
      poll_hz: float = 2.0,
      push_hz: float = 1.0,
  ):
    self.audio_queue = asyncio.Queue()
    self.video_queue = asyncio.Queue()
    self.text_queue = asyncio.Queue()
    self.robot = robot_client.FrankaRobotClient(base_url=robot_url)
    self.poller = camera_poller.CameraPoller(
        robot_client=self.robot,
        video_input_queue=self.video_queue,
        camera_ids=["camera"],
        poll_hz=poll_hz,
        push_hz=push_hz,
    )
    self.poller_task = asyncio.create_task(self.poller.run())
    logger.info("FrankaEmbodiment initialized with URL: %s", robot_url)

  def get_audio_queue(self) -> asyncio.Queue:
    return self.audio_queue

  def get_video_queue(self) -> asyncio.Queue:
    return self.video_queue

  def get_text_queue(self) -> asyncio.Queue:
    return self.text_queue

  def get_tools(self) -> list[dict]:
    return tools_lib.franka_tools()

  def get_system_instruction(self) -> str:
    return ""

  async def execute_action(self, action_name: str, **kwargs):
    logger.info(
        "FrankaEmbodiment executing action: %s with args: %s",
        action_name,
        kwargs,
    )
    if action_name == "ack":
      return {"status": "ok"}
    if action_name == "pick":
      return await self.robot.pick(
          x=int(kwargs["x"]),
          y=int(kwargs["y"]),
          label=kwargs["label"],
      )
    if action_name == "place":
      return await self.robot.place(
          x=int(kwargs["x"]),
          y=int(kwargs["y"]),
          label=kwargs["label"],
      )
    if action_name == "home":
      return await self.robot.home()
    if action_name == "stop":
      return await self.robot.stop()
    if action_name == "conveyor":
      return await self.robot.conveyor(
          state=kwargs["state"],
          direction=kwargs["direction"],
      )
    raise ValueError(f"Unknown action: {action_name}")

  async def close(self):
    self.poller.stop()
    await self.poller_task
    await self.robot.close()
