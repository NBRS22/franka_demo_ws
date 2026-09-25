"""Human embodiment for local/browser mode.

Video input: browser webcam frames forwarded over the WebSocket.
run_instruction is blocking: it waits until the user clicks "Done" in the UI.
"""

import asyncio
import logging
from typing import Any

from embodiment import base
from tools import tools as tools_lib

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# HumanEmbodiment
# ---------------------------------------------------------------------------

class HumanEmbodiment(base.Embodiment):
  """Embodiment for a human user (browser webcam/mic)."""

  def __init__(self) -> None:
    self.audio_queue: asyncio.Queue = asyncio.Queue(maxsize=50)
    self.video_queue: asyncio.Queue = asyncio.Queue(maxsize=5)
    self.text_queue: asyncio.Queue = asyncio.Queue(maxsize=10)
    self._instruction_done: asyncio.Event | None = None
    self._ui_callback = None  # set by server via set_ui_callback()

  @classmethod
  async def create(cls) -> "HumanEmbodiment":
    """Async factory kept for API consistency with other embodiments."""
    return cls()

  # ---- Callback wiring (set by main.py after websocket is open) ----

  def set_ui_callback(self, callback) -> None:
    """Register a coroutine callback to push JSON events to the browser."""
    self._ui_callback = callback

  def on_instruction_done(self) -> None:
    """Called by server when the browser sends {type: instruction_done}."""
    if self._instruction_done is not None and not self._instruction_done.is_set():
      self._instruction_done.set()

  # ---- Queues ----

  def get_audio_queue(self) -> asyncio.Queue:
    return self.audio_queue

  def get_video_queue(self) -> asyncio.Queue:
    return self.video_queue

  def get_text_queue(self) -> asyncio.Queue:
    return self.text_queue

  # ---- Action execution ----

  async def execute_action(self, action_name: str, **kwargs: Any) -> str:
    if action_name == "ack":
      return "ok"

    if action_name == "send_message":
      target = kwargs.get("target", "unknown")
      message = kwargs.get("message", "")
      logger.info("[HumanEmbodiment] send_message to %s: %s", target, message)
      return f"Message sent to {target}"

    if action_name == "run_instruction":
      instruction = kwargs.get("instruction", "")
      logger.info("[HumanEmbodiment] run_instruction: %s", instruction)

      # Notify the UI so it shows the "Done" button.
      if self._ui_callback is not None:
        try:
          await self._ui_callback(
              {"type": "run_instruction", "instruction": instruction}
          )
        except Exception:
          pass

      # Block until the user clicks "Done" in the browser.
      self._instruction_done = asyncio.Event()
      try:
        await self._instruction_done.wait()
        return f"Instruction completed: {instruction}"
      finally:
        self._instruction_done = None

    logger.info(
        "[HumanEmbodiment] action: %s args: %s", action_name, kwargs
    )
    return f"Human executed {action_name}"

  # ---- Tools & system instruction ----

  def get_tools(self) -> list[dict[str, Any]]:
    return tools_lib.human_tools()

  def get_system_instruction(self) -> str:
    return ""

  async def close(self) -> None:
    # Unblock any pending run_instruction (e.g. on disconnect)
    if self._instruction_done is not None and not self._instruction_done.is_set():
      self._instruction_done.set()
