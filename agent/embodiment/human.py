"""Human embodiment for local/browser mode.

Video input priority:
  1. Intel RealSense camera connected via USB (server-side, pyrealsense2)
  2. Browser webcam frames forwarded over the WebSocket (fallback)

run_instruction is blocking: it waits until the user clicks "Done" in the UI.
"""

import asyncio
import io
import logging
import queue as stdlib_queue
import threading
import time
from typing import Any

from camera import poller as camera_poller
from embodiment import base
from tools import tools as tools_lib

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# RealSense camera client — same interface as FrankaRobotClient / SpotRobotClient
# ---------------------------------------------------------------------------

class RealSenseCameraClient:
  """CameraPoller-compatible adapter for an Intel RealSense USB camera.

  On Windows, librealsense requires wait_for_frames() to be called from the
  same thread that called pipeline.start().  A single daemon thread owns the
  full pipeline lifecycle (start → warm-up → continuous read).  It encodes
  each frame as JPEG and puts it into a bounded queue.  Consumers call
  stream_camera() or get_camera_snapshot() to read frames.

  Stream priority: colour → infrared → depth (colourised).  This covers both
  cameras with a colour sensor (D415/D435/D455) and depth-only models.

  Use the factory function make_realsense_client() instead of instantiating
  directly — it handles ImportError and device-not-found gracefully.
  """

  # Class-level singletons so that multiple HumanEmbodiment instances within
  # the same process share a single pipeline.
  _shared_pipeline = None
  _frame_queue: stdlib_queue.Queue | None = None
  _reader_thread: threading.Thread | None = None
  _startup_done: threading.Event = threading.Event()
  _startup_error: Exception | None = None
  _stream_type: str | None = None
  _colorizer = None

  def __init__(self) -> None:
    self._running = True

    if RealSenseCameraClient._shared_pipeline is None:
      RealSenseCameraClient._frame_queue = stdlib_queue.Queue(maxsize=3)
      RealSenseCameraClient._startup_done = threading.Event()
      RealSenseCameraClient._startup_error = None

      t = threading.Thread(
          target=RealSenseCameraClient._reader_loop,
          daemon=True,
          name="rs-frame-reader",
      )
      t.start()
      RealSenseCameraClient._reader_thread = t

      # Block the calling thread (NOT the event loop) until the reader thread
      # signals that the pipeline is warm and ready.  Called via asyncio.to_thread
      # so the event loop stays responsive during the wait.
      if not RealSenseCameraClient._startup_done.wait(timeout=120):
        raise RuntimeError("RealSense pipeline startup timed out (120 s)")
      if RealSenseCameraClient._startup_error is not None:
        raise RuntimeError(
            f"RealSense init failed: {RealSenseCameraClient._startup_error}"
        )
    else:
      logger.info("RealSense: reusing existing pipeline")

  @classmethod
  def _reader_loop(cls) -> None:
    """Daemon thread: owns the full pipeline lifecycle and buffers JPEG frames."""
    import numpy as np
    from PIL import Image

    try:
      import pyrealsense2 as rs

      pipeline = rs.pipeline()
      cfg = rs.config()
      cfg.enable_all_streams()
      active_profile = pipeline.start(cfg)

      for s in active_profile.get_streams():
        logger.info(
            "RealSense stream enabled: type=%s format=%s",
            s.stream_type(), s.format(),
        )

      colorizer = rs.colorizer()
      stream_type = None

      # Warm-up: allow up to 60 s to get the first usable frame.
      for i in range(120):
        try:
          frames = pipeline.wait_for_frames(timeout_ms=500)
          if frames.get_color_frame():
            stream_type = "color"
            logger.info("RealSense: colour frame on iteration %d", i)
            break
          if frames.get_infrared_frame(1):
            stream_type = "infrared"
            logger.info("RealSense: IR frame on iteration %d", i)
            break
          if frames.get_depth_frame():
            stream_type = "depth"
            logger.info("RealSense: depth frame on iteration %d", i)
            break
        except Exception:
          pass

      if stream_type is None:
        active = [str(s.stream_type()) for s in active_profile.get_streams()]
        raise RuntimeError(
            f"no frames after 60 s warm-up (enabled streams: {active})"
        )

      cls._shared_pipeline = pipeline
      cls._colorizer = colorizer
      cls._stream_type = stream_type
      logger.info("RealSense pipeline ready — stream: %s", stream_type)

    except Exception as exc:
      cls._startup_error = exc
      cls._startup_done.set()
      return

    cls._startup_done.set()

    # Continuous read loop — all wait_for_frames calls stay in this thread.
    while True:
      try:
        frames = cls._shared_pipeline.wait_for_frames(timeout_ms=2000)

        if cls._stream_type == "color":
          frame = frames.get_color_frame()
          if not frame:
            continue
          data = np.asanyarray(frame.get_data())
          fmt = frame.get_profile().format()
          if fmt == rs.format.bgr8 or fmt == rs.format.bgra8:
            img = Image.fromarray(data[..., ::-1] if data.ndim == 3 else data, "RGB")
          else:
            img = Image.fromarray(data, "RGB")

        elif cls._stream_type == "infrared":
          frame = frames.get_infrared_frame(1)
          if not frame:
            continue
          img = Image.fromarray(np.asanyarray(frame.get_data()), "L").convert("RGB")

        else:  # depth
          frame = frames.get_depth_frame()
          if not frame:
            continue
          colored = cls._colorizer.colorize(frame)
          img = Image.fromarray(np.asanyarray(colored.get_data()), "RGB")

        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=85)
        jpeg = buf.getvalue()

        if cls._frame_queue.full():
          try:
            cls._frame_queue.get_nowait()
          except stdlib_queue.Empty:
            pass
        cls._frame_queue.put_nowait(jpeg)

      except Exception as exc:
        logger.warning("RealSense reader: %s", exc)
        time.sleep(0.05)

  def _get_latest_jpeg(self) -> bytes | None:
    """Blocking: wait up to 3 s for the next frame from the reader thread."""
    try:
      return RealSenseCameraClient._frame_queue.get(timeout=3.0)
    except stdlib_queue.Empty:
      logger.warning("RealSense: no frame within 3 s")
      return None

  def stop(self) -> None:
    self._running = False

  async def stream_camera(self, camera_id: str):
    """Async generator yielding JPEG frames — same interface as FrankaRobotClient."""
    while self._running:
      jpeg = await asyncio.to_thread(self._get_latest_jpeg)
      if jpeg:
        yield jpeg
      else:
        await asyncio.sleep(0.05)

  async def get_camera_snapshot(self, camera_id: str) -> bytes | None:
    return await asyncio.to_thread(self._get_latest_jpeg)


def make_realsense_client() -> RealSenseCameraClient | None:
  """Try to build a RealSenseCameraClient; return None on any failure.

  This is a blocking function — call it via asyncio.to_thread().
  """
  try:
    import pyrealsense2 as rs  # noqa: F401
  except ImportError:
    logger.info("pyrealsense2 not installed — browser webcam will be used")
    return None
  try:
    return RealSenseCameraClient()
  except Exception as exc:
    logger.info("RealSense unavailable (%s) — browser webcam will be used", exc)
    return None


# ---------------------------------------------------------------------------
# HumanEmbodiment
# ---------------------------------------------------------------------------

class HumanEmbodiment(base.Embodiment):
  """Embodiment for a human user (browser webcam/mic + optional RealSense).

  Use HumanEmbodiment.create() (async factory) instead of __init__ so that
  the blocking RealSense warm-up runs in a thread and doesn't stall the loop.
  """

  def __init__(self) -> None:
    self.audio_queue: asyncio.Queue = asyncio.Queue(maxsize=50)
    self.video_queue: asyncio.Queue = asyncio.Queue(maxsize=5)
    self.text_queue: asyncio.Queue = asyncio.Queue(maxsize=10)
    self.poller: camera_poller.CameraPoller | None = None
    self._poller_task: asyncio.Task | None = None
    self._rs_client: RealSenseCameraClient | None = None
    self._instruction_done: asyncio.Event | None = None
    self._ui_callback = None  # set by server via set_ui_callback()

  @classmethod
  async def create(cls) -> "HumanEmbodiment":
    """Async factory: initialise and start the camera (non-blocking)."""
    self = cls()
    rs_client = await asyncio.to_thread(make_realsense_client)
    if rs_client is not None:
      try:
        self.poller = camera_poller.CameraPoller(
            robot_client=rs_client,
            video_input_queue=self.video_queue,
            camera_ids=["camera"],
            poll_hz=5.0,
            push_hz=1.0,
        )
        self._poller_task = asyncio.create_task(self.poller.run())
        self._rs_client = rs_client
        logger.info("HumanEmbodiment: RealSense camera active")
      except Exception as exc:
        logger.warning("RealSense poller failed (%s) — browser webcam", exc)
        self.poller = None
        rs_client.stop()
    else:
      logger.info("HumanEmbodiment: browser webcam active")
    return self

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
    if self.poller is not None:
      self.poller.stop()
      if self._poller_task is not None:
        await self._poller_task
    if self._rs_client is not None:
      self._rs_client.stop()
    # Unblock any pending run_instruction (e.g. on disconnect)
    if self._instruction_done is not None and not self._instruction_done.is_set():
      self._instruction_done.set()
