"""Proactive Agent — entry point.

Usage:
    python main.py --api_key YOUR_KEY --port 8000
    python main.py --agent_peers spot=http://localhost:8001
"""

import argparse
import logging
import os
import signal

import uvicorn

from app.config import ServerConfig
from app.server import create_app

logger = logging.getLogger(__name__)


def main():
  parser = argparse.ArgumentParser(description="Proactive Agent Server")
  parser.add_argument("--model", default="models/gemini-robotics-er-2-streaming-preview")
  parser.add_argument("--robot_url", default="http://localhost:8888")
  parser.add_argument("--api_key", default=os.getenv("GEMINI_API_KEY"))
  parser.add_argument("--tts_api_key", default=os.getenv("TTS_API_KEY"))
  parser.add_argument("--use_tts", action="store_true", default=True)
  parser.add_argument("--no_tts", dest="use_tts", action="store_false")
  parser.add_argument("--tts_voice", default="en-US-Chirp3-HD-Puck")
  parser.add_argument("--tts_language_code", default="en-US")
  parser.add_argument("--tts_audio_gain", type=float, default=1.0)
  parser.add_argument("--response_modality", default="AUDIO")
  parser.add_argument("--dump_video_dir", default="")
  parser.add_argument("--port", type=int, default=8000)
  parser.add_argument("--heartbeat_interval_seconds", type=float, default=2.0)
  parser.add_argument("--heartbeat_min_delay_seconds", type=float, default=2.0)
  parser.add_argument("--heartbeat_enabled", action="store_true", default=True)
  parser.add_argument("--no_heartbeat", dest="heartbeat_enabled", action="store_false")
  parser.add_argument("--use_event_driven_heartbeat", action="store_true", default=True)
  parser.add_argument("--heartbeat_safety_timeout_seconds", type=float, default=10.0)
  parser.add_argument("--media_resolution", default="low", choices=["low", "medium", "high", "ultra_high"])
  parser.add_argument("--enable_send_message_to_user", action="store_true", default=True)
  parser.add_argument("--mock_robot", action="store_true", default=False)
  parser.add_argument("--agent_peers", default="", help="name=url,name=url")
  args = parser.parse_args()

  # Load API key from file if not provided via CLI or env
  api_key = args.api_key
  if not api_key:
    for path in [
        os.path.expanduser("~/.config/gemini/API_KEY"),
        os.path.expanduser("~/.config/safari_sdk/API_KEY"),
    ]:
      if os.path.isfile(path):
        with open(path) as f:
          api_key = f.read().strip()
        break

  # Parse agent peers: "spot=http://localhost:8001,franka=http://localhost:8002"
  agent_peers = {}
  for pair in args.agent_peers.split(","):
    pair = pair.strip()
    if "=" in pair:
      name, url = pair.split("=", 1)
      agent_peers[name.strip()] = url.strip()

  config = ServerConfig(
      model=args.model,
      robot_url=args.robot_url,
      api_key=api_key,
      tts_api_key=args.tts_api_key,
      use_tts=args.use_tts,
      tts_voice=args.tts_voice,
      tts_language_code=args.tts_language_code,
      tts_audio_gain=args.tts_audio_gain,
      response_modality=args.response_modality,
      dump_video_dir=args.dump_video_dir,
      heartbeat_interval_seconds=args.heartbeat_interval_seconds,
      heartbeat_min_delay_seconds=args.heartbeat_min_delay_seconds,
      heartbeat_enabled=args.heartbeat_enabled,
      use_event_driven_heartbeat=args.use_event_driven_heartbeat,
      heartbeat_safety_timeout_seconds=args.heartbeat_safety_timeout_seconds,
      media_resolution=args.media_resolution,
      enable_send_message_to_user=args.enable_send_message_to_user,
      mock_robot=args.mock_robot,
      agent_peers=agent_peers,
      port=args.port,
  )

  app = create_app(config)
  server = uvicorn.Server(uvicorn.Config(app, host="0.0.0.0", port=config.port, log_level="info"))

  def _shutdown(sig, frame):
    del sig, frame
    server.should_exit = True

  signal.signal(signal.SIGINT, _shutdown)
  signal.signal(signal.SIGTERM, _shutdown)

  server.run()


if __name__ == "__main__":
  main()
