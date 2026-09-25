"""Load server configuration from environment / .env file."""

import os

from dotenv import load_dotenv

from app.config import ServerConfig

load_dotenv()


def _bool(key: str, default: bool = False) -> bool:
    return os.getenv(key, "true" if default else "false").lower() in ("true", "1", "yes")


def _float(key: str, default: float) -> float:
    raw = os.getenv(key, str(default))
    try:
        return float(raw)
    except ValueError:
        raise ValueError(f"Environment variable {key}={raw!r} is not a valid float") from None


def _int(key: str, default: int) -> int:
    raw = os.getenv(key, str(default))
    try:
        return int(raw)
    except ValueError:
        raise ValueError(f"Environment variable {key}={raw!r} is not a valid integer") from None


def load_config() -> ServerConfig:
    """Build a ServerConfig from environment variables (loaded from .env)."""
    api_key = os.getenv("GEMINI_API_KEY")
    if not api_key:
        for path in [
            os.path.expanduser("~/.config/gemini/API_KEY"),
            os.path.expanduser("~/.config/safari_sdk/API_KEY"),
        ]:
            if os.path.isfile(path):
                with open(path) as f:
                    api_key = f.read().strip()
                break

    agent_peers: dict[str, str] = {}
    for pair in os.getenv("AGENT_PEERS", "").split(","):
        pair = pair.strip()
        if "=" in pair:
            name, url = pair.split("=", 1)
            agent_peers[name.strip()] = url.strip()

    return ServerConfig(
        model=os.getenv("MODEL", "models/gemini-robotics-er-2-streaming-preview"),
        robot_url=os.getenv("ROBOT_URL", "http://localhost:8888"),
        api_key=api_key,
        tts_api_key=os.getenv("TTS_API_KEY"),
        use_tts=_bool("USE_TTS", default=True),
        tts_voice=os.getenv("TTS_VOICE", "en-US-Chirp3-HD-Puck"),
        tts_language_code=os.getenv("TTS_LANGUAGE_CODE", "en-US"),
        tts_audio_gain=_float("TTS_AUDIO_GAIN", 1.0),
        response_modality=os.getenv("RESPONSE_MODALITY", "AUDIO"),
        dump_video_dir=os.getenv("DUMP_VIDEO_DIR", ""),
        port=_int("PORT", 8000),
        heartbeat_interval_seconds=_float("HEARTBEAT_INTERVAL_SECONDS", 2.0),
        heartbeat_min_delay_seconds=_float("HEARTBEAT_MIN_DELAY_SECONDS", 2.0),
        heartbeat_enabled=_bool("HEARTBEAT_ENABLED", default=True),
        use_event_driven_heartbeat=_bool("USE_EVENT_DRIVEN_HEARTBEAT", default=True),
        heartbeat_safety_timeout_seconds=_float("HEARTBEAT_SAFETY_TIMEOUT_SECONDS", 10.0),
        media_resolution=os.getenv("MEDIA_RESOLUTION", "low"),
        enable_send_message_to_user=_bool("ENABLE_SEND_MESSAGE_TO_USER", default=True),
        mock_robot=_bool("MOCK_ROBOT", default=False),
        agent_peers=agent_peers,
    )
