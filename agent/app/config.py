"""Server-level configuration and helpers."""

import dataclasses

from session import config as session_config


@dataclasses.dataclass
class ServerConfig:
  """Server-level configuration from CLI flags."""

  model: str = "models/gemini-robotics-er-2-streaming-preview"
  robot_url: str = "http://localhost:8888"
  api_key: str | None = None
  tts_api_key: str | None = None
  use_tts: bool = True
  tts_voice: str = "en-US-Chirp3-HD-Puck"
  tts_language_code: str = "en-US"
  tts_audio_gain: float = 1.0
  response_modality: str = "AUDIO"
  dump_video_dir: str = ""
  heartbeat_interval_seconds: float = 2.0
  heartbeat_min_delay_seconds: float = 2.0
  heartbeat_enabled: bool = True
  use_event_driven_heartbeat: bool = True
  heartbeat_safety_timeout_seconds: float = 10.0
  media_resolution: str = "low"
  enable_send_message_to_user: bool = True
  mock_robot: bool = False
  agent_peers: dict[str, str] = dataclasses.field(default_factory=dict)
  custom_si: dict[str, str] = dataclasses.field(default_factory=dict)
  custom_di: dict[str, str] = dataclasses.field(default_factory=dict)
  custom_heartbeat_text: dict[str, str] = dataclasses.field(default_factory=dict)
  custom_disabled_tools: dict[str, list[str]] = dataclasses.field(default_factory=dict)
  port: int = 8000


def resolve_thinking_level(
    model_name: str,
    session_override: str | None = None,
) -> str:
  """Resolve thinking level: session override > model default > global default."""
  if session_override is not None:
    return session_override
  normalized = model_name.removeprefix("models/")
  spec = session_config.KNOWN_MODELS.get(
      normalized, session_config.DEFAULT_MODEL_SPEC
  )
  return spec.thinking_level
