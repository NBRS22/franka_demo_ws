"""Session configuration and model specification for Proactive Agent.

This module defines the per-session configuration and model abstraction layer.
ThinkingMode, ModelSpec, and KNOWN_MODELS live here because thinking mode is a
session-specific concern, not a server-level one.
"""

import dataclasses
import enum
import logging

logger = logging.getLogger(__name__)


class EndpointType(enum.Enum):
  GEMINI_LIVE_API = "gemini_live_api"


# Supported thinking levels for the Gemini API.
# "none" disables thinking (no thinkingConfig sent).
THINKING_LEVELS = ("none", "minimal", "low", "medium", "high")
DEFAULT_THINKING_LEVEL = "low"



@dataclasses.dataclass
class ModelSpec:
  """Specification for a model and its default attributes.

  Wraps the model name together with model-specific defaults (e.g. thinking
  budget). Use KNOWN_MODELS to register defaults for specific models. Users can
  still override individual attributes via CLI flags or ORCA eval query params.
  """

  name: str
  thinking_level: str = DEFAULT_THINKING_LEVEL
  # Future attributes: temperature, max_output_tokens, etc.


# Registry of known models and their default attributes.
# Only public Gemini Live API models are included for Lite.
# thinking_level: "none" = disabled, "minimal"/"low"/"medium"/"high" = API thinking levels.
KNOWN_MODELS: dict[str, ModelSpec] = {
    "gemini-3.1-flash-live-preview": ModelSpec(
        name="gemini-3.1-flash-live-preview",
        thinking_level="low",
    ),
    "gemini-robotics-er-2-streaming-preview": ModelSpec(
        name="gemini-robotics-er-2-streaming-preview",
        thinking_level="low",
    ),
    "robotics_er_live_text_only_2p0_no_safety_classifiers": ModelSpec(
        name="robotics_er_live_text_only_2p0_no_safety_classifiers",
        thinking_level="low",
    ),
    "robotics_er_live_text_only_2p0_2_no_safety_classifiers": ModelSpec(
        name="robotics_er_live_text_only_2p0_2_no_safety_classifiers",
        thinking_level="low",
    ),
    "robotics_er_live_text_only_optimized": ModelSpec(
        name="robotics_er_live_text_only_optimized",
        thinking_level="low",
    ),
}

# Fallback spec for unknown models.
DEFAULT_MODEL_SPEC = ModelSpec(name="default", thinking_level=DEFAULT_THINKING_LEVEL)


@dataclasses.dataclass
class SessionConfig:
  """Configuration for a specific session/agent instance at runtime."""

  # Agent name (e.g., "human", "apollo", "custom")
  agent_name: str = "human"

  # Custom system instruction (used if agent_name is "custom")
  custom_si: str = ""

  # Custom developer instruction
  custom_di: str = ""

  # List of tool names to enable. If empty, all tools from ServerConfig are enabled.
  enabled_tools: list[str] = dataclasses.field(default_factory=list)

  # Response modality override (AUDIO or TEXT). If None, use ServerConfig default.
  response_modality: str | None = None

  # Model override. If None, use ServerConfig default.
  model: str | None = None

  # Enable TTS override. If None, use ServerConfig default.
  use_tts: bool | None = None

  # Thinking level override. None = use model default from KNOWN_MODELS.
  # "none" = disabled, "minimal"/"low"/"medium"/"high" = API thinking levels.
  thinking_level: str | None = None

  # Heartbeat text override. If None, use ServerConfig default or built-in default.
  heartbeat_text: str | None = None

  # Endpoint type (gemini_live_api)
  endpoint_type: str = "gemini_live_api"
