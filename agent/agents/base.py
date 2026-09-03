"""Agent base dataclass and prompt loader."""

import dataclasses

from pathlib import Path
from typing import Any

_PROMPTS_DIR = Path(__file__).resolve().parent.parent / "prompts"


def _load(filename: str) -> str:
  """Load a developer instruction file from prompts/."""
  return (_PROMPTS_DIR / filename).read_text(encoding="utf-8")


@dataclasses.dataclass
class Agent:
  """A named composition of developer instruction and tools."""
  name: str
  system_instruction: str
  developer_instruction: str
  tools: list[dict[str, Any]]
