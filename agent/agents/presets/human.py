"""Agent preset for the human operator embodiment."""

from agents.base import Agent, _load
from tools.human_tools import human_tools


def human() -> Agent:
  """Local/browser mode with webcam — agent guides a human operator."""
  return Agent(
      name="human",
      system_instruction="",
      developer_instruction=_load("human_di.md"),
      tools=human_tools(),
  )
