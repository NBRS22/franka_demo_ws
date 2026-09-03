"""Agent preset for the Franka arm embodiment (pixel-coordinate pick/place)."""

from agents.base import Agent, _load
from tools.franka_tools import franka_tools


def franka() -> Agent:
  """Franka arm robot agent — pixel-coordinate pick/place."""
  return Agent(
      name="franka",
      system_instruction="",
      developer_instruction=_load("franka_di.md"),
      tools=franka_tools(),
  )
