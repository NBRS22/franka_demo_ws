"""Agent preset for the Franka VLA embodiment (natural language instructions)."""

from agents.base import Agent, _load
from tools.franka_vla_tools import franka_vla_tools


def franka_vla() -> Agent:
  """Franka FR3 arm agent controlled via a VLA model."""
  return Agent(
      name="franka_vla",
      system_instruction="",
      developer_instruction=_load("franka_vla_di.md"),
      tools=franka_vla_tools(),
  )
