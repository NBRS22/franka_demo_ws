"""Agent preset for the Boston Dynamics Spot embodiment."""

from agents.base import Agent, _load
from tools.spot_tools import spot_tools


def spot() -> Agent:
  """Boston Dynamics Spot robot agent."""
  return Agent(
      name="spot",
      system_instruction="",
      developer_instruction=_load("spot_di.md"),
      tools=spot_tools(),
  )
