"""Agent registry — maps agent names to their factory functions."""

from agents.base import Agent
from agents.presets.franka import franka
from agents.presets.franka_vla import franka_vla
from agents.presets.human import human
from agents.presets.spot import spot

_PRESETS = {
    "human": human,
    "spot": spot,
    "franka": franka,
    "franka_vla": franka_vla,
}

__all__ = ["Agent", "from_name"]


def from_name(name: str) -> Agent:
  """Return an Agent for the given name.

  Args:
    name: One of "human", "spot", "franka", "franka_vla".

  Raises:
    ValueError: If the name is unknown.
  """
  if name not in _PRESETS:
    raise ValueError(
        f"Unknown agent name: {name!r}. Available: {list(_PRESETS.keys())}"
    )
  return _PRESETS[name]()
