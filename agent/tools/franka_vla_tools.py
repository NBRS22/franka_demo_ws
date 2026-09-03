"""Tool declarations for the Franka arm VLA embodiment."""

from typing import Any

from tools.common_tools import _tool, _wrap, ack_tool, send_message_tool
from tools.franka_tools import franka_home_tool, franka_stop_tool


def franka_vla_run_instruction_tool() -> dict[str, Any]:
  return _tool(
      name="run_instruction",
      description=(
          "Send a short natural language instruction to the Franka arm."
          " The VLA model maps the instruction directly to robot motor commands."
          " Each call handles one single physical action. After the call returns,"
          " a fresh camera frame is sent to you — inspect it to visually confirm"
          " whether the action succeeded before proceeding to the next step."
      ),
      parameters={
          "type": "OBJECT",
          "properties": {
              "instruction": {
                  "type": "STRING",
                  "description": (
                      "A short, concrete instruction for the robot arm."
                      " Rules: (1) One action per call — never use 'and' to"
                      " combine two actions. (2) Use simple verbs: 'pick up',"
                      " 'place on', 'put in', 'push', 'slide', 'grasp'."
                      " (3) Describe objects by color, shape, or position —"
                      " never use pixel coordinates or numbers."
                      " Good: 'pick up the red cube'."
                      " Bad: 'pick up the cube and put it in the tray'."
                  ),
              },
          },
          "required": ["instruction"],
      },
      behavior="BLOCKING",
  )


def franka_vla_tools() -> list[dict[str, Any]]:
  """Franka VLA tools: natural language run_instruction, home, stop, ack, send_message."""
  return _wrap([
      franka_vla_run_instruction_tool(),
      franka_home_tool(),
      franka_stop_tool(),
      ack_tool(),
      send_message_tool(enable_send_message_to_user=True),
  ])
