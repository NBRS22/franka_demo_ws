"""Tool declarations for the human embodiment."""

from typing import Any

from tools.common_tools import _tool, _wrap, ack_tool, send_message_tool


def _run_instruction_tool() -> dict[str, Any]:
  return _tool(
      name="run_instruction",
      description=(
          "Give the human operator a single, concrete physical instruction to"
          " perform. The human acts as the arm — they will carry out the step"
          " and signal when done. Each call must describe exactly one physical"
          " action (pick, place, push, …). After the call returns, inspect the"
          " camera feed to visually confirm the step succeeded before issuing"
          " the next instruction."
      ),
      parameters={
          "type": "OBJECT",
          "properties": {
              "instruction": {
                  "type": "STRING",
                  "description": (
                      "A short, unambiguous instruction for the human operator."
                      " One action per call — never combine two actions with"
                      " 'and'. Use simple verbs and describe objects by color,"
                      " shape, or position."
                      " Good: 'pick up the red cube'."
                      " Bad: 'pick up the cube and place it on the tray'."
                  ),
              },
          },
          "required": ["instruction"],
      },
      behavior="BLOCKING",
  )


def _stop_tool() -> dict[str, Any]:
  return _tool(
      name="stop",
      description=(
          "Ask the human operator to stop immediately and hold their current"
          " position. Call this if the task appears unsafe or the user asks to"
          " pause."
      ),
      parameters={"type": "OBJECT", "properties": {}},
      behavior="BLOCKING",
  )


def human_tools() -> list[dict[str, Any]]:
  """Tool set for the human (local webcam/mic) embodiment."""
  return _wrap([
      _run_instruction_tool(),
      _stop_tool(),
      ack_tool(),
      send_message_tool(enable_send_message_to_user=True),
  ])
