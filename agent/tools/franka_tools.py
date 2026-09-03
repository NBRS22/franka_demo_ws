"""Tool declarations for the Franka arm embodiment."""

from typing import Any

from tools.common_tools import _tool, _wrap, ack_tool, send_message_tool


def franka_pick_tool() -> dict[str, Any]:
  return _tool(
      name="pick",
      description=(
          "Command the Franka arm to pick the object located at the given"
          " pixel position in the overhead camera image. Coordinates are"
          " integers 0-1000 where (0,0) is top-left and (1000,1000) is"
          " bottom-right. After this call returns, inspect the fresh camera"
          " frame to visually confirm the pick succeeded."
      ),
      parameters={
          "type": "OBJECT",
          "properties": {
              "x": {"type": "NUMBER", "description": "Horizontal pixel, 0-1000."},
              "y": {"type": "NUMBER", "description": "Vertical pixel, 0-1000."},
              "label": {
                  "type": "STRING",
                  "description": (
                      'Detailed visual description of the object for SAM3 segmentation. '
                      'Use the format: "this [color] [material/texture if visible] [shape]". '
                      'Always include color and shape. Add material or texture whenever distinguishable '
                      '(e.g. matte, shiny, translucent, wooden, cardboard, metal). '
                      'Add size if relevant (small, large, tall). '
                      'Be as specific as possible — vague labels produce poor segmentation masks. '
                      'Examples: "this matte red wooden cube", "this shiny blue metal cylinder", '
                      '"this small green translucent sphere", "this large cardboard box".'
                  ),
              },
          },
          "required": ["x", "y", "label"],
      },
      behavior="BLOCKING",
  )


def franka_place_tool() -> dict[str, Any]:
  return _tool(
      name="place",
      description=(
          "Command the Franka arm to place the currently held object at the"
          " given pixel position in the overhead camera image. Call this only"
          " after a successful pick has been visually confirmed. Coordinates"
          " are integers 0-1000. After this call returns, inspect the fresh"
          " camera frame to confirm the object is at the intended location."
      ),
      parameters={
          "type": "OBJECT",
          "properties": {
              "x": {"type": "NUMBER", "description": "Horizontal pixel, 0-1000."},
              "y": {"type": "NUMBER", "description": "Vertical pixel, 0-1000."},
              "label": {
                  "type": "STRING",
                  "description": 'Color and shape of the target location, starting with "this", e.g. "this green tray", "this blue zone", "this red container".',
              },
          },
          "required": ["x", "y", "label"],
      },
      behavior="BLOCKING",
  )


def franka_home_tool() -> dict[str, Any]:
  return _tool(
      name="home",
      description=(
          "Move the Franka arm back to its home rest position. Call this"
          " after a failed pick or place, when the user asks to reset the"
          " arm, or at the end of a task sequence."
      ),
      parameters={"type": "OBJECT", "properties": {}},
      behavior="BLOCKING",
  )


def franka_stop_tool() -> dict[str, Any]:
  return _tool(
      name="stop",
      description=(
          "Immediately stop all Franka arm motion. Call this whenever"
          " motion appears unsafe or the user asks to stop."
      ),
      parameters={"type": "OBJECT", "properties": {}},
      behavior="BLOCKING",
  )


def franka_conveyor_tool() -> dict[str, Any]:
  return _tool(
      name="conveyor",
      description=(
          "Control the conveyor belt. Turn it on or off and set its direction."
          " Use 'on' to start the belt and 'off' to stop it."
          " Direction must always be specified."
      ),
      parameters={
          "type": "OBJECT",
          "properties": {
              "state": {
                  "type": "STRING",
                  "enum": ["on", "off"],
                  "description": "Whether to turn the conveyor on or off.",
              },
              "direction": {
                  "type": "STRING",
                  "enum": ["forward", "backward"],
                  "description": (
                      "Direction of belt movement."
                      " 'forward' moves objects away from the arm;"
                      " 'backward' moves them toward the arm."
                  ),
              },
          },
          "required": ["state", "direction"],
      },
      behavior="BLOCKING",
  )


def franka_tools() -> list[dict[str, Any]]:
  """Franka arm manipulation tools (pixel-coordinate pick/place)."""
  return _wrap([
      franka_pick_tool(),
      franka_place_tool(),
      franka_home_tool(),
      franka_stop_tool(),
      franka_conveyor_tool(),
      ack_tool(),
      send_message_tool(enable_send_message_to_user=True),
  ])
