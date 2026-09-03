"""Tool declarations for Boston Dynamics Spot embodiment."""

from typing import Any

from tools.common_tools import _tool, _wrap, ack_tool, send_message_tool


def health_check_tool() -> dict[str, Any]:
  return _tool(
      name="health_check",
      description=(
          "Check robot connection, lease holder, and battery state. Use when"
          " the user asks for status, before physical motion when control"
          " state is unknown, or after evidence of a connection, lease, or"
          " battery problem. Do not call automatically at startup or while"
          " idle."
      ),
      parameters={"type": "OBJECT", "properties": {}},
      behavior="BLOCKING",
  )


def get_waypoints_tool() -> dict[str, Any]:
  return _tool(
      name="get_waypoints",
      description=(
          "List registered navigation destinations and report GraphNav"
          " readiness. Use only when the user asks for destinations or"
          " requests waypoint navigation; do not call at startup or for"
          " manipulation, camera, status, or local-drive tasks. If the user"
          " only asks for names, return them without discussing localization."
          " For navigation, use a returned name only when navigation_ready is"
          " true. If false, explain that GraphNav must be loaded and localized."
          " If null or absent, say readiness could not be verified; do not"
          " assume localization is missing and do not call navigate."
      ),
      parameters={"type": "OBJECT", "properties": {}},
      behavior="BLOCKING",
  )


def navigate_tool() -> dict[str, Any]:
  return _tool(
      name="navigate",
      description=(
          "Navigate the robot to a named waypoint using GraphNav. The robot"
          " will walk to the specified waypoint and report when it arrives."
          " Call only after get_waypoints returns navigation_ready=true, use"
          " an exact name from that result, and call stow before navigating."
      ),
      parameters={
          "type": "OBJECT",
          "properties": {
              "waypoint": {
                  "type": "STRING",
                  "description": (
                      "Exact waypoint name from the latest successful"
                      " get_waypoints result. Never invent, normalize, or reuse"
                      " a name from an earlier session."
                  ),
              },
          },
          "required": ["waypoint"],
      },
      behavior="BLOCKING",
  )


def drive_tool() -> dict[str, Any]:
  return _tool(
      name="drive",
      description=(
          "Drive Spot without a waypoint for a short, bounded interval using"
          " body-frame velocities. Positive v_x moves forward, positive v_y"
          " moves left, and positive v_rot turns counterclockwise. The robot"
          " powers on and stands automatically. The current camera-arm joint"
          " pose is held relative to the body, so the gripper camera moves"
          " with Spot. Use small commands near people, furniture, stairs, or"
          " other obstacles."
      ),
      parameters={
          "type": "OBJECT",
          "properties": {
              "v_x": {"type": "NUMBER", "minimum": -0.8, "maximum": 0.8,
                      "description": "Forward velocity m/s (-0.8 backward to 0.8 forward)."},
              "v_y": {"type": "NUMBER", "minimum": -0.5, "maximum": 0.5,
                      "description": "Sideways velocity m/s (-0.5 right to 0.5 left)."},
              "v_rot": {"type": "NUMBER", "minimum": -1.0, "maximum": 1.0,
                        "description": "Yaw velocity rad/s (-1.0 CW to 1.0 CCW)."},
              "duration": {"type": "NUMBER", "minimum": 0.1, "maximum": 2.0,
                           "description": "Duration in seconds (0.1 to 2.0)."},
          },
          "required": ["v_x", "v_y", "v_rot", "duration"],
      },
      behavior="BLOCKING",
  )


def stop_spot_tool() -> dict[str, Any]:
  return _tool(
      name="stop",
      description=(
          "Immediately cancel base motion and navigation and freeze the arm."
          " Call this whenever motion is unsafe or the user asks Spot to stop."
      ),
      parameters={"type": "OBJECT", "properties": {}},
      behavior="BLOCKING",
  )


def look_tool() -> dict[str, Any]:
  return _tool(
      name="look",
      description=(
          "Aim Spot's gripper-mounted camera by rotating only the arm; the"
          " robot body and feet remain stationary. Use small steps and"
          " inspect the fresh camera frame returned after each move."
      ),
      parameters={
          "type": "OBJECT",
          "properties": {
              "direction": {
                  "type": "STRING",
                  "enum": ["up", "down", "left", "right"],
                  "description": "Direction to aim the camera view.",
              },
              "angle_rad": {
                  "type": "NUMBER", "minimum": 0.05, "maximum": 0.35,
                  "description": "Positive angular step in radians (0.15 small, 0.3 large).",
              },
          },
          "required": ["direction"],
      },
      behavior="BLOCKING",
  )


def detect_tool() -> dict[str, Any]:
  return _tool(
      name="detect",
      description=(
          "Use the Spot backend's Gemini Robotics detector to locate exactly"
          " one language-specified object or placement location in the stable"
          " hand-camera image."
          " When the user asks to pick up an object, assume it is in the"
          " current view and call this tool instead of claiming it is not"
          " visible without attempting detection."
          " The backend returns and stores the grasp location and the UI"
          " overlays it. Call only after the camera view has remained stable"
          " for at least three seconds. After a correct detection, call pick"
          " or place with no arguments before any camera or robot movement."
      ),
      parameters={
          "type": "OBJECT",
          "properties": {
              "instruction": {
                  "type": "STRING",
                  "description": 'A concise description, e.g. "the red cube on the floor".',
              },
          },
          "required": ["instruction"],
      },
      behavior="BLOCKING",
  )


def pick_tool() -> dict[str, Any]:
  return _tool(
      name="pick",
      description=(
          "Grasp the one-time location stored by the most recent successful"
          " detect call. This tool accepts no pixel coordinates. Call it only"
          " after confirming the detection overlay is on the requested object"
          " and before any robot or camera motion. The target is consumed"
          " after one attempt. A completed call still requires visual success"
          " verification. The backend reduces the gripper hold torque after"
          " the native Spot grasp succeeds."
      ),
      parameters={"type": "OBJECT", "properties": {}},
      behavior="BLOCKING",
  )


def place_tool() -> dict[str, Any]:
  return _tool(
      name="place",
      description=(
          "Place the currently held object at the one-time location stored by"
          " the most recent successful detect call. This tool accepts no"
          " coordinates. Call it immediately after detecting the intended"
          " placement surface, before any robot or camera movement. The"
          " backend moves the hand to the projected 3D target and releases"
          " only after arm arrival is confirmed; a failed approach leaves the"
          " gripper closed."
      ),
      parameters={"type": "OBJECT", "properties": {}},
      behavior="BLOCKING",
  )


def wait_for_pick_up_tool() -> dict[str, Any]:
  return _tool(
      name="wait_for_pick_up",
      description=(
          "Wait for a person to pick up the object currently held by Spot."
          " Call from carry pose after reaching the recipient. The tool"
          " monitors upward hand motion; when the hand rises by the threshold,"
          " it opens the gripper, waits briefly, closes the gripper, and stows"
          " the arm. A timeout leaves the object held."
      ),
      parameters={
          "type": "OBJECT",
          "properties": {
              "monitor_sec": {"type": "NUMBER", "minimum": 1.0, "maximum": 120.0,
                              "description": "Maximum seconds to wait for pickup."},
              "upward_threshold_m": {"type": "NUMBER", "minimum": 0.005, "maximum": 0.2,
                                     "description": "Upward displacement that triggers release; default 0.02 m."},
              "open_duration_sec": {"type": "NUMBER", "minimum": 0.1, "maximum": 10.0,
                                    "description": "Seconds to keep the gripper open; default 3 s."},
          },
      },
      behavior="BLOCKING",
  )


def stand_tool() -> dict[str, Any]:
  return _tool(
      name="stand",
      description="Power on the robot and stand up. Call this to wake the robot from sitting.",
      parameters={"type": "OBJECT", "properties": {}},
      behavior="BLOCKING",
  )


def sit_tool() -> dict[str, Any]:
  return _tool(
      name="sit",
      description=(
          "Sit the robot down and power off its motors. Call this only when"
          " the user's current instruction explicitly asks Spot to sit; never"
          " sit automatically when a task ends or Spot becomes idle."
      ),
      parameters={"type": "OBJECT", "properties": {}},
      behavior="BLOCKING",
  )


def get_battery_tool() -> dict[str, Any]:
  return _tool(
      name="get_battery",
      description="Check the robot battery level and power state.",
      parameters={"type": "OBJECT", "properties": {}},
      behavior="BLOCKING",
  )


def stow_tool() -> dict[str, Any]:
  return _tool(
      name="stow",
      description=(
          "Stow the arm safely. If the robot is holding an object, it will"
          " move the arm to a stable carry pose to prevent collision. If it"
          " is not holding anything, it will stow the arm completely."
      ),
      parameters={"type": "OBJECT", "properties": {}},
      behavior="BLOCKING",
  )


def spot_tools() -> list[dict[str, Any]]:
  """Spot navigation + manipulation tools."""
  return _wrap([
      health_check_tool(),
      get_waypoints_tool(),
      navigate_tool(),
      drive_tool(),
      stop_spot_tool(),
      look_tool(),
      detect_tool(),
      pick_tool(),
      place_tool(),
      wait_for_pick_up_tool(),
      stand_tool(),
      sit_tool(),
      stow_tool(),
      ack_tool(),
      send_message_tool(enable_send_message_to_user=True),
  ])
