"""Shared tool helpers and tools used by all embodiments."""

from typing import Any


# ---------------------------------------------------------------------------
# Builder helpers
# ---------------------------------------------------------------------------

def _tool(name: str, description: str, parameters: dict[str, Any], behavior: str | None = None) -> dict[str, Any]:
  d: dict[str, Any] = {
      "name": name,
      "description": description,
      "parameters": parameters,
  }
  if behavior is not None:
    d["behavior"] = behavior
  return d


def _wrap(decls: list[dict[str, Any]]) -> list[dict[str, Any]]:
  return [{"functionDeclarations": decls}]


# ---------------------------------------------------------------------------
# Tools shared across all embodiments
# ---------------------------------------------------------------------------

def ack_tool() -> dict[str, Any]:
  return _tool(
      name="ack",
      description=(
          "Call this tool to acknowledge the regular heartbeat and indicate"
          " that no new intervention is needed. This is the default action when"
          " no other specific action is required."
      ),
      parameters={"type": "OBJECT", "properties": {}},
      behavior="BLOCKING",
  )


def send_message_tool(enable_send_message_to_user: bool = False) -> dict[str, Any]:
  if enable_send_message_to_user:
    desc = (
        "Send a text message to the user or another robot agent in the fleet."
        " Use this to delegate tasks or coordinate with other robots or to"
        " talk to the user."
    )
    target_desc = "The target robot agent name (e.g. 'duo', 'apollo', 'user')."
  else:
    desc = (
        "Send a text message to another robot agent in the fleet."
        " Use this to delegate tasks or coordinate with other robots."
    )
    target_desc = "The target robot agent name (e.g. 'duo', 'apollo')."

  return _tool(
      name="send_message",
      description=desc,
      parameters={
          "type": "OBJECT",
          "properties": {
              "target": {"type": "STRING", "description": target_desc},
              "message": {
                  "type": "STRING",
                  "description": (
                      "The message to send. Can be a task instruction, "
                      "status update, or coordination signal. Do not send "
                      "emojis in the message."
                  ),
              },
          },
          "required": ["target", "message"],
      },
      behavior="NON_BLOCKING",
  )
