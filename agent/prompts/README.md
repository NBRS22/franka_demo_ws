# Prompts — Developer Instructions

Each agent type has one markdown file here. The file is loaded directly
by `agent/agent.py` and sent to Gemini as the developer instruction for the session.

## Current agents

| File | Agent | Embodiment |
|---|---|---|
| `franka_di.md` | `franka` | Franka arm — pixel-coordinate pick/place |
| `franka_vla_di.md` | `franka_vla` | Franka arm — VLA natural language |
| `human_di.md` | `human` | Human operator guided step by step |
| `spot_di.md` | `spot` | Boston Dynamics Spot |

## Adding a new agent

**Step 1 — Create the instruction file**

Create `<agent_name>_di.md` in this directory with the following sections (in order):

```
# <Agent Name> Instructions

## Persona
One paragraph: role, communication style, what the agent controls.

## <Domain-specific workflow>
Step-by-step rules for the main tasks (pick/place, navigation, etc.).
See existing files for reference.

## Heartbeat
What to do at each heartbeat tick:
- No active task → call `ack`
- Task in progress → observe and decide the next step
- Task complete → inform the user via send_message

## Safety
Hard rules: stop conditions, forbidden actions, edge cases.

## Tool Constraints
- `send_message`: always use this to speak to the user — never reply directly.
- CRITICAL RULE: never reply with text or audio directly.
- CRITICAL RULE: call the action tool FIRST, then send_message in the same turn.
```

**Step 2 — Register the agent in `agent/agent.py`**

Add a classmethod and register it in `from_name()`:

```python
@classmethod
def my_agent(cls) -> "Agent":
    return cls(
        name="my_agent",
        system_instruction="",
        developer_instruction=_load("my_agent_di.md"),
        tools=tools_lib.my_agent_tools(),
    )
```

**Step 3 — Add the tool set in `tools/`**

Create `tools/my_agent_tools.py` and add a `my_agent_tools()` function.
Re-export it from `tools/tools.py`.

## Rules for writing instruction files

- **One action per heartbeat** — one tool call per turn, no chaining.
- **No direct reply** — always communicate via `send_message(target='user', ...)`.
- **Action first** — call the physical tool before `send_message` in the same turn.
- **Atomic instructions** — for NL-based agents (VLA, human), never combine two actions with "and".
- **No invented data** — never infer coordinates, waypoint names, or object positions not visible in the camera frame.
- **No emojis** in `send_message` content.
