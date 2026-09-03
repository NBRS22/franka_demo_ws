# Agent — Agent Configuration

This folder assembles agents. Each agent is a combination of a developer
instruction (a `.md` file) and a set of tool declarations.

## Structure

```
agent/
├── base.py           ← Agent dataclass + _load() helper
├── agent.py          ← _PRESETS registry + from_name() entry point
└── presets/
    ├── human.py      ← agent guides a human operator step by step
    ├── franka.py     ← Franka arm (pixel-coordinate pick/place)
    ├── franka_vla.py ← Franka arm via VLA (natural language instructions)
    └── spot.py       ← Boston Dynamics Spot
```

## How it works

`main.py` calls `agent_lib.from_name("franka")` with the agent name from
the WebSocket URL. `from_name()` returns an `Agent` object containing:

- `developer_instruction` — content of the `.md` file loaded from `prompts/`
- `tools` — tool declarations sent to Gemini at session setup
- `system_instruction` — always `""`: `SessionManager` then falls back to its
  default generic instruction, to which `developer_instruction` is appended

## Adding a new agent

1. Create `presets/<name>.py` with a factory function:

   ```python
   from agents.base import Agent, _load
   from tools import tools as tools_lib

   def my_agent() -> Agent:
       return Agent(
           name="my_agent",
           system_instruction="",
           developer_instruction=_load("my_agent_di.md"),
           tools=tools_lib.my_agent_tools(),
       )
   ```

2. Import and register it in `agent.py`:

   ```python
   from agents.presets.my_agent import my_agent
   _PRESETS = {
       ...
       "my_agent": my_agent,
   }
   ```

3. Create the prompt at `prompts/my_agent_di.md`
   (see `prompts/README.md` for writing rules).

4. Create the tool set in `tools/my_agent_tools.py`
   and re-export it from `tools/tools.py`.
