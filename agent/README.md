# Agent Server — Gemini Robotics ER 2.0

## What is this?

The **Agent Server** is the core orchestration layer of the application. It connects the **Gemini Robotics ER 2.0** model to physical robots (or a human operator) via the **Gemini Live API**.

It exposes:
- A **WebSocket server** that maintains a bidirectional audio/video streaming session with Gemini.
- A **REST API** (FastAPI) for runtime configuration and message injection.
- A **browser UI** for real-time interaction.

```
Browser ──WebSocket──► Agent Server ──WebSocket──► Gemini Live API
                             │
                     Robot Embodiment
                   (Spot / Franka / Human)
```

---

## Setup

### Prerequisites

- Python 3.10+
- [`uv`](https://github.com/astral-sh/uv) (recommended package manager)
- A valid Gemini API key → [aistudio.google.com/app/apikey](https://aistudio.google.com/app/apikey)

### 1. Install dependencies

```bash
cd agent
uv sync
```

### 2. Configure the environment

Copy `.env.example` to `.env` and fill in your API key:

```bash
cp .env.example .env
```

Open `.env` and replace the `GEMINI_API_KEY` value:

```dotenv
GEMINI_API_KEY = "your_key_here"
```

All other values have sensible defaults. Only `GEMINI_API_KEY` is required for a first run.

### 3. Environment variables

| Variable | Default | Description |
|----------|---------|-------------|
| `GEMINI_API_KEY` | *(required)* | Google AI Studio API key |
| `MODEL` | `models/gemini-robotics-er-2-streaming-preview` | Gemini model to use |
| `RESPONSE_MODALITY` | `TEXT` | `TEXT` or `AUDIO` |
| `PORT` | `8000` | Server port |
| `ROBOT_URL` | `http://localhost:8888` | URL of the physical robot server |
| `USE_TTS` | `false` | Enable Google Text-to-Speech |
| `TTS_API_KEY` | *(empty)* | Google TTS API key (required if `USE_TTS=true`) |
| `HEARTBEAT_ENABLED` | `true` | Keep the Gemini session alive between turns |
| `MEDIA_RESOLUTION` | `low` | Video resolution (`low` / `medium` / `high`) |
| `MOCK_ROBOT` | `false` | Simulation mode — no real robot needed |

---

## Run

```bash
cd agent
uv run python main.py
```

The server starts on **http://localhost:8000**.  
Open that URL in your browser to access the UI.

To change the port:
```bash
PORT=9000 uv run python main.py
```

---

## Project structure

```
agent/
├── main.py                  # Entry point — starts uvicorn
├── config.py                # Loads .env → ServerConfig
│
├── app/
│   ├── config.py            # ServerConfig dataclass
│   └── server.py            # FastAPI app: REST routes + WebSocket endpoint
│
├── agents/
│   ├── agent.py             # Agent registry (from_name)
│   ├── base.py              # Agent dataclass (name, system instruction, tools)
│   └── presets/             # One file per agent: human, spot, franka, franka_vla
│
├── embodiment/
│   ├── base.py              # Abstract Embodiment base class
│   ├── human.py             # Browser webcam/microphone
│   ├── franka/              # Franka arm (standard + VLA)
│   └── spot/                # Boston Dynamics Spot
│
├── tools/
│   ├── tools.py             # Entry point (imports all tool sets)
│   ├── common_tools.py      # Shared tools (ack, send_message)
│   ├── human_tools.py       # Tools for the human embodiment
│   ├── franka_tools.py      # Tools for Franka
│   ├── franka_vla_tools.py  # Tools for Franka VLA
│   └── spot_tools.py        # Tools for Spot
│
├── session/
│   ├── config.py            # SessionConfig + known model list
│   └── manager.py           # Main Live API loop (heartbeat, streaming)
│
├── model/
│   └── tts_client.py        # Google Cloud TTS client wrapper
│
└── ui/
    ├── index.html            # Single-page browser UI
    └── modules/              # ES module JS components
```

---

## Adding a new embodiment

An embodiment is the abstraction layer between the Gemini agent and a specific robot (or device). Follow these 4 steps to create a new one, e.g. `myrobot`.

### Step 1 — Create the embodiment class

Create `agent/embodiment/myrobot/myrobot_embodiment.py`:

```python
import asyncio
from typing import Any
from embodiment import base
from tools import tools as tools_lib

class MyRobotEmbodiment(base.Embodiment):

    def __init__(self, robot_url: str):
        self.audio_queue = asyncio.Queue()
        self.video_queue = asyncio.Queue()
        self.text_queue  = asyncio.Queue()
        self.robot_url   = robot_url
        # initialize your robot connection here

    def get_audio_queue(self) -> asyncio.Queue:
        return self.audio_queue

    def get_video_queue(self) -> asyncio.Queue:
        return self.video_queue

    def get_text_queue(self) -> asyncio.Queue:
        return self.text_queue

    def get_tools(self) -> list[dict[str, Any]]:
        return tools_lib.myrobot_tools()   # created in step 2

    def get_system_instruction(self) -> str:
        return "You control MyRobot. ..."

    async def execute_action(self, action_name: str, **kwargs: Any) -> str:
        if action_name == "ack":
            return "ok"
        # add your action logic here
        return f"MyRobot executed {action_name}"

    async def close(self) -> None:
        pass   # release resources if needed
```

### Step 2 — Declare the tools

Create `agent/tools/myrobot_tools.py`:

```python
def myrobot_tools() -> list[dict]:
    return [{
        "functionDeclarations": [
            {
                "name": "ack",
                "description": "Acknowledge a message.",
                "parameters": {"type": "object", "properties": {}},
            },
            {
                "name": "do_something",
                "description": "Do something on MyRobot.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "param": {"type": "string", "description": "A parameter."}
                    },
                    "required": ["param"],
                },
            },
        ]
    }]
```

Then export it in `agent/tools/tools.py`:

```python
from tools.myrobot_tools import myrobot_tools

__all__ = [..., "myrobot_tools"]
```

### Step 3 — Create the agent preset

Create `agent/agents/presets/myrobot.py`:

```python
from agents.base import Agent
from tools import tools as tools_lib

def myrobot() -> Agent:
    return Agent(
        name="myrobot",
        system_instruction="You are an assistant controlling MyRobot.",
        developer_instruction="",
        tools=tools_lib.myrobot_tools(),
    )
```

Register it in `agent/agents/agent.py`:

```python
from agents.presets.myrobot import myrobot

_PRESETS = {
    ...,
    "myrobot": myrobot,
}
```

### Step 4 — Wire it into the WebSocket endpoint

In `agent/app/server.py`, add a branch inside `websocket_endpoint`:

```python
elif agent_name == "myrobot":
    from embodiment.myrobot import myrobot_embodiment as myrobot_lib
    current_embodiment = myrobot_lib.MyRobotEmbodiment(robot_url=config.robot_url)
    app.state.active_poller_ref = None   # or your camera poller if you have one
```

That's it — the new agent is immediately available via `agent_name=myrobot` on the WebSocket.

---

## REST API — Quick reference

| Method | Route | Description |
|--------|-------|-------------|
| `GET` | `/` | Browser UI |
| `GET` | `/api/server_defaults` | Default model and options |
| `GET` | `/api/agent_config/{name}` | Resolved configuration for an agent |
| `POST` | `/api/send` | Inject text into the active session |
| `POST` | `/api/config/instructions` | Update agent instructions at runtime |
| `DELETE` | `/api/config/instructions` | Reset instructions to startup values |
| `GET` | `/api/camera` | MJPEG stream from the robot camera |
| `WS` | `/ws` | Main Live API WebSocket session |
