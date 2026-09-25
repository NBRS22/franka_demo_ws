# Gemini Robotics ER 2.0 — Robot Orchestrator

An application that connects **Google Gemini Robotics ER 2.0** to physical robot hardware via the Gemini Live API. It supports real-time audio/video interaction, tool dispatch, and robot control for multiple embodiments.

---

## Repository structure

| Package | Description | Doc |
|---------|-------------|-----|
| [`agent/`](./agent/README.md) | Core orchestration server — Gemini Live API, WebSocket, browser UI | [README](./agent/README.md) |
| [`franka/`](./franka/README.md) | Franka arm backend — HTTP → ZMQ bridge, camera stream | [README](./franka/README.md) |
| [`franka_vla/`](./franka_vla/) | Franka VLA backend — natural language → motor commands via VLA model | — |
| [`spot/`](./spot/) | Boston Dynamics Spot SDK integration | — |

---

## Architecture overview

```
Browser
  │  WebSocket
  ▼
Agent Server  ──────────────────────────────►  Gemini Live API
(agent/, port 8000)                          (audio/video stream)
  │
  │  HTTP (REST)
  ├──────────────────►  Franka Backend      (franka/,     port 8888)
  │                           │  ZMQ
  │                           ├──► Robot Controller
  │                           └──► Camera Publisher
  │
  └──────────────────►  Franka VLA Backend  (franka_vla/, port 8889)
```

---

## Running the Franka pipeline

The Franka pipeline requires **two servers running simultaneously**: the Franka backend and the Agent server.

### Prerequisites

- Python 3.10+
- [`uv`](https://github.com/astral-sh/uv)
- A valid Gemini API key → [aistudio.google.com/app/apikey](https://aistudio.google.com/app/apikey)
- A running ZMQ robot controller + ZMQ camera publisher (Franka SDK / ROS side)

---

### Step 1 — Start the Franka backend

```bash
cd franka
cp .config.example .config   # first time only
uv sync                       # first time only
uv run python main.py
```

Verify it is up:
```bash
curl http://localhost:8888/health
# Expected: {"status": "ok", "camera": "zmq", "has_frame": true}
```

> The backend binds on port **8888** by default. Change `BACKEND_PORT` in `franka/.config` if needed.

---

### Step 2 — Start the Agent server

In a **second terminal**:

```bash
cd agent
cp .env.example .env          # first time only
# Edit .env and set GEMINI_API_KEY = "your_key_here"
uv sync                       # first time only
uv run python main.py
```

Make sure `ROBOT_URL` in `agent/.env` points to the Franka backend:

```dotenv
ROBOT_URL = "http://localhost:8888"
```

---

### Step 3 — Open the UI

Go to **http://localhost:8000** in your browser.

1. In the agent selector (top-left), choose **Franka**.
2. Click **Connect** — the session starts and the camera feed appears.
3. Talk or type to control the arm.

---

### Franka VLA variant

To use the VLA (Vision-Language-Action) variant instead:

```bash
# Terminal 1 — start the VLA backend (port 8889)
cd franka_vla
uv sync
uv run python main.py

# Terminal 2 — start the Agent server pointing to the VLA backend
cd agent
ROBOT_URL="http://localhost:8889" uv run python main.py
```

Then select **Franka VLA** in the agent dropdown. Instead of pixel-coordinate pick/place, you give natural language instructions (e.g. *"pick up the red cube"*).

---

## Quick start — Human mode (no robot)

To test the UI and Gemini connection without any physical robot:

```bash
cd agent
cp .env.example .env
# set GEMINI_API_KEY in .env
uv sync
uv run python main.py
```

Open **http://localhost:8000**, keep the agent set to **Human**, and click **Connect**.  
Your browser webcam and microphone are used as the sensor input.

---

## Environment & configuration

Each package has its own configuration file:

| Package | Config file | Template |
|---------|------------|---------|
| `agent/` | `agent/.env` | `agent/.env.example` |
| `franka/` | `franka/.config` | `franka/.config.example` |
| `franka_vla/` | `franka_vla/.config` | `franka_vla/.config.example` |

All secrets (API keys) are loaded from these local files and are excluded from git via `.gitignore`.

---

## Further reading

- [Agent Server — setup, API reference, adding a new embodiment](./agent/README.md)
- [Franka Backend — ZMQ configuration, REST API](./franka/README.md)
