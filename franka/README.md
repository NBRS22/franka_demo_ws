# Franka Backend

## What is this?

The **Franka Backend** is a lightweight FastAPI server that sits between the **Agent Server** and the physical Franka arm. It:

- Receives HTTP commands from the Agent (`pick`, `place`, `home`, `stop`, `conveyor`).
- Forwards them to the robot controller via **ZMQ** (msgpack-encoded messages).
- Subscribes to a **ZMQ camera stream** and exposes it as an MJPEG HTTP stream.

```
Agent Server (port 8000)
        │  HTTP
        ▼
Franka Backend (port 8888)
        │  ZMQ REQ  ──► Robot Controller  (port 5558)
        │  ZMQ SUB  ◄── Camera Publisher  (port 5555)
```

---

## Requirements

- Python 3.10+
- [`uv`](https://github.com/astral-sh/uv)
- A running ZMQ robot controller and ZMQ camera publisher (from the Franka SDK / ROS side)

---

## Setup

### 1. Install dependencies

```bash
cd franka
uv sync
```

### 2. Configure

Copy `.config.example` to `.config` and edit if needed:

```bash
cp .config.example .config
```

`.config` content:

```ini
# Server
BACKEND_HOST = 0.0.0.0
BACKEND_PORT = 8888

# ZMQ endpoints
ZMQ_CAMERA_URL      = tcp://localhost:5555
ZMQ_ROBOT_URL       = tcp://localhost:5558
ZMQ_ROBOT_TIMEOUT_MS = 5000
```

| Variable | Default | Description |
|----------|---------|-------------|
| `BACKEND_HOST` | `0.0.0.0` | Host to bind the HTTP server |
| `BACKEND_PORT` | `8888` | Port of this backend (must match `ROBOT_URL` in `agent/.env`) |
| `ZMQ_CAMERA_URL` | `tcp://localhost:5555` | ZMQ PUB socket publishing JPEG frames |
| `ZMQ_ROBOT_URL` | `tcp://localhost:5558` | ZMQ REQ socket of the robot controller |
| `ZMQ_ROBOT_TIMEOUT_MS` | `5000` | Timeout for robot commands (ms) |

---

## Run

```bash
cd franka
uv run python main.py
```

The server starts on **http://localhost:8888**.

Check it is healthy:
```bash
curl http://localhost:8888/health
# {"status": "ok", "camera": "zmq", "has_frame": true}
```

---

## REST API

| Method | Route | Body | Description |
|--------|-------|------|-------------|
| `GET` | `/health` | — | Health check + camera status |
| `GET` | `/camera/stream` | — | MJPEG live stream |
| `GET` | `/camera/snapshot` | — | Latest frame as JPEG |
| `POST` | `/pick` | `{"x": int, "y": int, "label": str}` | Pick object at pixel (x, y) |
| `POST` | `/place` | `{"x": int, "y": int, "label": str}` | Place object at pixel (x, y) |
| `POST` | `/home` | — | Move arm to home position |
| `POST` | `/stop` | — | Emergency stop |
| `POST` | `/conveyor` | `{"state": "on\|off", "direction": "forward\|backward"}` | Control conveyor belt |

---

## Project structure

```
franka/
├── main.py          # FastAPI app — routes and entry point
├── core/
│   ├── config.py    # Loads .config → env vars helpers
│   ├── camera.py    # ZmqCamera — background thread, ZMQ SUB subscriber
│   └── robot.py     # ZMQ REQ commands (pick, place, home, stop, conveyor)
├── .config.example  # Configuration template
└── pyproject.toml
```

---

## Running the full Franka pipeline

See the [root README](../README.md#running-the-franka-pipeline) for the complete startup sequence.
