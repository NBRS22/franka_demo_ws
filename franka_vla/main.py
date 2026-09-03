"""Franka VLA backend server.

Bridges the agent server and the VLA robot worker via ZMQ.

Endpoints:
  GET  /camera/stream       MJPEG stream
  GET  /camera/snapshot     Single JPEG frame
  GET  /health
  POST /run_instruction     {"instruction": str}  — blocks until VLA done
  POST /home
  POST /stop
"""

import logging
import time
from contextlib import asynccontextmanager

import fastapi
import uvicorn
from fastapi.responses import Response, StreamingResponse
from pydantic import BaseModel

from core import robot
from core.camera import ZmqCamera
from core.config import backend_host, backend_port, load, zmq_camera_url

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

load()
camera = ZmqCamera(url=zmq_camera_url())


@asynccontextmanager
async def lifespan(app: fastapi.FastAPI):
    camera.start()
    yield
    camera.stop()


app = fastapi.FastAPI(title="Franka VLA Backend", lifespan=lifespan)


# ---------------------------------------------------------------------------
# Request models
# ---------------------------------------------------------------------------

class RunInstructionRequest(BaseModel):
    instruction: str


# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------

def _mjpeg_generator():
    while True:
        jpeg = camera.latest_jpeg
        if jpeg is None:
            time.sleep(0.05)
            continue
        yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"


@app.get("/camera/stream")
def camera_stream():
    return StreamingResponse(
        _mjpeg_generator(),
        media_type="multipart/x-mixed-replace; boundary=frame",
        headers={"Cache-Control": "no-cache", "Connection": "keep-alive"},
    )


@app.get("/camera/snapshot")
def camera_snapshot():
    jpeg = camera.latest_jpeg
    if jpeg is None:
        raise fastapi.HTTPException(status_code=503, detail="No frame available yet")
    return Response(
        content=jpeg,
        media_type="image/jpeg",
        headers={"Cache-Control": "no-store"},
    )


# ---------------------------------------------------------------------------
# Health
# ---------------------------------------------------------------------------

@app.get("/health")
def health():
    return {"status": "ok", "camera": "zmq", "has_frame": camera.has_frame}


# ---------------------------------------------------------------------------
# Robot actions — forwarded to the VLA worker over ZMQ
# ---------------------------------------------------------------------------

@app.post("/run_instruction")
def run_instruction(req: RunInstructionRequest):
    """Send a natural language instruction to the VLA worker.

    This call blocks until the VLA worker signals completion (or times out).
    """
    logger.info("run_instruction: %s", req.instruction)
    result = robot.run_instruction(req.instruction)
    if not result.get("success", True) and "error" in result:
        raise fastapi.HTTPException(status_code=500, detail=result["error"])
    return result


@app.post("/home")
def home():
    logger.info("home")
    result = robot.home()
    if not result.get("success", True) and "error" in result:
        raise fastapi.HTTPException(status_code=500, detail=result["error"])
    return result


@app.post("/stop")
def stop():
    logger.info("stop")
    result = robot.stop()
    if not result.get("success", True) and "error" in result:
        raise fastapi.HTTPException(status_code=500, detail=result["error"])
    return result


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    uvicorn.run(app, host=backend_host(), port=backend_port())
