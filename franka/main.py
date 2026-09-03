import logging
import time
from contextlib import asynccontextmanager

import fastapi
import uvicorn
from fastapi.responses import Response, StreamingResponse
from pydantic import BaseModel, Field

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


app = fastapi.FastAPI(title="Franka Backend", lifespan=lifespan)


# ---------------------------------------------------------------------------
# Request models
# ---------------------------------------------------------------------------

class PickRequest(BaseModel):
    x: int = Field(..., ge=0, le=1000)
    y: int = Field(..., ge=0, le=1000)
    label: str


class PlaceRequest(BaseModel):
    x: int = Field(..., ge=0, le=1000)
    y: int = Field(..., ge=0, le=1000)
    label: str


class ConveyorRequest(BaseModel):
    state: str = Field(..., pattern="^(on|off)$")
    direction: str = Field(..., pattern="^(forward|backward)$")


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
    return Response(content=jpeg, media_type="image/jpeg", headers={"Cache-Control": "no-store"})


# ---------------------------------------------------------------------------
# Health
# ---------------------------------------------------------------------------

@app.get("/health")
def health():
    return {"status": "ok", "camera": "zmq", "has_frame": camera.has_frame}


# ---------------------------------------------------------------------------
# Robot actions (mock)
# ---------------------------------------------------------------------------

@app.post("/pick")
def pick(req: PickRequest):
    return robot.pick(x=req.x, y=req.y, label=req.label)


@app.post("/place")
def place(req: PlaceRequest):
    return robot.place(x=req.x, y=req.y, label=req.label)


@app.post("/home")
def home():
    return robot.home()


@app.post("/stop")
def stop():
    return robot.stop()


@app.post("/conveyor")
def conveyor(req: ConveyorRequest):
    return robot.conveyor(state=req.state, direction=req.direction)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    uvicorn.run(app, host=backend_host(), port=backend_port())
