# SAM3 ZMQ Server

Loads the SAM3 segmentation model once and serves inference requests over a ZMQ REP socket.
Designed to run on a machine with a CUDA GPU and be called remotely by any ZMQ client (test client, ROS2 node, etc.).

---

## Architecture

```
sam3_server/
├── config.py       ServerConfig dataclass + CLI argument parser
├── model.py        SAM3 model loading and CUDA warm-up
├── inference.py    Point + text inference pipeline, best-mask selection
├── protocol.py     msgpack request/response serialization
├── server.py       ZMQ REP loop
└── __main__.py     Entry point — validates environment, loads model, starts server
```

---

## Requirements

- Python 3.10+
- CUDA-capable GPU (recommended)
- `pyzmq`, `msgpack` — install with `pip install pyzmq msgpack`
- SAM3 model weights loaded via `build_sam3_image_model()`

---

## Launching the server

```bash
cd <repo>/SAM3            # from the SAM3 directory, in the SAM3 conda env
python -m sam3_server
python -m sam3_server --port 5557 --threshold 0.05
python -m sam3_server --device cpu --no-warmup
```
Full setup (conda env, weights) in [`../README_FP3.md`](../README_FP3.md).

| Argument | Default | Description |
|---|---|---|
| `--port` | `5557` | ZMQ REP socket port |
| `--device` | `cuda` | Torch device (`cuda` or `cpu`) |
| `--threshold` | `0.05` | Confidence threshold — lower means more permissive |
| `--no-warmup` | off | Skip the dummy inference at startup |

---

## Startup sequence

1. Logs configuration (port, device, threshold)
2. Checks CUDA availability
3. Loads the SAM3 model and processor
4. Logs GPU memory allocated and number of parameters
5. Runs a warm-up inference to compile CUDA kernels
6. Binds the ZMQ socket and waits for requests

---

## Protocol

All messages are serialized with **msgpack**.

**Request fields**

| Field | Type | Required | Description |
|---|---|---|---|
| `image` | `bytes` | yes | JPEG or PNG compressed image |
| `point_x` | `float` | yes | Click X coordinate in pixels |
| `point_y` | `float` | yes | Click Y coordinate in pixels |
| `text` | `str` | yes | Object label (`"person"`, `"red cup"`, …) |
| `threshold` | `float` | no | Override confidence threshold (default `0.05`) |

**Health check** — `{"action": "health"}` (no image needed) is answered with `{"status": "ok"}`; the FP3 launch file polls it before starting the pipeline.

**Response fields — mask found**

| Field | Type | Description |
|---|---|---|
| `status` | `"ok"` | |
| `has_mask` | `True` | |
| `mask` | `bytes` | H×W bool array, row-major |
| `mask_shape` | `[H, W]` | Shape to reconstruct the numpy array |
| `score` | `float` | Confidence score of the best mask |

**Response fields — no mask above threshold**

| Field | Type |
|---|---|
| `status` | `"ok"` |
| `has_mask` | `False` |

**Response fields — error**

| Field | Type |
|---|---|
| `status` | `"error"` |
| `error_msg` | `str` |

---

## Warm-up

On startup the server runs one dummy inference before accepting real requests.
This triggers CUDA JIT compilation so the first real request is not penalized.
Use `--no-warmup` to skip it during development or when running on CPU.

---

## Notes

- The server is **single-threaded** — it processes one request at a time.
- The server always returns at most **one mask** (the highest-scoring one).
- The ZMQ REP pattern is synchronous: send → wait → receive. One outstanding request at a time per client.
