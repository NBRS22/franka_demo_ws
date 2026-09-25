# SAM3 ZMQ Client — Test Tool

A lightweight Python test client for the SAM3 ZMQ server.
Sends a segmentation request (image + click point + text label), receives the mask,
and displays the result as a side-by-side matplotlib figure.

**This client is for testing only.** In the FP3 pipeline the ROS 2 side is `franka_demo_ws/src/sam3_bridge`.

---

## Architecture

```
sam3_client/
├── config.py       ClientConfig dataclass + CLI argument parser
├── client.py       Sam3Client class — ZMQ REQ socket, encode/decode
├── visualize.py    Matplotlib visualization of the segmentation result
└── __main__.py     Entry point — reads config, calls client, shows result
```

---

## Requirements

- Python 3.10+
- `pyzmq`, `msgpack`, `Pillow`, `matplotlib`, `numpy`
- The SAM3 ZMQ server must be running and reachable

---

## Usage

```bash
python -m sam3_client <image> <x> <y> <text> [options]
```

| Argument | Type | Description |
|---|---|---|
| `image` | positional | Path to the input image |
| `x` | positional | Click X coordinate in pixels |
| `y` | positional | Click Y coordinate in pixels |
| `text` | positional | Object label (`"person"`, `"red cup"`, …) |
| `--host` | optional | SAM3 server IP (default: `localhost`) |
| `--port` | optional | SAM3 server port (default: `5557`) |
| `--threshold` | optional | Confidence threshold (default: `0.05`) |
| `--timeout` | optional | ZMQ receive timeout in ms (default: `30000`) |
| `--output` | optional | Output image save path (default: `sam3_client/output/result.png`) |
| `--no-display` | flag | Skip `plt.show()` — useful in headless environments |

---

## Visualization

The output figure shows two panels side by side:

- **Left** — original image with the click point marked as a red star
- **Right** — best mask overlaid in semi-transparent green, with a contour outline and the confidence score

---

## Mask reconstruction

The mask is returned as raw bytes. To use it programmatically:

```
mask = np.frombuffer(resp["mask"], dtype=bool).reshape(resp["mask_shape"])
# mask : (H, W) bool numpy array
```

---

## Notes

- The client sends the image compressed as JPEG (quality 95) to reduce transfer size.
- Point coordinates are in **pixels** — not normalized. The server handles normalization internally.
- Only one mask is returned (the highest-scoring one above the threshold).
- If `has_mask` is `False`, the threshold can be lowered with `--threshold 0.01`.
