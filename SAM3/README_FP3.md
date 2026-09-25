# SAM3 for the FP3 pick pipeline

This directory is a copy of [`facebookresearch/sam3`](https://github.com/facebookresearch/sam3)
(commit `96914d2`, the upstream `README.md` is kept untouched) **plus the parts this project added**,
so that cloning the FP3 repository is enough to get a working segmentation server:

| Added by this project | What it is |
|---|---|
| `sam3_server/` | ZMQ server: loads SAM3 once and answers "segment the object under this click". **This is what the robot pipeline talks to.** |
| `sam3_client/` | Small test client (image + click + label → mask figure), no ROS |
| `main.py` | Stand-alone smoke test of the model (needs an `img.png` next to it) |
| `pyproject.toml` | Adds an `inference` extra (`einops`, `pycocotools`, `psutil`, `setuptools<80`) needed to run the model |

Everything else (`sam3/`, `examples/`, `scripts/`, training code, …) is unmodified upstream.
The SAM License (`LICENSE`) applies to the SAM 3 materials and must stay with any redistribution.

## Role in the pipeline

```
franka_demo_ws / sam3_bridge_node ──ZMQ REQ──▶ sam3_server (this directory, port 5557, GPU)
   image + pixel (x, y) + label                     └─▶ best mask (bytes) + score
```

`franka_demo_ws/src/franka_demo_bringup` **starts this server for you** (and waits for its health check):
`ros2 launch franka_demo_bringup franka_demo.launch.py`. You only need to run it by hand to test it.

## Setup

**Quick way** (from the repository root): `scripts/install_conda.sh && scripts/setup_envs.sh SAM3 && scripts/download_models.sh sam3`
(the account steps for the gated weights are in step 4 below and in the root README, section 4.3). The manual steps follow.

Requirements: NVIDIA GPU with CUDA ≥ 12.6 (the model uses about 3.6 GB of GPU memory), Ubuntu 24.04, `conda`.

```bash
# 1. Environment. The name SAM3 (upper case) is what franka_demo.launch.py expects.
conda create -n SAM3 python=3.12 -y
conda activate SAM3

# 2. PyTorch with CUDA (versions used here: torch 2.10.0 + cu128)
pip install torch==2.10.0 torchvision --index-url https://download.pytorch.org/whl/cu128

# 3. This directory, editable, with the inference extra, plus the server's ZMQ dependencies
cd $FP3_ROOT/SAM3
pip install -e ".[inference]"
pip install pyzmq msgpack

# 4. Model weights: gated Hugging Face repository
#    Request access at https://huggingface.co/facebook/sam3, then create a token and:
pip install -U huggingface_hub
hf auth login
```
The weights are downloaded on the first start and cached in `~/.cache/huggingface` (they are **not** in this repository).
The editable install (`pip install -e`) records the location of this directory: run step 3 again if you move/clone it elsewhere.

## Run the server

```bash
conda activate SAM3
cd $FP3_ROOT/SAM3
python -m sam3_server                        # port 5557, cuda, threshold 0.05
python -m sam3_server --port 5557 --threshold 0.05
python -m sam3_server --device cpu --no-warmup
```

| Argument | Default | Meaning |
|---|---|---|
| `--port` | `5557` | ZMQ REP port (the pipeline expects 5557) |
| `--device` | `cuda` | `cuda` or `cpu` |
| `--threshold` | `0.05` | Confidence threshold (lower = more permissive) |
| `--no-warmup` | off | Skip the start-up dummy inference |

A healthy start ends with `Server ready — waiting for connections on port 5557`. Loading takes about 10 s once the
weights are cached. A `Warm-up failed (non-fatal) … BFloat16 and Float` warning is known and harmless.

## Test it

```bash
# Health check (what franka_demo_ws does before starting the pipeline)
python - <<'EOF'
import zmq, msgpack
s = zmq.Context().socket(zmq.REQ); s.connect("tcp://127.0.0.1:5557")
s.send(msgpack.packb({"action": "health"})); print(msgpack.unpackb(s.recv()))   # {'status': 'ok'}
EOF

# Segment something: image, click x, click y, label
python -m sam3_client path/to/image.jpg 640 360 "cube" --host localhost --port 5557 --no-display
# result written to sam3_client/output/result.png (git-ignored)
```

Protocol details (request/response fields, error format) are in [`sam3_server/README.md`](sam3_server/README.md).

## Troubleshooting

- **`hf auth login` / 401 / gated repo**: access to `facebook/sam3` must be granted to your Hugging Face account first.
- **`CUDA not available`**: check `nvidia-smi` and that the torch build matches (`python -c "import torch; print(torch.cuda.is_available())"`).
- **Port 5557 already in use**: a previous server is still running (`pkill -f sam3_server`).
- **`ModuleNotFoundError: sam3`**: the editable install was made in another location; rerun `pip install -e ".[inference]"` here.
- The server is single-threaded and returns at most one mask per request.
