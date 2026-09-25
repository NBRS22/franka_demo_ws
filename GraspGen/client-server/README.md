# GraspGen Standalone Server

GraspGen can be run as a standalone ZMQ server so that any application — on the same machine or across the network — can request 6-DOF grasp predictions without importing the model code or needing a GPU.

```
┌──────────────────────┐         ZMQ (tcp)         ┌──────────────────────┐
│   Client (any lang)  │  ──── point cloud / mesh ──────▶  │  GraspGen Server     │
│   - Python / C++ / … │  ◀── grasps + scores ───  │  - GPU, model loaded  │
│   - No CUDA needed   │                           │  - Runs in Docker     │
└──────────────────────┘                           └──────────────────────┘
```

The server loads a gripper model (Franka Panda, Robotiq 2F-140, or Single Suction Cup 30mm) and listens on a ZMQ REP socket. Clients send point clouds (as numpy arrays serialized with msgpack) and receive back 6-DOF grasp poses and confidence scores.

## With Docker (recommended)

Terminal window 1 — **start the server**:

```bash
# Build the base Docker image (one-time):
bash docker/build.sh

# Start the server (default: Robotiq 2F-140 on port 5558):
MODELS_DIR=/path/to/GraspGenModels docker compose -f docker/compose.serve.yml up --build

# Or with a custom gripper and port:
MODELS_DIR=/path/to/GraspGenModels \
SERVER_ARGS="--gripper_config /models/checkpoints/graspgen_franka_panda.yml --port 5557" \
  docker compose -f docker/compose.serve.yml up --build
```

You can customize the loaded gripper by providing `SERVER_ARGS` (see `client-server/graspgen_server.py --help`). Available gripper configs in the checkpoints directory:
- `graspgen_robotiq_2f_140.yml` (default)
- `graspgen_franka_panda.yml`
- `graspgen_single_suction_cup_30mm.yml`

Terminal window 2 — **run the client** (lightweight uv environment, no CUDA needed):

```bash
# Create a client environment (one-time):
uv venv --python 3.10 client-server/.venv
source client-server/.venv/bin/activate
uv pip install pyzmq msgpack msgpack-numpy numpy trimesh
uv pip install -e . --no-deps

# Run the client with a mesh file:
python client-server/graspgen_client.py \
    --mesh_file /path/to/GraspGenModels/sample_data/meshes/box.obj \
    --mesh_scale 1.0 \
    --host localhost --port 5558

# Or with a point cloud file (.pcd / .ply / .xyz / .npy):
python client-server/graspgen_client.py \
    --pcd_file assets/objects/example_object.pcd \
    --host localhost --port 5558
```

## Without Docker

Terminal window 1 — **start the server**:

```bash
# Activate your GraspGen environment (must have CUDA + all GraspGen dependencies):
conda activate GraspGen   # or source .venv/bin/activate

# Install serving dependencies:
pip install pyzmq msgpack msgpack-numpy

# Start the server:
python client-server/graspgen_server.py \
    --gripper_config /path/to/GraspGenModels/checkpoints/graspgen_robotiq_2f_140.yml \
    --port 5558
```

Terminal window 2 — **run the client**:

```bash
# Create a client environment (one-time):
uv venv --python 3.10 client-server/.venv
source client-server/.venv/bin/activate
uv pip install pyzmq msgpack msgpack-numpy numpy trimesh
uv pip install -e . --no-deps

# Run the client with a mesh file:
python client-server/graspgen_client.py \
    --mesh_file /path/to/GraspGenModels/sample_data/meshes/box.obj \
    --mesh_scale 1.0 \
    --host localhost --port 5558

# Or with a point cloud file:
python client-server/graspgen_client.py \
    --pcd_file assets/objects/example_object.pcd \
    --host localhost --port 5558
```

## Python Client API

The client only requires `pyzmq`, `msgpack`, `msgpack-numpy`, and `numpy` — no PyTorch or CUDA.

```python
from grasp_gen.serving.zmq_client import GraspGenClient

client = GraspGenClient(host="localhost", port=5558)

# Get server info
print(client.server_metadata)
# {'gripper_name': 'robotiq_2f_140', 'model_name': 'diffusion-discriminator', ...}

# Run inference
grasps, confidences = client.infer(
    point_cloud,          # (N, 3) numpy float32 array
    num_grasps=200,       # diffusion samples
    topk_num_grasps=100,  # return top-k by confidence
)
# grasps:       (M, 4, 4) float32 — 6-DOF grasp poses
# confidences:  (M,)      float32 — grasp quality scores [0, 1]

client.close()
```

## Protocol Reference

The server uses **msgpack** serialization over a **ZMQ REP** socket.

| Request | Fields | Response |
|---------|--------|----------|
| `{"action": "health"}` | — | `{"status": "ok"}` |
| `{"action": "metadata"}` | — | `{"gripper_name": ..., "model_name": ..., ...}` |
| `{"action": "infer", "point_cloud": ndarray, ...}` | `planner`, `grasp_threshold`, `num_grasps`, `topk_num_grasps`, `min_grasps`, `max_tries`, `remove_outliers`, `scene_point_cloud` (optional), `moe_*` (optional, `planner="graspmoe"` only) | `{"grasps": ndarray, "confidences": ndarray, "num_grasps": int, "timing": {...}, "planner": str, "branch_tags": [...] (planner="graspmoe" only)}` |

This makes it straightforward to write clients in any language with ZMQ and msgpack bindings (C++, Rust, etc.).

### `graspmoe` planner — guaranteed top-down candidates

The default planner (`planner="diffusion"`, unchanged) is the raw diffusion sampler: it has no notion of "up" or gravity, it only sees the object point cloud centered on itself, so for a given object shape it may generate mostly (or only) lateral/side grasps — there's no bias toward the top-down approach that's often preferable for small objects sitting on a surface.

`planner="graspmoe"` unions those diffusion grasps with a second, **deterministic** branch: candidates swept over the object's own oriented bounding box (OBB), including a world-aligned top-down grasp by construction (`moe_obb_density="dense-topandside"`, the default, also sweeps all 4 side faces). Both branches are scored by the same discriminator, so ranking/top-k treats them uniformly. This is the same planner already used by `scripts/demo_scene_pc.py --planner graspmoe` (`grasp_gen/samplers/graspmoe.py`), now available over ZMQ.

```python
grasps, confidences = client.infer(
    object_point_cloud,
    planner="graspmoe",
    topk_num_grasps=50,          # applied to the union of both branches
    moe_obb_density="dense-topandside",  # top face + all 4 sides (default)
    moe_num_yaws=36,             # yaw samples per OBB face
    moe_z_offsets_cm=(-8, -6, -4, -2, 0),  # standoff sweep per face
)
```

Fully opt-in: omit `planner` (or pass `"diffusion"`) and behavior is unchanged. When `planner="graspmoe"`, the response gains `branch_tags` (a `"diff"`/`"obb"` string per returned grasp, same order/length as `grasps`/`confidences` — sliced consistently if `scene_point_cloud` collision filtering also ran) and `skipped_obb` (`true` if the OBB branch was skipped — e.g. `moe_skip_obb_rule="auto"`, the default, skips it when every OBB extent exceeds the gripper's jaw width, i.e. the object is too big to enclose). Requires a parallel-jaw gripper config with a `width` field (Franka Panda, Robotiq 2F-140); raises a clear `error` in the response for grippers without one (e.g. suction cups) — use `planner="diffusion"` for those.

### Collision filtering against a scene point cloud

Pass an optional `scene_point_cloud` (ndarray `(S, 3)`) alongside `point_cloud` in an `infer` request to filter out any returned grasp whose gripper mesh would collide with the surroundings (table, other objects, ...). The target object's own points should already be excluded from `scene_point_cloud` by the caller — this is the same collision check used by `scripts/demo_scene_pc.py --filter_collisions` (`filter_colliding_grasps_fast`), now available over ZMQ.

```python
grasps, confidences = client.infer(
    object_point_cloud,               # (N, 3) — the object only
    scene_point_cloud=scene_pc,       # (S, 3) — everything else, object excluded
    collision_threshold=0.02,         # meters
    max_scene_points=8192,            # server-side random-downsample cap
)
```

Opt-in and fully backward compatible: omit `scene_point_cloud` (or pass `None`) and the server behaves exactly as before, no collision filtering. The server pre-samples the loaded gripper's own collision mesh once at startup (from its `gripper_config`), so this adds no per-request mesh-sampling cost. When collision filtering ran, the response also includes `num_grasps_before_collision_filter` and `timing.collision_filter_ms`.
