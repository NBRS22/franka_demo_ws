# GraspGen for the FP3 pick pipeline

This directory is a copy of [`NVlabs/GraspGen`](https://github.com/NVlabs/GraspGen) (commit `2dd8852`; the
upstream `README.md` is kept) **with the changes this project needed**, so that cloning the FP3 repository gives a
working grasp-generation server. It is used through its ZMQ server, never imported by ROS.
The NVIDIA License (`LICENSE`) applies and must stay with any redistribution.

## What was changed compared to upstream

| File | Change |
|---|---|
| `grasp_gen/serving/zmq_server.py`, `zmq_client.py` | **Scene-aware collision filtering.** `infer` accepts an optional `scene_point_cloud` (+ `collision_threshold`, `max_scene_points`): the grasps already generated are filtered with `filter_colliding_grasps_fast` against the real gripper geometry (pre-sampled once at start-up). Also a `planner` field (`"diffusion"` or `"graspmoe"`), and the response reports `num_grasps_before_collision_filter` and `timing.collision_filter_ms` when a filter ran. Without `scene_point_cloud` the behaviour is unchanged. |
| `client-server/graspgen_server.py` | Default gripper config = Franka Panda (`GraspGenModels/checkpoints/graspgen_franka_panda.yml`), default port **5558** |
| `client-server/graspgen_client.py`, `docker/*`, `README.md`, `client-server/README.md` | Port 5556 → 5558, documentation of the new fields |
| `install_pointnet.sh` | Compiler pinned to `gcc-12` / `g++-12` (the CUDA 12.1 toolchain does not accept newer ones) |
| `mcp/`, `tests/test_serving.py` | Kept consistent with the above |

## Role in the pipeline

```
franka_demo_ws / graspgen_bridge_node ──ZMQ REQ──▶ graspgen_server.py (port 5558, GPU)
   object cloud (+ scene cloud without the object)        └─▶ grasps (4x4 poses) + scores, collision-free
```
`ros2 launch franka_demo_bringup franka_demo.launch.py` **starts this server for you** from this directory
(so the models must be at `GraspGen/GraspGenModels/`). Run it by hand only to test it.

## Setup

Requirements: NVIDIA GPU + CUDA toolkit matching PyTorch (12.1), `gcc-12`/`g++-12` (`sudo apt install gcc-12 g++-12`), `conda`.

```bash
# 1. Environment. The name GraspGen is what franka_demo.launch.py expects.
conda create -n GraspGen python=3.10 -y
conda activate GraspGen

# 2. PyTorch 2.1.0 + CUDA 12.1 (as in the upstream README)
pip install torch==2.1.0 torchvision==0.16.0 torch-cluster torch-scatter \
  -f https://data.pyg.org/whl/torch-2.1.0+cu121.html

# 3. This directory (editable) + the PointNet CUDA extension + the server's ZMQ dependencies
cd $FP3_ROOT/GraspGen
pip install -e .
./install_pointnet.sh
pip install pyzmq msgpack msgpack-numpy

# 4. Model checkpoints (about 8 GB, NOT in this repository). Only graspgen_franka_panda_* is used by the FP3.
sudo apt install git-lfs && git lfs install
git clone https://huggingface.co/adithyamurali/GraspGenModels        # creates GraspGen/GraspGenModels/
```
Check the install: `python tests/test_inference_installation.py`.

## Run the server

```bash
conda activate GraspGen
cd $FP3_ROOT/GraspGen                        # the default config path is relative to this directory
python client-server/graspgen_server.py      # Franka Panda, 0.0.0.0:5558
python client-server/graspgen_server.py --gripper_config GraspGenModels/checkpoints/graspgen_robotiq_2f_140.yml --port 5558
```
Ready when the log shows `Model loaded and ready for inference` and `GraspGen ZMQ server listening on tcp://0.0.0.0:5558`.

## Test it

```bash
# Health check (what franka_demo_ws polls before starting the pipeline)
python - <<'EOF'
import zmq, msgpack
s = zmq.Context().socket(zmq.REQ); s.connect("tcp://127.0.0.1:5558")
s.send(msgpack.packb({"action": "health"})); print(msgpack.unpackb(s.recv()))    # {'status': 'ok'}
EOF

# Grasps for a mesh (see client-server/README.md for all options)
python client-server/graspgen_client.py --mesh_file GraspGenModels/sample_data/meshes/box.obj --host localhost --port 5558
```

## Protocol (`infer`)

| Request field | Meaning |
|---|---|
| `action` | `"infer"` (also `"health"`, `"metadata"`) |
| `point_cloud` | `ndarray (N,3)` object cloud, in the camera frame |
| `planner` | `"diffusion"` (default) or `"graspmoe"` (the FP3 pipeline uses `graspmoe`) |
| `num_grasps`, `topk_num_grasps` | how many grasps to sample / keep |
| `scene_point_cloud` | *optional* `ndarray (M,3)`: everything that is not the object (table, floor…) → colliding grasps are dropped |
| `collision_threshold`, `max_scene_points` | filter tolerance (m, default 0.02; the ROS bridge sends 0.01) and scene sub-sampling (default 8192) |

Serialization is msgpack with `msgpack_numpy` (arrays travel natively). More in `client-server/README.md`.

## Troubleshooting

- **`FileNotFoundError … graspgen_franka_panda.yml`**: the server was started from another directory, or the checkpoints are missing (step 4).
- **`install_pointnet.sh` fails**: check `nvcc --version` (CUDA 12.1), `gcc-12`/`g++-12` installed, and `TORCH_CUDA_ARCH_LIST` for your GPU (the upstream README shows an example).
- **Every grasp is rejected by the collision filter** (`0/N`): the scene cloud probably contains the object itself or the threshold is too strict; see `franka_demo_ws/src/graspgen_bridge/CLAUDE.md`.
- **Port 5558 already in use**: a previous server is still running (`pkill -f graspgen_server`).
