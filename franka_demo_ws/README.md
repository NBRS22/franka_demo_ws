# franka_demo_ws — vision-guided pick pipeline for the Franka FP3

ROS 2 (Jazzy) workspace that lets a Franka FP3 pick an object designated by a click: a VLM (or its
click-to-pick simulator) gives a pixel, **SAM3** segments the object, its point cloud is built from
the RealSense depth, **GraspGen** proposes grasps (filtered against the table/scene), and
**MoveIt Task Constructor** executes the best reachable one.

```
 Gemini ER (or ER/gemini_er_simulator.py, conda env ER)
        │  ZMQ  :5555 camera JPEG ▲ │ :5556 pick command
        ▼                          │ ▼
   gemini_er_bridge ──▶ robot_task_manager (pick_task_node)
                          │ 1 camera_buffer_node      RGB + aligned depth + intrinsics
                          │ 2 sam3_bridge      ── ZMQ :5557 ──▶ SAM3 server      (conda env SAM3, GPU)
                          │ 3 create_pointcloud_node  object cloud + scene cloud (no object)
                          │ 4 graspgen_bridge  ── ZMQ :5558 ──▶ GraspGen server  (conda env GraspGen, GPU)
                          ▼ 5 (only if execute_pick:=true)
                     /mtc_pick ─▶ fp3_moveit_server (move_group, MTC, franka_gripper) ─▶ FP3
```

The pipeline is **safe by default**: `execute_pick` is `false`, so it stops after grasp generation and
visualization (RViz) and the arm does not move.

Requires an eye-on-base **hand-eye calibration** (`fp3_link0 → camera_link`) made with
[`../calib_ws`](../calib_ws/README.md).

---

## Contents

1. [Packages](#1-packages) · 2. [Setup](#2-setup) · 3. [Launch](#3-launch) · 4. [Trigger a pick](#4-trigger-a-pick) ·
5. [Configuration](#5-configuration) · 6. [Ports and topics](#6-ports-and-topics) · 7. [Troubleshooting](#7-troubleshooting)

---

## 1. Packages

| Package | What it is for |
|---|---|
| `franka_demo_bringup` | **Root launch** `franka_demo.launch.py`: starts the SAM3 and GraspGen servers (`conda activate` + `exec`), waits until they answer (`wait_for_zmq_health.py`), then RealSense (with USB-reset retry), `handeye_tf_publisher`, the MoveIt stack and the whole pipeline. Shuts everything down if a server dies. |
| `robot_task_manager` | Orchestrator: `pick_task_node` (sequences a pick, service `/execute_pick_task`), `camera_buffer_node` (RGB + depth + intrinsics from one instant), `create_pointcloud_node` (masked back-projection, object cloud and scene cloud), `pointcloud_publisher_node`. Its launch also starts the three bridges. |
| `gemini_er_bridge` | ZMQ bridge to Gemini ER: `camera_bridge_node` (publishes the image, port 5555) and `command_bridge_node` (receives pick commands, port 5556). |
| `sam3_bridge` | Bridge to the SAM3 segmentation server + RViz overlay of the mask (`visualize_segmentation_node`). |
| `graspgen_bridge` | Bridge to the GraspGen server (sends object + scene clouds so colliding grasps are filtered) + RViz grasp arrows (`visualize_grasps_node`). |
| `fp3_moveit_server` | Sole owner of `move_group`. `pick_place_node` (MTC approach/lift, gripper through the real `franka_gripper` actions), `command_router_node` (public action `mtc_pick`), `scene_setup_node` (table + wall). Orders grasps from most vertical to most lateral. |
| `franka_fp3_moveit_config` | MoveIt configuration of the FP3 and Franka robot launch configs (`robot_configs/fp3.config.yaml`, `fp3_fake.config.yaml`). |
| `franka_demo_interfaces` | Shared services and actions (`ExecutePickTask`, `GetFrames`, `SegmentObject`, `CreatePointcloud`, `GenerateGraspPose`, `VisualizeGrasps`, `VisualizeSegmentation`, action `MtcPick`). |
| `handeye_tf_publisher` | Publishes `fp3_link0 → camera_link` from a saved `.calib`. (Also present in `calib_ws`.) |

Each package has a `CLAUDE.md` with design notes and history. `franka_reflex.md` documents the libfranka
network reflex; `camera.md` the RealSense USB troubleshooting.

---

## 2. Setup

Everything below assumes the repository layout of the root [`README`](../README.md):

```bash
export FP3_ROOT=~/Documents/FP3        # wherever you cloned the repository
```

### 2.1 Hardware and system

- Ubuntu 24.04, ROS 2 **Jazzy**, `colcon`, `rosdep`, `vcstool`
- FP3 reachable at `192.168.1.1` (direct link), FCI enabled on the Desk
- RealSense **D455** (USB 3, udev rules installed) — see `camera.md`
- NVIDIA GPU with CUDA (SAM3 and GraspGen), `conda`/`miniconda`
- A saved hand-eye calibration in `~/.ros2/easy_handeye2/calibrations/` (from `calib_ws`)

### 2.2 Franka ROS 2 (not in this repository)

Clone and build `franka_ros2_ws` first: `scripts/install_franka_ros2.sh` at the repository root ([root README](../README.md#3-franka-ros-2-external)).

### 2.3 System packages

```bash
sudo apt install ros-jazzy-moveit ros-jazzy-moveit-task-constructor-core \
  ros-jazzy-moveit-task-constructor-capabilities ros-jazzy-realsense2-camera \
  ros-jazzy-cv-bridge ros-jazzy-tf2-geometry-msgs \
  python3-zmq python3-msgpack python3-scipy python3-opencv python3-numpy
pip install --user --break-system-packages msgpack-numpy      # ZMQ numpy serialization (system Python)
```
The general way to get every declared ROS dependency is
`rosdep install --from-paths src --ignore-src -r -y` from `franka_demo_ws` (after `franka_ros2_ws` is built and sourced).

### 2.4 SAM3, GraspGen, ER

They are part of this repository, next to the workspaces, and run in **their own conda environments** — never mix
them with the ROS 2 system Python. Each directory has a `README_FP3.md` with the exact setup, weights and test commands.

| Project | Conda env | Setup |
|---|---|---|
| `SAM3/` | `SAM3` | [`SAM3/README_FP3.md`](../SAM3/README_FP3.md): env (Python 3.12, torch 2.10 cu128), `pip install -e ".[inference]"`, gated weights `facebook/sam3` via `hf auth login` |
| `GraspGen/` | `GraspGen` | [`GraspGen/README_FP3.md`](../GraspGen/README_FP3.md): env (Python 3.10, torch 2.1 cu121), `./install_pointnet.sh`, checkpoints (about 8 GB) cloned into `GraspGen/GraspGenModels/` |
| `ER/` | `ER` | Simulator only, no ROS: `opencv-python==4.9.0.80`, `numpy==1.26.4`, `pyzmq`. Do not source ROS in this env. |

`conda` does not need to be on the `PATH`: the launch file finds it (`$CONDA_EXE`, `~/miniconda3`, `~/anaconda3`, `~/miniforge3`, `/opt/conda`). Run `ros2 launch` from a shell where the conda **`base` environment is not activated** — its `python3` would shadow the system one used by the ROS nodes.
The launch file finds `SAM3/` and `GraspGen/` by walking up from its own location, or from `$FP3_ROOT` if set.

### 2.5 Build

```bash
source /opt/ros/jazzy/setup.bash
source $FP3_ROOT/franka_ros2_ws/install/setup.bash
cd $FP3_ROOT/franka_demo_ws
colcon build --symlink-install
source install/setup.bash
```
Each new terminal: the same three `source` lines.

---

## 3. Launch

**Everything at once** (real robot, no motion — grasps are only generated and shown):

```bash
source /opt/ros/jazzy/setup.bash
source $FP3_ROOT/franka_ros2_ws/install/setup.bash
source $FP3_ROOT/franka_demo_ws/install/setup.bash
ros2 launch franka_demo_bringup franka_demo.launch.py use_rviz:=true
```
Startup takes a few minutes: the two model servers must load (health check timeout 180 s each) before the camera
and the pipeline start. Ctrl-C stops the launch; if a server crashes the whole launch shuts down.

**With the arm moving** — only after checking the grasps in RViz, workspace clear, E-stop in hand:

```bash
ros2 launch franka_demo_bringup franka_demo.launch.py use_rviz:=true execute_pick:=true
```

| Argument | Default | Meaning |
|---|---|---|
| `robot_ip` | `192.168.1.1` | FP3 controller IP |
| `use_fake_hardware` | `false` | Simulated arm instead of the real robot |
| `use_rviz` | `false` | RViz with the MoveIt configuration |
| `calibration_name` | `fp3_link0_d455_camera_color_optical_frame_001` | `.calib` published as `fp3_link0 → camera_link` |
| `execute_pick` | `false` | Send the best grasp to `mtc_pick` (**arm moves**) |

**Gemini ER simulator** (separate terminal, env `ER`, not ROS):

```bash
conda activate ER
cd $FP3_ROOT/ER && python gemini_er_simulator.py --label cube      # host 127.0.0.1, ports 5555/5556
```

**Partial launches** (debugging):

```bash
ros2 launch fp3_moveit_server bringup.launch.py use_fake_hardware:=false      # arm stack only (move_group, MTC)
ros2 launch robot_task_manager robot_task_manager.launch.py                    # pipeline + bridges, no camera, no servers
ros2 launch sam3_bridge sam3_bridge.launch.py
ros2 launch graspgen_bridge graspgen_bridge.launch.py
```
`robot_task_manager.launch.py` does **not** start the SAM3/GraspGen servers nor the camera: start them yourself
(`cd $FP3_ROOT/SAM3 && conda run -n SAM3 python -m sam3_server`, `cd $FP3_ROOT/GraspGen && conda run -n GraspGen python client-server/graspgen_server.py`).

---

## 4. Trigger a pick

In the simulator window, **click on the object**. The pixel goes to `command_bridge`, which calls
`/execute_pick_task`. Without the simulator:

```bash
ros2 service call /execute_pick_task franka_demo_interfaces/srv/ExecutePickTask \
  "{object_label: 'red mug', point_x: 640, point_y: 360}"
```
Pixels are in the camera resolution (1280×720). Watch RViz: `/pick/segmentation_visualization`,
`/pick/pointcloud`, `/pick/scene_pointcloud`, `/pick/grasp_markers`, and with `execute_pick:=true`
`/pick/filtered_grasp_markers`, `/pick/executed_grasp_pose`, `/pick/executed_grasp_marker`.

---

## 5. Configuration

| What | Where |
|---|---|
| Table and wall in the planning scene (heights measured by contact) | `fp3_moveit_server/config/scene.yaml` |
| Grasp filtering, speed scaling, object size, gripper widths | `fp3_moveit_server/config/pick_place.yaml` |
| Controller override (trajectory end velocity) | `fp3_moveit_server/config/controller_overrides.yaml` |
| GraspGen planner, collision threshold, scene points | ROS parameters of `graspgen_bridge_node` |
| Camera resolution | `REALSENSE_*_PROFILE` in `franka_demo.launch.py` (nothing else hardcodes it) |
| SAM3 / GraspGen conda env names and ports | constants at the top of `franka_demo.launch.py` |

---

## 6. Ports and topics

| Port | Link | Direction |
|---|---|---|
| 5555 | `camera_bridge` → Gemini ER | PUB/SUB (JPEG) |
| 5556 | Gemini ER → `command_bridge` | REQ/REP (pick command) |
| 5557 | `sam3_bridge` → SAM3 server | REQ/REP |
| 5558 | `graspgen_bridge` → GraspGen server | REQ/REP |

ZMQ sockets have no authentication: use a closed lab network only.

---

## 7. Troubleshooting

- **The launch shuts down right after start**: a server did not become healthy in 180 s (GPU busy, model not downloaded,
  `conda` not on `PATH`). Run the failing server by hand (section 3) to see its error.
- **`Cannot locate the FP3 repo root`**: `SAM3/` and `GraspGen/` are not next to the workspaces; set `FP3_ROOT`.
- **`Controller is not running` / `communication_constraints_violation`**: network real-time issue on the robot link,
  see `franka_reflex.md`. Relaunch the arm stack afterwards.
- **`realsense2_camera_node` survives Ctrl-C** and blocks the next launch: `pkill -9 -f realsense2_camera_node`; more in `camera.md`.
- **Object cloud offset from the real object**: check the hand-eye calibration first (`calib_ws`), then the table height.
- **Stale `ros2` nodes/actions after a crash**: `ros2 daemon stop && ros2 daemon start`.
- **A grasp filter rejects everything**: adjust `filter.*` in `pick_place.yaml`; the scene cloud mask is dilated in `create_pointcloud_node`.

## Known limitations

- `command_bridge` blocks during a pick: a `stop` message cannot interrupt it.
- Real-hardware pick with the final MTC/table settings has been validated piecewise; the complete chain
  (click → SAM3 → GraspGen → `mtc_pick`) was last run end to end before the recent table/calibration changes.
- `handeye_tf_publisher` exists in two slightly different versions (here and in `calib_ws`).
- Package licenses are still `TODO` in four `package.xml` files.
