# FP3 — vision-guided pick-and-place for the Franka FP3

A complete pipeline for a **Franka FP3** arm (ROS 2 Jazzy): a click on an object (from a VLM, or its simulator) is
turned into a segmentation (**SAM3**), a point cloud, a set of collision-free grasps (**GraspGen**) and finally a pick
executed with **MoveIt Task Constructor**. The repository also contains the **hand-eye calibration** tooling that the
pipeline depends on.

```
   Gemini ER / ER simulator ──click──▶ franka_demo_ws ──ZMQ──▶ SAM3 server        (GPU, conda env SAM3)
        (conda env ER)                  ROS 2 pipeline  ──ZMQ──▶ GraspGen server    (GPU, conda env GraspGen)
                                             │
                RealSense D455 ─────────────▶│── MoveIt Task Constructor ──▶ Franka FP3 (+ gripper)
                                             ▲
                       calib_ws ── hand-eye calibration (fp3_link0 → camera) ──┘
```

## Repository layout

Every folder has its own README — start there for the details of that part.

| Folder | What it is | Documentation |
|---|---|---|
| [`franka_demo_ws/`](franka_demo_ws/) | ROS 2 workspace of the pick pipeline: bridges, task manager, MoveIt/MTC server, launch files | [`franka_demo_ws/README.md`](franka_demo_ws/README.md) |
| [`calib_ws/`](calib_ws/) | ROS 2 workspace for the eye-on-base hand-eye calibration and its verification | [`calib_ws/README.md`](calib_ws/README.md) |
| [`SAM3/`](SAM3/) | Segmentation server (Meta's SAM 3 + our ZMQ server) | [`SAM3/README_FP3.md`](SAM3/README_FP3.md) |
| [`GraspGen/`](GraspGen/) | Grasp-generation server (NVIDIA's GraspGen + our collision-filtering ZMQ server) | [`GraspGen/README_FP3.md`](GraspGen/README_FP3.md) |
| [`ER/`](ER/) | Click-to-pick simulator standing in for Gemini ER | [`ER/README.md`](ER/README.md) |
| `scripts/` | `install_franka_ros2.sh`, `smoke_test.sh` | this file |
| `franka_ros2_ws/` | Franka's `franka_ros2` — **not versioned here**, created by `scripts/install_franka_ros2.sh` | [section 3](#3-franka-ros-2-external) |

## Setup at a glance

```bash
git clone -b new https://github.com/NBRS22/franka_demo_ws.git FP3 && cd FP3      # 2. clone (remote/branch as pushed)
export FP3_ROOT=$PWD
source /opt/ros/jazzy/setup.bash
scripts/install_franka_ros2.sh                                                   # 3. Franka's library (clone + build)
# 4. conda envs + model weights: SAM3/README_FP3.md, GraspGen/README_FP3.md, ER/README.md
source franka_ros2_ws/install/setup.bash
(cd franka_demo_ws && colcon build --symlink-install)                            # 5. build the pipeline
source franka_demo_ws/install/setup.bash
(cd calib_ws && scripts/install_dependencies.sh && colcon build --symlink-install)   # 5. build the calibration tools
scripts/smoke_test.sh                                                            # 8. verify, simulated hardware only
```
Details of each step below.

## 1. Prerequisites

- Ubuntu 24.04, **ROS 2 Jazzy** (`/opt/ros/jazzy`), `colcon`, `rosdep`, `vcstool`, `git`, `git-lfs`
- For the real robot: Franka FP3 on a direct link (default `192.168.1.1`), FCI enabled on the Desk
- For perception: an NVIDIA GPU with CUDA (SAM3 and GraspGen), `conda`/`miniconda`, an Intel RealSense D455
  (USB 3, udev rules — see [`franka_demo_ws/camera.md`](franka_demo_ws/camera.md))
- MoveIt and MoveIt Task Constructor: `sudo apt install ros-jazzy-moveit ros-jazzy-moveit-task-constructor-core ros-jazzy-moveit-task-constructor-capabilities`

Every command in the documentation assumes:

```bash
export FP3_ROOT=<path where you cloned this repository>
```

## 2. Clone

```bash
git clone -b new https://github.com/NBRS22/franka_demo_ws.git FP3     # remote / branch as pushed
cd FP3 && export FP3_ROOT=$PWD
```
`SAM3/` and `GraspGen/` are **included** (no submodules, nothing to patch). Not versioned: the model weights and Franka's library.

## 3. Franka ROS 2 (external)

`franka_ros2` is Franka's library and is deliberately not versioned here. One script clones it at the tested version,
imports its dependencies (`libfranka`, `franka_description`, …), runs `rosdep` and builds it:

```bash
source /opt/ros/jazzy/setup.bash
scripts/install_franka_ros2.sh              # add --no-build to only fetch the sources
```
What it does, if you prefer manually: `git clone -b jazzy https://github.com/frankarobotics/franka_ros2.git franka_ros2_ws/src`,
`git -C franka_ros2_ws/src checkout 73a1501d76efa2bc4bf09cb2af9c2b72c2c642da`, `vcs import franka_ros2_ws/src < franka_ros2_ws/src/dependency.repos`,
`rosdep install --from-paths franka_ros2_ws/src --ignore-src -y`, `colcon build --symlink-install` in `franka_ros2_ws`.

Nothing of this project is patched into `franka_ros2`: the FP3 MoveIt configuration (`franka_fp3_moveit_config`) and the FP3
robot launch configs (`robot_configs/fp3.config.yaml`) live in `franka_demo_ws/src/franka_fp3_moveit_config`.

## 4. Conda environments and model weights

Three separate conda environments (never mix them with the ROS 2 system Python). Weights are not versioned.

| Env | Folder | Guide | Weights |
|---|---|---|---|
| `SAM3` | `SAM3/` | [`SAM3/README_FP3.md`](SAM3/README_FP3.md) | gated Hugging Face `facebook/sam3` (`hf auth login`) |
| `GraspGen` | `GraspGen/` | [`GraspGen/README_FP3.md`](GraspGen/README_FP3.md) | about 8 GB, `git clone https://huggingface.co/adithyamurali/GraspGenModels` into `GraspGen/` |
| `ER` | `ER/` | [`ER/README.md`](ER/README.md) | none |

## 5. Build (order matters)

```bash
source /opt/ros/jazzy/setup.bash
cd $FP3_ROOT/franka_ros2_ws && colcon build --symlink-install && source install/setup.bash      # done by install_franka_ros2.sh
cd $FP3_ROOT/franka_demo_ws && colcon build --symlink-install && source install/setup.bash
cd $FP3_ROOT/calib_ws && scripts/install_dependencies.sh && colcon build --symlink-install && source install/setup.bash
```
Each new terminal: `source /opt/ros/jazzy/setup.bash` and the three `install/setup.bash` files in that order
(`franka_ros2_ws`, `franka_demo_ws`, `calib_ws`). `calib_ws/scripts/install_dependencies.sh` clones the two pinned
third-party packages the calibration needs (`apriltag_ros`, `easy_handeye2`) and installs their system dependencies.

## 6. Calibrate the camera (once per camera position)

The pipeline needs the transform `fp3_link0 → camera` produced by an eye-on-base hand-eye calibration, saved in
`~/.ros2/easy_handeye2/calibrations/`. **The arm never moves by itself**: poses are set by hand.
Procedure, verification and troubleshooting: [`calib_ws/README.md`](calib_ws/README.md).

## 7. Run the pipeline

```bash
source /opt/ros/jazzy/setup.bash
source $FP3_ROOT/franka_ros2_ws/install/setup.bash
source $FP3_ROOT/franka_demo_ws/install/setup.bash
ros2 launch franka_demo_bringup franka_demo.launch.py use_rviz:=true          # no arm motion (execute_pick:=false)
```
In another terminal, the click-to-pick simulator: `conda activate ER && cd $FP3_ROOT/ER && python gemini_er_simulator.py`.
Check the grasps in RViz, then — workspace clear, E-stop in hand — restart with `execute_pick:=true` to let the arm pick.
All launch arguments, partial launches and troubleshooting: [`franka_demo_ws/README.md`](franka_demo_ws/README.md).

## 8. Verify an installation without hardware

```bash
scripts/smoke_test.sh                     # every ROS piece with SIMULATED hardware, isolated ROS domain
scripts/smoke_test.sh --with-servers      # also the real SAM3/GraspGen servers and the whole pipeline (GPU, weights needed)
scripts/smoke_test.sh moveit sam3_bridge  # only some pieces
```
It starts each launch file with `use_fake_hardware:=true`, checks that the expected nodes come up and none crashes, stops
it and prints a summary (logs in `/tmp/fp3_smoke`). It never talks to a real robot. A RealSense camera is optional
(without one the camera process is reported as a warning).

## Safety

- `execute_pick` is `false` by default everywhere: nothing moves the arm until you ask for it.
- Automatic motion during calibration was removed on purpose (two hardware incidents). `fp3_apriltag_demo` (in `calib_ws`) and `execute_pick:=true` are the only paths where the arm moves by itself.
- Keep the E-stop in hand for any first run on the real robot. On libfranka `communication_constraints_violation` see [`franka_demo_ws/franka_reflex.md`](franka_demo_ws/franka_reflex.md).

## Licenses and third-party code

`SAM3/` (SAM License) and `GraspGen/` (NVIDIA License) are copies of upstream projects with our additions; their `LICENSE`
files apply to them and must stay with any redistribution. `franka_ros2` and the `calib_ws` external clones are not
included. This project's own code has no license declared yet (`TODO` in several `package.xml`).
