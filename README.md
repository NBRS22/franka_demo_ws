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
| `scripts/` | `install_franka_ros2.sh`, `install_conda.sh`, `setup_envs.sh`, `download_models.sh`, `smoke_test.sh` | this file |
| `franka_ros2_ws/` | Franka's `franka_ros2` — **not versioned here**, created by `scripts/install_franka_ros2.sh` | [section 3](#3-franka-ros-2-external) |

## Setup at a glance

```bash
git clone -b new https://github.com/NBRS22/franka_demo_ws.git FP3 && cd FP3      # 2. clone (remote/branch as pushed)
export FP3_ROOT=$PWD
source /opt/ros/jazzy/setup.bash
scripts/install_franka_ros2.sh                                                   # 3. Franka's library (clone + build)
scripts/install_conda.sh && scripts/setup_envs.sh && scripts/download_models.sh   # 4. conda, envs, model weights
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
`rosdep install --from-paths franka_ros2_ws/src --ignore-src -y`, `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF` in `franka_ros2_ws`.

Nothing of this project is patched into `franka_ros2`: the FP3 MoveIt configuration (`franka_fp3_moveit_config`) and the FP3
robot launch configs (`robot_configs/fp3.config.yaml`) live in `franka_demo_ws/src/franka_fp3_moveit_config`.

## 4. Conda environments and model weights

The perception servers run in **their own conda environments** (never mix them with the ROS 2 system Python). Model
weights are not versioned. Three scripts do everything; each one is idempotent and prints what it does.

### 4.1 Install conda (once)

```bash
scripts/install_conda.sh          # installs Miniconda into ~/miniconda3 if conda is not already there (no sudo)
~/miniconda3/bin/conda init bash  # optional, then open a new terminal, so that `conda activate` works
conda config --set auto_activate_base false     # keep `base` OFF: its python3 would shadow the system one used by ROS 2
```
The launch files find conda by themselves (`$CONDA_EXE`, `~/miniconda3`, `~/anaconda3`, `~/miniforge3`, `/opt/conda`),
so conda does not have to be on the `PATH`. Already have Miniconda/Anaconda? Skip this step.

### 4.2 Create the environments

Requirements: an NVIDIA driver (`nvidia-smi` works); for GraspGen also the CUDA compiler and gcc 12:
`sudo apt install nvidia-cuda-toolkit gcc-12 g++-12`.

```bash
scripts/setup_envs.sh                 # ER, SAM3 and GraspGen   (or name some: scripts/setup_envs.sh ER SAM3)
```

| Env | Python | Main contents | Used by |
|---|---|---|---|
| `ER` | 3.12 | `opencv-python==4.9.0.80`, `numpy==1.26.4`, `pyzmq`, `msgpack` | `ER/gemini_er_simulator.py` (no ROS, no GPU) |
| `SAM3` | 3.12 | `torch==2.10.0` (CUDA 12.8 build), this repo's `SAM3/` installed editable, `pyzmq`, `msgpack` | SAM3 server |
| `GraspGen` | 3.10 | `torch==2.1.0` (CUDA 12.1 build), `torch-cluster`, `torch-scatter`, this repo's `GraspGen/` editable, the compiled `pointnet2_ops` extension, `pyzmq`, `msgpack`, `msgpack-numpy` | GraspGen server |

The environment names are what `franka_demo.launch.py` expects. Downloads are large (several GB of PyTorch/CUDA libraries)
and the `pointnet2_ops` compilation takes a few minutes. The editable installs point at the clone in which you ran the script:
run it again (after `conda env remove -n <name>`) if you move the repository.

### 4.3 Download the model weights

```bash
scripts/download_models.sh                    # GraspGen Franka checkpoints + SAM3 weights
scripts/download_models.sh graspgen           # only GraspGen (public, about 1 GB; add --all-graspgen for all grippers, about 8 GB)
scripts/download_models.sh sam3               # only SAM3 (needs the account steps below)
```

| Weights | Source | Size | Goes to | Access |
|---|---|---|---|---|
| GraspGen checkpoints | [`huggingface.co/adithyamurali/GraspGenModels`](https://huggingface.co/adithyamurali/GraspGenModels) | about 1 GB (Franka Panda only) / 8 GB (all) | `GraspGen/GraspGenModels/` | public |
| SAM 3 | [`huggingface.co/facebook/sam3`](https://huggingface.co/facebook/sam3) | a few GB | Hugging Face cache `~/.cache/huggingface` | **gated** |

SAM 3 is gated by Meta. Once: create a free Hugging Face account, open the model page above and **request access**
(usually granted quickly), create a *read* token at [`huggingface.co/settings/tokens`](https://huggingface.co/settings/tokens),
then `conda activate SAM3 && hf auth login` and paste the token. The script then downloads the weights (it stops with a clear message
if you are not logged in). Without the script, the weights are also fetched automatically the first time the SAM3 server starts.

### 4.4 Checklist: a complete, working repository

After sections 2–5 you should have (this is exactly what `scripts/smoke_test.sh --with-servers` needs):

| Item | Check |
|---|---|
| repository cloned, `FP3_ROOT` exported | `ls $FP3_ROOT` shows `franka_demo_ws calib_ws SAM3 GraspGen ER scripts` |
| `franka_ros2_ws/install` | `ls $FP3_ROOT/franka_ros2_ws/install/setup.bash` |
| `franka_demo_ws/install`, `calib_ws/install` | same with each workspace |
| conda envs `ER`, `SAM3`, `GraspGen` | `conda env list` |
| GraspGen checkpoints | `ls GraspGen/GraspGenModels/checkpoints/graspgen_franka_panda.yml` |
| SAM3 weights cached | `ls ~/.cache/huggingface/hub | grep sam3` |
| a hand-eye calibration (real robot only) | `ls ~/.ros2/easy_handeye2/calibrations/` |

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
