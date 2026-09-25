# FP3

Pick-and-place pipeline for a Franka FP3 (ROS 2 Jazzy): perception (SAM3, GraspGen), a VLM
command source (Gemini ER), MoveIt Task Constructor execution, and hand-eye calibration tooling.

```
FP3/
├── franka_demo_ws/   pick pipeline (bridges, task manager, fp3_moveit_server, franka_fp3_moveit_config)
├── calib_ws/         eye-on-base hand-eye calibration and verification tools
├── ER/               Gemini ER simulator / client
├── GraspGen/         submodule (NVlabs/GraspGen), local changes in patches/GraspGen
├── SAM3/             submodule (facebookresearch/sam3), local changes in patches/SAM3
├── patches/          our changes to the two submodules (applied by scripts/apply_patches.sh)
├── scripts/          apply_patches.sh
└── franka_ros2_ws/   NOT in this repo -- clone it yourself (see below)
```

Each workspace has its own README with setup and launch instructions:
[`franka_demo_ws`](franka_demo_ws/README.md) (the pick pipeline) and [`calib_ws`](calib_ws/README.md) (hand-eye calibration).

## Clone

```bash
git clone --recurse-submodules -b new https://github.com/NBRS22/franka_demo_ws.git FP3   # branch name/remote as pushed
cd FP3
export FP3_ROOT=$PWD
scripts/apply_patches.sh          # applies patches/ to GraspGen and SAM3 (idempotent)
```

Model weights are not versioned: GraspGen checkpoints (`git clone https://huggingface.co/adithyamurali/GraspGenModels`
inside `GraspGen/`) and the gated SAM3 weights (`hf auth login`, Hugging Face `facebook/sam3`). Conda environments
`SAM3`, `GraspGen` and `ER` are described in `franka_demo_ws/README.md`.

All commands in the docs assume:

```bash
export FP3_ROOT=~/Documents/FP3     # wherever you cloned this repo
```

## Franka ROS 2 (not included)

`franka_ros2` is Franka's own library and is deliberately not versioned here. Clone it next to
the other workspaces, at the version this project was developed against:

```bash
cd $FP3_ROOT
mkdir -p franka_ros2_ws && cd franka_ros2_ws
git clone -b jazzy https://github.com/frankarobotics/franka_ros2.git src
git -C src checkout 73a1501d76efa2bc4bf09cb2af9c2b72c2c642da   # v3.2.0-344-g73a1501
vcs import src < src/dependency.repos                            # libfranka 0.20.5, franka_description 2.8.1, ...
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

Nothing of this project is patched into `franka_ros2` anymore: the FP3 MoveIt configuration
(`franka_fp3_moveit_config`) lives in `franka_demo_ws/src`, and the FP3 robot launch
configurations are shipped by that package (`robot_configs/fp3.config.yaml`,
`fp3_fake.config.yaml`). Pass them to Franka's launch files by absolute path:

```bash
ros2 launch franka_bringup example.launch.py \
  robot_config_file:=$(ros2 pkg prefix franka_fp3_moveit_config)/share/franka_fp3_moveit_config/robot_configs/fp3.config.yaml \
  controller_names:=gravity_compensation_example_controller
```

## Build order

```bash
source /opt/ros/jazzy/setup.bash
cd $FP3_ROOT/franka_ros2_ws && colcon build --symlink-install && source install/setup.bash
cd $FP3_ROOT/franka_demo_ws && colcon build --symlink-install && source install/setup.bash
cd $FP3_ROOT/calib_ws       && colcon build --symlink-install && source install/setup.bash
```
