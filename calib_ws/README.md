# calib_ws — hand-eye calibration for the Franka FP3

ROS 2 (Jazzy) workspace that calibrates a fixed RealSense **D455** against the FP3 robot base
(*eye-on-base*): an AprilTag is rigidly mounted on the gripper, the arm is moved to a series of poses,
and [`easy_handeye2`](https://github.com/marcoesposito1988/easy_handeye2) solves `AX = XB` for the
transform **`fp3_link0 → camera`**. The rest of the workspace publishes that result as TF, checks it,
and offers an independent cross-check with a second camera (D405 mounted on the wrist).

```
                     fp3_link0 ──(easy_handeye2 result)──▶ camera_link ──(realsense TF)──▶ camera_color_optical_frame
                        │                                                                        ▲
     fp3_hand  (tag on the gripper)  ───────────── tag seen by AprilTag detection ────────────────┘
```

> **Safety rule of this workspace: the arm never moves by itself during a calibration.** Poses are
> set by hand (free-drive). Automatic pose tours were removed after two hardware incidents.

---

## Contents

| Section | |
|---|---|
| [1. Workspace layout](#1-workspace-layout) | what each package is for |
| [2. Setup](#2-setup) | prerequisites, external dependencies, build |
| [3. Prepare the tag](#3-prepare-the-tag) | printing, size, detector config |
| [4. Run a calibration](#4-run-a-calibration-eye-on-base-d455) | the main procedure |
| [5. Publish and verify](#5-publish-and-verify-a-calibration) | TF, quantitative check |
| [6. Cross-check with the D405](#6-optional-cross-check-with-a-wrist-d405) | independent validation |
| [7. Package reference](#7-package-reference) | launch arguments, nodes, topics |
| [8. Files and locations](#8-files-and-locations) | where results live |
| [9. Troubleshooting](#9-troubleshooting) | known problems |

---

## 1. Workspace layout

| Package | Kind | What it is for |
|---|---|---|
| **`calib_bringup`** | launch | One command that starts the camera, AprilTag detection, the `easy_handeye2` calibration UI, `calib_sample_guard`, and (optionally) the MoveIt arm stack. Also contains `evaluate_calibration.launch.py` to measure the error of a saved calibration. |
| **`calib_sample_guard`** | node | Live guardrail while sampling: prints reprojection error and tilt of the calibration tag and tells you whether the current view is safe to sample (`OK`) or not. |
| **`handeye_tf_publisher`** | node + launch | Reads a saved `.calib`, composes it with the RealSense internal TF and publishes `fp3_link0 → camera_link` as a static transform. Ships the tag detector configs (`tags/`) and `tools/watch_calibration_convergence.py`. |
| **`calib_eye_in_hand`** | launch | Calibrates a **D405 mounted on the wrist** against a fixed tag (mirror image of the D455 session). |
| **`calib_bridge`** | node + launch | Derives the D455 calibration from the D405 one: both cameras look at the same fixed tag, the D455 extrinsics follow from pure transform composition (no solver). Used to validate the direct D455 calibration independently. |
| **`fp3_apriltag_demo`** | node + launch | Detects a tag, estimates its 3D pose (`solvePnP`), transforms it into `fp3_link0` with the published calibration and **moves `fp3_hand_tcp` onto it** (gripper open by default, no grasp). A physical sanity check of a calibration. The arm moves by itself here — keep the E-stop within reach. |
| `apriltag_ros` | external | AprilTag detector (clone, see [setup](#22-external-dependencies)) |
| `easy_handeye2`, `easy_handeye2_msgs` | external | Calibration solver + `rqt` sampling UI (clone, see [setup](#22-external-dependencies)) |

Not part of this workspace but required at run time:

| Where | What |
|---|---|
| `$FP3_ROOT/franka_ros2_ws` | Franka's `franka_ros2` (drivers, `franka_bringup`, `franka_description`) — **not versioned in this repo**, see the [root README](../README.md) |
| `$FP3_ROOT/franka_demo_ws` | `fp3_moveit_server` + `franka_fp3_moveit_config` (MoveIt stack started by `calib_bringup` by default) |

---

## 2. Setup

### 2.1 Prerequisites

- Ubuntu 24.04, **ROS 2 Jazzy** (`/opt/ros/jazzy`), `colcon`, `rosdep`, `vcstool`
- Franka FP3 reachable on the direct link (default `192.168.1.1`), FCI enabled from the Desk
- Intel RealSense D455 (and D405 for the optional cross-check) plugged in USB 3
- An AprilTag (`tag36h11`, id 0) on a rigid support, see [section 3](#3-prepare-the-tag)
- The two sibling workspaces **built** (the order matters):

```bash
export FP3_ROOT=~/Documents/FP3          # wherever the repo is cloned
source /opt/ros/jazzy/setup.bash

cd $FP3_ROOT/franka_ros2_ws && colcon build --symlink-install && source install/setup.bash   # clone it first: see root README
cd $FP3_ROOT/franka_demo_ws && colcon build --symlink-install && source install/setup.bash
```

### 2.2 External dependencies

Two third-party repositories are **not versioned here**; they are cloned into `src/external/`
(git-ignored) at the exact commits this workspace was developed against (`calib.repos`):

| Repository | Commit | Provides |
|---|---|---|
| [`christianrauch/apriltag_ros`](https://github.com/christianrauch/apriltag_ros) | `beffb4c` (3.4.0-2) | `apriltag_ros` (detector node) |
| [`marcoesposito1988/easy_handeye2`](https://github.com/marcoesposito1988/easy_handeye2) | `0ad1ddd` | `easy_handeye2`, `easy_handeye2_msgs` |

**One command does everything** (clones the two repositories, installs the rosdep/apt packages):

```bash
cd $FP3_ROOT/calib_ws
source /opt/ros/jazzy/setup.bash
scripts/install_dependencies.sh
```

What the script does, if you prefer to do it by hand:

```bash
# 1. external sources, pinned
sudo apt install python3-vcstool python3-rosdep
vcs import src < calib.repos

# 2. ROS / system packages declared by every package.xml
rosdep update
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys "fp3_moveit_server franka_demo_interfaces python-transforms3d-pip"

# 3. easy_handeye2 needs transforms3d; use apt (the rosdep pip key fails on Ubuntu 24.04)
sudo apt install python3-transforms3d
```

The system packages this pulls in include `ros-jazzy-realsense2-camera`, `ros-jazzy-apriltag`,
`ros-jazzy-apriltag-msgs`, `ros-jazzy-camera-ros`, `ros-jazzy-image-transport-plugins`,
`ros-jazzy-rqt-gui`, `ros-jazzy-rqt-gui-py`, `ros-jazzy-cv-bridge`, `python3-opencv`, `python3-scipy`.

**RealSense udev rules** (otherwise the camera streams then drops with `Frames didn't arrive within
5 seconds`): install librealsense's `99-realsense-libusb.rules` into `/etc/udev/rules.d/`, then
`sudo udevadm control --reload-rules && sudo udevadm trigger`
([instructions](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)).

### 2.3 Build

```bash
cd $FP3_ROOT/calib_ws
source /opt/ros/jazzy/setup.bash
source $FP3_ROOT/franka_ros2_ws/install/setup.bash
source $FP3_ROOT/franka_demo_ws/install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Every new terminal needs the same four `source` lines, in this order.
Sanity check: `ros2 launch calib_bringup calib_bringup.launch.py --show-args` prints the arguments.

---

## 3. Prepare the tag

1. Get the tag image from [AprilRobotics/apriltag-imgs](https://github.com/AprilRobotics/apriltag-imgs)
   (`tag36h11`, id 0), scale it and print at **100 % scale** (no "fit to page").
2. **Measure the black square only** (white border excluded), in meters. That number is the tag `size`.
3. Mount it on a **rigid** support fixed to the gripper. Any slip during the session invalidates every
   sample taken so far: never re-open the gripper before the end.
4. Use the detector config with the matching size. Ready-made configs are in
   `handeye_tf_publisher/tags/`:

   | File | Tag size |
   |---|---|
   | `36h11_0_0.04.yaml` | 4 cm |
   | `36h11_0_0.12.yaml` | 12 cm |
   | `36h11_0_0.129.yaml` | 12.9 cm |

   Pass it with `apriltag_params_file:=<absolute path>`. To add a size, copy a file and change
   `size` (name convention `<family>_<id>_<size>.yaml`). The size **must** match the printed tag,
   otherwise every translation is scaled.

---

## 4. Run a calibration (eye-on-base, D455)

The arm is guided **by hand** with Franka's gravity-compensation controller. `calib_bringup` is
therefore started with `start_arm_stack:=false` (the MoveIt stack would hold the arm stiff, and only
one process may hold the robot connection).

**Terminal 1 — free-drive controller.** On the Desk: execution mode, unlock the joints, activate FCI. Then:

```bash
source /opt/ros/jazzy/setup.bash && source $FP3_ROOT/franka_ros2_ws/install/setup.bash
source $FP3_ROOT/franka_demo_ws/install/setup.bash          # for the robot_configs below
ros2 launch franka_bringup example.launch.py \
  robot_config_file:=$(ros2 pkg prefix franka_fp3_moveit_config)/share/franka_fp3_moveit_config/robot_configs/fp3.config.yaml \
  controller_names:=gravity_compensation_example_controller
```

**Terminal 2 — put the tag in the gripper and close it firmly** (do not re-open it until the end):

```bash
ros2 action send_goal /franka_gripper/move franka_msgs/action/Move "{width: 0.08, speed: 0.1}"
# place the tag support between the fingers, then:
ros2 action send_goal /franka_gripper/grasp franka_msgs/action/Grasp \
  "{width: 0.06, speed: 0.05, force: 70.0, epsilon: {inner: 0.06, outer: 0.08}}"
```
Then check the achieved width with `ros2 topic echo /franka_gripper/joint_states --once` (sum of the two
finger positions): with these wide tolerances `success: true` can also mean the fingers closed on
nothing (width ≈ 0). Adjust `width` to your support.

**Terminal 3 — calibration session** (camera, tag detection, `easy_handeye2` UI, sample guard):

```bash
source /opt/ros/jazzy/setup.bash
source $FP3_ROOT/franka_ros2_ws/install/setup.bash
source $FP3_ROOT/franka_demo_ws/install/setup.bash
source $FP3_ROOT/calib_ws/install/setup.bash
ros2 launch calib_bringup calib_bringup.launch.py \
  use_fake_hardware:=false start_arm_stack:=false \
  apriltag_params_file:=$FP3_ROOT/calib_ws/src/handeye_tf_publisher/tags/36h11_0_0.12.yaml \
  calibration_name:=fp3_link0_d455_camera_color_optical_frame_003
```

Increment the number in `calibration_name` for each attempt: reusing a name overwrites the previous `.calib`.
An `rqt` window opens. In its algorithm drop-down choose **Park** (not the default Tsai-Lenz).

**Take samples** (aim for 15–20; the arm must be *still* before each one):

1. Guide the arm by hand to a new pose and release it; wait ≥ 2 s.
2. Read the `calib_sample_guard` line in Terminal 3: take the sample only when it says **`OK`**
   (reprojection error small, tilt neither too flat nor too oblique).
3. Click **Take sample** in `rqt`.

Sampling rules that matter (AX = XB is under-determined otherwise):
- Vary **wrist orientation** (roll, pitch, yaw), not only position — over at least 2–3 non-parallel axes.
- Never chain two poses that differ by a pure translation.
- Avoid tilts > 60° relative to the camera axis; avoid near head-on views.
- Cover the volume the robot will actually work in (near and far from the camera, left and right).
- The first ~8–10 samples make the estimate swing wildly; that is normal.

Optional live convergence readout (prints the translation change after every sample):

```bash
ros2 launch calib_bringup calib_bringup.launch.py <same arguments> 2>&1 \
  | tee /tmp/calib.log | python3 $FP3_ROOT/calib_ws/src/handeye_tf_publisher/tools/watch_calibration_convergence.py
```
It prints `<-- CONVERGE` after 3 consecutive small deltas.

**Save**: click **Save calibration** in `rqt` → `~/.ros2/easy_handeye2/calibrations/<calibration_name>.calib`.
To be able to add samples later, also save the raw samples:

```bash
ros2 service call /easy_handeye2/calibration/save_samples easy_handeye2_msgs/srv/SaveSamples "{}"
```

Stop everything with Ctrl-C, then open the gripper (Terminal 2 `move` with `width: 0.08`).

> Using `use_fake_hardware:=true` (the default) starts MoveIt with simulated hardware, useful only to check
> that everything launches. Without `start_arm_stack:=false`, the arm is held stiff by MoveIt and cannot be hand-guided.

---

## 5. Publish and verify a calibration

**Publish** `fp3_link0 → camera_link` (do *not* use `easy_handeye2`'s own `publish.launch.py`: it publishes
straight onto `camera_color_optical_frame` and conflicts with the RealSense TF tree):

```bash
ros2 launch handeye_tf_publisher publish.launch.py \
  calibration_name:=fp3_link0_d455_camera_color_optical_frame_003
ros2 run tf2_ros tf2_echo fp3_link0 camera_link          # check the chain
```

**Quantitative check** — `evaluate_calibration.launch.py` starts the camera, tag detection, the published
calibration and `easy_handeye2`'s evaluator. Move the arm by hand (free-drive
bringup of section 4, terminal 1, and `start_arm_stack:=false`) to a few **new** poses, tag visible, arm still, and read
**Maximum divergence** in the window:

```bash
ros2 launch calib_bringup evaluate_calibration.launch.py \
  use_fake_hardware:=false start_arm_stack:=false \
  calibration_name:=fp3_link0_d455_camera_color_optical_frame_003
```
Rough guide: < 1 cm good, 1–3 cm usable, > 3 cm redo the calibration.

**Physical check** — `fp3_apriltag_demo` moves the tool onto the tag pose. **The arm moves by itself**:
clear the workspace, keep the E-stop in hand, start with a tag placed on the table.

```bash
ros2 launch fp3_apriltag_demo apriltag_move_once.launch.py \
  use_fake_hardware:=false calibration_name:=fp3_link0_d455_camera_color_optical_frame_003
ros2 run tf2_ros tf2_echo tag36h11:0 fp3_hand_tcp     # translation ≈ [0,0,0] once arrived
```

---

## 6. Optional: cross-check with a wrist D405

Calibrating a second camera independently, then comparing both views of the same fixed tag, validates the
D455 result without circularity.

1. **Calibrate the D405** (eye-in-hand, fixed tag, **D455 unplugged**), then sample by hand as in section 4:

   ```bash
   ros2 launch calib_eye_in_hand calibrate_eye_in_hand.launch.py \
     use_fake_hardware:=false start_arm_stack:=false      # free-drive bringup running, as in section 4
   ```
2. **Derive the D455 calibration** with both cameras plugged in (serial numbers from `rs-enumerate-devices -s`):

   ```bash
   ros2 launch calib_bridge calib_bridge.launch.py use_fake_hardware:=false \
     d455_serial_no:=<...> d405_serial_no:=<...> \
     d405_calibration_name:=fp3_hand_d405_camera_color_optical_frame_001
   ros2 service call /calib_bridge/take_sample std_srvs/srv/Trigger {}     # at several different poses, arm still
   ros2 service call /calib_bridge/save_calibration std_srvs/srv/Trigger {}
   ```
   Output: `…_d455_camera_color_optical_frame_derived_001.calib` (name differs on purpose; compare it with the
   direct one via `evaluate_calibration.launch.py` before replacing anything).

---

## 7. Package reference

### `calib_bringup`
`ros2 launch calib_bringup calib_bringup.launch.py` — arguments:

| Argument | Default | Meaning |
|---|---|---|
| `use_fake_hardware` | `true` | Simulated arm. **Set `false` for the real robot.** |
| `robot_ip` | `192.168.1.1` | FP3 controller IP |
| `start_arm_stack` | `true` | Start `fp3_moveit_server` (arm held stiff by MoveIt). `false` for hand-guiding |
| `use_rviz` | `true` | RViz of the MoveIt stack |
| `calibration_type` | `eye_on_base` | `easy_handeye2` calibration type |
| `robot_effector_frame` | `fp3_hand` | Link carrying the tag |
| `tracking_marker_frame` | `tag36h11:0` | TF frame of the tag |
| `calibration_name` | `fp3_link0_d455_camera_color_optical_frame_001` | `.calib` file created/overwritten |
| `apriltag_params_file` | `…/tags/36h11_0_0.04.yaml` | Detector config (family, size) |
| `target_tag_id` | `0` | Tag id watched by `calib_sample_guard` |

`evaluate_calibration.launch.py`: `camera` (`d455`|`d405`), `calibration_name`, `apriltag_params_file`,
`use_fake_hardware`, `robot_ip`, `use_rviz`, `start_arm_stack`.
The camera is started with `align_depth.enable:=true initial_reset:=true` at 1280x720x30 through
`scripts/launch_realsense_with_retry.sh`, which restarts it automatically if the USB re-enumeration
makes the first attempt crash.

### `calib_sample_guard`
Node `sample_guard`. Subscribes `/detections` and `camera_info`; parameters `target_tag_id` (0) and
`tag_size` (0.04 m). Prints every 0.5 s: reprojection error, tilt from head-on, depth, and a verdict
(warns if reprojection > 1.5 px, tilt < 20° or > 60°). `calib_bringup` forwards only `target_tag_id`:
with a tag that is not 4 cm the displayed depth is scaled, the error and tilt stay valid.

### `handeye_tf_publisher`
`ros2 launch handeye_tf_publisher publish.launch.py calibration_name:=<name> [calib_dir:=~/.ros2/easy_handeye2/calibrations]`.
Works for eye-on-base and eye-in-hand `.calib` files. Publishes `fp3_link0 → camera_link` (static).

### `calib_eye_in_hand`
`calibrate_eye_in_hand.launch.py`: `use_fake_hardware`, `robot_ip`, `use_rviz`, `start_arm_stack`, `load_gripper` (`true`),
`robot_effector_frame` (`fp3_hand`), `tracking_marker_frame`, `calibration_name`
(`fp3_hand_d405_camera_color_optical_frame_001`), `apriltag_params_file`, `camera_serial_no`.
D405 profile: 848x480, depth disabled. Details in `src/calib_eye_in_hand/CLAUDE.md`.

### `calib_bridge`
`calib_bridge.launch.py` + node `bridge_calibration_node`; services `/calib_bridge/take_sample` and
`/calib_bridge/save_calibration` (`std_srvs/Trigger`). Both cameras run under namespace `franka`
(`/franka/d455/…`, `/franka/d405/…`). Details in `src/calib_bridge/CLAUDE.md`.

### `fp3_apriltag_demo`
`apriltag_move_once.launch.py`: `tag_size`, `target_tag_id`, `camera`, `calibration_name`,
`apriltag_params_file`, `flip_tag_orientation` (`true`), `force_top_down` (`false`),
`offset_x/y/z` (m, test only), `gripper_width_m` (`0.08`). Node stops after one move.
Details in `src/fp3_apriltag_demo/CLAUDE.md`.

---

## 8. Files and locations

| What | Where |
|---|---|
| Saved calibrations | `~/.ros2/easy_handeye2/calibrations/<name>.calib` (**outside the repo — back them up**) |
| External clones | `src/external/` (git-ignored, recreated by `scripts/install_dependencies.sh`) |
| Pinned external versions | `calib.repos` |
| Tag detector configs | `src/handeye_tf_publisher/tags/` |
| Per-package notes / history | `CLAUDE.md` at the workspace root and in some packages |

---

## 9. Troubleshooting

- **`realsense2_camera_node` survives Ctrl-C** and the next launch fails: `pkill -9 -f realsense2_camera_node`.
- **Camera stops after a few seconds / `Frames didn't arrive`**: udev rules missing (section 2.2), or USB power
  management; use a USB 3 port.
- **RViz/TF looks wrong during a calibration session**: `easy_handeye2`'s `calibrate.launch.py` starts a
  `dummy_publisher` with a fake `fp3_link0 → camera_color_optical_frame` transform. It is not used by the solver;
  `kill` it (`ps aux | grep dummy_publisher`) if it disturbs the view.
- **`ros2` shows nodes that are dead**: `ros2 daemon stop && ros2 daemon start`.
- **`libfranka: … communication_constraints_violation`** on the arm: network real-time issue on the robot link,
  see `franka_demo_ws/franka_reflex.md`. After such a reflex the controllers are inactive: relaunch the arm stack.
- **Nothing detected**: check lighting, that the tag is in the field of view, that `size`/family in the
  `apriltag_params_file` match the printed tag, and `ros2 topic hz /camera/camera/color/image_raw`.
- **`colcon build` warns about a package shadowed by another workspace**: harmless, the workspace sourced last wins.

## Known limitations

- The default arm stack of `calib_bringup`, `evaluate_calibration` and `calib_eye_in_hand` is MoveIt-based (arm held
  stiff): hand-guiding needs `start_arm_stack:=false` and the separate free-drive bringup (section 4).
- Commands in sections 4–6 come from procedures validated on the real robot earlier in the project; the
  workspace build and every launch file (`--show-args`) were re-checked, the full sessions were not re-run.
- `handeye_tf_publisher` also exists, in a slightly different version, in `franka_demo_ws`.
