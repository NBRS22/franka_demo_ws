# Eye-to-Hand Calibration

Complete guide to calibrate the `robot_base → camera_link` transform.

**Stack:** ROS2 · `apriltag_ros` (christianrauch) · `easy_handeye2` (marcoesposito1988) · `realsense2_camera` · `handeye_tf_publisher` (this package)

---

# 1 — Installation

> To be done **once** on the machine.

## Prerequisites

- ROS2 installed
- `realsense2_camera` driver installed and working

## 1.1 Clone external dependencies

```bash
cd ~/franka_demo_ws/src

# AprilTag detection
git clone https://github.com/christianrauch/apriltag_ros.git

# Hand-eye calibration tool
git clone https://github.com/marcoesposito1988/easy_handeye2.git
```

## 1.2 Build the workspace

```bash
cd ~/franka_demo_ws
colcon build
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

---

# Chapter 2 — Configuration

> To be done once, or repeated when adding a new tag.

## 2.1 Prepare an AprilTag

### Download and print

**1. Download the PNG:**

AprilTag images for all families are available at:
```
https://github.com/AprilRobotics/apriltag-imgs
```

Navigate to the folder of the desired family (e.g. `tag36h11/`, `tag25h9/`) and pick the desired ID.

**2. Scale up and generate a printable PDF** using `tools/apriltag_pdf_generator.py`:

> **Not written yet** — this script is referenced by this guide but does not exist in `tools/` at the time of writing. Either write it (a PNG → scaled/labelled PDF generator, e.g. with `Pillow`+`reportlab`) or print the raw AprilTag PNG directly at a known DPI and skip this step — just make sure whatever method you use lets you hit an exact, measurable physical size.

```bash
# Dependencies (isolated venv so this doesn't pollute the ROS2 Python env)
python3 -m venv ~/.venvs/apriltag_pdf
source ~/.venvs/apriltag_pdf/bin/activate
pip install pillow reportlab
deactivate

# Generate the PDF
source ~/.venvs/apriltag_pdf/bin/activate
python3 ~/franka_demo_ws/src/handeye_tf_publisher/tools/apriltag_pdf_generator.py \
  --input <tag_image>.png \
  --size 4 \
  --border 2 \
  --label "<family> ID=<id>" \
  --output <family>_<id>_4cm.pdf
deactivate
```

- `--size` : desired physical size of the black square in cm
- `--border` : white border around the tag in cm
- `--label` : optional text printed below the tag (e.g. `tag36h11 ID=0`)

**3. Print** at **100% scale** — disable "fit to page".

**4. Measure** the side of the **black square only** (white border excluded) → record in meters. This value goes into `size`.

**5. Mount** on a rigid support.

### Create the yaml config file

Folder: `~/franka_demo_ws/src/handeye_tf_publisher/tags/`
Naming convention: `<family>_<ID>_<size>.yaml` — e.g. `36h11_0_0.04.yaml`

```bash
cat > ~/franka_demo_ws/src/handeye_tf_publisher/tags/<family>_<ID>_<size>.yaml << 'EOF'
apriltag:
  ros__parameters:
    family: <family>    # e.g. 36h11, 25h9, Standard41h12
    size: 0.XX          # actual black square side in meters
    max_hamming: 0
    detector:
      threads: 2
      decimate: 1.0
      blur: 0.0
      refine: true
      sharpening: 0.25
    tag:
      ids: [X]
      frames: [tag<family>:X]
      sizes: [0.XX]
EOF
```

> **Tag used in this guide:** family `36h11`, ID `0`, black square `4 cm` → file `36h11_0_0.04.yaml`

---

# Chapter 3 — Calibration (FP3 + RealSense)

> Repeat for each new calibration session.

## Terminal overview

```
Robot Terminal 1   : FP3 bringup (gravity compensation)   (franka_ros2_ws)
Robot Terminal 2   : gripper commands                     (franka_ros2_ws)
Camera Terminal    : realsense2_camera
AprilTag Terminal  : apriltag_ros                          (franka_demo_ws)
Calibration Terminal : easy_handeye2                        (franka_demo_ws)
Publisher Terminal : handeye_tf_publisher                   (franka_demo_ws)
```

---

## Step 1 — Prepare the robot

### Robot Terminal 1 — Bringup

```bash
cd ~/franka_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

From the **Desk** (Franka web interface):
- Select execution mode
- Unlock the joints and end effector
- Activate FCI

Launch the free-drive controller (needed to move the arm by hand — this is what `freehand_robot_movement: true` in the calibration parameters assumes):
```bash
ros2 launch franka_bringup example.launch.py \
  robot_config_file:=fp3.config.yaml \
  controller_names:=gravity_compensation_example_controller
```

### Robot Terminal 2 — Gripper

```bash
cd ~/franka_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Open the gripper:
```bash
ros2 action send_goal /franka_gripper/move franka_msgs/action/Move \
  "{width: 0.08, speed: 0.1}"
```

Place the cube with the tag in the gripper, then close with maximum force:
```bash
ros2 action send_goal /franka_gripper/grasp franka_msgs/action/Grasp \
  "{width: 0.06, speed: 0.05, force: 70.0, epsilon: {inner: 0.06, outer: 0.08}}"
```

> **Do not open the gripper again until the end of the session** — see the rigidity note in Chapter 2, any re-grasp changes the tag's offset relative to `fp3_hand` and invalidates every sample taken so far.

---

## Step 2 — Launch the camera

```bash
ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true
```

Verify the image stream is running:
```bash
ros2 topic hz /camera/camera/color/image_raw
```

> If you get repeated `Frames didn't arrived within 5 seconds` warnings with no actual USB disconnect in `dmesg`, this machine is missing the RealSense udev permission rules (`/etc/udev/rules.d/99-realsense-libusb.rules`) — see `franka_demo_bringup/CLAUDE.md`, "Dépannage" section, for the fix. If it happens again after the node has run fine once already (only on relaunch), try `sudo usbreset 8086:0b5c` before relaunching — same section documents why `initial_reset:=true` is not a reliable substitute.

---

## Step 3 — Launch AprilTag detection

```bash
cd ~/franka_demo_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run apriltag_ros apriltag_node \
  --ros-args \
  -r image_rect:=/camera/camera/color/image_raw \
  -r camera_info:=/camera/camera/color/camera_info \
  --params-file ~/franka_demo_ws/src/handeye_tf_publisher/tags/36h11_0_0.04.yaml
```

Verify detection (tag facing the camera):
```bash
ros2 run tf2_ros tf2_echo camera_color_optical_frame tag36h11:0
```
Should display an updating transform. If nothing appears → check lighting and that the tag is in the camera's field of view.

**Recommended extra check** — visualize in RViz before starting calibration: add **TF** and a **PointCloud2** (with `pointcloud.enable:=true` added to Step 2's launch command) and confirm the tag's TF frame visually sits on the physical tag in the point cloud. This is just a sanity check on tag detection + camera intrinsics, not a calibration check (the native RealSense cloud has its own known alignment offset — see Step 8).

---

## Step 4 — Launch easy_handeye2

```bash
cd ~/franka_demo_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch easy_handeye2 calibrate.launch.py \
  calibration_type:=eye_on_base \
  name:=fp3_link0_d455_camera_color_optical_frame_001 \
  robot_base_frame:=fp3_link0 \
  robot_effector_frame:=fp3_hand \
  tracking_base_frame:=camera_color_optical_frame \
  tracking_marker_frame:=tag36h11:0
```

> Increment the session number (`_001`, `_002`...) for each new calibration attempt — reusing a name silently overwrites the previous `.calib` file.

### ⚠️ If RViz breaks / TF looks wrong during this step

`calibrate.launch.py` starts a `dummy_publisher` node (`static_transform_publisher`) that publishes a **fake** `fp3_link0 → camera_color_optical_frame` transform (placeholder, 1m offset) — this conflicts with the real edge RealSense already publishes (`camera_color_frame → camera_color_optical_frame`), giving `camera_color_optical_frame` two parents and breaking TF/RViz visualization for the duration of the calibration session.

This dummy transform is **not used by the actual sample-taking or solver math** (verified in `handeye_sampler.py` — it only reads `robot_base_frame ↔ robot_effector_frame` and `tracking_base_frame ↔ tracking_marker_frame`), so it's safe to kill:

```bash
ps aux | grep dummy_publisher | grep -v grep
kill <PID>
```

`handeye_server` and `rqt_calibrator` keep running fine without it.

---

## Step 5 — Collect samples

**Key rules:**
- Move **only the arm** — never touch the cube or the gripper
- Wait for the arm to be **completely still** before each sample (at least 2 seconds — `easy_handeye2` samples TF ~200ms in the past, moving too soon after settling reads a stale/mismatched robot pose)
- Vary **wrist orientations** (roll, pitch, yaw) above all — not just positions, and across **at least 2-3 non-parallel rotation axes** (a hand-eye solve is under-determined if all samples only vary one axis)
- Never make two consecutive poses that differ only by a pure translation
- Avoid tilt angles > 60° relative to the camera axis (grazing angles degrade the tag's PnP pose estimate)

**For each sample:**
```
① Guide the arm by hand to the new pose
② Release gently — wait 2 seconds
③ Verify that tag36h11:0 is visible and stable in RViz
④ Click "Take sample" in the easy_handeye2 interface
```

**Suggested pose plan (18 poses):**

| Group | Variation | Count |
|---|---|---|
| Center ~60 cm from camera | ±30° roll, ±25° pitch, ±20° yaw | 7 |
| Shifted left/right | Combined translation + rotation | 4 |
| Close ~40 cm | ±15° roll/pitch | 3 |
| Far ~80 cm | ±25° yaw/roll | 3 |
| Combined poses | Roll + pitch simultaneously | 3 |

### Watch convergence live

`handeye_server` recomputes and prints the full calibration matrix after every single sample — pipe the launch output from Step 4 through `tools/watch_calibration_convergence.py` to track it without eyeballing raw matrices:

```bash
ros2 launch easy_handeye2 calibrate.launch.py \
  calibration_type:=eye_on_base \
  name:=fp3_link0_d455_camera_color_optical_frame_001 \
  robot_base_frame:=fp3_link0 robot_effector_frame:=fp3_hand \
  tracking_base_frame:=camera_color_optical_frame tracking_marker_frame:=tag36h11:0 \
  2>&1 | tee /tmp/calib.log | python3 ~/franka_demo_ws/src/handeye_tf_publisher/tools/watch_calibration_convergence.py
```

Each sample prints a line like:
```
[sample 12] translation=(0.15, 0.98, 0.43)  delta=0.3cm
```
`delta` is the translation change vs. the previous sample. The **first ~8-10 samples will swing wildly** (tens of cm) — this is normal, Tsai-Lenz is very unstable with few samples, ignore it. Once you see `<-- CONVERGE` (3 consecutive deltas under 1cm), that's the signal to stop and save — collecting more samples past that point has diminishing returns.

---

## Step 6 — Save the calibration

In the rqt `easy_handeye2` interface, click **"Save calibration"** once `watch_calibration_convergence.py` shows convergence.

> There is **no residual-error readout at this step** — `easy_handeye2`'s calibrator UI does not compute or display one. The only way to get a quantitative error number is the separate evaluator in Step 8, run *after* publishing. Don't wait for a number here that will never appear.

Saved to:
```
~/.ros2/easy_handeye2/calibrations/<name>.calib
```

---

## Step 7 — Publish the calibration

```bash
cd ~/franka_demo_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch handeye_tf_publisher publish.launch.py \
  calibration_name:=fp3_link0_d455_camera_color_optical_frame_001
```

> ⚠️ **Do not use `easy_handeye2`'s own `publish.launch.py`** (`ros2 launch easy_handeye2 publish.launch.py`) — it publishes `fp3_link0 → camera_color_optical_frame` directly, which conflicts with the real edge RealSense already publishes for that same child frame (`camera_color_frame → camera_color_optical_frame`) and breaks the TF tree. `handeye_tf_publisher` (this package) re-anchors the calibration on `camera_link` instead — the one frame in the camera's internal TF subtree that has no parent of its own — so it doesn't conflict with anything.

Verify the full TF chain:
```bash
ros2 run tf2_ros tf2_echo fp3_link0 camera_link
ros2 run tf2_tools view_frames && evince frames.pdf
```

---

## Step 8 — Verify the calibration

### Quantitative check (do this first)

```bash
ros2 launch easy_handeye2 evaluate.launch.py name:=fp3_link0_d455_camera_color_optical_frame_001
```

Move the arm to a few **new** poses (not the ones used for calibration), tag kept visible, waiting for a steady state at each one. The UI's **"Maximum divergence"** field is the residual error: it repeatedly measures `robot_effector_frame → tag` through the full calibration chain, which should be constant since the tag is rigid on the effector — the spread across poses *is* the error.

Rough guide:
- **< 1 cm** : good calibration
- **1-3 cm** : usable for most objects with a parallel gripper, but tight relative to this pipeline's own GraspGen collision-filter margin (`collision_threshold` 5-10mm, cf. `graspgen_bridge/CLAUDE.md`) — test with a real pick before trusting it on tight-tolerance objects
- **> 3 cm** : likely under-converged (check sample count/diversity with Step 5's convergence tool) or a rigidity issue (cf. Chapter 2 tag-mounting note)

### Visual check in RViz (secondary, qualitative only)

- Fixed frame: `fp3_link0`
- Add **RobotModel** + **TF** + **PointCloud2** (needs `pointcloud.enable:=true` added to Step 2's launch command)
- The point cloud should roughly overlap the URDF model (table, arm)

> This check alone is **not reliable** for judging calibration quality — a few-cm offset is easy to miss by eye, and the RealSense native point cloud has its own known small alignment offset (`align_depth.enable` + `pointcloud.enable` combined, documented upstream, cf. root `CLAUDE.md` dette technique) independent of calibration accuracy. Use it as a rough sanity check only ("is this in the right ballpark") — trust the evaluator's number for the actual go/no-go call.
