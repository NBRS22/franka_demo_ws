# ER — Gemini ER click-to-pick simulator

Stands in for the Gemini Robotics-ER VLM while developing: it shows the robot camera image and, when you **click an
object**, sends a pick command to the pipeline exactly like the real model would. It runs **outside ROS**, in its own
conda environment (`ER`).

```
camera_bridge (ZMQ PUB :5555, JPEG)  ──▶  gemini_er_simulator.py  ──click──▶  command_bridge (ZMQ REP :5556)
                                                                              └─▶ /execute_pick_task (robot_task_manager)
```

The click is sent in Gemini's convention (**coordinates normalized to 0–1000**) together with the object label;
`command_bridge` converts it back to image pixels. Replacing this simulator by the real API only requires producing
the same message: `{"task_type": "pick"|"place", "point_x", "point_y", "object_label"}` (msgpack, REQ/REP).

## Setup

From the repository root: `scripts/install_conda.sh && scripts/setup_envs.sh ER`. Manually:

```bash
conda create -n ER python=3.12 -y && conda activate ER
pip install opencv-python==4.9.0.80 numpy==1.26.4 pyzmq msgpack
```
Do **not** source ROS in this environment (OpenCV/NumPy versions differ from the system ones).

## Run

The pipeline must be running first (`ros2 launch franka_demo_bringup franka_demo.launch.py`, see
[`../franka_demo_ws/README.md`](../franka_demo_ws/README.md)); it publishes the camera stream on port 5555.

```bash
conda activate ER
cd $FP3_ROOT/ER
python gemini_er_simulator.py --label cube                 # host 127.0.0.1, ports 5555/5556
python gemini_er_simulator.py --robot-ip 192.168.x.y       # pipeline running on another machine
```

| Key / action | Effect |
|---|---|
| left click | send the command for the clicked pixel |
| `l` | change the object label |
| `t` | toggle `pick` / `place` |
| `q` | quit |

`main.py` is an equivalent entry point (`--label`, `--task`, `--robot-ip`, `--camera-port`, `--bridge-port`).

The pipeline is safe by default (`execute_pick:=false`: the arm does not move); a click then only produces the
segmentation, the point clouds and the grasp candidates in RViz. A pick blocks the command socket until it ends
(timeout 180 s): a `stop` cannot be sent in the meantime.
