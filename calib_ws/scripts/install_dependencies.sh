#!/usr/bin/env bash
# One-shot dependency setup for calib_ws (ROS 2 Jazzy, Ubuntu 24.04).
# Usage: scripts/install_dependencies.sh            (asks for sudo for apt)
# Prerequisite: /opt/ros/jazzy installed, franka_ros2_ws and franka_demo_ws
# built and sourced (see README.md, "Prerequisites").
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ -z "${ROS_DISTRO:-}" ]; then
  echo "ROS is not sourced: run 'source /opt/ros/jazzy/setup.bash' first." >&2
  exit 1
fi

echo "==> 1/3 External sources (pinned, see calib.repos)"
command -v vcs >/dev/null || sudo apt-get install -y python3-vcstool
command -v rosdep >/dev/null || sudo apt-get install -y python3-rosdep
vcs import "$WS/src" < "$WS/calib.repos"

echo "==> 2/3 ROS / system dependencies (rosdep)"
# fp3_moveit_server and franka_demo_interfaces come from franka_demo_ws (built
# separately, not resolvable by rosdep); python-transforms3d-pip is replaced by
# the apt package below (the pip key breaks on Ubuntu 24.04's externally-managed Python).
rosdep update
rosdep install --from-paths "$WS/src" --ignore-src -r -y \
  --skip-keys "fp3_moveit_server franka_demo_interfaces python-transforms3d-pip"
python3 -c "import transforms3d" 2>/dev/null || sudo apt-get install -y python3-transforms3d

echo "==> 3/3 RealSense udev rules (needed for the D455/D405 to open without root)"
if ! ls /etc/udev/rules.d 2>/dev/null | grep -qi realsense; then
  echo "    Not found: install librealsense2's 99-realsense-libusb.rules into /etc/udev/rules.d/,"
  echo "    then: sudo udevadm control --reload-rules && sudo udevadm trigger"
  echo "    (https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)"
fi

echo "Done. Next: colcon build --symlink-install (see README.md)."
