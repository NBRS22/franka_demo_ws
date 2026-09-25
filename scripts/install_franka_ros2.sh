#!/usr/bin/env bash
# Clones Franka's franka_ros2 (NOT versioned in this repository) at the version this
# project was developed against, imports its dependencies and builds it.
# Usage: scripts/install_franka_ros2.sh [--no-build]
set -euo pipefail

FRANKA_ROS2_COMMIT=73a1501d76efa2bc4bf09cb2af9c2b72c2c642da    # branch jazzy, v3.2.0-344-g73a1501
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$ROOT/franka_ros2_ws"

if [ -z "${ROS_DISTRO:-}" ]; then
  echo "ROS is not sourced: run 'source /opt/ros/jazzy/setup.bash' first." >&2
  exit 1
fi

mkdir -p "$WS"
if [ ! -d "$WS/src/.git" ]; then
  echo "==> Cloning franka_ros2 into $WS/src"
  git clone -b jazzy https://github.com/frankarobotics/franka_ros2.git "$WS/src"
fi
git -C "$WS/src" fetch -q origin jazzy
git -C "$WS/src" checkout -q "$FRANKA_ROS2_COMMIT"
echo "==> franka_ros2 at $(git -C "$WS/src" describe --tags --always)"

echo "==> Importing dependencies (libfranka, franka_description, ...)"
command -v vcs >/dev/null || sudo apt-get install -y python3-vcstool
vcs import "$WS/src" < "$WS/src/dependency.repos"

echo "==> rosdep (asks for sudo only if some package is missing)"
rosdep update >/dev/null
rosdep install --from-paths "$WS/src" --ignore-src --rosdistro "$ROS_DISTRO" -y -r

if [ "${1:-}" = "--no-build" ]; then echo "Skipping build."; exit 0; fi
echo "==> Building (this takes several minutes)"
cd "$WS"
colcon build --symlink-install
echo "Done. Source it with: source $WS/install/setup.bash"
