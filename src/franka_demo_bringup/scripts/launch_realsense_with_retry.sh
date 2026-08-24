#!/bin/bash
# Wraps `ros2 launch realsense2_camera rs_launch.py "$@"` with an automatic
# retry, working around a known race with initial_reset:=true: the SDK-level
# hardware_reset() this triggers makes the D455 fully disconnect/re-enumerate
# on the USB bus (the only thing confirmed to fix "Depth stream start
# failure" + "Frames didn't arrived within 5 seconds" on this machine --
# plain `usbreset` does NOT, it only resets the USB link, not the camera's
# own firmware state -- cf. franka_demo_bringup/CLAUDE.md). The ROS wrapper
# node sometimes tries to reopen the device before that re-enumeration has
# finished, crashing with "Device or resource busy" -> "No such device"
# within a few seconds of startup. Confirmed manually: a bare second attempt
# (no physical unplug) then succeeds -- this script automates that retry.
#
# A crash is only treated as this startup race if it happens within
# STARTUP_GRACE_S of the attempt starting; anything after that (or a normal
# Ctrl-C) is passed through as-is, no retry.
set -u

MAX_ATTEMPTS=4
STARTUP_GRACE_S=20
RETRY_DELAY_S=3

CHILD_PID=""

_forward_and_exit() {
  sig="$1"
  if [ -n "$CHILD_PID" ] && kill -0 "$CHILD_PID" 2>/dev/null; then
    kill "-$sig" "$CHILD_PID"
    wait "$CHILD_PID"
  fi
  exit $?
}
trap '_forward_and_exit TERM' TERM
trap '_forward_and_exit INT' INT

attempt=1
while [ "$attempt" -le "$MAX_ATTEMPTS" ]; do
  echo "[launch_realsense_with_retry] attempt ${attempt}/${MAX_ATTEMPTS}: ros2 launch realsense2_camera rs_launch.py $*"
  start_ts=$(date +%s)

  ros2 launch realsense2_camera rs_launch.py "$@" &
  CHILD_PID=$!
  wait "$CHILD_PID"
  code=$?
  elapsed=$(( $(date +%s) - start_ts ))
  CHILD_PID=""

  if [ "$code" -eq 0 ]; then
    exit 0
  fi
  if [ "$elapsed" -ge "$STARTUP_GRACE_S" ]; then
    echo "[launch_realsense_with_retry] exited with code ${code} after ${elapsed}s (past the ${STARTUP_GRACE_S}s startup grace) -- not retrying" >&2
    exit "$code"
  fi

  echo "[launch_realsense_with_retry] exited with code ${code} after only ${elapsed}s -- likely the known initial_reset re-enumeration race, retrying in ${RETRY_DELAY_S}s" >&2
  sleep "$RETRY_DELAY_S"
  attempt=$((attempt + 1))
done

echo "[launch_realsense_with_retry] giving up after ${MAX_ATTEMPTS} attempts" >&2
exit 1
