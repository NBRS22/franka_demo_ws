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
# How long to give the normal signal-forwarding shutdown chain (this script
# -> nested `ros2 launch realsense2_camera` -> realsense2_camera_node) before
# the watchdog below force-kills any straggler. Chosen higher than the
# ~5s default sigterm_timeout an outer `ros2 launch` typically allows an
# ExecuteProcess before SIGKILLing it directly -- cf. camera.md,
# "realsense2_camera_node reste orphelin".
REAPER_GRACE_S=8

CHILD_PID=""

# realsense2_camera_node consistently survives as an orphan after this
# script's own process tree is torn down (confirmed repeatedly: an outer
# `ros2 launch` sending SIGINT to this script cleanly stops everything else
# it manages, but not this node). Root cause: the nested
# `ros2 launch realsense2_camera` + its own node can take longer to shut
# down (real USB/driver teardown, made slower still by this machine's own
# USB flakiness, cf. camera.md) than the outer launch's patience for this
# script to exit -- if that patience runs out first, the outer launch sends
# SIGKILL directly to THIS script, which is uncatchable: no trap in this
# script can react, so `kill "-$sig" "$CHILD_PID"` below never gets a chance
# to even run, let alone finish.
#
# Fix: a fully detached watchdog (own session via setsid, not a process the
# outer launch is itself tracking) that doesn't depend on this script
# surviving long enough to clean up after itself. It watches this script's
# own PID; once that PID is gone (however it died -- clean exit OR SIGKILL),
# it gives the normal chain REAPER_GRACE_S more seconds in case it was just
# slow but still working, then force-kills any realsense2_camera_node still
# alive. Runs for the whole lifetime of this script, harmless if the normal
# shutdown chain already finished the job (pkill on a name match with
# nothing left to match is a silent no-op).
setsid bash -c '
  parent_pid='"$$"'
  while kill -0 "$parent_pid" 2>/dev/null; do sleep 1; done
  sleep '"$REAPER_GRACE_S"'
  pkill -9 -f realsense2_camera_node 2>/dev/null
' </dev/null >/dev/null 2>&1 &

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
