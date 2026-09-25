#!/usr/bin/env bash
# Launches every piece of the project with SIMULATED hardware and checks that it comes up.
# Nothing is sent to a real robot: use_fake_hardware:=true everywhere, and an isolated ROS domain.
#
# Usage: scripts/smoke_test.sh [--with-servers] [piece ...]
#   pieces: moveit handeye sam3_bridge graspgen_bridge task_manager calib_bringup evaluate eye_in_hand
#           calib_bridge apriltag_demo   (default: all) | full (whole pipeline, needs --with-servers)
#   --with-servers   also start the real SAM3/GraspGen servers (needs conda envs, weights and a GPU)
# The workspaces must be built (README, "Setup"). A RealSense camera is optional: without one the
# camera process fails and is reported as a warning only.
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOGDIR="${SMOKE_LOGDIR:-/tmp/fp3_smoke}"; mkdir -p "$LOGDIR"
export ROS_DOMAIN_ID="${SMOKE_ROS_DOMAIN_ID:-79}"     # isolated from any real-robot session
export FP3_ROOT="$ROOT"

set +u
source /opt/ros/jazzy/setup.bash
for ws in franka_ros2_ws franka_demo_ws calib_ws; do
  [ -f "$ROOT/$ws/install/setup.bash" ] && source "$ROOT/$ws/install/setup.bash"
done
set -u

WITH_SERVERS=0; PIECES=()
for a in "$@"; do [ "$a" = "--with-servers" ] && WITH_SERVERS=1 || PIECES+=("$a"); done
[ ${#PIECES[@]} -eq 0 ] && PIECES=(moveit handeye sam3_bridge graspgen_bridge task_manager calib_bringup evaluate eye_in_hand calib_bridge apriltag_demo)
[ $WITH_SERVERS -eq 1 ] && [ ${#PIECES[@]} -eq 10 ] && PIECES+=(full)

RESULTS=()

# run_piece <name> <wait_s> <required regexes, '|' separated, all must match> <must-not-die executables, '|'> -- <command...>
run_piece() {
  local name="$1" wait_s="$2" required="$3" nodie="$4"; shift 5
  local log="$LOGDIR/$name.log" snap="$LOGDIR/$name.snapshot" status="PASS" notes=""
  echo "=== [$name] $*"
  setsid "$@" > "$log" 2>&1 < /dev/null &
  local pgid=$!
  sleep "$wait_s"
  cp "$log" "$snap"
  local clean="$snap.clean" died
  sed -E 's/\x1b\[[0-9;]*m//g' "$snap" > "$clean"          # files, not pipes: logs can be tens of MB
  died=$(grep -E "process has died" "$clean" || true)
  IFS='|' read -ra req <<< "$required"
  for r in "${req[@]}"; do
    [ -z "$r" ] && continue
    grep -qE "$r" "$clean" || { status="FAIL"; notes+="missing: '$r'; "; }
  done
  IFS='|' read -ra nd <<< "$nodie"
  for e in "${nd[@]}"; do
    [ -z "$e" ] && continue
    if grep -q "$e" <<< "$died"; then status="FAIL"; notes+="'$e' died; "; fi
  done
  if grep -q "realsense2_camera_node\|realsense-" <<< "$died"; then notes+="(camera not available: warning only) "; fi
  kill -INT -- "-$pgid" 2>/dev/null
  for _ in $(seq 1 40); do kill -0 -- "-$pgid" 2>/dev/null || break; sleep 1; done
  if kill -0 -- "-$pgid" 2>/dev/null; then kill -TERM -- "-$pgid" 2>/dev/null; sleep 3; kill -KILL -- "-$pgid" 2>/dev/null; notes+="(needed forced stop) "; fi
  pkill -9 -f realsense2_camera_node 2>/dev/null || true
  sleep 2
  echo "    -> $status $notes"
  RESULTS+=("$status|$name|$notes")
}

for p in "${PIECES[@]}"; do
  case "$p" in
    moveit)
      run_piece moveit 35 "pick_place_node ready|command_router_node ready|Scene applied" "move_group|ros2_control_node|pick_place_node|command_router_node" -- \
        ros2 launch fp3_moveit_server bringup.launch.py use_fake_hardware:=true use_rviz:=false ;;
    handeye)
      run_piece handeye 15 "process started" "handeye_tf_publisher" -- \
        ros2 launch handeye_tf_publisher publish.launch.py ;;
    sam3_bridge)
      run_piece sam3_bridge 15 "sam3_bridge_node.*process started|visualize_segmentation_node.*process started" "sam3_bridge_node|visualize_segmentation_node" -- \
        ros2 launch sam3_bridge sam3_bridge.launch.py ;;
    graspgen_bridge)
      run_piece graspgen_bridge 15 "graspgen_bridge_node.*process started|visualize_grasps_node.*process started" "graspgen_bridge_node|visualize_grasps_node" -- \
        ros2 launch graspgen_bridge graspgen_bridge.launch.py ;;
    task_manager)
      run_piece task_manager 20 "pick_task_node.*process started|camera_bridge_node.*process started|command_bridge_node.*process started|create_pointcloud_node.*process started" "pick_task_node|command_bridge_node|camera_buffer_node|create_pointcloud_node" -- \
        ros2 launch robot_task_manager robot_task_manager.launch.py ;;
    calib_bringup)
      run_piece calib_bringup 30 "apriltag_node.*process started|handeye_server.*process started|rqt_calibrator.*process started|sample_guard.*process started" "handeye_server|rqt_calibrator|sample_guard" -- \
        ros2 launch calib_bringup calib_bringup.launch.py use_fake_hardware:=true start_arm_stack:=false ;;
    evaluate)
      run_piece evaluate 30 "apriltag_node.*process started|handeye_tf_publisher.*process started" "handeye_tf_publisher" -- \
        ros2 launch calib_bringup evaluate_calibration.launch.py use_fake_hardware:=true start_arm_stack:=false ;;
    eye_in_hand)
      run_piece eye_in_hand 30 "apriltag_node.*process started|handeye_server.*process started" "handeye_server" -- \
        ros2 launch calib_eye_in_hand calibrate_eye_in_hand.launch.py use_fake_hardware:=true start_arm_stack:=false ;;
    calib_bridge)
      # bridge_calibration_node exits (FATAL) when the D405 .calib does not exist: that is a precondition, not a bug.
      d405=$(ls "$HOME"/.ros2/easy_handeye2/calibrations/fp3_hand_d405*.calib 2>/dev/null | head -1)
      if [ -n "$d405" ]; then
        run_piece calib_bridge 35 "apriltag_node.*process started|bridge_calibration_node.*process started" "apriltag_node|bridge_calibration_node" -- \
          ros2 launch calib_bridge calib_bridge.launch.py use_fake_hardware:=true d405_calibration_name:="$(basename "$d405" .calib)"
      else
        echo "    (no D405 calibration in ~/.ros2/easy_handeye2/calibrations: bridge_calibration_node is expected to stop; checking the rest)"
        run_piece calib_bridge 35 "apriltag_node.*process started|D405 calibration file not found" "apriltag_node" -- \
          ros2 launch calib_bridge calib_bridge.launch.py use_fake_hardware:=true
      fi ;;
    apriltag_demo)
      run_piece apriltag_demo 40 "apriltag_node.*process started|apriltag_move_once_node.*process started|pick_place_node ready" "pick_place_node|move_group" -- \
        ros2 launch fp3_apriltag_demo apriltag_move_once.launch.py use_fake_hardware:=true use_rviz:=false ;;
    full)
      run_piece full 120 "SAM3 is up|GraspGen is up|pick_task_node.*process started|pick_place_node ready" "sam3_bridge_node|graspgen_bridge_node|pick_task_node|pick_place_node|move_group" -- \
        ros2 launch franka_demo_bringup franka_demo.launch.py use_fake_hardware:=true use_rviz:=false ;;
    *) echo "unknown piece: $p" ;;
  esac
done

echo; echo "================ SUMMARY (logs in $LOGDIR) ================"
FAILS=0
for r in "${RESULTS[@]}"; do IFS='|' read -r st nm nt <<< "$r"; printf "%-6s %-16s %s\n" "$st" "$nm" "$nt"; [ "$st" = "FAIL" ] && FAILS=$((FAILS+1)); done
[ $FAILS -eq 0 ] && echo "All checked pieces came up." || echo "$FAILS piece(s) failed."
exit $FAILS
