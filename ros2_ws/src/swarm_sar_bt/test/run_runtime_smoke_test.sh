#!/bin/bash
# Runtime smoke test for swarm_sar_bt: 3 robots, no Gazebo, each robot's
# real Nav2 stack replaced by fake_navigate_to_pose_server.py (a stub
# navigate_to_pose action server) so mission_bt_node - the real package
# code, unmodified - runs against something that answers like a real
# bt_navigator. Exercises the actual BT plugins and the decentralized
# SwarmKnowledge coordination (/swarm/claims, /swarm/rescues) over real
# ROS2 topics between 3 separate node graphs.
#
# Run inside the built image, with this file's directory bind-mounted over
# the baked-in copy so edits here don't require a rebuild:
#   docker run --rm \
#     -v "$(pwd)/ros2_ws/src/swarm_sar_bt/test:/opt/overlay_ws/src/swarm_sar_bt/test:ro" \
#     swarm_sar_bt_dev bash -c \
#     "source /opt/overlay_ws/install/setup.bash && bash /opt/overlay_ws/src/swarm_sar_bt/test/run_runtime_smoke_test.sh"
set -uo pipefail

RUN_SECONDS=${RUN_SECONDS:-45}
FAIL_ROBOT_AT=${FAIL_ROBOT_AT:-15}   # seconds; kill robot1's stub nav server to test reclaim
LOG_DIR=$(mktemp -d)
echo "Logs: $LOG_DIR"

BT_XML=/opt/overlay_ws/install/swarm_sar_bt/share/swarm_sar_bt/bt_xml/sar_mission_tree.xml
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

PIDS=()
cleanup() {
  echo "--- shutting down ---"
  for pid in "${PIDS[@]}"; do
    kill "$pid" >/dev/null 2>&1
  done
  wait >/dev/null 2>&1
}
trap cleanup EXIT

start_robot() {
  local ns=$1 x0=$2 y0=$3
  python3 "$SCRIPT_DIR/fake_navigate_to_pose_server.py" \
    --ros-args -r __ns:="/$ns" -p x0:="$x0" -p y0:="$y0" -p speed:=2.0 \
    > "$LOG_DIR/${ns}_navserver.log" 2>&1 &
  PIDS+=("$!")
  echo "$!" > "$LOG_DIR/${ns}_navserver.pid"

  sleep 1  # give the action server a moment to come up before the BT node needs it

  ros2 run swarm_sar_bt mission_bt_node \
    --ros-args -r __ns:="/$ns" \
    -p robot_id:="${ROBOT_ID}" \
    -p bt_xml_filename:="$BT_XML" \
    -p "plugin_lib_names:=[swarm_sar_bt_nodes,nav2_navigate_to_pose_action_bt_node]" \
    -p global_frame:=map -p robot_base_frame:=base_link \
    -p bt_loop_duration_ms:=200 -p server_timeout_ms:=20000 -p wait_for_service_timeout_ms:=10000 \
    > "$LOG_DIR/${ns}_mission.log" 2>&1 &
  PIDS+=("$!")
}

echo "--- starting victim_ground_truth_node ---"
ros2 run swarm_sar_bt victim_ground_truth_node \
  --ros-args \
  -p "victim_ids:=[0,1,2,3]" \
  -p "victim_x:=[6.0,-6.0,5.0,-5.0]" \
  -p "victim_y:=[6.0,6.0,-5.0,-5.0]" \
  -p publish_rate_hz:=2.0 \
  > "$LOG_DIR/victims.log" 2>&1 &
PIDS+=("$!")

echo "--- starting robot0 (0,0) ---"
ROBOT_ID=0 start_robot robot0 0.0 0.0
echo "--- starting robot1 (-8,8) ---"
ROBOT_ID=1 start_robot robot1 -8.0 8.0
echo "--- starting robot2 (8,-8) ---"
ROBOT_ID=2 start_robot robot2 8.0 -8.0

echo "--- logging /swarm/claims and /swarm/rescues ---"
ros2 topic echo /swarm/claims > "$LOG_DIR/claims.log" 2>&1 &
PIDS+=("$!")
ros2 topic echo /swarm/rescues > "$LOG_DIR/rescues.log" 2>&1 &
PIDS+=("$!")

sleep "$FAIL_ROBOT_AT"
echo "--- t=${FAIL_ROBOT_AT}s: killing robot1's stub nav server (simulated failure) ---"
kill "$(cat "$LOG_DIR/robot1_navserver.pid")" >/dev/null 2>&1

sleep "$((RUN_SECONDS - FAIL_ROBOT_AT))"

echo
echo "=== claims.log ==="
cat "$LOG_DIR/claims.log"
echo "=== rescues.log ==="
cat "$LOG_DIR/rescues.log"
echo "=== mission_bt_node stderr tails (errors only) ==="
for ns in robot0 robot1 robot2; do
  echo "--- $ns ---"
  grep -iE "error|fatal|exception" "$LOG_DIR/${ns}_mission.log" || echo "(none)"
done
echo
echo "Full logs kept in $LOG_DIR (this shell only - copy out before the container exits if you need them)"
