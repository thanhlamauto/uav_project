#!/usr/bin/env bash
# Owned by run_forest.sh's process group; visualizes only, sends no commands.
set -euo pipefail
project="$1"; run="$2"; shift 2
ros_prefix="${UAV_MAC_ROS_PREFIX:-/opt/miniconda3/envs/ardupilot-rviz}"
set +u
source "$ros_prefix/setup.bash"
set -u
export PATH="$ros_prefix/bin:$PATH"
export ROS_DOMAIN_ID=175 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE QT_PLUGIN_PATH
export QT_QPA_PLATFORM_PLUGIN_PATH="$ros_prefix/plugins"
pids=()
group_signal=0
cleanup() {
  trap - EXIT INT TERM
  # run_forest terminates the whole process group. Do not send a second signal
  # to RViz/rclpy while their first shutdown handler is still running.
  if [[ $group_signal == 0 ]]; then
    for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  fi
  wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'group_signal=1; exit 130' INT TERM
ssh -N -o BatchMode=yes -o ConnectTimeout=8 -o ExitOnForwardFailure=yes \
  -o ServerAliveInterval=5 -o ServerAliveCountMax=2 \
  -L 127.0.0.1:7448:127.0.0.1:7448 nvidia@192.168.55.1 &
pids+=("$!")
"$ros_prefix/bin/python" -u "$project/simulation/visualization/relay_client.py" \
  --geometry "$project/simulation/worlds/forest_geometry.json" \
  --ready "$run/rviz.ready" --goal "$@" >"$run/rviz_relay.log" 2>&1 &
pids+=("$!")
"$ros_prefix/bin/rviz2" -d "$project/simulation/config/forest.rviz" \
  --ros-args -p use_sim_time:=true &
pids+=("$!")
wait "${pids[2]}"
