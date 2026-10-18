#!/usr/bin/env bash
# One local simulator and one USB-connected Orin; run from any directory.
set -euo pipefail
project="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project"
if [[ "${1:-}" == --help ]]; then
  cat <<'HELP'
bash scripts/run_forest.sh [--dry-run]
Defaults: USB nvidia@192.168.55.1, Gazebo GUI + RViz on, goal (20,4,4), 60 seconds.
Overrides: UAV_GUI=0 UAV_RVIZ=0 UAV_DURATION=30 UAV_GOAL_X=12 UAV_GOAL_Y=2 UAV_GOAL_Z=4
Always builds/syncs the current source before takeoff. Ctrl+C requests LAND.
Logs: state/simulation/run_<timestamp>_<pid>/. No real-aircraft support.
HELP
  exit 0
fi
[[ $# == 0 || ( $# == 1 && "$1" == --dry-run ) ]] || { echo 'Use --help'; exit 2; }
export UAV_DURATION="${UAV_DURATION:-60}" UAV_GUI="${UAV_GUI:-1}"
export UAV_RVIZ="${UAV_RVIZ:-1}"
export UAV_GOAL_X="${UAV_GOAL_X:-20}" UAV_GOAL_Y="${UAV_GOAL_Y:-4}" UAV_GOAL_Z="${UAV_GOAL_Z:-4}"
python="${UAV_PYTHON:-/opt/miniconda3/envs/ardupilot-rviz/bin/python}"
orin=nvidia@192.168.55.1
remote=/home/nvidia/uav_project
ssh_opts=(-o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=2)
if [[ "${1:-}" == --dry-run ]]; then
  cat <<'PLAN'
Mac: preflight -> CMake simulation + tools/octomap -> full prior map
Mac -> USB SSH/rsync -> Orin: build Docker GPU + simulation images
Mac: gz server + GUI -> ArduPilot SITL -> sitl_control.py takeoff (HOVER READY)
Orin: orin_runtime.sh start -> Docker -> forest.launch.py -> app.launch.py
Mac: gazebo_sender -> USB TCP/JPEG -> Orin gazebo_receiver
Optional RViz on Mac: Orin topic subscriber -> SSH/USB tunnel -> isolated local ROS domain 175
Orin: observe_run.py -> wait for fresh inputs + GUIDED/armed -> publish goal
Finally: sitl_control.py land -> Docker stop -> collect logs -> stop owned processes
PLAN
  exit 0
fi
[[ "$(uname -s)" == Darwin ]] || { echo 'Run this entry point on the Mac.'; exit 2; }
"$python" - <<'PY'
import math, os
values = [float(os.environ[n]) for n in ('UAV_DURATION','UAV_GOAL_X','UAV_GOAL_Y','UAV_GOAL_Z')]
assert all(math.isfinite(v) for v in values), 'Arguments must be finite'
duration, x, y, z = values
assert 5 <= duration <= 600, 'Duration must be 5..600 s'
assert -4 < x < 26 and -12 < y < 12 and 1 < z < 9, 'Goal outside configured map bounds'
assert os.environ['UAV_GUI'] in ('0','1'), 'UAV_GUI must be 0 or 1'
assert os.environ['UAV_RVIZ'] in ('0','1'), 'UAV_RVIZ must be 0 or 1'
import pymavlink
PY
for tool in cmake rsync ssh lsof; do command -v "$tool" >/dev/null; done
if [[ $UAV_RVIZ == 1 ]]; then
  ros_prefix="${UAV_MAC_ROS_PREFIX:-/opt/miniconda3/envs/ardupilot-rviz}"
  [[ -x "$ros_prefix/bin/rviz2" ]] || { echo 'RViz missing; set UAV_MAC_ROS_PREFIX or UAV_RVIZ=0.'; exit 2; }
  "$ros_prefix/bin/python" -c 'import rclpy, visualization_msgs.msg, tf2_msgs.msg'
  if lsof -nP -iTCP:7448 -sTCP:LISTEN >/dev/null 2>&1; then
    echo 'RViz tunnel port 7448 already in use. Stop the earlier viewer first.'; exit 2
  fi
fi
[[ -x /opt/homebrew/bin/gz && -x state/simulation/bin/arducopter ]] || {
  echo 'Missing Gazebo/SITL dependency; see simulation/README.md section 1.'; exit 2;
}
if lsof -nP -iTCP:5760 -iTCP:5762 -iUDP:9002 >/dev/null 2>&1; then
  echo 'An existing simulator owns port 5760/5762/9002. Stop that run first.'; exit 2
fi
# Fixed USB destination: do not silently fall back to Wi-Fi/Tailscale.
ssh "${ssh_opts[@]}" "$orin" 'test "$(uname -m)" = aarch64 && docker info >/dev/null &&
  test -r /home/nvidia/uav_deploy/depth_fp16.plan &&
  test -d /usr/lib/aarch64-linux-gnu/nvidia &&
  docker image inspect uav-navigation:humble >/dev/null &&
  ! docker container inspect uav-forest >/dev/null 2>&1' || {
  echo 'USB/SSH/Docker/base image/engine unavailable, or uav-forest already exists.'; exit 2;
}
run_id="run_$(date +%Y%m%d_%H%M%S)_$$"
run="$project/state/simulation/$run_id"
mkdir -p "$run"
printf 'Logs: %s\n' "$run"
pids=()
sitl_started=0
orin_started=0
takeoff_pid=''
build_pid=''
start() {
  local name="$1"; shift
  "$@" >"$run/$name.log" 2>&1 &
  pids[${#pids[@]}]=$!
}
remote_task() {
  ssh "${ssh_opts[@]}" "$orin" bash "$remote/simulation/orin_runtime.sh" "$@"
}
cleanup() {
  local status=$?
  trap - EXIT INT TERM
  set +e
  # Stop an interrupted preparation command before requesting LAND.
  [[ -z $takeoff_pid ]] || kill -TERM -- "-$takeoff_pid" 2>/dev/null
  [[ -z $build_pid ]] || kill -TERM -- "-$build_pid" 2>/dev/null
  if [[ $sitl_started == 1 ]]; then
    echo 'Requesting LAND; waiting for disarm...'
    "$python" simulation/sitl_control.py land >"$run/land.log" 2>&1
    if [[ $? != 0 ]]; then
      echo "LAND not confirmed. Leaving Gazebo/SITL/bridge alive; see $run/land.log"
      echo 'Recover with: python simulation/sitl_control.py land (use the configured Python).'
      [[ $orin_started == 1 ]] && remote_task stop "$run_id" >>"$run/orin_stop.log" 2>&1
      exit 1
    fi
  fi
  if [[ $orin_started == 1 ]]; then
    remote_task stop "$run_id" >"$run/orin_stop.log" 2>&1 || {
      echo "Could not confirm Orin container stopped; see $run/orin_stop.log"; status=1;
    }
    rsync -az -e 'ssh -o BatchMode=yes -o ConnectTimeout=8' \
      "$orin:$remote/state/simulation/$run_id/" "$run/orin/" || status=1
  fi
  # Job control gives each owned background job a separate process group.
  if [[ ${#pids[@]} -gt 0 ]]; then
    for pid in "${pids[@]}"; do kill -TERM -- "-$pid" 2>/dev/null; done
    sleep 2
    for pid in "${pids[@]}"; do kill -KILL -- "-$pid" 2>/dev/null; done
    for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null; done
  fi
  echo "Stopped. Logs: $run"
  exit "$status"
}
trap cleanup EXIT
trap 'echo "Failed near line $LINENO. Inspect the component logs in $run." >&2' ERR
trap 'exit 130' INT
trap 'exit 143' TERM
set -m
echo '[1/7] Build Mac simulator and prior map...'
cmake -S simulation -B build/simulation -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/opt/homebrew/opt/qt@5 >"$run/build_mac.log" 2>&1
cmake --build build/simulation -j4 >>"$run/build_mac.log" 2>&1
ctest --test-dir build/simulation --output-on-failure >>"$run/build_mac.log" 2>&1
bash scripts/prepare_octomap.sh --world simulation/worlds/forest.sdf \
  --output "$run/maps" --min -4 -12 1 --max 26 12 9 \
  --resolution 0.5 --hidden-fraction 0 --seed 7 >"$run/map.log" 2>&1
echo '[2/7] Sync source over USB; rebuild Orin images (cached, before flight)...'
ssh "${ssh_opts[@]}" "$orin" mkdir -p "$remote/state/simulation/$run_id/maps"
rsync -az --exclude build --exclude install --exclude log --exclude state \
  --exclude weights --exclude .git ./ "$orin:$remote/" >"$run/sync.log" 2>&1
rsync -az "$run/maps/prior.bt" "$orin:$remote/state/simulation/$run_id/maps/"
remote_task build >"$run/build_orin.log" 2>&1 &
build_pid=$!
while kill -0 "$build_pid" 2>/dev/null; do
  echo "  Building on Orin; follow: tail -f '$run/build_orin.log'"
  sleep 10
done
wait "$build_pid"
build_pid=''
echo '[3/7] Start Gazebo and SITL...'
export GZ_PARTITION="uav_$run_id"
export GZ_SIM_RESOURCE_PATH="$project/simulation/models:$project/simulation/worlds"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$project/build/simulation/plugins"
start gazebo /opt/homebrew/bin/gz sim -s -r -v2 simulation/worlds/forest.sdf
[[ $UAV_GUI == 0 ]] || start gui /opt/homebrew/bin/gz sim -g -v2
mkdir -p "$run/sitl"
start sitl bash -c 'cd "$1"; exec "$2" -w --model JSON --speedup 1 --slave 0 \
  --serial1=tcp:2 --defaults "$3" --sim-address=127.0.0.1 -I0 \
  --home=-35.363262,149.165237,584,0' _ "$run/sitl" \
  "$project/state/simulation/bin/arducopter" "$project/simulation/config/sitl.parm"
sitl_started=1
for attempt in {1..40}; do
  lsof -nP -iTCP:5760 -sTCP:LISTEN >/dev/null 2>&1 && break
  sleep 1
done
echo '[4/7] Normal arm/takeoff; wait for HOVER READY...'
"$python" simulation/sitl_control.py takeoff --altitude 3 >"$run/takeoff.log" 2>&1 &
takeoff_pid=$!
while kill -0 "$takeoff_pid" 2>/dev/null; do
  echo "  Waiting for hover; follow: tail -f '$run/takeoff.log'"
  sleep 5
done
wait "$takeoff_pid"
takeoff_pid=''
echo '[5/7] Start Orin GPU pipeline...'
orin_started=1
remote_task start "$run_id" >"$run/orin_start.log" 2>&1
for attempt in {1..30}; do
  if remote_task ready; then break; fi
  [[ $attempt != 30 ]] || { echo 'Receiver did not listen; inspect Orin logs.'; exit 1; }
  sleep 1
done
echo '[6/7] Stream camera/state/clock over USB...'
start sender build/simulation/gazebo_sender 192.168.55.1 7447
if [[ $UAV_RVIZ == 1 ]]; then
  echo 'Start RViz on Mac: A* / MPPI / cloud / actual flight / live stop reason...'
  remote_task visualize "$run_id"
  start rviz bash simulation/visualization/start_mac.sh "$project" "$run" \
    "$UAV_GOAL_X" "$UAV_GOAL_Y" "$UAV_GOAL_Z"
  rviz_pid="${pids[${#pids[@]}-1]}"
  for attempt in {1..25}; do
    kill -0 "$rviz_pid" 2>/dev/null || { echo "RViz exited; see $run/rviz.log and rviz_relay.log"; exit 1; }
    [[ -f "$run/rviz.ready" ]] && break
    [[ $attempt != 25 ]] || { echo "Visualization stream unavailable; see $run/rviz_relay.log"; exit 1; }
    sleep 1
  done
fi
echo "[7/7] Wait for fresh ROS inputs; goal ($UAV_GOAL_X,$UAV_GOAL_Y,$UAV_GOAL_Z), ${UAV_DURATION}s..."
remote_task observe "$run_id" "$UAV_DURATION" "$UAV_GOAL_X" "$UAV_GOAL_Y" "$UAV_GOAL_Z" \
  | tee "$run/observe.log"
# EXIT trap lands, stops the container, retrieves map/logs and stops owned jobs.
