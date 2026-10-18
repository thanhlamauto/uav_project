#!/usr/bin/env bash
# Called via USB SSH by scripts/run_forest.sh. All ROS/dependencies stay in Docker.
set -euo pipefail
project="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project"
case "${1:-}" in
  build)
    if ! docker image inspect uav-navigation:simulation-build >/dev/null 2>&1; then
      docker build -f simulation/Dockerfile.build -t uav-navigation:simulation-build .
    fi
    docker build -f deploy/Dockerfile.gpu -t uav-navigation:humble-gpu .
    docker build -f simulation/Dockerfile -t uav-navigation:simulation .
    ;;
  start)
    run_id="$2"
    [[ $run_id =~ ^run_[0-9_]+$ ]] || exit 2
    out="$project/state/simulation/$run_id"
    mkdir -p "$out/output"
    docker run -d --name uav-forest --stop-signal SIGINT --runtime nvidia --network host \
      -e ROS_DOMAIN_ID=75 -e NVIDIA_VISIBLE_DEVICES=all \
      -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
      -v /usr/lib/aarch64-linux-gnu/nvidia:/jetson-driver:ro \
      -v /home/nvidia/uav_deploy/depth_fp16.plan:/models/depth_fp16.plan:ro \
      -v "$out/maps/prior.bt:/maps/prior.bt:ro" -v "$out/output:/state" \
      -v "$project/simulation/observe_run.py:/observe_run.py:ro" \
      -v "$project/simulation/visualization:/visualization:ro" \
      --label "uav.run=$run_id" uav-navigation:simulation
    ;;
  ready)
    # Check the actual listener, not an arbitrary startup sleep or stale log line.
    docker exec uav-forest python3 -c \
      'import pathlib; assert any(":1D17 " in line and " 0A " in line for line in pathlib.Path("/proc/net/tcp").read_text().splitlines())' \
      >/dev/null 2>&1
    ;;
  visualize)
    actual="$(docker inspect -f '{{index .Config.Labels "uav.run"}}' uav-forest)"
    [[ $actual == "${2:-}" ]] || { echo 'Container belongs to another run'; exit 2; }
    docker exec -d uav-forest bash -c \
      'source /opt/ros/humble/setup.bash; exec python3 -u /visualization/relay_server.py > /state/visualization_server.log 2>&1'
    ;;
  observe)
    run_id="$2"; shift 2
    actual="$(docker inspect -f '{{index .Config.Labels "uav.run"}}' uav-forest)"
    [[ $actual == "$run_id" ]] || { echo 'Container belongs to another run'; exit 2; }
    docker exec uav-forest bash -c \
      'source /opt/ros/humble/setup.bash; exec python3 -u /observe_run.py "$@"' _ "$@"
    ;;
  stop)
    # Only touch a container carrying the runner label.
    run_id="$(docker inspect -f '{{index .Config.Labels "uav.run"}}' uav-forest 2>/dev/null)" || exit 0
    [[ $run_id =~ ^run_[0-9_]+$ ]] || { echo 'Not owned by this runner'; exit 2; }
    [[ $run_id == "${2:-}" ]] || { echo 'Container belongs to another run'; exit 2; }
    docker stop -t 15 uav-forest
    docker logs uav-forest >"state/simulation/$run_id/runtime.log" 2>&1
    docker rm uav-forest
    ;;
  *) echo 'Internal helper: build | start RUN | ready | visualize RUN | observe RUN SECONDS X Y Z | stop RUN'; exit 2 ;;
esac
