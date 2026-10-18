#!/usr/bin/env bash
# Camera driver publishes rectified ROS images/CameraInfo on the host network.
set -euo pipefail
project="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
image="${UAV_IMAGE:-uav-navigation:humble-gpu}"
engine="${UAV_DEPTH_ENGINE:-}"
prior="${UAV_MAP_FILE:-}"
params="${UAV_PARAMS:-$project/src/uav_navigation_bringup/config/runtime.yaml}"
state="${UAV_STATE_DIR:-$project/state}"
driver_dir="${UAV_JETSON_DRIVER_DIR:-/usr/lib/aarch64-linux-gnu/nvidia}"
fcu_url="${UAV_FCU_URL:-/dev/ttyACM0:57600}"
start_mavros="${UAV_START_MAVROS:-true}"
localization="${UAV_LOCALIZATION_SOURCE:-gazebo}"
[[ "$localization" = gazebo || "$localization" = mavros ]] || {
  echo 'UAV_LOCALIZATION_SOURCE must be gazebo or mavros'; exit 2;
}
default_sim_time=true
[[ "$localization" = gazebo ]] || default_sim_time=false
sim_time="${UAV_USE_SIM_TIME:-$default_sim_time}"
[[ "$sim_time" = true || "$sim_time" = false ]] || exit 2
for file in "$engine" "$prior" "$params"; do
  [[ "$file" = /* && -r "$file" ]] || {
    echo 'Set absolute UAV_DEPTH_ENGINE (.plan), UAV_MAP_FILE (.bt), and UAV_PARAMS (.yaml)'; exit 2;
  }
done
[[ "$start_mavros" = true || "$start_mavros" = false ]] || exit 2
mkdir -p "$state"
[[ -r "$driver_dir/libnvdla_compiler.so" ]] || {
  echo 'Missing Jetson driver libraries; set UAV_JETSON_DRIVER_DIR'; exit 2;
}
state="$(cd "$state" && pwd)"
devices=(); groups=()
if [[ "$start_mavros" = true && "$fcu_url" = /dev/* ]]; then
  device="${fcu_url%%:*}"
  [[ -c "$device" ]] || { echo "Missing FCU device: $device"; exit 2; }
  devices=(--device "$device")
  groups+=(--group-add "$(stat -c '%g' "$device")")
fi
for group in $(id -G); do groups+=(--group-add "$group"); done
exec docker run --rm --init --name uav-navigation --runtime nvidia \
  --network host --ipc host --read-only --tmpfs /tmp:rw,nosuid,nodev \
  --cap-drop ALL --security-opt no-new-privileges --user "$(id -u):$(id -g)" \
  "${groups[@]}" "${devices[@]}" \
  --env HOME=/tmp --env ROS_LOG_DIR=/state/ros_logs \
  --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-75}" \
  --env NVIDIA_VISIBLE_DEVICES=all --env NVIDIA_DRIVER_CAPABILITIES=compute,utility \
  --env LD_LIBRARY_PATH=/jetson-driver:/opt/tensorrt/lib:/opt/cuda/lib:/opt/opencv/lib \
  --mount "type=bind,src=$driver_dir,dst=/jetson-driver,readonly" \
  --mount "type=bind,src=$engine,dst=/models/depth_fp16.plan,readonly" \
  --mount "type=bind,src=$prior,dst=/maps/prior.bt,readonly" \
  --mount "type=bind,src=$params,dst=/params/runtime.yaml,readonly" \
  --mount "type=bind,src=$state,dst=/state" \
  "$image" ros2 launch uav_navigation_bringup app.launch.xml \
  params_file:=/params/runtime.yaml fcu_url:="$fcu_url" start_mavros:="$start_mavros" \
  localization_source:="$localization" use_sim_time:="$sim_time"
