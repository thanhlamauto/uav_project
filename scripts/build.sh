#!/usr/bin/env bash
set -euo pipefail
project="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project"
case "${1:-}" in
  --core-only)
    cmake -S src/uav_navigation_core -B build/core-standalone \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_DISABLE_FIND_PACKAGE_ament_cmake=TRUE \
      -DCMAKE_INSTALL_PREFIX="$project/install/core-standalone" -DBUILD_TESTING=ON
    cmake --build build/core-standalone -j "${UAV_BUILD_JOBS:-4}"
    ctest --test-dir build/core-standalone --output-on-failure
    cmake --install build/core-standalone
    exit 0
    ;;
  '') ;;
  *) echo 'Usage: bash scripts/build.sh [--core-only]'; exit 2 ;;
esac
if [[ "$(uname -s)" = Darwin ]]; then
  ros_prefix="${UAV_MAC_ROS_PREFIX:-/opt/miniconda3/envs/ardupilot-rviz}"
  opencv_dir="${OpenCV_DIR:-/opt/homebrew/lib/cmake/opencv5}"
else
  ros_prefix="${UAV_ROS_PREFIX:-/opt/ros/humble}"
  opencv_dir="${OpenCV_DIR:-/opt/opencv/lib/cmake/opencv5}"
fi
set +u
source "$ros_prefix/setup.bash"
set -u
export PATH="$ros_prefix/bin:$PATH"
if [[ "$(uname -s)" = Darwin ]]; then
  export PKG_CONFIG_PATH="/opt/homebrew/lib/pkgconfig:/opt/homebrew/share/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
fi
colcon_bin="${UAV_COLCON_BIN:-$(command -v colcon || true)}"
if [[ -z "$colcon_bin" ]]; then
  colcon_bin="${XDG_CACHE_HOME:-$HOME/.cache}/uav_project/colcon/bin/colcon"
fi
[[ -x "$colcon_bin" ]] || {
  echo 'colcon not found. See README.md or set UAV_COLCON_BIN.'; exit 2;
}
"$colcon_bin" build --base-paths src \
  --packages-select uav_navigation_core uav_navigation_ros uav_navigation_bringup \
  --parallel-workers "${UAV_BUILD_JOBS:-4}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
    -DOpenCV_DIR="$opencv_dir"
"$colcon_bin" test --packages-select \
  uav_navigation_core uav_navigation_ros uav_navigation_bringup \
  --event-handlers console_direct+
"$colcon_bin" test-result --verbose
