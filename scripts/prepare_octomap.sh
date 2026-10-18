#!/usr/bin/env bash
# Offline C++ tool; does not source ROS or start Gazebo.
set -euo pipefail
project="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build="$project/build/octomap-tools"
if [[ "$(uname -s)" = Darwin ]]; then
  export PKG_CONFIG_PATH="/opt/homebrew/lib/pkgconfig:/opt/homebrew/share/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
fi
cmake -S "$project/tools/octomap" -B "$build" \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build "$build" -j "${UAV_BUILD_JOBS:-4}"
exec "$build/build_octomap_world" "$@"
