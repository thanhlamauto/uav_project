#!/usr/bin/env bash
set -euo pipefail
project="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project"
[[ "$(uname -m)" = aarch64 ]] || { echo 'Build GPU image on Jetson Orin'; exit 2; }
docker info >/dev/null
cpu_image="${UAV_CPU_IMAGE:-uav-navigation:humble}"
docker build -f deploy/Dockerfile.cpu -t "$cpu_image" .
docker build -f deploy/Dockerfile.gpu --build-arg "CPU_IMAGE=$cpu_image" \
  -t "${UAV_IMAGE:-uav-navigation:humble-gpu}" .
