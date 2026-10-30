#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${AUTOCHAIR_WS:-${HOME}/autochair_ros2_ws}"
SLAM_TARGET="${WORKSPACE}/src/slam_toolbox"
LOCAL_NODES_TARGET="${WORKSPACE}/local_nodes"
BACKUP_ROOT="${WORKSPACE}/backups/scan_gate_$(date +%Y%m%d_%H%M%S)"

if [[ ! -d "${SLAM_TARGET}" ]]; then
  echo "Missing Slam Toolbox target: ${SLAM_TARGET}" >&2
  echo "Set AUTOCHAIR_WS if the ROS2 workspace is elsewhere." >&2
  exit 1
fi

if [[ ! -d "${LOCAL_NODES_TARGET}" ]]; then
  echo "Missing local_nodes target: ${LOCAL_NODES_TARGET}" >&2
  exit 1
fi

FILES=(
  "slam_toolbox/include/slam_toolbox/slam_toolbox_common.hpp"
  "slam_toolbox/src/slam_toolbox_common.cpp"
  "slam_toolbox/lib/karto_sdk/src/Mapper.cpp"
  "slam_toolbox/launch/online_sync_launch.py"
  "slam_toolbox/config/mapper_params_online_sync.yaml"
  "local_nodes/mapping_session.py"
)

mkdir -p "${BACKUP_ROOT}"

for relative_path in "${FILES[@]}"; do
  source_path="${SCRIPT_DIR}/${relative_path}"

  if [[ "${relative_path}" == local_nodes/* ]]; then
    target_path="${WORKSPACE}/${relative_path}"
  else
    target_path="${WORKSPACE}/src/${relative_path}"
  fi

  backup_path="${BACKUP_ROOT}/${relative_path}"

  if [[ ! -f "${source_path}" ]]; then
    echo "Missing repository file: ${source_path}" >&2
    exit 1
  fi

  if [[ -f "${target_path}" ]]; then
    mkdir -p "$(dirname "${backup_path}")"
    cp -p "${target_path}" "${backup_path}"
  fi

  mkdir -p "$(dirname "${target_path}")"
  cp -p "${source_path}" "${target_path}"
  echo "Installed ${target_path}"
done

echo
echo "Backup: ${BACKUP_ROOT}"
echo "Deployment complete. Build Slam Toolbox before running experiments."
