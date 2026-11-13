#!/usr/bin/env bash
# Deploy the ROS handedness/metadata repair for the Python RPLIDAR bridge.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${AUTOCHAIR_WS:-${HOME}/autochair_ros2_ws}"
TARGET="${WORKSPACE}/local_nodes"
FILES=(lidar_scan_geometry.py rplidar_python_bridge.py mapping_session.py)

[[ -d "${TARGET}" ]] || { echo "Missing ${TARGET}" >&2; exit 1; }
for name in "${FILES[@]}"; do
  [[ -f "${SCRIPT_DIR}/local_nodes/${name}" ]] || {
    echo "Missing repository file: ${SCRIPT_DIR}/local_nodes/${name}" >&2
    exit 1
  }
done

mkdir -p "${WORKSPACE}/backups"
BACKUP="$(mktemp -d "${WORKSPACE}/backups/lidar_geometry_XXXXXXXX")"
for name in "${FILES[@]}"; do
  if [[ -f "${TARGET}/${name}" ]]; then
    cp -p "${TARGET}/${name}" "${BACKUP}/${name}"
  fi
  cp -p "${SCRIPT_DIR}/local_nodes/${name}" "${TARGET}/${name}"
  echo "Installed ${TARGET}/${name}"
done

echo
echo "Backup: ${BACKUP}"
echo "Deployment complete. No colcon build is needed."
echo "This changes lidar angle handedness only."
echo "Legacy Karto-compatible LaserScan count metadata is retained."
