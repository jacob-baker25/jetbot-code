#!/usr/bin/env bash
# Python-only diagnostics deployment; no mapper settings or C++ build changes.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${AUTOCHAIR_WS:-${HOME}/autochair_ros2_ws}"
TARGET="${WORKSPACE}/local_nodes"
[[ -d "${TARGET}" ]] || { echo "Missing ${TARGET}" >&2; exit 1; }
FILES=(mapping_session.py matcher_profiles.py capture_sensors.py phidget_probe.py)
for name in "${FILES[@]}"; do
  [[ -f "${SCRIPT_DIR}/local_nodes/${name}" ]] || exit 1
done
mkdir -p "${WORKSPACE}/backups"
BACKUP="$(mktemp -d "${WORKSPACE}/backups/sensor_diagnostics_XXXXXXXX")"
for name in "${FILES[@]}"; do
  if [[ -f "${TARGET}/${name}" ]]; then
    cp -p "${TARGET}/${name}" "${BACKUP}/${name}"
  fi
  cp -p "${SCRIPT_DIR}/local_nodes/${name}" "${TARGET}/${name}"
done
echo "Installed Python diagnostics. Backup: ${BACKUP}"
echo "No colcon build needed. Existing C++ mapper and profiles must already be installed."
