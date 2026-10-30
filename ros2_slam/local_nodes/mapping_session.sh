#!/bin/bash

set -e

source /opt/ros/eloquent/setup.bash

if [ -f "$HOME/autochair_ros2_ws/install/setup.bash" ]; then
    source "$HOME/autochair_ros2_ws/install/setup.bash" || true
fi

cd "$HOME/autochair_ros2_ws/local_nodes"

exec python3 mapping_session.py "$@"
