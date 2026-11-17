#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
exec bash tools/ros_env.sh --overlay /usr/bin/python3 /opt/ros/humble/bin/ros2 \
  launch start autoland.launch.py \
  mission_config:="$PWD/src/start/config/mission.yaml" \
  perception_config:="$PWD/src/start/config/perception.yaml" \
  control_config:="$PWD/src/start/config/control.yaml" \
  parameter_config:="$PWD/src/start/config/parameter_manager.yaml" \
  fc_profile:="$PWD/src/start/config/fc_parameters.yaml" "$@"
