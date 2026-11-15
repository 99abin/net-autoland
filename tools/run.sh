#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
: "${ROS_DISTRO:?Source your ROS 2 environment first}"
if [[ ! -f install_handoff/setup.bash ]]; then
  printf 'Run bash tools/build.sh first.\n' >&2
  exit 1
fi
set +u
source install_handoff/setup.bash
set -u
exec ros2 launch start autoland.launch.py \
  mission_config:="$PWD/src/start/config/mission.yaml" \
  perception_config:="$PWD/src/start/config/perception.yaml" \
  control_config:="$PWD/src/start/config/control.yaml" \
  parameter_config:="$PWD/src/start/config/parameter_manager.yaml" \
  fc_profile:="$PWD/src/start/config/fc_parameters.yaml" "$@"
