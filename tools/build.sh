#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
: "${ROS_DISTRO:?Source your ROS 2 environment first}"
command -v colcon >/dev/null
colcon --log-base log_handoff build --base-paths src \
  --build-base build_handoff --install-base install_handoff --symlink-install
printf '\nBuild complete. Run: source install_handoff/setup.bash\n'
