#!/usr/bin/env bash
# Run a command with Ubuntu's Python and only the Humble underlay.
# This is a child process: it never changes the calling shell or startup files.
set -euo pipefail
cd "$(dirname "$0")/.."

overlay=false
if [[ ${1:-} == --overlay ]]; then
  overlay=true
  shift
fi
if [[ $# == 0 ]]; then
  printf 'Usage: bash tools/ros_env.sh [--overlay] COMMAND [ARG ...]\n' >&2
  exit 2
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  printf 'ROS 2 Humble is required at /opt/ros/humble.\n' >&2
  exit 1
fi
if [[ $overlay == true && ! -f install_handoff/local_setup.bash ]]; then
  printf 'Run bash tools/build.sh first.\n' >&2
  exit 1
fi

# Keep session/display and ROS transport choices, but discard inherited Python,
# Conda, pyenv, ROS 1, workspace prefixes, compiler and library search paths.
session_env=()
for name in HOME USER LOGNAME LANG LC_ALL LC_CTYPE TERM DISPLAY XAUTHORITY \
  XDG_RUNTIME_DIR ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION \
  ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE \
  CYCLONEDDS_URI ROS_LOG_DIR RCUTILS_COLORIZED_OUTPUT; do
  if [[ -v $name ]]; then
    session_env+=("$name=${!name}")
  fi
done
exec /usr/bin/env -i "${session_env[@]}" \
  PATH=/usr/bin:/bin PYTHONNOUSERSITE=1 \
  COLCON_PYTHON_EXECUTABLE=/usr/bin/python3 \
  /bin/bash --noprofile --norc -c '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    if [[ $1 == true ]]; then
      source install_handoff/local_setup.bash
    fi
    shift
    exec "$@"
  ' ros_env "$overlay" "$@"
