#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ $# != 0 ]]; then
  printf 'Usage: bash tools/build.sh\n' >&2
  exit 2
fi
bash tools/ros_env.sh /bin/bash --noprofile --norc -s <<'BUILD'
set -euo pipefail
/usr/bin/python3 -c 'import em, catkin_pkg, setuptools, colcon_core, rosidl_adapter' || {
  printf 'Missing system build dependency. Check python3-empy (imports em), python3-catkin-pkg, python3-setuptools, python3-colcon-common-extensions and ros-humble-rosidl-default-generators. Do not pip install em.\n' >&2
  exit 1
}
# colcon uses its own sys.executable to install ament_python packages.
# CMake selects Python independently; reset its cache and pin that interpreter too.
/usr/bin/python3 /usr/bin/colcon --log-base log_handoff build --base-paths src \
  --build-base build_handoff --install-base install_handoff --symlink-install \
  --cmake-clean-cache --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
BUILD
printf '\nBuild complete. In a clean Humble shell: source install_handoff/setup.bash\n'
