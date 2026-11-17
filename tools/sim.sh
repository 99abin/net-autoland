#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
exec bash tools/ros_env.sh --overlay /usr/bin/python3 tools/simulation.py "$@"
