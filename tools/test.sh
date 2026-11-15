#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
python3 -c 'import cv2, numpy, yaml' || {
  printf 'Install test dependencies first: python3 -m pip install -r requirements-test.txt\n'
  exit 1
}
python3 -B -m unittest discover -s tests -v
