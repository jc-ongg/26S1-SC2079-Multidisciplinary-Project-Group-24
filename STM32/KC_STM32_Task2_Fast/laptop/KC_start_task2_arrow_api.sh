#!/bin/bash
set -euo pipefail
KC_API_ROOT="$(cd -- "$(dirname -- "$0")" && pwd)"
KC_DEFAULT_PYTHON="/Users/kiachun/Downloads/26S1-SC2079-Multidisciplinary-Project-Group-24-rpi-task1-services-and-fixes/RPi-services/image-api/venv/bin/python"
exec "${KC_TASK2_PYTHON:-$KC_DEFAULT_PYTHON}" "$KC_API_ROOT/KC_task2_arrow_api/main.py" "$@"
