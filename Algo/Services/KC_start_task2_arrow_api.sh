#!/usr/bin/env bash
set -euo pipefail
KC_API_ROOT="$(cd -- "$(dirname -- "$0")/../ImageRecognition/Task2" && pwd)"
exec "${KC_TASK2_PYTHON:-python3}" "$KC_API_ROOT/main.py" "$@"
