#!/usr/bin/env bash
# Compatibility launcher. Never kills other processes or runs the sensor demo.
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
exec python3 -u task1_main.py "$@"
