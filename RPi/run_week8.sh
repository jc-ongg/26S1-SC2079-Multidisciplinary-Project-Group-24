#!/usr/bin/env bash
# Bring up final_week_8.py with the two exclusive resources it needs.
#
#   videostream/main.py holds the camera via Picamera2, so rpicam-still fails
#   with "Device or resource busy" while it runs.
#   bt_echo.py (or a previous run) holds RFCOMM channel 1, so AndroidLink
#   cannot accept the tablet, or binds channel 2 where the tablet never looks.
set -u
cd "$(dirname "$0")"

# Stop a process the way uvicorn actually wants.
#
# SIGTERM only starts a GRACEFUL shutdown: uvicorn prints "Waiting for
# connections to close" and then waits forever, because the MJPEG generator is
# an infinite loop that never finishes. A second signal forces the quit -- the
# same as pressing Ctrl-C twice. Never SIGKILL the camera holder: killing it
# while it owns the camera wedges the V4L2 driver and only a reboot clears that.
stop_pid() {
    local pid=$1 name=$2 i
    kill -TERM "$pid" 2>/dev/null || return 0
    for i in $(seq 1 5); do
        ps -p "$pid" >/dev/null 2>&1 || { echo "  $name stopped"; return 0; }
        sleep 1
    done
    echo "  $name ignoring SIGTERM -> SIGINT"
    kill -INT "$pid" 2>/dev/null
    for i in $(seq 1 10); do
        ps -p "$pid" >/dev/null 2>&1 || { echo "  $name stopped"; return 0; }
        sleep 1
    done
    echo "  !! $name ($pid) will not die -- NOT sending SIGKILL"
    return 1
}

# Real PIDs by exact script name. A bare `pgrep -f final_week_8` also matches
# the ssh shell running this, which then kills itself.
pids_of() {
    ps -eo pid=,args= | awk -v pat="$1" -v me="$$" \
        '$0 ~ pat && $1 != me && $0 !~ /awk/ {print $1}'
}

echo "== stopping conflicting services =="
for pid in $(pids_of "python3 .*bt_echo\\.py"); do stop_pid "$pid" "bt_echo"; done
# Old runs: no camera involved, and their Manager children ignore SIGTERM, so
# SIGKILL is safe and necessary here.
for pid in $(pids_of "python3 .*final_week_8\\.py"); do
    kill -9 "$pid" 2>/dev/null && echo "  killed old final_week_8 ($pid)"
done
# Anything still holding the serial port is a leftover child of an old run.
fuser -k /dev/ttyACM0 >/dev/null 2>&1 || true
# videostream's cmdline is just "python3 main.py", so find it by the camera.
cam=$(fuser /dev/video0 2>/dev/null | tr -d ' ')
[ -n "$cam" ] && stop_pid "$cam" "videostream"

echo "== preflight =="
for i in $(seq 1 10); do
    fuser /dev/video0 >/dev/null 2>&1 || break
    sleep 1
done
if fuser /dev/video0 >/dev/null 2>&1; then
    echo "  !! camera still held by $(fuser /dev/video0 2>&1) - abort"
    echo "     stop it by hand:  kill -TERM <pid> ; sleep 5 ; kill -INT <pid>"
    exit 1
fi
echo "  camera free"

ls /dev/serial/by-id/ >/dev/null 2>&1 && echo "  STM serial present" || { echo "  !! no STM serial - is the USB-C plugged into the right port?"; exit 1; }

# The Bluetooth device class resets on every boot; without it some tablets do
# not list the Pi. Needs sudo; skipped (with a hint) if it would prompt.
if sudo -n hciconfig hci0 class 0x1f00 2>/dev/null; then
    echo "  bluetooth class set"
else
    echo "  (could not set BT class without a password: sudo hciconfig hci0 class 0x1f00)"
fi

python3 - <<'PY' || exit 1
import requests
from settings import API_IP, API_PORT, API_IP_IMAGE, API_PORT_IMAGE
ok = True
try:
    r = requests.get(f"http://{API_IP}:{API_PORT}/status", timeout=6)
    print(f"  algo {API_IP}:{API_PORT} -> {r.status_code} {r.json()}")
except Exception as e:
    print(f"  !! algo unreachable: {e}"); ok = False
try:
    r = requests.get(f"http://{API_IP_IMAGE}:{API_PORT_IMAGE}/health", timeout=6)
    print(f"  image-api {API_IP_IMAGE}:{API_PORT_IMAGE} -> {r.status_code} {r.json().get('model')}")
except Exception as e:
    print(f"  !! image-api unreachable: {e}"); ok = False
raise SystemExit(0 if ok else 1)
PY

echo
echo "== starting final_week_8.py =="
echo "   It will block on 'Awaiting Bluetooth connection' - connect the tablet,"
echo "   place robot + obstacles, then press Start on the Challenge screen."
echo
exec python3 -u final_week_8.py
