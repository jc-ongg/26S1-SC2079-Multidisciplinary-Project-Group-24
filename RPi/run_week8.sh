#!/usr/bin/env bash
# Bring up final_week_8.py with the two exclusive resources it needs.
#
#   videostream/main.py holds the camera via Picamera2, so rpicam-still fails
#   with "Device or resource busy" while it runs.
#   bt_echo.py holds RFCOMM channel 1, so AndroidLink cannot accept the tablet.
set -u
cd ~/SC2079-MDP/rpi

# Stop a process the way uvicorn actually wants.
#
# SIGTERM only starts a GRACEFUL shutdown: uvicorn prints "Waiting for
# connections to close" and then waits forever, because the MJPEG generator is
# an infinite loop that never finishes. A second signal forces the quit -- the
# same as pressing Ctrl-C twice. Never SIGKILL: killing it while it owns the
# camera wedges the V4L2 driver ("Operation already in progress") and only a
# reboot clears that.
stop_pid() {
    local pid=$1 name=$2 i
    kill -TERM "$pid" 2>/dev/null || return 0
    for i in $(seq 1 5); do
        ps -p "$pid" >/dev/null 2>&1 || { echo "  $name stopped"; return 0; }
        sleep 1
    done
    echo "  $name ignoring SIGTERM (graceful shutdown stalls) -> SIGINT"
    kill -INT "$pid" 2>/dev/null
    for i in $(seq 1 10); do
        ps -p "$pid" >/dev/null 2>&1 || { echo "  $name stopped"; return 0; }
        sleep 1
    done
    echo "  !! $name ($pid) will not die -- NOT sending SIGKILL (wedges the camera)"
    return 1
}

echo "== stopping conflicting services =="
# bt_echo by name. Exclude this script's own PID: 'bt_echo' appears in our
# command line too, so a bare pgrep -f would match us and we would kill ourselves.
for pid in $(pgrep -f "bt_echo.py" | grep -v "^$$\$"); do
    stop_pid "$pid" "bt_echo"
done
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

ls /dev/serial/by-id/ >/dev/null 2>&1 && echo "  STM serial present" || { echo "  !! no STM serial"; exit 1; }

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
echo "   place robot + obstacles, then press BEGIN."
echo
exec python3 -u final_week_8.py
