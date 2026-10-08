# Historical session notes

These notes include superseded configurations and experimental commands.
Use the [current startup guide](../README.md), not these notes, as the runbook.
In particular, Task 1 is task1_main.py -> final_week_8_backup.py; do not launch
or kill hardware processes merely because an old note describes doing so.

- [2026-09-11] MDP Pi (Trixie, Python 3.13): pybluez is impossible (distutils removed, no libbluetooth headers, no internet in hotspot mode) - use stdlib socket.AF_BLUETOOTH + 'sdptool add --channel=1 SP' with bluetoothd --compat instead.
- [2026-09-11] Pi system python has no pip and PEP 668 blocks installs; pure-python wheels can be installed by unzipping into ~/.local/lib/python3.13/site-packages/.
- [2026-09-11] Mac roams off the Pi hotspot back to campus WiFi mid-task and silently kills SSH - run long Pi scripts detached with setsid nohup and poll the log.
- [2026-09-17] Pi BT device class 0x1f00 does not persist across reboot (reverts to 0x400000); adapter alias MDPGrp24 does persist. Re-set class each boot.
- [2026-09-17] Interactive Pi scripts like bt_echo.py can still be run detached: mkfifo /tmp/btsend, feed it as stdin, and hold the write end open with a background `sleep 86400 > /tmp/btsend`. Send with `echo msg > /tmp/btsend`, read replies from the log.
- [2026-09-17] `setsid nohup ... &` over ssh still hangs the ssh client until the channel FDs close - wrap the remote command with `timeout`/`exit 0` and verify from a second connection instead of waiting.
- [2026-09-17] `setsid` does not exist on macOS - use plain `nohup cmd > log 2>&1 &` for Mac-side services (image-api, viewer).
- [2026-09-17] macOS 26 Local Network Privacy blocks Chrome from LAN IPs (ERR_ADDRESS_UNREACHABLE on 192.168.50.5:5000) while curl from Terminal works. Fix: Privacy & Security -> Local Network -> enable Chrome + full ⌘Q restart. Better: use the viewer on localhost:5173, which proxies to the Pi server-side in Node and is exempt.
- [2026-09-17] When the Pi hotspot "disappears" and networksetup says "Could not find network MDP-group-24", the Pi has usually rebooted (check `uptime`/`who -b`) - /tmp is wiped so videostream+bt_echo logs vanish and both must be restarted. Toggling `networksetup -setairportpower en0 off/on` makes the Mac auto-rejoin.
- [2026-09-17] image-api returning {"image_id":"NA"} means the model ran fine but saw no symbol; the viewer normalises NA to an empty array and shows "Nothing detected yet". Detection is manual - click "Single shot"/"Start live detection".
- [2026-09-17] The STM runs MDP_test.hex, NOT robot/mdp-grp-16 source. Protocol is `<id>/<COMPONENT>/<CMD>/<p1>/<p2>;` terminated by ';' (newline is silently ignored), reply `!<id>/DONE;`. rpi/communication/stm32.py sends `FW010` + readline() and is completely incompatible - needs rewriting.
- [2026-09-17] Decode unknown STM firmware by parsing the .hex to binary and dumping its string table - sscanf format strings and error literals reveal the whole wire protocol without a debugger.
- [2026-09-17] "Port opens + CTS asserted" proves only that the USB-serial bridge has power; it says nothing about the MCU. Don't conclude wiring is wrong from silence alone - a protocol/framing mismatch looks identical.
- [2026-09-17] Algo server runs on the Mac (Pi too slow for YOLO) and serves BOTH /path and /infer, making the separate image-api redundant. /infer needs multipart field name "file"; "image" returns 400.
- [2026-09-17] Algo stm_commands are "<steering>,<angle>,<direction>,<distance>" (e.g. "center,0,forward,120"), NOT FW010. final_week_8's stm32_prefixes tuple must include center/left/right or every command raises "Unknown command".
- [2026-09-17] Never kill -9 a Picamera2 process while it holds the sensor - it wedges the V4L2 driver ("Failed to queue buffer: Operation already in progress") and modprobe -r cannot clear it (module is bcm2835_unicam_legacy); only a reboot works. Plain kill lets it release cleanly.
- [2026-09-17] pkill -f "<pattern>" over ssh kills the remote bash -c shell too, because the pattern appears in that shell's own command line - ssh then exits 255. Kill by PID instead.
- [2026-09-17] Moving a service to a different port can silently change BEHAVIOUR, not just plumbing: giving :8000 to the Algo server swapped image recognition from best_v11i_m.onnx to best.pt - different networks, different label sets, worse accuracy on arrows/digits. Run both on separate ports instead of evicting one.
- [2026-09-17] The two MDP models are not interchangeable: onnx is YOLOv8m-SEG (needs task='segment' or ultralytics returns junk class ids and conf>1.0), has CIRCLE but NO 'stop' class; best.pt is a detect model whose class names are already the mapped target ids. Check the assessment image list against the label set before picking.
- [2026-09-17] Load optional ML weights lazily, not at import: the Algo server did `yolo_model = YOLO(path)` at module level, so deleting an unused model would have killed path planning too.
- [2026-10-01] The tablet L/R buttons send FL00/FR00, which after the switch to parameterised TURNL/TURNR became "turn 0 degrees". Legacy turns with no angle now default to 90.
- [2026-10-01] To make another laptop reproduce a working run, pin from `pip freeze` of the venv that actually ran (the requirements.txt was all >=) and replay recorded inputs from that run (golden/ + laptop_verify.py); a fresh clone + install then matched 7/7.
- [2026-10-08] Most of the ~5 s Task 1 obstacle stop was rpicam-still cold-starting the sensor (2-3 s, doubled on a retry); keep one Picamera2 open in the process that runs snap_and_rec and grab a frame per SNAP instead. Also: a model "removed" only on one laptop comes back on the next teammate push - delete it from the repo.
