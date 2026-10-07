# RPi Task 1 deployment - 2026-10-07

**Later update:** [speed50 / 5 cm deployment and backups](TASK1_GRID5_TEST.md)
supersedes this earlier four-file install. `task1_controller.py` is now the active
controller, settings overrides are deployed, and route requests allow 180 seconds.

The requested Task 1 source update is installed on `nat@192.168.50.5` in
`/home/nat/SC2079-MDP/rpi`. No controller was running before or during deployment.
The controller was left stopped. No motor commands, camera captures, Bluetooth
connections, STM firmware updates, or tablet installations were performed.

## Installed files

Only these four runtime files were applied:

- `final_week_8_backup.py`
- `communication/android_protocol.py`
- `communication/android.py`
- New `path_plan.py`

The actual controller matched the committed baseline apart from whitespace and
the previously pending CV HTTP-404 handling; that handling is included in the
updated controller. The existing protocol/socket files matched their baseline.
There were no conflicting teammate changes in these targets.

The deployed `task1_main.py`, STM link, manual controller, logger and constants
matched the repository baseline and were preserved. Live `settings.py` was
preserved: Algo/CV address `192.168.50.181`, ports 8000/8001, and the existing
serial-device path. Its environment overrides have not been deployed; do not
assume `LAPTOP_IP=...` will override these live settings.

## Backup and records

Pi backup directory:

`/home/nat/mdp-backups/task1-update-2026-10-07-7d75a8d3`

It contains `before.tar.gz`, the original-file checksum manifest, a staged update
and offline tests, validation records, and the deployment result. A local copy
of the before archive, manifests, tests log and results is under
`output/rpi-deploy-2026-10-07/`. Original file contents and unchanged dependencies
were checked immediately before installation. Each target was replaced using a
temporary file and atomic rename; installed checksums were checked afterward.

## Validation

- All 57 curated offline Pi tests passed on the Pi's Python 3.13.5, using the
  staged runtime and the actual unchanged Pi dependencies with mocked hardware.
- A real route response from the laptop's Algo server passed the new Pi parser:
  8 aligned entries, ending in FIN. This request did not execute a robot route.
- Installed-file checksums, syntax compilation and actual live-module imports
  passed. MAP preparation, matching BEGIN and PLAN READY framing checks passed.
- Installed completion watchdog is 60 seconds.
- Laptop Algo/CV were reachable and healthy from the Pi. Algo reported base
  `original`, recommended speed50, and both overlays `speed50-v2`.
- `rpicam-still` and the configured serial-device path exist. Their existence
  is not a camera-capture or motor-motion test.

## Next use

Use the matching updated Android build to get the Start readiness gate and
target Chat logging. Tablet installation has not been performed or verified by
this deployment. The laptop services were already running; they were not
started or restarted here, and their route response includes the required data.

In a Pi SSH terminal:

```bash
cd ~/SC2079-MDP/rpi
python3 -u task1_main.py
```

Start only one controller parent. Connect the tablet, send a simple map, wait
for Route ready, then press Start when you intend to run the robot. Physical
movement, recognition, tablet alignment and official-time stopping still need
verification. The readiness gaps and assessment workflow remain in
[Task 1 run guide](TASK1_RUN_GUIDE_2026-10-07.md).
