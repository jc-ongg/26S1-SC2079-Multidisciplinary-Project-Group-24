# Physical Pi code and Task 1

Core source was recovered from `/home/nat/SC2079-MDP/rpi` on 2026-10-01.
This replaces a reconstructed branch copy where it conflicted with the Pi.
The Pi is not a Git checkout; pushing this repository does not deploy it.
The user no longer has access to the Pi as of 2026-10-02. Live state and any
later changes are unknown; this repository is a source snapshot with local fixes.

## Entry points

- `python3 -u task1_main.py`: Task 1; imports `final_week_8_backup.RaspberryPi`.
- `python3 -u task1_main.py --manual`: manual arrows only; map Start disabled.
- `bt_echo.py`: Bluetooth diagnostics only; cannot move the robot.
- `final_week_8.py`: sensor/Task 2 helper; **not** the Task 1 runner.
- `task2_rpi_runner.py` and `task2_rpi_runner_v12.py`: preserved experimental
  Task 2 sources. Their names do not establish which is the approved version.

The repository's `run_week8.sh` is now just a Task 1 compatibility wrapper.
It does not kill other processes. The older deployed Pi script may still
behave differently; prefer the explicit Python command above.

## Configuration

`settings.py` in the repository supports these session overrides:

```bash
LAPTOP_IP=192.168.50.181 python3 -u task1_main.py
```

Use your actual laptop Wi-Fi address. Optional overrides are `CV_IP`,
`ALGO_PORT`, `IMAGE_API_PORT`, and `STM_SERIAL_PORT`.
Default ports are Algo 8000 and CV 8001. Default motion speed remains **50**.
Do not apply speed-25 turn measurements as if they were verified at speed 50.
The launcher checks Algo's profile before opening hardware. `--calibrated`
selects the rejected speed25 trial, not the new speed50 overlays. For speed50
forward/reverse trials, configure Algo's overlay options and run Task1 normally.

Current Task 1 needs `requirements-task1.txt`, the OS Bluetooth tools and
`rpicam-still`. Historical `requirements.txt` is preserved from the Pi;
its old PyBluez/picamera packages are not needed by this Task 1 controller.
Do not reinstall a working Pi environment just to match that legacy file.

## Offline tests

```bash
python3 -B tests/test_offline.py
```

The runner blocks serial construction and uses fake links/camera/HTTP.
Never discover all `test*.py` in an old Pi working directory: several
historical scripts are live motor tests, not unit tests.

## Deployment

Do not use `scp -r RPi/* ...`, `git reset`, or a GitHub pull to replace the
Pi's newer files. First stop the robot program yourself, take a source backup,
compare each target against its expected original hash, review a narrow patch,
and test before restarting. Keep the backup outside the live source directory.

See [repository guide](../docs/REPOSITORY_GUIDE.md) for what remains undeployed.

Known unresolved issue: `RaspberryPi.stop()` does not terminate/join all its
workers. In the last accessible session, two Task1 parents and a missed STM ACK
were observed after SSH resets/restarts. Do not mistake worker children for
duplicate controllers, but do not run two independent parents on the same links.
This cleanup does not claim to fix shutdown or validate it on the unavailable Pi.
