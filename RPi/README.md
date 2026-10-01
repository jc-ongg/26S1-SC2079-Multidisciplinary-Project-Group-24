# RPi — Task 1 orchestration

Drop-in copy of `~/SC2079-MDP/rpi/` on the Pi: the Task 1 runner and the
`communication/` package it needs. Rebuilt from the session history after the
Pi's working copy (which did a clean 3/3 run) became unavailable.

```
final_week_8.py              Task 1 runner (tablet -> algo -> STM -> camera -> tablet)
settings.py                  serial port, laptop IPs/ports
run_week8.sh                 frees camera/BT/serial, preflights, starts the runner
communication/
  stm32.py                   MDP_test.hex protocol: <id>/MOTOR/<CMD>/<p1>/<p2>;
  android.py                 Bluetooth RFCOMM link, //FRAME// on the wire
  android_protocol.py        frame parsing + tablet <-> internal message mapping
  link.py, __init__.py       unchanged
tests/test_offline.py        40 checks, hardware and network mocked
```

Not included because they were never changed and already live on the Pi:
`logger.py` (`prepare_logger`) and `consts.py` (`SYMBOL_MAP`, optional).

## Run

On a laptop: `python RPi-services/laptop_start.py` (set it up once with
`laptop_setup.py`; see `RPi-services/README.md`). It prints its own IP. The
laptop must be on the Pi's hotspot.

```bash
scp -r RPi/* nat@192.168.50.5:~/SC2079-MDP/rpi/
ssh -t nat@192.168.50.5 'cd ~/SC2079-MDP/rpi && LAPTOP_IP=<laptop ip> ./run_week8.sh'
```

`LAPTOP_IP` defaults to `192.168.50.147`, the laptop used on 2026-09-25.

Tablet: connect, place robot + obstacles, press Start on the Challenge screen.
After the run, open `http://<laptop ip>:8001/stitch` for the tiled RAW display.

Offline tests (no Pi needed): `python3 tests/test_offline.py`

## What changed from the original final_week_8.py

Carried over from the version that ran 3/3:

- Algo server API: `POST /path` with the text payload, `data.stm_commands`
  (`center,0,forward,120` form), `center/left/right` commands forwarded to the STM
- `SNAP1_L` direction suffix stripped to get the obstacle id
- `rpicam-still` (there is no `libcamera-still` on this OS)
- image-api on `:8001` `/predict`, field `image`
- tablet gets `TARGET,<obstacle>,<numeric image id>` — the app drops non-numeric ids
- `/stitch` requested from image-api, not the Algo server
- robot start pose from the tablet passed to the planner
- movement lock released on failed captures, ACK wait bounded at 15s
- Start no longer waits for the command queue to be filled (startup race that
  left the robot sitting still after BEGIN)
- FIN retry looks failed ids up in the obstacle map (used to crash)

Found while rebuilding and **not yet run on the robot**:

- `FL00`/`FR00` (tablet L/R buttons) sent `TURNL/TURNR` with angle 0; now 90
- image-api's 404 ("no objects in frame") is retried like NA instead of
  abandoning the obstacle
- an STM `ERROR` reply (e.g. `TURN_TIMEOUT`) releases the movement lock instead
  of stalling 15s
- an unknown command is logged and skipped instead of killing command_follower
- each Start clears the previous run's obstacles and results, so Reset Map +
  re-place starts clean

Turns are sent unverified. Gyro-verified turning (`STMLink.turn_by`) works in
standalone scripts, but inside this multi-process runner it collides with
`recv_stm` on the serial port; it would need to move into `recv_stm`.
