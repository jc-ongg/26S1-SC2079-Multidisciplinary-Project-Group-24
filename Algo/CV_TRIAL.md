# Temporary Task 1 CV server

Uses the existing `Algo/best.pt` **detection** model, not the missing
`best_v11i_m.onnx` segmentation model. Its accuracy must be checked against
real target photos. The class names in this model are numeric MDP image IDs;
they are not the class-index mapping from the original ONNX server.

Windows PowerShell, from the `Algo` directory:

```powershell
py -3.11 -m venv .venv-cv-win
.\.venv-cv-win\Scripts\python.exe -m pip install -r requirements-cv-trial.txt
.\.venv-cv-win\Scripts\python.exe -u cv_trial_server.py
```

Leave that terminal open. Ctrl+C stops only this CV server.
Algo remains a separate program on port 8000. This server uses port 8001.

If the original `.venv-win` environment is unavailable, this environment also
includes the dependencies needed to launch the unchanged Algo server. In a
second terminal, from `Algo` (only when no Algo instance is already running):

```powershell
.\.venv-cv-win\Scripts\python.exe -u main.py
```

- `GET /health` reports the temporary model and readiness.
- `POST /predict` accepts multipart field `image`. Returns the original Pi
  format: `image_id` is a label, and `obstacle_id` is the numeric **image ID**
  (the old API's confusing naming). No detection returns `image_id: "NA"`.
- `GET /stitch` returns tiled annotated detections from the last 20 minutes.
- Captures and annotations are saved under `output/cv-trial`, using unique
  filenames so existing captures are not overwritten.

On the Pi, check the laptop's Wi-Fi IP (currently `192.168.50.181`):

```bash
curl --max-time 5 http://192.168.50.181:8001/health
```

No changes to the Pi are required to match this API. Starting this service
does **not** fix the Pi controller's existing movement-lock/timeout bugs.
Don't interpret a controller timeout as permission to keep moving blindly.

## Pi Task 1 and log watcher

Keep the laptop on the Pi Wi-Fi; phone tethering can provide internet separately.
The Pi must reach the laptop's Wi-Fi IP, not localhost or the phone-tether IP.

In a Pi SSH terminal, verify both services before starting one controller instance:

```bash
curl --max-time 5 http://192.168.50.181:8000/status
curl --max-time 5 http://192.168.50.181:8001/health
cd ~/SC2079-MDP/rpi
python3 -u task1_main.py
```

Don't run `bt_echo.py`, another Task 1 instance, or `--manual` alongside the
Task 1 controller. `--manual` is for arrow-button testing, not map missions.
After a controller restart/reconnection, connect Android and resend the map.
The current controller's known timeout/lock bugs remain unresolved; stop the
robot safely if it reports continuing without ACK or an aborted movement.

In a second Pi SSH terminal, watch the actual command flow:

```bash
cd ~/SC2079-MDP/rpi
tail -n 100 -f logfile.txt | grep -a --line-buffered -E 'Algo payload|Commands received from API|Sent to STM32|Capturing image|image API|Image recognition|TARGET|DONE|ERROR|WARNING'
```

`Algo payload` is RPi -> Algo; `Commands received from API` is Algo -> RPi;
`Sent to STM32` is RPi -> STM32; `TARGET` is recognition sent to Android.
Ctrl+C in this log-watcher terminal stops only the watcher.

## Original model and controller reference

The original CV server source on `origin/rpi/task1-services-and-fixes` expects
`RPi-services/image-api/best_v11i_m.onnx`, with that directory as the working
directory. The ONNX is excluded by that folder's `.gitignore`; it was not found
in the checked laptop/Pi locations. This temporary server does not replace it.

`task1_main.py` is just a launcher for `RaspberryPi().start()` in
`final_week_8_backup.py`. The current `final_week_8.py` is a Task 2 helper with
a sensor-test entry point. Both controller files matched the full pre-setup
backup byte-for-byte when checked. An older Task 1 backup should be used to
identify last week's working controller; do not overwrite the current files.

No-hardware regression tests:

```powershell
.\.venv-cv-win\Scripts\python.exe -m unittest discover -s tests -p test_cv_trial_server.py -v
```
