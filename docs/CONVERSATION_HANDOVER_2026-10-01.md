# NTU SC2079 MDP Group 24 — conversation handover

Prepared on **2026-10-01**, using the laptop's Asia/Singapore date.
This is the context to give the next conversation. Runtime observations are a
dated snapshot: inspect again before assuming a service, connection or file is
still in the same state. This file documents the session; it does not start or
deploy anything.

## Current-access correction — 2026-10-02

The user explicitly reports they **no longer have access to the physical Pi**.
Do not continue attempting SSH or claim this repository is a live mirror.
All following runtime observations are historical. Repo cleanup is local/GitHub
only; the last-known launcher patch and the source backup remain documented.
Use [latest status](REPO_STATUS_2026-10-02.md) before acting on the older notes.
Later cleanup recovered five project-reference documents, verified existing
source was integrated, and archived both complete old stashes under local
`archive/2026-10-02/stash-task1-services` and `archive/2026-10-02/stash-main` tags.
The user then approved removing the two active stash entries; the stash list
is empty and disk files were not deleted. Historical `stash@{0}`/`stash@{1}`
indices below are no longer valid. See [stash audit](STASH_AUDIT_2026-10-02.md).

The final accessible troubleshooting session found two independent Task1
parents (1181 and1478), a missing ACK after FWD/50/74, and garbled STM telemetry
after the second start. `KeyboardInterrupt` followed Ctrl+C; `client_loop ...
Connection reset` was an SSH disconnect, not an Algo exception. The controller
shutdown does not explicitly terminate/join all workers. This is a known
unresolved issue, not a demonstrated calibration defect. No shutdown fix was
deployed. Physical testing and comparisons must wait until access is available.

### Latest addendum: optional calibration restored on request

**Latest forward update:** result tables show speed50, TURNL15cm forward/33cm
sideways and TURNR25cm forward/37.5cm sideways (the command examples still say25,
treated as stale template text; user should correct this if actual speed was25).
Bare90-degree turns assumed, one trial each, heading unreported. Independent
`MDP_FORWARD_CALIBRATION=speed50-v2` uses FL2/4 cells +forward5/7cm and FR3/4 cells
+forward5/3cm. Existing reverse-speed50 geometry/padding is unchanged. Both
overlays are now active in Algo, retaining base profile `original`/motor50.
The server was restarted and verified, CV unchanged; no Pi edits or starts.
See [forward-speed50 details](FORWARD_CALIBRATION_SPEED50.md). Software checks
pass for combined and isolated overlays, with original golden maps preserved
when both overlays are off. Steering onset and swept-arc/padding clearance
remain physically unvalidated. This supersedes "forward settings unchanged"
in the earlier reverse-only session notes below.

**Newest calibration request:** new reverse-only values were supplied, with the
speed25 label explicitly corrected by the user to speed50: REVL29.5cm back/15cm
side, REVR34cm back/21.5cm side. One trial each; bare turns assumed, final heading
unreported. New opt-in `MDP_REVERSE_CALIBRATION=speed50-v2` keeps movement profile
`original` and motor speed50 (compatible with the physical Pi launcher), with
BL3/2 cells +reverse1/5cm and BR4/3 cells +reverse6/8cm, and directed route caches.
Forward settings and 10cm scaling are unchanged. See
[reverse-speed50 trial](REVERSE_CALIBRATION_SPEED50.md). Algo alone was
restarted and verified healthy with `reverse_calibration: speed50-v2`; CV was
verified healthy and its PID unchanged. Pi processes are untouched. SSH timed out, so
the Pi's actual running speed cannot be verified. If it still runs
`--calibrated`, the user must stop it and restart without that flag. This update
supersedes the earlier all-turn measured25 trial and its rollback runtime notes.

**Subsequent update:** the user rejected the speed-25 trial again and requested
fresh measurement guidance. The assistant restarted only Algo8000 back to
`original`, verified `/status` (`original`, recommended speed50, `ok`), and left
CV8001 unchanged. No Pi process was stopped/restarted. SSH timed out at the
start of the new measurement guidance; do not assume the Pi controller has
reverted until the user stops any `--calibrated` instance. The optional trial
code is preserved but inactive; the preceding runtime observations are historical.

The user subsequently requested another try of the previous speed-25 calibration.
This addendum supersedes historical "do not reapply"/"reverted" notes **for the
explicit trial only**. Original remains the default. See
[CALIBRATION_TRIAL.md](CALIBRATION_TRIAL.md) for exact commands and rollback.

- Algo option: `--movement-profile measured25` in `RPi-services/laptop_start.py`
  (sets `MDP_MOVEMENT_PROFILE=measured25` for Algo).
- Physical Pi launcher: `python3 -u task1_main.py --calibrated` sets motor speed
  25 after checking the Algo `/status` profile. No flag means original/speed50.
- Trial grid longitudinal/lateral: FL3/3, FR4/3, BL5/4, BR5/3. Extra straight cm
  before/after: FL5/8, FR8/4, BL5/7, BR6/3. Straight scale remains10cm/cell.
- Asymmetric trial turns use directed A* caches and directed TSP costs. Offline
  checks passed; endpoint math does NOT prove physical accuracy or swept-arc
  collision clearance. Test with space around the robot.
- Only the physical Pi launcher was patched; its Task1 core/settings were not
  overwritten. Original launcher backup:
  `/home/nat/mdp-backups/calibration-trial-20261001-DLA9Sp/task1_main.py`.
- No server/controller was started or stopped. The last check still found two
  Task1 parent processes (13983 and17670) on the Pi and an Algo8000 server loaded
  from before this change (`/status` had only `status: ok`). The user must stop
  old programs and restart with matching flags before the trial.
- This addendum and the new calibration changes are local working-tree changes;
  not committed/pushed in this turn.

## 1. Start here in the next conversation

Suggested opening message:

> Please read `docs/CONVERSATION_HANDOVER_2026-10-01.md` in my SC2079 MDP
> repository before doing anything. Work from the current state and the latest
> request I give you. The physical Raspberry Pi is the source of truth for its
> deployed code. GitHub changes do not update it. I start and restart robot
> programs myself. Inspect and explain first; preserve my files and backups.

### User's working preferences and decisions

- The user wants practical, reasonably fast progress, concise updates and exact
  commands. They can run commands and provide outputs when interactive setup
  is needed; direct read-only SSH inspection is now available too.
- **The user starts/restarts robot programs.** Do not automatically start Task 1,
  manual control, echo scripts, motor tests or robot movements.
- Prior authorization covered code fixes, commits, branch consolidation and
  normal GitHub pushes. Do not interpret it as permission to discard newer
  Pi source, flash firmware, delete recovery material or drive the robot.
- Keep the old branch history and the two existing stashes recoverable. No
  force-push, destructive reset, blanket checkout of modified files or git clean.
- The physical Pi source was newer than GitHub. Back up and compare its actual
  files before editing or deploying; never replace it wholesale from GitHub.
- Calibration decision: **10 cm per grid cell in both directions; original
  turn settings retained.** A new padded-turn experiment was rejected and
  reverted. Do not silently reapply it.
- Default Task 1 speed is currently **50**. The supplied measurements were at
  **25**; the difference must remain explicit.
- The user previously requested links to seniors' Task 2 arrow models, rather
  than downloading them. No verified external recommendation is recorded here.
- Do not expose SSH passwords, private-key contents or access tokens. This
  handover intentionally contains no passwords or private-key material.

## 2. Project and workspace

Windows PowerShell workspace:

```text
C:\Users\ongjc\Documents\MDP\26S1-SC2079-Multidisciplinary-Project-Group-24
```

Private GitHub repository:

<https://github.com/jc-ongg/26S1-SC2079-Multidisciplinary-Project-Group-24>

The four main components are Raspberry Pi, STM32, Algorithm and Android.
The laptop runs both planning and CV; the Pi orchestrates Bluetooth, serial
movement commands and camera capture.

```text
Android tablet
  -- Bluetooth --> Raspberry Pi
                       |-- USB serial --> STM32 --> motors
                       |-- camera --> JPEG capture
                       |-- Wi-Fi HTTP --> laptop Algo :8000 /path
                       |-- Wi-Fi HTTP --> laptop CV   :8001 /predict
                       <-- recognition result --> Android TARGET frame
```

| Component | Directory / entry point | Execution location |
| --- | --- | --- |
| Android | `android/MDPGroup24` | Android Studio on Windows; application on tablet |
| Algo | `Algo/main.py` | Laptop, port 8000 |
| Current temporary CV | `Algo/cv_trial_server.py`, `Algo/best.pt` | Laptop, port 8001 |
| Original CV alternative | `RPi-services/image-api/main.py`, `best_v11i_m.onnx` | Laptop, port 8001 |
| Deployed Task 1 | `~/SC2079-MDP/rpi/task1_main.py` | Pi |
| Task 1 implementation | `final_week_8_backup.py`, class `RaspberryPi` | Imported by the launcher |
| STM32 source | `STM32/` | Existing flashed robot firmware; not reflashed in this work |
| Optional frontend | `Simulator/` | Separate from the tablet/Pi Task 1 flow |
| Optional camera viewer | `RPi-services/viewer/` | Additional laptop UI; not needed for Task 1 |

`task1_main.py --manual` runs `manual_control.py`: arrow-button testing only.
It cannot execute a map mission. `bt_echo.py` tests Bluetooth messages and does
not forward motor commands. This explains why the icon could move while the
physical robot stayed still when echo mode was being used.

**The actual current `final_week_8.py` is a sensor/Task 2 helper, not the Task 1
controller.** An earlier reconstructed GitHub branch used that filename for a
Task 1 controller. The actual Pi files resolved the ambiguity.

Selected Task 2 sources `task2_rpi_runner.py` and `task2_rpi_runner_v12.py` were
recovered into the repository. The Pi snapshot contains more variants and
diagnostic scripts. The latest-looking filename does not establish the approved
Task 2 entry point. Those runners have not been validated as part of consolidation.

## 3. Network, SSH and direct Pi access

The Pi is headless and creates its own Wi-Fi hotspot, `MDP-group-24`. The Windows
laptop joins it. Phone **USB tethering** supplies internet while Wi-Fi remains
connected to the Pi. School Wi-Fi was discussed but has not been configured or
verified as the development network.

| Item | Last verified value |
| --- | --- |
| Pi address | `192.168.50.5` |
| Pi SSH username | `nat` |
| SSH / SFTP port | `22` |
| Physical source directory | `/home/nat/SC2079-MDP/rpi` |
| Laptop Pi-facing Wi-Fi address | `192.168.50.181` |
| Laptop Algo endpoint reachable from Pi | `http://192.168.50.181:8000/status` |
| Laptop CV endpoint reachable from Pi | `http://192.168.50.181:8001/health` |
| SSH key file on this laptop | `C:\Users\ongjc\.ssh\mdp_rpi_codex` |

The laptop's Wi-Fi address can change. Recheck it with `ipconfig`; do not use
the phone-tether address for Pi-to-laptop traffic. `localhost` on the Pi means
the Pi, not the laptop. A previous SSH timeout coincided with Windows Wi-Fi
being disconnected; reconnecting to the Pi hotspot restored access.

The key-based SSH command used successfully by the assistant is:

```powershell
ssh -i C:\Users\ongjc\.ssh\mdp_rpi_codex -o BatchMode=yes -o ConnectTimeout=5 -o StrictHostKeyChecking=yes nat@192.168.50.5
```

The same command can run a narrow quoted remote read-only command. A new
conversation still needs access to this Windows workspace and permission to
use its SSH executable/key. Do not assume a remote Pi project is automatically
attached to the chat simply because its source copy exists locally.

The Pi directory was **not a Git repository** when inspected. Direct editing
can be done through SSH, using a backup, current-file comparison and a narrow
reviewed patch. The workflow does not require repeatedly copying the entire
codebase down and uploading it again.

WinSCP is optional for SFTP browsing and viewing Pi files: host `192.168.50.5`,
port 22, user `nat`, appropriate authentication. Windows OpenSSH already works;
PuTTY is not required for this setup.

## 4. Runtime snapshot at handover creation

Read-only checks during creation of this handover found:

- `http://127.0.0.1:8000/status` answered `{"status":"ok"}`.
- `http://127.0.0.1:8001/health` reported a healthy temporary detection model,
  `Algo/best.pt`, with 31 classes.
- SSH to the Pi succeeded.
- The Pi has `python3 -u task1_main.py` running, with several multiprocessing
  children. Multiple matching PIDs alone do not prove duplicate controller
  launches; inspect their parent relationships before making that claim.
- Actual Pi `settings.py` still has hardcoded laptop address `192.168.50.181`,
  Algo port 8000 and CV port 8001.
- Actual Pi `communication/stm32.py` has `DEFAULT_SPEED = 50`.
- Deployed `final_week_8_backup.py` SHA-256 is:
  `0378f119886717316df9e0b9f6b22ed38cb0a7efd5def1f2907e5bd2fc6696b5`.

The Pi has previously reached both laptop health endpoints over Wi-Fi.
In the scan-distance investigation, a diagnostic POST to the live laptop Algo
returned `center,0,forward,120` for a 12-cell straight segment, confirming that
the server then running used **10 cm per cell**. The diagnostic route was not
sent to the Pi or robot.

The Pi clock is offset: logs display August 22 while the Windows session is
October 1. Do not use its timestamp alone to decide which file or run is newer.

## 5. Operator startup and health commands

These are commands for the **user to run when appropriate**, not instructions
for the next assistant to start a robot or duplicate a running server.

From PowerShell:

```powershell
cd "C:\Users\ongjc\Documents\MDP\26S1-SC2079-Multidisciplinary-Project-Group-24"

# Read-only checks; starts nothing.
.\Algo\.venv-cv-win\Scripts\python.exe RPi-services/laptop_start.py --python .\Algo\.venv-cv-win\Scripts\python.exe --cv-backend trial --check

# Start both laptop services ONLY after existing instances have been stopped.
.\Algo\.venv-cv-win\Scripts\python.exe -u RPi-services/laptop_start.py --python .\Algo\.venv-cv-win\Scripts\python.exe --cv-backend trial
```

The launcher refuses occupied ports, prints laptop addresses, appends logs under
`RPi-services/logs/`, and stops only the laptop children it created on Ctrl+C.
It does not launch the Pi. Both ports are checked before either child starts,
preventing an orphan Algo process if the CV port is already occupied.

For separate laptop terminals, from `Algo/`:

```powershell
.\.venv-cv-win\Scripts\python.exe -u main.py
.\.venv-cv-win\Scripts\python.exe -u cv_trial_server.py
```

Each command belongs in its own terminal; do not run it if that server already
owns its port. The shared Python 3.11 CV environment was verified to import
the needed dependencies. Earlier environment launchers were broken/missing;
`Algo/.venv` contained macOS Python paths. Do not copy/revive a macOS venv on
Windows or restore thousands of stashed environment files.

For a genuinely new temporary environment, preserving any existing one first:

```powershell
py -3.11 -m venv Algo/.venv-cv-win
.\Algo\.venv-cv-win\Scripts\python.exe -m pip install -r Algo/requirements-cv-trial.txt
```

PowerShell health checks:

```powershell
Get-NetTCPConnection -State Listen -LocalPort 8000,8001
Invoke-RestMethod http://127.0.0.1:8000/status
Invoke-RestMethod http://127.0.0.1:8001/health
```

In an SSH terminal on the Pi:

```bash
curl --max-time 5 http://192.168.50.181:8000/status
curl --max-time 5 http://192.168.50.181:8001/health
cd ~/SC2079-MDP/rpi
python3 -u task1_main.py
```

The final line starts Task 1; the user runs it after stopping an existing
controller. For manual-only arrows the alternative is:

```bash
python3 -u task1_main.py --manual
```

Use one controller owning Bluetooth, serial and camera at a time. Following
a restart/reconnection, reconnect the tablet and resend the map before Begin.

Repository `settings.py` supports `LAPTOP_IP`, `CV_IP`, `ALGO_PORT`,
`IMAGE_API_PORT` and `STM_SERIAL_PORT` environment overrides. The deployed Pi
settings do **not yet** support these overrides. Therefore a command such as
`LAPTOP_IP=... python3 -u task1_main.py` only works after that settings patch is
deployed; until then inspect/update the actual Pi settings through a narrow patch.

The repository's `RPi/run_week8.sh` now simply executes `task1_main.py` and
passes arguments. The old physical Pi wrapper still requires review/deployment:
it pointed at the sensor helper and contained automatic process-killing logic.
Prefer the explicit Python command for current testing.

## 6. Protocol and command flow

The tablet uses framed Bluetooth messages `//<payload>//`. Bluetooth is a stream;
the Pi parser handles fragmented messages and multiple frames per read.

Examples observed in the conversation:

```text
//ROBOT,11,5,N//
//OBSTACLE,1,12,14,S//
//MAP/ROBOT,11,5,N/OBSTACLE,1,12,14,S//
//BEGIN,1//
//F//   //B//   //L//   //R//
```

The Pi accumulates the map and translates Begin into obstacles followed by
`control:start`. Internal headings are NORTH=0, EAST=2, SOUTH=4, WEST=6.
The start pose placed on Android must be passed to Algo rather than silently
using `(1,1,NORTH)`.

Task 1 sends a text/plain HTTP body to laptop `POST /path`, for example:

```text
STATUS,OBSTACLE_LIST,false,10;1;NORTH,false,1;10;17;SOUTH | 2;2;11;EAST,true,true
```

The retry boolean is the third comma-separated field. The response includes
`data.commands` (planner commands), `data.stm_commands` (adjusted movement
commands) and `data.path` (planned poses). These are different representations.

Example adjusted commands:

```text
center,0,forward,120
center,0,reverse,37
left,90,forward,0
SNAP1
SNAP2_C
FIN
```

The Pi strips the optional SNAP direction suffix to obtain the obstacle number.
`SNAP` invokes camera capture and recognition; `FIN` handles route completion
and possibly one failed-obstacle revisit.

The flashed firmware was identified in session sources as `MDP_test.hex`, with
this serial protocol, baud 115200:

```text
<id>/MOTOR/<command>/<speed>/<distance-or-angle>;
!<id>/DONE;
!<id>/ERROR/<reason>;
```

For example, `1/MOTOR/FWD/50/10;` requests a 10 cm forward move at speed 50.
The **semicolon is required**. Firmware commands include FWD, REVS, TURNL,
TURNR, REVL, REVR and STOP. Parameterised TURNL/TURNR with an angle are used;
old square-turn commands were reported not to change the measured heading.
Do not assume repository STM32 sources exactly match the currently flashed
firmware without confirming it. No STM32 firmware was flashed during cleanup.

Current Android arrows translate to FW010, BW010, FL090 and FR090, then to the
firmware frames. The original log had FL00/FR00; treating these as zero-degree
turns was part of the manual-control issue addressed in the actual Pi code.

Recognition goes back to the tablet as:

```text
//TARGET,<actual obstacle number>,<numeric MDP image ID>//
```

For instance, `//TARGET,1,38//` means obstacle 1 was recognised as right arrow.
The CV response uses confusing old names: `image_id` is the symbol label;
`obstacle_id` is actually the numeric **image ID**. The Pi adds the true obstacle
number. Do not accidentally swap these or send a nonnumeric label as TARGET ID.

### Watch commands and recognition

In a separate Pi SSH terminal:

```bash
cd ~/SC2079-MDP/rpi
tail -n 100 -f logfile.txt | grep -a --line-buffered -E 'Algo payload|Commands received from API|Sent to STM32|Capturing image|image API|Image recognition|TARGET|DONE|ERROR|WARNING|At FIN|Attempting to go to failed'
```

`grep -a` is necessary because the log sometimes contains binary/NUL bytes;
without it, grep reports `binary file matches` and hides the lines.

- `Algo payload`: Pi -> laptop planner.
- `Commands received from API`: planner -> Pi.
- `Sent to STM32`: translated motor frame and source command.
- `DONE`: STM completion.
- Recognition/TARGET: CV result and tablet update.

Ctrl+C in this watcher stops only the watcher, not the controller in another
terminal. A generic Algo API error alone is not enough to diagnose the cause;
inspect the laptop traceback, payload, network reachability and response status.

## 7. CV models and viewing pictures

### Current temporary detector

- Server: `Algo/cv_trial_server.py` on 8001.
- Model: the existing tracked `Algo/best.pt`; 31 classes; task `detect`.
- The model's class names are already numeric MDP image IDs. The temporary
  service validates them and does not apply the ONNX class-index mapping.
- Multipart upload field: `image`, endpoint `POST /predict`.
- Bullseye/NA is ignored as a scored target; accepted detections are selected
  by largest box area, then confidence, using a minimum confidence of 0.30.
- `GET /health` identifies it as temporary and reports the model path.
- This is a useful working fallback; its accuracy has not been certified to
  match the original model. Do not claim that the original model disappeared
  permanently just because it was absent from the inspected checkout.

### Original ONNX alternative

Expected model file:

```text
RPi-services/image-api/best_v11i_m.onnx
```

The corresponding server is `RPi-services/image-api/main.py`, launched with
that directory as its working directory. It explicitly uses `task='segment'`.
It has a different class ordering/mapping from `best.pt`; the weights and
mappings are not interchangeable. The filename itself is not proof of the
underlying architecture.

The teammate's setup manifest references the private release:

<https://github.com/jc-ongg/26S1-SC2079-Multidisciplinary-Project-Group-24/releases/tag/cv-model-v11i>

Expected asset SHA-256:
`8c4f382f800d123e069fe93f0069e279175594c0a6ef631c699fbc3d957ee7d5`.

The original ONNX was not found in the locations checked during this work.
The referenced release asset's current availability and original-model
inference were not independently verified. No ONNX download was performed
during consolidation. The setup expects roughly 109 MB and private-repo access.

`laptop_setup.py` builds the original locked environments using Python 3.12/3.13
or uv. It now refuses to overwrite a different existing model, checks a
downloaded candidate, and installs it without replacing another concurrent
file. Do not execute that setup merely to inspect a working system.

The original `/predict` service returns HTTP 404 when nothing is detected.
The repository Pi controller now treats that as NA and retries; that small
follow-up patch is **not yet deployed** on the actual Pi. The currently used
temporary CV returns a normal NA JSON response instead.

The Algo server's legacy `/infer` uses a different upload field (`file`) and
is not the current Task 1 CV path. Use the separate CV `/predict` endpoint.
Algo model loading was made lazy so planning does not depend on loading CV.

### Pictures the user can inspect

Open this on the laptop:

<http://127.0.0.1:8001/stitch>

For temporary CV this shows up to 30 annotated detections from the last
**20 minutes**. Refresh after a new scan. Pictures with no recognised target
are not included; a 404/no-recognised-captures response does not prove that
the camera failed.

Local picture folder:

```text
C:\Users\ongjc\Documents\MDP\26S1-SC2079-Multidisciplinary-Project-Group-24\output\cv-trial
```

From repository-root PowerShell:

```powershell
explorer .\output\cv-trial
```

Sort by Date modified. `*_original.jpg` is the received camera image;
`*_annotated.jpg` has the detection rectangle, symbol, numeric ID and confidence.
The temporary server uses unique filenames for each processed capture.
Its output-directory recovery fix recreates the folder if it disappears while
the server is running. That fix needs a user-performed CV restart to affect a
process which was started before the code changed.

The Pi also saves raw images in the Task 1 working directory as
`<timestamp>_<obstacle_id>.jpg`. Repeated attempts within a single SNAP reuse
that filename, so the Pi file may contain only the final attempt. If an API
request fails before a laptop copy is saved, inspect the Pi raw JPEG via SFTP.
Original ONNX annotations use `RPi-services/image-api/own_results/` instead.

## 8. Failed scans, revisits and snapshot distance

### Actual deployed retry mechanism

The deployed Task 1 controller already has two levels of retry:

1. At a SNAP position, it takes up to **7 pictures at the same position** if
   CV returns NA. A camera/network/API error can terminate that sequence early.
2. It records unresolved obstacle IDs in `failed_obstacles`.
3. At the first FIN, it asks Algo for one additional route containing only
   those failed obstacles, from its recorded `current_location`, with
   `retrying=True`.
4. `failed_attempt` becomes True, so there is only one additional planned pass
   per controller lifetime. Successful recognition removes the failure entry.

In a recent actual Pi log, obstacle **5** was marked failed, the controller
logged `Attempting to go to failed obstacles`, then submitted a retry payload
for obstacle 5. At the next FIN, the failed list was empty. This is evidence
that a revisit occurred, not proof that every future missed obstacle is handled.

Known limitations:

- `failed_attempt` is a plain process-local boolean initialised in the
  constructor. It is not reset for a new map/run, so later runs in the same
  process can skip the extra pass. Failed/success lists and obstacle-map state
  also need a coordinated reset; clearing them mid-mission would be wrong.
- An obstacle the planner never includes does not necessarily become a failed
  scan entry. Do not assume all unvisited/unreachable obstacles are retried.
- Recorded poses can be out of sync with adjusted motor commands, which can
  make the retry plan start from an incorrect pose; see the remaining-issues
  section below.

### How far away when the snapshot is taken?

The user specifically means the camera-to-picture gap at capture time. Do not
answer that question solely with movement-command distance or cell size.

There is **no fixed 10 cm / 20 cm camera-gap threshold** or distance-sensor
trigger in the current Task 1 snapshot logic. Algo selects a viewing pose, the
Pi executes the path, waits for the preceding motor command's matching DONE,
then processes SNAP and invokes `rpicam-still` while the movement lock is held.

Viewing candidates are defined in `Algo/entities/Entity.py`,
`Obstacle.get_view_state(retrying)`. With `EXPANDED_CELL = 1` and 10 cm cells:

| Pass | Separation along the obstacle-facing axis, measured from robot centre to obstacle grid position |
| --- | --- |
| Initial visit | 3 or 4 cells: **30 or 40 cm** |
| Failed-obstacle revisit | 4 or 5 cells: **40 or 50 cm** |

There are also candidates 4 cells away with a sideways offset of 1 cell
(10 cm); those are not exactly 40 cm in Euclidean distance. The aligned
initial 3-cell candidate has penalty 5; the 4-cell candidate has penalty 0.
Route cost, other obstacles and reachability determine the selected candidate.

A diagnostic query to the live server placed obstacle 1 at `(10,17)` and chose
scan pose `(10,13)`: **40 cm robot-centre-to-obstacle-grid separation**.

The actual lens-to-target-image gap is smaller/different according to camera
offset from robot centre and the target face's location within the obstacle.
Those physical offsets were not supplied/measured. **Do not invent an exact
10 cm or 20 cm lens-to-image gap.** Measure it at the stopped scan position or
obtain the mounting dimensions to establish the actual distance.

## 9. Calibration measurements, experiment and accepted baseline

All measurements supplied by the user below were taken at **speed 25**.
Distances are centimetres. Turn displacement is measured from the robot centre;
longitudinal means along the starting heading axis, lateral means sideways.

### Straight measurements

| Command | Speed | Requested distances | Actual distances |
| --- | ---: | --- | --- |
| FWD | 25 | 10, 20, 40, 80, 120 | 10, 20, 40, 80, 120 |
| REVS | 25 | 10, 20, 40, 80 | 10, 20, 40, 80 |

This supports empirical distance correction 1.0 at speed 25. Separately,
`UNIT_TO_CM_MULTIPLIER` converts grid cells to centimetres, so its accepted
value is **10**, not 1 or 20. Main previously used 20; that doubled commanded
straight distances relative to the 10 cm arena grid.

### Bare 90-degree turn measurements

| Turn | Longitudinal trials | Lateral trials | Mean longitudinal | Mean lateral |
| --- | --- | --- | ---: | ---: |
| TURNL | 25.5, 25.5 | 22, 22 | 25.50 | 22.00 |
| TURNR | 32.5 | 26 | 32.50 | 26.00 |
| REVL | 45, 45, 45 | 34, 32, 32.5 | 45.00 | 32.83 |
| REVR | 44, 44, 45 | 26.5, 27.5, 28 | 44.33 | 27.33 |

Blank trials were not treated as zeros. No final heading-angle measurements
were supplied. The forward-right mean is based on only one trial.

### Rejected experiment — historical only

An experiment rounded the measured turn endpoints up to whole grid cells and
added straight padding around turns. It temporarily set Task 1 speed to 25 and
changed directional route caching. The user said it performed worse and asked
to restore the original promptly. Those calibration/planning changes and the
speed-25 launcher override were rolled back; the CV lock fix remained.

The rejected proposed values were FL geometry 3/3 cells with padding 5/8 cm,
FR 4/3 with 8/4, BL 5/4 with 5/7 reverse, and BR 5/3 with 6/3 reverse.
These are **not active** and are listed only to prevent accidental reuse.
`Algo/MOVEMENT_CALIBRATION.md` preserves the historical experiment/rollback
record; its old commands and deployment claims are not current instructions.

The user later explicitly approved changing straight scaling to 10 cm while
keeping the original turns. Current settings:

| Setting | Accepted current value |
| --- | --- |
| Forward / reverse cm per grid cell | 10 / 10 |
| FL planner longitudinal / lateral cells | 2 / 4 |
| FR planner longitudinal / lateral cells | 2 / 4 |
| BL planner longitudinal / lateral cells | 3 / 2 |
| BR planner longitudinal / lateral cells | 3 / 2 |
| FL straight before / after | 3 / 5 cm, forward |
| FR straight before / after | 0 / 4 cm, forward |
| BL straight before / after | 0 / 2 cm, reverse |
| BR straight before / after | 0 / 4 cm, reverse |
| Pi default motion speed | 50 |

`Algo/CALIBRATION.md` is the active calibration document. The retained original
turns are a baseline; they were not newly proven accurate by the supplied
speed-25 measurements. The robot is not certified as fully calibrated.

`RPi/calibration_plan.py` is a **print-only** helper. It prints unique-ID
firmware frames for forward/reverse distances and three trials of each turn:

```bash
python3 calibration_plan.py --speed 25
# Or --speed 50 to measure the current Task 1 baseline speed.
```

This file is repository-only until deployed; it never opens serial or sends
a command. The operator should measure bare and padded turns separately at
the same chosen speed, recording endpoint, heading, battery and surface.
Start with controlled single moves and a simple one-obstacle route.

## 10. Changes made and deployment status

### Deployed Pi improvements carried into the repository

- Standard-library Bluetooth RFCOMM handling and stream frame translation;
  the Pi's Bluetooth daemon was observed in compatibility mode. Adapter alias
  is `MDPGrp24`; it was powered, discoverable and pairable when inspected.
- Firmware-aware serial translation, with semicolon framing, command IDs,
  parameterised turns and telemetry parsing.
- Android manual F/B/L/R control forwarding. The manual controller ignores
  unrelated ACKs, rejects stale busy presses and stops/blocks further commands
  on its timeout/error path, with status messages to the tablet.
- Task 1 quiet sensor logging: telemetry still updates state, but the roughly
  8 Hz sensor-value spam is suppressed. Motor commands, ACKs and errors remain.
- Task 1 capture now runs synchronously in the command follower while holding
  the movement lock. FIN cannot overtake ongoing CV retries or failure-list
  bookkeeping. Capture no longer releases the caller's lock multiple times.
- Motor completion is gated by matching pending command ID; duplicate/late
  ACKs cannot release a camera operation's lock.
- Camera subprocess is bounded (10-second timeout), HTTP CV requests use
  `(3,10)` timeouts, and image file handles are closed.
- Fault handling latches a pause and prevents additional command dispatch;
  network errors during stitching are caught instead of killing that worker.
- Begin no longer relies on the path queue being immediately filled by another
  process; the Android start-pose payload reaches Algo.

The actual Pi file/hash and source snapshot are authoritative for which of
these are deployed. The controller still has the remaining issues below.

### Repository/laptop/Android changes committed and pushed

- Consolidated both RPi branches and current Android main history.
- Recovered actual Pi core files to resolve differences against reconstructed
  teammate branch source. Preserved selected Task 2 files without claiming
  they were tested.
- Android Send Map confirmation in chat and user feedback.
- Android **Save | Use** buttons under Undo: SharedPreferences stores robot
  pose and obstacle IDs/positions/directions; Use restores them to the arena,
  validates the layout, clears stale interaction/history state and updates UI.
  Use loads the layout locally; Send Map still sends it to the Pi.
- Separate temporary CV server with numeric-label mapping and annotated captures.
- CV output directory is recreated if removed during a branch switch/cleanup.
- Lazy Algo model import/loading; path planning no longer imports the ML stack
  solely to calculate a route.
- Safe dual laptop launcher and original-model download preservation/checksum
  handling, described above.
- 10 cm straight scaling, original turn parameters, meaningful offline tests.
- Current startup README, calibration guide and repository/deployment guide.
- Historical Mac runbooks marked superseded; inline passwords removed from
  current versions. Old Git history still contains historical credentials;
  cleanup did not rewrite history or rotate device credentials.
- Android's machine-specific deployment-target file stopped being tracked;
  its local copy was kept. Root ignore rules cover runtimes, generated captures,
  logs, build products and per-machine IDE files. Shell scripts use LF endings.

An earlier Android Studio error said `No Android facet found for
MDP_Group_24.app.main`. The correct Android project is the nested
`android/MDPGroup24` directory. The recent command-line debug build passed;
there is no reliable record here of a specific IDE configuration repair to
repeat automatically if that dialog reappears.

### Follow-ups still only in repository, not on physical Pi

- `RPi/settings.py`: environment overrides for laptop/CV addresses, ports and
  serial-device path, retaining current defaults.
- `RPi/run_week8.sh`: safe Task 1 wrapper replacing the old sensor-launching,
  process-killing behaviour.
- `RPi/final_week_8_backup.py`: HTTP 404 from the original ONNX CV API is treated
  as no detection and retried.
- Curated offline test runner, protocol tests, current requirements file and
  calibration-plan printer.

No new Pi deployment, controller start/stop or firmware flash occurred during
the repository consolidation or branch synchronisation. Starting a new chat
does not change that boundary. Back up and patch the latest physical files
before deploying these follow-ups, after the user has stopped Task 1.

## 11. Remaining functional issues

1. **Pose queue alignment:** Algo `data.path` is indexed by the unadjusted
   planner movement list. `data.stm_commands` adds before/after turn movements,
   removes zero moves and can cancel opposite straights. The Pi consumes one
   path entry per matching motor ACK, so intermediate Android pose and
   `current_location` can drift from the intended command endpoint. A revisit
   can therefore be planned from the wrong start pose. Needs a consistent
   command-to-pose contract or another explicitly tested tracking approach.
2. **New-run reset:** `failed_attempt`, failed/success IDs and obstacle-map
   lifecycle need a coordinated reset when a fresh mission is accepted. Reset
   must account for the controller's multiple processes and active missions.
3. **Planner failures:** network/JSON/request errors in `request_algo` are not
   fully handled. A worker may terminate or wait indefinitely after a failed
   path request. Retry planning must fail cleanly and avoid dispatching stale
   movements. Some responses are only logged as errors.
4. **Timeout validation:** Task 1 has a 15-second pending-movement lock wait,
   while serial `READ_TIMEOUT` is 30 seconds. Validate the actual longest move
   against these values; increasing timeouts does not solve pose/CV ordering
   bugs. A fault pause is not a measurement of physical stopping position.
5. **Physical turn accuracy:** default speed 50 and original turns need
   controlled measurements. The rejected experiment must remain reverted.
6. **Original CV/model:** verify the actual ONNX asset and predictions before
   claiming the original model is restored or switching backends.
7. **Camera gap:** actual lens-to-target distance is unmeasured. Do not present
   30/40 cm centre-to-grid values as lens-to-picture measurements.
8. **Task 2:** confirm approved runner/firmware and model requirements with the
   user before treating preserved variants or senior arrow models as production.

The recent successful obstacle-5 revisit and software tests do not remove
these limitations. Respond to the next user's specific request, rather than
starting every outstanding change automatically just because it is listed.

## 12. Git consolidation and the 19k-files issue

At the start of handover creation, current local main and the last verified
pushed GitHub main were:

```text
62e253dd1c145628e580f56cace93251a954720d
```

All four GitHub branch tips were synchronised to that same commit in the last
successful push, and the local tracking references still match:

| Branch | Current role |
| --- | --- |
| `main` | Use for ongoing integrated work |
| `android` | Retained branch, now contains the integrated snapshot |
| `rpi/stm-protocol-and-a5` | Retained branch, now contains the integrated snapshot |
| `rpi/task1-services-and-fixes` | Retained branch, now contains the integrated snapshot |

The local `codex/repository-consolidation` branch was also advanced to it.
An additional read-only attempt to recheck GitHub during handover creation
failed to connect to github.com on port 443. It changed no remote state. If
fresh remote state matters in the next conversation, verify internet/tethering
and fetch again rather than treating cached tracking references as a live check.
Each branch is a full repository snapshot, not a component-specific folder;
seeing all four components on each branch is expected.

The former branch positions were preserved as annotated tags, pushed to GitHub:

| Archive tag | Original commit |
| --- | --- |
| `archive/2026-10-01/android-before-consolidation` | `6207ffa65dcd88c2a3a769d703d0160b53f4999f` |
| `archive/2026-10-01/rpi-stm-protocol-and-a5-before-consolidation` | `a8e978ed959069d32a657531e75f3279b5daeff8` |
| `archive/2026-10-01/rpi-task1-services-and-fixes-before-consolidation` | `35126fc4a706998ea21ca2adb9111b7afe3eacda` |

History was consolidated with normal merges and branch advancement. Branches
and tags were pushed in an atomic update; no force-push or branch deletion.

### Why thousands of files reappeared after cleanup

The user switched GitHub Desktop back to the old RPi services branch. Its
older `.gitignore` lacked the runtime rules added to main. The working folder
then showed **19,283** files in `Algo/.venv-cv-win`, plus another environment
and generated `output/` and `tmp/` files, as pending untracked changes.
These were installed dependencies/generated files, not thousands of source edits.

The fix was to synchronise the retained branches, return the workspace to main,
and install repository-local ignore rules active across historical checkouts:

```text
git config --local core.excludesFile
C:/Users/ongjc/Documents/MDP/26S1-SC2079-Multidisciplinary-Project-Group-24/.gitignore-local
```

The `.gitignore-local` file is itself ignored and stays on this laptop. Main's
committed `.gitignore` provides equivalent source/runtime exclusions to other
laptops. Existing tracked `Algo/best.pt` stays tracked despite the new-model
ignore rule. The runtime folders and generated images were not deleted.

### Stashes remain intact

```text
stash@{0}
426475c94c966e779e0734ee842b174acd0bd3aa
On rpi/task1-services-and-fixes: !!GitHub_Desktop<rpi/task1-services-and-fixes>

stash@{1}
f8fd5f9f02f9433e6c7972d322f935c11f9c601c
On main: !!GitHub_Desktop<main>
```

Meaningful temporary CV source was recovered from stash 0; Android Save/Use
source was recovered from stash 1. The stashes also contain old environments,
generated artifacts and assessment files. **Do not pop/apply a whole stash**
over the consolidated checkout. Inspect and recover a specific source file if
needed. They were not dropped. Unfiltered stash stats can be enormous.

### Relevant commit trail

| Commit | Work |
| --- | --- |
| `c0505dca` | Android Send Map confirmation in previous main |
| `f0cf3e39` | Ignore local runtimes/generated artifacts |
| `be6e8699` | Merge protocol/A5 branch history |
| `7c6270b0` | Record rejected calibration and restored baseline |
| `0b1b17e3` | Merge services with authoritative Pi core source |
| `df819ba4` | Android saved-layout Save/Use feature |
| `d2aaef47` | CV handling, safe launcher and calibration checks |
| `137c7771` | Current startup/deployment/calibration guides |
| `9c81058d` | Historical runbooks and removal of inline credentials |
| `62e253dd` | Explain synchronised branches and persistent local ignores |

The working tree was clean before this handover was created. **This handover
file is a new local document and has not itself been committed or pushed.**
Its appearance as one untracked Markdown file is expected and is unrelated to
the 19k runtime-file issue. Its creation does not move any branch or deploy code.

## 13. Backups and safe physical-source reconciliation

The source-only backup created before consolidation is:

```text
Pi:
/home/nat/mdp-backups/repo-consolidation-20261001-bzHGLl/rpi-source.tar.gz

Matching Windows copy:
C:\Users\ongjc\Documents\MDP\26S1-SC2079-Multidisciplinary-Project-Group-24\output\repo-cleanup-20261001\rpi-source.tar.gz

SHA-256:
e4e1b1833374645b1007e8e7846b5fa544a80b8d2cb5528bd640b95cec04ba8d
```

The local hash was rechecked while preparing this handover. The extracted
reference is in ignored `tmp/repo-cleanup-20261001/pi-source/`. The archive
contains source scripts, including older Task 2 versions and diagnostics;
it is not a full OS, firmware, installed-package or captured-image backup.

Earlier session backup locations were recorded as follows. Recheck their
existence/content before depending on them; do not restore them blindly:

| Purpose | Pi location under `/home/nat/mdp-backups/` |
| --- | --- |
| Before initial Task 1 setup | `task1-setup-20261001-c64ea8/rpi-before-setup.tar.gz` |
| Before manual-control work | `manual-controls-20261001-c7f2/before-manual.tar.gz` |
| Before quiet telemetry logging | `task1-log-20261001-7a9f/before-quiet-sensors.tar.gz` |
| Before speed-25 launcher experiment | `calibration-speed25-20261001-X8FU3G/task1_main.py` |
| Before CV lock fix | `cv-lock-fix-20261001-fSov7G/final_week_8_backup.py` |
| Before calibration rollback | `calibration-rollback-20261001-AnQww0/` |

The removed provisional calibration tests were preserved locally under
`tmp/calibration-reverted-20261001/`. They are historical, not the active suite.

When a Pi edit/deployment is requested:

1. Inspect the latest physical source and runtime; compare relevant hashes/diff.
2. Have the user stop the controller when a deployment requires it. Do not
   restart/drive the robot automatically.
3. Make a fresh, exact-target backup outside the live source directory.
4. Prepare a narrow patch against those current physical files. Check that its
   expected original content still matches before applying it.
5. Run appropriate offline checks and review the diff.
6. Mirror verified physical changes into Git without replacing newer Pi edits.
   Commit/push within the scope the user has authorised.
7. Give the user the restart and simple test commands appropriate to the change.

Do not run `scp -r RPi/* ...`, destructive reset/clean or a blind GitHub checkout
on the Pi. A Git push only updates GitHub, not the robot, tablet application or
already-running Python processes.

## 14. Verification already completed

The consolidation passed **52 Python tests**:

- 34 curated Pi tests: manual controls, stream/map protocol, ACK matching,
  capture/FIN serialization, capture/API failures and quiet telemetry.
- 13 Algo/CV tests: temporary model mapping, invalid images, output-directory
  recreation, 10 cm scaling, original turn geometry/padding and recorded maps.
- 5 launcher tests: explicit backends, read-only checks, occupied-port handling
  and cleanup of its own children.

The two recorded map payloads in `RPi-services/golden/expected_path.json`
produced exactly the expected adjusted STM command lists in isolated Flask
test clients. That validates the software baseline; it does not validate
physical distances, collision clearance, heading or original ONNX accuracy.

Commands, from repository-root PowerShell with the working environment:

```powershell
.\Algo\.venv-cv-win\Scripts\python.exe -B RPi/tests/test_offline.py
.\Algo\.venv-cv-win\Scripts\python.exe -B -m unittest discover -s Algo/tests -v
.\Algo\.venv-cv-win\Scripts\python.exe -B -m unittest discover -s RPi-services/tests -v
```

The Pi offline runner blocks serial construction and mocks hardware. Do not
discover/run every `test*.py` in the actual old Pi source directory: several
historical diagnostics send real motor commands on import.

One earlier test import caused Ultralytics to install optional `pi-heif 1.4.0`
in the ignored local CV environment. The unnecessary Algo ML imports were
removed; the path regression test disables Ultralytics auto-installation.

Android verification succeeded without installing an APK on the tablet:

```powershell
cd "C:\Users\ongjc\Documents\MDP\26S1-SC2079-Multidisciplinary-Project-Group-24\android\MDPGroup24"
$env:JAVA_HOME = 'C:\Program Files\Android\Android Studio\jbr'
.\gradlew.bat :app:assembleDebug :app:testDebugUnitTest --no-daemon --console=plain
```

The local SDK path is `C:\Users\ongjc\AppData\Local\Android\Sdk`.
The checked project uses compile/target SDK 37, minimum SDK 24 and AGP 9.3.1.
Those are the repository's observed settings, not a recommendation to upgrade
another machine before inspecting it.

`laptop_verify.py` is an optional original-backend regression tool. It compares
recorded paths, original ONNX image results/confidences and JPEG stitching.
Do not expect temporary best.pt to match ONNX confidences or describe a passing
software replay as proving the robot completed a physical 3/3 course.

## 15. First checks for the next conversation

Start with the latest user request, this handover and the current files.
Avoid rerunning the entire setup or redoing completed merges automatically.

Useful read-only checks:

```powershell
git status --short --branch
git branch -avv
git stash list
ipconfig
Invoke-RestMethod http://127.0.0.1:8000/status
Invoke-RestMethod http://127.0.0.1:8001/health
```

On the Pi, inspect source/runtime using SSH. If there are multiple controller
PIDs, inspect the process tree before attempting any shutdown. If SSH times
out, inspect laptop Wi-Fi and routing first rather than overwriting/configuring
the Pi from a stale local copy.

Read the relevant current files before a change:

| Topic | Files |
| --- | --- |
| Current startup and layout | `README.md`, `RPi/README.md`, `RPi-services/README.md` |
| Git/deployment boundaries | `docs/REPOSITORY_GUIDE.md` |
| Active vs rejected calibration | `Algo/CALIBRATION.md`, historical `Algo/MOVEMENT_CALIBRATION.md` |
| Motion scaling and padding | `Algo/consts.py`, `Algo/helper.py` |
| Viewing candidates / mission planning | `Algo/entities/Entity.py`, `Algo/algo/algo.py`, `Algo/main.py` |
| Actual Task 1 runtime | Physical `task1_main.py`, `final_week_8_backup.py`, `communication/stm32.py` |
| Manual and Bluetooth | `RPi/manual_control.py`, `RPi/communication/android.py`, `android_protocol.py` |
| Current CV | `Algo/cv_trial_server.py`, `Algo/CV_TRIAL.md` |
| Original CV/setup | `RPi-services/image-api/main.py`, `laptop_setup.py` |
| Android feature source | `ConfigurationFragment.kt`, `HomeFragment.kt`, `ArenaGridView.kt`, `fragment_configuration.xml` |

`docs/RUNBOOK.md` and `docs/LEARNINGS.md` contain historical Mac-era instructions
and superseded configurations. `Algo/CV_TRIAL.md` also contains older discussion
of lock problems; its blanket historical statement that all such bugs are
unresolved must not override the deployed CV/ACK fixes and the specific remaining
issues in this handover. Inspect actual current code when documents disagree.

### Recent questions to preserve accurately

- “Do we go back to missed obstacles?” — Yes, one additional route for recorded
  scan failures after up to seven attempts at the original position; recent
  logs showed obstacle 5 being revisited. The new-run flag/reset and pose bugs
  remain.
- “How far away before it takes a snapshot?” — The verified example used a
  40 cm robot-centre-to-obstacle-grid separation; initial candidates are 30/40,
  retry candidates 40/50, with optional sideways offsets. The user means the
  actual lens-to-image gap; that exact physical distance has not been measured
  or configured as a fixed 10/20 cm trigger.
- “How do I check camera pictures?” — Open laptop `/stitch`, or the raw and
  annotated JPEGs in `output/cv-trial`, sorted by Date modified. For an API
  failure before saving a laptop copy, inspect the Pi raw JPEGs.
- “Why do all branches still show lots of files?” — A branch is a full project
  snapshot. The actual 19k pending-change problem came from an older branch's
  ignore rules. All retained branches were synchronised and persistent local
  ignores added, preserving archives/stashes and runtime files.

Continue from these facts and the new user's direction. Do not claim that
repository cleanup deployed the Pi fixes, installed the Android build, restored
the original ONNX model or completed physical calibration.
