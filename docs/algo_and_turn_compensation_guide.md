# Algorithm & Turn Compensation Reference Guide

This document details all recent code changes, why they were made, how they interact with the robot hardware, how to calibrate/update them in the future, and the complete step-by-step procedure to run tests with the servers and Android tablet.

---

## 1. Summary of Updated Files

It is not only [algo.py](file:///c:/Users/ethan/Downloads/MDP/26S1-SC2079-Multidisciplinary-Project-Group-24/Algo/algo/algo.py). There are **3 files** updated in the `Algo/` module:

| File | Role | Changes Made |
| :--- | :--- | :--- |
| [Algo/consts.py](file:///c:/Users/ethan/Downloads/MDP/26S1-SC2079-Multidisciplinary-Project-Group-24/Algo/consts.py) | Physical Turn & Grid Calibrations | Defines pre-turn and post-turn straight padding distances (in cm) so turns land on 5 cm grid multiples. |
| [Algo/helper.py](file:///c:/Users/ethan/Downloads/MDP/26S1-SC2079-Multidisciplinary-Project-Group-24/Algo/helper.py) | Command Translation & Merging | Merges consecutive straight movements in the same direction into a single smooth command so the robot does not stutter. |
| [Algo/algo/algo.py](file:///c:/Users/ethan/Downloads/MDP/26S1-SC2079-Multidisciplinary-Project-Group-24/Algo/algo/algo.py) | Pathfinding & Route Assembly | Defensive route cache check to prevent crashes if an obstacle configuration produces an unreachable segment. |

> [!NOTE]
> **STM32 Firmware (`main.c`) is completely untouched.**
> All physical compensation is handled upstream in Python. Modifying STM32 firmware for turn padding previously induced severe tire scrub and widened the turn arc.

---

## 2. Detailed Code Changes & How They Work

### A. Turn Compensation in [Algo/consts.py](file:///c:/Users/ethan/Downloads/MDP/26S1-SC2079-Multidisciplinary-Project-Group-24/Algo/consts.py#L39-L42)

#### The Physical Problem
The robot operates on a 5 cm grid. A standard 90-degree turn should ideally land exactly on a 5 cm boundary (e.g. 25 cm x 35 cm). However, bare hardware tests revealed:
- **Forward Right Turn (`TURNR`):** Moves 22 cm straight forward, 35 cm rightwards. (Missing 3 cm straight).
- **Forward Left Turn (`TURNL`):** Moves 17 cm straight forward, 30 cm leftwards. (Missing 3 cm straight).
- **Reverse Right Turn (`REVR`):** Moves 35 cm backwards, 23 cm rightwards. (Missing 2 cm right).
- **Reverse Left Turn (`REVL`):** Moves 30 cm backwards, 15 cm leftwards. (Already exact multiples of 5 cm!).

#### The Code Change
In [Algo/consts.py](file:///c:/Users/ethan/Downloads/MDP/26S1-SC2079-Multidisciplinary-Project-Group-24/Algo/consts.py):
```python
# Format: (pre_turn_straight_cm, post_turn_straight_cm)
FORWARD_RIGHT = (3, 0) # 3 cm forward before turn -> 25 cm straight, 35 cm right (5x7 grid)
FORWARD_LEFT  = (3, 0) # 3 cm forward before turn -> 20 cm straight, 30 cm left  (4x6 grid)
REVERSE_RIGHT = (0, 2) # 2 cm reverse after turn -> 35 cm back, 25 cm right (7x5 grid)
REVERSE_LEFT  = (0, 0) # Pure turn -> 30 cm back, 15 cm left (6x3 grid)
```

#### How It Affects Motion
When the algorithm plans a forward-right turn, it issues:
1. `center,0,forward,3` (Robot rolls straight forward 3 cm).
2. `right,0,forward,0` (Robot executes physical 90-degree turn).
Because the 3 cm straight forward movement is executed with front wheels straight, there is zero tire scrubbing, and the turn lands at exactly 25 cm forward and 35 cm right.

---

### B. Straight Command Merging in [Algo/helper.py](file:///c:/Users/ethan/Downloads/MDP/26S1-SC2079-Multidisciplinary-Project-Group-24/Algo/helper.py#L376-L393)

#### The Stuttering Problem
Without command merging, if the robot needed to drive 20 cm forward and then do a forward-right turn (which needs +3 cm forward padding), the command generator would emit:
1. `center,0,forward,20` -> STM32 executes `FWD 20` (accelerates, decelerates, stops).
2. `center,0,forward,3` -> STM32 executes `FWD 3` (accelerates, stops).
3. `right,0,forward,0` -> STM32 executes `TURNR`.

This caused an awkward pause/jerk at the 20 cm mark.

#### The Code Change
In [Algo/helper.py](file:///c:/Users/ethan/Downloads/MDP/26S1-SC2079-Multidisciplinary-Project-Group-24/Algo/helper.py#L376-L393):
```python
# check if both are straight movement commands
if curr[0] == "center" and prev_cmd[0] == "center":
    curr_dist = int(curr[3])
    prev_dist = int(prev_cmd[3])

    if curr[2] == prev_cmd[2]:
        # Same direction: combine distances into single smooth movement
        prev_cmd = ["center", "0", curr[2], str(prev_dist + curr_dist)]
    else:
        # Opposite direction: cancel out
        diff = abs(curr_dist - prev_dist)
        if diff == 0:
            prev_cmd = None
        else:
            direction = curr[2] if curr_dist > prev_dist else prev_cmd[2]
            prev_cmd = ["center", "0", direction, str(diff)]
```

#### How It Affects Motion
`center,0,forward,20` and `center,0,forward,3` are merged into **`center,0,forward,23`**.
The STM32 receives a single command: `MOTOR/FWD/20/23;`.
The robot drives smoothly in one single deceleration profile right into the turn position without any stutter.

---

### C. Defensive Route Cache in [Algo/algo/algo.py](file:///c:/Users/ethan/Downloads/MDP/26S1-SC2079-Multidisciplinary-Project-Group-24/Algo/algo/algo.py#L203-L218)

#### The Problem
During route recalculation or complex obstacle placements, if an obstacle target had no valid path (e.g. boxed in by obstacles), attempting to look up `self.route_cache[(actual_current_state, target_node)]` would raise an unhandled `KeyError` and crash the FastAPI server.

#### The Code Change
```python
if (actual_current_state, target_node) not in self.route_cache:
    self._run_astar(actual_current_state, target_node)
if (actual_current_state, target_node) not in self.route_cache:
    current_path = []
    break
```

#### How It Affects Operation
The algorithm fails gracefully instead of crashing the server, allowing the API to return an empty path or error status cleanly.

---

## 3. How to Calibrate / Update ("Uprate") in the Future

If physical arena conditions change (e.g. fresh battery voltage, different arena carpet friction, tire wear) and turns begin over- or under-shooting:

### Step 1: Run Bare Turn Tests
Send a pure turn command directly from RPi or Serial GUI without any padding:
- Forward Right: `:MOTOR/TURNR/20/0;`
- Forward Left: `:MOTOR/TURNL/20/0;`
- Reverse Right: `:MOTOR/REVR/20/0;`
- Reverse Left: `:MOTOR/REVL/20/0;`

Measure with a measuring tape:
- **Longitudinal distance:** Forward/backward displacement from start center to end center.
- **Lateral distance:** Sideways displacement from start center to end center.

### Step 2: Calculate Needed Padding
Target both displacements to the nearest multiple of 5 cm.
Example:
- If Forward Right measures **21 cm forward** and **35 cm right**:
  - Longitudinal target = 25 cm. Difference = `25 - 21 = +4 cm`.
  - Update `FORWARD_RIGHT = (4, 0)` in [Algo/consts.py](file:///c:/Users/ethan/Downloads/MDP/26S1-SC2079-Multidisciplinary-Project-Group-24/Algo/consts.py).

### Tuple Format Reference:
```python
FORWARD_RIGHT = (pre_forward_cm, post_forward_cm)
FORWARD_LEFT  = (pre_forward_cm, post_forward_cm)
REVERSE_RIGHT = (pre_backward_cm, post_backward_cm)
REVERSE_LEFT  = (pre_backward_cm, post_backward_cm)
```
- First number: straight distance added **before** the turn.
- Second number: straight distance added **after** the turn.

---

## 4. Complete System Setup & Test Run Procedure

Follow these steps to run an end-to-end test on the arena.

### Step 1: Physical Setup
1. Place robot on arena (usually grid cell (1, 1), heading North).
2. Turn on robot motor battery switch and STM32 power switch.
3. Power on Raspberry Pi.
4. Ensure the Raspberry Pi Wi-Fi hotspot is running (e.g. `MDP-Group-24` or similar).

### Step 2: Laptop Connection & Server Launch
1. Connect your Windows laptop to the Raspberry Pi Wi-Fi hotspot.
2. Open PowerShell and check your laptop IP address:
   ```powershell
   ipconfig
   ```
   Note the IPv4 address under the Wi-Fi adapter (e.g. `192.168.50.181`).
3. Start the Laptop Algo & CV services using the launcher:
   ```powershell
   & "C:\Python313\python.exe" -u RPi-services/laptop_start.py --python "C:\Python313\python.exe" --cv-backend trial
   ```
   *(Use `--cv-backend trial` for mock image recognition, or `--cv-backend image-api` if using full YOLO model).*
4. Verify both services start:
   - Algo Server: `http://localhost:8000` (or `http://<laptop-ip>:8000`)
   - CV Server: `http://localhost:8001` (or `http://<laptop-ip>:8001`)

### Step 3: Raspberry Pi Controller Launch
1. SSH into the Raspberry Pi from your laptop:
   ```bash
   ssh pi@192.168.50.1
   ```
2. Navigate to the RPi project directory:
   ```bash
   cd ~/RPi-services
   ```
3. Start Task 1 controller, pointing to your laptop IP:
   ```bash
   LAPTOP_IP=192.168.50.181 python3 -u task1_main.py
   ```
   *(Replace `192.168.50.181` with your actual laptop IP from Step 2).*
4. The Pi will initialize:
   - USB Serial connection to STM32 (`/dev/ttyUSB0` at 115200 baud).
   - Bluetooth RFCOMM server for Android.
   - HTTP clients to Laptop Algo & CV servers.

### Step 4: Android Tablet Connection
1. Open the **MDPGroup24** app on the Android tablet.
2. Go to Bluetooth settings in the app and connect to Raspberry Pi (`Group 24`).
3. Once connected, place obstacles on the tablet arena grid.
4. Tap **Send Map** (this sends `//MAP/...//` to RPi, which forwards it to Algo server on laptop).
5. The Algo server computes the path, merges micro-movements, and sends the path back to Android for visualization.
6. When ready, tap **Start** (sends `//BEGIN,1//`).
7. Watch the robot execute smooth, calibrated turns without cumulative grid drift!
