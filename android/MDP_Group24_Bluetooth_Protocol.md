# MDP Group 24 Bluetooth Protocol

## Framing

All machine messages use:

```text
//<payload>//
```

- No newline required.
- `//` is reserved for message framing.
- Normal `/` may appear inside a payload.
- Directions: `N`, `E`, `S`, `W`.
- Direction `-1` means remove/delete.
- Chat messages may remain raw text and are separate from this protocol.

---

## RPi → Android

| Purpose | Example | Description |
|---|---|---|
| Robot update | `//ROBOT,8,12,N//` | Move robot centre to `(8,12)` facing North. Does not enter Android Undo history. |
| Target detected | `//TARGET,3,11//` | Obstacle `3` detected target ID `11`. Android displays the target ID on that obstacle. |
| Status update | `//STATUS,Looking for target 2//` | Display the text after `STATUS,` as the current task status. |
| Prepared route | `//PLAN,abc123,READY//` | The route for this map request is stored on the Pi; Task 1 Start may be enabled. |
| Preparation failed | `//PLAN,abc123,ERROR,Algo unreachable//` | Keep Start disabled; show the failure and send the map again. |

---

## Android → RPi

| Purpose | Example | Description |
|---|---|---|
| Forward | `//F//` | Move robot forward. |
| Backward | `//B//` | Move robot backward. |
| Rotate left | `//L//` | Rotate robot left. |
| Rotate right | `//R//` | Rotate robot right. |
| Robot placement/update | `//ROBOT,3,3,N//` | Robot centre is `(3,3)`, facing North. |
| Robot removal | `//ROBOT,3,3,-1//` | Remove the robot at `(3,3)`. |
| Obstacle placement/update | `//OBSTACLE,1,5,7,E//` | Obstacle `1` is at `(5,7)`, facing East. Also used after moving or changing orientation. |
| Obstacle removal | `//OBSTACLE,1,5,7,-1//` | Remove obstacle `1`. Used for delete, drag-outside, or Undo. |
| Start Task 1 | `//BEGIN,1,abc123//` | Execute the prepared route with this request ID; do not recompute it. |
| Challenge result | `//RESULT,1,01:23.45//` | Stop the selected challenge task and send its elapsed time in `mm:ss.cc` format. |

### Challenge message fields

```text
//BEGIN,1,<map-request-id>//
//RESULT,<task>,<elapsed-time>//
```

- `<task>` is `1` or `2`, matching the task selected in the Android app.
- `<elapsed-time>` uses `mm:ss.cc`, where `cc` is centiseconds.
- The task number in `RESULT` identifies the same task that was started by `BEGIN`.
- Task 1's Pi runner rejects Task 2 BEGIN messages; Task 2 needs its own runner.
- `RESULT` halts further Task 1 dispatch. An in-flight movement may finish; this is not an emergency stop.
- `STATUS,finished` stops the Android timer automatically after results have been applied to the map.

---

## Full Map

Example:

```text
//MAP,abc123/ROBOT,3,3,N/OBSTACLE,1,5,7,E/OBSTACLE,2,10,12,S//
```

This is the complete current Android map. The RPi should treat it as an authoritative snapshot and rebuild its map state from it.

MAP immediately requests `/path` on the laptop. The Pi validates `stm_commands`
and the corresponding `command_poses`, queues them without moving, and replies
with `PLAN,<map-request-id>,READY`. Android only accepts the latest pending ID.
Edits, reset, saved-layout loading, manual movement and Bluetooth reconnection
invalidate readiness. Send the map again before another run.

Robot positions are attached to actual motor commands and reported after a
matching STM completion. Padding inside a turn has no future waypoint attached.
These positions are estimates from the planner, not measured odometry.
Connected manual controls update the tablet after `STATUS,Manual forward complete`
(or backward/left/right); disconnected controls remain a local simulator.

TARGET frames update both the map and timestamped Chat history. Chat retains the
latest 500 entries across tab/view recreation within the current app session;
it is not a saved transcript after app termination.

Reset / empty map:

```text
//MAP//
```

This means there is no robot and no obstacles.

---

## Obstacle ID Behaviour

- Normal placement increments IDs: `1`, `2`, `3`, ...
- Manual delete or dragging an obstacle outside **does not reuse** its ID.
- Undo of the latest obstacle **does reuse** that latest ID.
- Reset Map starts numbering from `1` again.

Example:

```text
Place 1
Place 2
Place 3
Undo 3
Next obstacle = 3
```

But:

```text
Place 1
Place 2
Place 3
Delete 3 manually
Next obstacle = 4
```

---

## Parsing Note

Bluetooth is a byte stream, so one read may contain part of a message or several messages together.

Example:

```text
//STATUS,Moving////ROBOT,5,8,E////TARGET,2,15//
```

Parse each `//...//` frame separately.
