# MDP Group 24 Bluetooth Message Protocol

This document defines the Bluetooth message format used between the Android app and the RPi side.

## 1. Global Framing Rule

All machine/protocol messages use:

```text
//<payload>//
```

Examples:

```text
//ROBOT,2,3,N//
//TARGET,4,11//
//STATUS,Looking for target 2//
```

### Rules

- `//` marks the start and end of one logical message.
- No newline is required.
- `//` must not appear inside the payload.
- Normal `/` may still appear inside a payload, such as in the full `MAP` message.
- Directions use `N`, `E`, `S`, `W`.
- Direction `-1` means deletion/removal.
- Human Chat messages may remain raw text and are separate from the machine protocol.

---

# 2. Inbound Messages: RPi → Android

## 2.1 Robot Position Update

Format:

```text
//ROBOT,<x>,<y>,<direction>//
```

Example:

```text
//ROBOT,8,12,N//
```

Meaning:

- Robot centre is `(8,12)`.
- Robot direction is North.

Android should:

- Move the robot on the arena.
- Update its direction.
- Update the Vessel coordinates.
- Update the Vessel direction.
- Not add this update to Android Undo history.

Valid directions:

```text
N
E
S
W
```

---

## 2.2 Target Recognition

Format:

```text
//TARGET,<obstacleId>,<targetId>//
```

Example:

```text
//TARGET,3,11//
```

Meaning:

- Obstacle ID `3` detected Target ID `11`.

Android should:

- Find obstacle ID `3`.
- Keep it internally as obstacle ID `3`.
- Display Target ID `11` prominently on that obstacle.
- Preserve the target-facing side indicator.

---

## 2.3 Status Update

Format:

```text
//STATUS,<text>//
```

Examples:

```text
//STATUS,Ready to start//
//STATUS,Looking for target 2//
//STATUS,Returning to start//
```

Android should display everything after `STATUS,` in the Vessel status area.

If Bluetooth disconnects:

```text
Task Status = --
```

The RPi side decides which meaningful status messages to send.

---

# 3. Outbound Messages: Android → RPi

## 3.1 Manual Robot Movement

Forward:

```text
//F//
```

Backward:

```text
//B//
```

Rotate left:

```text
//L//
```

Rotate right:

```text
//R//
```

---

## 3.2 Robot Configuration / Placement

Format:

```text
//ROBOT,<x>,<y>,<direction>//
```

Example:

```text
//ROBOT,3,3,N//
```

Meaning:

- Android configured the robot at centre `(3,3)`.
- Robot is facing North.

---

## 3.3 Robot Removal

Format:

```text
//ROBOT,<x>,<y>,-1//
```

Example:

```text
//ROBOT,3,3,-1//
```

Meaning:

- Remove the robot currently configured at `(3,3)`.

Typical use:

- Undo removes the robot.

---

## 3.4 Obstacle Placement

Format:

```text
//OBSTACLE,<id>,<x>,<y>,<direction>//
```

Example:

```text
//OBSTACLE,1,5,7,E//
```

Meaning:

- Obstacle ID = `1`
- Position = `(5,7)`
- Target-facing side = East

---

## 3.5 Obstacle Movement

Same format:

```text
//OBSTACLE,<id>,<newX>,<newY>,<direction>//
```

Example:

```text
//OBSTACLE,1,8,10,E//
```

Send this once when the drag is committed/released.

Do not send continuously during drag movement.

---

## 3.6 Obstacle Target-Face / Orientation Update

Same format:

```text
//OBSTACLE,<id>,<x>,<y>,<newDirection>//
```

Example:

```text
//OBSTACLE,2,10,6,N//
```

The RPi side should treat this as an update to the existing obstacle state.

---

## 3.7 Obstacle Removal

Format:

```text
//OBSTACLE,<id>,<x>,<y>,-1//
```

Example:

```text
//OBSTACLE,3,8,10,-1//
```

Meaning:

- Delete Obstacle `3`.

Use this for:

- Manual deletion.
- Dragging an obstacle outside the arena.
- Undo removing an obstacle.

No separate `//UNDO//` command is required.

---

# 4. Full Map Messages

## 4.1 Full Map Snapshot

Format:

```text
//MAP[/ROBOT,...][/OBSTACLE,...]...//
```

Example:

```text
//MAP/ROBOT,3,3,N/OBSTACLE,1,5,7,E/OBSTACLE,2,10,12,S//
```

Meaning:

- This is the complete authoritative current Android map.

Recommended RPi-side handling:

1. Receive `MAP`.
2. Clear previous map state.
3. Parse every following segment.
4. Rebuild the map from the snapshot.

---

## 4.2 Empty Map / Reset

Format:

```text
//MAP//
```

Meaning:

- No robot exists.
- No obstacles exist.
- The map has been reset.

The Android `RESET MAP` action should send:

```text
//MAP//
```

No separate `//RESET//` command is required.

---

# 5. Obstacle ID Rule

Obstacle IDs should only increase during one map session.

Example:

```text
Place -> 1
Place -> 2
Place -> 3
Delete 3
Next -> 4
```

Likewise:

```text
Place -> 1
Place -> 2
Place -> 3
Undo 3
Next -> 4
```

Only `RESET MAP` resets obstacle numbering back to:

```text
1
```

This avoids reusing IDs that the RPi side may already have seen.

---

# 6. Summary Table

| Direction | Message | Meaning |
|---|---|---|
| RPi → Android | `//ROBOT,8,12,N//` | Update robot pose |
| RPi → Android | `//TARGET,3,11//` | Obstacle 3 detected Target ID 11 |
| RPi → Android | `//STATUS,Looking for target 2//` | Update task status |
| Android → RPi | `//F//` | Move forward |
| Android → RPi | `//B//` | Move backward |
| Android → RPi | `//L//` | Rotate left |
| Android → RPi | `//R//` | Rotate right |
| Android → RPi | `//ROBOT,3,3,N//` | Configure robot |
| Android → RPi | `//ROBOT,3,3,-1//` | Remove robot |
| Android → RPi | `//OBSTACLE,1,5,7,E//` | Add/update obstacle |
| Android → RPi | `//OBSTACLE,1,5,7,-1//` | Remove obstacle |
| Android → RPi | `//MAP/...//` | Full authoritative map |
| Android → RPi | `//MAP//` | Empty/reset map |

---

# 7. RPi Parsing Guidance

Treat `ROBOT` and `OBSTACLE` messages as state updates.

Example:

```text
//OBSTACLE,2,5,5,E//
```

followed later by:

```text
//OBSTACLE,2,8,9,N//
```

means:

- Update Obstacle `2` from its old state to `(8,9,N)`.

Then:

```text
//OBSTACLE,2,8,9,-1//
```

means:

- Delete Obstacle `2`.

For framed messages, the parser should:

1. Search for opening `//`.
2. Search for the next closing `//`.
3. Extract the payload between them.
4. Process the payload.
5. Continue parsing the remaining buffer in case multiple framed messages arrived together.

Example combined stream:

```text
//STATUS,Moving////ROBOT,5,8,E////TARGET,2,15//
```

This contains three separate messages and should be parsed as:

```text
STATUS,Moving
ROBOT,5,8,E
TARGET,2,15
```
