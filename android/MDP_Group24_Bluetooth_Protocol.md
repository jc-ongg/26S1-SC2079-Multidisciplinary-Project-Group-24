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
| Start challenge | `//BEGIN,1//` | Start the selected challenge task. The task number is `1` or `2`. |
| Challenge result | `//RESULT,1,01:23.45//` | Stop the selected challenge task and send its elapsed time in `mm:ss.cc` format. |

### Challenge message fields

```text
//BEGIN,<task>//
//RESULT,<task>,<elapsed-time>//
```

- `<task>` is `1` or `2`, matching the task selected in the Android app.
- `<elapsed-time>` uses `mm:ss.cc`, where `cc` is centiseconds.
- The task number in `RESULT` identifies the same task that was started by `BEGIN`.

---

## Full Map

Example:

```text
//MAP/ROBOT,3,3,N/OBSTACLE,1,5,7,E/OBSTACLE,2,10,12,S//
```

This is the complete current Android map. The RPi should treat it as an authoritative snapshot and rebuild its map state from it.

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
