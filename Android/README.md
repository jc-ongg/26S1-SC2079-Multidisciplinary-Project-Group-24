# Android tablet application

[← Project overview](../README.md)

One Kotlin application provides arena configuration, Bluetooth communication and task controls. Its arena uses **20 × 20 cells**, with **10 cm per cell**.

## Build and install

1. Open this `Android` directory in Android Studio.
2. Resolve the Gradle wrapper/project and install the configured SDK.
3. Build and install on the tablet.
4. Enable Bluetooth, pair with the Pi and connect from the app.

| Setting | Value |
| --- | --- |
| Application ID | `com.example.mdpgroup24` |
| Minimum SDK | 24 |
| Compile / target SDK | 37 |
| Gradle wrapper | Included in this directory. |
| Task support | Shared app; task-specific Pi/firmware runners. |

The Gradle daemon toolchain is configured in `gradle/gradle-daemon-jvm.properties`. Keep SDK paths in local configuration rather than committing `local.properties`.

## Task 1 operation

1. Set the robot start position/heading and obstacle positions/image-facing directions.
2. Connect and send the map.
3. Wait for **Route ready** with a confirmation matching the latest request.
4. Press Start. Robot positions update after corresponding motor completions; image IDs appear against their obstacles.

Each submission gets a new request ID. Map edits/connection changes invalidate preparation. The app waits up to **190 seconds**; a standalone status message does not enable Start.

Saved layouts are JSON in SharedPreferences. Bluetooth uses framed text:

```text
//MAP,example123/ROBOT,1,1,N/OBSTACLE,1,5,7,E//
//PLAN,example123,READY//
//BEGIN,1,example123//
```

See [BLUETOOTH_PROTOCOL.md](BLUETOOTH_PROTOCOL.md). The Task 1 Pi controller handles READY; Task 2's selected runner is described in the [Pi guide](../RPi/README.md#run-task-2).

## Code guide

Source: `app/src/main/java/com/example/mdpgroup24/`.

| Area | Purpose |
| --- | --- |
| `MainActivity.kt` | Application shell and navigation. |
| `fragments/` | Arena, configuration, Bluetooth, chat, controls and challenge UI. |
| `views/ArenaGridView.kt` | Robot/obstacle grid display and interaction. |
| `bluetooth/BluetoothSession.kt` | Connection, preparation and message handling. |
| `bluetooth/Task1Preparation.kt` | Latest-map READY/Start state. |
| `bluetooth/BluetoothFrameBuffer.kt` | Framing fragmented/combined Bluetooth messages. |

Resources, layouts and build configuration stay in the repository. The course archive contains application source listings instead of those resources.

## Validation

From `Android`, with the configured Java toolchain and SDK:

```powershell
.\gradlew.bat testDebugUnitTest assembleDebug
```

On Linux/macOS, use `./gradlew testDebugUnitTest assembleDebug`.

The debug build and eight local unit tests passed when the final layout was checked. Tablet/Pi pairing and real robot operation require hardware and are not covered by JVM tests.
