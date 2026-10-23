# STM32 motor controller

[← Project overview](../README.md)

STM32F407 firmware controls the drive motors, steering servo and sensors. Two independent projects are retained because different firmware was flashed for the tasks.

| Project | CubeIDE name | Use |
| --- | --- | --- |
| `Task1/` | `MDP_Task1` | Task 1 movement commands and motor/sensor support. |
| `Task2/` | `MDP_Task2` | O1/O2 state machine, calibrated manoeuvres and Pi handoff. |

Both retain HAL/CMSIS, FreeRTOS, startup code, board configuration and linker scripts. The source submission excludes library/generated support while retaining the integrated application listings.

## Build and flash

1. Import the selected folder into STM32CubeIDE as an **existing project**.
2. Confirm its project name, build configuration and STM32F407 target.
3. Build and flash the resulting firmware using the available ST-LINK workflow.
4. Restart the board; keep the robot still during gyro calibration.
5. Start the matching Pi runner and laptop recognition service.

Board configurations are `Task1/MDP_Task1.ioc` and `Task2/MDP_Task2.ioc`. Do not import one task's configuration over the other task's source. Generated HEX/ELF files remain outside Git.

## Source guide

| Source | Purpose |
| --- | --- |
| `Core/Src/main.c` | Integrated motor, steering, sensor, serial and task logic in each project. |
| `Core/Inc/main.h`, `Core/Src/ir_sensor.c` | Application declarations and IR setup. |
| `Task2/Core/Inc/kc_task2.h` | Task 2 state machine and movement sequencing. |
| `Task2/Core/Inc/kc_fast_o1.h` | Fast O1 approach and bypass. |
| `Task2/Core/Inc/kc_o2_uturn.h` | Continuous O2 U-turn. |
| `Task2/Core/Inc/kc_entry_geometry.h` | Carpark-entry geometry helpers. |
| `Task2/Core/Src/kc_task2_platform.inc` | Adapter to motor, encoder, heading and braking routines. |

The selected `full-pi-return` mode transfers return/parking control to the Pi after O1/O2. It differs from a complete native firmware return.

## Portable host tests

From `STM32/Task2`, using GCC:

```bash
# Classic O1 regression; fast O1 has separate tests.
gcc -std=gnu11 -O2 -ICore/Inc -DKC_FAST_O1=0 test_task2.c -lm -o test_task2
./test_task2

gcc -std=gnu11 -O2 -ICore/Inc test_o1_range.c -lm -o test_o1_range
./test_o1_range

gcc -std=gnu11 -O2 -ICore/Inc test_platform_settle.c -lm -o test_platform_settle
./test_platform_settle

gcc -std=gnu11 -O2 -ICore/Inc test_fast_o1.c -lm -o test_fast_o1
```

On Windows, run the generated `.exe` files. GNU C11 supplies `M_PI`, used by the fast-O1 test.

Bounded nominal fast-O1 checks:

```bash
./test_fast_o1 1 100 60 1 1.25 0.013 0 400 50
./test_fast_o1 2 100 60 1 1.25 0.013 0 400 50
```

Arguments: arrow, initial range, obstacle gap, radius scale, steering-lag scale, brake decay, lateral error, arrow arrival time and O2 half-width. Inspect the printed `ok`, clearance, heading and endpoint metrics: single-scenario mode returns zero regardless of those printed values.

## Calibration and test limits

Motor settings and turn geometry reflect the selected robot calibration. Hardware, steering limits, tyres, battery and floor surface can affect the physical path.

The no-argument fast-O1 stress sweep explores over 90,000 parameter combinations and includes clearance-limit failures. It is not an all-pass guarantee. The cleanup preserved the selected firmware logic rather than changing calibration to fit simulations. Host tests, nominal cases and assessment results describe different evidence.

Existing STM32, CMSIS and FreeRTOS notices remain with their components.
