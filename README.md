# mtrn3100-micromouse

Firmware and offline tooling for a micromouse robot built for MTRN3100, running on an Arduino Nano (ATmega328). The robot navigates a 9x9 maze using two encoded DC motors, an IMU, three time-of-flight distance sensors, and an OLED status display.

## Hardware

- **MCU:** Arduino Nano (`nanoatmega328`)
- **Motors:** 2x DC motor with quadrature encoders (700 counts/rev), driven via PWM + direction pins
- **IMU:** MPU6050 (heading only, via `MPU6050_light`)
- **Distance sensors:** 3x VL6180X time-of-flight lidar (front, left, right), on a shared I2C bus with per-sensor enable pins for address assignment
- **Display:** SSD1306 128x64 OLED (via `U8g2`/`U8x8`)

## Firmware architecture

- `include/` — headers declaring interfaces only (classes and free functions); no implementation bodies.
- `src/` — one `.cpp` per header, plus `main.cpp` (the PlatformIO entry point with `setup()`/`loop()`).

Core building blocks:

- `Motor`, `Gyroscope`, `LidarSystem`, `OLED`, `PIDController` — thin wrappers around each piece of hardware.
- `Robot` — a singleton (`GET_ROBOT()`) owning one instance of every subsystem plus the PID controllers used for rotation, forward position, heading-hold, and wall-centering.
- `Movement` — the motion primitives everything else is built from: `robot_rotate`, `robot_drive_straight_with_lidars_no_profile_soft_start` (lidar wall-centering + front-wall stop), `robot_drive_straight_no_lidars_soft_start` (IMU-only), `robot_align`, and `chaining` (runs an `'f'`/`'r'`/`'l'` move string).
- `AutoMapping` — DFS maze exploration that builds a wall map on the fly, then BFS's the shortest path from start to goal.
- `Misc` — shared constants/utilities (`RingBuffer`, `Stack`, `delayWhileUpdating`, etc).

### Task selection

`include/Task.hpp` defines `TASK_4_POINT`, which selects which assessment task's code compiles and runs:

| `TASK_4_POINT` | Entry point | Behaviour |
|---|---|---|
| `1` | `do_maze_completion()` (`MazeCompletion.hpp`) | Task 4.1 — drives a hardcoded move sequence through the maze and times it. |
| `2` | `do_cont_planning()` (`ContPlanning.hpp`) | Task 4.2 — drives a hardcoded rotate/drive sequence for continuous path planning. |
| `3` | `do_auto_mapping()` (`AutoMapping.hpp`) | Task 4.3 — autonomously maps the maze via DFS, then drives the shortest path to the goal. |

`Movement`'s tuning (PID gains, soft-start ramp times, lidar smoothing) also varies per task, since each task's tolerances and speeds were tuned separately.

## Building & flashing

This is a [PlatformIO](https://platformio.org/) project.

```bash
pio run              # build
pio run --target upload   # flash to the Nano
pio device monitor   # serial monitor
```

Set `TASK_4_POINT` in `include/Task.hpp` before building to choose which task's code runs.

## `computer_vision/`

Offline Python/Jupyter tooling used alongside the firmware, kept separate from the embedded build:

- `image_processing.py`, `hsv_picker.py`, `pick_points.py` — overhead-camera maze image processing (colour marker detection, perspective correction).
- `graph.py`, `path_finder.py` — maze graph representation and shortest/optimal-command path search, used to precompute the hardcoded move sequences for Tasks 4.1/4.2.
- `*.ipynb` — notebooks used to develop and visualise the above during the project.

## Other directories

- `lib/`, `test/` — PlatformIO's standard project-library and unit-test scaffolding (currently unused).
- `configure_platform.py` — PlatformIO pre-build script; sets the upload port on Windows and stamps a `BUILD_TIMESTAMP` define.
