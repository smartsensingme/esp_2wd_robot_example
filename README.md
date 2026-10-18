# ESP 2WD robot — board application

ESP-IDF 6.0 application for an **ESP32-C3 SuperMini** differential-drive chassis,
a DRV8833 bridge and two MT6701 wheel encoders. This application owns wiring,
hardware lifetime, native USB console, the maintenance button and a bounded PID
bench trial. The reusable [esp_2wd_robot](components/esp_2wd_robot/README.md)
component owns control, odometry, maintenance acquisition and its wire protocol;
it is not exclusive to C3, although C3 is the currently validated robot target.

Boot does not command motion. The console waits for an explicit operation.
ULT/nFAULT is informational in this application, including unknown contexts and
persistent LOW. Sensor validity, output API errors and timing failures retain
their separate stop policies. There is no IMU, outer heading/position controller,
general teleoperation interface or command watchdog in this bench application.

## Repository and reproducible setup

The root application is a separate Git repository. `components/` contains pinned
Git submodules; `.gitmodules` uses public HTTPS URLs. SSH account aliases belong
only in local `remote.origin.pushurl` settings and are not carried by a clone.
See [local component revisions](docs/local-components.md) for the dependency graph.

After obtaining this application repository:

```sh
git -C /path/to/application submodule update --init --recursive
```

Use ESP-IDF **6.0** and its C3 toolchain. From the application root, with the IDF
environment activated:

```sh
idf.py build
idf.py -p /dev/cu.usbmodemYOURPORT flash
idf.py -p /dev/cu.usbmodemYOURPORT monitor
```

Replace the example port with the connected board's port. Close Octave before
opening a monitor and close the monitor before Octave. Flashing writes firmware;
it does not intentionally erase LUT storage. Do not use erase-flash to repair a
routine startup/serial failure: existing per-sensor calibration lives in NVS.

Root CMake deliberately disables Component Manager and compiles the local pinned
submodules. Their manifests remain the installation mechanism for independent
component consumers, not the acquisition path used by this application.
`sdkconfig.defaults` selects C3, a 1000-Hz RTOS tick, USB Serial/JTAG console,
exclusive-owner MT6701 access and detailed diagnostics. Existing `sdkconfig`
settings take precedence over defaults. Configure the maintenance GPIO/LED in
`idf.py menuconfig` → **Robot application**; there is no boot-mode selector.

## Hardware and coordinates

Body +X is forward, +Y left and positive yaw counter-clockwise. Positive wheel
rad/s means forward after sign normalization. Geometry and limits are in
[main/board_config.h](main/board_config.h); sensor addresses are 7-bit I2C values.

| Item | Physical LEFT | Physical RIGHT |
| --- | --- | --- |
| Motor/bridge | Motor 2 / B | Motor 1 / A |
| MT6701 address | `0x06` | `0x46` |
| Encoder sign | +1 | -1 |
| Motor sign | +1 | +1 |
| NVS namespace | `robot_right` | `robot_left` |
| Wheel radius / sensed-shaft ratio | 0.03305 m / 1:1 | 0.03305 m / 1:1 |

The historical NVS names follow **sensor identity**, not their old side labels.
Do not swap tables or erase them. Old files named `cal_left` may refer to physical
RIGHT; old `cal_right` may refer to physical LEFT. Check capture dates/addresses
and the corrected mapping before reinstalling historical data.

| Shared connection | GPIO / setting |
| --- | --- |
| I2C SDA / SCL | 8 / 9, 400 kHz |
| Bridge IN1 / IN2 / IN3 / IN4 | 0 / 1 / 2 / 3, PWM 20 kHz |
| EEP / nSLEEP | 10; check the actual module sleep jumper |
| ULT / nFAULT | 7; informational telemetry only |
| Maintenance input | 5 by default, normally-open button to GND, internal pull-up |
| Optional external LED | Disabled by default; onboard GPIO8 LED shares SDA |

Wheel track is 0.1186 m. Board limits are 6 rad/s wheel reference, 4 rad/s² ramp,
75% PID duty and 100-ms brake duration. These are bench settings, not measured
motor ratings. The reported supply is 6 V. GPIO2/8/9 boot straps and external I2C
pull-ups require attention when wiring this particular board. See
[original wiring notes](docs/pins.txt); current physical-side mapping above and
in `board_config.h` supersedes older labels in notes/captures.

## Startup, ownership and console

`app_main()` installs native USB first (TX4096/RX1024 software rings), selects LF
endings and nonblocking stdin. Hardware initialization then opens NVS without
automatic erase, probes both encoders, loads independent LUTs and creates the
coasting bridge plus maintenance service/protocol. No periodic controller is
created until `pid`. Setup failures retain a queryable console response:

```text
ERR command=STARTUP code=ESP_ERR_INVALID_RESPONSE resource=right message=hardware_initialization_failed
```

The resource names the failed initialization side or phase; the error alone does
not diagnose wiring. Partial cleanup respects retained owner handles. USB setup
or button-worker setup failures use `ESP_ERROR_CHECK` and can abort instead of
entering this retained-error console.

| Command | Behavior |
| --- | --- |
| `help` | Print available console and maintenance commands |
| `ping` | Reply `OK command=PING` |
| `status` | Read raw encoders and report LUT state; no motion |
| `MODE` | Query normal/calibration state; does not enter maintenance |
| `pid` | Run the bounded both-wheel closed-loop trial in normal mode |
| `cal left`, `cal right` | Raw capture aliases; require maintenance already active |
| `CAL ...`, `ESTIMATE START ...` | Component maintenance protocol, described below |

Commands accept CR, LF or CRLF; command names are case-insensitive. The console
stores up to 255 characters and discards excess characters until the terminator.
Dispatch is synchronous: while a trial/capture is executing, incoming commands
are not processed concurrently. There is no interactive serial emergency-stop
command during these blocking experiments. GPIO requests are observed by the PID
supervisor; a raw capture runs its bounded component plan to completion or error.

The application owns all sensor/bus/bridge/LUT handles for its lifetime. Either
the controller owner or the synchronous maintenance caller uses them, never both
in the intended successful flow. Join/coast errors retain handles and prevent
maintenance entry. They do not authorize freeing/reusing borrowed hardware.

## Maintenance button and LED

A stable LOW on GPIO5 requests maintenance without reset. The priority-1 worker
polls every 10 ms and debounces both LOW acceptance and HIGH rearming for 50 ms.
It only accesses GPIO and atomic flags. The console performs mode entry after
joining any controller and confirming coast. Releasing the button does not exit
maintenance; reset with the input released returns to normal mode. A LOW held at
startup can also request entry. Button entry itself does not move either wheel.

| Kconfig option | Default / behavior |
| --- | --- |
| `ROBOT_CALIBRATION_GPIO` | 5; board permits free exposed pins 4, 5, 6, 20, 21 |
| `ROBOT_CALIBRATION_DEBOUNCE_MS` | 50; range 20..1000 ms, scheduling adds latency |
| `ROBOT_CALIBRATION_LED_GPIO` | -1 disables; use a free pin distinct from input |
| `ROBOT_CALIBRATION_LED_ACTIVE_LEVEL` | 1; choose external LED polarity |

An external LED needs a series resistor. It toggles every 250 ms after successful
entry (500-ms full period). The onboard GPIO8 LED cannot be used because it shares
encoder SDA. Invalid/colliding pins fail setup. Rebuild/flash for Kconfig changes.

## LUT calibration and filter evaluation

Suspend the chassis with the driven wheels free before any motion experiment.
Select maintenance with the button, close the serial monitor, then run Octave
from the application root:

```octave
calibrate_menu
```

The wrapper adds the robot tools path and opens the menu. Select the current USB
port; Octave queries `MODE` and requires calibration mode already active. There
is no reset handshake/window. The host package/dependency setup and all menu
options are documented in [the Octave guide](components/esp_2wd_robot/tools/octave/README.md).
Only one program may own the serial port.

Raw calibration records one wheel at 200 Hz, 70/90/70% open-loop duty for 8 seconds
per stage (24 seconds), with the previous LUT disabled for raw acquisition.
Kalman is not used to fit the LUT. Samples are stored in internal RAM, then the
motor is braked/coasted and the result transferred. Host analysis fits the LUT;
installation is an explicit user menu choice. Exact-byte readback/CRC verification
precedes enabling an uploaded table. Both-wheel calibration has been accepted
by the user; repeating calibration is not required by a software migration.

Options **10 LEFT / 11 RIGHT** instead evaluate smoothing/lag with the installed
LUT enabled, 70/90/70% for 4 seconds per stage (12 seconds). They require typed
`TESTAR LEFT` or `TESTAR RIGHT`, record direct/LPF/CV velocities and timing, and
write no LUT/NVS data. They are not calibration runs and do not use PID.

The component owns `CAL STATUS/START/WRITE/READ/ENABLE/DISABLE/CLEAR` and
`ESTIMATE START`; the application supplies exact-byte USB read/write/drain
callbacks. Binary writes bypass the VFS character writer, accumulate partial
native writes and report failure rather than pretending a truncated payload is
complete. Reads accumulate bytes from nonblocking stdin. A failed protocol
transfer suppresses its prompt; reopen/reconcile using length/CRC/status after
transport failures. Full framing, little-endian layout, CRC and error behavior
are documented in [the component protocol](components/esp_2wd_robot/README.md).
No USB transfer, formatting or flash write belongs in the acquisition loop.

## Current PID trial and estimator

Close Octave, release the maintenance input and reset to normal mode. With wheels
suspended, open the serial monitor, type `pid` and Enter. Firmware creates the
200-Hz owner, waits up to one second for valid angle/velocity feedback, arms and
confirms RUNNING before submitting +3 rad/s to both wheels. It observes at most
five seconds after reference submission, requests stop, waits 150 ms and attempts
destroy. No automatic fault clear/restart is performed.

Board PI gains are Kp=3, Ki=20, Kd=0. There is no fixed startup kick/feedforward;
the integral reaches the breakaway duty. Both CV filters use position sigma
0.05 degrees and acceleration sigma 5 rad/s². These are trial settings, not
identified noise statistics. Processing is native angle → LUT → unwrap/sign/ratio
→ Kalman velocity → PI. The LPF is bypassed when Kalman is selected; wheel
odometry uses corrected travel independently of velocity smoothing.

The supervisor polls state every 10 ms and saves at most 50 snapshots at 10 Hz.
Reports are disabled during motion. After the stop/join attempt it prints stored
left/right target, ramped/filtered/direct speed, duty, validity and final timing/
error counters. These sparse snapshots do not establish per-cycle tracking
statistics. If destroy fails, the handle remains alive; printed output is not
proof of a successful join or physical stop. Reset/recovery is required before
assuming another owner can safely reuse the hardware.

Up to two consecutive invalid sensor reads hold the existing output and skip
PID/estimator/odometry updates; the third on either side or stale feedback beyond
15 ms faults/coasts. Timing/backlog/output checks remain active. ULT events and
monitoring errors remain informational, counted separately by the application.

For offline analysis without moving wheels:

```sh
sh tools/build_velocity_replay.sh
```

```octave
addpath('components/esp_2wd_robot/tools/octave');
robot_analyze_velocity('tests', 'build/velocity-evaluation');
```

Existing MAT captures may be replayed using the same float estimator source as
firmware. Synthetic phase delay excludes physical sensor/driver/motor latency;
ripple is not absolute velocity error. No automatic filter choice or PID retune
is performed. See [timing](docs/control-timing.md) and
[validation](docs/validation.md) for evidence and remaining measurements.

## Source map and verification

| Path | Responsibility |
| --- | --- |
| `main/main.c` | Hardware adapters/lifetime, USB console, explicit PID bench |
| `main/board_config.h` | Wiring, physical mapping, geometry and board limits |
| `main/robot_fault_policy.h` | Informational-only ULT classification/policy |
| `main/board/` | GPIO debounce worker, atomic request and optional LED |
| `main/Kconfig.projbuild` | Board maintenance input/indicator options |
| `calibrate_menu.m` | Application-root Octave entry point |
| `tools/` | Native regression/replay build runners |
| `tests/` | Application regression fixtures and optional user measurement files |
| `components/` | Independent, pinned reusable components |

The encoder-only identification/recovery experiment is a separate driver project
at `components/esp-mt6701/test_apps/dual_i2c`; it is not a main boot mode.

```sh
sh tools/test.sh
python3 tests/test_calibration_mode_serial.py
python3 tests/test_velocity_evaluation_serial.py
idf.py build
```

The optional `tests/test_velocity_analysis.py` additionally requires generated
`build/velocity-evaluation/synthetic_*.csv` files from `robot_analyze_velocity()`
and the native replay executable. It is not a self-contained clean-clone test.

Native tests use ASan/UBSan and mock hardware. PTY tests exercise host transport
without opening the board USB port. Builds/tests do not prove hardware jitter,
loaded tracking, stopping distance/current, thermal behavior or power-cut
persistence. `TODO.md` is a historical development record; current behavior is
specified here and by the source. Local captures, build outputs and IDE/cache
files are excluded from the application commit.
