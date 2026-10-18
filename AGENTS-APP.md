# AGENTS-APP.md — esp_2wd_robot

> Project-specific instructions for coding agents. Read `AGENTS.md` first; this file specializes it and does not override its safety, resource-ownership, verification, or no-autonomous-commit rules. Execute work using `TODO.md` in order. These files are a design specification, not proof that proposed APIs already exist.

## Current documented baseline (2026-10-18)

The root README and docs/control-timing.md describe the current implementation.
Older chronological clarifications below are historical where superseded:
physical LEFT=B/0x06/sign+1/namespace robot_right, RIGHT=A/0x46/sign-1/namespace
robot_left. ULT is informational only in all application flows. No encoder-only
boot mode or reset handshake remains in main; maintenance uses the debounced
GPIO request and the component-owned service/protocol. PID uses +3 rad/s for
five seconds, PI 3/20/0, 75% cap and CV 0.05-deg/5-rad/s² settings.

The reusable robot and all dependencies now have independent Git repositories.
This application tracks their public HTTPS submodules at exact revisions.
Root CMake intentionally keeps Component Manager disabled. Do not copy or edit
component source in the application commit; commit component work independently
and update its gitlink explicitly. User authorized this documentation/application
commit; that authorization does not imply a push or motor/flash operation.

## 1. Mission and project boundary

### User clarifications (2026-10-17)

- User requests a smaller main and component-owned calibration protocol. The
  optional esp_2wd_robot_calibration_protocol module owns CAL/ESTIMATE/MODE
  parsing, maintenance gating, CSV/binary framing, transfer CRC and LUT buffer.
  It borrows the calibration handle, copies chassis geometry and calls bounded
  binary-safe read/write/drain adapters; no USB dependency inside the component.
  main owns hardware, serial console, GPIO mode request and PID bench integration.
  Board button helpers live in main/board. Sensor identification and recovery
  accounting belong to the standalone driver test
  components/esp-mt6701/test_apps/dual_i2c, with independent Kconfig/build. Remove
  its boot mode and dependencies from main. Preserve host protocol/NVS identities
  and periodic behavior.

- User selected the next PID bench configuration after both filter evaluations:
  explicitly set both-wheel Kalman position sigma=0.05 deg and acceleration
  sigma=5 rad/s² in the board application. Keep reusable defaults at 0.05/20,
  gains Kp=3 Ki=20 Kd=0, +3-rad/s five-second trial, 4-rad/s² reference ramp
  and 75% duty cap. Use normal-mode command `pid`; no new LUT calibration.

- Latest policy: ULT/nFAULT (GPIO7, shared bridge signal) is informational only
  for PID, raw calibration and filter evaluation. No level, latched event or
  unknown context may stop/block control. Monitoring/acknowledgement errors are
  counted as telemetry errors and do not abort. Preserve sensor, timing and
  actuator API error policies. This supersedes older ULT stop rules below.

- Before PID retuning, user requests offline/synthetic comparison and hardware
  evaluation of velocity smoothing/lag. Shared float CV source must be used by
  control, replay and evaluation. Maintenance evaluation records one wheel at
  200 Hz, 70/90/70% for 4 s/stage, with active LUT, no PID and no NVS writes.
  Store direct/LPF/Kalman outputs and timestamps/timing in internal RAM, dump only
  after stop. Keep timing/fault policies and distinguish variation from true
  error. Synthetic phase delay excludes sensor/mechanical/scheduling delay.
  Do not automatically select noise parameters or retune PI from offline RMS.
  Menu options 10/11 are explicit LEFT/RIGHT trials with typed confirmation;
  keep existing options 1-9 unchanged. User performs physical bench steps.

- User requested migration of calibration into esp_2wd_robot. Its public
  calibration handle owns maintenance state, controller join, configurable
  capture plan, periodic timer, internal-RAM samples, stop/error policies and
  guarded LUT management. main remains the physical I/O/namespace initializer
  and USB protocol adapter; GPIO/debounce/LED stay application-specific. Preserve
  serial protocol, raw sample units, per-sensor NVS identity and Octave tools.
  Do not race maintenance with a shared controller; keep borrowed I/O/task alive
  on retained timer/join/coast errors. No new calibration is needed for migration.

- User accepts completion of both-wheel LUT calibration and requests Kalman
  before returning to PID. Use smartsensingme/kalman specialized CV model per
  wheel, actual per-wheel dt, after native LUT/unwrap/sign/shaft-ratio conversion.
  Kalman replaces the LPF when selected; no double smoothing. Keep reusable
  defaults opt-in and enable in the board PID bench. Preserve direct-difference
  velocity for post-stop comparisons, direct-increment odometry, and existing
  invalid-read/stale/stop/fault policies. Noise choices are provisional until
  hardware ripple/lag/timing tests; do not claim finished estimator tuning.

- Calibration mode is selected without reset by an active-LOW input, default
  GPIO5 with internal pull-up and 50-ms software debounce (10-ms polling).
  Input, debounce and optional external LED GPIO/polarity are Kconfig options.
  A low-priority task publishes only an atomic request. The main command task
  stops/joins any active PID owner and confirms coast before latching maintenance.
  Releasing the button does not exit; reset returns normal unless held LOW.
  Octave opens the port and queries MODE; it requires calibration already active.
  Remove the former reset/serial-handshake selection. The onboard LED shares
  GPIO8/SDA and must not blink. Optional external LED defaults disabled.

- Physical-side correction from user: all previous LEFT/RIGHT identifications
  were reversed. Physical LEFT = bridge B / motor 2, encoder 0x06, encoder sign
  +1; physical RIGHT = bridge A / motor 1, encoder 0x46, encoder sign -1.
  Both motor signs remain +1. This supersedes all older side mappings below.
  Preserve LUT sensor identity using historical NVS namespaces: physical left
  uses `robot_right` (0x06), physical right uses `robot_left` (0x46). Do not swap
  table contents or clear NVS. Old MAT filenames/log side labels are historical:
  old cal_left is physical right, old cal_right is physical left.

- Following the successful hardware PID trial (+3 rad/s both wheels, 5 s, zero
  sensor errors, max lateness 51 us, stiction breakaway at ~64% duty, residual
  ~±0.15 rad/s speed ripple from uncalibrated MT6701), user approved advancing
  directly to Phase 5: Per-wheel Angle LUT calibration before further PID gain
  tuning or feedforward.
- Calibration must not depend on Kconfig recompilation. Provide a unified
  runtime command dispatcher in `main.c` supporting interactive serial commands
  (`cal left`, `cal right`, `pid`, `status`).
- Calibration collects data across multiple speed levels (e.g. 70% -> 90% -> 80%
  duty) with one motor at a time (LEFT alone, then RIGHT alone) as the definitive
  target. Multi-level collection with step-up/step-down isolates cyclic angular
  error from speed-dependent dynamic variations.
- Zero serial/USB logging during motor movement: samples are buffered in internal
  RAM (`cal_sample_t`), transition to brake/coast after the final plateau, and
  dump structured CSV post-stop.
- Robot-specific Octave analysis scripts belong in
  `components/esp_2wd_robot/tools/octave/`, utilizing the underlying calculation
  routines from `components/esp_angle_lut/tools/octave/`.
- After LUT calibration and NVS activation, return to Phase 3/4 to re-tune PID
  gains against linearized velocity feedback and introduce breakaway feedforward.

- Current bench mode was PID, following the completed two-motor open-loop
  trials and user approval to skip another sign-only test. Serial `start` arms
  +3 rad/s per wheel for 5 s; board duty cap=75%, speed-reference limit=6 rad/s,
  acceleration=4 rad/s², PI Kp=3 Ki=20 Kd=0. No fixed kick/feedforward yet: the
  integral reaches the breakaway range. Encoder signs=-1/+1; motor signs=+1/+1.
  This trial opts into sensor_error_fault_count=3 and max_sample_gap_us=15000:
  hold last output on at most two invalid reads, without PI/velocity/odometry
  updates; fault on the third or stale feedback. Reusable defaults stay strict.
  Reports stay disabled during movement; print stored snapshots after stop/join.
  These settings supersede earlier default-mode, 25%-cap and 1-rad/s notes.


- User confirms positive PWM already drives BOTH wheels forward; preserve bridge
  inversion/motor_sign=+1 on both sides. Mirrored MT6701 mounting produces left
  raw-negative and right raw-positive forward rotation. Normalize encoder
  feedback with LEFT=-1 and RIGHT=+1, once after native LUT/unwrap in the robot.
  Do not invert the bridge to correct a sensor-coordinate sign. Open-loop logs
  expose both raw and normalized wheel mean speed for clarity.


- Never stop the system or latch a generic fault solely because an event has
  a known, expected cause with a defined continuation/recovery policy. Classify
  the cause first, record it with the affected side/resource, and apply that
  specific policy instead of treating every error return as a fatal fault.
  Examples: transition-associated ULT is acknowledged telemetry; isolated I2C
  read failures in the open-loop test are counted/skipped within its documented
  consecutive-error and freshness limits. Do not invent a known cause from an
  error code alone: ESP_ERR_INVALID_RESPONSE does not establish its cause.
  Unknown causes and violations of the policy's explicit operating/recovery
  limits remain separate conditions with their specified stop/coast response.
  When implementing a new mode, define these classifications before adding
  generic abort-on-error paths; do not silently carry a stricter policy from
  another mode into it.

- Open-loop bench acquisition tolerates isolated invalid reads: skip their
  velocity contribution/reset derivative history, hold existing duty for at most
  two failures, and require a valid pair to start/change PWM. Stop/coast on three
  consecutive failures per side or >15 ms without a valid reading. Preserve
  strict PID behavior until separately changed. This replaces the earlier
  single-read abort for this characterization test only.
- Latest bench decision: user observed motion only around 70% PWM. Before PID,
  run the serial-selected one-motor open-loop characterization. Default stages
  60/65/70/75/70/65/60%, 2 s each, then coast; no motion until `left` or `right`.
  Keep the other motor at zero and print only after stopping. This supersedes
  the earlier choice of PID as the current default boot mode. Do not increase
  PID limits or add feedforward until characterizing the actual response.


- User reports physical sides/default wiring and encoder directions were already
  checked and accepts the current mappings/signs for PID testing. Proceed with
  PID; do not repeat the side-identification gate. Any later reversal should be
  corrected in configuration, applying each sign exactly once. Current bench
  mode waits for serial `start`, then commands both wheels at 1 rad/s for at
  most 5 seconds, 25% duty limit, PI Kp=3 Ki=8 Kd=0, with no automatic restart.
  Disable periodic reports during motion and dump sampled snapshots after
  stop/coast and owner join. Sensor/ULT-unknown/output/timing fault policies
  remain active; accepting protoboard errors does not implicitly waive them.


- Every hardware-related application log must explicitly identify `left` or
  `right` whenever the information belongs to one side. Label each value in
  paired readings, speeds, duties and errors; do not rely on slash ordering.
  Include the sensor address or bridge channel where useful. Shared resources
  such as the I2C bus or ULT signal must be identified as shared rather than
  assigned an invented side. During physical address identification, log the
  address and `side=unconfirmed` until the user verifies the wheel association.
- User confirmed left encoder=0x46; right encoder=0x06. Keep mappings configurable.
- The current bench step reads both sensors independently at 200 Hz using
  GPTimer and finite 1-ms I2C waits. For the current logging comparison, collect
  60 seconds without periodic output/publication, stop the timer, then format
  the full-run diagnostic snapshot and per-side error/recovery totals.
  Do not initialize the bridge or start the robot owner in this maintenance mode.

- The user requests an ESP-IDF application at the repository root. Keep board
  integration in `main/`, the reusable library in `components/esp_2wd_robot/`,
  and all upstream checkouts in `components/`. Develop/test/build here. Root
  CMake uses explicit local components with Component Manager disabled; preserve
  portable manifests for future component consumers. Do not extract or publish
  the robot repository yet. The local `components/` copies are now authoritative
  for new edits; original sibling checkouts retain their earlier dirty changes.
- Include `esp_rt_diagnostics` plus its reporter in the periodic integration.
  One accumulator/owner, complete notification counts, closed cycles on every
  path, copied snapshots after cycle end, lower-priority presentation. Concrete
  rates/budgets/IDs live in `docs/control-timing.md`.

- Both MT6701 sensors are physically on the wheel shafts: sensed-shaft to wheel
  ratio is 1:1. Keep the ratio independently configurable per wheel.
- Encoder sides are confirmed: left address 0x46, right 0x06;
  motor 1/bridge A left and motor 2/bridge B right. Motor signs +1/+1; encoder
  signs -1/+1, as confirmed by the user and open-loop observations.
- User measured wheel diameter 0.0661 m (radius 0.03305 m) and driven-wheel
  separation 0.1186 m on 2026-10-17. These supersede the provisional geometry.
  Agent may choose initial configurable limits; actual limits remain unmeasured.
- Current bridge supply is 6 V. Motor nominal voltage/current remain unknown.
- User confirms ULT appears at motor start/coast/reversal. The application must
  classify its captured driver context: known transition contexts are telemetry
  and acknowledged without automatic stop; NONE/unrecognized contexts request
  fault/coast. A continuously LOW known capture keeps its association after ACK.
  API/output errors and sensor faults retain their separate error handling.
- User requests a normalized speed interface. Support forward/yaw references in
  [-100,+100] percent of configured speed limits; this is distinct from PWM duty.
  Retain wheel-side SI units internally for PID, geometry and odometry. Supply
  voltage is bench metadata, not a mandatory PID/driver input.
- `docs/pins.txt` is present and inspected. It specifies EEP GPIO10 and ULT
  GPIO7. The bridge image is in the sibling driver's repository at
  `components/esp_drv8833/docs/drv8833/drv8833-pinout.png`; inspect its sleep jumper
  against the actual module before relying on software sleep.
- User authorizes changes in owned local upstream component repositories,
  followed by later remote updates. This does not authorize commits or pushes.

These clarifications supersede older statements below about missing wiring
documents, unknown encoder mounting, and needing confirmed sides before software
work. Physical verification is still required before actuating hardware.

Implement an ESP-IDF application hosting **`components/esp_2wd_robot`**, a reusable ESP-IDF C component controlling a differential-drive four-wheel robotics chassis: two centrally located, independently driven left/right wheels and two passive caster wheels. Target hardware is **ESP32-C3 SuperMini** with **DRV8833** dual DC motor bridge and two **MT6701** I2C magnetic encoders. Use the existing drivers as dependencies rather than duplicating their functionality.

The user owns the upstream components and permits changes when warranted. Keep modifications minimal, backward-compatible where practical, and in the appropriate upstream repository; do not silently fork APIs or alter published dependencies. Do not commit or push without explicit user request.

The component is intended for reusable robotics experimentation, with explicit physical units, predictable task execution, closed-loop wheel-speed control, basic wheel odometry, configurable geometry, and a calibration mode. A future IMU and Kalman-based estimators must be possible without making them V1 requirements.

## 2. Authoritative inputs and priority

- Existing root `AGENTS.md`: repository-wide instructions, including bounded real-time paths, verification, and **never create commits without explicit approval**.
- `TODO.md`: execution sequence, acceptance gates, and task progress. Update checkboxes only for verified completion; document blockers/results.
- Actual source, public headers, examples and docs of dependencies: verify signatures, argument ranges, error handling, and lifetime; do not assume sample pseudocode is literal code.
- Actual application hardware documents, when available: `./docs/esp32-c3-supermini`, `./docs/pins.txt`, `./components/esp_drv8833/docs/drv8833/drv8833-pinout.png`. The wiring and board documents are supplied and inspected; motor sides/signs remain pending: do not invent GPIOs, nSLEEP wiring, motor-to-bridge mappings, pin polarities, or unverified sensor side mapping. Cross-check hardware before initializing actuators.
- Online upstream references (pin tested revisions in application manifests):
  - https://github.com/smartsensingme/esp-mt6701
  - https://github.com/smartsensingme/esp_drv8833
  - https://github.com/smartsensingme/esp_pid
  - https://github.com/smartsensingme/esp_angle_lut
  - https://github.com/smartsensingme/kalman (future, optional)

When ambiguity remains, adopt a safe configurable interface, note it in `TODO.md`, and never assume a wiring-specific default that could unexpectedly energize the motors.

## 3. Physical model and conventions

Use differential-drive / unicycle kinematics, **not holonomic XY vector control**. Robot body frame: +X forward, +Y left, +yaw counter-clockwise (CCW), with positive wheel angular velocity representing forward travel **after per-wheel electrical and encoder sign mapping**. Use SI units in API and state: meters, seconds, rad, rad/s, m/s.

Configurable physical parameters:
- `wheel_radius_left_m > 0`, `wheel_radius_right_m > 0`: *effective* radii, independently calibrated.
- `wheel_separation_m > 0`: effective left-to-right track width, measured/calibratable.
- For each wheel, `motor_revolutions_per_wheel_revolution > 0`; use `1.0` for a directly sensed wheel shaft. Critically determine whether each MT6701 reads the motor shaft, gearbox output, or wheel; never silently assume 1:1.
- Individually configurable signs for motor output and sensor angle, and individually configurable physical LEFT/RIGHT sensor-to-I2C-device association; user verified LEFT=0x46 and RIGHT=0x06.
- Maximum allowable wheel angular speed (motor-side or wheel-side explicitly labeled), acceleration/deceleration limits, angular acceleration if used, duty/output limits and control period.

For chassis command `(v [m/s], yaw_rate [rad/s])`:
```
v_left  = v - (wheel_separation_m / 2) * yaw_rate
v_right = v + (wheel_separation_m / 2) * yaw_rate
omega_left_wheel  = v_left  / wheel_radius_left_m
omega_right_wheel = v_right / wheel_radius_right_m
omega_motor_ref = omega_wheel_ref * motor_revolutions_per_wheel_revolution
```
If motor shaft and wheel shaft differ, convert feedback and commands consistently (prefer wheel-side units at chassis boundary). Do not confuse motor RPM with wheel RPM. Saturate commands by **common proportional scaling** of both wheel targets to retain commanded curvature where feasible. Support a documented behavior for extreme commands, zero-radius turning, and negative speeds. Optionally expose wheel-speed commands separately for diagnostics.

Forward kinematics from measured *wheel* speeds:
```
v = (r_R * omega_R + r_L * omega_L) / 2
w = (r_R * omega_R - r_L * omega_L) / wheel_separation_m
x_dot = v * cos(theta)
y_dot = v * sin(theta)
theta_dot = w
```
Integrate incremental wheel travel at actual sample timestamps, preferably midpoint/exact SE(2) integration and normalized heading; expose estimated pose `(x, y, theta)` with validity and timestamps. Wheel odometry drifts under slip, caster scrub and imperfect geometry. **A 6-DOF IMU does not provide drift-free absolute yaw**; gyro-aided relative yaw is useful, but accelerometers or barometers do not furnish a magnetic/global heading. An 8-DOF board without magnetometer also cannot promise absolute yaw.

## 4. Boundaries / architecture

- **esp_2wd_robot:** public configuration and handles, geometry, setpoints, command ramps, task/state machine, two speed-control loops, status, wheel odometry, calibration orchestration, brake-to-coast policy, resource lifecycle, and fail-safe transitions.
- **esp-mt6701:** one independent encoder instance per motor, I2C communication and raw/native 14-bit angle; coordinate zero/direction and velocity only when compatible with LUT placement. The application owns I2C bus and device handles; prefer finite I2C timeouts for bounded control cycles. Avoid a second unnecessary sensor read in one cycle.
- **esp_angle_lut:** two independent **handle-based** correction objects, persisted under distinct keys/identities with version and integrity checks, no global singleton. Apply each LUT to cyclic angle **before unwrap/differentiation/velocity**. Preserve access to uncorrected angle for calibration data and tests. If an integration hook in mt6701 is needed, keep it optional and generic, not a hard dependency on the LUT library. Avoid applying LUT twice or mixing software-zero conventions.
- **esp_pid:** one distinct PID state per wheel, saturation compatible with `esp_drv8833` signed duty scale [-100, +100], anti-windup, deterministic `dt`, bumpless start/stop and reset upon mode transitions. Initial PI tuning is acceptable; D may be set to zero. Check actual API before coding.
- **esp_drv8833:** sole owner of the bridge PWM/direction output; map LEFT/RIGHT to bridge channel correctly. Respect documented drive/coast/brake truth tables and physical module wiring.
- **kalman:** future optional estimator provider, initially disabled and not required at build/runtime. Do not add dead runtime complexity for V1.
- **Firmware/demo:** initialize/configure physical GPIOs, external bus and driver handles, exercise API and display diagnostics. Keep board-specific pin mapping outside the reusable component.

Use a single clear hardware/control owner task, which serializes the two I2C transactions on their shared bus and writes both motor outputs. Other tasks may update desired commands and read coherent snapshots through bounded synchronization. ESP32-C3 is single-core: do not copy dual-core/S3 task-affinity assumptions from the general `AGENTS.md`. No formatting logs, flash/NVS writes, allocations, indefinite device waits, or USB streaming in the critical control loop.

## 5. Control loop and errors

V1 starting point (verify experimentally, no hard guarantees): 200 Hz encoder sampling and wheel PI loops; PWM around 20 kHz if supported and electrically suitable; command-update rate 20–50 Hz; odometry 100–200 Hz. Configure timing instead of hardcoding unvalidated rates. Check MT6701 ambiguity bound: shaft rotation per sample must stay below half a turn (with margin for skipped samples), considering gearbox ratio and motor maximum RPM. Record effective sensor timestamps, not merely nominal task period.

Per iteration: bounded read of left and right encoders -> verify freshness -> apply LUTs -> unwrap/reconstruct wheel velocities -> estimate or filter velocities -> compute constrained references -> update individual PID states -> bound signed duties -> drive bridge -> integrate odometry and publish a coherent state snapshot. Avoid treating a stale sample as zero velocity. Specify skip/fault/coast behavior on read failure and accumulated missed periods. Never use stale angle differences across an unbounded interval without reset/resync.

Public API should clearly separate `init/create`, `start`, `set_velocity(v, w)`, `set_wheel_speeds`, `stop`, `get_state`, `set_pose`/`reset_odometry`, `calibration_start`, `calibration_stop`, and `destroy` (names are proposals and must be designed after inspecting existing headers). `set_velocity` updates only reference; it must not block on I2C or run the PI synchronously. Define whether components own or borrow dependency handles, who cleans up on partial initialization, and how API calls synchronize during calibration or faults.

## 6. Brake → coast and power policy (required in V1)

User prioritizes battery. Default stop behavior: initiate a **time-bounded active/electrical braking phase only when appropriate**, then explicitly transition to **COAST**, with drive PWM absent. Brake duration, optional low-speed exit threshold and maximum duration must be configurable; avoid a permanent braking state. Zero command or emergency/error stop transitions must be defined carefully; on faults default to a safe de-energized/coast behavior if uncertain. Distinguish active braking versus coast accurately using the actual driver bridge API/truth table. Do not confuse bridge coast with complete device sleep: **nSLEEP** (if physically wired and supported) may be an additional optional power-management step, requiring verified startup/re-enable timing; never claim that coast disables all DRV8833 quiescent consumption. Measure battery current for DRIVE, BRAKE, COAST, and SLEEP (only if available).

Make the braking-to-coast transition safe if sensors are faulty: use a finite max brake interval without requiring valid encoder data. Prevent integrator windup while stopped, and avoid accidental restart after a calibration or mode change. No command watchdog is required in V1; leave a documented extension point, but **do implement explicit sensor-error safety handling immediately**.

## 7. Angle calibration workflow (required, host computation acceptable)

This is a maintenance mode, not normal PID control. Calibrate each motor/sensor separately initially; plan simultaneous acquisition as a later validated extension. Physically support chassis so both driven wheels spin freely. Block normal drive commands and reset/freeze both PIDs during a calibration session.

Take raw native samples (consistent direction/zero convention), timestamp each reading, record duty and channel identity. Controlled open-loop PWM plateaus, discard acceleration transients, collect multiple full turns, and validate across more than one speed and direction. Keep each sensor's data separated even in concurrent runs; do not assume identical instantaneous timestamps. Support bounded ring buffer with defined overflow policy, or nonblocking transfer to host. No blocking serial I/O in control owner loop. GNU Octave workflow computes LUTs from captured data, installation stores left/right independently in NVS with validation and rollback, and an independent verification run compares pre/post correction. Never write NVS in the active periodic task; suspend it or route to maintenance worker. Verify `esp_angle_lut` format, native counts, resolution (typically 16384), wrap interpolation, per-instance persistence, and API signatures from actual code.

Calibration of cyclic sensor nonlinearity **does not** calibrate wheel radius, gearbox ratio, wheel track, or compensate wheel slip; treat these as separate optional geometric calibration functions.

## 8. Future extensions (not V1 acceptance gates)

- Integrate `https://github.com/smartsensingme/kalman` as selectable per-wheel estimate after inspecting its actual supported models; test model validity and numerical/timing cost. Do not treat LUT correction, low-pass filtering, and Kalman as interchangeable.
- Integrate optional 6-DOF IMU (gyro+accelerometer) for drift-limited relative heading, motion diagnostics, and later odometry fusion. An IMU without magnetometer cannot directly measure absolute compass bearing; barometric pressure/temperature do not change that.
- Add command watchdog, autonomous XY goal/path follower, trajectories with acceleration/jerk, sensor-fusion localization, remote transport. Don't conflate navigational `(x, y, heading)` goals with immediate lateral velocity commands.

## 9. Validation and deliverables

Make native host unit tests for pure kinematics, limits, odometry, signed motor/encoder mapping, PID lifecycle, brake-to-coast state timing, calibration gating, and simulated sensor I/O failures. Cover straight motion, reverse, pivot both ways, asymmetrical turn, saturated turn, gearbox scaling, positive/negative wrap, missed samples, start/stop and re-entry from calibration. Build against the actual supported ESP-IDF version and **ESP32-C3**, check `git diff --check`, and test on real hardware before marking integration complete. Measure loop duration/jitter, actual speed tracking, stop/brake current consumption, motor polarity, address-to-wheel mapping and LUT quality. Note hardware results still pending if hardware is unavailable. Do not fabricate measurements.

Provide a concise README with wiring prerequisites and example configuration, plus automated tests, plus progress updates in `TODO.md`. Leave `AGENTS.md` unchanged unless expressly requested. Source code, comments, and documentation language should follow the existing project conventions; this design specification is in English for unambiguous agent execution.
