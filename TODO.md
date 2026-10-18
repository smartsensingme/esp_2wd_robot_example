# TODO.md — esp_2wd_robot implementation plan

> Agent execution checklist. Read `AGENTS.md` and `AGENTS-APP.md` before any work. Implement in order, run checks at each gate, and record evidence below tasks. **Never commit or push unless the user explicitly requests it.** `[ ]` means not yet completed, even when a design decision is documented.

## Phase 0 — Inventory and reproducible setup

Current bench step: both LUTs accepted; Kalman .05-deg/5-rad/s² and the
five-second +3-rad/s PID trial completed on hardware. User accepts the current
response and defers gain/feedforward refinement. Next functional validation is
bounded stopping/current, then motion/odometry examples. Sensor communication
still has isolated errors (19 reported in the last PID run).
Calibration capture and its Octave-compatible protocol now belong to the robot
component; the board owns GPIO/USB adapters and chassis configuration. The old
sensor-identification boot mode is a standalone MT6701 driver hardware test.

- [x] Inspect supplied SuperMini documents, `./docs/pins.txt`, and the actual pinout at `../esp_drv8833/docs/drv8833/drv8833-pinout.png`; map wiring and document C3 restrictions. Physical polarity, jumper and side tests remain pending.
- [x] Verify which sensor address (`0x06` or `0x46`) belongs to LEFT and RIGHT on hardware; keep both address mappings configurable even after discovery.
- [ ] Inspect latest source/public headers, README, examples, manifests and licenses for `esp-mt6701`, `esp_drv8833`, `esp_pid`, `esp_angle_lut`, `kalman`; document exact supported calls, bridge brake/coast logic and identified incompatibilities. Pin tested revisions rather than silently following moving branches.
- [ ] Confirm ESP-IDF toolchain/version, C3 build environment, power supply/USB logistics, physical chassis details, encoder shaft location, gear ratios and motor RPM limits. Add placeholders to configuration for geometry values not yet measured.
- [x] Create/verify CMake component structure, manifest and a minimal ESP32-C3 example; document ownership of handles and partial-failure cleanup.

**Gate 0:** Hardware assumptions are explicit; component dependencies and build target resolve. Do not energize motors before verifying signs, bridge channels and power wiring.

## Phase 1 — Independent angle LUT support and measurement integration

- [x] Design **multi-instance handle API** for `esp_angle_lut`, with independent left/right tables, enable flags, NVS identity, validation, table revisions and integrity. Preserve legacy signatures/namespace; require quiescent readers during maintenance and document that contract.
- [x] Implement/test independent LUT lifecycle and NVS persistence for two handles; include empty/corrupt/mismatched table and interrupted-update tests.
- [x] Choose and implement an optional correction integration path around `esp-mt6701` such that corrected cyclic angle is used **before** unwrap and velocity estimation. Preserve raw angle for calibration. Check software zero and direction coordinate consistency.
- [ ] Verify two MT6701 devices on one application-owned I2C bus with their distinct configured addresses; use bounded transfer timeouts, skip invalid samples, avoid duplicate reads.
- [ ] Test wraparound, both directions, angle sign/zero and LUT quality with deterministic synthetic vectors.

**Gate 1:** Two sensor instances yield independent valid corrected angles and velocities; LUT failures don't spuriously drive the motors. Record actual upstream changes.

## Phase 2 — Math and library-level API

- [x] Define public config structs: left/right radii, track width, gear ratios and encoder locations, address/device-handle mapping, signs, max wheel speed, acceleration, loop/PWM configuration, per-wheel PID tuning, brake/coast policy.
- [x] Specify ownership (borrowed/owned) and lifecycle (`create/init`, `start`, `stop`, `destroy`), physical units, concurrency, error codes, and a coherent `state` snapshot.
- [x] Implement pure differential inverse/forward kinematics. Include per-wheel gear transforms, sign normalization, common wheel target scaling on saturation, finite/invalid input validation.
- [x] Implement midpoint/exact incremental wheel odometry with real timestamps, reset/set pose and validity/staleness indicators.
- [x] Add host unit tests for forward/backward, counter-rotation, straight line, arcs, asymmetrical geometry, very small turns, saturation and odometry integration.

**Gate 2:** Kinematics and odometry pass native tests without connected hardware or RTOS.

## Phase 3 — Two wheel-speed loops

- [x] Confirm `esp_pid` exact API; instantiate independent PIDs with bounded [-100, +100] outputs, anti-windup, appropriate per-wheel signs and effective sample `dt`.
- [x] Implement one owner task on single-core ESP32-C3: sequential sensor reads, corrected angular rates, kinematics, acceleration limiting, two PID updates, bridge output, odometry and state snapshot. Keep peripheral waits bounded and hot path free of allocations, logging and flash writes.
- [ ] Begin with configurable example values (not guarantees): 200 Hz sampling+PID, 20 kHz PWM (hardware-dependent), 20–50 Hz higher-level commands. Confirm sensor half-turn/sample margin at actual motor shaft speed.
- [x] Make `set_velocity(v_m_s, yaw_rad_s)` and `set_wheel_speeds(...)` update references only; do not read I2C or run PID in caller context.
- [x] Add safe start/stop and PID reset, ramp limits, stale sensor handling and fault/disabled transitions; preserve legitimate reverse motor commands.
- [x] Test with fake encoders and fake DRV8833 adapter; confirm bridge arguments match actual driver channel order and signed duty policy.

**Gate 3:** Closed-loop speed following works at a conservative test rate; no unexpected activation on errors.

## Phase 4 — Battery-conscious stopping (V1 required)

- [x] Inspect actual `esp_drv8833` brake/coast functions and DRV8833 truth table. Implement explicit `DRIVE → BRAKE → COAST` transitions with configurable finite brake duration and optional early-stop threshold.
- [x] Freeze/reset PI accumulation when stopped or transitioning, prevent unwanted restart, and guarantee brake timeout even when encoder reads fail.
- [x] Distinguish COAST from hardware SLEEP in API/docs. If `nSLEEP` is wired and software-controlled, implement **optional** low-power SLEEP with safe wake timing; otherwise leave unavailable, not simulated.
- [ ] Bench-test stop modes, energy/current and actual deceleration at representative speeds; document effects on overshoot, battery, safety and motor heating.

**Gate 4:** No indefinite active-brake hold; a completed stop leaves outputs coasting, optionally sleeping if supported and selected.

## Phase 5 — Per-wheel calibration orchestration (V1 required)

- [x] Implement firmware `CAL` protocol handlers in `main.c` (`CAL STATUS`, `CAL START`, `CAL WRITE`, `CAL READ`, `CAL ENABLE`, `CAL DISABLE`, `CAL CLEAR`) parameterized by `<left|right>`, interfacing directly with modern `esp_angle_lut` instance handles.
- [x] Implement open-loop multi-plateau acquisition runner in firmware (70% -> 90% -> 80% duty) with RAM buffering during movement and structured stream output (`ANGLE_CAPTURE/1`).
- [x] Implement binary NVS stream handlers in firmware: `CAL WRITE` (buffered serial binary reception, CRC32 check, `esp_angle_lut_instance_install_detailed`) and `CAL READ` (header + 512 binary bytes).
- [x] Implement Octave transport & protocol drivers in `components/esp_2wd_robot/tools/octave/`:
  - `robot_serial_cmd.m` (command dispatch, response parsing, timeout handling)
  - `robot_cal_status.m` (status query and decode)
  - `robot_cal_read.m` (stream download, header parsing, binary unpacking, CRC32 verification)
  - `robot_cal_write.m` (preflight validation, binary packing, stream upload, readback check, enablement)
- [x] Implement master Octave calibration orchestrator `components/esp_2wd_robot/tools/octave/robot_calibrate_wheel.m`:
  - Sequence: acquisition trigger (`CAL START`) -> receive stream -> estimate non-linearities (`esp_angle_lut_calculate.m`) -> plot before/after harmonics -> upload LUT via `robot_cal_write.m` -> verify readback -> enable -> display status.
- [x] Implement interactive Octave console menu `components/esp_2wd_robot/tools/octave/robot_2wd_calibrate_menu.m` (`calibrate_menu.m`):
  - Serial port auto-detection, boot handshake capture (2 s), pre-calibration LUT disabling, .mat data persistence, user approval before NVS flash, manual upload, NVS readback plot, enable/disable toggle, clear, ping, and offline inspection.
- [x] Calibrate physical LEFT on bench: capture analyzed offline; user reports successful LUT installation (CRC 5430E186).
- [x] Calibrate physical RIGHT on bench: capture analyzed offline; user reports successful LUT installation (CRC 316A315A), limited speed-ripple improvement.
- [ ] Verify both LUT CRCs/enabled flags after reboot and acquire an independent verification run; saved calibration MAT files do not establish post-reboot status.
- [ ] Re-test closed-loop PID following (Phase 3/4) with active LUTs: compare speed tracking, adjust gains, and introduce breakaway feedforward.

**Gate 5:** Each physical sensor has an individually stored, independently verified and enabled LUT correction in NVS managed directly via host-firmware protocol. Normal PID following benefits from linearized feedback.

## Phase 6 — Integration example, tests, docs

- [x] Produce a minimal application for ESP32-C3 SuperMini using actual checked `docs/pins.txt` without embedding those GPIO defaults in the reusable component.
- [ ] Add example for straight travel, pivot, arc, stop→brake→coast, state/odometry readout, calibration mode and restart with NVS-stored LUTs.
- [ ] Build for ESP32-C3 in selected ESP-IDF version. Run host tests, target checks and `git diff --check`; record exact results. Do not label hardware tests passed without a hardware run.
- [ ] Measure and record jitter, worst observed execution time, I2C latency, stale-data response, motor tracking, bridge polarity, stop current and LUT improvement.
- [ ] Review README and public API for units, concurrency, geometry, motor/encoder shaft distinction, NVS ownership, braking versus coast versus sleep, pin assumptions and error paths.
- [ ] Update checkboxes and provide a summary of changed files, tests, known limitations and pending hardware checks. **Do not commit without approval.**

**Gate 6:** V1 integrated example compiles, unit tests pass, and hardware-dependent acceptance is explicitly evidenced or clearly pending.

## Future backlog — not V1 requirements

- [x] Optional `smartsensingme/kalman` wheel angular estimator integrated, configurable per wheel; native/synthetic replay and both-wheel hardware comparisons completed. Physical latency characterization remains a refinement.
- [ ] Optional 6-DOF IMU for relative-yaw fusion; don't claim absolute compass heading without appropriate external reference. Barometer/temperature are not heading references.
- [ ] Application command watchdog with explicit timeout/fail-safe policy.
- [ ] Wheel radius/track calibration against measured trajectories or external ground truth.
- [ ] Position/waypoint controller, path following, jerk-limited motion, autonomous pose targeting.
- [ ] Advanced simultaneous two-wheel LUT calibration once independent captures are validated.

## Decision log and open measurements

| Item | Status / rule |
| --- | --- |
| Public component name | `esp_2wd_robot` |
| MCU | ESP32-C3 SuperMini (single core) |
| Chassis | Two middle driven wheels, two passive casters |
| Bridge | DRV8833 via `esp_drv8833`; signed PWM/duty [-100, 100] |
| Encoders | Two MT6701 I2C, addresses 0x06 and 0x46; LEFT=0x46 / RIGHT=0x06, configurable |
| Motion command | Forward/yaw percentages; optional SI interface, internal wheel rad/s |
| Geometry | Left radius, right radius, wheel separation configurable; diameter 0.0661 m / radius 0.03305 m, track 0.1186 m (user measured 2026-10-17) |
| Gear ratios / encoder shaft | Wheel-shaft MT6701, user-confirmed 1:1; ratio remains configurable |
| Speed control | Two `esp_pid` loops, Kalman disabled in V1 |
| LUT | Two per-instance handles and separate persistent tables; sequenced tests first |
| Stop policy | BRAKE then COAST; optional true SLEEP only with supported/wired nSLEEP |
| IMU | Future relative-heading assistance; no magnetometer currently |
| Watchdog of commands | Deferred; immediate I2C/sensor fault behavior still required |
| GPIO map | Implemented in `main/board_config.h` from docs; physical polarity/jumper checks pending |
| Initial rate targets | Tentative: 200 Hz sensors/PID, ~20 kHz PWM, 20–50 Hz commands; measure |

## Execution notes

### 2026-10-17 — First board run and classified ULT policy

- User's first board run failed MT6701 initialization with INVALID_RESPONSE.
  The prior code attempted provisional left/0x06 first and returned immediately;
  no evidence about right/0x46 was obtained. Startup now probes/reports both
  configured addresses with side, GPIOs, bus speed and exact per-device errors,
  then aborts without creating the bridge if either sensor failed.
- User requests ignoring transition-associated ULT for motor-stop decisions.
  Application now uses the driver's captured context rather than the raw latch:
  known initialization/wake/start/coast/reversal/brake captures are counted and
  acknowledged; NONE/unrecognized captures request fault/coast. Known LOW
  assertions keep their association after ACK. API errors still propagate.
- Documented this optional policy, polling example and ACK semantics in the
  local `components/esp_drv8833/README.md`; no driver behavior/API change.
- Added native application policy tests, including unknown transients and
  continuously LOW acknowledged known captures. Added per-wheel encoder error
  fields to coherent state and reporter output; no periodic logging added.
- Native suites pass; updated C3 application build passes. Hardware retest of
  both addresses and this policy remains pending. No flash/commit/push performed.

### 2026-10-17 — Initial Phase 0 work

- User confirms wheel-shaft encoders, 1:1; ratio remains per-wheel configurable.
  Provisional sides/geometry/limits are authorized; see `AGENTS-APP.md` and
  example `board_config.h`. Supply currently 6 V.
- Inspected all supplied board/wiring documents and bridge pinout in the local
  upstream repository. Recorded boot-pin/LED interaction and sleep-jumper checks
  in `docs/dependency-audit.md`; physical verification remains pending.
- Added root ESP-IDF component, manifest, configuration validator, native tests,
  and a non-actuating C3 example. Hardware handles, control, odometry and LUT
  integration are not implemented. No dependency is silently substituted.
- Compared exact remote revisions and public APIs: bridge lacks active brake;
  LUT is singleton. User authorizes local upstream edits, later publication.
- Native validation tests passed with Apple clang using Xcode's MacOSX26.4 SDK.
  The default command-line SDK/linker combination failed on `arm64e.x1`; using
  the installed matching Xcode SDK resolved this tooling mismatch.
- ESP-IDF 6.0.0 C3 build passed (985 steps) using the Component Manager:
  `idf.py -C examples/basic -B /tmp/esp-2wd-robot-basic-build -D IDF_TARGET=esp32c3 build`.
  Binary size 0x21800 bytes; effective tick rate 1000 Hz and USB Serial/JTAG
  console verified. This validates only the configuration scaffold, not drivers.
- Full dependency source/license audit, dependency resolution, hardware tests
  and Gate 0 remain pending. Checkboxes intentionally stay open for incomplete
  composite tasks. No Git repository exists here; no commits created.

### 2026-10-17 — Local component APIs and pure robot math

- Geometry updated from actual user measurements: diameter 66.1 mm, equal radii
  33.05 mm, driven-wheel track 118.6 mm. Supply 6 V remains bench metadata and
  is not an input required by the PID/driver API.
- Changed local `esp_angle_lut` checkout: opaque handles, independent namespaces,
  create/destroy and per-instance operations, internal RAM allocation, legacy
  wrappers. Installation durably disables correction before changing slots;
  detailed NVS failures and unexpected erase errors propagate. Maintenance
  requires sampling readers stopped. Native ASan/UBSan tests pass for two-table
  isolation, exhaustive count-domain wrap, reload, invalid tables, injected
  persistence failures, corrupt-slot fallback/bypass and legacy calls. Physical
  NVS power-cut qualification and acquisition integration remain pending.
- Changed local `esp_drv8833`: explicit static-high brake, awake coast, braking
  state and BRAKE fault context; existing coast/sleep behavior preserved.
  Added native tests for brake/drive/coast/sleep transitions and failures at each
  high-output write; existing ULT tests still pass with ASan/UBSan. These are
  low-level operations: robot-owned finite brake timing is not implemented yet.
- Added pure differential kinematics, common target scaling and exact arc
  odometry with timestamp ordering/finite-input checks. Robot native tests pass.
  Acquisition-side sign/ratio transforms, freshness and pose validity are pending.
- Added normalized forward/yaw reference conversion, [-100,+100] percent of
  configured limits, retaining internal wheel rad/s. Tests distinguish speed
  references from duty and cover reverse, both pivots and combined saturation.
- Added explicit local dependency smoke example, separate from basic robot
  example. No uncommitted dependency is represented by a fictitious remote pin.
  Neither example initializes motor outputs. See validation results below.
- Final validation: robot native tests, LUT native tests and driver native tests
  pass with ASan/UBSan. ESP-IDF 6.0 builds pass for the basic C3 robot example,
  dependency smoke on C3/LEDC, and dependency smoke on S3/MCPWM. Effective PWM
  backend selections checked in sdkconfig. Both edited upstream repositories
  pass `git diff --check`; root is still not a Git repository. No hardware run,
  motor activation, commit or push was performed. Robot periodic integration
  and the complete Phase 1/2 acceptance gates remain open.

Record work/results beneath the appropriate phase when it is actually done. Avoid marking tasks complete merely because code was generated; build, test, and inspect hardware-dependent assumptions. When missing hardware/documents block one step, complete safe independent software tests and document the blocker instead of guessing.

### 2026-10-17 — Application restructure and periodic integration

- User requested application-root layout. Moved the reusable component into
  `components/esp_2wd_robot`; all six dependencies are local Git checkouts in
  `components/` with HTTPS origins and preserved revisions/working changes.
  Original sibling checkouts were not reset or cleaned. No extraction/commit.
- `main/` owns board wiring, bus/device/driver/LUT handles and bounded adapters.
  Root `idf.py build` selects only local sources. Native test outputs are under
  `build/native/`; old scaffold examples are archived under ignored `build/`.
- Implemented the owner task/GPTimer, eight-entry bounded commands, independent
  PID instances, actual per-sensor dt, LUT-before-unwrap, signs/shaft ratios,
  ramps, coherent state, validity flags, fault latching and explicit re-arm.
- Stop latch bypasses FIFO; normal stop is finite brake then coast, emergency
  and sensor/output/timing faults coast directly. Destroy retains context on
  failed coast/timer cleanup until retry succeeds. Optional true sleep is
  exposed but disabled in the sample until the physical jumper is checked.
- Integrated pinned diagnostics/reporter: 200-Hz cycle, 5-s windows, full ISR
  notification counts, numeric stages/events/intervals, snapshots after cycle
  end and priority-1/Core-0 compact presentation. Mapping/budgets and limits are
  in `docs/control-timing.md`. No task-side formatted log/flash/allocation.
- Native ASan/UBSan tests pass with actual PID source and unequal synthetic
  motor plants, wrap/sign/ratio vectors, stale/failing reads, backlog, deadline,
  state transitions, brake timeout and output-failure/coast retries. Existing
  local LUT/bridge regression tests also pass. Root C3 application build passes.
- The application boots passive: no start/reference command. Checkboxes above
  describe verified software implementation only; physical acceptance gates,
  calibration workflow and production timing/current/speed tests stay open.

- Final profile validation: root C3 application builds pass with full diagnostics,
  reduced timing, and diagnostics disabled. Operational safety timestamp capture
  remains active in the disabled build. All selected application component paths
  were verified as this workspace's local `components/` directories. Effective
  1000-Hz tick, owner-only MT6701 access and watchdog enablement were checked.
  Native regression suites pass; no hardware run or motor activation performed.

### 2026-10-17 — Hardware PID bench trial and transition to Phase 5 LUT

- User conducted 5-second serial-start PID trial at +3 rad/s both wheels:
  50 coherent snapshots recorded (10 Hz), zero sensor errors across 2,000 I2C
  transactions, zero deadline misses, zero missed releases.
  Max cycle processing time: 1,157 us (period 5,000 us; CPU headroom > 76%).
  Max wake lateness: 51 us (~1% jitter).
  Transition ULT event context=1 recognized and accepted without false trip.
- Bench dynamics: breakaway occurred around 64-65% duty after 1.3 s of integral
  accumulation (Kp=3 Ki=20). Steady-state tracking reached ~2.98 rad/s left
  (mean duty 63.7%) and ~2.99 rad/s right (mean duty 62.6%).
  Residual speed ripple ~±0.15 rad/s observed, consistent with MT6701 cyclic
  magnetic non-linearity without LUT correction (loaded=0 enabled=0).
- User approved moving directly to Phase 5: Per-wheel LUT calibration before
  fine-tuning PID gains or adding breakaway feedforward.
- Design choices established for Phase 5:
  1. Multi-level duty collection (70% -> 90% -> 80%) with speed step-up and
     step-down to isolate cyclic angle error from dynamic speed changes.
  2. Sequential calibration (one motor at a time: LEFT alone, then RIGHT alone)
     as definitive target for maximum sample rate, zero cross-chassis vibration,
     and optimal accuracy.
  3. No Kconfig dependency: runtime command dispatcher in `main.c` supporting
     `cal left`, `cal right`, `pid`, `status` without recompilation.
  4. Zero logging/USB during movement: RAM sample buffering with post-stop
     structured stream output (`ANGLE_CAPTURE/1`).
  5. Dedicated Octave scripts placed in `components/esp_2wd_robot/tools/octave/`.

### 2026-10-17 — Phase 5 protocol design: Host <-> Firmware live calibration

- Approved live bidirectional calibration protocol modeled after
  `smartsensingme/esp-mt6701-example` adapted for dual wheels (`left` / `right`)
  and modern multi-instance `esp_angle_lut` APIs:
  1. Serial commands:
     - `CAL STATUS <left|right>`: reports loaded, enabled, generation, bins, counts, CRC32.
     - `CAL START <left|right>`: executes open-loop multi-level ramp (70% -> 90% -> 80%),
       buffers raw counts and timestamps in RAM, then streams structured text
       (`ANGLE_CAPTURE/1 wheel=<side> samples=<N> ... END-CAPTURE`).
     - `CAL WRITE <left|right> <bins> <full_scale> <crc32_hex>`: readies firmware for
       512 binary bytes (256 int16_t), computes CRC32, commits to NVS via
       `esp_angle_lut_instance_install_detailed`.
     - `CAL READ <left|right>`: emits `ANGLELUT/2` text header followed by 512 binary
       bytes read directly from NVS via `esp_angle_lut_instance_read`.
     - `CAL ENABLE <left|right>` / `CAL DISABLE <left|right>`: toggles correction state.
     - `CAL CLEAR <left|right>`: clears table from NVS.
  2. Octave host toolchain under `components/esp_2wd_robot/tools/octave/`:
     - Driver utilities for transport, serial binary streaming, and CRC32 verification
       (`robot_serial_cmd.m`, `robot_cal_status.m`, `robot_cal_read.m`, `robot_cal_write.m`).
     - Master `robot_calibrate_wheel.m(port, wheel_side)` running live acquisition,
       harmonic analysis, NVS upload, verification, and enablement in one step
       without manual file handling or recompilation.
- Implemented and verified complete toolchain:
  - Firmware compiled for ESP32-C3 SuperMini (`esp_2wd_robot_app.bin`, 0x3df00 bytes, 76% flash free).
  - Native tests pass (`sh tools/test.sh`, 8/8 suites passing).
  - All GNU Octave tools validated under Octave 11.3.0 and `instrument-control 0.10.1`.
  - Added 2-second boot handshake window (`SYNC READY timeout=2000`) allowing
    Octave to auto-synchronize on power-up/reset or fallback to interactive console.



### 2026-10-17 — LUT reference audit

- Compared against esp-mt6701-example and its pinned esp_angle_lut revision
  ff8f51fad4dd9be596905d0ddb19114f2caf74f3: calculator is byte-identical.
  Reference captures at 500 Hz; this C3 integration uses 200 Hz. Both buffer
  during movement and transfer after stopping; a second USB is not required.
- Corrected calibration timestamps to use I2C read midpoints. Invalid reads
  consume a release without storing a fabricated zero angle or changing duty.
  Failed acquisition/coast no longer exports a successful partial capture.
- Both live Octave entry points now require END-CAPTURE and the advertised
  sample count, and reject invalid native counts/nonchronological timestamps.
- Native suites (8), reusable Octave LUT tests and ESP-IDF 6.0/C3 build pass.
  Root has no Git metadata, so git diff --check is unavailable.
- Hardware verification and analysis of the user's saved MAT capture remain
  pending. Reported phase error is a constant-speed-model residual, not an
  absolute-angle accuracy measurement. Speed RMS currently uses all selected
  segments, including training turns; it is not a held-out speed test.

### 2026-10-17 — Saved LUT capture analysis and partial USB readback

- Analyzed tests/cal_left_20261017_182415.mat and
  tests/cal_right_20261017_183622.mat: 4,800 samples each, strictly increasing
  timestamps, no zero-count samples. Both precede the timestamp corrections.
- Applied each stored 256-bin table to its original settled segments (1.5 s
  settle, 0.5 s tail). Left speed RMS reduction by stage: 2.1/6.3/1.8%; right:
  14.6/41.4/16.9%. These are offline replay metrics, not new hardware runs.
- Separate 64-bin diagnostic fits (same eight harmonics, sufficient per-stage
  coverage) show first/last 70% profile correlation 0.8984 left / 0.9886 right;
  profile difference RMS 0.1668/0.1137 deg. This supports greater repeatability
  on the right; it does not prove the periodic pattern is exclusively sensor
  nonlinearity. First/last mean speed magnitude changes 49.70 -> 53.33 rpm
  left and 56.63 -> 59.49 rpm right despite the same PWM.
- Matched reference's accumulation of partial binary reads with a bounded
  host helper; added offline regression for 303+64+145 bytes, an empty read,
  and a permanently truncated 303-byte transfer. CRC remains mandatory.
- Readback now requires END-HEADER and valid wheel/resolution/dimensions;
  reconstructs little-endian int16 explicitly and provides the CRC alias used
  by the menu. Upload verifies before enabling and refreshes final status.
  A newly installed table remains disabled if readback fails.
- Clarified the menu warning: its 50% threshold concerns speed ripple, which
  explains the warning despite right phase reduction being 53.8%.
- Octave regression and reusable LUT tests pass. No firmware change in this
  follow-up, no flashing or motor activation. USB hardware validation pending;
  the reported 303 bytes alone cannot distinguish a partial host read from
  actual firmware/USB byte loss. Existing firmware VFS can drop TX bytes after
  timeout; CRC/exact-length checks continue to reject such a transfer.

### 2026-10-17 — Correct physical LEFT/RIGHT mapping

- User corrected all previous physical-side identifications: old left is
  physical RIGHT, old right is physical LEFT. New mapping is LEFT bridge B /
  motor 2 / encoder 0x06 / encoder sign +1; RIGHT bridge A / motor 1 / encoder
  0x46 / encoder sign -1. Motor signs remain +1; wiring is unchanged.
- Board driver bridge selection, sensor addresses and encoder signs updated
  together. Logical drive callback preserves the driver's right,left order.
- Historical NVS names remain bound to their original sensor: LEFT uses
  robot_right, RIGHT uses robot_left. No table rewrite or flash migration;
  installed LUT contents and enabled flags follow the original sensors.
- Old MAT filenames/logs retain historical labels. The old right calibration
  (CRC 5D87F4E7) belongs to physical LEFT; old left (9FCB1B8C) to physical RIGHT.
- Updated current application guidance/docs and marked historical validation
  labels. Added native mapping regression for bridge, addresses, forward
  encoder signs and legacy NVS identity. Hardware validation remains pending;
  no flashing, motor activation, commits or pushes performed.

### 2026-10-17 — Match reference native USB LUT transport

- Revisited reference transport revision ae0414651473b0bd7869c0749587686098a5a5aa
  and Octave tools revision 7269a41a41b0677a834c74ae6ebf4132d69ce87c.
  The reference installs the USB Serial/JTAG interrupt driver and sends exact
  bytes via usb_serial_jtag_write_bytes, checking partial writes. This app was
  still using fwrite through the basic polling console VFS. ESP-IDF 6.0's VFS
  can discard characters on its 50-ms TX timeout while reporting full size.
  This is a concrete transport defect; hardware causality remains unmeasured.
- Installed native driver with 4096-byte TX / 1024-byte RX rings, routed console
  VFS RX/TX through it, retained nonblocking getchar for bounded maintenance
  waits, and sent LUT header + payload through checked native-driver writes
  with 5-second waits and explicit TX completion. No line-ending conversion
  touches binary LUT bytes. Console and direct writes share the same driver.
- Reference macOS/Linux DUMP FRAMED versus Windows DUMP BLOCK HEX applies to
  recorder captures. CAL READ uses direct binary on both; no Windows block
  protocol is introduced here. Previous host-only fix was incomplete.
- Nine native suites, Octave fragmented/truncated read regression and IDF 6.0
  ESP32-C3 build pass. No flashing or motor operation. Recheck USB readback of
  existing NVS tables on macOS, then timing under enabled-driver interrupts;
  no need to repeat calibration motion to validate the readback transport.

### 2026-10-17 — Calibration session notification lifecycle

- User reported CAL START right timeout after successful left calibration.
  Found timer still active throughout the 100-ms braking delay after capture;
  ~20 pending releases could survive into the next session's notification 0.
  This explains a re-entry backlog, but the old generic error cannot establish
  which timeout condition occurred on the reported hardware run.
- Stop/disable/delete timer and drain pending notifications before braking;
  also drain before starting a new timer epoch. Restore the caller's original
  priority instead of assuming priority 1. Preserve 500-us wake/backlog limits.
- Failure response now includes reason, samples, releases, notification count
  and lateness, all formatted only after stopping. Brake errors also prevent
  a successful calibration response; coast is still attempted.
- Existing nine native suites pass; hardware LEFT -> RIGHT -> LEFT sequence
  remains pending. No motor operation, flashing or NVS changes performed.

### 2026-10-17 — Both physical-wheel LUT captures installed (user report)

- Latest MAT files: cal_left_20261017_184954.mat and
  cal_right_20261017_185443.mat; corrected physical side names. Both have 4800
  samples and strictly increasing timestamps. Fixed the saved-capture analysis
  script to select the newest file per side rather than the first directory entry.
- LEFT: 25 turns, max correction 64 counts; phase RMS 0.8229 -> 0.4282 deg
  (48.0%); speed RMS 3.5599 -> 2.4353 rpm (31.6%); CRC 5430E186.
- RIGHT: 22 turns, max correction 26 counts; phase RMS 0.4548 -> 0.3329 deg
  (26.8%); speed RMS 2.3435 -> 2.2919 rpm (2.2%); CRC 316A315A.
- Right residual speed RMS is slightly lower than left after correction despite
  its small relative improvement. Its residual cannot be attributed specifically
  to mounting from these encoder-only recordings. Mechanics, drive dynamics,
  sensor noise and acquisition uncertainty are not independently identified.
- Capture/calculation/install workflow can be considered completed for this
  bench iteration. No further LUT fitting is required before a PID comparison.
  Full Gate 5 remains pending post-reboot persistence/status and independent
  corrected-feedback/PID verification. No new motor run or flash operation by
  the agent; installation success is user-reported, not encoded in the MATs.

### 2026-10-17 — Explicit reset-time calibration mode selection

- Replaced the old 2-second "any command" boot handshake with a 5-second
  post-initialization window accepting only CAL HELLO. Repeat SYNC READY every
  250 ms to tolerate host opening/initial flushing; acknowledge mode explicitly.
- Octave menu and standalone live calibrator wait up to 60 seconds for a reset
  beacon after opening the selected port. No PING probing or repeated flushing
  during synchronization. Restore the serial timeout after synchronization.
- No handshake selects the current normal console without automatic motion.
  CAL commands require maintenance mode; PID is blocked while in that mode.
  Reset exits; neither session selection nor menu exit changes saved LUTs.
- Added native handshake acceptance/rejection coverage; ten native suites pass.
  Octave parses the modified entry points. Hardware reset/timeout handshake and
  serial-port behavior across reset remain pending; no flash/motion performed.

### 2026-10-17 — Physical calibration input replaces reset handshake

- User replaced reset-based entry with configurable GPIO5 active LOW, internal
  pull-up, 50-ms debounce. One request per stable press; 10-ms priority-1 polling
  task sets atomic state only. Main task stops/joins the PID owner before mode
  confirmation and confirms awake coast. Mode stays latched until reset; a held
  LOW at startup re-enters after debounce. No serial command enters the mode.
- Octave now opens the chosen port and queries MODE, requiring calibration
  already active. No reset waits/beacons/reconnect logic. Firmware's five-second
  boot window and its obsolete acceptance helper/test were removed.
- Optional configurable external LED toggles every 250 ms after entry. Onboard
  GPIO8 LED shares MT6701 SDA, so default LED is disabled; do not repurpose SDA.
  Pin conflict validation restricts selection to this board's unused pins.
- Added debounce bounce/hold/rearm/boot-low tests and actual Octave serial-client
  MODE acceptance/rejection tests using pseudo-terminals. Ten native suites,
  Octave serial integration and ESP-IDF 6.0/C3 build pass. Effective defaults
  GPIO5 / 50 ms / LED disabled verified in sdkconfig.
- Physical GPIO behavior, LED indication if enabled, stop-to-mode transition and
  timing impact of the input worker remain hardware validation tasks. The prior
  observed startup I2C fault is not attributed conclusively to wiring or fixed
  by this mode-selection change. No hardware access or flashing in this task.

### 2026-10-17 — Optional per-wheel Kalman CV velocity integration

- User accepts LUT workflow complete and requests Kalman before PID retuning.
  Added unmodified local smartsensingme/kalman revision
  5bbe0a624acbf06548bb963a8498dc7a19c92b3b; portable direct dependency pin in robot
  manifest and compile relationship in CMake. Root still uses local components.
- Optional CV state per wheel estimates velocity from corrected angular position
  with real per-wheel dt. Relative-coordinate rebasing avoids unbounded float
  position growth. No LPF cascade or odometry smoothing. Library default stays
  LPF; root PID bench opts into Kalman with initial sigma_position=0.05 deg and
  sigma_acceleration=20 rad/s^2, independently configurable per wheel. These are
  provisional values, not noise identification from the LUT phase residuals.
- Keeps differentiated velocities in snapshots and post-stop PID samples for
  comparison. Invalid readings skip estimator updates; bounded recovery uses
  actual gap; lost baseline reinitializes state/covariance. Numerical failure
  faults/coasts; no silent fallback. Existing PI gains/limits remain unchanged.
- Native integration covers 4000 signed/noisy variable-dt samples, positive and
  negative wraps, independent wheel speeds, bounded filter position/covariance,
  ripple reduction, invalid-read hold/recovery, reset after numerical failure,
  invalid/underflowing noise configuration. Existing native suites pass.
- Hardware execution timing, release latency/deadline counts, startup/transient
  lag, actual noise identification and corrected-feedback PID comparison remain
  pending. No flashing, motor operation, commits or pushes performed.

- Final Kalman integration validation: ten application native suites pass;
  upstream library's eight CTest cases (float/double) pass with sanitizers;
  ESP-IDF 6.0/C3 build passes. Upstream working tree remains clean and
  git diff --check passes there. Root integration has no Git metadata.

### 2026-10-17 — Migrate calibration ownership into esp_2wd_robot

- Added public calibration service/config/sample/result APIs. Each handle owns
  maintenance mode, copied stage plan, internal-RAM sample array, GPTimer/ISR
  context and notification lifecycle. Entry joins a supplied controller and
  retains its handle on failure; exit never restarts control. No global timer,
  sample buffer or maintenance-mode flag remains in main.
- Default behavior preserved: 70/90/70% duty, 1600 releases each, 200 Hz, native
  raw counts and I2C midpoint timestamps, 500-us lateness/backlog rejection,
  bounded isolated read omission, 100-ms brake then coast, stopped-only dump.
  Timer setup/teardown errors now propagate instead of ESP_ERROR_CHECK resets;
  failed timer/coast teardown retains resources and blocks maintenance until
  destroy. Fault callback API errors are rejected rather than ignored.
- Component now guards LEFT/RIGHT LUT status/read/install/enable/clear. Board
  still owns handles/namespaces; no flash data changes or remapping. main retains
  serial framing, command parsing, transfer buffer and USB driver. GPIO5 input,
  debounce and optional external LED remain application adapters. Octave intact.
- Added native tests of the real calibration source with deterministic RTOS/
  timer/I/O/LUT mocks: normal-mode gating, join failure/pointer retention, two
  complete consecutive wheel captures, stage boundaries, raw timestamps,
  omitted isolated errors, burst failures, backlog, lateness, fault API failure,
  setup error, retained timer/coast failure, retry destroy, namespace delegation,
  inactive destroy and config validation. Removed orphaned app-only sensor
  policy/header test; its cases are covered in the component service tests.
- The reusable manifest declares esp_angle_lut's base pin with an explicit local
  multi-instance-extension requirement. That remote base does not expose the
  instance API; publish extensions and update the pin before any remote-only
  consumer release. Current root disables Component Manager and uses local code.
- Native suites and Octave MODE/fragmented read regressions pass. Hardware
  capture/stop/jitter/notification-reentry validation of the migration remains
  pending. No hardware access, serial monitor, flash, commits or pushes here.

- Final migration check: ESP-IDF 6.0 ESP32-C3 build passes (app 0x41b60 bytes);
  ten native suites pass, plus Octave MODE and partial-payload regressions.
  Whitespace check against the saved application baseline is clean.

### 2026-10-17 — Evaluate smoothing/lag before PID retuning

- Extracted the unchanged CV equations into a shared float helper used by the
  controller, maintenance evaluation and native replay executable. Reusable
  estimator choice/defaults and root PI gains remain unchanged.
- Built exact-source native replay; latest source captures are physical LEFT
  cal_left_20261017_192809.mat (CRC 6117B7E2) and physical RIGHT
  cal_right_20261017_185443.mat (CRC 316A315A). Applied their own fitted LUTs with
  firmware-compatible integer/truncate interpolation before signed unwrap.
- Swept 9 position/acceleration noise pairs. Fixed-seed synthetic .05-deg noise
  plus 14-bit quantization, 4-rad/s² ramps/start/stop and a two-read dropout:
  current .05/20 gives 7.18-ms clean 1-Hz phase delay, ramp RMSE .0670 rad/s;
  .05/5 gives 16.39 ms and .0372 rad/s; former LPF gives 27.23 ms and .0569 rad/s.
  These are conditional simulations, not identified hardware noise/true accuracy.
  Gain/delay measured across .5/1/2/5/10 Hz; direct/LPF results verified against
  their analytic discrete responses. Reports/CSVs/plots in build/velocity-evaluation.
- Added component-owned optional open-loop evaluation with the active target LUT,
  native sign/ratio and simultaneous direct/LPF/Kalman outputs. Uses default plan
  duties for 800 releases/stage (2400/12 s), 76,800-byte internal-RAM trace allocated
  before movement, first velocity invalid, bounded gaps and explicit cycle deadline.
  Frees trace after stopped-only transfer; never computes/installs or disables LUTs.
- Added ESTIMATE START adapter and Octave menu options 10/11 while preserving
  options 1-9. Requires maintenance mode, enabled matching LUT and typed TESTAR
  LEFT/RIGHT confirmation. Saves tests/eval_<side>_<timestamp>.mat before analysis.
  Checks live/replay differences and compares candidates on the same dataset.
- Native tests cover both signs, valid-LUT gating, omitted reads, restored regular
  4800-sample calibration plan, trace lifetime and induced evaluation deadline.
  Full 2400-sample fragmented PTY/Octave roundtrip/replay and rejection of truncated
  transfer pass. Existing native controller/calibration suites pass.
- Physical evaluation, memory availability, production execution margin, GPIO/USB
  competing load and real transient behavior remain pending. No hardware access,
  motor activation, flashing, NVS writes, commits or pushes in this task.

- Final evaluation preparation: ESP-IDF 6.0/C3 build passes (0x42990-byte app),
  ten native suites pass; analytic synthetic gain/delay, full 2400-row Octave
  roundtrip/replay, truncated evaluation rejection, MODE and partial-LUT-read
  regressions pass. Source/plots/report refreshed under build/velocity-evaluation.
  Further motor evaluations are optional and deferred: existing calibration MATs
  and synthetic traces are sufficient for the current offline comparison. Do not
  repeat LUT calibration to evaluate the filter.

### 2026-10-17 — Diagnose silent USB MODE timeout

- A passive USB capture during user RESET and a fresh connection with MODE
  produced no response. Both connections were closed; no motor/NVS commands.
  This does not yet identify the cause of the board's silence.
- USB driver/console now starts before hardware initialization. A hardware
  initialization failure retains its code/resource and runs a command-rejecting
  diagnostic console after cleanup instead of returning silently from app_main.
  MODE then receives ERR command=STARTUP; GPIO mode changes and all hardware
  commands remain blocked. Initial sensor errors retain the first failing side.
- Octave distinguishes an opened-but-silent port, explicit startup error and
  ordinary port-open failures. ESP-IDF 6.0/C3 build and PTY MODE tests (including
  startup-error propagation) pass. New firmware has not been flashed here;
  hardware confirmation is pending.

### 2026-10-17 — ULT/nFAULT informational-only policy

- User explicitly supersedes the prior unknown-context ULT stop policy: all
  shared GPIO7 ULT levels/events are telemetry only in PID, raw calibration and
  velocity evaluation. Classification/counters/acknowledgement remain active.
- Application fault callback never requests a stop from ULT. State-query and
  acknowledgement errors are counted as telemetry errors, not propagated into
  control. Reusable component fault callbacks retain their existing contract;
  sensor/timing/drive error policies are unchanged.
- Updated application policy regressions for unknown/transitional contexts and
  all LOW/latch/pending combinations. Hardware confirmation after flash pending.
- Verification: all ten native suites and ESP-IDF 6.0 ESP32-C3 build pass
  (app 0x42b20 bytes). No USB session opened or firmware flashed for this change.

### 2026-10-17 — User completed both filter evaluations

- LEFT tests/eval_left_20261017_210409.mat: 2399/2400 samples, maximum
  processing 755 us, wake lateness 25 us, replay discrepancy <=3.9e-6 rad/s.
- RIGHT tests/eval_right_20261017_210850.mat: 2396/2400 samples, maximum
  processing 758 us, wake lateness 24 us, replay discrepancy <=3.0e-5 rad/s.
  Largest recorded right sample gap 10013 us; isolated omissions remain present.
- Weighted settled-plateau RMS variation about fitted linear trends [rpm]:
  LEFT direct 3.217, LPF25ms 1.730, current KF .05/20 2.858,
  replay candidate .05/5 2.284; RIGHT 2.931, 1.579, 2.662, 2.107.
  Candidate improves variation reduction from ~9-11% to ~28-29%; LPF smooths
  more. These are variations, not errors against independent true velocity.
- Actual float firmware/replay agreement and observed execution timing confirmed
  for these two open-loop runs. Synthetic 1-Hz phase delay .05/5=16.39ms,
  current .05/20=7.18ms, LPF=27.23ms; physical sensor/control latency not measured.
  Recommend considering .05/5 for the next closed-loop trial. No parameters,
  gains, LUTs or firmware changed during this analysis; no extra capture needed.

### 2026-10-17 — Selected Kalman for next PID trial

- User approved 0.05-deg position sigma / 5-rad/s² acceleration sigma on both
  wheels. Set explicitly in application run_pid_test; reusable defaults and
  evaluation command defaults remain unchanged. Startup log names the setting.
- PI gains remain Kp=3 Ki=20 Kd=0; +3 rad/s both wheels, 4-rad/s² reference ramp,
  75% duty cap and five-second bounded run. ULT remains informational. LUTs and
  sensor/timing/output error policies are preserved.
- Next hardware step: close Octave, flash, remove GPIO5 from GND and reset to
  normal mode, open one monitor, send pid. Capture full post-stop sample/summary
  output before considering gain changes. This is not another LUT calibration.

### 2026-10-17 — Accept serial-monitor Enter in the command console

- User reports pid+Enter produces no response. Console previously discarded CR
  and dispatched only LF, while IDF Monitor can send CR for Enter. Accept CR,
  LF and CRLF (one dispatch) in the text console. Binary payload readers are
  unchanged. Existing firmware can use Ctrl+J to send LF as a workaround.

### 2026-10-17 — Wait for feedback before arming the PID bench

- User's first run ended ESP_ERR_INVALID_STATE with zero stored samples, no
  missed releases/deadlines and final STOPPED/healthy feedback. Application
  checked sample_valid/velocity_valid immediately after create(), whose ready
  semaphore only confirms task/timer startup, not acquisition/baselining.
- Wait at most one second in STOPPED/coast for both validity flags before START.
  Preserve owner sensor/fault limits, stop errors and GPIO maintenance requests;
  no fault clearing/restart. Wait sleeps between checks. Five-second movement
  duration starts after reference submission, not during feedback preparation.
- Final summary now names wait_feedback/start/reference/run to identify the
  failed phase before stop cleanup overwrites the final healthy snapshot.
  Duty cap remains 75%; no unsolicited 70% kick/feedforward or gain changes.

### 2026-10-17 — Finish calibration protocol migration and remove sensor test

- User requests component-owned calibration and a smaller main. Moved CAL,
  ESTIMATE and MODE parsing, maintenance gating, capture/evaluation CSV output,
  LUT headers/payload/CRC and transfer state into optional
  esp_2wd_robot_calibration_protocol.[ch]. Copies chassis geometry, borrows the
  service, uses application read/write/drain callbacks; no USB/GPIO dependency.
- All text/binary goes through checked writes. Failure before capture/READY
  prevents movement/upload; output failures suppress remaining response/prompt.
  Legacy cal left/right now obey the same maintenance gate as host commands.
  New native protocol tests cover framing, CRC, short input, failed output/drain,
  both physical sides, normal-mode gating, captures and geometry propagation.
- Board GPIO/debounce helpers organized in main/board. On explicit user steering,
  moved sensor identification/recovery entirely out of main into the MT6701
  driver's standalone test_apps/dual_i2c project. Independent Kconfig/build and
  native recovery runner; no robot/bridge/LUT/PID dependency. Removed its robot
  boot-mode choice and direct main GPTimer dependency. Diagnostic test has its
  own application timing guidance; no files discarded or driver behavior changed.
- Current PID/Kalman settings, ULT telemetry-only policy, acquisition loop,
  NVS tables and Octave wire formats preserved. No flash or motor trial needed
  to repeat LUT fitting; protocol hardware smoke check remains pending.

- Final refactor verification: eleven reported native test groups pass (including
  the added maintenance protocol group), Octave MODE and 2400-row/truncated
  evaluation PTY regressions pass. ESP-IDF 6.0/C3 robot build passes (0x43020
  bytes) and independent dual-I2C driver test build passes (0x26630 bytes).
  Whitespace checks pass. main.c reduced from 1045 to 700 lines (~33%). No real
  serial session, flash, NVS write, motor run, commit or push performed here.

### 2026-10-18 — Comprehensive esp_2wd_robot documentation and spacing

- Applied the explicit comprehensive-documentation profile: all public types,
  fields, callbacks and functions now state units, ownership, mode/lifetime,
  concurrency, error and return contracts. Documented 81 production/native
  function definitions with purpose and internal/external callers, plus native
  fixture helpers. Added functional-block comments and blank-line separation.
- Rewrote component README against current implementation: dependency caveat,
  full non-actuating lifecycle example, async queue semantics, retained shutdown,
  validity/recovery, timing boundaries, raw calibration vs filter evaluation,
  transport framing/byte order/CRC/reconciliation and troubleshooting. Corrected
  obsolete Octave menu/reset-handshake documentation and documented host helpers.
  No component Kconfig exists; per-instance C configuration is documented instead.
- Behavior preservation checked: executable tokens unchanged across 13 C/native/
  test files; executable lines unchanged across 14 Octave files. Host ABI probe
  confirms size/alignment/field offsets and mode values for 15 public value types.
  README control example passes C11 syntax/warning checks. No API, numeric
  constant, gain, LUT format, peripheral policy or application main changed.
- Validation: native suites/protocol group, MODE and full/truncated evaluation
  PTY tests pass; final ESP-IDF 6.0 ESP32-C3 build passes (0x43020 bytes).
  git diff --no-index --check against pre-documentation copy reports no whitespace
  errors (root/component have no Git repository metadata). Other component Git
  working trees remain clean. No hardware session/flash, commit or push.


### Application documentation and initial Git snapshot — 2026-10-18

- Documented current board lifecycle/adapters, per-sensor LUT namespaces,
  informational ULT policy, GPIO maintenance, USB framing boundary, Kalman/PID
  settings, timing ownership, configuration, verification and pending hardware
  evidence. Replaced obsolete root usage instructions; retained old validation
  observations under an explicit historical heading.
- Documented every application C/header function and formatted functional blocks;
  lexical comparison preserved all executable C tokens/literals. No motor/flash
  action or behavioral change accompanied this documentation pass.
- Created the application Git repository and registered eight public HTTPS
  submodules at exact committed revisions. Public main heads match all gitlinks.
  No application remote is assigned; component remotes remain independent.
- Native regression groups, Octave mode/evaluation PTY tests and ESP-IDF 6.0/C3
  build passed (0x43020-byte image, 74% application partition free).
  Local MAT recordings, derived reports, build/cache/IDE outputs are excluded.
