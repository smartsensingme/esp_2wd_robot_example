> Historical initial dependency audit. Base revisions and missing APIs below
> describe the original integration stage. Current committed dependencies are in
> local-components.md and the application gitlinks; current APIs are documented
> in the component READMEs.

# Initial dependency and hardware audit

2026-10-17. This is a partial Phase 0 audit, not integration acceptance.

## Hardware documents

Read `pins.txt`; visually inspected the SuperMini pinout and rendered schematic.
Found and visually inspected the actual bridge module image at
`../esp_drv8833/docs/drv8833/drv8833-pinout.png` (the path is `docs`, not `ocs`).
The local image labels IN1..4, OUT1..4, VCC, GND, EEP and ULT. It shows a
sleep jumper; its actual solder state and circuitry still need inspection before
assuming GPIO10 can control sleep. EEP is active high; ULT active low per wiring.

The schematic shows GPIO9 attached to BOOT and GPIO8 to an on-board LED circuit.
GPIO2/8/9 are strapping pins; the selected wiring uses all three. Check boot and
I2C waveforms with the actual attached loads. Keep the user's GPIO choices for
now; no remapping has been assumed. Reference:
https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32c3/schematic-checklist.html

User confirms 6 V bridge supply and encoders on the wheel shafts, 1:1. Motor
rating/current, measured geometry, polarity and physical side assignment remain
unverified. User authorizes provisional sides/geometry/limits for later tests.

## Revisions observed upstream

All hashes below were checked using public Git HEAD retrieval. They identify
audited inputs, **not tested full-integration pins**.

| Component | Revision | Inspected API / integration issue |
| --- | --- | --- |
| esp-mt6701 | eb9e0ac6f1c7ea8274cee3d453d5a0c230552c9f | Local header, README and build metadata: `mt6701_init_with_timeout`, `mt6701_read_raw_angle`, tracking-from-sample APIs. Application supplies I2C device handle, so each device address is chosen by the application. Native counts: 16384. |
| esp_drv8833 | 527f5c17b118a4067e2671048d8965da86309499 | Local header/README, command and LEDC implementation. `create`, `set_speed(right,left)`, `coast`, state/fault and delete. **No brake API.** Zero on both wheels also sleeps when EEP exists. ULT only notifies; application must stop on fault. |
| esp_pid | 9ff1debce5d9ea6a2d9c73d1de2921d053e90a61 | Downloaded exact public header: value-owned `esp_pid_t`, `init/reset/update/get_state`, configurable saturation, back-calculation and maximum dt. Full source/build/license audit pending. Do not substitute a newer header from an unrelated application copy. |
| esp_angle_lut | ff8f51fad4dd9be596905d0ddb19114f2caf74f3 | Downloaded public header and matching local header: singleton `init/apply/install/read/set_enabled/clear/get_status`. Format v2. Needs additive per-instance API and independent persistence; no two-wheel integration yet. |
| kalman | 5bbe0a624acbf06548bb963a8498dc7a19c92b3b | Retrieved README for future review only; no V1 build/runtime dependency. Full API audit deferred. |

Repositories: https://github.com/smartsensingme/esp-mt6701,
https://github.com/smartsensingme/esp_drv8833,
https://github.com/smartsensingme/esp_pid,
https://github.com/smartsensingme/esp_angle_lut,
https://github.com/smartsensingme/kalman.

The bridge supports ESP-IDF >=6.0,<6.1 and C3 LEDC; MCPWM is not the C3 backend.
Its command mutex requests 20 ms (rounded to ticks), wake busy-waits 1000 us,
and reversal defaults to 50 us. A 5-ms control period needs dependency/ownership
analysis, not just setting a timer. MT6701 finite waits must convert to at least
one RTOS tick; example tick rate is 1000 Hz. Two sequential transfers consume
separate budgets. Full timing contract, late-release policy and fault recovery
must precede the periodic implementation.

Proposed LUT path: read each raw native angle once, apply its native-coordinate
table, then sign/unwrap/velocity in the robot owner. Preserve raw samples for
calibration. Avoid MT6701's uncorrected velocity and double application of LUT.

## Local upstream work

User explicitly authorizes updating owned component repositories locally before
later remote publication. Do not commit/push automatically. Known local roots:
`../esp_drv8833`, `../esp-mt6701`, and
`../as5600-sensor-zephyr/components/esp_angle_lut` (independent Git checkout,
clean when inspected). Verify clean state and applicable instructions again
before editing. Other application copies of PID are not automatically upstream.

Remaining audit: complete sources/examples/licenses/manifests for all required
dependencies, LUT multi-instance tests, genuine bridge brake operation, explicit
coast/sleep policy, local override build and final published revision pins.

## Local changes verified after this audit

The table above records original revisions, before working-tree edits. Current
local `esp_angle_lut` implements handle-based operations and separate namespace
lifetimes while retaining legacy wrappers. Its native tests simulate individual
persistent writes, failed installs and corrupt-slot recovery. Readers must stop
for maintenance; atomic pointer publication alone does not protect reused buffers.

Current local `esp_drv8833` implements `brake` (static HIGH inputs),
`coast_awake`, a braking-state flag and a BRAKE fault context. Native tests cover
normal exit to drive/coast/sleep and best-effort stop after write errors. Brake
duration belongs to the future robot owner state machine; no driver timer exists.
Hardware transition pulses, current and stop distance still require measurement.

The dedicated `examples/dependency_smoke` uses explicit CMake paths and no remote
manifest pins for unpublished edits. C3 build passed with LEDC, with LUT and
MT6701 linked from their local roots. Revisions remain uncommitted/unpublished.
The original full source/license audit and final Component Manager consumer
verification against published revisions are still pending.

Geometry was subsequently measured by the user: wheel diameter 66.1 mm, driven
track 118.6 mm. The example uses radius 0.03305 m and track 0.1186 m. Bridge
supply is descriptive bench metadata, not a required PID/control input.

## Application-root integration update

The user requested all integration sources/builds in this project. Current source
locations are `components/esp_drv8833`, `components/esp-mt6701`,
`components/esp_angle_lut`, `components/esp_pid`, `components/esp_rt_diagnostics`
and `components/esp_rt_diagnostics_reporter`. Robot code is in
`components/esp_2wd_robot`, and the application is in `main/`. The sibling paths
above describe the earlier audit only; new edits now happen here.

PID source/public headers and README at 9ff1deb... were checked directly. The
periodic integration uses actual `esp_pid_init/reset/update` per wheel, signed
output bounds and back-calculation. No PID API changes were needed. Diagnostics
and reporter AGENTS/public headers/READMEs were read before integration; only the
reporter is declared as a direct acquisition dependency, with its compatible
accumulator revision resolved transitively for future Component Manager users.
The current app uses explicit local components, and their paths were verified
in the generated project description. The root remains offline/local via
Component Manager disabling. See `local-components.md` and `validation.md`.
