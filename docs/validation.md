# Application validation status — 2026-10-18

The current application is the synchronous USB console, GPIO maintenance entry
and explicitly requested five-second PID trial described in the root README.
It no longer contains the encoder-only boot test. Physical LEFT is B / 0x06 /
encoder sign +1, RIGHT is A / 0x46 / sign -1. ULT is informational in all modes.

The user has completed and accepted both-wheel LUT calibration and subsequently
accepted the PID bench response as sufficient to continue; refinement remains
possible. This acceptance is not a measured loaded-tracking, drift or timing
qualification. Calibration uses raw angle without Kalman. The board PID trial
uses CV sigma 0.05 deg / acceleration sigma 5 rad/s², PI 3/20/0 and a 75% cap.

Documentation/packaging checks passed on 2026-10-18:

- `sh tools/test.sh`: all eleven printed regression groups passed with ASan/UBSan.
- Both Octave PTY tests passed: mode acceptance/rejection and full 2400-record
  evaluation/replay plus truncated-transfer rejection.
- ESP-IDF 6.0/C3 build passed, binary 0x43020 bytes, 74% application partition free.
- A lexical comparison against the pre-documentation snapshot preserved all
  executable tokens/literals in the six application C/header files.
- All eight selected component hashes matched public GitHub main heads.

 Native ASan/UBSan tests exercise numeric control, simulated
plants, maintenance lifecycle/protocol, ULT policy, physical mapping and debounce.
PTY tests exercise Octave mode selection and full/truncated evaluation transfer
without opening the hardware port. ESP-IDF 6.0/C3 builds verify interfaces, not
physical timing. No new motor run, flash or hardware measurement accompanies a
documentation commit.

Still to qualify on the final hardware: loaded tracking, estimator response lag,
wheel slip/effective geometry, external I2C waveforms and supply integrity,
stop current/distance, sustained timing under competing load/peripheral failures,
and real NVS power-cut recovery. Historic reduced/off diagnostic builds below
are earlier evidence, not fresh qualification of this complete revision.

## Historical development observations

The remainder is preserved from the development log. Statements such as
“current”, “next” and “pending”, old physical-side labels and old boot modes refer
to their historical stage. They do not override the source, root README or
current status above. Use I2C addresses to interpret reversed historical labels.

> Physical-side correction (2026-10-17): historical LEFT/RIGHT labels below
> were reversed. Physical LEFT is bridge B / 0x06 / encoder sign +1;
> physical RIGHT is bridge A / 0x46 / encoder sign -1. Historical observations
> retain their original labels; use the I2C address to identify the sensor.

# Validation — application integration, 2026-10-17

## Current workspace

The latest user-supplied board logs show a running 200-Hz owner with no missed
releases or overruns in the supplied windows, but nearly all paired samples
fail validation/readout. Both addresses have reported ESP_ERR_INVALID_RESPONSE;
the logs do not establish the electrical cause or physical side mapping.
The user subsequently confirmed LEFT=0x46 and RIGHT=0x06. The current bench
firmware reads both sensors independently with per-side error/attempt counters,
leaving the bridge uninitialized. This firmware builds successfully for C3;
reliable two-sensor acquisition on hardware remains pending.

The root is an ESP-IDF application. `main/` contains the board integration;
`components/esp_2wd_robot/` is reusable. All selected upstream sources are local
checkouts in `components/`, verified against `build/project_description.json`.
Their base revisions and preserved dirty changes are in `local-components.md`.
Earlier scaffold examples/builds are historical and superseded.

## Passed software checks

`sh tools/test.sh`, using the local actual PID source, AddressSanitizer and
UndefinedBehaviorSanitizer:

- configuration, normalized references, differential kinematics, saturation,
  straight/reverse/pivot/arc and small-turn odometry;
- corrected native-count wrap in both directions, encoder sign, wheel-shaft
  ratio conversion, motor sign and raw-versus-corrected sample retention;
- two independent PIDs, acceleration ramp, anti-windup/reset, explicit re-arm,
  stale/failed/duplicate/implausible feedback, missed releases and deadlines;
- timed brake-to-coast, early brake completion, sensor failure during braking,
  emergency stop, output failures and failed-coast retry;
- two unequal synthetic motor plants converge using independent references;
- pre-existing bridge control/ULT plus new brake/coast regression cases;
- independent LUT persistence, invalid/corrupt slots, injected individual-write
  failures and legacy API compatibility.
- application ULT context policy: known transition captures, unknown transients,
  continuously LOW acknowledged captures and stale inactive history. Per-wheel
  encoder errors are retained separately in the coherent state.

Native output executables are created/cleaned under `build/native/`. The host
script selects the installed Xcode SDK on macOS to avoid the local command-line
SDK/linker mismatch. No dependency source is read from an external temp checkout.

ESP-IDF 6.0 / C3 application builds pass with three effective configurations:

| Profile | Build directory | Instrumentation |
| --- | --- | --- |
| Production/development default | `build/` | Enabled, detailed timing |
| Reduced | `build/diagnostics-reduced/` | Enabled, stages/intervals disabled |
| Disabled | `build/diagnostics-off/` | Disabled, operational timestamps retained |

The reduced/off sdkconfig copies were generated under `build/` from the effective
root config, changing only diagnostic enable/detail symbols. Each was compiled
with `IDF_COMPONENT_MANAGER=0`; root defaults were not overwritten. Build logs
are `build/application-build.log`, `build/diagnostics-reduced.log` and
`build/diagnostics-off.log`. Every selected application component path points to
this project's `components/`. Native whitespace checks and edited upstream
`git diff --check` pass. Main/robot C/header formatting was applied.

Effective root config checked: ESP32-C3, 1000-Hz FreeRTOS tick, MT6701 mutex
disabled for exclusive ownership, both watchdog protections enabled, dynamic
power management disabled. GPTimer resolution is checked at runtime; driver
setup must succeed before any robot reference can be applied.

## Scope and pending hardware evidence

The native controller tests do not simulate the FreeRTOS scheduler, interrupt
allocation, actual bus/bridge timing or electrical motor dynamics. Target builds
verify the actual RTOS, timer, diagnostics and application adapter interfaces.
The user performed the first board startup: MT6701 initialization returned
ESP_ERR_INVALID_RESPONSE before bridge/control creation. The prior initialization
order tested provisional left/0x06 first and did not reach right/0x46. Updated
startup probes/reports both addresses before returning failure; that hardware
retest is pending. No successful periodic/control run or physical timing/stop
measurements have been established. No physical measurements are inferred from
synthetic tests, and the agent has not flashed firmware.

The application boots passive (STOPPED): it samples and reports but issues no
start/movement command. Hardware checks still required: address-to-wheel and
bridge mapping, polarity, sleep jumper, bus pull-ups/waveforms, actual speed
limits and PID tuning, fault response, jitter/deadline behavior under competing
load, transition pulses, brake current/distance and LUT improvement. Mock NVS
failures are not physical power-cut qualification. See `control-timing.md` for
the timing/diagnostic contract and measurement limitations.

Calibration capture/orchestration and host installation workflow remain to be
integrated. The robot component has not been extracted into a separate Git
repository. No commits or pushes were made.

## Two-encoder manual test and next gate

User log: 189 valid displayed reads per side, zero read errors and no warning
or error lines over approximately 19 seconds at about 10 Hz, with manual wheel
rotation visible in both raw positions. This does not establish 200-Hz stability.
The next firmware uses encoder-only 200-Hz acquisition, 1-ms read timeouts and
five-second asynchronous diagnostic/error windows; board execution is pending.

Software checks for this bench step: ESP-IDF C3 build passed; native regression
suites passed; authored acquisition source passed whitespace diff checking.
These checks do not substitute for the pending board run.

## User 200-Hz board result and recovery test

User-supplied 18 windows (~90 s): 18,001 attempts per side; LEFT/0x46 had zero
errors, RIGHT/0x06 had one ESP_ERR_INVALID_RESPONSE in window 3 and later valid
reads. No delivered/inferred missed releases or deadline overruns; maximum
reported wake+processing was 692 us against 5000 us. This is measured evidence
for this protoboard run, not a worst-case bound or a proven fault cause.
The next bench firmware preserves the acquisition policy and adds independent
isolated/burst recovery counters and completion-to-completion recovery time.
Actual hardware validation of those new fields remains pending. It does not
change the normal robot controller's sensor-fault policy or actuate the bridge.

## Recovery result and quiet comparison

The subsequent user log contains 12,002 attempts per side over ~60 s. Left had
zero errors; right had six isolated ESP_ERR_INVALID_RESPONSE errors, each with
recovery on the next read (4817..4871 us completion-to-completion), no bursts.
All six errors occurred in log-affected windows, none in quiet windows. This
correlation motivates the next test but does not prove a cause. No missed
releases or deadline overruns were reported; maximum cycle was 636 us.
The current firmware removes periodic presentation during a 60-second run and
prints full-run totals only after stopping acquisition. Hardware result pending.

## Quiet result and PID gate

User quiet 60-second board run: 12,000 valid reads per side, zero errors,
zero missed/inferred releases and zero overruns; max wake/proc/cycle 14/573/587 us.
The user also confirms earlier direction/wiring checks and authorizes advancing
to PID with current signs. Next trial uses serial start, +1 rad/s each wheel,
25% duty cap and 5-s duration. Physical PID response/tuning remains pending.

## Open-loop characterization before PID

User reports motor only began turning slowly at ~70% PWM in earlier bench tests.
The current test therefore selects one motor by serial command, runs seven
2-second 60/65/70/75/70/65/60% plateaus, then coasts and prints raw encoder
statistics for both sides. Current PID 25% cap and gains remain unchanged.
No claim of physical motion or successful tracking is made; new board run pending.

## First open-loop attempt and isolated-error correction

User log: first acquisition had right/0x06 ESP_ERR_INVALID_RESPONSE; stage 0
reported applied=0, cycles=1, coast=ESP_OK. No PWM plateau was applied, so this
run provides no motor-duty response evidence. The bench-only policy now allows
isolated failures, requires a valid pair before new PWM and aborts/coasts at
three consecutive failures or >15-ms sensor age. Native boundary checks cover
those cutoffs. Board confirmation of the corrected sequence remains pending.

## Forward direction normalization

Both user open-loop runs completed ESP_OK/coast=ESP_OK, no missed releases or
overruns, with one tolerated right read error each. Left raw speed was negative,
right positive; user confirms positive bridge output moves both wheels forward.
Board encoder signs are therefore left=-1/right=+1; motor polarity stays intact.
Open-loop logs now expose raw and normalized wheel mean speeds. Hardware
validation of the configured closed-loop sign mapping remains pending.

## First characterized PID trial

Current board trial: +3 rad/s each, Kp=3 Ki=20 Kd=0, duty<=75%, 5 s after serial
start. Left feedback is inverted once, right unchanged; bridge polarity intact.
Reference limit=6 rad/s gives a 12-rad/s plausibility threshold above observed
open-loop peaks. Opt-in sensor tolerance is bounded at three consecutive bad
reads or a 15-ms feedback gap; default reusable policy remains first-error fault.
Native regression covers unchanged integral/output on held invalid reads,
feedback recovery using actual dt, third-error/stale faults and config bounds.
Physical tracking/starting response of this revision remains pending.
