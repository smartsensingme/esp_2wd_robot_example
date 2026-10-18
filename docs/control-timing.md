# Current application timing contract

This document describes the C3 integration, not a measured worst-case guarantee.
See the robot component README for numeric stage/event definitions and lifecycle
APIs. Historical encoder-only experiments now live in the MT6701 test project.

## Controller and supervisory work

The PID trial creates one priority-8/Core-0 owner with a 1-MHz GPTimer and
5000-us hardware auto-reload (200 Hz, integer periods). Installation occurs in
the owner task. ISR work captures minimal timestamps/notifies; no floating point,
I2C, logs or USB occurs in the ISR. Full notification counts are consumed.
Backlog faults/coasts without catch-up. A software startup epoch additionally
infers hidden periods; it does not measure hardware alarm-to-ISR latency exactly.

| Requirement | Current setting |
| --- | --- |
| Release-to-task lateness | At most 500 us against the software epoch |
| Release-to-output/first state copy | At most 5000 us |
| LEFT / RIGHT I2C stage targets | 1000 us each |
| Numeric control / output / first state copy targets | 500 / 500 / 250 us |
| Per-read driver timeout request | 1 ms; scheduler/recovery costs are additional |
| Sensor age / inter-wheel skew limits | 2500 us each |
| PID board maximum valid-sample gap | 15000 us |
| Sensor error policy | Hold/skip at most two consecutive invalid reads per side |
| Brake phase | Up to 100 ms nominal; checked on releases |

Reads are sequential with independent transaction-midpoint timestamps, not
simultaneous samples. LUT correction precedes unwrap/sign/shaft ratio and the
float CV estimator; odometry uses corrected travel. CV computation belongs in
the numeric stage. No allocation, flash write, ordinary formatted logging or
USB dump occurs in the periodic owner. Output driver lock/wake error paths may
consume more than a stage budget; exclusive ownership does not prove deadlines.

An invalid pair skips estimator/PI/odometry updates and holds prior output only
within the explicit count/freshness bounds. Third consecutive failure, stale or
ambiguous feedback, implausible speed, backlog/lateness, deadline or output error
faults/coasts. ULT is informational, including monitoring/acknowledgement errors.
A whole-turn aliased encoder reading cannot always be detected at this rate.

The console task waits up to one second for valid baseline feedback before START,
waits 20 ms to observe asynchronous arming, then submits +3 rad/s references.
During the at-most-five-second observation it polls state every 10 ms, copies at
most 50 snapshots at 10 Hz and observes GPIO mode requests. Stop waits 150 ms
before final snapshot/destroy; a failed destroy retains the controller handle.
Braking is not a measured mechanical-rest guarantee. No automatic fault recovery
or interactive console command processing occurs during this synchronous trial.

## Diagnostics and observation boundary

One accumulator belongs to the owner. Each begun cycle ends exactly once,
including faults. Stages 0..4 are LEFT I2C, RIGHT I2C, numeric control, output and
first state copy; events 0..5 count valid pair, invalid pair, fault cycle, drive
attempt, output error and inferred missing hardware periods. Intervals 0..1 join
successive successful LEFT/RIGHT sample midpoint timestamps.

The safety deadline ends at the first output/state copy. Fault-coast handling,
final state refresh and cycle end can follow and are included in diagnostic
processing. Snapshot extraction/handoff follows cycle end and consumes headroom
outside that measurement. Five-second windows continue even when reporting is
disabled. The board PID trial disables periodic reports; direct post-trial logs
show stored snapshots and lifetime counters. The optional component reporter is
priority 1/Core 0 with one-slot overwrite behavior; C3 has no second core.

Counters/maxima do not establish a proven worst-case bound. Retained handles on
shutdown errors can leave ownership active, so post-attempt logging must not be
interpreted as a guaranteed stopped-owner phase.

## Maintenance acquisition and transport

The calibration service temporarily owns the calling task's notification index
0 and acquisition priority. It allocates internal-RAM buffers before motion and
runs a 1-MHz GPTimer, 5000-us period, 500-us lateness limit, one notification per
processed release, finite sensor error/freshness policy and no catch-up.
Raw calibration: one wheel, 70/90/70% for 8 s/stage, 4800 maximum packed 7-byte
records (33,600 bytes). No Kalman is used for fitting the raw LUT.

Evaluation: one wheel, enabled LUT, 70/90/70% for 4 s/stage, 2400 releases. The
32-byte evaluation records need 76,800 additional internal-RAM bytes; direct,
25-ms LPF and CV estimates plus record-copy time are included in the deadline.
First velocity is invalid; bounded read gaps leave estimators untouched until
recovery with actual elapsed time. Acquisition buffers are borrowed for stopped
transfers and released through the service lifecycle.

At completion/error the service stops/disables/deletes GPTimer, drains pending
notifications, then performs the finite brake/coast sequence and restores caller
priority. No release source runs during the normal post-capture brake wait/dump.
Cleanup failures retain state/resources; do not start another owner prematurely.

The protocol drains output before acquisition. It formats/transfers only before
or after motion and delegates binary USB to main. No NVS operation runs in the
acquisition loop; LUT install/enable/readback are serialized idle maintenance
operations. USB read checks timeout on empty input, write uses an elapsed deadline
after stdout flush, drain bounds its driver wait. VFS flush time is outside those
explicit budgets. These are maintenance waits, not periodic-cycle guarantees.

## GPIO and hardware validation

The priority-1 input worker polls every 10 ms, debounces for 50 ms by default and
sets atomic flags. Optional LED updates occur every 250 ms. The console accepts
entry only after joining any controller and confirming coast. Scheduling adds
latency; GPIO polling/debounce is not a hard response deadline. Raw acquisition
is synchronous and does not interpret a new GPIO request as emergency stop.

Watchdogs remain enabled. Internal sample memory, task affinity and timer settings
do not establish flash/cache immunity. Measure with production settings and
representative USB/CPU activity and sensor/output failures: delivered/inferred
misses, deadline counts with denominators, execution/wake maxima, per-sensor
intervals, stop timing/current and observer cost. External timing capture is
needed to separate hardware alarm-to-ISR latency from ISR-to-task latency.
Historical quiet-run measurements are recorded in validation.md, not promoted
to guarantees for the current complete firmware.
