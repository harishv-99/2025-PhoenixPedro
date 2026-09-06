---
tags:
  - Learn
---

# Evidence and experiments

**Question:** What does a sensor fact, status row, or experiment result actually establish?

This is an on-demand evidence reference. Reading requires no sensor, code edit, installation, or
robot run. For the complete first observation path, read
[one switch without motion](<../../build/Read a Switch.md>). Its expected trace explains the
software result; optionally running the supplied test produces an observation of your own run.

## Climb the canonical evidence ladder

Choose the smallest boundary that answers the question. The five levels are defined in
[How to test a Sushi component](<../../testing-calibration/How to test a Sushi component.md#the-five-evidence-levels>):

| Level | What stays real | Limit |
| --- | --- | --- |
| 1. Semantic intent | control, Task, or policy decision; a recorder receives requests | no actuator or physical evidence |
| 2. Software-device scenario | production mechanism and explicit device observations | commands do not simulate physical response |
| 3. Supplied managed slice | production phase order and cleanup for the declared owners | no robot-specific response claim |
| 4. Maintainer regression | edge cases and structural contracts | useful to run; not necessarily a beginner code template |
| 5. Physical bring-up, calibration, or experiment | the actual assembly under recorded conditions | no claim beyond the observed conditions |

The ladder widens evidence scope, not permission. Reading an expected result is a learning
checkpoint; it does not claim the test was run. A passing software test cannot establish wiring,
motion, tuning, or physical safety.

A software scenario or modeled simulation is a test shape within this ladder.
A modeled simulation adds an authored dynamics model and must name its assumptions; it is still
not physical evidence. Passive software probes should never copy a commanded power, velocity, or
position automatically into feedback. Supply that external observation independently so a broken
feedback path cannot pass by reading its own request.

## Electrical level, semantic fact, and policy differ

An FTC digital input reports one of two electrical levels, HIGH or LOW. These levels do not decide
what “pressed” means. `FtcSensors.digitalHigh(...)` reads HIGH as true;
`digitalLow(...)` reads LOW as true. The robot author chooses the interpretation from the circuit.
Switch contacts can briefly flicker between levels while changing state. **Debounce** filters brief
changes so an accepted reading need not follow every raw reading. The switch lesson shows this in
a [sampled signal chart](<../../build/Read a Switch.md#first-pass-observations-every-loop>).

This debouncer adds elapsed loop intervals for sampled values that differ from its accepted state,
changing that state when the configured delay is reached. A sample that agrees with the accepted state clears that
pending change. Transitions between samples are unseen; this does not establish continuous
physical stability, discover polarity, or supply physical meaning.

The [switch Build lesson](<../../build/Read a Switch.md>) keeps raw and conditioned `pressed`
facts separate, supplies the effective debounce values beside the code, and shows one managed
owner publishing cached status. A presenter can display that evidence without choosing an action.
Stopping an intake because a switch is pressed would be an additional robot-policy requirement.

**Key APIs:** [`FtcSensors`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/FtcSensors.html>)
adapts the electrical input;
[`BooleanSource`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/core/source/BooleanSource.html>)
composes its meaning and shared-clock conditioning.

## Read status without manufacturing evidence

**Cached** facts are saved from the most recent update. A **snapshot** is a fixed record of them;
**immutable** means that record's contents cannot change. The Starter intake presenter reads such
a record through names meaningful to the intake:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/robot/StarterRobot.java -->
```java
StarterIntake.Status status = intake.status();
telemetry.addData("intake.mode", status.mode());
telemetry.addData("intake.appliedTargetPower", status.appliedPower());
```

The presenter formats the requested mode and applied target; it does not update the mechanism or
poll hardware. `appliedPower()` is the Plant's cached final target, not a measurement of motor
motion. The [intake scenario](<../../build/Continuous Intake.md#software-checkpoint-request-first-apply-on-heartbeat>)
shows why a request can change before the next output write.

The same distinction applies to feedback: a measured velocity can meet a controller's tolerance
(allowed difference from the request)
without proving balance, a successful launch, or a score. A telemetry row such as `objectPresent`
does not automatically act as a feeding interlock.

## Extend the evidence only for the next question

The optional
[`ReferenceInventoryStatusService`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/reference/capability/inventory/ReferenceInventoryStatusService.html>)
conditions three active-low inputs into one immutable cached snapshot. Its occupied-position count
counts asserted sensor positions, not proven physical objects; its order issue reports a pattern,
not a diagnosis of a broken sensor. The
[software-scenario index](<../../examples/Hardware-free Reference Scenarios.md>) links the maintained
inventory publication scenario without making that multi-sensor policy part of the first lesson.

For a physical subsystem experiment, first write the question, safe range, criterion, procedure,
stop conditions, and external observations in the
[Subsystem Experiments](<../../examples/Subsystem Experiments.md>) card. Software reports facts it
calculates, such as elapsed time and measured velocity. The operator records vibration, clearance,
sound, and other visible or audible outcomes.

If software reports `TARGET_REACHED` while the operator sees unacceptable vibration, that timing
condition was met but the physical configuration may still fail the team's criterion. Preserve
both facts. The [paired-flywheel example](<../../advanced/Paired Flywheel Velocity.md>) explains its
additional readiness evidence; the [Test & Tune path](<../../testing-calibration/README.md>)
provides the optional operational procedures.
