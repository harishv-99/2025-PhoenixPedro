---
tags:
  - Advanced
---

# Subsystem experiments

**Learning mode:** Architecture reference

An experiment is a robot-specific use of existing capabilities and tester infrastructure, not a
new framework abstraction. Its purpose is to answer one decision before ordinary robot code relies
on the answer. Software tests establish software behavior; a bounded experiment collects the
physical evidence software cannot invent.

Use this order:

```text
semantic test
    -> software device scenario
    -> optional modeled simulation when its fidelity serves the question
    -> canonical actuator bring-up
    -> reviewed lab card
    -> locked, bounded subsystem tester
    -> production-mechanism integration check
    -> promote only accepted configuration into robot use
```

Before powered work, use a
[focused Build software checkpoint](<../build/README.md>) or
a [software-device scenario](<Hardware-free Reference Scenarios.md>) to establish that
the production mechanism interprets injected inputs and sends expected outputs. A software
scenario supplies readings explicitly; it does not mirror commands into feedback or model physics.
Its green result removes software uncertainty, but it does not authorize motion or replace the
physical bring-up and lab card.

## Copyable lab card and results sheet

Copy this block into the team's normal experiment record before enabling motion:

```markdown
# Subsystem experiment: ____________________

- Experiment ID: ____________________
- Date / operator / safety observer: ____________________
- Robot and configuration revision: ____________________
- Evidence file or video location: ____________________

## Decision question

One falsifiable question: ________________________________________________

## Starting conditions

- Mechanism state and game-piece load: ____________________
- Battery / environment / field setup: ____________________
- Reviewed command envelope and units: ____________________
- Start control: __________  Abort control: __________
- Emergency STOP owner: ____________________
- Immediate stop conditions: ____________________________________________

## Evidence

- Computed fields, source, and units: ___________________________________
- Operator-observed fields: _____________________________________________
- Repetitions: ______
- Accept when ____________________ in ______ of ______ trials
  AND _________________________________________________________________

| Trial | Starting condition | Command | Computed result | Observed result | Stop? | Notes |
|---:|---|---|---|---|---|---|
| 1 | | | | | | |
| 2 | | | | | | |
| 3 | | | | | | |

## Decision

- [ ] Accept configuration
- [ ] Revise and repeat
- [ ] Reject configuration
- Reason and evidence location: _________________________________________
```

The adopting team supplies every blank. Checked-in examples do not choose a safe command, a
repetition count, or an acceptance threshold for another robot.

## Computed evidence versus observation

Driver Station telemetry always keeps the state and control hint visible. Add a trial number for
correlation, then show command, measurement, timing, error/readiness, and retained outcome only
while they describe the current or most recently frozen trial. Print a value only when the
experiment code must compute it. A person records directly visible facts—direction, clearance,
vibration, sound, jams, damage, containment, or game-piece result—in the external trial row.

Do not print `PASS` merely because a controller reached target. `TARGET_REACHED` is one computed
trial outcome; the lab card's complete criteria and operator observations determine accept, revise,
or reject. Do not persist trial results on the Robot Controller.

## Worked locked card: Reference flywheel spin-up

### Critical code

The maintained experiment validates and snapshots the locked criteria in `onInit()`, constructs the
mechanism only when motion was reviewed, admits A only while idle, freezes terminal evidence before
requesting reusable idle, and terminally stops the owned mechanism from `onStop()`.

**What to notice**

- Criteria are copied and validated before hardware construction; the checked-in motion lock stays false.
- INIT constructs the selected tester but does not command motion.
- One bounded trial freezes a single immutable terminal result before requesting reusable idle, so
  deceleration cannot rewrite its flywheel Status, authored target, elapsed time, or outcome.
- STOP terminally cleans the owned mechanism even after abort or partial progress.

**Key APIs**

- `BaseTeleOpTester`: supplies the shared non-blocking tester lifecycle and clock.
- `ReferenceFlywheelSpinUpCriteria`: owns the locked question, command, and powered-time boundary.
- [`ReferenceFlywheels.Status`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/reference/capability/flywheel/ReferenceFlywheels.Status.html>): combines one grouped Plant snapshot with independent per-wheel measurements and readiness without declaring the lab-card decision.
- `FtcTeleOpTesterOpMode`: hosts a fresh tester tree without creating another FTC loop.

[`ReferenceFlywheelSpinUpExperiment.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/tester/ReferenceFlywheelSpinUpExperiment.java>)
answers one narrow question: how quickly do both flywheels reach the reviewed velocity condition
from the card's starting state? The trial action changes only flywheel velocity and never requests
a release or transfer pulse because the focused mechanism owns only the two flywheels. No object is
launched or scored.

The checked-in card remains locked:

| Card decision | Reference location | Meaning |
|---|---|---|
| Motion review | `ReferenceFlywheelSpinUpCriteria.current().reviewedForMotion` | `false` prevents construction of the motion-capable mechanism. |
| Command | `targetVelocityTicksPerSec` | Team-reviewed target in encoder ticks per second, positive, above the copied tolerance, and no greater than the configured maximum. |
| Safety cap | `maximumPoweredRunSec` | Cooperative elapsed-time boundary checked each active loop; zero is applied on the first loop observed at or after it, not by a hard real-time cutoff. It is not automatically the acceptable spin-up threshold. |
| Computed evidence | terminal result plus `ReferenceFlywheels.Status` | Authored target and elapsed time, requested/applied target, left/right measured velocity and at-target facts, and readiness; display errors are derived from the frozen target and measurements. |
| Minimal display | experiment telemetry | State and control hint always; trial number plus target, left/right evidence, elapsed/spin-up time, and frozen result only when a trial supplies them. |
| Physical evidence | operator results row | Direction, vibration, sound, damage, clearance, and STOP response. |
| Decision | reviewed lab card | Accept, revise, or reject from all criteria; `TARGET_REACHED` alone is not `PASS`. |

The evidence flow is deliberately one-way:

```text
reviewed card values
    -> ReferenceFlywheelSpinUpCriteria
    -> ReferenceFlywheels.Status
    -> minimal telemetry with trialNumber
    -> operator matches the external row
    -> team decision
```

Press A to begin one trial and B to abort it. A terminal transition freezes one immutable result
containing the flywheel Status, authored target, and elapsed time before requesting zero, so
deceleration readings cannot change a retained `TARGET_REACHED`, `TIME_LIMIT_REACHED`, or `ABORTED`
result. A while `RUNNING` is ignored to prevent
overlapping or restarted work. After a terminal result, pressing A again increments the trial
number and starts a distinct row. Readiness published strictly before the powered-time boundary
freezes `TARGET_REACHED` in that same loop. If readiness has not already been observed when elapsed
time reaches the boundary, `TIME_LIMIT_REACHED` wins before the output update and zero is applied;
a measurement first available in that boundary cycle cannot relabel the retained timeout. Because
the tester is cooperative, it checks this boundary once per active loop and freezes the elapsed
time actually observed there, which may exceed `maximumPoweredRunSec`. The reviewed lab card and
STOP plan must allow for the worst-case loop delay.

Each trial ignores the Status present when it began. A same-valued ready publication retained from
the prior trial cannot finish the new trial before its own output heartbeat publishes evidence.

That boundary behavior relies on the flywheel owner's all-or-nothing publication timing. The experiment
checks the prior successful Status before the deadline, then checks newly published per-wheel
evidence after the output heartbeat. It does not replace those later per-wheel facts with the
grouped Plant's aggregate arrival sample taken earlier in the mechanism update.

## Two independent motion gates

The Reference example has two permissions with different owners:

- `ReferenceFlywheelSpinUpCriteria.current().reviewedForMotion` authorizes only this bounded tester
  after its lab-card review.
- `ReferenceFlywheelMechanismOpMode.MOTION_REVIEWED` is the separate, checked-in lock on the
  disabled mechanism-only host.

Enabling one does not enable, imply, or replace the other. Tester evidence may support a later
host decision, but enabling any production robot still requires its own complete review.

## Add a team-owned subsystem experiment

First give the subsystem a focused software device scenario with explicit input observations,
recorded outputs, and a **Proves / Does not prove / Next gate** evidence boundary. Then use the
existing tester lifecycle for facts that require hardware; do not create another FTC loop or
generic experiment framework:

1. Create data-only `<Subsystem>ExperimentCriteria` or reuse the subsystem `Spec`. Its `current()`
   returns a fresh locked value. Validate and defensively snapshot every retained criterion before
   hardware lookup.
2. Create `<Subsystem>Experiment extends BaseTeleOpTester`. Acquire its fresh production-aligned
   mechanism only in `onInit()` after the menu selects the tester. A partial construction failure
   must safely stop every resource it completed before ownership transfer.
3. Keep INIT observational. Display the lock, question, units, and controls; never command motion
   from `onInit()` or `onInitLoop(...)`.
4. Bind one explicit A start and B abort. Start one bounded trial, update each owned mechanism
   exactly once per shared-clock loop, and prevent overlapping trials.
5. At success, time limit, or abort, freeze the evidence first and then request the reusable idle
   state. `onStop()` performs full terminal cleanup even after partial initialization.
6. Add a supplier to `<Robot>Testers.create()` so every menu entry constructs a fresh inactive
   tester. Register `StandardTesters` there if the robot also exposes the canonical bring-up tools.
7. Keep the FTC entry thin: extend `FtcTeleOpTesterOpMode` and return the fresh tester tree from
   `createTester()`.
8. Record external observations outside the Robot Controller and promote configuration only after
   the reviewed decision accepts it.

## Maintained experiment slice

Create a criteria card, one `BaseTeleOpTester` experiment, and one robot-owned tester registry. The
checked-in criteria begin locked:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/tester/ReferenceFlywheelSpinUpCriteria.java -->
```java
ReferenceFlywheelSpinUpCriteria criteria = new ReferenceFlywheelSpinUpCriteria();
criteria.reviewedForMotion = false;
criteria.targetVelocityTicksPerSec = 3000.0;
criteria.maximumPoweredRunSec = 3.0;
return criteria;
```

The registry supplies a fresh experiment from that card for each menu entry; the experiment owns
the bounded trial state machine and hardware lifecycle described above.

- [`ReferenceExperimentTesters`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/reference/tester/ReferenceExperimentTesters.html>)
- [Complete source: `ReferenceFlywheelSpinUpCriteria.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/tester/ReferenceFlywheelSpinUpCriteria.java>)
- [Complete source: `ReferenceFlywheelSpinUpExperiment.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/tester/ReferenceFlywheelSpinUpExperiment.java>)
- [Complete source: `ReferenceExperimentTesters.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/tester/ReferenceExperimentTesters.java>)

## Verify the slice

Run:

```powershell
.\gradlew.bat --console=plain :TeamCode:compileDebugJavaWithJavac `
  :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.reference.tester.*
```

Expected checkpoint: the experiment compiles, lifecycle tests pass, and the checked-in menu reports
`LOCKED`. Hardware success still requires the reviewed lab card and supervised trials.

## Criterion shapes by subsystem

The shape of a criterion can be copied; its values cannot:

| Subsystem | Possible computed criterion shape | Operator record |
|---|---|---|
| Intake/transfer | successful acquisition sensor sequence in `N` of `M` trials; bounded cycle time | acquisition/release, jams, damage |
| Referenced lift | reference completes before its cap; move and hold error remain inside a team threshold | clearance, repeatability marks, unwanted contact |
| Flywheel | both wheels independently meet tolerance; bounded spin-up and post-disturbance recovery | direction, vibration, sound, game-piece condition |
| Vision/localization | observation age and residual remain within reviewed bounds in `N` of `M` setups | lighting, occlusion, field-layout notes |
| Parking/guidance | bounded final pose error plus literal full-footprint containment status | full containment, clearance, shared-area safety |

If the program cannot truthfully compute an answer, leave it out of telemetry. If the decision
requires a visible physical result, put that field in the operator row instead.

## Related workflow

Use [`Actuator bring-up`](<../testing-calibration/Actuator Bring-up.md>) before a subsystem trial and
the [`Control tuning workflow`](<../testing-calibration/Control Tuning Workflow.md>) for supported
controller experiments. The [Evidence and experiments](<../getting-started/learn-sushi/Evidence and Experiments.md>)
chapter explains the full evidence ladder and how to read status without overclaiming it.

[Choose another focused build](<../build/README.md>)

[Back to the examples index](<README.md>)
