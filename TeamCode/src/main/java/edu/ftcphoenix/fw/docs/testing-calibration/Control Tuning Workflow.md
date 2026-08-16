# Control Tuning Workflow

Live control tuning while hardware moves is an experiment, not ordinary robot operation.
Phoenix therefore gives velocity and position tuning one framework-owned workflow:

```text
one tester declaration
  -> one fresh Plant from the production recipe
  -> the controller topology discovered from that completed Plant
  -> one explicit A press per immutable experiment segment
```

The workflow owns the Panels draft, complete-candidate validation, target changes, controller
readback, response metrics, history, restoration, and terminal Plant cleanup. Robot code supplies
only the experiment name, a finite allowed physical target range, and the canonical fresh-Plant
factory. It does not repeat a motor name, controller type, gain schema, target object, or output.

## Choose the workflow that matches production

```java
// Velocity: FTC device-managed or Phoenix standard software control.
return FtcPanelsTuners.velocityControl(
        "Flywheel velocity control",
        ScalarRange.bounded(config.minimumTestVelocity,
                            config.maximumTestVelocity),
        hardwareMap -> Shooter.createVelocityPlantForTuning(hardwareMap, config));

// Position: already referenced, statically mapped, or assume-current capable.
return FtcPanelsTuners.positionControl(
        "Arm position control",
        ScalarRange.bounded(config.minimumTestAngleRad,
                            config.maximumTestAngleRad),
        hardwareMap -> Arm.createPositionPlantForTuning(hardwareMap, config));

// Position that needs a home/index search. A fresh Task is built for each attempt.
return FtcPanelsTuners.positionControl(
        "Lift position control",
        ScalarRange.bounded(config.minimumTestHeightIn,
                            config.maximumTestHeightIn),
        hardwareMap -> Lift.createPositionPlantForTuning(hardwareMap, config),
        (hardwareMap, plant) -> Lift.createReferenceTask(hardwareMap, plant, config));
```

Both standard software control and FTC device-managed control use these same robot-facing
factories. The completed Plant decides which exact fields Panels shows:

| Completed Plant | Editable controller candidate |
| --- | --- |
| Standard control | PID plus only the selected `NONE`, `MOTION`, `LIFT`, or `ARM` feedforward gains |
| FTC device-managed velocity | Complete `RUN_USING_ENCODER` PIDF tuple for every member |
| Single-motor FTC device-managed position | Outer position P plus inner velocity PIDF |
| `controlFromCustomRegulator(...)` | Unsupported: the custom owner must define its own candidate and evidence contract |

Bounds, mappings, coordinate-reference policy, tolerance, motion-profile limits, gravity geometry,
PID sub-limits, voltage policy, target guards, and output-power limits remain checked-in Plant
facts. The tuner does not make every numeric builder answer mutable.

## The target range is permission, not a sweep

The `ScalarRange` passed to `velocityControl(...)` is the set of **velocity targets the operator is
allowed to type for this session**. It is not:

- a range of PID gains;
- a list that Phoenix automatically iterates;
- the Plant's full production range; or
- an instruction to sweep between its endpoints.

For example, `ScalarRange.bounded(1_500, 3_000)` says that an accepted segment may request one
manually chosen velocity from 1,500 through 3,000 Plant units per second. The initial draft is the
member of that range nearest zero. If the operator changes `experiment.targetVelocity` to 2,200
and presses A, that one segment targets 2,200. A later A may target 2,500, repeat 2,200 with new
gains, or repeat the complete prior request.

The complete allowed range must lie inside the completed Plant's finite target range. The Plant
range must also contain `0.0`, even when the allowed test range does not, because zero is the
velocity recovery target used before a cold start, by B, and by automatic stop.

For `positionControl(...)`, the range is a finite **physical experiment envelope**. Panels exposes
two exact endpoints inside it. Each accepted A alternates one leg to endpoint A or endpoint B; the
range is not automatically sampled. Current position must already lie inside the envelope before
the tuner arms. B and automatic hold capture current feedback and hold it; if ordinary overshoot
put the mechanism just outside the envelope but still inside the Plant range, recovery explicitly
targets the nearest experiment boundary.

## What one velocity experiment does

Suppose the active controller is software PID plus motion feedforward and the draft contains:

```text
controller.feedback.kP
controller.feedback.kI
controller.feedback.kD
controller.feedforward.kS
controller.feedforward.kV
controller.feedforward.kA
experiment.targetVelocity
experiment.autoStopAfterSec
```

The workflow is manual and deliberate:

1. Edit the complete active draft in Panels and use **Update All**.
2. Verify the displayed values, then press A.
3. The OpMode waits for the map to remain unchanged briefly, takes one synchronized snapshot, and
   rejects missing, extra, non-finite, or topology-incompatible fields before effects.
4. On the first A, or any A after B/timeout, Phoenix requests zero and waits non-blockingly for
   finite feedback plus `plant.atTarget(0.0)`.
5. If controller gains changed, Phoenix applies the complete candidate. A software controller is
   reset and reseeded from the current measurement; an FTC controller is written and read back.
6. Phoenix commits the captured target through the Plant's one command target and starts a numbered
   segment. The Plant's normal PID/feedforward loop then pursues that target every heartbeat.
7. Metrics and graphs describe the response. B ends the segment and requests zero. A positive
   `autoStopAfterSec` does the same automatically; zero disables the timer.

A target-only change does not rewrite or reset the controller. A controller change on a
single-motor velocity Plant may be applied while a segment is running; that is a hot transition,
not a bumpless-transition promise. A grouped FTC controller change first returns every member to a
verified cold zero because sequential SDK writes briefly give members different controller values.

### Why the PID is related to the chosen target

The test range selects the reference conditions under which the controller is judged. It does not
change the PID equation. During a segment, feedback still computes error from the current control
setpoint and measured velocity:

```text
error = velocitySetpoint - measuredVelocity
output = PID(error) + selected feedforward model
```

A direct setpoint equals the Plant's applied target. An acceleration-limited setpoint moves toward
that target and exposes a truthful acceleration term. Testing several manually chosen velocities
reveals whether one checked-in candidate behaves well across the mechanism's intended operating
range; Phoenix does not silently optimize the gains from those trials.

## Motion and lift feedforward in a velocity controller

Motion feedforward models effort associated with moving:

```text
kS * sign(velocitySetpoint) + kV * velocitySetpoint + kA * accelerationSetpoint
```

It is the normal choice for a flywheel or other mechanism whose load does not need a constant
gravity term. `kA` appears only when the selected setpoint profile provides acceleration evidence.

Lift feedforward uses the same motion terms and adds signed constant gravity support:

```text
kG + kS * sign(velocitySetpoint)
   + kV * velocitySetpoint
   + kA * accelerationSetpoint
```

That model is meaningful for a vertical carriage controlled by velocity: when the requested
velocity is zero, the motor may still need `kG` merely to avoid falling. It is usually the wrong
model for a horizontal flywheel. Model selection belongs in the canonical Plant recipe; the tuner
edits only the gains that model actually owns.

FTC device-managed velocity instead exposes the controller's native `velocity.kF`. That is the
SDK/firmware PIDF coefficient in native controller units; Phoenix does not relabel it as physical
`kV`, infer a gravity term, or claim a trajectory feedforward model that the device does not expose.

## Tune the physical model before residual feedback

This workflow records manual experiments; it does not choose coefficients. Use a supported
mechanism, conservative output policy, physical stop access, and one reviewed change at a time:

1. Prove the Plant units, direction, coordinate reference, safe target envelope, output limit, and
   profile constraints before interpreting any gain.
2. For a lift, keep conservative stabilizing feedback and adjust signed `kG` at several safe holds
   until drift and the displayed PID correction are small. For an arm, first verify the checked-in
   maximum-gravity position and radians-per-Plant-unit geometry, then test `kG` near maximum
   gravity, near vertical where its cosine term approaches zero, and on the opposite side where
   the term changes sign.
3. With a profiled moving setpoint, identify `kS` from very slow motion in both directions, `kV`
   from constant-speed portions, and `kA` from acceleration/deceleration evidence. Leave `kA` at
   zero when the experiment does not provide trustworthy acceleration evidence.
4. Tune `kP` and then `kD` for the remaining tracking error and damping. Add `kI` last, only when a
   persistent residual remains and the configured integral/output policy makes its behavior
   bounded and observable.
5. Repeat the complete path, loads, directions, battery conditions, and recovery tests. Copy only
   reviewed accepted/readback values into checked-in production configuration.

Do not begin a gravity-loaded mechanism by assuming zero feedback is a safe hold. Fixtures and
physical support remain required even when the software candidate is mathematically valid.

## Grouped FTC velocity Plants

Two motors may truthfully form one scalar velocity Plant—for example, two coupled flywheels that
receive one target and intentionally share one PIDF candidate. Phoenix supports that case without
pretending the FTC SDK writes are atomic:

- the completed Plant supplies the exact ordered motor identities; robot code does not name them
  again;
- the one logical candidate is validated before the first setter, then written sequentially;
- each member retains its own initial configuration, accepted readback, native commanded target,
  native measurement, native error, and mapped tolerance;
- gain changes require same-cycle zero evidence for every member; target-only changes may remain
  hot; and
- any partial setter/readback failure terminally stops the Plant and best-effort restores every
  captured member. State is reported uncertain rather than as a transactional rollback.

Per-member evidence matters because an average group measurement can hide equal and opposite
errors. Sharing one candidate is appropriate only when the robot recipe intentionally treats the
motors as one scalar mechanism. Motors needing independently tuned candidates should be modeled as
separate controller owners.

## What one position experiment does

Position tuning uses two exact physical endpoints, not velocity's zero/spin cycle:

1. During INIT the tester claims the completed PositionPlant but produces no normal control output.
2. If a coordinate reference already exists—or assume-current is the declared policy—the tester
   samples current feedback, stages that same legal position through the Plant command graph, and
   reseeds standard control before its first normal update.
3. A `needsReference(...)` Plant requires the four-argument factory. The first A creates and starts
   one fresh single-use reference Task. The Task owns search/index motion while the same Plant stays
   alive. Success must actually establish the coordinate reference; Phoenix then stages current
   hold before normal control resumes. B cancels an active attempt and a later A builds a fresh one.
4. After the hold is proven, A captures the complete draft and authorizes exactly one alternating
   leg to endpoint A or B. A controller change is accepted only at a settled hold/endpoint.
5. B or `autoHoldAfterSec` samples current position once in that heartbeat before normal
   realization, stages the legal hold through the Plant, and lets the normal update consume that
   same memoized sample. BACK, disconnect, failure, or OpMode stop is terminal: the Plant stops and a
   gravity-loaded mechanism may fall, so support it physically.

Periodic position Plants are not inherently rejected. The first workflow supports only exact
unwrapped command targets: endpoint `370` means the physical unwrapped coordinate `370`, not
“equivalent to 10 degrees, choose a turn later.” Equivalent-position selection is deferred because
it could change the physical representative during a trial. An ARM feedforward on a periodic Plant
also requires the configured period and radians-per-Plant-unit geometry to describe whole gravity
cycles consistently.

FTC device-managed position has two controller loops. Its editable candidate is:

```text
outer RUN_TO_POSITION kP
inner RUN_USING_ENCODER kP, kI, kD, kF
```

Output-power policy and position tolerance remain fixed recipe facts. The initial exact algorithms
and tuples are captured and restored best-effort. The generic ready-made workflow deliberately
does not tune a grouped FTC position Plant: alignment, gravity/load sharing, safe hold, and
multi-member failure policy require mechanism-specific evidence beyond an aggregate position.

## Read segments, transitions, history, and metrics

Every accepted A creates an immutable segment ID. Its compact transition label compares only the
controller tuple and this segment's selected physical target with the last accepted segment:

| Label | Meaning |
| --- | --- |
| `TARGET_CHANGE` | Physical target/leg changed, controller did not |
| `CONTROLLER_CHANGE` | Controller changed, experiment target did not |
| `CONTROLLER_AND_TARGET_CHANGE` | Both changed |
| `REPEAT` | Controller and selected target are unchanged; A deliberately starts a new observation |

The label is useful for reading comparisons, but it is not the only retained information. For the
current OpMode lifetime, Phoenix keeps every completed segment's full candidate, controller
readbacks, both endpoint/timer fields, selected target, timing, termination reason, evidence
availability, and response metrics. Thus a timer-only edit creates a new full `REPEAT` record
without pretending the physical target changed or rewriting that target. The recent rows appear in
telemetry; the full in-memory history ends with the OpMode. It is not a production profile, an
on-device database, or a persisted optimizer history.

Velocity metrics include first truthful `atTarget` time, settling time, post-settled directional
droop, overshoot,
output-limited duration when observable, and post-settled disturbance peak/recovery. Position adds
travel direction, peak measured rate, hold error/drift, disturbance displacement/recovery, and the
same settling/output-limit facts. Standard software control can expose its setpoint, PID,
feedforward, pre-limit output, final output, and saturation facts. FTC device-managed controllers
do not expose those internal terms, so Phoenix marks them unavailable rather than inventing zeros.
Directional velocity droop/overshoot is likewise unavailable for a zero-velocity segment, and
directional position overshoot is unavailable for a zero-travel leg; neither case invents a
positive direction.
Every optional numeric Panels key has a companion `.available` key. Its numeric `0.0` fallback is
only transport presentation when availability is `0.0`; it is not evidence that the mechanism,
error, metric, or controller term was actually zero. Active and recent segment telemetry also
publishes the controller evidence under stable, domain-qualified keys, including each grouped FTC
member's name, native target, native measurement, native error, tolerance, and accepted readback.

`plant.atTarget(...)` and a small error mean the controller reached the commanded state; they do
not mean a ball entered a basket or an object stayed on a tray. Those are external observations.

## Recording distance and shot success

TUNE-03 intentionally does not add an arbitrary metadata map, success button, file writer, or
session database. Those features would mix robot-specific experiment meaning and Android storage
policy into a controller tuner. Use a simple lab sheet or spreadsheet keyed by the displayed short
session ID and segment ID:

```text
distanceIn | sessionId | segmentId | targetVelocity | shots | makes | accepted | notes
```

A practical shooter workflow is:

1. Place or measure the robot at one distance and write that distance in the next spreadsheet row.
2. Choose a target velocity/gain candidate, press A, and copy the displayed session/segment IDs.
3. Fire several controlled trials; record `shots` and `makes` in that same row.
4. Change the target or gains and press A for a new immutable segment; add another row.
5. Mark `accepted` only after the team's external success criterion is met.

After review, copy only the accepted finite distance/velocity rows into checked-in configuration.
Sort them by strictly increasing distance, then declare the table directly:

```java
InterpolatingTable1D velocityByRange = InterpolatingTable1D.ofSortedPairs(
        24.0, 3500.0,
        30.0, 3600.0,
        36.0, 3700.0);
```

`ofSortedPairs(...)` owns the authored-data check: every distance and velocity must be finite, and
duplicate or out-of-order distances are rejected during construction. Robot code does not need a
second validation loop. A live sensor range is runtime evidence, so gate on the finite lookup
result before commanding hardware; an unavailable range must remain unavailable rather than
select an endpoint.

This keeps exact controller evidence correlated without pretending Phoenix can infer success. If a
future robot repeatedly proves that one typed piece of external evidence should be framework-owned,
it can justify a separate bounded item; TUNE-03 does not introduce a string-keyed metadata registry.

## Panels draft and apply semantics

`FtcPanelsTuners.draft` is a synchronized ordered Map used only as Configurables transport. On each
tuner INIT Phoenix replaces it with exactly the active controller and experiment fields, then
refreshes Configurables. Do not add/remove keys in Panels. The schema stays fixed while ACTIVE.

Panels **Update All** edits the draft; it never applies hardware. A is the sole acceptance action.
Because Configurables publishes entry values individually, Phoenix waits for a short unchanged
interval, snapshots the map under its lock, and validates the entire exact schema before mutation.
That is coherent capture, not a claim that the browser transport provides an atomic transaction.

## Safety, restoration, and production adoption

- Bring up direction, feedback sign, units, bounds, coordinate reference, and stop behavior first.
- Secure/support gravity-loaded mechanisms and keep physical access to robot power. Browser STOP is
  not an emergency stop.
- Connect exactly one Panels client; disconnect terminally stops the session.
- Invalid/unstable drafts change nothing. A controller transport or post-mutation failure ends the
  session because state may be uncertain.
- Software-controller gains live only in the disposable fresh Plant. Terminal cleanup stops it;
  production reconstructs checked-in values next time.
- FTC controller settings are device state. If the workflow attempted a controller mutation,
  cleanup stops the Plant first, then best-effort restores every exact captured configuration.
  An untouched or target-only session performs no needless controller rewrite; restoration still
  cannot undo physical motion.
- Copy reviewed accepted/readback values into the mechanism's data-only configuration, commit them,
  and validate a fresh production TeleOp or Auto. A tuning draft/history is never production
  authority.

`controlFromCustomRegulator(...)` remains the advanced complete-law seam. Its owner must define a
typed complete candidate, reset/reseed rules, evidence, hot-transition policy, and restoration.
The obsolete peer `PidfRegulator`, `ScalarRegulators.pid(...)`, `pidf(...)`, and opaque
`setpointFeedforward(...)` construction paths have been removed; standard Plant control has one
current typed grammar and one Plant-derived tuning path.

## Related reading

- [`Testing and calibration`](<README.md>)
- [`Actuator bring-up`](<Actuator Bring-up.md>)
- [`FTC Actuators & Plants`](<../ftc-boundary/FTC Actuators & Plants.md>)
- [`FTC UI Helpers`](<../ftc-boundary/FTC UI Helpers.md>)
- [`Recommended Robot Design`](<../design/Recommended Robot Design.md>)
- [`Phoenix calibration guide`](<../../../robots/phoenix/Phoenix Calibration Guide.md>)
