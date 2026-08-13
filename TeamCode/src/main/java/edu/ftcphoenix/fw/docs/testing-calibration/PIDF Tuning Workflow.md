# PIDF Tuning Workflow

PID and feedforward tuning changes a controller while real hardware is moving. Phoenix uses one
systematic vocabulary for standard software control and provides one ready-made live Panels
workflow for FTC device-managed velocity PIDF:

```text
robot declaration
  -> FtcPanelsTuners.velocityPidf(...)
  -> fresh Plant from the production owner's canonical recipe
  -> one framework-owned tuning session
```

For that FTC workflow, the framework owns Panels drafts, complete-candidate validation, hot/cold
segments, controller
readback, charts, restoration, and Plant cleanup. Robot code declares only a safe positive
test-target range and how to build a fresh Plant; the motor name is answered once inside that Plant
recipe. Production TeleOp and Auto never read
Panels Configurables.

## Start here

- **Tuning the Phoenix flywheel now?** Go straight to
  [Phoenix flywheel: the ready-made Panels workflow](#phoenix-flywheel-the-ready-made-panels-workflow).
- **Adding FTC device-managed velocity tuning to another robot?** Read
  [Declare one framework tuner](#declare-one-framework-tuner-mentor-or-author).
- **Using Phoenix standard software control?** Read
  [Phoenix software-regulated control](#phoenix-software-regulated-control) after the shared
  workflow.

Ordinary mechanisms need none of this structure merely because they use checked-in PIDF values.

## Choose the controller production actually uses

Phoenix supports two genuinely different controller owners. Tune the controller used by the
production Plant; do not substitute one merely because its API looks familiar.

| Production Plant | Controller owner | Tuning path |
| --- | --- | --- |
| `.velocity().deviceManaged()` or `.deviceManagedWithOverrides()` | FTC motor controller | `FtcPanelsTuners.velocityPidf(...)` |
| `.position().regulated()` or `.velocity().regulated()` with inline `setpointFrom...` / `feedbackFromPid(...)` | Plant-owned Phoenix standard control | Checked-in Plant recipe and controlled fresh-Plant experiments; no ready-made live Panels editor yet |
| `.regulated()` with `controlFromCustomRegulator(...)` | Caller-supplied advanced regulator | Its owner must define the complete candidate, reset, evidence, and restoration contract |

The staged Plant builder remains the one ordinary FTC construction grammar. The FTC tuning
workflow's controller handle changes configuration only; the Plant remains the sole target and
actuation path.

## Declare one framework tuner (mentor or author)

Keep the production design ordinary:

```text
HardwareMap + defensively copied data-only config
        -> one mechanism/subsystem owner
        -> private Plants + semantic requests + update + stop
```

For an exclusive tuning OpMode, add one clearly named advanced seam that returns a **fresh Plant**
from the same canonical private recipe production calls. Then pass that factory to the framework:

```java
return FtcPanelsTuners.velocityPidf(
        "Lift Velocity PIDF",
        ScalarRange.bounded(config.minimumTestVelocity,
                            config.maximumTestVelocity),
        hardwareMap -> Lift.createVelocityPlantForTuning(hardwareMap, config));
```

The test range must be finite, bounded, and positive. Zero remains part of the Plant's command
range because B and automatic stop request `0.0`; it is deliberately not a runnable test target.
The lower test-range endpoint seeds the initial `testTarget` draft.

The fresh-Plant method and production constructor must call the same recipe rather than copy it:

```java
public static Plant createVelocityPlantForTuning(
        HardwareMap hardwareMap,
        LiftConfig config) {
    return buildVelocityPlant(hardwareMap, config.copy());
}

private static Plant buildVelocityPlant(HardwareMap hardwareMap,
                                        LiftConfig config) {
    // The production constructor calls this exact method too.
}
```

This method is an explicitly advanced diagnostic seam. It is not another ordinary mechanism
constructor, a public robot command API, or permission to share a Plant between OpModes. The
framework workflow is the returned Plant's sole heartbeat and lifecycle owner. Production owns a
different Plant instance built from the same recipe.

This split intentionally avoids extra robot-specific objects:

- the data-only config remains checked-in production authority;
- the ordinary mechanism remains the sole production owner; and
- the framework tester owns its fresh Plant, Panels draft, FTC controller session, experiment
  segments, evidence, and cleanup.

Do not add a robot-specific tuning-session class, parameter registry, tuner menu, or separate
flywheel/lift owner when `FtcPanelsTuners.velocityPidf(...)` already expresses the workflow. A thin
direct OpMode is enough:

```java
@TeleOp(name = "Robot: Lift Tuning (Panels)")
public final class LiftTuningOpMode extends FtcPanelsTeleOpTesterOpMode {
    public LiftTuningOpMode() {
        super(InputSource.PANELS, PanelsClientRequirement.EXACTLY_ONE);
    }

    @Override
    protected TeleOpTester createTester() {
        LiftConfig config = RobotProfile.current().lift.copy();
        return FtcPanelsTuners.velocityPidf(
                "Lift Velocity PIDF",
                ScalarRange.bounded(config.minimumTestVelocity,
                                    config.maximumTestVelocity),
                hardwareMap -> Lift.createVelocityPlantForTuning(hardwareMap, config));
    }
}
```

The framework's public static Configurables fields are UI drafts, not robot configuration.
Production classes must not import or read them. The one active tuner seeds those fields from
controller readback and captures them only after the operator presses A.

The generic factory is intentionally limited to FTC device-managed **velocity** PIDF. Device-managed
position has an outer position P plus an inner velocity PIDF, while standard software control owns
typed setpoint/profile and feedforward state inside its Plant. Do not force those controllers
through this factory.

## The shared workflow

1. Bring up direction, feedback sign, units, bounds, and immediate stop behavior first.
2. Secure the robot, clear people and loose objects, and choose conservative test targets.
3. Start the dedicated tuner with a zero request and its controller-readback baseline.
4. Edit every candidate field, publish the UI draft, and deliberately apply the complete candidate.
5. Observe target, measurement, error, signed absolute-speed rate, and the physical result that
   matters.
6. Request zero, then copy the accepted controller readback into the robot profile.
7. Review and commit the source change.
8. Stop the tuner and start a fresh production TeleOp or Auto to confirm the checked-in values.

A finite tuple is not automatically safe. Robot code still chooses safe Plant bounds and test
targets; the operator still owns physical precautions and emergency power removal.

## Phoenix flywheel: the ready-made Panels workflow

Start **Phoenix: Tuning (Panels)**. It opens **Phoenix Flywheel Velocity PIDF** directly; there is
no intermediate menu or second selection.

The thin OpMode calls `FtcPanelsTuners.velocityPidf(...)` and supplies a Plant factory backed by
`PhoenixScoring.createFlywheelPlantForTuning(...)`. That method uses the same private flywheel Plant
recipe as production scoring, including direction, target bounds, device-managed control,
feedback, tolerance, and mapping. The tuner receives a fresh exclusive Plant; it does not construct
the complete scoring mechanism, share the production Plant, or write the motor directly.

### Before INIT

- Complete motor direction and feedback-sign bring-up.
- Secure the shooter and keep the firing path clear.
- Connect **exactly one** Panels client. Do not keep a second browser tab or device connected.
- Keep physical access to robot power. Browser STOP is not an emergency stop.

Zero or multiple clients fail the run closed. Losing the one client also terminally stops the Plant
and best-effort restores the controller session baseline. Reconnecting cannot rearm that OpMode
instance; begin a fresh INIT.

### The normal loop

```text
edit every field -> Panels Update All -> A -> observe
                                      ^
                                      repeat while running

B when finished
```

A is the only tuning apply action and B is the recoverable zero-request action. X and Y have no
tuning action, and there is no second confirmation after A.

The Configurables fields owned by `FtcPanelsTuners` are:

- `kP`, `kI`, `kD`, and `kF` — the complete FTC velocity-PIDF tuple;
- `testTarget` — the requested Plant velocity in that Plant's units, inside the declared positive
  test range; and
- `autoStopAfterSec` — the duration of the newly captured test segment.

For Phoenix, `testTarget` is native ticks per second and its allowed test range is checked-in
`velocityMin..velocityMax`. `autoStopAfterSec > 0` requests zero after that many seconds. Exactly
`0` means **no automatic stop**: the segment continues until B, BACK, disconnect, failure, or
OpMode stop. Negative or non-finite values reject the whole candidate. A hot update restarts the
timer from the new segment's own start boundary.

Panels **Update All changes only the browser draft**. Configurables publishes fields one at a time
from its transport thread; it does not apply hardware and exposes no atomic batch-completion marker.
Wait for Update All to finish, verify that the displayed **Draft values** match all six intended
values, and only then press A. The OpMode owner reads all six fields after a short best-effort
quiescence interval, validates the resulting complete tuple, and applies it. That delay filters
ordinary in-flight changes; it is not transport atomicity. An invalid or still-changing draft
changes nothing: an already-running segment and its timer continue unchanged.

### What A and B mean

| Action | Result |
| --- | --- |
| A with no active segment | Wait non-blockingly for finite feedback and `plant.atTarget(0.0)`, apply/read back all four gains, then start a numbered `COLD_START` segment. |
| A while running | Apply/read back all four gains first, then commit the new target and duration as a numbered `HOT_UPDATE` segment. No deliberate zero is inserted. |
| B | Request `0.0` through the Plant's command target and end the current segment. The Plant remains usable for another A. |
| BACK, OpMode stop, disconnect, or terminal failure | Terminally stop the Plant and best-effort restore the exact controller configuration captured for this tuning session. |

B requests zero; it does not claim inertia has stopped the wheel. If A follows immediately, the
cold start waits for finite feedback and the Plant's own truthful `atTarget(0.0)` result. Phoenix's
canonical recipe makes that arrival decision with checked-in `velocityToleranceNative`.

A hot update is useful for comparing shot behavior at different gains and speeds without waiting
for a complete spin-down. The pinned FTC SDK setter does not intentionally change target, run mode,
power, or direction. It does **not** promise that firmware preserves controller history or that the
physical transition will be bumpless. Watch for dips, spikes, oscillation, and mechanism stress.

### Read the evidence correctly

The tuner separates facts that are easy to confuse:

- **browser draft** — the latest fields published by Panels; changed draft text is not applied;
- **captured request** — the complete candidate frozen by one accepted A press;
- **controller readback** — the FTC controller's accepted or quantized PIDF tuple;
- **current segment** — immutable segment ID, `COLD_START`/`HOT_UPDATE`, target, duration, and start;
- **last completed segment** — retained after the current segment ends; and
- **checked-in profile** — production authority only after readback values are copied, reviewed,
  and committed.

The tuner uses Panels' built-in Configurables, Combined Gamepad, and Graph views. Graph consumes
these stable finite numeric telemetry keys:

```text
tune.velocityPidf.segmentId
tune.velocityPidf.requestedTarget
tune.velocityPidf.measuredAbs
tune.velocityPidf.error
tune.velocityPidf.measuredAbsRatePerSec
```

Use target and measurement to judge tracking, error to compare settling, and the signed rate of
absolute measured speed plus the physical mechanism to notice acceleration and deceleration. A
negative rate means the absolute speed is falling; it is not an absolute acceleration magnitude.
Phoenix does not invent controller output power or internal P/I/D/F terms that the FTC device does
not expose. If feedback or the derived rate is unavailable, its availability row says
`UNAVAILABLE` and Graph receives a finite `0.0` presentation fallback. That fallback never counts
as stopped evidence.

The controller may quantize gains. Copy the displayed accepted **readback values**, not the UI
draft, into Phoenix's checked-in configuration:

```java
PhoenixProfile.current().scoring.applyFlywheelVelocityPIDF = true;
PhoenixProfile.current().scoring.flywheelVelKp = /* displayed kP readback */;
PhoenixProfile.current().scoring.flywheelVelKi = /* displayed kI readback */;
PhoenixProfile.current().scoring.flywheelVelKd = /* displayed kD readback */;
PhoenixProfile.current().scoring.flywheelVelKf = /* displayed kF readback */;
```

Do not copy `testTarget` or `autoStopAfterSec` into production merely because a test succeeded.
They define the experiment, not automatically the robot's shot policy.

## FTC device-managed velocity PIDF boundary

Production constructs a device-managed velocity Plant through the staged grammar:

```java
this.flywheel = FtcActuators.plant(hardwareMap)
        .motor(motorName, direction)
        .velocity()
        .deviceManagedWithOverrides()
            .velocityPidf(kP, kI, kD, kF)
        .bounded(0.0, maxTicksPerSec)
        .nativeUnits()
        .velocityTolerance(toleranceTicksPerSec)
        .targetFromNewCommand(0.0)
        .build();
```

`FtcPanelsTuners.velocityPidf(...)` internally derives the narrow
`FtcMotorControllers.velocityPidf(plant)` configuration handle from the supplied Plant. The Plant
must be a single-motor FTC device-managed velocity Plant from `FtcActuators`; there is no separate
motor-name answer that could select another controller, and one Plant can supply only one handle.
The tuner also verifies that zero and the complete positive test range lie inside that Plant's
target range before its first command. The
handle can set and read back a complete gain tuple and restore the captured initial
`RUN_USING_ENCODER` configuration, including its algorithm. It exposes no target, power, velocity,
direction, or run-mode command, so the supplied Plant remains the sole actuation writer.

The handle is an advanced FTC boundary, not another Plant builder. Robot code should normally use
the complete `FtcPanelsTuners` workflow instead of rebuilding its controller session, draft
capture, segment logic, or cleanup. Position control has a different controller shape and is not
represented by this velocity-only factory.

## Phoenix software-regulated control

Ordinary software-regulated Plants declare the complete standard model inline after units and
tolerance. There is no peer PIDF object for robot code to retain or mutate:

```java
this.flywheel = FtcActuators.plant(hardwareMap)
        .motor(motorName, direction)
        .velocity()
        .regulated()
            .internalEncoder()
        .bounded(0.0, maximumRpm)
        .scaleToNative(TICKS_PER_RPM)
        .velocityTolerance(toleranceRpm)
        .setpointFromAccelerationLimitedProfile(maximumRpmPerSec)
        .feedbackFromPid(kP, kI, kD)
        .feedbackIntegralLimitedTo(-0.15, 0.15)
        .feedforwardFromMotion(kS, kV, kA)
        .outputPowerLimitedTo(0.0, maximumPower)
        .targetFromNewCommand(0.0)
        .build();
```

Configuration values belong in the mechanism's checked-in data-only config, and production and
diagnostic construction call the same private Plant recipe. The current framework does not expose
a live-mutable standard-controller handle or a ready-made Panels workflow for this branch. Compare
software candidates with fresh exclusive Plant lifetimes; do not read Configurables continuously in
TeleOp/Auto or rebuild a live Plant in place. A later tuning capability can make the same model
editable without changing this robot-facing construction grammar.

### Tune the model in a stable order

1. **Prove the mechanism facts first.** Verify feedback sign, Plant/native scaling, coordinate
   reference, target bounds, safe output range, and stop behavior. Choose conservative profile
   velocity/acceleration limits.
2. **Characterize feedforward in its physical units.** For a flywheel or moving mechanism, estimate
   `kS`, then `kV`, then `kA` only when the chosen setpoint provides acceleration. For a lift,
   estimate signed constant `kG` at a safe stationary height before motion terms. For an arm, verify
   `positionAtMaximumGravity` and `radiansPerPlantUnit`, then tune `kG` across several angles before
   adding motion terms.
3. **Tune feedback around the model.** Start with `kP`, add `kD` only when damping is needed, and add
   `kI` last for persistent residual error. Use `feedbackIntegralLimitedTo(...)` deliberately;
   standard control prevents further integral growth into final saturation but cannot make an
   unsafe output range safe.
4. **Validate the complete path.** Exercise cold starts, target changes, reversals where legal,
   steady hold, voltage range, profile-settled `atTarget()`, failure cleanup, and terminal stop.

The equations and coefficient units are documented in
[`FTC Actuators & Plants`](<../ftc-boundary/FTC Actuators & Plants.md#framework-regulated-position-the-standard-control-model>).
Copy only reviewed values into the checked-in configuration and confirm them in a fresh production
mode.

`controlFromCustomRegulator(...)` remains the advanced exit for a complete nonlinear or otherwise
custom law. Its owner must define how a candidate is applied, how all nested state is reset, what a
hot transition means, what telemetry proves, and what restoration can truthfully claim. Deprecated
`PidfRegulator` and `ScalarRegulators.pid(...)`, `pidf(...)`, and
`setpointFeedforward(...)` exist only for current custom/tuning compatibility; do not use them as a
parallel ordinary recipe.

## Failure and restoration truth

- **Invalid or unstable candidate:** reject every field before effects and preserve the current
  segment/controller tuple.
- **Valid but poor candidate:** press B to request zero, then submit a known-good complete tuple.
- **FTC setter or readback failure:** terminally stop the Plant, best-effort restore the captured
  controller configuration, and end the tuner. A transport failure may leave device state
  uncertain; do not claim transactional rollback.
- **Software-regulator apply/reset/update failure:** terminally stop its owned Plant and begin a
  fresh Plant lifetime before trying again.
- **BACK, disconnect, or OpMode stop:** terminally stop the Plant and restore any separately
  captured controller configuration.

Restoring numbers cannot undo physical motion or prove that firmware controller history returned
to an earlier state. Only a controlled hardware test can validate the resulting transition.

## Avoid these patterns

- Reading Configurables continuously from production TeleOp or Auto.
- Treating Panels Update All as a hardware apply action.
- Applying one field at a time or reading a draft field repeatedly during one attempt.
- Calling controller, Plant, or hardware methods from a Panels/UI callback thread.
- Writing the motor directly or duplicating the production Plant recipe in a tuner.
- Constructing the complete production mechanism merely to tune one independently owned Plant.
- Adding a robot-specific tuning-session object, menu, or registry when the framework factory fits.
- Sharing one Plant instance between production and tuning OpModes.
- Claiming a hot update is bumpless or that a requested gain equals controller readback.
- Treating live values as production configuration before copying, reviewing, and committing them.

## Related reading

- [`Testing and calibration`](<README.md>)
- [`Actuator bring-up`](<Actuator Bring-up.md>)
- [`FTC Actuators & Plants`](<../ftc-boundary/FTC Actuators & Plants.md#13-velocity-bounds-mapping-and-tuning>)
- [`FTC UI Helpers`](<../ftc-boundary/FTC UI Helpers.md>)
- [`Recommended Robot Design`](<../design/Recommended Robot Design.md#keep-standard-software-control-inside-realization>)
- [`Phoenix calibration guide`](<../../../robots/phoenix/Phoenix Calibration Guide.md>)
