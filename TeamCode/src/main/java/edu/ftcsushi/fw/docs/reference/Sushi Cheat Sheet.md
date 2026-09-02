# Sushi Cheat Sheet

This is a reminder, not a first lesson. Read
[`Sushi in one picture`](<../getting-started/Framework Overview.md>) first.

## The ordinary program

Extend `FtcRobotOpMode` and override only `configure(RobotProgram)`:

```java
import edu.ftcsushi.fw.ftc.input.GamepadDevice;

@TeleOp(name = "My TeleOp")
public final class MyTeleOp extends FtcRobotOpMode {
    @Override
    protected void configure(RobotProgram program) {
        MyMechanism mechanism = program.output(
                new MyMechanism(hardwareMap, MyProfile.current().mechanism));
        MyTeleOpControls controls =
                new MyTeleOpControls(new GamepadDevice(gamepad1));

        controls.bind(
                program.callbackBindings(),
                program.taskBindings(),
                mechanism);
        program.presenter((clock, telemetry) ->
                telemetry.addData("mechanism", mechanism.status()));
    }
}
```

`GamepadDevice` and the optional two-controller `Gamepads` aggregate live in
`edu.ftcsushi.fw.ftc.input` because they accept raw FTC SDK gamepads. Their ordinary construction
calls are `new GamepadDevice(gamepad1)` and `Gamepads.create(gamepad1, gamepad2)`.

Do not override FTC `init`, `start`, `loop`, or `stop`; those callbacks are final. Do not create a
second `LoopClock` or `TaskRunner` in ordinary robot code.

## Program roles

| Declaration | Owner's job | Phase |
|---|---|---|
| `program.prestart(...)` | Data-only INIT selection and readiness | INIT, then freeze at START |
| `program.service(...)` | Upstream sensing, localization, or vendor heartbeat | Before controls |
| `program.callbackBindings()` | Map current signals to synchronous robot intent | Before Tasks |
| `program.taskBindings()` | Construct and enqueue a fresh Task from an input event; the shared FIFO runner starts it at the queue head | Bindings phase |
| `program.rootTask(...)` | One fresh Auto/root Task graph | START and Tasks phase |
| `program.output(...)` | Realize and stop a mechanism's private Plants | After Tasks |
| `program.drive(...)` | Sample one final `DriveSource` and write one sink | Output order |
| `program.presenter(...)` | Add read-only telemetry rows | Last, before one commit |
| `program.stopHandoff(...)` | Advanced cleanup-gated state publication | Normal active STOP only |

Managed active order:

```text
LoopClock -> Services -> Bindings -> Tasks -> Outputs/Drive -> Presenters -> telemetry commit
```

## Buttons and axes

```java
public void bind(CallbackBindings callbackBindings,
                 TaskBindings taskBindings,
                 MyMechanism mechanism) {
    callbackBindings.onRise(driver.a(), mechanism::requestCollect);
    callbackBindings.onFall(driver.leftBumper(), mechanism::releaseManualMode);
    callbackBindings.mirrorOnChange(
            driver.rightBumper(),
            held -> mechanism.requestSlowMode(held));
    callbackBindings.copyEachCycle(
            driver.leftTrigger(),
            mechanism::requestManualPower);
    taskBindings.onRise(driver.y(), mechanism::createScoreTask);
}
```

Use `onRise(...)` for one action per press. Use `mirrorOnChange(...)` for a persistent held state.
Use `copyEachCycle(...)` for a frame-valued command that must be refreshed every loop.

The controls constructor creates and retains `driver`; it does not register behavior. Call this
surface-first `bind(...)` method exactly once. When a button launches behavior over time, use the
`TaskBindings` argument and supply a fresh Task:

```java
taskBindings.onRise(driver.y(), mechanism::createScoreTask);
```

## Direct mecanum drive

```java
GamepadDevice driver = new GamepadDevice(gamepad1);

DriveSource driveSource = new GamepadDriveSource(
        driver.leftX(),
        driver.leftY(),
        driver.rightX(),
        GamepadDriveSource.Config.defaults()
).scaledWhen(driver.rightBumper(), 0.35, 0.20);

program.drive(driveSource, FtcDrives.mecanum(hardwareMap, profile.drive));
```

Compose one final robot-centric `DriveSource`; declare one final `DriveCommandSink`. Sushi robot
axes are `+X` forward and `+Y` left. Positive omega turns counter-clockwise. Before enabling a
drivetrain, review every motor name and direction and set explicit, conservatively reduced
first-motion limits; framework defaults are normalized software values, not physical safety
approval.

## One power mechanism

The mechanism receives `HardwareMap` plus data-only config, snapshots that config, owns its Plant,
and implements `RobotProgram.Output`:

```java
final class IntakeMechanism implements RobotProgram.Output {
    private final Plant intake;

    IntakeMechanism(HardwareMap hardwareMap, IntakeConfig config) {
        IntakeConfig snapshot = config.copy();
        intake = FtcActuators.plant(hardwareMap)
                .motor(snapshot.motorName, snapshot.direction)
                .power()
                .targetFromNewCommand(0.0)
                .build();
    }

    void requestPower(double power) {
        intake.commandTarget().set(power);
    }

    Task runForSeconds(double power, double seconds) {
        return ScalarTasks.set(intake.commandTarget(), power)
                .forSeconds(seconds)
                .then(0.0)
                .build();
    }

    @Override
    public void update(LoopClock clock) {
        intake.update(clock);
    }

    @Override
    public void stop() {
        intake.stop();
    }
}
```

Direct power Plant targets are normalized to `[-1, +1]`. A Task changes the command target; the
mechanism remains the only Plant update and stop owner. `Plant.stop()` is final for that Plant
instance: it applies the realization's natural stop and makes later updates inert without rewriting
the resolver or command target. Use a zero command for active-match idle; construct a fresh Plant
for another lifecycle.

## Choose a Plant recipe

Before copying a new actuator into a mechanism, run
[`HW: Actuator Bring-up`](<../testing-calibration/Actuator Bring-up.md>) to establish its FTC
direction and any human-approved safe native endpoints. The wizard reports evidence; the mechanism
still chooses meaningful Plant units, bounds, mapping, reference, and production policy.

| Hardware goal | Builder branch |
|---|---|
| Motor or CR-servo power | `.power()` |
| Standard-servo position | `.position()`; choose Plant units/bounds, then map into native servo commands `[0, 1]` |
| Motor velocity | `.velocity()` then choose device-managed or framework-regulated control |
| Motor or CR-servo position | `.position()` then answer control, periodicity, bounds, units, reference, and feedback questions |

Ordinary FTC mechanisms begin with `FtcActuators.plant(hardwareMap)`. `Plants.fromOutputs()` is the
advanced hardware-neutral/custom-adapter gateway, not a second ordinary FTC recipe.

For FTC motor control:

| Need | Answer |
| --- | --- |
| Ordinary FTC controller | `deviceManaged()` |
| Deliberate FTC coefficient override | `deviceManagedWithOverrides()` then its required answer / `doneOverrides()` |
| Sushi standard software control | `regulated()` → feedback → units/tolerance → `setpointFrom...` → `feedbackFromPid(...)` → optional typed feedforward → optional `outputPowerLimitedTo(...)` |
| Custom complete law | the advanced `controlFromCustomRegulator(...)` exit after units/tolerance |

For an exclusive live experiment, pass one fresh canonical Plant recipe to
`FtcPanelsTuners.velocityControl(...)` or `positionControl(...)`. The finite range is permission for
manually selected physical targets, not an automatic sweep. Panels Update All edits a draft; A
accepts one complete immutable segment. See the
[`Control Tuning Workflow`](<../testing-calibration/Control Tuning Workflow.md>).

Keep the nouns distinct: the coordinate reference aligns Plant and native position, the target is
the final mechanism goal, the setpoint is the per-cycle control state, and output power is normalized
actuator effort.

Detailed staged examples: [`FTC Actuators & Plants`](<../ftc-boundary/FTC Actuators & Plants.md>).

## Tasks

```java
Task routine = Tasks.sequence(
        Tasks.runOnce(mechanism::requestReady),
        Tasks.branchOnOutcome(
                Tasks.waitUntil(mechanism.readySource(), 1.5),
                mechanism.createFeedTask(),
                Tasks.runOnce(mechanism::requestSafeIdle))
);

Task together = Tasks.parallelAll(firstTask, secondTask);
Task bounded = Tasks.withTimeout(operation, 2.0);
Task repairAfterAnyNaturalEnding = Tasks.sequenceOnCompletion(operation, repairRequest);
```

- Every Task object is single-use. Call the factory again to repeat it.
- `Tasks.sequence(...)` is the ordinary prerequisite chain: only exact `SUCCESS` starts the next
  eagerly constructed child; `TIMEOUT`, `CANCELLED`, and `UNKNOWN` remain exact and stop it.
- `Tasks.sequenceOnCompletion(...)` is only for intentional recovery, takeover, or repair after a
  valid natural ending. It retains the first non-success result. Direct cancellation starts no
  later child, so this is not `finally`.
- `Tasks.parallelAll(...)` waits for every child and succeeds only when all succeed. Matching
  non-success outcomes remain exact; mixed non-success outcomes report `UNKNOWN`.
- `Tasks.parallelDeadline(deadline, companions...)` lets one child own the lifetime.
- `Tasks.withTimeout(...)` adds one outer elapsed-time budget.
- `Tasks.branchOnOutcome(...)` runs only the exact-success or exact-timeout branch;
  `CANCELLED`/`UNKNOWN` fail closed.
- A cancellation hook must clean the requests owned by that Task; cancellation is not a direct
  hardware write.

Detailed rules: [`Tasks and Macros`](<../design/Tasks & Macros Quickstart.md>).

## Sources

```java
Source<MyStatus> status = Source.of(clock -> calculateStatus(clock)).memoized();
ScalarSource requestedPower = ScalarSource.of(() -> requestedPowerValue);
BooleanSource enabled = BooleanSource.of(() -> enabledValue);
Source<MyStatus> fixedStatus = Source.constant(MyStatus.idle());
```

Use `Source.of(...)` for a clock-aware object value, primitive `of(...)` for clockless primitive
leaves, and `constant(...)` for a fixed value. Stateful sources own their documented same-cycle
behavior; do not add private cycle caches in ordinary robot code.

## Units and bounds

- Plant bounds, target values, position/velocity tolerances, and references use **plant units**.
- Only names containing `Native` or a controller-native unit such as `Ticks` use native units.
- Standard-servo native position is `[0, 1]`.
- Direct motor and CR-servo power is `[-1, +1]`.
- Distances are inches and angles are radians unless the name says otherwise.
- Values that reach a final target, controller, or drive command must be finite.

## If something fails

Read the complete exception and the Driver Station telemetry; Sushi errors normally name the
invalid answer and the expected domain. Then use [`Common Problems`](<../troubleshooting/Common Problems.md>).

[Reference index](<README.md>) · [Glossary](<Glossary.md>) · [Docs home](<../README.md>)
