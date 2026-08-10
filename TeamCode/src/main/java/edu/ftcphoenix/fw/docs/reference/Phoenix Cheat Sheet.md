# Phoenix Cheat Sheet

This is a reminder, not a first lesson. Follow the
[`beginner course`](<../getting-started/Beginner's Guide.md>) once before using it.

## The ordinary program

Extend `FtcRobotOpMode` and override only `configure(RobotProgram)`:

```java
@TeleOp(name = "My TeleOp")
public final class MyTeleOp extends FtcRobotOpMode {
    @Override
    protected void configure(RobotProgram program) {
        MyMechanism mechanism = program.output(
                new MyMechanism(hardwareMap, MyProfile.current().mechanism));
        GamepadDevice driver = new GamepadDevice(gamepad1);

        program.bindings().onRise(driver.a(), mechanism::requestCollect);
        program.taskBindings().onRise(driver.y(), mechanism::createScoreTask);
        program.presenter((clock, telemetry) ->
                telemetry.addData("mechanism", mechanism.status()));
    }
}
```

Do not override FTC `init`, `start`, `loop`, or `stop`; those callbacks are final. Do not create a
second `LoopClock` or `TaskRunner` in ordinary robot code.

## Program roles

| Declaration | Owner's job | Phase |
|---|---|---|
| `program.prestart(...)` | Data-only INIT selection and readiness | INIT, then freeze at START |
| `program.service(...)` | Upstream sensing, localization, or vendor heartbeat | Before controls |
| `program.bindings()` | Map current signals to robot intent | Before Tasks |
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
GamepadDevice driver = new GamepadDevice(gamepad1);

program.bindings().onRise(driver.a(), mechanism::requestCollect);
program.bindings().onFall(driver.leftBumper(), mechanism::releaseManualMode);
program.bindings().mirrorOnChange(
        driver.rightBumper(),
        held -> mechanism.requestSlowMode(held));
program.bindings().copyEachCycle(
        driver.leftTrigger(),
        mechanism::requestManualPower);
```

Use `onRise(...)` for one action per press. Use `mirrorOnChange(...)` for a persistent held state.
Use `copyEachCycle(...)` for a frame-valued command that must be refreshed every loop.

When a button launches behavior over time, supply a fresh Task:

```java
program.taskBindings().onRise(driver.y(), mechanism::createScoreTask);
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

Compose one final robot-centric `DriveSource`; declare one final `DriveCommandSink`. Phoenix robot
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
        CleanupActions.attemptAll(
                () -> intake.commandTarget().set(0.0),
                intake::stop);
    }
}
```

Direct power Plant targets are normalized to `[-1, +1]`. A Task changes the command target; the
mechanism remains the only Plant update and stop owner.

## Choose a Plant recipe

| Hardware goal | Builder branch |
|---|---|
| Motor or CR-servo power | `.power()` |
| Standard-servo position | `.position()` with `[0, 1]` native servo commands |
| Motor velocity | `.velocity()` then choose device-managed or framework-regulated control |
| Motor or CR-servo position | `.position()` then answer control, periodicity, bounds, units, reference, and feedback questions |

Ordinary FTC mechanisms begin with `FtcActuators.plant(hardwareMap)`. `Plants.fromOutputs()` is the
advanced hardware-neutral/custom-adapter gateway, not a second ordinary FTC recipe.

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
```

- Every Task object is single-use. Call the factory again to repeat it.
- `Tasks.sequence(...)` runs children in order.
- `Tasks.parallelAll(...)` waits for every child.
- `Tasks.parallelDeadline(deadline, companions...)` lets one child own the lifetime.
- `Tasks.withTimeout(...)` adds one outer elapsed-time budget.
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

Read the complete exception and the Driver Station telemetry; Phoenix errors normally name the
invalid answer and the expected domain. Then use [`Common Problems`](<../troubleshooting/Common Problems.md>).

[Reference index](<README.md>) · [Glossary](<Glossary.md>) · [Docs home](<../README.md>)
