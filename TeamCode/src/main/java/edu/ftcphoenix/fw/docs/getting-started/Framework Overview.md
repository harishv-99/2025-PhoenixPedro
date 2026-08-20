# Phoenix in five minutes

## Goal

Understand where ordinary TeleOp and Auto code belongs without learning the framework internals.

**Time:** 5–10 minutes

**Prerequisites:** Basic Java classes and FTC OpModes are helpful, but no Phoenix experience is
required.

**Files for this lesson:**

- [`FtcRobotOpMode.java`](<../../ftc/FtcRobotOpMode.java>)
- [`RobotProgram.java`](<../../ftc/RobotProgram.java>)
- [`StarterTeleOp.java`](<../../../robots/examples/starter/opmode/StarterTeleOp.java>)
- [`StarterRobot.java`](<../../../robots/examples/starter/robot/StarterRobot.java>)

**Safety:** This lesson only reads code. Do not enable a checked-in example or command hardware yet.

## The whole idea

Phoenix separates **what your robot should do** from **when the FTC loop calls it**.

Your OpMode declares a robot once:

```java
public final class StarterTeleOp extends FtcRobotOpMode {
    @Override
    protected void configure(RobotProgram program) {
        StarterProfile profile = StarterProfile.current();
        new StarterRobot(hardwareMap).declareTeleOp(program, profile, gamepad1);
    }
}
```

`FtcRobotOpMode` then owns INIT, START, each active loop, STOP, one telemetry commit, and fail-stop
cleanup. The fresh profile is consumed synchronously by this declaration; the root does not retain
an editable aggregate. Ordinary robot code does not override those callbacks.

## The beginner mental model

```text
gamepads and sensors
        |
        v
controls and Tasks choose robot intent
        |
        v
capabilities express meanings such as COLLECT or STOPPED
        |
        v
mechanism outputs and drive apply the final commands
        |
        v
hardware
```

The framework runs the declared roles in the safe order. Students work mostly with these five
ideas:

1. **`FtcRobotOpMode`** is the FTC host.
2. **`RobotProgram`** is the declaration surface received by `configure(...)`.
3. **Bindings** connect gamepad signals to semantic robot actions.
4. **Tasks** describe non-blocking behavior that takes time.
5. **Outputs and drive** are the owners that finally command hardware.

## What `RobotProgram` declarations mean

You will meet these methods in the course:

| Declaration | Meaning |
|---|---|
| `program.callbackBindings()` | Register synchronous button or axis meanings. |
| `program.taskBindings()` | Construct and enqueue a fresh Task from a control event. |
| `program.output(owner)` | Register a mechanism that owns update and safe stop. |
| `program.drive(source, sink)` | Connect the final drive intent to the drivetrain owner. |
| `program.rootTask(task)` | Declare the one Auto routine that starts at FTC START. |
| `program.presenter(...)` | Add read-only rows to the one telemetry frame. |

The program returns registered owners from `output(...)` and `drive(...)`, so construction and
ownership stay visible at the declaration site.

## TeleOp and Auto use the same robot vocabulary

The starter intake exposes meanings such as:

```java
void setMode(Mode mode);
Task collectForSeconds(double durationSec);
```

TeleOp controls call `setMode(...)`. Auto asks for a fresh `collectForSeconds(...)` Task. The
mechanism owns the hardware in both modes, so the two modes do not duplicate motor code.

## The three habits Phoenix expects

### Keep behavior non-blocking

Do not call `sleep(...)` or spin in `while (...)` waiting for time or a sensor. Build a Task and let
the managed program advance it while the rest of the robot remains responsive.

### Keep hardware inside its owner

An ordinary mechanism constructor receives `HardwareMap` plus data-only configuration, defensively
copies and validates its active slice before its own hardware lookup, privately builds its Plants,
and implements `RobotProgram.Output`. The composition root owns robot-level permissions and
cross-owner relationships; controls and Auto call capability methods instead of reaching into a
Plant or FTC motor.

### Create a fresh Task for every run

A Task instance represents one run. A method such as `collectForSeconds(...)` builds and returns a
new Task whenever a button or Auto routine needs that behavior again.

## What you can defer

You do not need these topics to finish the beginner course:

- the framework clock and same-cycle safeguards;
- direct Task-runner lifecycle;
- target resolvers, overlays, and custom Plant adapters;
- custom Task state machines;
- prestart selection and Auto-to-TeleOp handoff;
- Pedro adapter heartbeat and route-execution internals.

The framework and integration references document them when a robot actually needs them.

## Expected checkpoint

You can answer these questions:

- Which method does an ordinary Phoenix OpMode override? `configure(RobotProgram)`.
- Who calls the FTC lifecycle and updates the robot? `FtcRobotOpMode` and its `RobotProgram`.
- Where do button meanings live? In robot-owned controls registered through bindings.
- Where does motor or servo construction live? In the mechanism that owns the Plant.
- How does code wait without blocking? With a fresh Task.

## Common problems

**“Where is the loop method I should edit?”**

There is no student-owned loop method in the managed path. Declare owners and behavior in
`configure(...)`; the framework calls them.

**“Should my controls set motor power directly?”**

No. Controls call capability methods such as `setMode(...)`. The mechanism translates that intent
to its private Plant.

**“Do I need to understand every `RobotProgram` role?”**

No. The course introduces only the roles used by the starter. Services and prestart policy belong
to later robot features.

**Next:** [`Phoenix beginner course`](<Beginner's Guide.md>)
