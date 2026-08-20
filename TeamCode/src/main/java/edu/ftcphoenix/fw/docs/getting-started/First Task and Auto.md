# Your first Task and Auto

## Goal

Understand the timed Task used by the one-motor checkpoint, then adapt the starter Auto without
blocking the robot loop.

**Time:** 25–40 minutes.

**Prerequisites:**

- the safe one-motor Auto checkpoint from [`Your first mechanism`](<First Mechanism.md>);
- the reviewed intake slice with `allowIntakeMotion = true`; and
- the controls and ownership model from [`Your first TeleOp`](<First TeleOp.md>).

**Files for this lesson:**

- [`StarterIntakeMechanism.java`](<../../../robots/examples/starter/capability/intake/StarterIntakeMechanism.java>) —
  fresh timed Task factory;
- [`StarterRobot.java`](<../../../robots/examples/starter/robot/StarterRobot.java>) — root declaration;
- [`StarterAuto.java`](<../../../robots/examples/starter/opmode/StarterAuto.java>) — FTC Auto entry;
- [`ScalarTasks.java`](<../../actuation/ScalarTasks.java>) and
  [`Tasks.java`](<../../task/Tasks.java>) — current Task factories.

## Safety

- Secure the robot while keeping the unloaded intake clear and free to rotate, exactly as in the
  mechanism checkpoint. Never hold or stall the powered mechanism by hand.
- Keep the reviewed low powers in `StarterProfile` and leave `allowDriveMotion = false`; this Auto
  does not inspect or authorize drive.
- Disable the starter TeleOp while running the Auto checkpoint so the Driver Station choice is
  unambiguous.
- Keep one operator ready to press STOP. Active Task cancellation returns the intake request to
  zero; terminal mechanism stop applies hardware zero without rewriting that persistent request.

## 1. A Task describes behavior over time

A Task does not pause the thread. The managed program advances it alongside the rest of the robot.
The starter mechanism creates a fresh timed Task with:

```java
@Override
public Task collectForSeconds(double durationSec) {
    if (!(durationSec > 0.0) || !Double.isFinite(durationSec)) {
        throw new IllegalArgumentException(
                "durationSec must be finite and > 0, got " + durationSec);
    }
    return ScalarTasks.set(plant.commandTarget(), collectPower)
            .forSeconds(durationSec)
            .then(STOPPED_POWER)
            .build();
}
```

Read the builder as a sentence:

> Set the intake command to collect power, keep that request for this many seconds, then request
> stopped power.

Calling this method constructs a Task; it does not command the motor. The Task writes its first
request when the managed program starts it.

## 2. The ending is explicit

`.then(STOPPED_POWER)` applies the final request after normal timed completion and after active
cancellation. When cancellation happens while the managed loop remains active, the next output
phase realizes that request through the same Plant. FTC STOP and runtime failure are stronger
lifecycle endings: the host best-effort cancels active work and then calls the registered
mechanism's `stop()` immediately. The Plant applies its natural final stop and becomes terminal; it
does not depend on another ordinary output update or a rewritten command target.

The parallel ending choice is `.leaveThere()`, which keeps the timed request after completion or
active cancellation. Use it only when leaving that request active is the intended robot behavior.

For this intake, returning to zero is the safe and obvious policy.

## 3. Each Task object represents one run

Task instances are single-use. This method is a **factory** because every call executes `build()`
and returns a new object:

```java
Task firstRun = intake.collectForSeconds(0.35);
Task secondRun = intake.collectForSeconds(0.35);
```

Do not put the same Task object in two places or save it for another button press. Ask the capability
for a new Task whenever the behavior should run again.

## 4. Auto declares one root Task

`StarterRobot.declareAuto(...)` constructs and registers the same mechanism used by TeleOp, then
declares one root:

```java
StarterIntakeMechanism intake = program.output(
        new StarterIntakeMechanism(hardwareMap, profile.intake));
Task root = intake.collectForSeconds(collectDurationSec);
program.rootTask(root);
```

`StarterAuto` supplies the duration from one visible constant:

```java
private static final double COLLECT_DURATION_SEC = 0.75;

@Override
protected void configure(RobotProgram program) {
    StarterProfile profile = StarterProfile.current();
    new StarterRobot(hardwareMap)
            .declareAuto(program, profile, COLLECT_DURATION_SEC);
}
```

At FTC START, the framework starts the root and realizes its first request. It also cancels active
work and stops the registered mechanism on FTC STOP or a runtime failure.

## 5. Make a small duration adaptation

Change `COLLECT_DURATION_SEC` to another short, positive duration that is safe for the mechanism,
for example `0.35`. Compile, enable only `StarterAuto`, and repeat the secured, unloaded run.

The motor should run for the new duration and return to zero while Driver Station telemetry changes
`auto.idle` to `true` when the root finishes.

## 6. Compose two fresh pulses

For a second software exercise, replace the single root construction in `declareAuto(...)` with two
fresh capability Tasks separated by a non-blocking wait:

```java
Task root = Tasks.sequence(
        intake.collectForSeconds(0.35),
        Tasks.waitForSeconds(0.25),
        intake.collectForSeconds(0.35)
);
program.rootTask(root);
```

Add the current factory import:

```java
import edu.ftcphoenix.fw.task.Tasks;
```

Each collection call creates a distinct Task. `Tasks.waitForSeconds(...)` leaves the loop free to
run; do not replace it with `Thread.sleep(...)`.

Run this adaptation only if two short intake pulses are physically safe. Otherwise, compile it as a
software exercise and restore the single-pulse root.

## Expected checkpoint

- You can explain when the Task changes the Plant command and when hardware is updated.
- A changed duration changes the visible motor interval without blocking the OpMode.
- The Task returns the intake request to zero after completion and active cancellation.
- A sequence uses two separately built intake Tasks rather than the same Task twice.
- Auto remains a one-method managed `FtcRobotOpMode`; it owns no manual clock or runner.

## Common problems

**The Task appears to run during INIT.**

The checked-in managed program does not start the root during INIT. Confirm the motor is not owned
by another enabled OpMode or modified code, then inspect the selected Driver Station entry.

**The sequence reports that a Task is duplicated or single-use.**

Do not write `Tasks.sequence(oneTask, oneTask)`. Call `collectForSeconds(...)` separately for each
run so each child has a distinct identity.

**The intake never returns to zero.**

Confirm the timed branch ends with `.then(STOPPED_POWER)`, the mechanism remains registered with
`program.output(...)`, and no later control request replaces zero. Press STOP and power down before
debugging hardware.

**Can I use `sleep(...)` because Auto has no driver controls?**

No. Services, safety policy, telemetry, drive followers, and STOP handling still need the loop.
Represent elapsed time with Task factories.

**Auto is missing from Driver Station.**

Remove `@Disabled` from `StarterAuto`, compile and redeploy, and confirm the Driver Station is using
that Robot Controller build. Restore `@Disabled` when the checkpoint is finished if the example is
not a team match entry.

The four-lesson core beginner course is now complete.

**Optional next track:** [`Pedro Auto software walkthrough`](<First Pedro Auto.md>)
