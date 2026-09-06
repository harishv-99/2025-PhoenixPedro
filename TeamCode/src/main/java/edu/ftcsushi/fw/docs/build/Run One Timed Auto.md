---
tags:
  - Build
---

# Run one timed intake Task in Auto

**Outcome:** start a collection request at FTC START and end it on the first loop that observes
0.75 seconds have elapsed, without sleeping or writing a private loop.

**Knowledge before this page:** understand the persistent named request in
[Continuous Intake](<Continuous Intake.md>). Its explained observations are enough. No installation,
code edit, test run, hardware gate, or drive knowledge is required to read the whole lesson.

**One idea:** a Task gives an existing capability request a lifetime. Only an optional physical
run requires the intake's isolated hardware gate; drive is not part of this fixture.

## First pass: work that continues across loops

A Sushi `Task` is a bookmark for unfinished work. The FTC loop advances it a little, keeps running
the rest of the robot, and returns to the bookmark on the next loop. It is not a sleeping or blocked
thread.

The maintained Auto creates its intake and gives the program one top-level bookmark during INIT:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/opmode/StarterAuto.java -->
```java
@Override
protected void configure(RobotProgram program) {
    StarterProfile profile = StarterProfile.current();
    StarterIntake intake = new StarterRobot(hardwareMap).declareAuto(program, profile);
    program.rootTask(oneTimedCollect(intake));
}
```

Configuration builds and saves the work, but it does not start collecting. FTC START starts the
saved Task. That START call requests `COLLECT` and then lets the intake output apply the request.
Later loops check the elapsed time without `sleep()`; when 0.75 seconds have elapsed, the Task
requests `STOPPED` and finishes. Other loop work remains responsive throughout.

The small **factory method** returns a new bookmark every time it is called. `static` lets setup
call this method without constructing another Auto object. `Objects.requireNonNull(...)` rejects
a missing intake before asking it to build work:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/opmode/StarterAuto.java -->
```java
static Task oneTimedCollect(StarterIntake intake) {
    return Objects.requireNonNull(intake, "intake")
            .collectForSeconds(COLLECT_DURATION_SEC);
}
```

Each Task object is single-use. To run the behavior again, call the factory again instead of
restarting an old Task. On an early FTC STOP, the program cancels the active Task first; its
cancellation selects `STOPPED`, and cleanup then shuts down the intake output and commands zero.
A Task's **outcome** names how its work ended: this duration completes with `SUCCESS`, while
early cancellation reports `CANCELLED`. Success here means the software interval finished, not
that the intake collected an object.
Sequences, parallel work, and outcome branches belong in the later
[First Autonomous](<First Autonomous.md>) lesson.

## Full build: reconstruct the production path

Continue here to trace the complete production path and its expected software observations.
Reading completes the lesson; running the supplied scenario or
[authoring this slice](<README.md#author-in-your-robot>) is optional. Keep the example disabled until
the separate hardware gate says otherwise.

## Critical production idea

`StarterAuto` declares only the already-proven intake capability and gives the managed program one
fresh root Task:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/opmode/StarterAuto.java -->
```java
@Override
protected void configure(RobotProgram program) {
    StarterProfile profile = StarterProfile.current();
    StarterIntake intake = new StarterRobot(hardwareMap).declareAuto(program, profile);
    program.rootTask(oneTimedCollect(intake));
}
```

The profile still starts with `allowIntakeMotion = false`. `declareAuto(...)` enforces that lock
before it constructs the motion-capable intake owner:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/robot/StarterRobot.java -->
```java
StarterProfile activeProfile = Objects.requireNonNull(profile, "profile");
requireMotionAllowed(
        "Auto",
        "StarterProfile.allowIntakeMotion",
        activeProfile.allowIntakeMotion);

return declareIntake(program, activeProfile);
```

Thus the checked-in Auto fails closed during INIT rather than looking up or commanding the motor.
Only the test fixture and the supervised hardware gate make a fresh private profile copy and set
that one permission true; neither substitution changes the production declaration or routine.

The active duration is one named production value beside the Auto. Change this assignment when the
reviewed routine needs a different collection interval:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/opmode/StarterAuto.java -->
```java
public final class StarterAuto extends FtcRobotOpMode {

    private static final double COLLECT_DURATION_SEC = 0.75;
```

The small robot-owned factory uses that value and constructs new work whenever it is called:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/opmode/StarterAuto.java -->
```java
static Task oneTimedCollect(StarterIntake intake) {
    return Objects.requireNonNull(intake, "intake")
            .collectForSeconds(COLLECT_DURATION_SEC);
}
```

Calling the factory during INIT validates and builds a Task; it does not publish `COLLECT` or write
the motor. At FTC START, `FtcRobotOpMode` resets the one `LoopClock`, starts the root through the
program-owned runner, and then performs one downstream output update. That makes the positive-
duration request observable immediately and starts the 0.75-second interval at its own START
boundary—not during INIT and not from the previous loop's `dtSec()`.

The mechanism's Task uses the same semantic command and final Plant as TeleOp. Read this builder
as “select COLLECT when started, keep it for this duration, then select STOPPED.” Its explicit
ending publishes that request on both normal completion and active cancellation:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntakeMechanism.java -->
```java
return SemanticScalarTasks.set(modeCommand, Mode.COLLECT)
        .forSeconds(durationSec)
        .then(Mode.STOPPED)
        .build();
```

The Task changes a persistent request; the later mechanism output owns the motor write. FTC STOP
first cancels active root work, which selects `STOPPED`, and then terminally stops the Plant so it
submits zero immediately instead of waiting for another loop. A Task object may start only once. Repeating this
behavior means calling the factory again, never restarting the old object.

Notice:

- `program.rootTask(...)` declares one cooperative Auto graph; the managed host owns START,
  updates, cancellation, and cleanup.
- A timed Task gives the request a lifetime and outcome, while TeleOp's direct setter leaves a
  request selected until another command replaces it.
- `.then(STOPPED)` is request policy; the mechanism's `stop()` is still the terminal hardware stop.

## Files in this checkpoint

**Main:**

- [`StarterAuto`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/starter/opmode/StarterAuto.html>) — managed host and one timed routine choice.
  [Complete source: `StarterAuto.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/opmode/StarterAuto.java>)
- [`StarterIntake`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntake.html>) — shared direct and Task capability vocabulary.
  [Complete source: `StarterIntake.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntake.java>)
- [`StarterIntakeMechanism`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntakeMechanism.html>) — timed semantic Task factory and final Plant owner.
  [Complete source: `StarterIntakeMechanism.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntakeMechanism.java>)
- [`StarterRobot`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/starter/robot/StarterRobot.html>) — independent intake-only Auto declaration.
  [Complete source: `StarterRobot.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/robot/StarterRobot.java>)

**Test:**

- [Complete source: `StarterTimedAutoSoftwareScenarioTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/starter/opmode/StarterTimedAutoSoftwareScenarioTest.java>)

## Software checkpoint: time begins at START

**Expected observations:** INIT creates work but writes no power. START selects `COLLECT`. At
`0.74` seconds after START it is still active; the loop at `0.75` seconds selects `STOPPED` and
reports `SUCCESS`. Early STOP instead reports `CANCELLED` and submits motor zero. The supplied
clock values below expose those boundaries without requiring a real motor or clock experiment.

- **Question:** Does the exact Starter routine start on the managed START boundary, remain active
  for its duration, select `STOPPED` on completion or FTC STOP, and create fresh single-use work?
- **Keep real:** `StarterAuto.oneTimedCollect(...)`, `StarterRobot.declareAuto(...)`, the production
  intake mechanism and Plant, the semantic Task builder, and `FtcRobotOpMode` lifecycle.
- **Replace:** the FTC motor and telemetry with recording devices, the OpMode clock with a
  deterministic runtime override, and the checked-in fail-closed profile with a private copy whose
  intake permission alone is enabled. The production declaration, routine factory, and managed
  lifecycle stay real.
- **Observe:** no INIT power write, the START write, root outcome, semantic request, duration
  boundary, cancellation result, terminal motor zero, and distinct Task identity.
- **Cannot conclude:** physical timing under load, motor direction, current draw, collection, or a
  safe mechanism envelope.

The managed test supplies deterministic FTC runtime values. A five-second INIT history is
deliberately followed by START at ten seconds. Arrangement keeps the production declaration,
routine factory, and lifecycle, while replacing only the outside devices, clock, telemetry, and
that private profile copy:

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/starter/opmode/StarterTimedAutoSoftwareScenarioTest.java -->
```java
// ARRANGE: keep production declaration/routine/lifecycle; replace devices, time, and gate.
StarterProfile profile = enabledProfile();
FtcTestHardware hardware = new FtcTestHardware();
FtcTestHardware.MotorProbe motor = hardware.addMotor(profile.intake.motorName);
ManagedAuto mode = prepare(
        new ManagedAuto(profile),
        hardware,
        new StarterTestHardware.TelemetryProbe(),
        new Gamepad());
mode.advanceTo(5.0);
mode.init();
```

INIT is the before-request observation. START is the request boundary and first managed output
heartbeat:

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/starter/opmode/StarterTimedAutoSoftwareScenarioTest.java -->
```java
// BEFORE START: configuration built the fresh Task but neither Task nor output has run.
assertEquals(0, motor.powerWrites());
assertEquals(StarterIntake.Mode.STOPPED, mode.intake.status().mode());

// START: the host resets its clock, starts the root, then realizes COLLECT once.
mode.advanceTo(10.0);
mode.start();
assertEquals(profile.intake.collectPower, motor.power(), 0.0);
assertEquals(StarterIntake.Mode.COLLECT, mode.intake.status().mode());
```

The next heartbeat is still before the Task's own deadline, so the request remains active:

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/starter/opmode/StarterTimedAutoSoftwareScenarioTest.java -->
```java
// HEARTBEAT: INIT time was not charged; the request remains active before 0.75 seconds.
mode.advanceTo(10.74);
mode.loop();
assertFalse(mode.root.isComplete());
assertEquals(profile.intake.collectPower, motor.power(), 0.0);
```

At 10.75 seconds, the observation changes only because another managed heartbeat reaches the
duration boundary:

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/starter/opmode/StarterTimedAutoSoftwareScenarioTest.java -->
```java
mode.advanceTo(10.75);
mode.loop();
assertTrue(mode.root.isComplete());
assertEquals(TaskOutcome.SUCCESS, mode.root.getOutcome());
assertEquals(StarterIntake.Mode.STOPPED, mode.intake.status().mode());
assertEquals(0.0, motor.power(), 0.0);
mode.stop();
```

The second method supplies another managed host, `first`, with its own software motor, `firstMotor`,
and the same production configuration/routine. It stops while the root is active:

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/starter/opmode/StarterTimedAutoSoftwareScenarioTest.java -->
```java
// STOP: managed cancellation publishes the safe request before the Plant is terminally zeroed.
first.stop();
assertEquals(TaskOutcome.CANCELLED, first.root.getOutcome());
assertEquals(StarterIntake.Mode.STOPPED, first.intake.status().mode());
assertEquals(0.0, firstMotor.power(), 0.0);
```

**Optional regression detail:** the complete test also builds a second managed host and checks that
its root is a different object. Attempting to start the already-used first root fails fast with a
single-use error. Those checks protect the factory contract; writing a second host or exception
test is not required to understand this timed Auto.

Optionally run the maintained scenario after [software setup](<../getting-started/Build and Run.md>):

=== "Windows"

    ```powershell
    .\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.starter.opmode.StarterTimedAutoSoftwareScenarioTest
    ```

=== "macOS"

    ```bash
    ./gradlew --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.starter.opmode.StarterTimedAutoSoftwareScenarioTest
    ```

**Read the causal chain:** INIT builds but does not start the root; START publishes `COLLECT` and
the same START call realizes it; pre-boundary loops retain the request; the exact duration boundary
publishes and realizes `STOPPED`, which the final assertions observe. The second scenario stops
early, observes `CANCELLED` and zero, then constructs a distinct root and proves the first rejects
reuse.

**Proves:** managed START timing, cooperative duration, same-cycle safe completion, active STOP
cancellation, terminal output stop, and fresh single-use routine construction.

**Does not prove:** 0.75 seconds is physically sufficient or safe for a real intake.

**Reading checkpoint:** explain why five seconds spent in INIT do not shorten collection, why a
loop must observe the duration boundary, and why repeating the routine needs a new Task. Together
with the combined TeleOp lesson, this completes the basic managed-program learning path. It is a
timed mechanism Auto, not a drive route.

## Isolated hardware gate

This separate procedure applies only if you choose to operate the real intake in Auto.

Keep `StarterAuto` disabled and `allowIntakeMotion` false while reviewing the motor name, direction,
power, duration, and clear mechanism envelope. Re-run the intake's dead-man direction check,
restrain loose material, and appoint an immediate STOP operator. Only then enable this independent
Auto and its intake permission for one supervised run. Observe that motion begins only at START,
ends near the reviewed duration, and FTC STOP zeros the motor during an early-abort check.

**Next gate:** choose an optional extension from the [Build course](<README.md#add-feedback-when-your-robot-needs-it>).
For feedback-aware Auto, first understand [lift reference](<Referenced Lift.md>) and
[lift movement](<Move a Referenced Lift.md>), then [sequence those Tasks](<First Autonomous.md>).
You do not need those mechanisms to understand or author this timed Auto.
