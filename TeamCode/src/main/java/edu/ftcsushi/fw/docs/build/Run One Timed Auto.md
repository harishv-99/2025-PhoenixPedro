---
tags:
  - Build
---

# Run one timed intake Task in Auto

**Outcome:** start collecting exactly at FTC START, keep the request active for 0.75 seconds, then
request `STOPPED` without sleeping or writing a loop.

**Prerequisites:** complete [Continuous Intake](<Continuous Intake.md>), including its isolated
hardware gate before attempting physical motion. Drive is not part of this independent fixture.

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

The small robot-owned factory keeps the duration choice beside the Auto and constructs new work
whenever it is called:

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

The mechanism's Task uses the same semantic command and final Plant as TeleOp. Its explicit ending
publishes the safe request on both normal completion and active cancellation:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntakeMechanism.java -->
```java
return SemanticScalarTasks.set(modeCommand, Mode.COLLECT)
        .forSeconds(durationSec)
        .then(Mode.STOPPED)
        .build();
```

The Task changes a persistent request; the later mechanism output owns the motor write. FTC STOP
first cancels active root work, which selects `STOPPED`, and then terminally stops the Plant so
physical zero does not depend on another loop. A Task object may start only once. Repeating this
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
deliberately followed by START at ten seconds; the root still receives its complete 0.75-second
interval:

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

Run:

```powershell
.\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.starter.opmode.StarterTimedAutoSoftwareScenarioTest
```

**Read the causal chain:** INIT builds but does not start the root; START publishes `COLLECT` and
the same START call realizes it; pre-boundary loops retain the request; the exact duration boundary
publishes and realizes `STOPPED`. The second scenario stops early, observes `CANCELLED` and zero,
then proves another factory call returns a distinct Task while the started one rejects reuse.

**Proves:** managed START timing, cooperative duration, same-cycle safe completion, active STOP
cancellation, terminal output stop, and fresh single-use routine construction.

**Does not prove:** 0.75 seconds is physically sufficient or safe for a real intake.

## Isolated hardware gate

Keep `StarterAuto` disabled and `allowIntakeMotion` false while reviewing the motor name, direction,
power, duration, and clear mechanism envelope. Re-run the intake's dead-man direction check,
restrain loose material, and appoint an immediate STOP operator. Only then enable this independent
Auto and its intake permission for one supervised run. Observe that motion begins only at START,
ends near the reviewed duration, and FTC STOP zeros the motor during an early-abort check.

**Next gate:** compose feedback-aware prerequisite steps in [First Autonomous](<First Autonomous.md>).
