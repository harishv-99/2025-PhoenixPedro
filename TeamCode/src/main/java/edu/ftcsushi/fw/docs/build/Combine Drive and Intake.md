---
tags:
  - Build
---

# Combine drive and intake in one TeleOp

**Outcome:** drive continuously with the sticks while A/B/X independently request intake
`COLLECT`/`EJECT`/`STOPPED`, under one managed heartbeat and one terminal STOP path.

**Prerequisites:** complete the software checkpoints in [First Drive](<First Drive.md>) and
[Continuous Intake](<Continuous Intake.md>). Before a physical combined run, each owner must also
have passed its own isolated hardware gate.

## Critical production idea

The FTC host still declares the robot only once during INIT:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/opmode/StarterTeleOp.java -->
```java
@Override
protected void configure(RobotProgram program) {
    StarterProfile profile = StarterProfile.current();
    new StarterRobot(hardwareMap).declareTeleOp(program, profile, gamepad1);
}
```

The maintained composition root checks cross-owner names, constructs one intake owner, creates one
controls owner around the shared gamepad, connects its callbacks, and then declares one drive
source/sink pair:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/robot/StarterRobot.java -->
```java
requireDistinctMotorOwners(activeProfile);

StarterIntakeMechanism intake = declareIntake(program, activeProfile);
StarterTeleOpControls controls = new StarterTeleOpControls(
        new GamepadDevice(requiredGamepad));
controls.bind(program.callbackBindings(), intake);

program.drive(
        controls.driveSource(),
        FtcDrives.mecanum(hardwareMap, activeProfile.drive));
```

The production controls also keep one held-level precision-drive meaning in that same source:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/robot/StarterTeleOpControls.java -->
```java
driveSource = new GamepadDriveSource(
        this.driver.leftX(),
        this.driver.leftY(),
        this.driver.rightX(),
        GamepadDriveSource.Config.defaults()
).scaledWhen(this.driver.rightBumper(), SLOW_TRANSLATE_SCALE, SLOW_OMEGA_SCALE);
```

`StarterTeleOpControls` gives the shared `GamepadDevice` two different jobs. Its A/B/X sources
register short synchronous callbacks that replace the intake's persistent semantic request only on
a rising edge. Its three stick sources form a `DriveSource` that is sampled every active output
cycle. Holding a stick therefore keeps driving without manufacturing button events or Tasks.
Holding the right bumper scales the current translation to `0.35` and turn to `0.20`; because this
is continuous level-based intent, it belongs in the source decorator rather than a callback.

`declareIntake(...)` adds the intake mechanism as the first output and adds its read-only
presenter. `program.drive(...)` then adds the final source-driven drive output. Presenter
declaration time does not move telemetry ahead of actuation; the managed phase order remains:

`Clock → Bindings → Tasks → intake output → drive output → Presenters → one telemetry commit`

The name check trims the intake name and each drive name and rejects an exact duplicate before any
hardware lookup. This prevents two owners from silently resolving the same FTC motor. Each owner
then copies and validates its own configuration. Keep the checked-in low drive caps and fail-closed
motion permissions until the hardware gates have actually passed.

At FTC STOP, the program first cancels Tasks and clears bindings, then stops outputs in declaration
order. The intake Plant and drive sink each write physical zero immediately; no later loop is
needed, and repeated STOP is inert.

Notice:

- Buttons publish persistent named mechanism intent; held sticks are sampled continuous drive intent.
- The intake and drivetrain remain separate final writers even though one controls owner reads the
  same gamepad.
- Managed ordering makes same-cycle callback effects visible to outputs and cached telemetry.

## Files in this checkpoint

**Main:**

- [`StarterTeleOp`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/starter/opmode/StarterTeleOp.html>) — thin managed FTC host.
  [Complete source: `StarterTeleOp.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/opmode/StarterTeleOp.java>)
- [`StarterRobot`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/starter/robot/StarterRobot.html>) — owner construction, collision check, managed declarations, and presenter.
  [Complete source: `StarterRobot.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/robot/StarterRobot.java>)
- `StarterTeleOpControls` — shared gamepad meanings for drive and intake.
  [Complete source: `StarterTeleOpControls.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/robot/StarterTeleOpControls.java>)
- [`StarterProfile`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/starter/robot/StarterProfile.html>) — active names, directions, limits, and motion permissions.
  [Complete source: `StarterProfile.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/robot/StarterProfile.java>)

**Test:**

- [Complete source: `StarterDriveAndIntakeSoftwareScenarioTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/starter/robot/StarterDriveAndIntakeSoftwareScenarioTest.java>)

## Software checkpoint: one managed cycle serves both outcomes

- **Question:** Does one gamepad produce edge-triggered intake intent, normal/slow continuous drive
  output in the documented managed order, collision rejection, and total STOP?
- **Keep real:** `StarterProfile.current()` candidate values, `StarterRobot`,
  `StarterTeleOpControls`, the intake mechanism and Plant, the mecanum drive owner, and
  `FtcRobotOpMode` lifecycle.
- **Replace:** the physical gamepad, motors, and telemetry with software devices; replace the thin
  `StarterTeleOp` subclass with a test host that injects the private permission-enabled profile and
  deterministic runtime while retaining the managed base lifecycle.
- **Observe:** motor commands, cached intake status, event order, repeated drive writes, held-bumper
  scaling, early name rejection, and terminal zeros.
- **Cannot conclude:** wheel or intake direction, traction, current draw, mechanism clearance, or
  safe simultaneous physical motion.

The test flips the two fail-closed permissions only on its private copy of
`StarterProfile.current()` because every registered motor is a software probe. It does not copy the
axis map, intake power, or drive cap into a second test policy:

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/starter/robot/StarterDriveAndIntakeSoftwareScenarioTest.java -->
```java
driver.a = true;
driver.left_stick_y = -1.0f;
mode.advanceTo(0.02);
mode.loop();

// ASSERT: bindings run before the intake output, drive output, and cached presenter.
assertEquals(profile.intake.collectPower,
        hardware.motor(profile.intake.motorName).power(), 0.0);
assertEquals(profile.drive.drivebase.maxAxial,
        hardware.motor(profile.drive.wiring.frontLeftName).power(), 0.0);
assertEquals(StarterIntake.Mode.COLLECT, telemetry.dataValue("intake.mode"));
```

The same scenario then holds the bumper across two cycles, releases it, and separately checks turn.
That proves the production source—not a test-only mapping—uses held-level intent and applies both
documented scales. First it captures normal forward output and checks the first slow sample:

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/starter/robot/StarterDriveAndIntakeSoftwareScenarioTest.java -->
```java
double fullForwardPower =
        hardware.motor(profile.drive.wiring.frontLeftName).power();
driver.right_bumper = true;
mode.advanceTo(0.06);
mode.loop();
assertEquals(fullForwardPower * StarterTeleOpControls.SLOW_TRANSLATE_SCALE,
        hardware.motor(profile.drive.wiring.frontLeftName).power(), 1e-9);
```

The next cycle is still slow without another event, while releasing the level immediately restores
the normal translation scale:

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/starter/robot/StarterDriveAndIntakeSoftwareScenarioTest.java -->
```java
mode.advanceTo(0.08);
mode.loop();
assertEquals(fullForwardPower * StarterTeleOpControls.SLOW_TRANSLATE_SCALE,
        hardware.motor(profile.drive.wiring.frontLeftName).power(), 1e-9);

// RELEASE: level-based precision mode ends immediately; normal translation returns.
driver.right_bumper = false;
mode.advanceTo(0.10);
mode.loop();
assertEquals(fullForwardPower,
        hardware.motor(profile.drive.wiring.frontLeftName).power(), 1e-9);
```

Finally, the scenario isolates turn and checks the separate omega scale:

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/starter/robot/StarterDriveAndIntakeSoftwareScenarioTest.java -->
```java
// TURN: the same held-level decorator independently applies the documented omega scale.
driver.left_stick_y = 0.0f;
driver.right_stick_x = -1.0f;
mode.advanceTo(0.12);
mode.loop();
double fullTurnPower =
        hardware.motor(profile.drive.wiring.frontLeftName).power();
driver.right_bumper = true;
mode.advanceTo(0.14);
mode.loop();
assertEquals(fullTurnPower * StarterTeleOpControls.SLOW_OMEGA_SCALE,
        hardware.motor(profile.drive.wiring.frontLeftName).power(), 1e-9);
```

Run:

```powershell
.\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.starter.robot.StarterDriveAndIntakeSoftwareScenarioTest
```

**Read the causal chain:** one A rise runs in Bindings and selects `COLLECT`; the intake output
applies that request; the later drive output samples the still-held stick and writes its capped
signal; the presenter reads cached intake evidence; a held right bumper keeps scaling translation
on every cycle, release restores full translation, and the same level source scales turn; STOP
immediately zeros all five motors. The second test rejects whitespace-equivalent ownership before
lookup.

**Proves:** the maintained Starter composition shares input without sharing hardware ownership,
preserves its phase order, samples normal and held-bumper drive continuously, rejects one
cross-owner name collision, and stops every output through the managed lifecycle.

**Does not prove:** either mechanism is wired correctly or that operating both together is safe.

## Isolated hardware gate

Keep `StarterTeleOp` disabled and both motion permissions false while reviewing the combined
profile. Confirm all five FTC names are unique, retain the low drive caps, put the drivetrain on
blocks, clear the intake and wheel envelopes, and appoint an immediate STOP operator. Re-run each
isolated check first. Only then enable this OpMode and both permissions for a supervised test that
begins with centered sticks and `STOPPED`, checks one control at a time, and ends by verifying that
FTC STOP zeros both owners.

**Next gate:** reuse the proven intake capability in [one timed Auto](<Run One Timed Auto.md>).
