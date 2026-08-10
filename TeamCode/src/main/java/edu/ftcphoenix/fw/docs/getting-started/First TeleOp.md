# Your first TeleOp

## Goal

Add four mecanum drive motors to the reviewed starter profile, run the managed TeleOp, and use A/B/X
to control the intake through robot meanings.

**Time:** 30–45 minutes, including raised-wheel direction checks.

**Prerequisites:**

- the one-motor checkpoint from [`Your first mechanism`](<First Mechanism.md>);
- four drive motors in the active Robot Controller configuration; and
- the robot supported so its wheels can turn without driving away.

**Files for this lesson:**

- [`StarterProfile.java`](<../../../robots/examples/starter/StarterProfile.java>) — drive names and
  directions plus the reviewed intake facts;
- [`StarterTeleOp.java`](<../../../robots/examples/starter/StarterTeleOp.java>) — FTC entry;
- [`StarterRobot.java`](<../../../robots/examples/starter/StarterRobot.java>) — TeleOp declarations;
- [`StarterTeleOpControls.java`](<../../../robots/examples/starter/StarterTeleOpControls.java>) —
  sticks, slow mode, and buttons.

## Safety

- Raise and secure the drive wheels for the first direction test.
- Keep the intake empty and clear while checking its button mappings.
- Keep one person ready to press STOP.
- Release both sticks and all triggers before every INIT. `GamepadDevice` calibrates the current
  axis positions as neutral when it is constructed, so a held axis would teach it the wrong
  baseline. Reconfirm neutral controls before START.
- Treat the drive configuration as a new review: set `hardwareConfigurationReviewed = false` while
  editing, then return it to `true` only after reviewing the configured facts for all five motors
  and the three drive limits. The raised-wheel steps perform the later physical direction check.

## 1. Add the four drive facts

Keep the intake values that passed the first mechanism checkpoint. In `StarterProfile.current()`,
fill the drive fields from the active Robot Controller configuration:

```java
profile.drive.wiring.frontLeftName = "YOUR_FRONT_LEFT_NAME";
profile.drive.wiring.frontRightName = "YOUR_FRONT_RIGHT_NAME";
profile.drive.wiring.backLeftName = "YOUR_BACK_LEFT_NAME";
profile.drive.wiring.backRightName = "YOUR_BACK_RIGHT_NAME";

profile.drive.wiring.frontLeftDirection = Direction.FORWARD;
profile.drive.wiring.frontRightDirection = Direction.REVERSE;
profile.drive.wiring.backLeftDirection = Direction.FORWARD;
profile.drive.wiring.backRightDirection = Direction.REVERSE;

// Deliberately conservative first-motion limits.
profile.drive.drivebase.maxAxial = 0.25;
profile.drive.drivebase.maxLateral = 0.25;
profile.drive.drivebase.maxOmega = 0.20;
```

The directions show a common value shape, not facts about your drivetrain. Verify each one with the
wheels raised. All five starter motor names must be nonblank and distinct after trimming; matching
is case-sensitive. The explicit scales keep a full stick request conservative during first motion;
the framework defaults are valid normalized values, not reviewed physical limits for your robot.
Increase one scale at a time only after controlled floor tests prove the current value safe.

After reviewing the names, directions, three conservative scales, and the intake from the mechanism
checkpoint, set:

```java
profile.hardwareConfigurationReviewed = true;
```

`StarterRobot.declareTeleOp(...)` validates both the drive and intake before its first hardware
lookup. The mode fails with one actionable issue list instead of starting a partial robot.

## 2. Read the one-method FTC host

The complete robot-programming override is:

```java
@Override
protected void configure(RobotProgram program) {
    new StarterRobot(hardwareMap, profile).declareTeleOp(program, gamepad1);
}
```

Do not add `init()`, `start()`, `loop()`, or `stop()`. `FtcRobotOpMode` owns those callbacks and the
cleanup path.

## 3. Follow the TeleOp declarations

`StarterRobot.declareTeleOp(...)` registers the intake mechanism first, constructs the controls,
then declares one final source-driven drivetrain:

```java
StarterIntakeMechanism intake = program.output(
        new StarterIntakeMechanism(hardwareMap, profile.intake));

StarterTeleOpControls controls = new StarterTeleOpControls(
        program.bindings(),
        new GamepadDevice(gamepad1),
        intake);

program.drive(
        controls.driveSource(),
        FtcDrives.mecanum(hardwareMap, profile.drive));
```

The composition root wires owners together. It does not reach into the intake Plant or manually run
any owner.

## 4. Read the driver meanings

The drive source uses the left stick for translation and right stick X for rotation. Right bumper
selects a slower scale:

```java
driveSource = new GamepadDriveSource(
        requiredDriver.leftX(),
        requiredDriver.leftY(),
        requiredDriver.rightX(),
        GamepadDriveSource.Config.defaults()
).scaledWhen(
        requiredDriver.rightBumper(),
        SLOW_TRANSLATE_SCALE,
        SLOW_OMEGA_SCALE
);
```

The same controls owner maps button edges to the capability from the mechanism lesson:

```java
requiredBindings.onRise(
        requiredDriver.a(),
        () -> requiredIntake.setMode(StarterIntake.Mode.COLLECT));
requiredBindings.onRise(
        requiredDriver.b(),
        () -> requiredIntake.setMode(StarterIntake.Mode.EJECT));
requiredBindings.onRise(
        requiredDriver.x(),
        () -> requiredIntake.setMode(StarterIntake.Mode.STOPPED));
```

`onRise(...)` runs once when a button becomes pressed. The selected intake mode persists until
another semantic request replaces it.

If a mechanism should run only while a button is held, use an edge for each state transition:

```java
bindings.onRise(driver.a(), () -> intake.setMode(StarterIntake.Mode.COLLECT));
bindings.onFall(driver.a(), () -> intake.setMode(StarterIntake.Mode.STOPPED));
```

That is clearer for a persistent motor request than repeating the same command with `whileHigh(...)`
on every loop.

## 5. Enable only the starter TeleOp

Remove `@Disabled` from `StarterTeleOp`. Re-add or retain `@Disabled` on `StarterAuto` while testing
TeleOp so the Driver Station choice is unambiguous. Keep the Pedro example disabled.

Compile and deploy:

```powershell
.\gradlew.bat --console=plain :TeamCode:compileDebugJavaWithJavac
```

Select **FW Starter: TeleOp**.

## 6. Test in stages

1. With both sticks and all triggers released, press INIT. Nothing should move.
2. Reconfirm neutral controls, then press START with the wheels raised.
3. Test forward, strafe, and turn separately at low stick input.
4. Hold right bumper and confirm translation and turning are slower.
5. Release the sticks and verify all drive motors command zero.
6. With the intake clear, tap A and verify `COLLECT`.
7. Tap X and verify immediate `STOPPED`.
8. Tap B and verify `EJECT`, then tap X again.
9. Press FTC STOP and verify both drive and intake stop.
10. Remove motion power, lower the stopped robot safely, and clear an open test area. Never lower or
    carry the robot while the OpMode is active.
11. Restore power with sticks and triggers released, then run a fresh INIT/START and repeat only small,
    separate forward, strafe, and turn requests on the floor. Press STOP after the checkpoint.

The presenter reports `intake.mode` and `intake.appliedTargetPower`, so the Driver Station shows the
semantic request and the Plant's retained applied target.

## Expected checkpoint

- The Driver Station lists **FW Starter: TeleOp**.
- INIT causes no hardware motion.
- Left stick commands translation and right stick X commands turning.
- Right bumper reduces translation and turn speed.
- The explicit first-motion limits keep axial, lateral, and turn commands conservative.
- A/B/X request collect/eject/stopped without touching the Plant from controls.
- Neutral sticks stop drive, X stops intake, and FTC STOP stops both outputs.

## Common problems

**INIT reports that `StarterProfile` is not ready for TeleOp.**

Read the complete issue list. TeleOp requires both the reviewed intake and all four drive names,
directions, and finite drive scales. Do not bypass validation.

**A configured device cannot be found.**

Compare the exact case-sensitive name with the active Robot Controller configuration. Check for
spaces and confirm the expected configuration is selected.

**One wheel runs backward.**

Press STOP. Correct that wheel's `Direction` in the profile, rebuild, and repeat the raised-wheel
test. Do not compensate with scattered negative powers.

**The intake keeps running after A or B is released.**

That is the checked-in edge-triggered design. Press X to request `STOPPED`. If the team's intended
meaning is “run while held,” replace that button's mapping with the paired `onRise`/`onFall` recipe
above.

**The robot moves during INIT.**

Press STOP and disconnect motion power. The managed program does not update bindings or outputs
during INIT; inspect wiring, another active owner, or local changes before continuing.

**Next:** [`Your first Task and Auto`](<First Task and Auto.md>)
