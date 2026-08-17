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
- Treat the drive configuration as a new review: keep `allowIntakeMotion = true` from the mechanism
  checkpoint, set `allowDriveMotion = false` while editing, then return it to `true` only after
  reviewing all four drive motors, the brake choice, and the three drive limits. The raised-wheel
  steps perform the later physical direction and response checks.

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

// Review this explicitly for your drivetrain.
profile.drive.enableZeroPowerBrake = true;

// Deliberately conservative first-motion limits.
profile.drive.drivebase.maxAxial = 0.25;
profile.drive.drivebase.maxLateral = 0.25;
profile.drive.drivebase.maxOmega = 0.20;
```

The directions and `true` brake setting show a common value shape, not facts about your drivetrain.
Before enabling the starter, run either **FW: Testers (Driver Station) → HW: Actuator Bring-up** or
**FW: Testers (Panels) → HW: Actuator Bring-up** with the wheels raised and isolate each of the four
configured motors. Test which temporary `Direction` makes that wheel's positive rotation contribute to
robot-forward motion, then copy the four reported values here. All five starter motor names must be
nonblank and distinct after trimming; matching
is case-sensitive. The explicit scales keep a full stick request conservative during first motion;
the framework defaults are valid software values, not reviewed physical facts or limits for your
robot. Increase one scale at a time only after controlled floor tests prove the current value safe.

After copying the isolated direction results and reviewing the names, brake choice, three
conservative scales, and intake from the mechanism checkpoint, set:

```java
profile.allowIntakeMotion = true;
profile.allowDriveMotion = true;
```

The root checks both permissions and any trimmed intake-versus-drive name collision before every
lookup. The intake mechanism then snapshots and validates its slice before its lookup and registers
immediately. `FtcDrives` owns validation of the copied drive slice before drive lookup. A bad later
drive slice can therefore fail after the intake and controls exist; managed `RuntimeException`
cleanup clears bindings, stops the registered intake, and rethrows the exact primary error. It does
not promise transactional rollback of SDK configuration effects.

## 2. Read the one-method FTC host

The complete robot-programming override is:

```java
@Override
protected void configure(RobotProgram program) {
    StarterProfile profile = StarterProfile.current();
    new StarterRobot(hardwareMap).declareTeleOp(program, profile, gamepad1);
}
```

Do not add `init()`, `start()`, `loop()`, or `stop()`. `FtcRobotOpMode` owns those callbacks and the
cleanup path.

## 3. Follow the TeleOp declarations

`StarterRobot.declareTeleOp(...)` registers the intake mechanism first, constructs the controls,
binds their meanings once, then declares one final source-driven drivetrain:

```java
import edu.ftcphoenix.fw.ftc.input.GamepadDevice;

StarterIntakeMechanism intake = program.output(
        new StarterIntakeMechanism(hardwareMap, profile.intake));

StarterTeleOpControls controls =
        new StarterTeleOpControls(new GamepadDevice(gamepad1));
controls.bind(program.callbackBindings(), intake);

program.drive(
        controls.driveSource(),
        FtcDrives.mecanum(hardwareMap, profile.drive));
```

`GamepadDevice` is the FTC boundary adapter: it accepts the SDK gamepad and gives controls ordinary
Phoenix axis and button sources. Its package keeps SDK details out of reusable input and drive
code; the beginner-facing `new GamepadDevice(gamepad1)` call stays direct.

The controls constructor creates stable input sources only. The explicit, one-shot `bind(...)` call
then declares what those inputs mean through the program-owned callback surface. The composition
root wires owners together; it does not reach into the intake Plant or manually run any owner.

## 4. Read the driver meanings

The drive source uses the left stick for translation and right stick X for rotation. Right bumper
selects a slower scale:

```java
driveSource = new GamepadDriveSource(
        driver.leftX(),
        driver.leftY(),
        driver.rightX(),
        GamepadDriveSource.Config.defaults()
).scaledWhen(
        driver.rightBumper(),
        SLOW_TRANSLATE_SCALE,
        SLOW_OMEGA_SCALE
);
```

`Config.defaults()` is a software-valid shaping baseline, not a claim that the resulting speed or
response is safe for this robot. `GamepadDriveSource` copies and validates that configuration once
when it is constructed; changing the original object afterward cannot retune the live source. If
you customize its deadband, exponents, or normalized maximum scales, use the exact domains in
[`Sources and Signals`](<../core-concepts/Sources and Signals.md#gamepad-drive-shaping>). The raised-
wheel test remains the evidence for physical direction and safe response.

The same controls owner maps button edges to the capability from the mechanism lesson:

```java
requiredCallbacks.onRise(
        driver.a(),
        () -> requiredIntake.setMode(StarterIntake.Mode.COLLECT));
requiredCallbacks.onRise(
        driver.b(),
        () -> requiredIntake.setMode(StarterIntake.Mode.EJECT));
requiredCallbacks.onRise(
        driver.x(),
        () -> requiredIntake.setMode(StarterIntake.Mode.STOPPED));
```

`onRise(...)` runs once when a button becomes pressed. The selected intake mode persists until
another semantic request replaces it.

If a mechanism should run only while a button is held, use an edge for each state transition:

```java
callbackBindings.onRise(driver.a(), () -> intake.setMode(StarterIntake.Mode.COLLECT));
callbackBindings.onFall(driver.a(), () -> intake.setMode(StarterIntake.Mode.STOPPED));
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
2. Reconfirm neutral controls, then press Driver Station START with the wheels raised. This production-mode check
   verifies that the copied directions, four-motor grouping, mecanum transform, and final writer
   work together; it is not replaced by the isolated wizard.
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

**INIT reports that intake or drive motion is not allowed.**

TeleOp requires both `allowIntakeMotion` and `allowDriveMotion`. Review the corresponding slice
before setting each permission; the booleans acknowledge human review but cannot prove it.

**INIT reports an invalid intake or drive configuration field.**

Read the owner-qualified message and correct that field. A later drive failure may occur after the
intake registered, but the managed failure path stops that intake before returning the original
error.

**A configured device cannot be found.**

Compare the exact case-sensitive name with the active Robot Controller configuration. Check for
spaces and confirm the expected configuration is selected.

**One wheel runs backward.**

Press STOP. Correct that wheel's `Direction` in the profile, rebuild, and repeat the raised-wheel
test. If the isolated result is uncertain, return to **HW: Actuator Bring-up** first. Do not
compensate with scattered negative powers.

**The intake keeps running after A or B is released.**

That is the checked-in edge-triggered design. Press X to request `STOPPED`. If the team's intended
meaning is “run while held,” replace that button's mapping with the paired `onRise`/`onFall` recipe
above.

**The robot moves during INIT.**

Press STOP and disconnect motion power. The managed program does not update bindings or outputs
during INIT; inspect wiring, another active owner, or local changes before continuing.

**Next:** [`Your first Task and Auto`](<First Task and Auto.md>)
