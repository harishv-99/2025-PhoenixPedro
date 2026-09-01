# Get your first robot driving

**Learning mode:** Buildable implementation

<!-- buildable-files: TeamCode/src/main/java/edu/ftcsushi/robots/examples/firstdrive/FirstDriveTeleOp.java -->

This one-file lesson gets a standard mecanum robot under joystick control. Complete
[Set up and verify the project](<Build and Run.md>) first.

## Goal

Copy one disabled TeleOp, verify four wiring facts, and reach a slow first drive. Controls own stick
meaning, the root wires one source to one sink, and the managed host owns looping and cleanup.

**Time:** about 25–35 minutes after the project builds.

**Safety boundary:** compilation cannot prove wiring, direction, clearance, braking, traction, or
STOP. Secure raised-wheel work on a stand and use clear space on the floor. Keep people assigned to
Driver Station STOP and robot power.

## 1. Copy the complete program

Create this concrete team-owned file:

`TeamCode/src/main/java/edu/ftcsushi/robots/myrobot/FirstDriveTeleOp.java`

Use `package edu.ftcsushi.robots.myrobot;`. The public class and filename must both remain
`FirstDriveTeleOp`. Rename the `@TeleOp`, but keep `@Disabled` until step 4.

### Critical code

The complete one-file milestone is short enough to keep open while you work:

<!-- source-file: TeamCode/src/main/java/edu/ftcsushi/robots/examples/firstdrive/FirstDriveTeleOp.java -->
```java
package edu.ftcsushi.robots.examples.firstdrive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.drive.DriveSource;
import edu.ftcsushi.fw.drive.source.GamepadDriveSource;
import edu.ftcsushi.fw.ftc.FtcDrives;
import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.ftc.input.GamepadDevice;

@TeleOp(name = "FW First Drive", group = "FW Examples")
@Disabled
public final class FirstDriveTeleOp extends FtcRobotOpMode {

    @Override
    protected void configure(RobotProgram program) {
        FirstDriveControls controls = new FirstDriveControls(new GamepadDevice(gamepad1));
        FtcDrives.MecanumConfig drive = FtcDrives.MecanumConfig.defaults();

        drive.wiring.frontLeftName = "frontLeftMotor";
        drive.wiring.frontRightName = "frontRightMotor";
        drive.wiring.backLeftName = "backLeftMotor";
        drive.wiring.backRightName = "backRightMotor";
        drive.wiring.frontLeftDirection = Direction.FORWARD;
        drive.wiring.frontRightDirection = Direction.REVERSE;
        drive.wiring.backLeftDirection = Direction.FORWARD;
        drive.wiring.backRightDirection = Direction.REVERSE;
        drive.enableZeroPowerBrake = true; // Review BRAKE versus FLOAT for this drivetrain.

        program.drive(controls.driveSource(), FtcDrives.mecanum(hardwareMap, drive));
    }

    private static final class FirstDriveControls {
        private static final double FIRST_RUN_TRANSLATION_SCALE = 0.25;
        private static final double FIRST_RUN_TURN_SCALE = 0.20;

        private final DriveSource driveSource;

        FirstDriveControls(GamepadDevice driver) {
            driveSource = new GamepadDriveSource(
                    driver.leftX(),   // Left/right translation.
                    driver.leftY(),   // Forward/back translation.
                    driver.rightX(),  // Clockwise/counter-clockwise turn.
                    GamepadDriveSource.Config.defaults())
                    .scaled(FIRST_RUN_TRANSLATION_SCALE, FIRST_RUN_TURN_SCALE);
        }

        DriveSource driveSource() {
            return driveSource;
        }
    }
}
```

**What to notice**

- `FirstDriveControls` owns stick meanings but never writes a motor.
- `configure(...)` declares one connection; there is no student `loop()` or scheduler.
- `.scaled(0.25, 0.20)` caps translation components at 25% and turn at 20%; combined axes can sum
  to a larger wheel command. The first gates therefore test one axis at a time. These are
  conservative starting values, not proof that motion is safe.
- The short `FtcDrives.mecanum(hardwareMap)` assumes reviewed defaults; this lesson's configured
  overload exposes names, directions, and BRAKE/FLOAT.
- The returned sink owns final motor writes; `program.drive(...)` owns managed connection/cleanup.

**Key APIs**

- `FtcRobotOpMode` — managed FTC host.
- `GamepadDevice` — stable gamepad Sources.
- `GamepadDriveSource` — three axes to drive intent.
- `FtcDrives.MecanumConfig` / `FtcDrives.mecanum(...)` — sink configuration/factory.
- `RobotProgram.drive(...)` — one drive path.

## 2. Replace assumptions with your wiring facts

The checked-in file makes every default name and direction visible:

| Wheel | Configuration name | Starting logical direction |
|---|---|---|
| Front left | `frontLeftMotor` | `Direction.FORWARD` |
| Front right | `frontRightMotor` | `Direction.REVERSE` |
| Back left | `backLeftMotor` | `Direction.FORWARD` |
| Back right | `backRightMotor` | `Direction.REVERSE` |

Run [HW: Actuator Bring-up](<../testing-calibration/Actuator Bring-up.md>) one motor at a time at low
power. Record its name and forward-rolling logical direction, then edit the matching
`drive.wiring.*` fields. A Robot Configuration name cannot fix polarity; `Direction` does.

`drive.enableZeroPowerBrake = true` declares the initial BRAKE choice; review and record BRAKE or
FLOAT before enabling. The stand cannot qualify stopping distance—the bounded floor gate does that
before scales rise. Gamepad defaults use `0.05` deadband, `1.5` exponents, and `1.0` internal
scales; the outer `.scaled(...)` supplies this lesson's limits.

**Run this:** complete four isolated bring-up records, pressing STOP during each.

**Expect this:** every name selects one wheel, its direction is unambiguous, and STOP ends output.

**If it fails:** correct the name or that wheel's `Direction`; do not reverse a joystick axis to
hide one wrong motor fact.

**Advance when:** all wiring matches the record and BRAKE/FLOAT is recorded.

## 3. Compile while motion remains disabled

From the repository root, compile all TeamCode, including your copy:

```powershell
.\gradlew.bat --console=plain :TeamCode:compileDebugJavaWithJavac
```

**Run this:** build, install using [Build and Run](<Build and Run.md>), and inspect the Driver
Station OpMode list without removing `@Disabled`.

**Expect this:** `BUILD SUCCESSFUL`, and your first-drive entry is absent from Driver Station.

**If it fails:** use the first compiler error. Check the package/path pair, filename/class-name
pair, and copied imports before changing framework code.

**Advance when:** the team-owned copy compiles and remains unavailable for motion.

## 4. Verify wheel patterns on the stand

After isolated checks pass, remove `@Disabled` from the copy, rebuild/install, secure the raised
robot, and mark each tire.

Use one tiny axis at a time. “Forward” means rotation that would roll that corner forward.

| Driver command | Front left | Front right | Back left | Back right |
|---|---|---|---|---|
| Left stick forward | forward | forward | forward | forward |
| Left stick right | forward | reverse | reverse | forward |
| Right stick right | forward | reverse | forward | reverse |
| Release both sticks | stop | stop | stop | stop |

**Run this:** INIT released, START, try each row separately, release, then STOP. Stop on a mismatch.

**Expect this:** every pattern matches. Release and STOP command zero, though wheels may coast.
This does not establish chassis stopping distance.

**If it fails:** STOP, return to isolated bring-up, and correct only the wrong fact.

**Advance when:** every raised-wheel pattern matches twice and the STOP plan was rehearsed.

## 5. Make the first controlled floor drive

Only after the stand passes, move to clear floor, keep the `0.25`/`0.20` limits, clear the stopping
area, and cover STOP.

**Run this:** without combining axes, make one brief forward command, stop, one brief right strafe,
stop, and one brief clockwise turn, then press STOP.

**Expect this:** the chassis moves slowly in the three requested directions and stops accepting
commands at STOP.

**If it fails:** press STOP immediately. Return to the earliest invalid fact—wheel direction,
wheel placement, configuration name, or mecanum layout—before another floor test.

**Advance when:** the three bounded motions and the observed stopping distance are repeatable in
the clear test area. Raise the two scale constants only in small reviewed increments after the team
has evidence for controllability and stopping distance at the new limit.

## 6. Grow beyond one file

This file is a complete first drive, not the final shape for mechanisms, shared TeleOp/Auto
behavior, profiles, tests, and experiments. Continue with
[Build a robot step by step](<Build a Robot Step by Step.md>). It introduces one owner at a time
while this proven drive path stays functional. Use
[Modern starter robot](<../examples/Modern Starter Robot.md>) when the course asks for a complete
copyable ownership slice.
