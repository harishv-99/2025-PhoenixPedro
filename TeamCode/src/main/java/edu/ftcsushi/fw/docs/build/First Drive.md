---
tags:
  - Build
---

# Drive slowly with one gamepad

**Outcome:** turn three gamepad axes into one robot-centric drive request while keeping the first
physical run deliberately slow.

**Prerequisites:** [Build and run](<../getting-started/Build and Run.md>) succeeds. No robot or
matching drivetrain is required for the software checkpoint; physical prerequisites begin only at
the isolated hardware gate below.

## Critical production idea

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/firstdrive/FirstDriveTeleOp.java -->
```java
drive.enableZeroPowerBrake = true; // Review BRAKE versus FLOAT for this drivetrain.
drive.drivebase.maxAxial = 0.25;
drive.drivebase.maxLateral = 0.25;
drive.drivebase.maxOmega = 0.20;

program.drive(controls.driveSource(), FtcDrives.mecanum(hardwareMap, drive));
```

Notice:

- [`GamepadDriveSource`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/drive/source/GamepadDriveSource.html>)
  owns stick shaping and robot-frame signs; the drive configuration owns the physical output caps.
- [`RobotProgram.drive`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/RobotProgram.html>)
  connects one intent source to one final drivetrain sink and updates it on the managed heartbeat.
- The `0.25` and `0.20` values are cautious first-run limits, not tuned season values.

## Files in this checkpoint

**Main:**

- [`FirstDriveTeleOp`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/firstdrive/FirstDriveTeleOp.html>) — API reference.
- [Complete source: `FirstDriveTeleOp.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/firstdrive/FirstDriveTeleOp.java>)

**Test:**

- [Complete source: `FirstDriveSoftwareScenarioTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/firstdrive/FirstDriveSoftwareScenarioTest.java>)

## Software checkpoint: sticks have one coordinate meaning

- **Question:** Do the three chosen stick axes become Sushi's forward, left, and counter-clockwise
  drive components?
- **Keep real:** `GamepadDevice`, `GamepadDriveSource`, and the shared loop clock.
- **Replace:** only the driver's physical gamepad with the FTC SDK's software object.
- **Observe:** the one `DriveSignal` returned on the next sample.
- **Cannot conclude:** motor wiring, wheel directions, traction, current draw, or safe motion.

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/firstdrive/FirstDriveSoftwareScenarioTest.java -->
```java
// REQUEST: author the FTC stick readings that a driver would physically supply.
gamepad.left_stick_y = -0.50f; // FTC up is negative; GamepadDevice makes up positive.
gamepad.left_stick_x = 0.25f;  // FTC right becomes Sushi's negative-left command.
gamepad.right_stick_x = -0.40f; // FTC left becomes Sushi's positive-CCW command.

// HEARTBEAT + ASSERT: one normal sample exposes the robot-centric command.
DriveSignal signal = drive.get(time.nextCycle(0.02));
assertEquals(0.50, signal.axial, 1e-6);
assertEquals(-0.25, signal.lateral, 1e-6);
assertEquals(0.40, signal.omega, 1e-6);
```

Run:

```powershell
.\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.firstdrive.FirstDriveSoftwareScenarioTest
```

**Read the causal chain:** the test authors controller readings; the normal source sample performs
the same calibration, shaping, and sign conversion used by the OpMode; the assertions inspect the
resulting robot-centric intent.

**Proves:** the selected axes and Sushi coordinate signs are connected as intended.

**Does not prove:** any physical wheel turns in the correct direction or stops safely.

## Isolated hardware gate

Keep `FirstDriveTeleOp` disabled while reviewing configuration. Put the robot on blocks, clear the
mechanism envelope, verify each motor name/direction, retain the low caps, and appoint one person to
stop the OpMode. Only then enable it for separate forward, left, and counter-clockwise checks. Lower
the robot only after those wheels-up observations agree with the written expectations.

**Next gate:** drive is independent of the actuator lessons. When you need an actuator, start the
cumulative path with the [continuous intake](<Continuous Intake.md>) before connecting that
mechanism to the robot.
