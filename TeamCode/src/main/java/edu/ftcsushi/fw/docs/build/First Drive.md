---
tags:
  - Build
---

# Drive slowly with one gamepad

**Outcome:** wire one FTC gamepad through production controls into a complete mecanum drive
configuration, then prove robot-frame signs, capped motor commands, and managed stop in software
before any wheel touches the floor.

**Prerequisites:** [Set up and verify](<../getting-started/Build and Run.md>) succeeds. You do not need
to know Sushi sources, bindings, or drive APIs yet. No robot is required until the isolated hardware
gate below.

## First pass: current values every loop

If you have written an iterative FTC `OpMode`, think of `configure(...)` as the one-time place where
you connect the parts that later loops will use. This exact production excerpt shows the class
declaration and complete `configure(...)` method; its helper definitions continue in the full build:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/firstdrive/FirstDriveTeleOp.java -->
```java
@TeleOp(name = "FW First Drive", group = "FW Examples")
@Disabled
public final class FirstDriveTeleOp extends FtcRobotOpMode {

    @Override
    protected void configure(RobotProgram program) {
        FirstDriveControls controls = new FirstDriveControls(
                new GamepadDevice(gamepad1));
        FtcDrives.MecanumConfig drive = firstRunDriveConfig();

        program.drive(controls.driveSource(), FtcDrives.mecanum(hardwareMap, drive));
    }
```

Sushi calls `configure(...)` once during the first INIT. Read the body as: make one reusable gamepad
reader, make one complete example drive configuration, and connect that reader to the drivetrain. The
`program.drive(...)` line saves the connection; it does not capture one stick position or perform
one drive event while configuration is running.

At START and on every active FTC loop, the saved reader supplies the **current** forward, sideways,
and turn values. Centered sticks produce a current zero request. Holding a stick keeps producing its
current held value; moving or releasing it changes the next loop's value. Drive values are therefore
sampled continuously, not treated as button press/release events.

The `FtcRobotOpMode` base class owns those later loops and also sends zero during STOP cleanup. Keep
`@Disabled` in place until the full build below has reconstructed and reviewed all four hardware
names, directions, the BRAKE/FLOAT choice, and the cautious output caps. Software can prove which
commands were submitted, not how real wheels will turn or stop.

## Full build: reconstruct the production path

**Start here:** this is the complete first-drive lesson. Follow the data from the FTC object through
the input adapter, controls-owned meaning, drive configuration, managed program, software proof,
and finally the wheels-up gate. Do not enable the example just because it compiles.

## Critical production idea

### 1. Adapt the FTC gamepad during INIT

The composition root constructs the FTC adapter, constructs the controls owner, creates one complete
drive configuration, and declares one source-to-sink path:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/firstdrive/FirstDriveTeleOp.java -->
```java
@Override
protected void configure(RobotProgram program) {
    FirstDriveControls controls = new FirstDriveControls(
            new GamepadDevice(gamepad1));
    FtcDrives.MecanumConfig drive = firstRunDriveConfig();

    program.drive(controls.driveSource(), FtcDrives.mecanum(hardwareMap, drive));
}
```

[`GamepadDevice`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/input/GamepadDevice.html>)
is the FTC boundary. Its constructor immediately calibrates all sticks and triggers, treating their
current readings as neutral. Leave every stick centered and every trigger released before pressing
INIT. If an axis is held during construction, that held reading becomes its zero until recalibrated.

The adapter then exposes live `ScalarSource` axes. It flips the FTC Y convention so stick up is
positive, corrects the construction-time center, rescales the remaining travel, and applies its
small device deadband. Its button sources report the current held level; edge behavior belongs to a
binding or another derived source, not to `GamepadDevice`.

### 2. Give the three axes robot-frame meanings

Controls own what operator inputs mean. This small owner retains one stable `DriveSource`; it does
not set motors and does not rebuild a source every loop:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/firstdrive/FirstDriveTeleOp.java -->
```java
static final class FirstDriveControls {
    private final DriveSource driveSource;

    /** Maps the selected driver axes into Sushi's robot-centric drive convention. */
    FirstDriveControls(GamepadDevice driver) {
        GamepadDevice requiredDriver = Objects.requireNonNull(driver, "driver");
        driveSource = new GamepadDriveSource(
                requiredDriver.leftX(),
                requiredDriver.leftY(),
                requiredDriver.rightX(),
                GamepadDriveSource.Config.defaults());
    }
```

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/firstdrive/FirstDriveTeleOp.java -->
```java
/** Returns the stable drive source sampled by the managed program. */
DriveSource driveSource() {
    return driveSource;
}
```

`GamepadDriveSource` receives raw lateral, axial, and turn axes in that order. It publishes Sushi's
robot-centric `DriveSignal` convention:

| Driver action | After `GamepadDevice` | Published component |
|---|---:|---:|
| Push left stick up | `leftY = +1` | `axial = +1` (robot forward) |
| Push left stick left | `leftX = -1` | `lateral = +1` (robot left) |
| Push right stick left | `rightX = -1` | `omega = +1` (counter-clockwise) |

The two X signs change because the FTC stick convention is positive right/clockwise while the Sushi
robot frame is positive left/counter-clockwise. This is robot-relative driving: “forward” follows
the robot's current heading, not the field.

The two configuration layers solve different problems:

- `GamepadDevice` corrects controller center error and uses a `0.02` device deadband.
- `GamepadDriveSource.Config.defaults()` shapes driver intent with a `0.05` deadband and `1.5`
  translation/rotation exponents. The exponent softens motion near center while preserving the
  full-scale endpoints; its translation and rotation scales remain `1.0` here.
- The drivebase caps below limit the command presented to the wheel mixer. Keep these cautious
  physical-output caps even though a source can also scale driver intent.

In short, shaping answers “how should the stick feel?”; the drive caps answer “how much command may
this first hardware run reach?”

### 3. Review one complete mecanum configuration

The factory starts from a fresh default value, but this lesson restates every robot-specific wiring
choice instead of asking a beginner to trust hidden defaults:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/firstdrive/FirstDriveTeleOp.java -->
```java
static FtcDrives.MecanumConfig firstRunDriveConfig() {
    FtcDrives.MecanumConfig drive = FtcDrives.MecanumConfig.defaults();
    drive.wiring.frontLeftName = "frontLeftMotor";
    drive.wiring.frontRightName = "frontRightMotor";
    drive.wiring.backLeftName = "backLeftMotor";
    drive.wiring.backRightName = "backRightMotor";
    drive.wiring.frontLeftDirection = Direction.FORWARD;
    drive.wiring.frontRightDirection = Direction.REVERSE;
    drive.wiring.backLeftDirection = Direction.FORWARD;
    drive.wiring.backRightDirection = Direction.REVERSE;
```

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/firstdrive/FirstDriveTeleOp.java -->
```java
    drive.enableZeroPowerBrake = true; // Review BRAKE versus FLOAT for this drivetrain.
    drive.drivebase.maxAxial = 0.25;
    drive.drivebase.maxLateral = 0.25;
    drive.drivebase.maxOmega = 0.20;
    return drive;
}
```

Replace all four names with the exact Robot Configuration names. Treat all four directions as
hypotheses until the wheels-up checks agree; `FORWARD` and `REVERSE` describe logical-to-electrical
polarity, not what an unobserved wheel must physically do. Decide whether zero power should use
`BRAKE` or `FLOAT` for this drivetrain. The three normalized caps independently scale axial,
lateral, and turn components before mecanum mixing; simultaneous components are still normalized if
their wheel mix would exceed the allowed motor range.

`FtcDrives.mecanum(hardwareMap, drive)` validates and snapshots the configuration, resolves all four
motors, and configures direction and zero-power behavior. Construction itself does not command
power. The managed drive phase performs the coordinated raw-power preflight and wheel writes.

### 4. Declare a continuously sampled drive path

`program.drive(source, sink)` is a persistent declaration, not a button event. On START realization
and every active loop, `RobotProgram` reaches the output/drive phase, advances the sink heartbeat,
samples the final `DriveSource`, rejects non-finite components, clamps the `DriveSignal`, and gives
the sink one final command. `FtcRobotOpMode` owns that heartbeat and phase order.

```text
FTC gamepad -> GamepadDevice -> FirstDriveControls -> DriveSource -> DriveSignal
             -> program.drive(source, sink) -> MecanumDrivebase -> four motor commands

FTC STOP or caught lifecycle failure -> RobotProgram cleanup -> sink.stop() -> four zero commands
```

Drive sticks already represent a complete value every cycle. They go directly through
`program.drive`; do not turn them into press/release events or copy them through a second loop.
Button transitions and work that continues over time are separate ideas introduced by
[Continuous Intake](<Continuous Intake.md>) and [Run One Timed Auto](<Run One Timed Auto.md>).

On STOP, the managed host terminalizes, performs its one cleanup pass, and the drive sink submits
zero to all four motors. Repeated STOP is inert. This is strong software evidence about submitted
commands, but only observation can establish that a real robot actually stopped.

Notice:

- FTC details end at `GamepadDevice` and `FtcDrives`; controls own operator meaning, while the
  drivebase owns mixing and final writes.
- Input shaping and source scaling tune the request; the retained first-run drive caps constrain
  realization and should not be removed merely because the sticks feel slow.
- A recorded motor command proves what software submitted, not wheel direction, traction, current,
  braking distance, or a safe mechanism envelope.

## Files in this checkpoint

**Main:**

- [`FirstDriveTeleOp`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/firstdrive/FirstDriveTeleOp.html>) — generated API reference.
- [Complete source: `FirstDriveTeleOp.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/firstdrive/FirstDriveTeleOp.java>)

**Test:**

- [Complete source: `FirstDriveSoftwareScenarioTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/firstdrive/FirstDriveSoftwareScenarioTest.java>)

## Software checkpoint: sticks have one coordinate meaning

- **Question:** Do the maintained controls and complete drive configuration produce Sushi's three
  positive robot-frame directions, the intended capped four-wheel commands, and zero commands on
  managed STOP?
- **Keep real:** `FirstDriveControls`, `firstRunDriveConfig()`, `GamepadDevice`,
  `GamepadDriveSource`, `RobotProgram`, `FtcRobotOpMode`, `FtcDrives`, and `MecanumDrivebase`.
- **Replace:** the physical controller, Control Hub motors, and Driver Station telemetry with an FTC
  SDK `Gamepad`, recording `DcMotorEx` boundary doubles in `FtcTestHardware`, and supplied silent
  telemetry. The production `FirstDriveTeleOp` host remains real.
- **Observe:** the production `DriveSignal` signs and the last command submitted to each named motor
  before and after STOP.
- **Cannot conclude:** physical wheel direction, robot motion, traction, current draw, braking
  distance, or whether the surrounding mechanism envelope is safe.

The first test constructs the same nested production controls used by the OpMode. It starts with a
neutral gamepad, which matters because the real adapter calibrates in its constructor:

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/firstdrive/FirstDriveSoftwareScenarioTest.java -->
```java
// ARRANGE: construct the exact production controls from a centered software gamepad.
Gamepad gamepad = new Gamepad();
FirstDriveTeleOp.FirstDriveControls controls =
        new FirstDriveTeleOp.FirstDriveControls(new GamepadDevice(gamepad));
DriveSource drive = controls.driveSource();
ManualLoopClock time = new ManualLoopClock();

// ASSERT: the construction-time neutral reading produces no drive request.
assertSignal(drive.get(time.clock()), 0.0, 0.0, 0.0);
```

Each later sample changes only one raw axis, so its sign has one visible cause:

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/firstdrive/FirstDriveSoftwareScenarioTest.java -->
```java
// REQUEST + ASSERT: FTC stick up becomes positive robot-forward axial intent.
gamepad.left_stick_y = -1.0f;
assertSignal(drive.get(time.nextCycle(0.02)), 1.0, 0.0, 0.0);

// REQUEST + ASSERT: FTC stick left becomes positive robot-left lateral intent.
gamepad.left_stick_y = 0.0f;
gamepad.left_stick_x = -1.0f;
assertSignal(drive.get(time.nextCycle(0.02)), 0.0, 1.0, 0.0);
```

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/firstdrive/FirstDriveSoftwareScenarioTest.java -->
```java
// REQUEST + ASSERT: FTC stick left-turn becomes positive counter-clockwise intent.
gamepad.left_stick_x = 0.0f;
gamepad.right_stick_x = -1.0f;
assertSignal(drive.get(time.nextCycle(0.02)), 0.0, 0.0, 1.0);
// NEXT GATE: motor commands and terminal stop still need the managed-host scenario.
```

Full-scale inputs are intentional: they prove the signs while preserving the production `0.05`
deadband and `1.5` exponent, whose full-scale result remains exactly `1.0`. The test does not replace
production shaping with a test-only identity configuration.

The second test instantiates the production configuration, the real FTC drive factory, and the
managed OpMode host. Supplied recording hardware and silent-telemetry helpers hide boundary proxy
plumbing from the teaching scenario:

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/firstdrive/FirstDriveSoftwareScenarioTest.java -->
```java
// ARRANGE: keep the complete production config/OpMode; replace FTC devices at the boundary.
FtcDrives.MecanumConfig config = FirstDriveTeleOp.firstRunDriveConfig();
FtcTestHardware hardware = hardwareFor(config);
Gamepad gamepad = new Gamepad();
FirstDriveTeleOp mode = configuredMode(hardware, gamepad);

// START: let the managed host own initialization, heartbeat, and cleanup.
mode.init();
mode.start();
```

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/firstdrive/FirstDriveSoftwareScenarioTest.java -->
```java
// REQUEST + HEARTBEAT + ASSERT: each isolated axis reaches the capped wheel mixer.
gamepad.left_stick_y = -1.0f;
mode.loop();
assertWheelPowers(hardware, config, 0.25, 0.25, 0.25, 0.25);

gamepad.left_stick_y = 0.0f;
gamepad.left_stick_x = -1.0f;
mode.loop();
assertWheelPowers(hardware, config, -0.25, 0.25, 0.25, -0.25);
```

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/firstdrive/FirstDriveSoftwareScenarioTest.java -->
```java
gamepad.left_stick_x = 0.0f;
gamepad.right_stick_x = -1.0f;
mode.loop();
assertWheelPowers(hardware, config, -0.20, 0.20, -0.20, 0.20);
```

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/firstdrive/FirstDriveSoftwareScenarioTest.java -->
```java
// STOP + ASSERT: one terminal cleanup writes four zeros and repeated stop remains inert.
mode.stop();
assertWheelPowers(hardware, config, 0.0, 0.0, 0.0, 0.0);

int writesAfterStop = hardware.totalMotorPowerWrites();
mode.stop();
mode.loop();
assertEquals(writesAfterStop, hardware.totalMotorPowerWrites());
// NEXT GATE: only a supported wheels-up run can prove physical directions and braking.
```

Run:

```powershell
.\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.firstdrive.FirstDriveSoftwareScenarioTest
```

**Read the causal chain:** the software gamepad changes; the production adapter and controls publish
one robot-centric request; the managed program samples it; the real mecanum mixer applies the
production caps; FTC boundary doubles record four commands; managed STOP records four zeros and
terminalizes the host.

**Proves:** neutral construction, the selected axis signs, `0.25` axial/lateral and `0.20` omega
component caps, the expected isolated-axis mecanum mixes, four submitted zero commands on STOP, and
idempotent managed termination all agree in the maintained production graph.

**Does not prove:** a configured physical motor is attached to the named wheel, any direction entry
produces the expected physical rotation, the robot translates or turns correctly, or BRAKE stops it
within a safe distance.

## Isolated hardware gate

Keep `FirstDriveTeleOp` disabled while completing this review:

1. Match all four names to the Robot Configuration. Inspect each motor and connector rather than
   inferring identity from code.
2. Choose `BRAKE` or `FLOAT` deliberately, leave the `0.25`, `0.25`, and `0.20` caps in place, and
   confirm the wheels and every nearby mechanism can move without contact.
3. Put the robot securely on blocks with wheels clear of people and objects. Use a charged battery,
   clear the work area, and assign one person solely to press STOP.
4. Before INIT, center both sticks and release both triggers. Keep them neutral through INIT so
   construction-time calibration records the real resting values.
5. Only now remove `@Disabled`, rebuild, select **FW First Drive**, press INIT, and watch for
   unexpected motion or configuration errors before START.
6. After START, pulse one small command at a time: left-stick up should drive all wheels forward;
   left-stick left should strafe robot-left; right-stick left should turn counter-clockwise. Press
   STOP immediately if any one wheel disagrees, then change configuration rather than compensating
   in controls.
7. Release the sticks, press STOP, and observe that all four real wheels cease being driven. Treat
   coast, braking distance, vibration, heat, or abnormal current as new physical evidence that the
   software test could not supply.

Keep the robot on blocks until all three isolated directions and STOP agree with the written
expectations. Lower it only for a cleared, low-speed floor trial with the same caps and stop
operator.

**Next gate:** drive is independent of the actuator lessons. When you need an actuator, start the
cumulative path with the [continuous intake](<Continuous Intake.md>) before connecting that
mechanism to the robot.
