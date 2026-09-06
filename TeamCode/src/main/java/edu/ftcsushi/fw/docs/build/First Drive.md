---
tags:
  - Build
---

# Drive slowly with one gamepad

**Outcome:** trace current gamepad values through one controls owner into a complete mecanum drive
configuration, including robot-frame signs, capped commands, and managed stop.

**Knowledge before this page:** the changing-value reader in [read a switch](<Read a Switch.md>)
or the [short tour](<../getting-started/First Software Tour.md>). The drive APIs are explained here.
No installation, gamepad, test run, or drivetrain is needed to read this independent fixture.

**One idea:** drive continuously samples current intent. It does not turn each held stick into a
button event. Optional software execution and [authoring](<README.md#author-in-your-robot>) use the
same maintained path; physical wheel checks are separate.

## First pass: current values every loop

If you have written an iterative FTC `OpMode`, think of `configure(...)` as the one-time place where
you connect the parts that later loops will use. This exact production excerpt shows the class
declaration and complete `configure(...)` method; its helper definitions continue in the full build:

`extends FtcRobotOpMode` reuses Sushi's managed FTC loop. `@Override` identifies the setup method
you supply. `@TeleOp` names the OpMode; `@Disabled` keeps it unselectable until hardware review.

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

Follow the data from the FTC object through the input adapter, controls-owned meaning, drive
configuration, and managed program. The expected software observations below make this a complete
reading lesson. The wheels-up gate is only for a later physical run; compiling does not authorize
it.

## Critical production idea

### 1. Adapt the FTC gamepad during INIT

The composition root constructs the FTC adapter, constructs the controls owner, creates one complete
drive configuration, and declares one source-to-sink path:

Here the **composition root** means setup code that connects objects. An **adapter** translates
FTC gamepad fields into Sushi readers. A **source** supplies the current desired drive value; a
**sink** accepts that value and owns the final motor writes. These are different jobs, not extra
loops. `new` constructs an object now; `program.drive(...)` saves the connection for later updates.

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

The adapter then exposes live `ScalarSource` axes: **scalar** means one number, here a stick reading.
It flips the FTC Y convention so stick up is
positive, corrects the construction-time center, and rescales the remaining travel. Its button
sources report the current held level; edge behavior belongs to a binding or another derived source,
not to `GamepadDevice`. Controls choose the active device deadband and driver shaping next.

### 2. Give the three axes robot-frame meanings

Controls own what operator inputs mean. A **robot frame** describes directions relative to the
robot: forward follows its front, even after the robot turns. This small owner retains one stable `DriveSource`; it does
not set motors and does not rebuild a source every loop:

!!! info "New concept: Stick shaping"

    A **deadband** is a small centered stick range that produces zero, filtering drift and noise.
    An **exponent** above `1.0` makes partial stick motion gentler near center while preserving full
    stick at `1.0`. These are driver-feel choices, not drivetrain safety limits.

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/firstdrive/FirstDriveTeleOp.java -->
```java hl_lines="3 5 6 7 8 9 10"
FirstDriveControls(GamepadDevice driver) {
    GamepadDevice requiredDriver = Objects.requireNonNull(driver, "driver");
    requiredDriver.setAxisDeadband(0.02);

    GamepadDriveSource.Config driveSourceConfig = GamepadDriveSource.Config.defaults();
    driveSourceConfig.deadband = 0.05;
    driveSourceConfig.translateExpo = 1.5;
    driveSourceConfig.rotateExpo = 1.5;
    driveSourceConfig.translateScale = 1.0;
    driveSourceConfig.rotateScale = 1.0;
```

The highlighted assignments are the driver-feel edit points: `0.02` is the device deadband;
`0.05` is the shaping deadband; both translation and rotation use exponent `1.5`; and their `1.0`
scales retain full-range intent.

The local config draft is handed immediately to `GamepadDriveSource`, which validates and copies
the values it retains. That defensive snapshot prevents later edits to the draft from secretly
reconfiguring the running source:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/firstdrive/FirstDriveTeleOp.java -->
```java
    driveSource = new GamepadDriveSource(
            requiredDriver.leftX(),
            requiredDriver.leftY(),
            requiredDriver.rightX(),
            driveSourceConfig);
}
```

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/firstdrive/FirstDriveTeleOp.java -->
```java
/** Returns the stable drive source sampled by the managed program. */
DriveSource driveSource() {
    return driveSource;
}
```

`GamepadDriveSource` receives raw lateral, axial, and turn axes in that order. **Axial** means
forward/back, **lateral** means sideways, and **omega** is the turn component. A `DriveSignal` holds
those three normalized requests: zero means none and `1.0` is the positive full-scale request,
not a measured speed. It uses Sushi's robot-centric convention:

| Driver action | After `GamepadDevice` | Published component |
|---|---:|---:|
| Push left stick up | `leftY = +1` | `axial = +1` (robot forward) |
| Push left stick left | `leftX = -1` | `lateral = +1` (robot left) |
| Push right stick left | `rightX = -1` | `omega = +1` (counter-clockwise) |

The two X signs change because the FTC stick convention is positive right/clockwise while the Sushi
robot frame is positive left/counter-clockwise. This is robot-relative driving: “forward” follows
the robot's current heading, not the field.

Each stage has one job, and all active values are visible in the lesson:

| Stage | Exact values | Effect |
| --- | --- | --- |
| Controller correction | Construction-time center calibration; device deadband `0.02` | Corrects resting-center error and reports smaller corrected noise as zero. |
| Driver shaping | Deadband `0.05`; translation exponent `1.5`; rotation exponent `1.5`; translation scale `1.0`; rotation scale `1.0` | Removes a wider driver-command center region and softens partial stick motion without reducing full-scale intent. |
| Drive caps | `maxAxial = 0.25`; `maxLateral = 0.25`; `maxOmega = 0.20` | Limits the command presented to the mecanum wheel mixer for the first hardware run. |

`GamepadDriveSource` defensively snapshots the controls-owned draft during construction. Controls
then retain only the stable source, with no mutable tuning draft left to edit. Shaping answers “how
should the stick feel?”; the retained drive caps answer “how much command may this first run reach?”

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

**Mecanum mixing** calculates four wheel commands from the requested forward, sideways, and turn
components. `BRAKE` resists rotation at zero command; `FLOAT` allows coasting. Neither guarantees
a stopping distance, and neither turns the normalized request into a physical speed measurement.

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

**Expected observations:** neutral sticks produce four zero requests. With the authored full-scale
isolated stick inputs, forward and sideways components are capped at `0.25`, and turn at `0.20`.
Releasing a stick changes the next sample; STOP submits zero to every motor. Read the assertions
below against the sign and shaping tables above; running them is optional.

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

### Optional: inspect the detailed software checks

The expected observations above complete the reading experiment. These supplied checks additionally
inspect shaping, each wheel command, and repeated STOP. `assertSignal(...)` compares forward,
sideways, and turn in that order; `assertWheelPowers(...)` compares front-left, front-right,
back-left, and back-right. An assertion fails when an actual value differs from the expected one.

The first test constructs the exact production controls used by the OpMode. It starts with a neutral
gamepad, which matters because the real adapter calibrates in its constructor:

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

A half-stick sample distinguishes the active `0.05` deadband and `1.5` exponent from an identity
shape before the later full-scale samples prove robot-frame signs. The `f` suffix in `-0.5f` is
Java's spelling for a `float`, the number type used by FTC gamepad axes:

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/firstdrive/FirstDriveSoftwareScenarioTest.java -->
```java
// REQUEST + ASSERT: half stick is softened by the retained 0.05 deadband/1.5 exponent.
gamepad.left_stick_y = -0.5f;
double shapedHalf = shaped(0.5, 0.05, 1.5);
assertSignal(drive.get(time.nextCycle(0.02)), shapedHalf, 0.0, 0.0);
```

Each full-scale sample changes only one raw axis, so its sign has one visible cause:

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

Full-scale inputs prove the signs because shaping preserves `1.0`; the separate half-stick assertion
proves that the test did not replace production shaping with an identity configuration.

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

The host scenario sends the same half-stick request through the production source and `0.25` axial
cap before checking the full-scale coordinate signs:

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/firstdrive/FirstDriveSoftwareScenarioTest.java -->
```java
// REQUEST + HEARTBEAT + ASSERT: production half-stick shaping reaches the capped mixer.
gamepad.left_stick_y = -0.5f;
mode.loop();
double shapedHalfPower = 0.25 * shaped(0.5, 0.05, 1.5);
assertWheelPowers(
        hardware, config,
        shapedHalfPower, shapedHalfPower, shapedHalfPower, shapedHalfPower);
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

Optionally run the maintained scenario after [software setup](<../getting-started/Build and Run.md>):

=== "Windows"

    ```powershell
    .\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.firstdrive.FirstDriveSoftwareScenarioTest
    ```

=== "macOS"

    ```bash
    ./gradlew --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.firstdrive.FirstDriveSoftwareScenarioTest
    ```

### What the observations establish

**Read the causal chain:** the software gamepad changes; the production adapter and controls publish
one robot-centric request; the managed program samples it; the real mecanum mixer applies the
production caps; FTC boundary doubles record four commands; managed STOP records four zeros and
terminalizes the host.

**Proves:** neutral construction, the selected axis signs, `0.25` axial/lateral and `0.20` omega
component caps, the `0.05`/`1.5` shaping at half stick, the expected isolated-axis mecanum mixes,
four submitted zero commands on STOP, and idempotent managed termination all agree in the maintained
production graph.

**Does not prove:** a configured physical motor is attached to the named wheel, any direction entry
produces the expected physical rotation, the robot translates or turns correctly, or BRAKE stops it
within a safe distance.

**Reading checkpoint:** explain when the source reads the sticks, why a held stick continues to
drive, and why the configured axis sign does not establish a physical wheel's direction.

## Isolated hardware gate

This separate procedure applies only if you choose to operate the drivetrain.

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

**Next gate:** if the named-intake path is familiar, [combine drive and intake](<Combine Drive and Intake.md>)
in one TeleOp. Otherwise read [the intake lesson](<Continuous Intake.md>) first. These are knowledge
prerequisites; the separate hardware gates apply when you operate the combined robot.
