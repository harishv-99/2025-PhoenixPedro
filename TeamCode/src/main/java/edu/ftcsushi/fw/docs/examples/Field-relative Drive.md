---
tags:
  - Advanced
---

# Field-relative Drive

**Learning mode:** Architecture reference

**Complete source:** [`FieldRelativeDriveExample.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/fieldrelative/opmode/FieldRelativeDriveExample.java>)

Study this after [driving with a gamepad](<../build/First Drive.md>) and
[combining drive and intake](<../build/Combine Drive and Intake.md>).
The example keeps the ordinary managed lifecycle: the OpMode only configures a `RobotProgram`, the
heading estimator is an upstream service, and the final drivetrain still consumes a robot-centric
`DriveSignal`.

**Architecture-reference promise:** this page reconstructs the field-relative owner graph and
managed lifecycle after the robot-relative Starter. It is not a copy-ready hardware profile: use
the linked complete field-relative source for the active motor names, BRAKE/FLOAT choice, IMU
hardware name and Hub orientation, and manual-drive shaping values, then review every one on the
adopting robot.

## What “up” means

A **coordinate frame** gives numbers an origin and directions. Robot-relative driving uses the
robot's forward and left directions; field-relative driving keeps the chosen field direction fixed
even when the robot turns. **Heading** is the direction the robot faces. The IMU measures rotation;
an upstream heading owner aligns that measurement to the field direction authored at START.
Angles use radians: `Math.toRadians(90.0)` is a quarter-turn, or `π/2` radians.

![The same stick-up field direction becomes robot-forward at zero heading and robot-right after a counter-clockwise quarter-turn.](<../assets/diagrams/field-relative-frames.svg>)

In this illustrative top-down view, the driver selected field `+X` as up. At heading `0°`, stick up
requests robot-forward motion. After the robot turns counter-clockwise to `90°`, the same stick input
requests robot-right motion (`-Y` in the robot frame), still toward field `+X`. The numbers are
directions, not a claim that a motor command produces an exact physical movement.

Stick up means the finite `controlUpFieldHeadingRad` authored for the named driver station selected
during INIT. It is not inferred from the robot's placement and it is not calculated by negating or
rotating another alliance. Each station separately authors:

- `initialRobotFieldHeadingRad`: how the robot is physically facing at START;
- `controlUpFieldHeadingRad`: the field direction the driver calls up.

Those angles may be equal, opposite, orthogonal, or unrelated. This works for square, inverted, and
diamond field layouts without embedding season geometry in the reusable source.

The checked-in station table is deliberately a neutral cardinal-direction practice table. It is
not BIOBUZZ field geometry. Replace it with reviewed official station facts when they are available,
and keep `allowDriveMotion` false until motor wiring, Hub orientation, headings, low-power motion,
and physical STOP have all been checked.

## Why this example uses Prestart and a Service

`RobotProgram.Prestart` owns data-only choices made during INIT. Here it lets the operator select a
named practice station, presents that selection, and freezes the station exactly once at START. It
does not own drivetrain hardware, move the robot, or need a stop hook.

`RobotProgram.Service` owns a stable resource or upstream process that must advance before bindings,
Tasks, and outputs during the active loop. Here the IMU heading estimator starts from the frozen
initial heading, updates once per managed cycle, and stops during program cleanup. Controls only
read its cached heading evidence.

These are distinct lifecycle jobs, not decoration required by every robot. Robot-centric mecanum
needs neither role. This focused example adds them because field-relative translation needs one
INIT-only station decision and one actively updated heading owner.

## Complete managed owner wiring

The profile starts from the ordinary mecanum, IMU, and manual-drive configuration factories. The
excerpt below shows its direction and cap overrides; it deliberately does not replace the complete
profile as the authority for the other active hardware and shaping values:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/fieldrelative/robot/FieldRelativeExampleProfile.java -->
```java
profile.drive = FtcDrives.MecanumConfig.defaults();
profile.drive.wiring.frontLeftDirection = Direction.FORWARD;
profile.drive.wiring.frontRightDirection = Direction.REVERSE;
profile.drive.wiring.backLeftDirection = Direction.FORWARD;
profile.drive.wiring.backRightDirection = Direction.REVERSE;
profile.drive.drivebase.maxAxial = 0.25;
profile.drive.drivebase.maxLateral = 0.25;
profile.drive.drivebase.maxOmega = 0.20;
```

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/fieldrelative/robot/FieldRelativeExampleProfile.java -->
```java
profile.imu = FtcImuHeadingEstimator.Config.defaults();
profile.manualDrive = GamepadDriveSource.Config.defaults();
profile.stations = Collections.unmodifiableList(Arrays.asList(
        new Station("PRACTICE_POS_X", "Practice +X up", 0.0, 0.0),
        new Station("PRACTICE_POS_Y", "Practice +Y up", Math.PI / 2.0, Math.PI / 2.0),
        new Station("PRACTICE_NEG_X", "Practice -X up", Math.PI, Math.PI),
        new Station("PRACTICE_NEG_Y", "Practice -Y up", -Math.PI / 2.0, -Math.PI / 2.0)
));
profile.allowDriveMotion = false;
```

The checked-in directions, caps, IMU defaults, shaping defaults, and station table are software
candidates, not robot facts. The composition root enforces the checked-in `false` before
constructing the IMU or drivetrain:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/fieldrelative/robot/FieldRelativeExampleRobot.java -->
```java
if (!activeProfile.allowDriveMotion) {
    throw new IllegalStateException(
            "FieldRelativeExampleProfile.allowDriveMotion must be true only after reviewing "
                    + "motor wiring, Hub orientation, station headings, low-power motion, and STOP."
    );
}
```

The maintained example leaves it false. An adopting robot changes it only after completing the
[`First Drive` hardware gate](<../build/First Drive.md#isolated-hardware-gate>) and separately
verifying Hub orientation plus the authored station headings.

The composition root then declares each owner in dependency order. First it registers and retains
the INIT selection and the upstream heading service:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/fieldrelative/robot/FieldRelativeExampleRobot.java -->
```java
FieldRelativeExamplePrestart prestart = program.prestart(
        new FieldRelativeExamplePrestart(activeProfile.stations, requiredGamepad));
FtcImuHeadingEstimator heading = program.service(new FtcImuHeadingEstimator(
        hardwareMap,
        activeProfile.imu,
        prestart::frozenInitialRobotFieldHeadingRad));
```

Next it builds the controls around stable gamepad sources and those retained owners, then declares
one final mecanum drive sink:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/fieldrelative/robot/FieldRelativeExampleRobot.java -->
```java
FieldRelativeExampleControls controls = new FieldRelativeExampleControls(
        new GamepadDevice(requiredGamepad),
        heading,
        prestart,
        activeProfile.manualDrive);
DriveSource drive = controls.drive();
program.drive(drive, FtcDrives.mecanum(hardwareMap, activeProfile.drive));
```

Finally it registers the presenter, which reads the selection and adds one explanation row after
drive output; `RobotProgram` commits the frame once:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/fieldrelative/robot/FieldRelativeExampleRobot.java -->
```java
program.presenter((clock, telemetry) -> {
    prestart.present(telemetry);
    telemetry.addLine("Field-relative translation uses the frozen station up direction.");
});
```

The OpMode itself only chooses this profile and composition root:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/fieldrelative/opmode/FieldRelativeDriveExample.java -->
```java
@Override
protected void configure(RobotProgram program) {
    new FieldRelativeExampleRobot(hardwareMap).declareTeleOp(
            program, FieldRelativeExampleProfile.current(), gamepad1);
}
```

At START, the selected station freezes, the one clock resets, and the heading service aligns the
IMU estimate. Each active cycle then runs `heading service -> field-relative drive source -> one
mecanum sink -> presenter`. STOP ends the drive and service through the same managed program.

## Heading backends

The example registers `FtcImuHeadingEstimator` as a service. At START it reads the selected
station's initial robot heading and aligns the current Hub yaw to the field frame. It never treats
magnetic north or the SDK's power-on zero as FTC field +X.

Full localization uses the same drive source directly:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/fieldrelative/robot/FieldRelativeExampleControls.java -->
```java
drive = new GamepadDriveSource(
        driver.leftX(),
        driver.leftY(),
        driver.rightX(),
        config
).fieldRelativeTo(heading, prestart::frozenControlUpFieldHeadingRad);
```

**What to notice**

- `fieldRelativeTo(...)` reshapes translation intent; the downstream signal remains robot-centric.
- The heading owner updates upstream, and reading its estimate does not trigger localization again.
- Control-up is a frozen station fact, separate from the robot's initial field heading.

**Key APIs**

- `HeadingEstimator`: exposes cached heading availability, quality, and timestamp evidence.
- `GamepadDriveSource`: maps stable input Sources into robot drive intent.
- `DriveSource.fieldRelativeTo(...)`: rotates accepted field/control translation into robot axes.
- `RobotProgram.drive(...)`: gives one sink final drive-write ownership.

`AbsolutePoseEstimator` projects its already-cached yaw, availability, quality, and timestamp into
`HeadingEstimate`. The drive source does not cause a second localization update.

## Loss behavior

A heading is **stale** when it is older than the allowed age. Its **quality** is the estimator's
score, not a guarantee of accuracy. If either fails the configured acceptance limit, the source
cannot safely preserve the promised field direction from that evidence. It therefore requests
zero translation instead of quietly changing what the sticks mean.

### Critical code

The focused loss tests sample the declared `DriveSource`, supply unavailable heading evidence, and
assert axial/lateral zero while preserving finite manual omega. The complete controls file below is
the production code; the assertion belongs in the focused test rather than in the robot class.

**What to notice**

- Missing, stale, low-quality, or non-finite heading disables translation instead of changing driver meaning.
- Manual omega is independent and remains available when finite.
- Runtime re-zero/restore is deliberately absent; that would be robot-owned policy.

**Key APIs**

- `HeadingEstimate`: carries the heading plus availability, quality, and timestamp truth.
- `DriveSignal`: keeps forward, left, and counter-clockwise omega components explicit.
- `LoopClock`: supplies the current reset epoch and age boundary used to judge evidence.

Fresh accepted heading evidence rotates control-frame translation into the robot frame. Missing,
non-finite, stale, or low-quality evidence disables translation while leaving finite manual omega
available. There is no silent switch to robot-relative translation.

The station-authored direction stays fixed for the match. This baseline deliberately omits runtime
re-zero and restore state: changing driver meaning mid-match is robot policy, not required
field-relative conversion.

## Software evidence and its cause

**Question:** are the chosen driver direction and the unavailable-heading response kept explicit?
**Keep real:** the prestart selection and field-relative source being tested. **Replace:** operator
input and the heading estimator's outside-world observations. **Observe:** frozen headings and the
resulting drive signal. **Cannot conclude:** correct installed IMU orientation or physical movement.

The prestart test arranges two stations with deliberately independent headings, one gamepad, the
real prestart owner, and one clock:

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/fieldrelative/robot/FieldRelativeExamplePrestartTest.java -->
```java
Gamepad gamepad = new Gamepad();
FieldRelativeExampleProfile.Station first =
        new FieldRelativeExampleProfile.Station("A", "A", 0.1, 0.2);
FieldRelativeExampleProfile.Station orthogonal =
        new FieldRelativeExampleProfile.Station("B", "B", -0.7, Math.PI / 2.0);
FieldRelativeExamplePrestart prestart = new FieldRelativeExamplePrestart(
        Arrays.asList(first, orthogonal),
        gamepad
);
LoopClock clock = new LoopClock();
clock.reset(0.0);
```

A D-pad request is sampled on the next INIT heartbeat; `freezeForStart()` then freezes the selected
value, and the assertions observe the authored `-0.7` initial heading and `pi/2` control-up heading:

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/fieldrelative/robot/FieldRelativeExamplePrestartTest.java -->
```java
prestart.update(clock); // edge baseline
gamepad.dpad_down = true;
clock.update(0.02);
prestart.update(clock);

assertEquals(RobotProgram.StartDisposition.READY, prestart.freezeForStart());
assertEquals(-0.7, prestart.frozenInitialRobotFieldHeadingRad(), 0.0);
assertEquals(Math.PI / 2.0, prestart.frozenControlUpFieldHeadingRad(), 0.0);
```

The loss test separately arranges a one-second clock, a half-second-old heading, forward stick
intent, and finite manual turn. Sampling the real `DriveSource` is the heartbeat for this focused
unit; the observation is zero translation with omega preserved:

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/fw/drive/source/GamepadDriveSourceFieldRelativeTest.java -->
```java
LoopClock clock = clockAt(1.0);
MutableHeading heading = new MutableHeading();
heading.estimate = new HeadingEstimate(
        0.0, true, 1.0, clock.timestampSecondsAgo(0.5));
GamepadDriveSource.Config config = GamepadDriveSource.Config.defaults();
config.deadband = 0.0;
DriveSource source = new GamepadDriveSource(
        ScalarSource.constant(0.0), ScalarSource.constant(1.0),
        ScalarSource.constant(0.4), config
).fieldRelativeTo(heading, () -> 0.0, 0.1, 0.0);
```

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/fw/drive/source/GamepadDriveSourceFieldRelativeTest.java -->
```java
DriveSignal signal = source.get(clock);

assertEquals(0.0, signal.axial, EPS);
assertEquals(0.0, signal.lateral, EPS);
assertEquals(-Math.pow(0.4, 1.5), signal.omega, EPS);
```

These software checks prove selection/freeze causality and fail-closed command shaping for authored
evidence. They do not prove Hub orientation, field headings, motor directions, or physical motion.

## Maintained files

The compiling example keeps the FTC host, profile, frozen prestart facts, controls owner, and robot
composition root separate. Read the short explanation above first, then use these authorities when
adapting the pattern:

- [`FieldRelativeDriveExample`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/fieldrelative/opmode/FieldRelativeDriveExample.html>)
- [Complete source: field-relative example](<https://github.com/harishv-99/2025-PhoenixPedro/tree/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/fieldrelative>)
- [Complete source: focused field-relative tests](<https://github.com/harishv-99/2025-PhoenixPedro/tree/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/fieldrelative>)
- [Complete source: `GamepadDriveSourceFieldRelativeTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/fw/drive/source/GamepadDriveSourceFieldRelativeTest.java>)

## Verify the slice

Optionally run the supplied software check after [software setup](<../getting-started/Build and Run.md>):

=== "Windows"

    ```powershell
    .\gradlew.bat --console=plain :TeamCode:compileDebugJavaWithJavac `
      :TeamCode:testDebugUnitTest --tests 'edu.ftcsushi.robots.examples.fieldrelative.*'
    ```

=== "macOS"

    ```bash
    ./gradlew --console=plain :TeamCode:compileDebugJavaWithJavac \
      :TeamCode:testDebugUnitTest --tests 'edu.ftcsushi.robots.examples.fieldrelative.*'
    ```

Expected checkpoint: compilation and the field-relative focused tests pass. Translation still
fails closed when heading evidence is unavailable; no software check proves Hub orientation or
physical drive direction.

[Back to examples](<README.md>)
