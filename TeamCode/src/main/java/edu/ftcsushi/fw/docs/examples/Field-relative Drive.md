# Field-relative Drive

**Source entry:** [`FieldRelativeDriveExample.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/fieldrelative/opmode/FieldRelativeDriveExample.java>)

Study this after the robot-relative [`StarterTeleOp`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/opmode/StarterTeleOp.java>).
The example keeps the ordinary managed lifecycle: the OpMode only configures a `RobotProgram`, the
heading estimator is an upstream service, and the final drivetrain still consumes a robot-centric
`DriveSignal`.

## What “up” means

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

## Heading backends

The example registers `FtcImuHeadingEstimator` as a service. At START it reads the selected
station's initial robot heading and aligns the current Hub yaw to the field frame. It never treats
magnetic north or the SDK's power-on zero as FTC field +X.

Full localization uses the same drive source directly:

Abbreviated shape (omissions shown):

<!-- teaching-shape -->
```java
HeadingEstimator heading = fusedAbsolutePoseEstimator;

DriveSource drive = new GamepadDriveSource(
        driver.leftX(),
        driver.leftY(),
        driver.rightX(),
        GamepadDriveSource.Config.defaults()
).fieldRelativeTo(heading, prestart::frozenControlUpFieldHeadingRad);
// ...declare this DriveSource with the robot's one final DriveCommandSink...
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

### Critical code

Abbreviated shape (omissions shown):

<!-- teaching-shape -->
```java
DriveSignal signal = drive.get(clock);
HeadingEstimate estimate = heading.getHeadingEstimate();
if (!estimate.hasHeading) {
    assertEquals(0.0, signal.axial, 0.0);
    assertEquals(0.0, signal.lateral, 0.0);
    // finite manual rotation remains available
    assertEquals(driverOmega, signal.omega, 0.0);
}
// ...there is no silent robot-relative translation fallback...
```

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

[Back to examples](<README.md>)
