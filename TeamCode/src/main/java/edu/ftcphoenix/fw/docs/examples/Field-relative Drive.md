# Field-relative Drive

**Source entry:** [`FieldRelativeDriveExample.java`](<../../../robots/examples/fieldrelative/opmode/FieldRelativeDriveExample.java>)

Study this after the robot-relative [`StarterTeleOp`](<../../../robots/examples/starter/opmode/StarterTeleOp.java>).
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

## Heading backends

The example registers `FtcImuHeadingEstimator` as a service. At START it reads the selected
station's initial robot heading and aligns the current Hub yaw to the field frame. It never treats
magnetic north or the SDK's power-on zero as FTC field +X.

Full localization uses the same drive source directly:

```java
HeadingEstimator heading = fusedAbsolutePoseEstimator;

DriveSource drive = new GamepadDriveSource(
        driver.leftX(),
        driver.leftY(),
        driver.rightX(),
        GamepadDriveSource.Config.defaults()
).fieldRelativeTo(heading, prestart::frozenControlUpFieldHeadingRad);
```

`AbsolutePoseEstimator` projects its already-cached yaw, availability, quality, and timestamp into
`HeadingEstimate`. The drive source does not cause a second localization update.

## Loss behavior

Fresh accepted heading evidence rotates control-frame translation into the robot frame. Missing,
non-finite, stale, or low-quality evidence disables translation while leaving finite manual omega
available. There is no silent switch to robot-relative translation.

The station-authored direction stays fixed for the match. This baseline deliberately omits runtime
re-zero and restore state: changing driver meaning mid-match is robot policy, not required
field-relative conversion.

[Back to examples](<README.md>)
