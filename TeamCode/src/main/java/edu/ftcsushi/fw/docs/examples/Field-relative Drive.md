# Field-relative Drive

**Learning mode:** Architecture reference

**Source entry:** [`FieldRelativeDriveExample.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/fieldrelative/opmode/FieldRelativeDriveExample.java>)

Study this after the robot-relative [`StarterTeleOp`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/starter/opmode/StarterTeleOp.html>).
The example keeps the ordinary managed lifecycle: the OpMode only configures a `RobotProgram`, the
heading estimator is an upstream service, and the final drivetrain still consumes a robot-centric
`DriveSignal`.

**Buildable promise:** copy the complete files in the collapsed working slice after copying the
Starter. The slice replaces only the host, profile, prestart, controls, and composition root.

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

## Files you will create

Create `FieldRelativeDriveExample`, `FieldRelativeExampleProfile`, `FieldRelativeExamplePrestart`,
`FieldRelativeExampleControls`, and `FieldRelativeExampleRobot` in the packages shown by the
maintained example. These two complete files show the FTC entry and the only field-relative intent
conversion; the profile, prestart, and root supply the reviewed station facts and lifecycle owners
described above.

## Complete working slice

<details>
<summary>Complete working slice: FTC host</summary>

<!-- source-file: TeamCode/src/main/java/edu/ftcsushi/robots/examples/fieldrelative/opmode/FieldRelativeDriveExample.java -->
```java
package edu.ftcsushi.robots.examples.fieldrelative.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.robots.examples.fieldrelative.robot.FieldRelativeExampleProfile;
import edu.ftcsushi.robots.examples.fieldrelative.robot.FieldRelativeExampleRobot;

/** Managed example of explicit station-relative TeleOp drive using the Hub IMU. */
@TeleOp(name = "FW Example: Field-relative drive", group = "FW Examples")
@Disabled
public final class FieldRelativeDriveExample extends FtcRobotOpMode {
    @Override
    protected void configure(RobotProgram program) {
        new FieldRelativeExampleRobot(hardwareMap).declareTeleOp(
                program, FieldRelativeExampleProfile.current(), gamepad1);
    }
}
```

</details>

<details>
<summary>Complete working slice: controls owner</summary>

<!-- source-file: TeamCode/src/main/java/edu/ftcsushi/robots/examples/fieldrelative/robot/FieldRelativeExampleControls.java -->
```java
package edu.ftcsushi.robots.examples.fieldrelative.robot;

import edu.ftcsushi.fw.drive.DriveSource;
import edu.ftcsushi.fw.drive.source.GamepadDriveSource;
import edu.ftcsushi.fw.ftc.input.GamepadDevice;
import edu.ftcsushi.fw.localization.HeadingEstimator;

/** Owns the field-relative example's driver meanings. */
final class FieldRelativeExampleControls {
    private final DriveSource drive;

    FieldRelativeExampleControls(GamepadDevice driver,
                                 HeadingEstimator heading,
                                 FieldRelativeExamplePrestart prestart,
                                 GamepadDriveSource.Config config) {
        drive = new GamepadDriveSource(
                driver.leftX(),
                driver.leftY(),
                driver.rightX(),
                config
        ).fieldRelativeTo(heading, prestart::frozenControlUpFieldHeadingRad);
    }

    DriveSource drive() {
        return drive;
    }
}
```

</details>

## Verify the slice

Run:

```powershell
.\gradlew.bat --console=plain :TeamCode:compileDebugJavaWithJavac `
  :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.fieldrelative.*
```

Expected checkpoint: compilation and the field-relative focused tests pass. Translation still
fails closed when heading evidence is unavailable; no software check proves Hub orientation or
physical drive direction.

[Back to examples](<README.md>)
