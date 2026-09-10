---
tags:
  - Test & Tune
---

# Estimate between calibration measurements

**Learning mode:** Optional calculation-only lesson

**Outcome:** predict a speed between recorded samples, using either one input or two, and explain
why that number is not permission to shoot.

**Before reading:** basic Java numbers and method calls. Arrays and tables are explained here.
No robot, installation, controller knowledge or earlier calibration run is required to read.
For optional test execution, complete [Build and Run](<../getting-started/Build and Run.md>).

Suppose a team has tried several shooter speeds at measured positions and kept the trials that
meet its written success criterion. That work is **calibration**: relating known inputs to useful
settings through measurement. What setting should the robot consider between two tested positions?
**Interpolation** estimates between stored samples by blending nearby values. It does not collect
measurements, fit a projectile model or prove that an untested position works.

This lesson's numbers are **illustrative, not measured or recommended shooter settings**. Speed is
in encoder **ticks per second**: how quickly an encoder's movement count changes. The example only
returns numbers. It is not an OpMode and has no Driver Station program to run.

## Start with one input

Choose a distance-only table when your reviewed measurements justify choosing speed from distance
alone. **One-dimensional**, or **1D**, means one input, not that the robot moves on a line.
The illustrative samples are 3000 ticks/second at 24 inches and 3400 ticks/second at 48 inches.
Halfway between those distances, at 36 inches, expect **3200 ticks/second**.

In Java, an **array** is an ordered list of values. `new double[] {24.0, 48.0}` creates a list of
decimal numbers. Matching positions in two arrays pair each distance with its speed. Keep the
distance array strictly increasing: later distances must be larger, with no duplicates.
Every authored number must be **finite**: neither infinity nor Java's `NaN` ("not a number").

[`InterpolatingTable1D`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/core/math/InterpolatingTable1D.html>)
is the stored table. Its `ofSorted(...)` method checks and copies the arrays, returning an
**immutable** table: its stored values cannot change afterward. The declaration below belongs
inside the example class. `private` keeps the table inside that class; `static final` retains one
shared table reference that is assigned once, when Java initializes the class.

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/calibration/ShotSpeedCalibration.java -->
```java
private static final InterpolatingTable1D DISTANCE_TO_SPEED = InterpolatingTable1D.ofSorted(
        new double[] {24.0, 48.0},
        new double[] {3000.0, 3400.0});
```

The first array is distance in inches; the second is speed in ticks/second. Those are the exact
arrays to replace with reviewed measurements when adapting this pattern. Sushi owns the copying,
ordering checks and blending; your robot owns the measurements and what their units mean.

The example's method gives the calculation a robot-specific name. `public static` lets another
class call it without constructing an example object. A call runs now and returns one number;
it does not save work for a later loop or send a motor request:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/calibration/ShotSpeedCalibration.java -->
```java
public static double speedForDistanceTicksPerSec(double distanceIn) {
    return DISTANCE_TO_SPEED.interpolate(distanceIn);
}
```

At 36 inches, `interpolate(...)` takes half the 400-tick/second difference and adds it to 3000.
The result is 3200. Repeated calls reuse the table; there is no changing state to update or stop.

## Use two inputs only when measurements need them

Sometimes forward and sideways displacement each affect the useful setting. A **two-dimensional**
or **2D** table uses two inputs. This is an alternative to the distance table, not another table
every robot must use. Collect a complete set of measurements for the chosen input combinations
before adopting it; do not fill missing measurements with guessed zeroes.

Here both inputs are target displacement from the robot center, in inches: positive **forward**
and positive **left**. Negative left means right. These are already robot-relative numbers;
this table does not convert camera or field coordinates or correct a displaced shooter.

A **grid** is a rectangular table. Each **row** below chooses the first input, forward displacement;
each **column** chooses the second input, left displacement. Every cell is speed in ticks/second.

| Forward inches (rows) / left inches (columns) | -12 (right) | 0 | +12 (left) |
| --- | ---: | ---: | ---: |
| 24 | 3100 | 3000 | 3100 |
| 48 | 3500 | 3400 | 3500 |

Read the top middle cell as “24 inches forward and 0 left gives 3000 ticks/second.” The bottom
right cell means “48 forward and 12 left gives 3500.” Rows are always first input, columns second.

In Java, `double[][]` is an array of row arrays. The outer braces contain the rows; each inner
brace pair contains that row's columns in left-value order. Array positions, called **indices**,
start at zero: `values[0][1]` would name the top middle cell. This example needs two rows of three
values, matching its two forward samples and three left samples.

An **axis** here is the ordered list of samples for one input.
[`InterpolatingTable2D`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/core/math/InterpolatingTable2D.html>)
uses the same `ofSorted(...)` spelling, now with both axes and the grid. Each axis must be finite
and strictly increasing; every row must exist and have exactly one finite value per second-axis
sample. Construction rejects malformed data and copies every row.

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/calibration/ShotSpeedCalibration.java -->
```java
private static final InterpolatingTable2D OFFSET_TO_SPEED = InterpolatingTable2D.ofSorted(
        new double[] {24.0, 48.0},
        new double[] {-12.0, 0.0, 12.0},
        new double[][] {
                {3100.0, 3000.0, 3100.0},
                {3500.0, 3400.0, 3500.0}
        });
```

The example keeps that alternative behind its offset-based method, again calculating immediately:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/calibration/ShotSpeedCalibration.java -->
```java
public static double speedForOffsetTicksPerSec(double targetForwardIn, double targetLeftIn) {
    return OFFSET_TO_SPEED.interpolate(targetForwardIn, targetLeftIn);
}
```

At `(36, 6)`, first blend halfway between the left 0 and left 12 columns in each row. At forward 24
that gives 3050; at forward 48 it gives 3450. Then blend halfway between those results for forward
36: **3250 ticks/second**. Blending along both axes is called **bilinear interpolation**. It
estimates from the surrounding four cells, not from the distance-only table.

## Separate a computable number from accepted evidence

For both table types, a finite input outside an axis is **clamped**: treated as the nearest endpoint
of that axis. Distance 60 therefore returns 3400. Forward 60 and left -20 independently clamp to
forward 48 and left -12, returning 3500. The table does not report that clamping as a rejection.

If any input is non-finite, the result is `Double.NaN`, not an endpoint speed. A robot can use
`Double.isFinite(result)` to check whether the returned number is finite, but that check alone
does not establish sensor availability, freshness or an accepted operating range. A stale but
finite position still produces a finite result. Check those facts separately before using a number
as a mechanism request; the numerical clamp is not range acceptance or a hardware limit.

**Optional edge case:** an axis may contain one sample. That input has only one endpoint, so a
2D table with one singleton axis behaves like a 1D table; with both axes singleton it is constant.
Non-finite inputs still return `NaN`. This supports fixed dimensions without another factory.

## Optional software checkpoint

**Question:** do the two alternative maps return the predicted values between samples?
**Keep real:** the maintained example's tables and Sushi's interpolation.
**Replace:** observed distance/offset with the explicit numeric inputs below; there is no hardware.
**Observe:** two returned speeds.
**Cannot conclude:** any measurement is fresh, either model is physically accurate, or a shot succeeds.

The supplied test already arranges distance 36, forward 36 and left 6. This exact excerpt calculates
both alternatives and checks them. `assertEquals(expected, actual, tolerance)` reports failure if
the difference exceeds the final argument; `1e-9` means 0.000000001 ticks/second for this arithmetic
check, not a motor tolerance. The full test's `@Test` marker lets the test runner discover the method.

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/calibration/ShotSpeedCalibrationTest.java -->
```java
double distanceSpeed = ShotSpeedCalibration.speedForDistanceTicksPerSec(distanceIn);
double offsetSpeed = ShotSpeedCalibration.speedForOffsetTicksPerSec(
        targetForwardIn, targetLeftIn);

// ASSERT: 36 is halfway from 24 to 48; 6 is halfway from left 0 to left 12.
assertEquals(3200.0, distanceSpeed, 1e-9);
assertEquals(3250.0, offsetSpeed, 1e-9);
```

**Read the causal chain:** authored input → real example lookup → returned number → assertion.
There is no heartbeat or simulated shot. **Proves:** a passing run checks these software values;
the second supplied test distinguishes finite clamping from unavailable input. **Does not prove:**
the example settings suit a real robot, or that your adapted code works. **Next gate:** when adopting
the pattern, test your own tables and verify them against independently measured physical outcomes.

Reading the predictions completes this lesson. After setup, you may run the supplied checkpoint
from the repository root; the command selects this test and requests a fresh execution:

=== "Windows"

    ```powershell
    .\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests 'edu.ftcsushi.robots.examples.calibration.ShotSpeedCalibrationTest' --rerun-tasks
    ```

=== "macOS"

    ```bash
    ./gradlew --console=plain :TeamCode:testDebugUnitTest --tests 'edu.ftcsushi.robots.examples.calibration.ShotSpeedCalibrationTest' --rerun-tasks
    ```

Expected result: `BUILD SUCCESSFUL` with both tests passing. No device is connected or commanded.

## Complete files and next choice

- Main file — [Complete source: `ShotSpeedCalibration.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/calibration/ShotSpeedCalibration.java>).
- Test file — [Complete source: `ShotSpeedCalibrationTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/calibration/ShotSpeedCalibrationTest.java>).
- Exact example API — [`ShotSpeedCalibration`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/calibration/ShotSpeedCalibration.html>).

For actual measurements, continue to [recording distance and shot success](<Control Tuning Workflow.md#recording-distance-and-shot-success>)
and that workflow's reviewed hardware prerequisites. This lesson supplies no shooter, operating
permission or physical test fixture. Keep accepted calibration and safety policy in the adopting
robot; keep the chosen lookup separate from deciding whether a shot is allowed.
