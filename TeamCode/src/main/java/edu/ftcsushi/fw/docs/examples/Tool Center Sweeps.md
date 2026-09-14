---
tags:
  - Advanced
---

# Check which ball centers a straight move would encounter

**Outcome:** calculate where a stationary ball's center would enter and leave a tool's chosen
window during one modeled straight move. **Before this page:** understand
[robot-relative coordinates](<../build/First Drive.md>) and the
[field-relative distinction](<Field-relative Drive.md#what-up-means>). This is an independent,
hardware-free geometry example. It does not need a camera, localization, an OpMode, or a running robot.

## Choose a center window, not the physical opening

Driving toward every ball center individually may be unnecessary: a wide intake could encounter
several nearby balls during one move. First we need a smaller question: does **this center point**
pass through the useful part of the intake? A **center window** is the rectangle of acceptable
center positions relative to a tool. It is not the physical opening, and overlap with a ball's edge
does not count. A center on the physical opening edge might leave half the ball outside and bounce away.

For illustration, a 12-inch opening and a 4-inch-diameter ball allow a 6-inch-wide center window
with an extra inch of side clearance at its boundaries. Diameter is the distance across the ball;
its radius is half that distance. Even the wider 8-inch center window would leave no extra clearance
at its edges. Choose already-reduced center bounds for your real mechanism and position uncertainty;
the framework does not add a radius or shrink the window again. These numbers are not physical defaults.

![Two illustrative top views compare a rejected ball center at the physical opening edge with an included center at the inset window boundary; circles show ball outlines and dots show their centers.](<../assets/diagrams/tool-center-window.svg>)

In words: measure left/right from the tool centerline. The physical sides are at -6 and +6 inches;
the chosen center bounds are -3 and +3 inches. A 4-inch ball centered at +6 extends to +8 and is
rejected. Centered at +3, it extends to +5, leaving one extra inch before the +6 physical side.
The accepted mathematical boundary is the **inset center boundary**, not a physical edge.

Forward/back limits are also chosen center bounds. Here the window extends from one inch behind
the tool origin to two inches ahead. A **tool origin** is the point on the mechanism from which
those coordinates are measured; it need not be robot center or the camera lens.

## Describe one move and its fixed tool

A **pose** combines position and heading (facing direction). In
[`Pose2d`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/core/geometry/Pose2d.html>),
`new Pose2d(x, y, heading)` constructs that value, with distances in inches and heading in radians.
Zero heading faces field +X. Tool coordinates use +X forward and +Y left; positive headings turn
counter-clockwise. The tool pose below is relative to the robot, not relative to the field.

A **sweep** is the set of positions visited by the window during the modeled move.
[`ToolSweep2d`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/spatial/ToolSweep2d.html>)
supports a straight robot-center segment with **one fixed heading** and a rigid tool mount.
The endpoint deliberately asks only for field X/Y, not another heading. It cannot describe turning
or a curved path. No commands are sent by constructing or querying it.

This is the complete geometry setup from
[`IntakeSweepExample`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/intakesweep/IntakeSweepExample.html>):

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/intakesweep/IntakeSweepExample.java -->
```java
private static final ToolSweep2d MODELED_MOVE = ToolSweep2d
        .straightFrom(new Pose2d(10.0, 20.0, 0.0))
        .toFieldPoint(30.0, 20.0)
        .throughTool(new Pose2d(6.0, 1.0, 0.0))
        .centerWindowInches(-1.0, 2.0, 6.0);
```

Each chained method answers the next required question, a **staged builder**. The last answer
returns the finished, unchanging sweep; there is no additional `build()` call. `private static final`
keeps that one object inside this example, initialized once and reused for its point queries.

| Answer to change | Meaning of the illustrative value |
|---|---|
| `straightFrom(...)` | Robot center starts at field `(10,20)`, with heading `0` throughout |
| `toFieldPoint(...)` | Robot center ends at `(30,20)`: a 20-inch segment |
| `throughTool(...)` | Intake origin is 6 inches forward and 1 inch left of robot center, facing forward |
| `centerWindowInches(-1, 2, 6)` | Tool-forward center limits `[-1,+2]`; full lateral width 6, giving side limits `[-3,+3]` |

The tool offset is applied once. Initially its field origin is `(16,21)`, so the window's field
X bounds are `[15,18]`. After the full move those X bounds are `[35,38]`; its field Y bounds remain
`[18,24]`. The calculation checks the continuous straight translation, not just the endpoints.

## Read the encounter, not a capture result

An **encounter** means the supplied center lies in that moving window for some part of the segment.
The result reports **robot-center travel in inches** at entry and exit. It does not report time,
measured progress, tool-to-ball range, or captured-ball count. The example exposes the result directly:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/intakesweep/IntakeSweepExample.java -->
```java
public static ToolSweep2d.Encounter encounterBallCenter(
        double fieldCenterXInches, double fieldCenterYInches) {
    return MODELED_MOVE.encounterFieldCenter(fieldCenterXInches, fieldCenterYInches);
}
```

The dot in `ToolSweep2d.Encounter` names the result type belonging to the sweep class. Call
`hasEncounter()` first. On a hit, `entryTravelInches()` and `exitTravelInches()` give the inclusive
interval. On a miss, those distance methods throw an error instead of supplying a fake distance.

| Authored field center | Expected result from this example |
|---|---|
| `(25,22)` | Entry at 7 inches; exit at 10 inches |
| `(25,24)` | Same interval; center on the chosen inner side boundary |
| `(25,27)` | Miss: center at the illustrative physical opening edge |
| `(25,25)` | Miss: ball edge overlaps the window, but its center is outside |
| `(38,21)` | Encounter at exactly 20 inches, the end of the move |
| `(38.5,21)` | Miss: the finite move does not reach this center |

For `(25,22)`, the advancing front bound reaches X=25 after 7 inches (`18+7`), and the rear bound
passes it after 10 inches (`15+10`). Its Y=22 remains inside the lateral center window. Repeating
that query gives the same interval; it does not identify a second ball.

### Optional software checkpoint

**Question:** over which part of the move is the authored center inside the window?
**Keep real:** the maintained example's geometry and framework calculation.
**Replace:** camera measurements with authored field coordinates.
**Observe:** the encounter flag and travel interval.
**Cannot conclude:** that a real ball was seen, reached, or captured.

The central assertions from `IntakeSweepExampleTest`'s
`reportsTravelIntervalForAuthoredBallCenter` scenario are:

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/intakesweep/IntakeSweepExampleTest.java -->
```java
assertTrue(encounter.hasEncounter());
assertEquals(7.0, encounter.entryTravelInches(), 1e-9);
assertEquals(10.0, encounter.exitTravelInches(), 1e-9);
```

`assertTrue` checks a condition. `assertEquals(expected, actual, tolerance)` checks the actual
number against an independently calculated answer; `1e-9` permits a tiny floating-point rounding
difference. Open the test below and optionally run the scenario using its Android Studio gutter icon.
It calls the maintained example; it does not recreate its implementation to obtain the result.

**Read the causal chain:** authored point → real configured sweep → encounter flag → travel distances.
**Proves:** the stated synthetic geometry and the separate boundary cases in this test file.
**Does not prove:** a physical trajectory, body clearance, useful allowances, or successful collection.
**Next gate:** validate the real mechanism and center estimates before using this geometry to plan motion.

Exact file manifest — no additional mechanism or OpMode is required:

- [Complete source: `IntakeSweepExample.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/intakesweep/IntakeSweepExample.java>) — example
- [Complete source: `IntakeSweepExampleTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/intakesweep/IntakeSweepExampleTest.java>) — checkpoint

## Keep planning, boundaries, and pickup separate

A ball encounter near a wall says nothing about whether the robot or tool hits that wall. This
window does **not** constrain the robot to its allowed autonomous territory. Whole-robot travel
bounds, motion execution, and independent capture feedback remain separate responsibilities.

The supplied point must already be an estimated center in the same field coordinates as the segment.
A camera's image-box center or targeting point is not automatically a physical ball center; see
[validate the center estimate](<../drive-vision/Vision Targets.md#plan-for-the-balls-center-not-edge-contact>).
Observation freshness and ball motion are not modeled by this geometry. Invalid or non-finite
coordinates raise an argument error, not an ordinary miss.

### Can a claw use it?

Yes, for a **fixed claw's center window**. Use the claw's rigid robot-relative pose in `throughTool`
and its own already-reduced center bounds. A zero-length segment asks about a stationary window;
an included point has entry and exit travel both zero. A zero-depth forward window is also allowed.
This is still two-dimensional positioning geometry: height, object orientation, moving arms, gripping,
lifting, and holding are not represented. A translating encounter is not a selected grasp pose.

The existing [tool-relative approach](<../drive-vision/Spatial Queries.md#face-a-point-and-stop-short-of-it>)
can describe a positive gap before a claw or intake reaches a point. Claw close/hold/lift commands
and grip confirmation belong to its mechanism's Tasks. Do not substitute close/open for the
camera-only helper's `finalIntake` callback: that helper stops the intake on completion, whereas a
claw may need to **keep holding**. No claw pickup lifecycle is implied by sharing the geometry.

**Next:** adapt only the authored geometry and checkpoint to your tool; keep these software results
separate from physical capture evidence and any allowed-travel check.
