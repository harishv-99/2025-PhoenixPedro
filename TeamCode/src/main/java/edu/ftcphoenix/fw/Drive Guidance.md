# Drive Guidance

Phoenix `DriveGuidance` is a small “driver assist” API built around two ideas:

1. **Describe what you want** (a *plan*): go to a point, look at a point, or both.
2. **Apply it without rewriting TeleOp** by turning that plan into a `DriveOverlay` and enabling it
   with `DriveSource.overlayWhen(...)`.

It replaces the older `drive.assist` utilities (`TagAim`, `BearingSource`, etc.). The new API:

* uses a single mental model for “go-to” and “aim-at” behaviors
* supports multiple feedback styles (vision-only, localization-only, or adaptive)
* makes the “controlled point” on the robot explicit via `ControlFrames`

---

## Layering: math, predicates, controllers

Phoenix keeps “where am I?” logic separate from “how do I drive?” logic. This makes it easier to:

* gate actions (shooting/intake) based on position without enabling a drive assist,
* reuse the same geometry checks in TeleOp, tasks, and testers, and
* keep DriveGuidance / tasks focused on generating commands.

The three layers are:

1) **Spatial math** (pure geometry): bearings, errors, frame transforms. See `edu.ftcphoenix.fw.spatial.SpatialMath2d`.
2) **Spatial predicates** (yes/no decisions): zone membership and “safe” booleans, often with hysteresis.
   See `ConvexRegion2d` / `ConvexRegions2d`, `RobotZones2d`, `ZoneLatch`, `RobotHeadings2d`, and `HeadingLatch` in `edu.ftcphoenix.fw.spatial`.
3) **Controllers** (produce commands): DriveGuidance overlays and drive tasks (e.g., `DriveGuidanceTask` / `GoToPoseTasks`) use errors from the math layer and turn them into drive outputs.

---

## Quick start

### 1) Start with a normal TeleOp drive source

```java
DriveSource base = GamepadDriveSource.teleOpMecanum(gamepads);
```

### 2) Build a guidance plan

This plan **aims** at the currently observed tag center (it does not translate).

```java
ObservationSource2d obs = ObservationSources.aprilTag(scoringTarget, cameraMount);

DriveGuidancePlan aimPlan = DriveGuidance.plan()
        .aimTo()
            .tagRelativePointInches(0.0, 0.0)   // center of the observed tag
            .doneAimTo()
        .feedback()
            .observation(obs)                 // vision-only feedback
            .doneFeedback()
        .build();
```

### 3) Enable the overlay

Drive overlays take a `BooleanSupplier`. The most common patterns are:

* **Hold to enable** (while the button is held)
* **Toggle to enable** (press once to turn it on, press again to turn it off)

#### Hold to enable

```java
DriveSource drive = base.overlayWhen(
        gamepads.p1().leftBumper()::isHeld,
        aimPlan.overlay(),
        DriveOverlayMask.OMEGA_ONLY
);
```

#### Toggle to enable

Use `Button.isToggled()` via a method reference to get a `BooleanSupplier` that flips on each press:

```java
DriveSource drive = base.overlayWhen(
        gamepads.p1().leftBumper()::isToggled,
        aimPlan.overlay(),
        DriveOverlayMask.OMEGA_ONLY
);
```

That’s it: your driver keeps full stick translation, and the overlay “owns” only omega.

#### Multiple overlays (recommended): overlay stack

When you want to layer multiple assists (for example: a **pose lock** that overrides translation
*and* an **auto-aim** that overrides omega), use an overlay stack.

```java
DriveSource drive = base.overlayStack()
        .add(
                "shootBrace",
                () -> shootBraceEnabled,
                DriveGuidance.poseLock(poseEstimator),
                DriveOverlayMask.TRANSLATION_ONLY
        )
        .add(
                "autoAim",
                gamepads.p2().leftBumper()::isHeld,
                aimPlan.overlay(),
                DriveOverlayMask.OMEGA_ONLY
        )
        .build();
```

Notes:

* Layers are evaluated **top to bottom**.
* If two enabled layers claim the same DOF, **the last layer wins** for that DOF.

---

## Common recipes

These are small, copy-paste friendly patterns that show up a lot in TeleOp.

### Aim-only: observed tag (vision-only)

```java
DriveGuidancePlan plan = DriveGuidance.plan()
        .aimTo().tagCenter().doneAimTo()
        .feedback().observation(obs).doneFeedback()
        .build();

DriveSource drive = base.overlayWhen(
        gamepads.p2().leftBumper()::isHeld,
        plan.overlay(),
        DriveOverlayMask.OMEGA_ONLY
);
```

### Query a plan's errors (without enabling the overlay)

Sometimes you want the *same math* as a DriveGuidance assist, but you don't actually want to
take control of the drivetrain.

Use `DriveGuidanceQuery` / `DriveGuidanceStatus` via `plan.query()`:

```java
DriveGuidanceQuery q = plan.query();

// Each loop:
DriveGuidanceStatus s = q.sample(clock, DriveOverlayMask.OMEGA_ONLY);

if (s != null && s.omegaWithin(Math.toRadians(2))) {
    telemetry.addLine(">>> AIMED <<<");
}
telemetry.addData("omegaErrDeg", s != null ? Math.toDegrees(s.omegaErrorRad) : Double.NaN);
```

### Translate + aim: observed tag-relative point (vision-only)

Drive to a point relative to the currently observed tag while also aiming at it.

```java
DriveGuidancePlan plan = DriveGuidance.plan()
        .translateTo().tagRelativePointInches(6, 0).doneTranslateTo() // 6" in front of the observed tag
        .aimTo().tagCenter().doneAimTo()
        .feedback().observation(obs).doneFeedback()
        .build();
```

### Translate + aim: fixed tag ID using field pose (tag may disappear)

If you have localization, you can keep driving/aiming even if the camera loses the tag.

```java
DriveGuidancePlan plan = DriveGuidance.plan()
        .translateTo().tagRelativePointInches(1, 6, 0).doneTranslateTo()
        .aimTo().tagCenter(1).doneAimTo()
        .feedback().fieldPose(poseEstimator, tagLayout).doneFeedback()
        .build();
```

### Aim to an absolute field heading

```java
DriveGuidancePlan plan = DriveGuidance.plan()
        .aimTo().fieldHeadingDeg(90).doneAimTo()
        .feedback().fieldPose(poseEstimator).doneFeedback()
        .build();
```

### Robot-relative “nudge” translation

Useful for micro adjustments. Captures the current pose when the overlay turns on.

```java
DriveGuidancePlan plan = DriveGuidance.plan()
        .translateTo().robotRelativePointInches(6, 0).doneTranslateTo()
        .feedback().fieldPose(poseEstimator).doneFeedback()
        .build();
```

---

## What a plan can do

`DriveGuidancePlan` can control up to two degrees of freedom:

* **Translation**: move a point on the robot to a target point
* **Omega**: rotate the robot so a point on the robot either “looks at” a target point
  <em>or</em> matches an absolute field heading

You can use either one independently, or both together.

### Translation target

* `.translateTo().fieldPointInches(x, y)` — a fixed point on the field
* `.translateTo().tagRelativePointInches(tagId, forward, left)` — a point in a tag’s coordinate frame
* `.translateTo().tagRelativePointInches(forward, left)` — a point in the **currently observed** tag’s frame
* `.translateTo().robotRelativePointInches(forward, left)` — "nudge" by an offset from wherever you are
  (requires field pose)

### Aim target

* `.aimTo().fieldPointInches(x, y)`
* `.aimTo().fieldHeadingDeg(deg)` / `.aimTo().fieldHeadingRad(rad)` — turn to an absolute field heading
  (requires field pose)
* `.aimTo().tagRelativePointInches(tagId, forward, left)`
* `.aimTo().tagRelativePointInches(forward, left)` — **currently observed** tag

---

## ControlFrames

By default, plans control the robot’s center.

If you want to aim using an off-center mechanism (like a shooter), you can set an aim control frame:

```java
ControlFrames frames = ControlFrames.robotCenter()
        .withAimFrame(new Pose2d(6.0, 0.0, 0.0));   // 6" forward of robot center

DriveGuidancePlan plan = DriveGuidance.plan()
        .aimTo().tagCenter(1).doneAimTo()
        .feedback().fieldPose(poseEstimator, tagLayout).doneFeedback()
        .controlFrames(frames)
        .build();
```

Now the robot rotates so that the shooter point is what “faces” the target.

---

## Feedback modes

Guidance needs a way to know where the robot is (or where the target is).

### Observation-only feedback

Use when you only care about **relative** movement and the target is visible.

```java
ObservationSource2d obs = ObservationSources.aprilTag(tagTarget, cameraMount);

DriveGuidancePlan plan = DriveGuidance.plan()
        .aimTo().tagRelativePointInches(0.0, 0.0).doneAimTo()
        .feedback().observation(obs).doneFeedback()
        .build();
```

Pros:

* simple
* no odometry required
* very accurate up close

Cons:

* if the target drops out of view, guidance can’t “see” anymore

### Field-pose feedback

Use when you have a localization estimate (odometry, fused vision, etc.) and want to aim/drive to a
known field point.

```java
DriveGuidancePlan plan = DriveGuidance.plan()
        .translateTo().fieldPointInches(12, 48).doneTranslateTo()
        .aimTo().fieldPointInches(0, 0).doneAimTo()
        .feedback().fieldPose(poseEstimator, tagLayout).doneFeedback()
        .build();
```

Pros:

* works even when you can’t see the target
* supports field points and tag-relative points (via `TagLayout`)

Cons:

* depends on localization quality

### Adaptive feedback

If you configure **both** observation and field pose feedback, DriveGuidance becomes adaptive:

* far away: prefer field pose (stable global behavior)
* close: prefer observation (better local accuracy)
* for omega: can prefer observation whenever the target is visible

```java
DriveGuidancePlan plan = DriveGuidance.plan()
        .translateTo().tagRelativePointInches(1, 6, 0).doneTranslateTo()
        .aimTo().tagCenter(1).doneAimTo()
        .feedback()
            .fieldPose(poseEstimator, tagLayout)
            .observation(obs)
            .gates(10.0, 14.0, 0.20)   // enter, exit, blend (inches, inches, seconds)
            .doneFeedback()
        .build();
```

---

## Loss policy

When guidance can’t compute an output (no pose, no target, too old, too low quality), it needs to
decide what to do.

* `PASS_THROUGH` (default): output mask is reduced to the DOFs we can solve; other DOFs remain under
  driver control.
* `ZERO_OUTPUT`: if any requested DOF can’t be solved, output **zeros** for all requested DOFs.

`ZERO_OUTPUT` is useful for “hold still” behaviors.

---

## Pose lock

If you want the robot to “brace” in TeleOp (resist bumps), use `poseLock`.

```java
DriveOverlay lock = DriveGuidance.poseLock(poseEstimator);

DriveSource drive = base.overlayWhen(
        () -> gamepads.p2().x().isHeld(),
        lock,
        DriveOverlayMask.ALL
);
```

Tip: pose lock only works as well as your localization.

---

## Debugging tips

* Start with `feedback().fieldPose(...).doneFeedback()` and verify your pose estimate looks sane.
* Then add `observation(...)` and switch to adaptive.
* Add `debugDump(...)` calls to your telemetry if you want to see which feedback source is active.
