# Drive Guidance

Drive Guidance is Phoenix’s structured way to help you:

- **Create driver assists** (e.g., “aim at the speaker while I drive”).
- **Run the same motion as a Task** (e.g., “go to this pose in auto”).
- **Query the same spatial logic** for gating and telemetry (e.g., “only shoot when aimed”).

This doc also covers a parallel, lighter-weight set of APIs for answering questions like:

- “Is the robot (partially) in the shooting zone?”
- “Is the robot fully inside the parking box?”
- “Is a mechanism-facing control frame pointing at a field point?”

…without needing any DriveGuidance plan at all.

---

## Spec vs Plan

DriveGuidance intentionally has **two** objects:

### DriveGuidanceSpec

A **controller-neutral** description of:

- **Targets** (translate to / aim to)
- **Control frames** (which point on the robot you’re controlling)
- **Feedback selection policy** (observation, field-pose, or adaptive blending)

A spec contains **no tuning** and does not generate drive commands by itself.

### DriveGuidancePlan

An **executable** bundle:

- `DriveGuidanceSpec` (**what**) +
- `DriveGuidancePlan.Tuning` (**how aggressively to drive the error to zero**)

Plans are what you attach to:

- a TeleOp overlay: `plan.overlay()`
- a task: `plan.task(...)`
- a query sampler: `plan.query()`

---

## The 3-layer architecture

DriveGuidance (and related geometry utilities) follow a simple layered structure:

1) **Spatial math** (pure geometry)
- Example: `SpatialMath2d`

2) **Spatial predicates** (answer yes/no questions)
- Examples: `RobotZones2d`, `RobotHeadings2d` (+ latches for hysteresis)

3) **Controllers** (turn errors into drive commands)
- Example: `DriveGuidancePlan` executed through `DriveGuidanceOverlay` / `DriveGuidanceTask`

The important idea: you can often solve “should I enable X?” using layer (2) without involving controllers at all.

---

## Targets you can express in a spec

A DriveGuidanceSpec can include any combination of:

- **Translation targets**
  - Field point: `fieldPointInches(x, y)`
  - Tag-relative point: `tagRelativePointInches(tagId, forward, left)`
  - Robot-relative delta: `robotRelativePointInches(forward, left)` (captured on enable)

- **Aim targets**
  - Field heading: `fieldHeadingRad(heading)`
  - Tag heading: `tagHeadingRad(tagId, offset)`
  - A point (aim at the point): `fieldPointInches(...)` or `tagRelativePointInches(...)`

---

## Feedback sources in a spec

A spec can use:

- **Observation feedback** (`ObservationSource2d`) – robot-relative, typically AprilTag based.
- **Field pose feedback** (`PoseEstimator`) – absolute field pose from localization.
- **Adaptive feedback** (both configured) – selects observation vs field pose per DOF using gates/hysteresis.

Key knobs:

- `gates(enterRangeIn, exitRangeIn, blendStepPerSec)`
- `preferObsOmega(true/false)`
- `lossPolicy(PASS_THROUGH or ZERO_OUTPUT)`

---

## Building specs and plans

### Build a spec (targets + feedback + control frames)

You build a spec using the dedicated `DriveGuidance.spec()` staged builder, then finish with `build()`:

```java
DriveGuidanceSpec spec = DriveGuidance.spec()
        .translateTo()
            .fieldPointInches(48, 24)
            .doneTranslateTo()
        .aimTo()
            .tagRelativePointInches(/*tagId=*/ 5, /*forward=*/ 6, /*left=*/ 0)
            .doneAimTo()
        .controlFrames(ControlFrames.robotCenter())
        .feedback()
            .observation(tagObsSource)
            .fieldPose(poseEstimator, tagLayout)
            .gates(36, 60, 2.0)
            .preferObsOmega(true)
            .lossPolicy(DriveGuidanceSpec.LossPolicy.PASS_THROUGH)
            .doneFeedback()
        .build();
```

### Build a plan from a spec (tuning only)

```java
DriveGuidancePlan plan = DriveGuidance.plan(spec)
        .tuning(
                DriveGuidancePlan.Tuning.defaults()
                        .withTranslateKp(0.06)
                        .withAimKp(2.0)
        )
        .build();
```

### Build a plan in one shot (spec + tuning together)

If you don’t need to reuse the spec separately:

```java
DriveGuidancePlan plan = DriveGuidance.plan()
        .translateTo()
            .fieldPointInches(48, 24)
            .doneTranslateTo()
        .aimTo()
            .tagRelativePointInches(5, 6, 0)
            .doneAimTo()
        .feedback()
            .observation(tagObsSource)
            .fieldPose(poseEstimator, tagLayout)
            .doneFeedback()
        .tuning(DriveGuidancePlan.Tuning.defaults())
        .build();
```

---

## Using a plan

### TeleOp overlay

```java
DriveOverlayStack overlays = new DriveOverlayStack();

// Add the guidance overlay when you want assist.
if (enableAssist) {
    overlays.add(plan.overlay());
}

DriveSignal driver = driverDriveSource.sample();
DriveSignal assisted = overlays.apply(driver, loopClock);
```

### Task execution

```java
DriveGuidanceTask task = plan.task(drivebase, DriveGuidanceTask.Config.defaults()
        .withTimeoutSec(2.0)
        .withTranslationTolInches(1.0)
        .withOmegaTolRad(Math.toRadians(2))
);
```

### Query sampler (reuse exact same evaluation)

```java
DriveGuidanceQuery q = plan.query();
q.reset();

DriveGuidanceStatus s = q.sample(loopClock);
boolean okToShoot = s.translationWithin(3.0) && s.omegaWithin(Math.toRadians(3));
```

---

## Manual queries

If you don’t need a drive assist (or you don’t want any tuning/controller behavior at all), prefer the “predicate” layer.

### Zone membership (robot vs convex region)

Create your robot geometry once:

```java
RobotGeometry2d robot = RobotGeometry2d.builder()
        .rectangleFootprint(/*length=*/ 18, /*width=*/ 18)
        .point("shooter", /*forwardIn=*/ 9, /*leftIn=*/ 0)
        .build();
```

Create a region:

```java
ConvexRegion2d shootingZone = ConvexRegions2d.convexPolygonInches(
        new double[] { 36, 60, 60, 36 },
        new double[] { 12, 12, 36, 36 }
);
```

Now choose the rule you care about:

- **“Robot overlaps the zone at least a little”** (good for “I can shoot if any part is in”)  
  Uses probe points on the footprint.

```java
RobotZone2d overlaps = RobotZones2d.in(shootingZone)
        .robot(robot)
        .footprintOverlaps(/*samplesPerEdge=*/ 3);
```

- **“Robot is fully inside the zone”** (good for parking box / endgame constraints)

```java
RobotZone2d fullyInside = RobotZones2d.in(shootingZone)
        .robot(robot)
        .footprintFullyInside(/*samplesPerEdge=*/ 3);
```

- **“A specific mechanism point is inside”** (good when the center is misleading)

```java
RobotZone2d shooterPointInside = RobotZones2d.in(shootingZone)
        .robot(robot)
        .pointInside("shooter");
```

Add hysteresis (recommended for safety/gating):

```java
ZoneLatch latch = new ZoneLatch(
        shooterPointInside,
        /*enterSec=*/ 0.10,
        /*exitSec=*/ 0.20
);

boolean inZone = latch.update(loopClock, fieldToRobotPose2d);
```

### Heading predicates (aiming)

The heading APIs are intentionally parallel to the zone APIs.

Example: “Is the shooter control frame pointing at a field point within tolerance?”

```java
RobotHeading2d aimedAtSpeaker = RobotHeadings2d
        .inAimFrame(ControlFrames.robotCenter().robotToAimFrame())
        .toFaceFieldPointInches(48, 72);

HeadingLatch aimLatch = new HeadingLatch(
        aimedAtSpeaker,
        /*enterTolRad=*/ Math.toRadians(2),
        /*exitTolRad=*/ Math.toRadians(4)
);

boolean aimed = aimLatch.update(loopClock, fieldToRobotPose2d);
```

### Combining zone + heading (typical TeleOp “auto-enable” gating)

```java
boolean enableShooting = inZone && aimed;
```

This is intentionally **independent** from any DriveGuidance plan. You can use it with:

- purely manual driving
- a DriveGuidance overlay
- an autonomous task

---

## When to use what

- **Use a DriveGuidancePlan** when you want the framework to generate drive commands.
- **Use RobotZones2d / RobotHeadings2d** when you only want to answer “is it true?” questions.
- **Use DriveGuidanceQuery** when you want to reuse the exact same evaluation logic as a plan,
  but you don’t want to enable the overlay.
