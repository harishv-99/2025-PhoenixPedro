# Shooter Case Study & Examples Walkthrough

This walkthrough explains the current shooter + auto-aim examples using the selector-based AprilTag model.

It covers the two example OpModes:

- `TeleOp_05_ShooterTagAimVision`
- `TeleOp_06_ShooterTagAimMacroVision`

The most important update is this:

> **the robot no longer relies on a hidden “best tag” helper.**
>
> Instead, raw camera detections stay available, and a shared `TagSelectionSource` is used anywhere the code needs one semantic answer to “which scoring tag do we mean right now?”

---

## 1. The architecture in one sentence

The examples share one selector across:

- drive auto-aim,
- telemetry,
- and shooter velocity-from-range logic.

That keeps all three systems talking about the same tag.

---

## 2. Why selection is separate from detections

The AprilTag camera boundary returns raw `AprilTagDetections` from one processed frame.

That is useful because:

- localization can use multiple fixed tags from the same frame,
- telemetry can show all visible IDs,
- selection can still choose one tag for aim-assist.

The examples then build a selector like this:

```java
TagSelectionSource scoringSelection =
        TagSelections.from(tagSensor)
                .among(SCORING_TAG_IDS)
                .freshWithin(MAX_TAG_AGE_SEC)
                .choose(TagSelectionPolicies.smallestAbsRobotBearing(cameraMount))
                .stickyWhen(gamepads.p1().leftBumper())
                .build();
```

That means:

- before the driver holds aim, the selector previews the likely winner,
- when aim is enabled, the selector latches the tag the robot is most directly facing,
- aim, shooter range, and telemetry all reuse that one decision.

---

## 3. Example 05: shooter + aim assist + vision distance

### 3.1 What it demonstrates

Example 05 is intentionally simple:

- the driver still translates manually,
- the guidance overlay owns only omega,
- shooter velocity comes from the selected tag range.

### 3.2 The guidance plan

```java
ReferencePoint2d scoringRef = References.relativeToSelectedTagPoint(scoringSelection, 0.0, 0.0);

DriveGuidancePlan aimPlan = DriveGuidance.plan()
        .faceTo()
            .point(scoringRef)
            .doneFaceTo()
        .resolveWith()
            .aprilTagsOnly()
            .aprilTags(tagSensor, cameraMount, MAX_TAG_AGE_SEC)
            .onLoss(DriveGuidanceSpec.LossPolicy.PASS_THROUGH)
            .doneResolveWith()
        .build();
```

That reads honestly:

- geometry: aim at the selected scoring tag center
- solving: use live AprilTags only
- loss behavior: if the solve disappears, let manual omega pass through

### 3.3 Why `PASS_THROUGH` matters

This is a TeleOp overlay, so if the tag disappears briefly:

- the driver still keeps manual control,
- the overlay does not freeze the robot,
- the assist just stops overriding omega until it can solve again.

### 3.4 Shooter velocity from the same selection

Instead of doing a separate camera lookup, the example reads the fresh selected observation:

```java
TagSelectionResult selection = scoringSelection.get(clock);
AprilTagObservation obs = selection.hasFreshSelectedObservation
        ? selection.selectedObservation
        : AprilTagObservation.noTarget(Double.POSITIVE_INFINITY);
```

That avoids duplicated “which tag?” logic.

---

## 4. Example 06: macro + aim assist + vision distance

Example 06 adds one more idea: a non-blocking shooting macro.

The macro still uses the same shared selection source.

When the driver presses the macro button:

1. read the currently selected fresh observation,
2. choose shooter velocity from the range table,
3. enqueue the macro task sequence.

That keeps the decision boundary clean:

- **selection** decides which tag is relevant,
- **shooter calibration** decides what velocity matches the range,
- **tasks** decide the feed sequence over time.

---

## 5. Why sticky selection matters for drivers

Without stickiness, aim assist can flap between two tags while the robot sweeps across the field.

With sticky selection:

- the driver points generally toward the target family,
- presses/holds aim,
- the robot commits to one tag,
- and keeps using that same tag while the assist stays active.

That makes the robot feel intentional instead of “twitchy.”

---

## 6. Per-tag offsets for mirrored goals

Sometimes the semantic goal is “the basket behind whichever scoring tag is relevant,” but the correct lateral offset differs by tag ID.

That is why selected-tag references support per-tag maps:

```java
Map<Integer, References.TagPointOffset> offsets = new LinkedHashMap<Integer, References.TagPointOffset>();
offsets.put(20, References.pointOffset(-8.0,  5.5));
offsets.put(24, References.pointOffset(-8.0, -5.5));

ReferencePoint2d basketAim = References.relativeToSelectedTagPoint(scoringSelection, offsets);
```

This is cleaner than splitting the whole behavior into one plan per tag just to swap the offset.

---

## 7. Telemetry you should actually show

The examples should surface the selection state directly to the driver:

- selected tag ID
- whether the selected tag is still freshly visible
- range
- bearing
- aim-ready state

A good minimal telemetry set is:

```java
TagSelectionResult sel = scoringSelection.get(clock);
telemetry.addData("tag.selected", sel.hasSelection ? sel.selectedTagId : "none");
telemetry.addData("tag.visibleNow", sel.hasFreshSelectedObservation);
telemetry.addData("tag.reason", sel.reason);
telemetry.addData("aim.ready", aimReady.getAsBoolean(clock));
```

That tells the driver and tuner the same thing the robot is actually using.

---

## 8. Idempotence and loop behavior

These examples still rely on Phoenix's one-loop heartbeat model.

Important consequence:

- `Bindings.update(clock)` is cycle-idempotent
- `TaskRunner.update(clock)` is cycle-idempotent
- `AprilTagSensor.get(clock)` is cycle-idempotent
- `TagSelectionSource.get(clock)` is cycle-idempotent

So you can safely share the same selector across drive logic, shooter logic, and telemetry in one loop.

---

## 9. What changed from the older design

Older versions often used a helper that silently picked the “best” tag.

The current design deliberately avoids that hidden policy.

Now the robot code states:

- where detections come from,
- which selection policy is used,
- when the tag becomes sticky,
- and how guidance behaves on loss.

That makes the examples easier to understand and much easier to debug.

---

## 10. Extension ideas

These examples are a good base for several future directions:

### Add localization fallback to aim assist

If the robot maintains a trusted field pose estimate, the same selected-tag reference can use:

```java
.resolveWith()
    .adaptive()
    .localization(fusedAbsolutePoseEstimator, 0.25, 0.0)
    .aprilTags(tagSensor, cameraMount, 0.25)
    .fixedAprilTagLayout(gameTagLayout)
```

That allows the robot to keep solving the same selected-tag-relative target even when the camera is briefly blocked.

### Use the selector for shooter gating

You can require:

- a selection exists,
- the selected observation is fresh,
- and guidance reports `omegaWithin(...)`

before feeding.

### Replace the selection policy

For some robots, `smallestAbsRobotBearing(cameraMount)` feels best.
For others, `closestRange()` or `priorityOrder(...)` is more appropriate.

The selector abstraction makes that choice explicit.
