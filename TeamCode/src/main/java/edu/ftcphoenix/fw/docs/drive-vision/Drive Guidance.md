# Drive Guidance

Drive Guidance is Phoenix's reference-first drive assist and autonomous motion system.

It is built around one idea:

> **Author the meaningful geometry once, then choose how guidance is allowed to solve it.**

That leads to a clean split:

- **References** describe geometry: field points, field frames, single-tag-relative points/frames, or selected-tag-relative points/frames.
- **Resolve lanes** describe solving: localization, live AprilTags, or explicit adaptive use of both.
- **Loss behavior** describes output policy when the requested channel cannot be solved this loop.

This document reflects the current API:

- `resolveWith()` replaces the older `feedback()` naming.
- `fixedAprilTagLayout(...)` replaces `fixedTagLayout(...)`.
- `onLoss(...)` replaces `lossPolicy(...)`.
- Guidance APIs use **radians only**.
- Raw multi-tag references were removed. Multi-tag intent now goes through an explicit `TagSelectionSource`.

---

## 1. Core principles

### 1.1 Geometry first

A reference is about **what location/heading is meaningful**, not about which sensor will be used.

Examples:

- `References.fieldPoint(...)`
- `References.fieldFrame(...)`
- `References.relativeToTagPoint(tagId, ...)`
- `References.relativeToSelectedTagPoint(selection, ...)`

The exact same reference can be:

- translated to,
- aimed at,
- sampled by a query,
- or reused in multiple plans.

### 1.2 One tag at a time

References never mean “many tags at once.” A reference is always resolved through **one active tag** at evaluation time.

That tag may come from:

- a fixed tag ID authored directly in the reference, or
- a shared `TagSelectionSource`.

This keeps the geometry model simple and inspectable.

### 1.3 Raw detections and selection are different layers

`AprilTagSensor` now returns **all detections from one frame** as `AprilTagDetections`.

That raw boundary is intentionally separate from selection:

- **detections** are for localization / estimation / telemetry,
- **selection** is for “which tag are we currently talking about?”

### 1.4 Selection is shared

When a robot means the same semantic target family in multiple places, it should build **one selector** and reuse it for:

- drive guidance,
- telemetry,
- shooter logic,
- gating,
- and mechanism decisions.

That avoids duplicated “best tag” logic.

### 1.5 Solve lanes are explicit

The solve mode should be obvious at the call site:

- `localizationOnly()`
- `aprilTagsOnly()`
- `adaptive()`

Mixed-lane behavior is powerful, so it should read as deliberate.

### 1.6 `onLoss(...)` is runtime output behavior

If guidance cannot solve the requested channel this loop:

- `PASS_THROUGH` leaves that channel to the base source (great for TeleOp overlays)
- `ZERO_OUTPUT` actively commands zero on that channel

Loss behavior is separate from both geometry and selection.

---

## 2. The builder shape

```java
DriveGuidance.plan() / DriveGuidance.spec()
    .translateTo()
        .fieldPointInches(...)
        .robotRelativePointInches(...)
        .point(ReferencePoint2d)
        .doneTranslateTo()

    .aimTo()
        .fieldPointInches(...)
        .fieldHeadingRad(...)
        .point(ReferencePoint2d)
        .frameHeading(ReferenceFrame2d)
        .frameHeading(ReferenceFrame2d, headingOffsetRad)
        .doneAimTo()

    .resolveWith()
        .localizationOnly() / .aprilTagsOnly() / .adaptive()
        .localization(...)
        .aprilTags(...)
        .fixedAprilTagLayout(...)
        .translationTakeover(...)
        .omegaPolicy(...)
        .onLoss(...)
        .doneResolveWith()

    .controlFrames(...)
    .tuning(...)      // plan only
    .build()
```

### Why is `.aimTo()` not flattened?

Because `aimTo()` still has more than one real family inside it:

- aim at a point,
- or match a heading.

Keeping the outer “channel selector” (`translateTo`, `aimTo`, `resolveWith`) and the inner “how” choice makes the structure parallel and readable.

---

## 3. References

### 3.1 Field-fixed references

```java
ReferencePoint2d speakerAim = References.fieldPoint(48.0, 24.0);
ReferenceFrame2d slotFace = References.fieldFrame(60.0, -18.0, Math.PI);
```

### 3.2 Single-tag-relative references

```java
ReferencePoint2d tagCenter = References.relativeToTagPoint(5, 0.0, 0.0);
ReferenceFrame2d slotApproach = References.relativeToTagFrame(5, -6.0, 0.0, Math.PI);
```

### 3.3 Points derived from frames

`References` now owns the geometry helpers that turn frames into points.

```java
ReferenceFrame2d slotFace = References.fieldFrame(60.0, -18.0, Math.PI);
ReferencePoint2d settlePoint = References.framePoint(slotFace, -6.0, 0.0);
```

That keeps frame-origin/frame-offset point construction in one place instead of duplicating it in the builder.

### 3.4 Selected-tag-relative references

For multi-tag workflows, build a shared selector and author the reference relative to that selector.

```java
TagSelectionSource basketSelection = ...;
ReferencePoint2d basketAim = References.relativeToSelectedTagPoint(basketSelection, 0.0, 0.0);
```

The reference object itself is still immutable. It resolves through the selector's current selected tag at evaluation time.

### 3.5 Per-tag offsets with one selector

This is useful when different tags represent the same semantic goal but need different geometry.

```java
Map<Integer, References.TagPointOffset> offsets = new LinkedHashMap<Integer, References.TagPointOffset>();
offsets.put(20, References.pointOffset(-8.0,  5.5));
offsets.put(24, References.pointOffset(-8.0, -5.5));

ReferencePoint2d basketAim = References.relativeToSelectedTagPoint(basketSelection, offsets);
```

This is the right tool for “same scoring concept, mirrored tag geometry.”

---

## 4. Raw AprilTag detections and selection

### 4.1 Raw detections

`AprilTagSensor` is a memoized `Source<AprilTagDetections>`.

```java
AprilTagSensor tags = Tags.aprilTags(hardwareMap, "Webcam 1");
AprilTagDetections dets = tags.get(clock);
```

One sensor instance may be safely shared across:

- telemetry,
- selection,
- localization,
- and guidance.

### 4.2 Tag selection

A `TagSelectionSource` answers:

- which tag would win now?
- which tag is currently selected?
- is that selection latched?
- is the selected tag still freshly visible?
- why did it win?

```java
TagSelectionSource basketSelection =
        TagSelections.from(tags)
                .among(BASKET_TAG_IDS)
                .freshWithin(0.25)
                .choose(TagSelectionPolicies.smallestAbsRobotBearing(cameraMount))
                .stickyWhen(gamepads.p2().leftBumper())
                .reacquireAfterLoss(0.20)
                .build();
```

### 4.3 Preview vs selection

Selectors intentionally expose two ideas:

- **preview**: which tag would win right now?
- **selection**: which tag are we currently committed to?

That matters for TeleOp:

- preview while the driver is lining up,
- latch when aim-assist is enabled,
- optionally reacquire after a real loss.

### 4.4 When to use sticky selection

**TeleOp assist:**

- preview while idle
- sticky while the aim button is held
- maybe reacquire after a short loss timeout

**Autonomous:**

- usually latch once at task start or selector reset
- avoid bouncing between tags mid-maneuver

---

## 5. `resolveWith()`

### 5.1 Localization only

Use this when the target is solvable from a field pose estimate.

```java
DriveGuidancePlan holdHeading = DriveGuidance.plan()
        .aimTo()
            .fieldHeadingRad(Math.PI / 2.0)
            .doneAimTo()
        .resolveWith()
            .localizationOnly()
            .localization(poseEstimator)
            .doneResolveWith()
        .build();
```

Localization-only can also solve:

- field-fixed references,
- robot-relative translation anchors,
- single fixed-tag-relative references **when** `fixedAprilTagLayout(...)` is provided,
- selected-tag-relative references **after** a selected tag exists and that tag is known in the fixed layout.

### 5.2 AprilTags only

Use this when the target should be solved directly from fresh live tag observations.

```java
DriveGuidancePlan aimAtTag = DriveGuidance.plan()
        .aimTo()
            .point(References.relativeToTagPoint(5, 0.0, 0.0))
            .doneAimTo()
        .resolveWith()
            .aprilTagsOnly()
            .aprilTags(tags, cameraMount, 0.25)
            .onLoss(DriveGuidanceSpec.LossPolicy.PASS_THROUGH)
            .doneResolveWith()
        .build();
```

Field-fixed references can also be solved in the AprilTags lane when you provide `fixedAprilTagLayout(...)`.

### 5.3 Adaptive

Use this when both lanes are intentionally available and you want the arbitration to be explicit.

```java
DriveGuidancePlan adaptiveAlign = DriveGuidance.plan()
        .translateTo()
            .point(References.framePoint(slotFace, -6.0, 0.0))
            .doneTranslateTo()
        .aimTo()
            .frameHeading(slotFace)
            .doneAimTo()
        .resolveWith()
            .adaptive()
            .localization(fusedPoseEstimator, 0.25, 0.0)
            .aprilTags(tags, cameraMount, 0.25)
            .fixedAprilTagLayout(gameTagLayout)
            .translationTakeover(72.0, 84.0, 0.25)
            .omegaPolicy(DriveGuidanceSpec.OmegaPolicy.PREFER_APRIL_TAGS_WHEN_VALID)
            .onLoss(DriveGuidanceSpec.LossPolicy.PASS_THROUGH)
            .doneResolveWith()
        .build();
```

Adaptive mode must be structurally meaningful:

- both lanes must be configured,
- every requested channel must be solvable by at least one lane,
- and at least one requested channel should genuinely be dual-capable.

---

## 6. Early validation rules

Drive Guidance now validates more of the geometry/solver mismatch at build time.

Examples of build-time failures:

- `localizationOnly()` with no `localization(...)`
- `aprilTagsOnly()` with no `aprilTags(...)`
- `adaptive()` without both lanes
- selected-tag or fixed-tag references in localization mode with no `fixedAprilTagLayout(...)`
- field-fixed references in AprilTags-only mode with no `fixedAprilTagLayout(...)`
- adaptive-only knobs configured in a single-lane plan

At runtime, dynamic loss is reported through status instead of exploding control flow.

---

## 7. Status, queries, and telemetry

`DriveGuidanceQuery` samples the exact same solver/controller math used by the overlay or task.

```java
DriveGuidanceStatus status = aimPlan.query().sample(clock, DriveOverlayMask.OMEGA_ONLY);
```

Useful status fields include:

- `translationSource`
- `omegaSource`
- `translationSelection`
- `aimSelection`
- `translationErrorInches`
- `omegaErrorRad`

This makes it easy to answer questions like:

- are we aimed enough?
- which tag is guidance currently using?
- is that selected tag still visible?
- are we running from localization or live AprilTags right now?

Example telemetry:

```java
DriveGuidanceStatus status = aimQuery.sample(clock, DriveOverlayMask.OMEGA_ONLY);
TagSelectionResult sel = status != null ? status.aimSelection : TagSelectionResult.none(Collections.<Integer>emptySet());

telemetry.addData("aim.ready", status != null && status.omegaWithin(0.03));
telemetry.addData("aim.tag", sel.hasSelection ? sel.selectedTagId : "none");
telemetry.addData("aim.visibleNow", sel.hasFreshSelectedObservation);
telemetry.addData("aim.reason", sel.reason);
```

---

## 8. Common use-cases

### 8.1 Aim at a single fixed AprilTag center in TeleOp

```java
DriveGuidancePlan aimPlan = DriveGuidance.plan()
        .aimTo()
            .point(References.relativeToTagPoint(5, 0.0, 0.0))
            .doneAimTo()
        .resolveWith()
            .aprilTagsOnly()
            .aprilTags(tags, cameraMount, 0.25)
            .onLoss(DriveGuidanceSpec.LossPolicy.PASS_THROUGH)
            .doneResolveWith()
        .build();
```

### 8.2 Aim at whichever scoring tag the robot is most directly facing

```java
TagSelectionSource scoringSelection =
        TagSelections.from(tags)
                .among(SCORING_TAG_IDS)
                .freshWithin(0.25)
                .choose(TagSelectionPolicies.smallestAbsRobotBearing(cameraMount))
                .stickyWhen(gamepads.p2().leftBumper())
                .build();

DriveGuidancePlan aimPlan = DriveGuidance.plan()
        .aimTo()
            .point(References.relativeToSelectedTagPoint(scoringSelection, 0.0, 0.0))
            .doneAimTo()
        .resolveWith()
            .aprilTagsOnly()
            .aprilTags(tags, cameraMount, 0.25)
            .onLoss(DriveGuidanceSpec.LossPolicy.PASS_THROUGH)
            .doneResolveWith()
        .build();
```

### 8.3 Same selector, different offset by tag ID

```java
Map<Integer, References.TagPointOffset> offsets = new LinkedHashMap<Integer, References.TagPointOffset>();
offsets.put(20, References.pointOffset(-8.0,  5.5));
offsets.put(24, References.pointOffset(-8.0, -5.5));

ReferencePoint2d basketAim = References.relativeToSelectedTagPoint(scoringSelection, offsets);
```

### 8.4 Tag-relative autonomous solved from localization

```java
ReferenceFrame2d slotApproach = References.relativeToTagFrame(5, -6.0, 0.0, Math.PI);

DriveGuidancePlan plan = DriveGuidance.plan()
        .translateTo()
            .point(References.framePoint(slotApproach))
            .doneTranslateTo()
        .aimTo()
            .frameHeading(slotApproach)
            .doneAimTo()
        .resolveWith()
            .localizationOnly()
            .localization(poseEstimator)
            .fixedAprilTagLayout(gameTagLayout)
            .doneResolveWith()
        .build();
```

### 8.5 Selected-tag guidance with localization fallback after acquisition

```java
DriveGuidancePlan plan = DriveGuidance.plan()
        .aimTo()
            .point(References.relativeToSelectedTagPoint(scoringSelection, offsets))
            .doneAimTo()
        .resolveWith()
            .adaptive()
            .localization(fusedPoseEstimator, 0.25, 0.0)
            .aprilTags(tags, cameraMount, 0.25)
            .fixedAprilTagLayout(gameTagLayout)
            .onLoss(DriveGuidanceSpec.LossPolicy.PASS_THROUGH)
            .doneResolveWith()
        .build();
```

This is the important mental model:

- the selected tag identity can remain sticky,
- live AprilTag solving can temporarily disappear,
- localization can keep solving relative to the same selected tag.

---

## 9. Practical guidance

### Prefer one shared selector per semantic target family

If the robot has one concept of “basket target,” build one selector and reuse it.

### Keep references semantic

Prefer `References.framePoint(...)`, `References.relativeToTagPoint(...)`, and selected-tag references over sprinkling raw offsets and special cases across robot code.

### Use radians at the API boundary

Phoenix guidance APIs are radians-only. Convert degrees only when reading human-facing config or emitting telemetry.

### Keep `onLoss(PASS_THROUGH)` for TeleOp overlays unless you have a reason not to

That gives the driver smooth manual fallback whenever the assist cannot solve the requested channel.

---

## 10. Summary

Drive Guidance is easiest to reason about when you keep these three questions separate:

1. **What is the geometry?** → references
2. **How may guidance solve it?** → `resolveWith()`
3. **What should output do on a bad loop?** → `onLoss(...)`

And for AprilTag-heavy robots, add one more:

4. **Which tag are we currently talking about?** → shared `TagSelectionSource`
