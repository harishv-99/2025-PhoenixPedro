# Drive Guidance

Drive Guidance is Phoenix’s structured way to:

- **create TeleOp assists** such as “aim while I still drive”
- **run the same movement as a Task** in autonomous
- **query the exact same solved errors** for “ready to shoot?” gating and telemetry

It is intentionally the drive-specific public API for spatial control. Phoenix may eventually extract a
more general spatial query layer, but only after a second real non-drive consumer proves out that API.

See also [`Recommended Robot Design`](<../design/Recommended Robot Design.md>) for how guidance fits
into the larger TeleOp + Auto robot architecture.

---

## 1. Spec vs Plan

Drive Guidance intentionally has **two** objects.

### `DriveGuidanceSpec`

A controller-neutral description of:

- the targets (`translateTo`, `aimTo`, or both)
- the control frames (which point on the robot is controlled)
- the available feedback lanes (field pose, live AprilTags, or both)

A spec says **what** you want.  
It contains **no tuning**.

### `DriveGuidancePlan`

An executable bundle:

- `DriveGuidanceSpec` (**what**) +
- `DriveGuidancePlan.Tuning` (**how aggressively to drive the error to zero**)

A plan is what you actually use from robot code:

- `plan.overlay()` for TeleOp assist
- `plan.task(...)` for autonomous execution
- `plan.query()` for readiness checks and telemetry

That split is valuable because one semantic target can often be reused with different tuning in
TeleOp and Auto.

---

## 2. The mental model: references, feedback, guidance

Drive Guidance is easiest to understand if you keep three ideas separate.

#### 2.1 References = where the meaningful thing is

A **reference** is just geometry.

Examples:

- “the speaker aim point”
- “slot 4 on the backdrop”
- “the face of tag 5”
- “a point 6 inches in front of tag 7”

A reference does **not** say what the robot should do. Guidance can later:

- translate to it
- aim at it
- align to its heading
- ask whether the robot is close enough to it

#### 2.2 Feedback = what the robot knows right now

Guidance can solve the same reference from different feedback lanes:

- **field pose** (`fieldPose(...)`)
- **live AprilTags** (`aprilTags(...)`)
- **fixed tag layout metadata** (`fixedTagLayout(...)`)

Those are intentionally separate because they mean different things:

- `fieldPose(...)` = estimated `field -> robot`
- `aprilTags(...)` = current camera observations
- `fixedTagLayout(...)` = static field metadata about which tags are fixed landmarks

#### 2.3 Guidance = turn the solved error into commands

Once guidance knows where the reference is relative to the robot, it can compute:

- translation error
- omega / aim error
- final drive commands

That same solve is reused by overlays, tasks, and queries.

---

## 3. Reference types

The public API exposes two semantic reference shapes:

- `ReferencePoint2d`
- `ReferenceFrame2d`

Create them with `References`.

#### 3.1 Field-fixed references

Use these when the meaningful thing is naturally a field location.

```java
ReferencePoint2d speakerAim = References.fieldPoint(48.0, 24.0);
ReferenceFrame2d backdropFace = References.fieldFrame(48.0, 24.0, Math.PI);
```

A field-fixed frame has:

- an origin point `(x, y)`
- a heading

That heading is the heading of the **reference frame**, not automatically the robot’s desired
heading. Guidance decides how to use it depending on which target builder method you choose.

#### 3.2 Tag-relative references

Use these when the meaningful thing is naturally described relative to a tag.

```java
ReferencePoint2d tagCenter = References.relativeToTagPoint(5, 0.0, 0.0);

ReferenceFrame2d tagFace = References.relativeToTagFrame(
        5,
        /*forward=*/ 6.0,
        /*left=*/ 0.0,
        /*headingRad=*/ Math.PI
);
```

For a tag-relative frame:

- `forward`, `left` define the frame origin in the tag frame
- `headingRad` defines the orientation of the **reference frame’s `+X` axis** in the tag frame

#### 3.3 What does `headingRad = 0` mean?

For `References.relativeToTagFrame(tagId, forward, left, headingRad)`:

- `0` → the reference frame’s `+X` is aligned with the tag’s `+X`
- `+π/2` → the reference frame’s `+X` points tag-left
- `π` → the reference frame’s `+X` points back toward the tag

Phoenix’s tag-frame convention here is:

- `+X` = out from the tag face
- `+Y` = left when looking in `+X`

That heading describes the **reference frame**, not automatically the robot. Later you may choose to:

- aim at the frame origin
- translate to an offset in that frame
- align the robot to the frame heading
- align to the frame heading plus another offset

---

## 4. Targets you can express in a plan

A plan can contain any combination of translation and aim targets.

### 4.1 Translation targets

```java
.translateTo()
    .fieldPointInches(x, y)
    .robotRelativePointInches(forward, left)
    .referencePoint(pointRef)
    .referenceFrameOrigin(frameRef)
    .referenceFrameOffsetInches(frameRef, forward, left)
    .doneTranslateTo()
```

### How to think about them

- `fieldPointInches(...)`  
  Go to this field-fixed point.

- `robotRelativePointInches(...)`  
  Capture an anchor when the plan enables, then move by that delta.

- `referencePoint(...)`  
  Go to the semantic point.

- `referenceFrameOrigin(...)`  
  Go to the origin point of the semantic frame.

- `referenceFrameOffsetInches(frame, fwd, left)`  
  Go to a point expressed in the frame’s coordinates.

That last one is especially useful for close-range alignment. Example:

```java
ReferenceFrame2d slotFace = References.fieldFrame(48.0, 24.0, Math.PI);

.translateTo()
    .referenceFrameOffsetInches(slotFace, -6.0, 0.0)
    .doneTranslateTo()
```

This means: go to a point **6 inches behind the frame’s `+X` axis direction**.

---

### 4.2 Aim targets

```java
.aimTo()
    .fieldPointInches(x, y)
    .fieldHeadingRad(h)
    .referencePoint(pointRef)
    .referenceFrameOrigin(frameRef)
    .referenceFrameOffsetInches(frameRef, forward, left)
    .referenceFrameHeading(frameRef)
    .referenceFrameHeading(frameRef, offsetRad)
    .doneAimTo()
```

### How to think about them

- `fieldPointInches(...)`  
  Turn to face that field point.

- `fieldHeadingRad(...)`  
  Match that absolute field heading.

- `referencePoint(...)`  
  Turn to face the semantic point.

- `referenceFrameOrigin(...)`  
  Turn to face the origin of the frame.

- `referenceFrameOffsetInches(...)`  
  Turn to face an offset point in the frame.

- `referenceFrameHeading(frame)`  
  Match the heading of the frame.

- `referenceFrameHeading(frame, offset)`  
  Match the frame heading plus an additional offset.

Example:

```java
ReferenceFrame2d backdropFace = References.fieldFrame(48.0, 24.0, Math.PI);

.aimTo()
    .referenceFrameHeading(backdropFace)
    .doneAimTo()
```

This means: align the robot’s aim frame to the backdrop frame heading.

---

## 5. Feedback lanes

A plan can use any combination of these lanes:

```java
.feedback()
    .fieldPose(poseEstimator)
    .aprilTags(tagSensor, cameraMount, 0.25)
    .fixedTagLayout(tagLayout)
    .gates(18.0, 30.0, 0.15)
    .preferAprilTagsForOmega(true)
    .lossPolicy(DriveGuidanceSpec.LossPolicy.PASS_THROUGH)
    .doneFeedback()
```

### 5.1 `fieldPose(...)`

Use this when you have a localizer or estimator that yields `field -> robot`.

This is usually the simplest way to solve field-fixed references.

### 5.2 `aprilTags(...)`

Use this when you want guidance to solve directly from the current AprilTag observation.

This lane does **not** require a localizer.

It is especially useful for:

- close-range final alignment
- simple auto-aim
- setups where odometry is weak or unavailable

### 5.3 `fixedTagLayout(...)`

This is static field metadata, not a live sensor.

It enables two important behaviors:

1. **Field-fixed references from AprilTags**  
   If the robot sees a fixed tag and knows where that tag lives on the field, guidance can derive a
   temporary live field pose and solve a field-fixed reference even without a fused localizer.

2. **Localizer fallback for fixed tag-relative references**  
   If you authored a reference relative to a single fixed tag, guidance can promote it into field
   coordinates and keep solving it from the localizer even after the camera loses sight of the tag.

That means you do **not** have to define the same semantic reference twice.

### Example: define once as tag-relative, still get localizer fallback

```java
ReferenceFrame2d slotApproach = References.relativeToTagFrame(
        5,
        /*forward=*/ 6.0,
        /*left=*/ 0.0,
        /*headingRad=*/ Math.PI
);

DriveGuidancePlan plan = DriveGuidance.plan()
        .translateTo()
            .referenceFrameOrigin(slotApproach)
            .doneTranslateTo()
        .aimTo()
            .referenceFrameHeading(slotApproach)
            .doneAimTo()
        .feedback()
            .fieldPose(poseEstimator)
            .aprilTags(tagSensor, cameraMount, 0.25)
            .fixedTagLayout(tagLayout)   // tag 5 is known fixed metadata here
            .doneFeedback()
        .build();
```

If tag 5 is visible, guidance can solve directly from AprilTags.  
If tag 5 is blocked, guidance can still solve the same reference from field pose because it can
promote `tag 5 -> reference` into `field -> reference`.

---

## 6. Adaptive guidance

When both `fieldPose(...)` and `aprilTags(...)` are configured, guidance becomes **adaptive**.

That means:

- translation may use field pose far away, then switch toward live AprilTags near the target
- omega may either follow that same switch, or always prefer AprilTags when valid

### Knobs

```java
.gates(enterRangeInches, exitRangeInches, takeoverBlendSec)
.preferAprilTagsForOmega(true)
```

The default intended mental model is:

- broad move from the localizer
- close-in refinement from live vision

This is usually the right starting point for TeleOp assists and final scoring alignment.

---

## 7. Building specs and plans

### 7.1 Build a spec first

Use this when you want to reuse the same semantic target with different tunings.

```java
ReferenceFrame2d scoringFrame = References.fieldFrame(48.0, 24.0, Math.PI);

DriveGuidanceSpec spec = DriveGuidance.spec()
        .translateTo()
            .referenceFrameOffsetInches(scoringFrame, -6.0, 0.0)
            .doneTranslateTo()
        .aimTo()
            .referenceFrameHeading(scoringFrame)
            .doneAimTo()
        .feedback()
            .fieldPose(poseEstimator)
            .aprilTags(tagSensor, cameraMount, 0.25)
            .fixedTagLayout(tagLayout)
            .doneFeedback()
        .build();
```

Then build plans from the same spec:

```java
DriveGuidancePlan teleOpPlan = DriveGuidance.plan(spec)
        .tuning(DriveGuidancePlan.Tuning.defaults()
                .withTranslateKp(0.04)
                .withAimKp(1.6))
        .build();

DriveGuidancePlan autoPlan = DriveGuidance.plan(spec)
        .tuning(DriveGuidancePlan.Tuning.defaults()
                .withTranslateKp(0.08)
                .withAimKp(2.2))
        .build();
```

### 7.2 Build a plan in one shot

Use this when you do not need to reuse the spec separately.

```java
ReferencePoint2d scoringAim = References.relativeToTagPoint(5, 0.0, 0.0);

DriveGuidancePlan aimPlan = DriveGuidance.plan()
        .aimTo()
            .referencePoint(scoringAim)
            .doneAimTo()
        .feedback()
            .aprilTags(tagSensor, cameraMount, 0.25)
            .lossPolicy(DriveGuidanceSpec.LossPolicy.PASS_THROUGH)
            .doneFeedback()
        .build();
```

---

## 8. Using a plan

### 8.1 TeleOp overlay

```java
DriveSource driveWithAim = baseDrive.overlayWhen(
        gamepads.p1().leftBumper(),
        aimPlan.overlay(),
        DriveOverlayMask.OMEGA_ONLY
);
```

That means the driver keeps translation while guidance owns omega.

### 8.2 Autonomous task

```java
DriveGuidanceTask.Config cfg = new DriveGuidanceTask.Config();
cfg.timeoutSec = 2.0;
cfg.positionTolInches = 1.0;
cfg.headingTolRad = Math.toRadians(2.0);

Task alignTask = plan.task(drivebase, cfg);
```

This is why the reference model matters: autonomous code uses the same semantic targets and
the same solve logic as TeleOp.

### 8.3 Query / gating

```java
DriveGuidanceQuery q = plan.query();
q.reset();

DriveGuidanceStatus s = q.sample(clock);
boolean okToScore =
        s.translationWithin(2.0)
        && s.omegaWithin(Math.toRadians(3.0));
```

Because the query reuses the same guidance engine, your gating logic stays aligned with what the
overlay or task would actually command.

---

## 9. Practical patterns

### 9.1 Auto-aim only

```java
ReferencePoint2d speakerAim = References.relativeToTagPoint(5, 0.0, 0.0);

DriveGuidancePlan aimPlan = DriveGuidance.plan()
        .aimTo()
            .referencePoint(speakerAim)
            .doneAimTo()
        .feedback()
            .aprilTags(tagSensor, cameraMount, 0.25)
            .doneFeedback()
        .build();
```

Use this when the driver still owns translation.

### 9.2 Final settle at a frame

```java
ReferenceFrame2d slotFace = References.fieldFrame(48.0, 24.0, Math.PI);

DriveGuidancePlan settle = DriveGuidance.plan()
        .translateTo()
            .referenceFrameOffsetInches(slotFace, -6.0, 0.0)
            .doneTranslateTo()
        .aimTo()
            .referenceFrameHeading(slotFace)
            .doneAimTo()
        .feedback()
            .fieldPose(poseEstimator)
            .aprilTags(tagSensor, cameraMount, 0.25)
            .fixedTagLayout(tagLayout)
            .doneFeedback()
        .build();
```

Use this for “last few inches” scoring alignment.

### 9.3 Hold position and only turn

```java
DriveGuidancePlan braceAndAim = DriveGuidance.plan()
        .translateTo()
            .robotRelativePointInches(0.0, 0.0)
            .doneTranslateTo()
        .aimTo()
            .fieldHeadingRad(Math.PI / 2.0)
            .doneAimTo()
        .feedback()
            .fieldPose(poseEstimator)
            .doneFeedback()
        .build();
```

The robot captures its position when the plan enables, then only tries to keep that spot while
turning to the heading.

---

## 10. When to use Drive Guidance vs lighter spatial tools

Use **Drive Guidance** when you want Phoenix to generate drive commands.

Use lighter-weight spatial tools when you only need a yes/no or scalar answer, such as:

- “am I inside the zone?”
- “is the intake point inside the region?”
- “am I aimed enough to shoot?”

That keeps the system clean:

1. spatial math
2. spatial predicates
3. controllers

Drive Guidance lives in layer (3).

---

## 11. Debugging tips

When guidance does not behave as expected, check these in order:

1. **Reference authoring**  
   Is the semantic point/frame what you intended?

2. **Feedback lane availability**  
   Did you actually configure the lane the target needs?
   - field-fixed references typically want `fieldPose(...)`, or `aprilTags(...) + fixedTagLayout(...)`
   - tag-relative references want `aprilTags(...)`
   - single fixed tag-relative references can also fall back through `fieldPose(...) + fixedTagLayout(...)`

3. **Tag-relative frame heading meaning**  
   Remember: `0` means the frame’s `+X` aligns with the tag’s `+X` (out from the face).

4. **Query status**  
   Sample `DriveGuidanceStatus` and inspect:
   - `mode`
   - `hasTranslationError`
   - `hasOmegaError`
   - `translationErrorMagInches()`
   - `omegaErrorRad`
   - `aprilTagsInRangeForTranslation`
   - blend factors

Those values usually make the chosen solve path obvious.

---

## 12. Summary

The most important habit is this:

> Define the meaningful thing once as a semantic reference, then let guidance decide whether to
> solve it from field pose, live AprilTags, or both.

That keeps:

- TeleOp assist
- autonomous movement
- readiness gating
- telemetry

all speaking the same language.
