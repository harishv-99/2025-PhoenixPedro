# AprilTag Localization & Fixed Layouts

This guide explains the framework policy for AprilTag localization, fixed-tag layouts, and the shared multi-tag field-pose solver.

The short version:

- **The FTC SDK AprilTag library** tells the detector which tags exist and what their printed size is.
- **A Phoenix `TagLayout`** tells localization / guidance which tags are safe to treat as **fixed field landmarks**.
- **Raw detections may see more tags than the fixed layout contains.** That is intentional.

---

## 1. Detectable tags vs fixed field tags

These are not the same thing.

### 1.1 Detectable tags

The FTC SDK `AprilTagLibrary` is detection metadata:

- tag IDs
- tag size
- FTC-provided field metadata when available

Phoenix does **not** assume every tag in that library is safe for global localization. Some seasons include tags that are useful for identification, scoring logic, or driver assists even though their exact placement is not deterministic enough to trust as a fixed field landmark.

### 1.2 Fixed field tags

A Phoenix `TagLayout` is the framework contract for **field-fixed** tags only.

That means anything that promotes AprilTags into a field pose should use a `TagLayout`, not the raw FTC library. In practice that includes:

- `TagOnlyPoseEstimator`
- drive guidance's temporary AprilTag → field-pose bridge
- any localizer or tool that solves `field -> robot` from visible tags

For official FTC games, the intended entrypoint is:

```java
TagLayout fixedLayout = FtcGameTagLayout.currentGameFieldFixed();
```

That keeps the season-specific “which tags are really fixed?” decision in one framework-owned place.

The helper may also normalize away known FTC SDK ancillary/sample tags that are present in a broad detector library but are not part of the official field-fixed policy. That way localization policy does not break just because the detector library contains extra SDK sample IDs.

---

## 2. Official FTC games

`FtcGameTagLayout.currentGameFieldFixed()` is the framework's best-known fixed-tag policy for the current FTC season.

Design rules:

- it should contain only tags Phoenix is willing to trust for localization / field-pose solving
- it should fail fast for unknown future official libraries rather than silently treating every SDK tag as fixed
- it should tolerate known ancillary SDK sample tags in broad/default detector libraries instead of forcing robot code to special-case them
- robot code should not carry its own season-specific “exclude these IDs” list when the framework policy is always required

If you intentionally work with an archived or custom library, use:

```java
FtcGameTagLayout.officialGameFieldFixed(library);
FtcGameTagLayout.fromLibraryFixedIds(library, ids);
FtcGameTagLayout.fromLibraryAllTags(library);   // escape hatch only
```

Use `fromLibraryAllTags(...)` only when you already know every tag in that library is truly fixed in the environment you are solving against.

---

## 3. Multi-tag localization

Phoenix's AprilTag localizer uses a shared solver:

- gather all visible observations whose IDs are in the fixed layout
- compute one candidate `field -> robot` pose per visible fixed tag
- weight closer / more centered tags more strongly
- prefer the FTC SDK's per-detection robot pose when it agrees with Phoenix's explicit geometry and remains plausible
- choose a consensus seed
- reject outliers
- when multiple fixed tags are present, require a configurable majority of candidate weight to agree before returning a pose
- compute one fused field pose and a quality score

The framework uses that same solver in both:

- `TagOnlyPoseEstimator`
- the Drive Guidance AprilTag lane when it temporarily needs a field pose

That keeps guidance and localization from drifting into two subtly different AprilTag policies.

The solver's quality score also reflects how much of the visible candidate set actually agreed
with the final consensus, and the solver can now reject a contradictory multi-tag frame outright if
too little of the candidate weight survives the consensus gate. That keeps a lone surviving tag
from masquerading as a trustworthy multi-tag solve when the rest of the frame strongly disagrees.

---

## 4. Sharing solver tuning cleanly

`TagOnlyPoseEstimator.Config` extends the shared fixed-tag solver config and adds the camera mount.

When another component should reuse the same weighting / outlier / plausibility policy, use:

```java
TagOnlyPoseEstimator.Config tagCfg = RobotConfig.Localization.aprilTags.copy()
        .withCameraMount(cameraMount);

TagOnlyPoseEstimator tagLocalizer = new TagOnlyPoseEstimator(tags, fixedLayout, tagCfg);

DriveGuidancePlan plan = DriveGuidance.plan()
        .resolveWith()
            .adaptive()
            .localization(fusedLocalizer)
            .aprilTags(tags, cameraMount, 0.50)
            .fixedAprilTagLayout(fixedLayout)
            .aprilTagFieldPoseConfig(tagCfg.toSolverConfig())
            .doneResolveWith()
        .build();
```

That is the intended way to keep the AprilTag-only localizer and the guidance AprilTag bridge aligned without passing camera-mount-only fields into unrelated APIs.

The guidance API also normalizes its solver-config input defensively at the boundary, so even if a caller accidentally passes a richer config subtype, only the shared fixed-tag solver fields are retained.

One intentional split remains:

- the shared solver config owns multi-tag weighting / consensus / plausibility policy
- `TagOnlyPoseEstimator.Config` adds localizer-specific concepts such as the camera mount and how strongly pose quality decays as a detections frame ages toward its freshness limit

---

## 5. Role-specific fixed-tag subsets

Often one robot wants different fixed-tag views for different jobs:

- **global localization**: all fixed tags
- **scoring alignment**: only scoring tags
- **a tester or practice routine**: only one wall or one rig

Use framework-owned subset views instead of duplicating tag metadata or repeating robot-side filtering logic:

```java
TagLayout fullFixed = FtcGameTagLayout.currentGameFieldFixed();
TagLayout scoringFixed = TagLayouts.subset(fullFixed, SCORING_TAG_IDS, "scoring tags");
```

The subset reuses the same underlying field metadata; it only narrows which fixed IDs a consumer is allowed to see.

---

## 6. Plausibility gating

The shared solver supports an optional field-region plausibility gate.

Use this when a team wants to reject obviously impossible AprilTag field solves before they perturb localization:

```java
TagOnlyPoseEstimator.Config cfg = TagOnlyPoseEstimator.Config.defaults()
        .withCameraMount(cameraMount)
        .withPlausibleFieldRegion(FtcFieldRegions.fullField());

cfg.maxOutsidePlausibleFieldRegionInches = 3.0;
```

Notes:

- this is a **reliability gate**, not a full rules engine
- a small outside tolerance is often useful near walls and corners
- if the FTC SDK per-detection robot pose is implausible but Phoenix's explicit geometry solve is plausible, the shared solver now falls back to the geometry solve instead of throwing the whole observation away

---

## 7. Pinpoint + AprilTag fusion and latency compensation

When a robot uses odometry plus AprilTag global solves, Phoenix's fusion localizer now does two related reliability jobs:

- deduplicate repeated camera frames by measurement timestamp
- when history is available, apply the AprilTag correction at the frame timestamp and replay odometry forward to the current loop

That is more trustworthy than repeatedly blending the same delayed frame against “now”.

Typical pattern:

```java
TagOnlyPoseEstimator.Config tagCfg = RobotConfig.Localization.aprilTags.copy()
        .withCameraMount(cameraMount);

TagOnlyPoseEstimator tagLocalizer = new TagOnlyPoseEstimator(tags, fixedLayout, tagCfg);

OdometryTagFusionPoseEstimator.Config fusionCfg = RobotConfig.Localization.pinpointAprilTagFusion.copy();
fusionCfg.maxVisionAgeSec = 0.35;
fusionCfg.odomHistorySec = 1.0;   // must cover maxVisionAgeSec when latency compensation is enabled

PoseEstimator fused = new OdometryTagFusionPoseEstimator(pinpoint, tagLocalizer, fusionCfg);
```

`TagOnlyPoseEstimator` also now ages its own quality down as the detections frame approaches
`maxDetectionAgeSec`, so a barely-fresh frame can still be used when appropriate without looking as
trustworthy as a brand-new multi-tag solve.

Notes:

- `odomHistorySec` should be at least `maxVisionAgeSec`; the config now validates this fail-fast.
- replay only uses measurements newer than the current replay base (initialization, manual reset, or last accepted correction).
- if exact replay is unavailable, the estimator falls back to a simpler projected-now correction rather than silently re-applying the same stale frame every loop.
- the fused estimator's reported quality now respects the quality of the most recently accepted AprilTag correction instead of giving every fresh correction the same confidence boost.
- manual `setPose(...)` calls are treated as manual anchors, so they clear the "recent accepted vision" hold rather than pretending that a real camera correction just happened.
- if the fused pose is pushed back into odometry, the replay base and odometry history are rebased at that corrected pose.

---

## 8. Selected-tag references and localization

Selected-tag references are still **single-tag semantic references**.

That means:

- a selector may preview or choose among many candidate IDs
- but a reference resolves through exactly **one selected tag at a time**
- the multi-tag fusion belongs in localization, not inside the reference object

Phoenix currently uses a conservative localization policy for selected-tag references:

> A selected-tag reference is considered localizable only when **all candidate IDs** that selector may choose are present in the fixed layout.

Why the framework currently prefers that rule:

- build-time validation stays static and honest
- readiness semantics stay predictable
- guidance does not change from “localizable” to “not localizable” merely because the selector changed its mind this loop

A future runtime policy could allow localization whenever the **currently selected** tag is fixed, but that is a deeper contract change and should only be added if a real use case appears.

---

## 9. Testers and calibration tools

The localization testers now support using the same AprilTag-localizer config as production robot code.

That matters because:

- solver tuning influences reliability
- plausibility gates influence whether a measurement is accepted
- a tester should ideally reflect the same assumptions the robot uses in TeleOp

For official-game layouts, the localization testers also surface the fixed-tag policy summary and source-library IDs so it is easy to confirm which IDs are trusted, which official-game IDs are intentionally excluded, and whether ancillary SDK tags are present in the detector library.

If you want that same summary elsewhere, use `FtcTagLayoutDebug.dumpSummary(...)`. It prints a compact
framework-owned view of the included IDs and, when available, the source FTC library IDs plus the
IDs intentionally excluded from the fixed layout.

---

## 10. Reliability checklist

If AprilTag localization feels worse than expected, check these in order:

1. camera mount calibration (`robot -> camera`)
2. odometry calibration / pod offsets
3. whether the fixed layout matches the environment
4. whether the team is accidentally localizing against non-fixed tags
5. whether the plausibility gate is too strict or too loose
6. whether the guidance bridge and localizer are actually sharing the same solver tuning
7. whether the fused localizer is rebasing odometry correctly after accepted vision corrections
8. whether `odomHistorySec` is large enough for the accepted vision age when latency compensation is enabled
9. whether the fusion tester is reporting lots of duplicate / out-of-order vision frames

Those basics usually matter more than fancy estimator math.

---

## 11. Season bring-up checklist for maintainers

When FIRST publishes a new game:

1. confirm which detected tags are truly field-fixed
2. update `FtcGameTagLayout.officialGameFieldFixed(...)`
3. keep the detector library broad enough for the season's vision use cases
4. keep localization / guidance layouts restricted to the fixed tags
5. check the tester layout summaries to confirm included vs excluded IDs

The framework should absorb that fixed-tag policy so robot code does not need to rediscover it.
