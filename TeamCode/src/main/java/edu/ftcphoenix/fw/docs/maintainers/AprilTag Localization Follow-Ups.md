# AprilTag & Localization Follow-Ups

This file is the lightweight backlog for the framework's AprilTag, fixed-tag layout, and localization work.

It exists so season-specific fixes and the next maintainership steps do not get lost between refactors.

---

## Landed in the current pass

- Added a shared multi-tag fixed-field pose solver so AprilTag-only localization and the drive-guidance AprilTag→field-pose bridge use the same math.
- Added an optional field-plausibility region gate so obviously impossible AprilTag field solves can be rejected consistently.
- Surfaced the live guidance lane's fixed-tag field-pose solver config through DriveGuidance so guidance can share the same tuning / plausibility policy as localization.
- Added framework-owned subset-tag-layout helpers for role-specific fixed-tag whitelists without robot-side metadata duplication.
- Added config validation / fail-fast checks for the shared fixed-tag field-pose solver and the FTC full-field helper.
- Made the shared solver fall back to Phoenix geometry when an otherwise-acceptable FTC SDK robot pose is implausible.
- Added `TagOnlyPoseEstimator.Config.toSolverConfig()` so guidance can share solver tuning without smuggling camera-mount-only fields into unrelated APIs.
- Switched Phoenix TeleOp to use Pinpoint + AprilTag fusion for pose lock and selected-tag localization fallback.
- Moved official-game fixed-tag knowledge into the framework via `FtcGameTagLayout.currentGameFieldFixed()`.
- Removed the need for robot code to carry season-specific "exclude these IDs from localization" logic.
- Updated framework tools that previously converted the entire FTC SDK game library into a field-fixed `TagLayout`.
- Updated the localization testers so they can use the same AprilTag-localizer config as production robot code.
- Added public documentation for fixed-tag layout policy, shared solver tuning, plausibility gating, and selected-tag localization semantics.
- Documented and preserved the ancillary/sample-tag normalization used by `FtcGameTagLayout.currentGameFieldFixed()` when the FTC SDK current-game library is broader than the official field policy.
- Consolidated overlapping AprilTag design notes around one canonical fixed-layout/localization guide to reduce documentation drift.
- Normalized AprilTag field-pose solver config at the guidance API boundary so subtype-only config cannot leak across unrelated APIs.
- Added a shared `FtcTagLayoutDebug.dumpSummary(...)` helper and surfaced the layout policy summary in AprilTag testers/calibrators.
- Fixed a fusion reliability bug by rebasing the odometry baseline after accepted vision corrections are pushed back into odometry.
- Upgraded fusion to deduplicate repeated camera frames by measurement timestamp and to apply accepted vision corrections at the frame timestamp before replaying odometry forward.
- Added fail-fast validation for fusion latency-compensation history length and surfaced replay/projected/duplicate counters in the fusion tester.
- Added a shared `VisionCorrectionPoseEstimator` contract so robot code and testers can swap between the lightweight fusion localizer and an optional EKF-style localizer without type-specific glue.
- Added `OdometryTagEkfPoseEstimator` as an optional covariance-aware global localizer with documented calibration requirements, innovation gating, and measurement-time replay.
- Extended the Pinpoint + AprilTag localization tester so teams can compare the default fusion estimator against the optional EKF estimator on the same calibrated inputs.

---

## Near-term follow-ups

1. When Phoenix is updated for a new FTC season, extend `FtcGameTagLayout.officialGameFieldFixed(...)` with the new season's fixed-tag policy instead of letting robot code paper over the difference.

---

## Longer-term upgrades worth considering

1. Better per-observation weighting:
   if FTC exposes richer confidence signals (ambiguity, reprojection error, tag area/corner fit), fold them into the fixed-tag pose solver.

2. Mixed candidate-set design review:
   today selected-tag references only promote through localization when all candidate IDs are field-fixed.
   If a real use case appears, consider a principled runtime policy that allows localization only when the currently selected tag is fixed.

3. Field-validate the optional EKF estimator before recommending it broadly:
   compare it against the simpler fusion estimator on real FTC fields and keep the advanced path opt-in until the tuning/docs feel mature.

4. Better uncertainty inputs if field data justifies it:
   for example, richer per-observation confidence from AprilTag detection quality, or velocity-aware process-noise tuning.
