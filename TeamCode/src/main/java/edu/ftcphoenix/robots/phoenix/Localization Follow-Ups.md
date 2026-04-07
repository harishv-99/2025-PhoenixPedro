# Phoenix Localization Follow-Ups

This file tracks Phoenix-specific follow-up work now that TeleOp uses framework-owned drive and localization lanes, the framework owns the fixed-tag policy, and Phoenix consumes those lanes through a thinner robot profile.

The current framework pass also upgraded the architecture around those lanes:

- `FtcLocalizationLane` now owns Pinpoint, AprilTag vision, fused estimator selection, per-loop localization updates, and camera cleanup.
- `FtcMecanumDriveLane` now owns mecanum wiring, brake behavior, and drivebase lifecycle.
- `PhoenixTeleOpControls` now owns all TeleOp input semantics, including the drive sticks and slow mode.
- `PhoenixRobot` now composes framework lanes and robot policy instead of rebuilding those owners inline.

Keep those owners documented so mouseover docs and markdown docs stay aligned with the code boundaries as Phoenix evolves or as the same lane pattern is reused by future robots.

---

## Near-term

1. Field-test and retune `PhoenixProfile.current().localization.aprilTags`:
   validate the multi-tag solve on the real field after camera-mount calibration.

2. Field-test and retune `PhoenixProfile.current().localization.odometryTagFusion`:
   confirm the correction gains feel stable during normal TeleOp driving and after brief vision dropouts now that the framework rebases odometry correctly after accepted vision corrections, deduplicates repeated frames, and replays odometry from the accepted frame timestamp.

3. Retune `shootBrace` / pose-lock gains with the fused localizer:
   the brace overlay now depends on the fused pose rather than odometry alone.

4. Verify on the real robot that the field-plausibility gate is permissive enough near walls/corners but still catches impossible AprilTag solves.

5. Check the fusion/EKF tester replay / projected / duplicate counters on the real field and make sure the camera pipeline is delivering fresh AprilTag frames at the expected cadence.

6. Compare `PhoenixProfile.current().localization.odometryTagFusion` against `PhoenixProfile.current().localization.odometryTagEkf` on the real field before changing `PhoenixProfile.current().localization.globalEstimatorMode`; keep the simpler fusion path as the production baseline until the EKF clearly earns it.

7. Keep scoring-tag selection intentionally limited to the fixed goal tags used for localization fallback.

---

## Longer-term

1. Add driver-facing telemetry for aim source / localization source only if the drivers find it genuinely useful in matches.

2. Reuse the same `FtcLocalizationLane` configuration across TeleOp and Auto initialization so both modes rely on one pose stack and one vision-resource owner.

3. Revisit automatic vision-based velocity capture once more range-to-shot data is collected from the real field.

4. If another future robot ends up with the same drive-controls pattern, extract only the reusable tuning/config ideas. Keep actual button identities in robot code unless several robots truly share the same operator semantics.
