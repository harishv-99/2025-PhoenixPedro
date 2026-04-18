# Phoenix Localization Follow-Ups

This file tracks Phoenix-specific follow-up work now that TeleOp uses framework-owned drive and localization lanes plus a backend-neutral AprilTag vision seam, and Phoenix consumes those through a thinner robot profile.

The current framework split is:

- `FtcMecanumDriveLane` owns mecanum wiring, brake behavior, and drivebase lifecycle.
- `AprilTagVisionLane` is the backend-neutral seam; Phoenix now instantiates either `FtcWebcamAprilTagVisionLane` or `FtcLimelightAprilTagVisionLane` for backend-specific device identity, camera mount, and AprilTag resource cleanup.
- `FtcOdometryAprilTagLocalizationLane` owns Pinpoint, AprilTag-only field solving, raw AprilTag solving, optional direct Limelight field pose, corrected estimator selection, and per-loop pose production.
- `PhoenixCapabilities` owns the shared mode-neutral robot vocabulary used by TeleOp and Auto.
- `PhoenixTeleOpControls` owns all TeleOp input semantics, including the drive sticks and slow mode.
- `PhoenixRobot` now composes framework lanes and robot policy instead of rebuilding those owners inline.

Keep those owners documented so Javadocs, markdown docs, and real code boundaries stay aligned as Phoenix evolves or as the same lane pattern is reused by future robots.

---

## Near-term

1. Field-test and retune `PhoenixProfile.current().localization.aprilTags` after camera-mount calibration.
2. Field-test and retune `PhoenixProfile.current().localization.correctionFusion` to confirm the default corrected localizer feels stable during real TeleOp driving.
3. Retune `driveAssist.shootBrace` pose-lock gains with the fused localizer.
4. Verify the field-plausibility gate is permissive enough near walls and corners but still catches impossible AprilTag solves.
5. Compare `PhoenixProfile.current().localization.correctionFusion` against `PhoenixProfile.current().localization.correctionEkf` on the real field before changing `correctedEstimatorMode`.
6. Keep scoring-tag selection intentionally limited to the fixed goal tags used for localization fallback.

## Longer-term

1. Add driver-facing telemetry for localization/aim source only if drivers actually find it useful in matches.
2. Reuse the same active `AprilTagVisionLane` configuration and `FtcOdometryAprilTagLocalizationLane` configuration across TeleOp and Auto initialization so both modes rely on one pose stack and one shared vision rig.
3. Revisit automatic vision-based velocity capture once more range-to-shot data is collected from the real field. If direct Limelight field pose stays trustworthy in motion, decide whether Phoenix should prefer `correctionSource.mode = LIMELIGHT_FIELD_POSE` on that robot or keep raw AprilTag correction as the default.
4. If another future robot ends up with the same drive-controls pattern, extract only the reusable tuning/config ideas. Keep actual button identities in robot code unless several robots truly share the same operator semantics.
