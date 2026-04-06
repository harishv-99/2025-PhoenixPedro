# Phoenix Localization Follow-Ups

This file tracks Phoenix-specific follow-up work now that TeleOp uses the fused Pinpoint + AprilTag localizer, the framework owns the fixed-tag policy, both localization + guidance can share the same fixed-tag field-pose solver policy, and the robot-specific testers can mirror the production AprilTag tuning.

---

## Near-term

1. Field-test and retune `RobotConfig.Localization.aprilTags`:
   validate the multi-tag solve on the real DECODE field after camera-mount calibration.

2. Field-test and retune `RobotConfig.Localization.pinpointAprilTagFusion`:
   confirm the correction gains feel stable during normal TeleOp driving and after brief vision dropouts now that the framework rebases odometry correctly after accepted vision corrections.

3. Retune `shootBrace` / pose-lock gains with the fused localizer:
   the brace overlay now depends on the fused pose rather than odometry alone.

4. Verify on the real robot that the new field-plausibility gate is permissive enough near walls/corners but still catches impossible AprilTag solves.

5. Keep scoring-tag selection intentionally limited to the fixed goal tags used for localization fallback.

---

## Longer-term

1. Add driver-facing telemetry for aim source / localization source only if the drivers find it genuinely useful in matches.

2. Share the fused localizer container across TeleOp and Auto initialization so both modes rely on one pose stack.

3. Revisit automatic vision-based velocity capture once more range-to-shot data is collected from the real field.
