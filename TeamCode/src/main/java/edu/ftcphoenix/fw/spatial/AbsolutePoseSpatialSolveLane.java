package edu.ftcphoenix.fw.spatial;

import java.util.Objects;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.localization.AbsolutePoseEstimator;
import edu.ftcphoenix.fw.localization.PoseEstimate;

/**
 * Spatial solve lane backed by any {@link AbsolutePoseEstimator}.
 *
 * <p>This lane is the shared “solve from field pose” adapter. It can be driven by a corrected
 * localization estimate, an AprilTag-only field pose, a Limelight field pose, or any future
 * absolute pose source.</p>
 */
public final class AbsolutePoseSpatialSolveLane implements SpatialSolveLane {

    public static final double DEFAULT_MAX_AGE_SEC = 0.50;
    public static final double DEFAULT_MIN_QUALITY = 0.10;

    private final AbsolutePoseEstimator poseEstimator;
    private final double maxAgeSec;
    private final double minQuality;

    /**
     * Creates an absolute-pose-backed solve lane with default freshness and quality gating.
     */
    public AbsolutePoseSpatialSolveLane(AbsolutePoseEstimator poseEstimator) {
        this(poseEstimator, DEFAULT_MAX_AGE_SEC, DEFAULT_MIN_QUALITY);
    }

    /**
     * Creates an absolute-pose-backed solve lane with explicit freshness and quality gating.
     */
    public AbsolutePoseSpatialSolveLane(AbsolutePoseEstimator poseEstimator,
                                        double maxAgeSec,
                                        double minQuality) {
        this.poseEstimator = Objects.requireNonNull(poseEstimator, "poseEstimator");
        this.maxAgeSec = maxAgeSec;
        this.minQuality = minQuality;
    }

    @Override
    public SpatialLaneResult solve(SpatialSolveRequest request) {
        PoseEstimate est = poseEstimator.getEstimate();
        double estAgeSec = (est != null)
                ? Math.max(est.ageSec, request.clock.nowSec() - est.timestampSec)
                : Double.POSITIVE_INFINITY;
        boolean valid = est != null && est.hasPose && estAgeSec <= maxAgeSec && est.quality >= minQuality;
        if (!valid) {
            return SpatialLaneResult.none();
        }

        Pose2d fieldToRobot = est.toPose2d();
        TranslationSolution translation = null;
        if (request.translationTarget != null) {
            Pose2d fieldToTargetPoint = SpatialQuerySupport.resolveFieldPointTarget(
                    request.translationTarget,
                    request.fixedAprilTagLayout,
                    request.clock
            );
            if (fieldToTargetPoint != null) {
                translation = SpatialSolveMath.translationFromFieldPose(
                        fieldToRobot,
                        request.robotToTranslationFrame,
                        fieldToTargetPoint,
                        false,
                        Double.NaN,
                        est.quality,
                        estAgeSec
                );
            }
        }

        AimSolution aim = null;
        if (request.aimTarget instanceof SpatialTargets.FieldHeading) {
            aim = SpatialSolveMath.aimFromFieldHeading(
                    fieldToRobot,
                    request.robotToAimFrame,
                    ((SpatialTargets.FieldHeading) request.aimTarget).fieldHeadingRad,
                    est.quality,
                    estAgeSec
            );
        } else if (request.aimTarget instanceof SpatialTargets.ReferenceFrameHeadingTarget) {
            SpatialTargets.ReferenceFrameHeadingTarget target = (SpatialTargets.ReferenceFrameHeadingTarget) request.aimTarget;
            Pose2d fieldToFrame = SpatialQuerySupport.resolveFieldFrameHeadingTarget(
                    target,
                    request.fixedAprilTagLayout,
                    request.clock
            );
            if (fieldToFrame != null) {
                aim = SpatialSolveMath.aimFromFieldHeading(
                        fieldToRobot,
                        request.robotToAimFrame,
                        fieldToFrame.headingRad + target.headingOffsetRad,
                        est.quality,
                        estAgeSec
                );
            }
        } else if (request.aimTarget != null) {
            Pose2d fieldToAimPoint = SpatialQuerySupport.resolveFieldPointTarget(
                    request.aimTarget,
                    request.fixedAprilTagLayout,
                    request.clock
            );
            if (fieldToAimPoint != null) {
                aim = SpatialSolveMath.aimFromFieldPoint(
                        fieldToRobot,
                        request.robotToAimFrame,
                        fieldToAimPoint,
                        est.quality,
                        estAgeSec
                );
            }
        }

        return SpatialLaneResult.of(
                translation,
                aim,
                SpatialQuerySupport.translationSelectionSnapshot(request.translationTarget, request.clock, null, Double.POSITIVE_INFINITY),
                SpatialQuerySupport.aimSelectionSnapshot(request.aimTarget, request.clock, null, Double.POSITIVE_INFINITY)
        );
    }

    @Override
    public String toString() {
        return "AbsolutePoseSpatialSolveLane{poseEstimator=" + poseEstimator
                + ", maxAgeSec=" + maxAgeSec
                + ", minQuality=" + minQuality + '}';
    }
}
