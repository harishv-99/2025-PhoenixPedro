package edu.ftcphoenix.fw.spatial;

import java.util.Objects;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.localization.AbsolutePoseEstimator;
import edu.ftcphoenix.fw.localization.PoseEstimate;

/**
 * Spatial solve lane backed by any {@link AbsolutePoseEstimator}.
 *
 * <p>This is the shared “solve from field pose” adapter. It can be driven by corrected
 * localization, AprilTag-only pose, Limelight field pose, or any future absolute pose source.</p>
 *
 * <p>When the pose estimate is delayed, the lane asks the query's time-aware control frames for the
 * frame pose at the pose timestamp. This keeps moving mechanism frames aligned with the robot pose
 * used to solve the spatial relationship.</p>
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

    /** Creates an absolute-pose-backed solve lane with explicit freshness and quality gating. */
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

        double timestampSec = est.timestampSec;
        Pose2d fieldToRobot = est.toPose2d();
        Pose2d translationFrame = request.robotToTranslationFrameAt(timestampSec);
        Pose2d facingFrame = request.robotToFacingFrameAt(timestampSec);

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
                        translationFrame,
                        fieldToTargetPoint,
                        false,
                        Double.NaN,
                        est.quality,
                        estAgeSec,
                        timestampSec
                );
            }
        }

        FacingSolution facing = null;
        if (request.facingTarget instanceof SpatialTargets.FieldHeading) {
            facing = SpatialSolveMath.facingFromFieldHeading(
                    fieldToRobot,
                    facingFrame,
                    ((SpatialTargets.FieldHeading) request.facingTarget).fieldHeadingRad,
                    est.quality,
                    estAgeSec,
                    timestampSec
            );
        } else if (request.facingTarget instanceof SpatialTargets.ReferenceFrameHeadingTarget) {
            SpatialTargets.ReferenceFrameHeadingTarget target = (SpatialTargets.ReferenceFrameHeadingTarget) request.facingTarget;
            Pose2d fieldToFrame = SpatialQuerySupport.resolveFieldFrameHeadingTarget(
                    target,
                    request.fixedAprilTagLayout,
                    request.clock
            );
            if (fieldToFrame != null) {
                facing = SpatialSolveMath.facingFromFieldHeading(
                        fieldToRobot,
                        facingFrame,
                        fieldToFrame.headingRad + target.headingOffsetRad,
                        est.quality,
                        estAgeSec,
                        timestampSec
                );
            }
        } else if (request.facingTarget != null) {
            Pose2d fieldToFacingPoint = SpatialQuerySupport.resolveFieldPointTarget(
                    request.facingTarget,
                    request.fixedAprilTagLayout,
                    request.clock
            );
            if (fieldToFacingPoint != null) {
                facing = SpatialSolveMath.facingFromFieldPoint(
                        fieldToRobot,
                        facingFrame,
                        fieldToFacingPoint,
                        est.quality,
                        estAgeSec,
                        timestampSec
                );
            }
        }

        return SpatialLaneResult.of(
                translation,
                facing,
                SpatialQuerySupport.translationSelectionSnapshot(request.translationTarget, request.clock, null, Double.POSITIVE_INFINITY),
                SpatialQuerySupport.facingSelectionSnapshot(request.facingTarget, request.clock, null, Double.POSITIVE_INFINITY)
        );
    }

    @Override
    public String toString() {
        return "AbsolutePoseSpatialSolveLane{poseEstimator=" + poseEstimator
                + ", maxAgeSec=" + maxAgeSec
                + ", minQuality=" + minQuality + '}';
    }
}
