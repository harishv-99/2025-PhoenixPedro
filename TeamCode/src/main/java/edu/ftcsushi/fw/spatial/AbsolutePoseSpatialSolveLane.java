package edu.ftcsushi.fw.spatial;

import java.util.Objects;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.localization.AbsolutePoseEstimator;
import edu.ftcsushi.fw.localization.PoseEstimate;

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
final class AbsolutePoseSpatialSolveLane implements SpatialSolveLane {

    public static final double DEFAULT_MAX_AGE_SEC = 0.50;
    public static final double DEFAULT_MIN_QUALITY = 0.10;

    private final AbsolutePoseEstimator poseEstimator;
    private final double maxAgeSec;
    private final double minQuality;

    /**
     * Creates an absolute-pose-backed solve lane with default freshness and quality gating.
     */
    AbsolutePoseSpatialSolveLane(AbsolutePoseEstimator poseEstimator) {
        this(poseEstimator, DEFAULT_MAX_AGE_SEC, DEFAULT_MIN_QUALITY);
    }

    /** Creates an absolute-pose-backed solve lane with explicit freshness and quality gating. */
    AbsolutePoseSpatialSolveLane(AbsolutePoseEstimator poseEstimator,
                                        double maxAgeSec,
                                        double minQuality) {
        this.poseEstimator = Objects.requireNonNull(poseEstimator, "poseEstimator");
        if (!Double.isFinite(maxAgeSec) || maxAgeSec < 0.0) {
            throw new IllegalArgumentException(
                    "maxAgeSec must be finite and >= 0, got " + maxAgeSec);
        }
        if (!Double.isFinite(minQuality) || minQuality < 0.0 || minQuality > 1.0) {
            throw new IllegalArgumentException("minQuality must be finite and in [0, 1], got " + minQuality);
        }
        this.maxAgeSec = maxAgeSec;
        this.minQuality = minQuality;
    }

    @Override
    public SpatialLaneResult solve(SpatialSolveRequest request) {
        PoseEstimate est = poseEstimator.getEstimate();
        boolean valid = est != null
                && est.hasPose
                && est.timestamp.isFresh(request.clock, maxAgeSec)
                && Double.isFinite(est.quality) && est.quality >= minQuality && est.quality <= 1.0
                && SpatialValidation.isFinite(est.fieldToRobotPose);
        if (!valid) {
            return result(request, null, null);
        }

        Pose2d fieldToRobot = est.toPose2d();
        Pose2d translationFrame = request.robotToTranslationFrameAt(est.timestamp);
        Pose2d facingFrame = request.robotToFacingFrameAt(est.timestamp);

        TranslationSolution translation = null;
        if (request.translationTarget != null && SpatialValidation.isFinite(translationFrame)) {
            Pose2d fieldToTargetPoint = SpatialQuerySupport.resolveFieldPointTarget(
                    request.translationTarget,
                    request.fixedAprilTagLayout,
                    request.clock
            );
            if (SpatialValidation.isFinite(fieldToTargetPoint)) {
                translation = SpatialSolveMath.translationFromFieldPose(
                        fieldToRobot,
                        translationFrame,
                        fieldToTargetPoint,
                        false,
                        Double.NaN,
                        est.quality,
                        est.timestamp
                );
            }
        }

        FacingSolution facing = null;
        if (SpatialValidation.isFinite(facingFrame)
                && request.facingTarget instanceof SpatialTargets.FieldHeading) {
            facing = SpatialSolveMath.facingFromFieldHeading(
                    fieldToRobot,
                    facingFrame,
                    ((SpatialTargets.FieldHeading) request.facingTarget).fieldHeadingRad,
                    est.quality,
                    est.timestamp
            );
        } else if (SpatialValidation.isFinite(facingFrame)
                && request.facingTarget instanceof SpatialTargets.ReferenceFrameHeadingTarget) {
            SpatialTargets.ReferenceFrameHeadingTarget target = (SpatialTargets.ReferenceFrameHeadingTarget) request.facingTarget;
            Pose2d fieldToFrame = SpatialQuerySupport.resolveFieldFrameHeadingTarget(
                    target,
                    request.fixedAprilTagLayout,
                    request.clock
            );
            if (SpatialValidation.isFinite(fieldToFrame)) {
                facing = SpatialSolveMath.facingFromFieldHeading(
                        fieldToRobot,
                        facingFrame,
                        SpatialSolveMath.wrappedHeadingSumRad(
                                fieldToFrame.headingRad,
                                target.headingOffsetRad
                        ),
                        est.quality,
                        est.timestamp
                );
            }
        } else if (SpatialValidation.isFinite(facingFrame) && request.facingTarget != null) {
            Pose2d fieldToFacingPoint = SpatialQuerySupport.resolveFieldPointTarget(
                    request.facingTarget,
                    request.fixedAprilTagLayout,
                    request.clock
            );
            Pose2d fieldToFacingFrame = fieldToRobot.then(facingFrame);
            if (SpatialValidation.isFinite(fieldToFacingPoint)
                    && SpatialValidation.isFinite(fieldToFacingFrame)
                    && SpatialValidation.isFinite(fieldToFacingFrame.inverse().then(fieldToFacingPoint))) {
                facing = SpatialSolveMath.facingFromFieldPoint(
                        fieldToRobot,
                        facingFrame,
                        fieldToFacingPoint,
                        est.quality,
                        est.timestamp
                );
            }
        }

        if (translation != null && (!SpatialValidation.isFinite(translation.robotToTargetPoint)
                || !SpatialValidation.isFinite(translation.translationFrameToTargetPoint)
                || !Double.isFinite(translation.frameDistanceInches()))) {
            translation = null;
        }
        if (facing != null && !Double.isFinite(facing.facingErrorRad)) facing = null;
        return result(request, translation, facing);
    }

    /** Preserve target identity even when pose or one requested geometry channel is unavailable. */
    private static SpatialLaneResult result(SpatialSolveRequest request,
                                             TranslationSolution translation, FacingSolution facing) {
        return SpatialLaneResult.of(
                SpatialQuerySupport.targetEvidence(translation, request.translationTarget, request.clock),
                SpatialQuerySupport.targetEvidence(facing, request.facingTarget, request.clock),
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
