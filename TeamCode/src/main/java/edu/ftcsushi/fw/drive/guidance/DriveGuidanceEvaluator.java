package edu.ftcsushi.fw.drive.guidance;

import java.util.Objects;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.sensing.vision.apriltag.TagSelectionResult;
import edu.ftcsushi.fw.spatial.FacingSolution;
import edu.ftcsushi.fw.spatial.SpatialLaneResult;
import edu.ftcsushi.fw.spatial.SpatialQuery;
import edu.ftcsushi.fw.spatial.SpatialQueryResult;
import edu.ftcsushi.fw.spatial.TranslationSolution;

/**
 * Spatial evaluation bridge for {@link DriveGuidancePlan}.
 *
 * <p>{@link DriveGuidanceEvaluator} consumes the shared spatial-query layer introduced for
 * reusable task-space solving, then adapts those per-lane results into the drive-specific
 * translation/omega solution shape expected by {@link DriveGuidanceCore}.</p>
 *
 * <p>The only remaining drive-specific solve logic here is the latched
 * {@link DriveGuidanceSpec.RobotRelativePoint} target. That target captures the translation frame's
 * field pose at the first valid solve after enable and is therefore intentionally kept outside the
 * generic spatial-query API.</p>
 */
final class DriveGuidanceEvaluator {

    private static final TagSelectionResult NO_SELECTION =
            TagSelectionResult.none();

    private final DriveGuidanceSpec spec;
    private final SpatialQuery spatialQuery;
    private Pose2d fieldToTranslationFrameAnchor = null;

    /**
     * Creates an evaluator for one immutable guidance spec.
     */
    DriveGuidanceEvaluator(DriveGuidanceSpec spec) {
        this.spec = Objects.requireNonNull(spec, "spec");
        this.spatialQuery = spec.spatialQuerySpec != null
                ? SpatialQuery.from(spec.spatialQuerySpec)
                : null;
    }

    /**
     * Resets evaluator-owned runtime state captured after enable, such as robot-relative translation
     * anchors, and clears the runtime spatial-query cache. Selected-tag policies and the other
     * spatial-spec collaborators remain owned by their suppliers.
     */
    void onEnable() {
        fieldToTranslationFrameAnchor = null;
        if (spatialQuery != null) {
            spatialQuery.reset();
        }
    }

    /**
     * Returns the currently latched field-space translation-frame anchor used by robot-relative
     * translation targets, or {@code null} when no anchor has been captured yet.
     */
    Pose2d fieldToTranslationFrameAnchor() {
        return fieldToTranslationFrameAnchor;
    }

    /** Evaluates only the configured evidence authority, never a fallback or second solver. */
    Solution solve(LoopClock clock) {
        if (spec.resolveWith.mode == DriveGuidanceSpec.SolveMode.ABSOLUTE_POSE) {
            return solveWithAbsolutePose(clock);
        }
        return solutionFromLane(sampleSpatialQuery(clock), 0);
    }

    /**
     * Attempts to solve the configured targets from the localization lane.
     *
     * <p>Most localization-backed targets are now handled by the shared spatial-query layer via the
     * {@code AbsolutePoseSpatialSolveLane}. The one exception is
     * {@link DriveGuidanceSpec.RobotRelativePoint}, whose latched-on-enable semantics remain local
     * to DriveGuidance.</p>
     */
    Solution solveWithAbsolutePose(LoopClock clock) {
        DriveGuidanceSpec.AbsolutePose cfg = spec.resolveWith.absolutePose;
        if (cfg == null) {
            return Solution.invalid();
        }

        SpatialQueryResult sample = sampleSpatialQuery(clock);
        SpatialLaneResult lane = laneResult(sample, 0);

        TranslationSolve translation;
        TagSelectionResult translationSelection;
        if (spec.translationTarget instanceof DriveGuidanceSpec.RobotRelativePoint) {
            translation = solveRobotRelativeTranslation(clock, cfg, sample);
            translationSelection = NO_SELECTION;
        } else {
            translation = toTranslationSolve(sample, lane);
            translationSelection = lane.translationSelection;
        }

        FacingSolve aim = toFacingSolve(lane);
        boolean valid = translation.canTranslate || aim.canOmega;
        return new Solution(
                valid,
                translation.canTranslate,
                aim.canOmega,
                translation.forwardErrorIn,
                translation.leftErrorIn,
                aim.omegaErrorRad,
                translation.hasRangeInches,
                translation.rangeInches,
                translationSelection,
                lane.facingSelection
        );
    }

    /**
     * Samples the configured spatial-query runtime once for this loop.
     */
    private SpatialQueryResult sampleSpatialQuery(LoopClock clock) {
        return spatialQuery != null ? spatialQuery.get(clock) : null;
    }

    /**
     * Returns one per-lane spatial result or a synthetic empty result when that lane is absent.
     */
    private static SpatialLaneResult laneResult(SpatialQueryResult sample, int laneIndex) {
        if (sample == null || laneIndex < 0 || laneIndex >= sample.laneCount()) {
            return SpatialLaneResult.none();
        }
        SpatialLaneResult result = sample.laneResult(laneIndex);
        return result != null ? result : SpatialLaneResult.none();
    }

    /**
     * Converts one spatial lane result into the drive-guidance solution format.
     */
    private Solution solutionFromLane(SpatialQueryResult sample, int laneIndex) {
        SpatialLaneResult lane = laneResult(sample, laneIndex);
        TranslationSolve translation = toTranslationSolve(sample, lane);
        FacingSolve aim = toFacingSolve(lane);
        boolean valid = translation.canTranslate || aim.canOmega;
        return new Solution(
                valid,
                translation.canTranslate,
                aim.canOmega,
                translation.forwardErrorIn,
                translation.leftErrorIn,
                aim.omegaErrorRad,
                translation.hasRangeInches,
                translation.rangeInches,
                lane.translationSelection,
                lane.facingSelection
        );
    }

    /**
     * Converts one shared translation solution into DriveGuidance's translation-error convention.
     *
     * <p>The shared spatial layer reports the fully solved target point in robot coordinates and in
     * the translation frame's coordinates. DriveGuidance keeps its historic convention: translation
     * error is the target point minus the translation frame's <em>origin</em>, expressed in robot
     * forward/left axes. Rotate the solved frame vector back to those axes, retaining the frame
     * sampled by that lane (which may be at capture time), not the query's current tool pose.</p>
     */
    private static TranslationSolve toTranslationSolve(SpatialQueryResult sample, SpatialLaneResult lane) {
        if (sample == null || lane == null || lane.translation == null) {
            return TranslationSolve.invalid();
        }
        TranslationSolution translation = lane.translation;
        Pose2d framePoint = translation.translationFrameToTargetPoint;
        double frameHeading = Pose2d.wrapToPi(
                Pose2d.wrapToPi(translation.robotToTargetPoint.headingRad)
                        - Pose2d.wrapToPi(framePoint.headingRad));
        double c = Math.cos(frameHeading);
        double s = Math.sin(frameHeading);
        double forwardErr = c * framePoint.xInches - s * framePoint.yInches;
        double leftErr = s * framePoint.xInches + c * framePoint.yInches;
        if (!Double.isFinite(forwardErr) || !Double.isFinite(leftErr)
                || !Double.isFinite(Math.hypot(forwardErr, leftErr))) {
            return TranslationSolve.invalid();
        }
        return new TranslationSolve(true,
                forwardErr,
                leftErr,
                translation.hasRangeInches && Double.isFinite(translation.rangeInches),
                translation.rangeInches);
    }

    /**
     * Converts one shared aim solution into DriveGuidance's omega-error convention.
     */
    private static FacingSolve toFacingSolve(SpatialLaneResult lane) {
        if (lane == null || lane.facing == null) {
            return FacingSolve.invalid();
        }
        FacingSolution aim = lane.facing;
        if (!Double.isFinite(aim.facingErrorRad)) return FacingSolve.invalid();
        return new FacingSolve(true, aim.facingErrorRad);
    }

    /**
     * Solves the drive-specific latched robot-relative translation target from the localization
     * pose estimate.
     */
    private TranslationSolve solveRobotRelativeTranslation(LoopClock clock,
                                                           DriveGuidanceSpec.AbsolutePose cfg,
                                                           SpatialQueryResult sample) {
        long cycle = clock.cycle();
        PoseEstimate est = cfg.poseEstimator.getEstimate();
        boolean valid = est != null
                && est.hasPose
                && est.timestamp.isFresh(clock, cfg.maxAgeSec)
                && Double.isFinite(est.quality) && est.quality >= cfg.minQuality && est.quality <= 1.0
                && isFinitePose(est.fieldToRobotPose);
        if (!valid) {
            return TranslationSolve.invalid();
        }

        Pose2d robotToTranslationFrame = Objects.requireNonNull(
                spec.controlFrames.translationFrame().getAt(clock, est.timestamp),
                "SpatialControlFrames.translationFrame().getAt(...) returned null");
        if (!isFinitePose(robotToTranslationFrame)) return TranslationSolve.invalid();

        Pose2d fieldToRobot = est.toPose2d();
        Pose2d anchor = fieldToTranslationFrameAnchor != null
                ? fieldToTranslationFrameAnchor : fieldToRobot.then(robotToTranslationFrame);
        if (!isFinitePose(anchor)) return TranslationSolve.invalid();

        DriveGuidanceSpec.RobotRelativePoint target =
                (DriveGuidanceSpec.RobotRelativePoint) spec.translationTarget;
        Pose2d fieldToTargetPoint = anchor.then(
                new Pose2d(target.forwardInches, target.leftInches, 0.0)
        );
        Pose2d robotToTargetPoint = fieldToRobot.inverse().then(fieldToTargetPoint);

        double forwardError = robotToTargetPoint.xInches - robotToTranslationFrame.xInches;
        double leftError = robotToTargetPoint.yInches - robotToTranslationFrame.yInches;
        if (!Double.isFinite(forwardError) || !Double.isFinite(leftError)
                || !Double.isFinite(Math.hypot(forwardError, leftError))) {
            return TranslationSolve.invalid();
        }
        if (clock.cycle() != cycle) {
            throw new IllegalStateException("Drive guidance callbacks must not advance or reset LoopClock");
        }
        fieldToTranslationFrameAnchor = anchor;
        return new TranslationSolve(true, forwardError, leftError, false, Double.NaN);
    }

    private static boolean isFinitePose(Pose2d pose) {
        return pose != null && Double.isFinite(pose.xInches)
                && Double.isFinite(pose.yInches) && Double.isFinite(pose.headingRad);
    }

    private static boolean isFinitePose(Pose3d pose) {
        return pose != null && Double.isFinite(pose.xInches) && Double.isFinite(pose.yInches)
                && Double.isFinite(pose.zInches) && Double.isFinite(pose.yawRad)
                && Double.isFinite(pose.pitchRad) && Double.isFinite(pose.rollRad);
    }

    static final class Solution {
        final boolean valid;
        final boolean canTranslate;
        final boolean canOmega;
        final double forwardErrorIn;
        final double leftErrorIn;
        final double omegaErrorRad;
        final boolean hasRangeInches;
        final double rangeInches;
        final TagSelectionResult translationSelection;
        final TagSelectionResult facingSelection;

        Solution(boolean valid,
                 boolean canTranslate,
                 boolean canOmega,
                 double forwardErrorIn,
                 double leftErrorIn,
                 double omegaErrorRad,
                 boolean hasRangeInches,
                 double rangeInches,
                 TagSelectionResult translationSelection,
                 TagSelectionResult facingSelection) {
            this.valid = valid;
            this.canTranslate = canTranslate;
            this.canOmega = canOmega;
            this.forwardErrorIn = forwardErrorIn;
            this.leftErrorIn = leftErrorIn;
            this.omegaErrorRad = omegaErrorRad;
            this.hasRangeInches = hasRangeInches;
            this.rangeInches = rangeInches;
            this.translationSelection = translationSelection != null ? translationSelection : NO_SELECTION;
            this.facingSelection = facingSelection != null ? facingSelection : NO_SELECTION;
        }

        static Solution invalid() {
            return new Solution(false, false, false, 0.0, 0.0, 0.0, false, Double.NaN, NO_SELECTION, NO_SELECTION);
        }
    }

    private static final class TranslationSolve {
        final boolean canTranslate;
        final double forwardErrorIn;
        final double leftErrorIn;
        final boolean hasRangeInches;
        final double rangeInches;

        TranslationSolve(boolean canTranslate,
                         double forwardErrorIn,
                         double leftErrorIn,
                         boolean hasRangeInches,
                         double rangeInches) {
            this.canTranslate = canTranslate;
            this.forwardErrorIn = forwardErrorIn;
            this.leftErrorIn = leftErrorIn;
            this.hasRangeInches = hasRangeInches;
            this.rangeInches = rangeInches;
        }

        static TranslationSolve invalid() {
            return new TranslationSolve(false, 0.0, 0.0, false, Double.NaN);
        }
    }

    private static final class FacingSolve {
        final boolean canOmega;
        final double omegaErrorRad;

        FacingSolve(boolean canOmega, double omegaErrorRad) {
            this.canOmega = canOmega;
            this.omegaErrorRad = omegaErrorRad;
        }

        static FacingSolve invalid() {
            return new FacingSolve(false, 0.0);
        }
    }
}
