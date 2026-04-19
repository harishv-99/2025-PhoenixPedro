package edu.ftcphoenix.fw.drive.guidance;

import java.util.Collections;
import java.util.Objects;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionResult;
import edu.ftcphoenix.fw.spatial.FacingSolution;
import edu.ftcphoenix.fw.spatial.SpatialLaneResult;
import edu.ftcphoenix.fw.spatial.SpatialQuery;
import edu.ftcphoenix.fw.spatial.SpatialQueryResult;
import edu.ftcphoenix.fw.spatial.TranslationSolution;

/**
 * Spatial evaluation bridge for {@link DriveGuidancePlan}.
 *
 * <p>{@link DriveGuidanceEvaluator} consumes the shared spatial-query layer introduced for
 * reusable task-space solving, then adapts those per-lane results into the drive-specific
 * translation/omega solution shape expected by {@link DriveGuidanceCore}.</p>
 *
 * <p>The only remaining drive-specific solve logic here is the latched
 * {@link DriveGuidanceSpec.RobotRelativePoint} target. That target captures the translation frame's
 * field pose on enable and is therefore intentionally kept outside the generic spatial-query API.</p>
 */
final class DriveGuidanceEvaluator {

    private static final TagSelectionResult NO_SELECTION =
            TagSelectionResult.none(Collections.<Integer>emptySet());

    private final DriveGuidanceSpec spec;
    private final SpatialQuery spatialQuery;
    private Pose2d fieldToTranslationFrameAnchor = null;

    /**
     * Creates an evaluator for one immutable guidance spec.
     */
    DriveGuidanceEvaluator(DriveGuidanceSpec spec) {
        this.spec = Objects.requireNonNull(spec, "spec");
        this.spatialQuery = spec.spatialQuerySpec != null ? new SpatialQuery(spec.spatialQuerySpec) : null;
    }

    /**
     * Resets runtime state that is captured on enable, such as robot-relative translation anchors
     * and sticky selected-tag references.
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

    /**
     * Attempts to solve the configured targets from the shared live-AprilTag spatial lane.
     */
    Solution solveWithAprilTags(LoopClock clock) {
        if (spec.resolveWith.aprilTags == null || spatialQuery == null || spec.aprilTagsLaneIndex < 0) {
            return Solution.invalid();
        }
        return solutionFromLane(sampleSpatialQuery(clock), spec.aprilTagsLaneIndex);
    }

    /**
     * Attempts to solve the configured targets from the localization lane.
     *
     * <p>Most localization-backed targets are now handled by the shared spatial-query layer via the
     * {@code AbsolutePoseSpatialSolveLane}. The one exception is
     * {@link DriveGuidanceSpec.RobotRelativePoint}, whose latched-on-enable semantics remain local
     * to DriveGuidance.</p>
     */
    Solution solveWithLocalization(LoopClock clock) {
        DriveGuidanceSpec.Localization cfg = spec.resolveWith.localization;
        if (cfg == null) {
            return Solution.invalid();
        }

        SpatialQueryResult sample = sampleSpatialQuery(clock);
        SpatialLaneResult lane = laneResult(sample, spec.localizationLaneIndex);

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
     * Samples the shared spatial query for this loop, reusing the cached per-cycle sample when both
     * localization and AprilTag solve paths inspect it.
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
     * forward/left axes. This preserves existing drive behavior while still exposing richer frame
     * coordinates to non-drive consumers through {@link TranslationSolution}.</p>
     */
    private static TranslationSolve toTranslationSolve(SpatialQueryResult sample, SpatialLaneResult lane) {
        if (sample == null || lane == null || lane.translation == null) {
            return TranslationSolve.invalid();
        }
        TranslationSolution translation = lane.translation;
        double forwardErr = translation.robotToTargetPoint.xInches - sample.robotToTranslationFrame.xInches;
        double leftErr = translation.robotToTargetPoint.yInches - sample.robotToTranslationFrame.yInches;
        return new TranslationSolve(true,
                forwardErr,
                leftErr,
                translation.hasRangeInches,
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
        return new FacingSolve(true, aim.facingErrorRad);
    }

    /**
     * Solves the drive-specific latched robot-relative translation target from the localization
     * pose estimate.
     */
    private TranslationSolve solveRobotRelativeTranslation(LoopClock clock,
                                                           DriveGuidanceSpec.Localization cfg,
                                                           SpatialQueryResult sample) {
        PoseEstimate est = cfg.poseEstimator.getEstimate();
        double estAgeSec = (est != null)
                ? Math.max(est.ageSec, clock.nowSec() - est.timestampSec)
                : Double.POSITIVE_INFINITY;
        boolean valid = est != null
                && est.hasPose
                && estAgeSec <= cfg.maxAgeSec
                && est.quality >= cfg.minQuality;
        if (!valid) {
            return TranslationSolve.invalid();
        }

        Pose2d robotToTranslationFrame = (sample != null)
                ? sample.robotToTranslationFrame
                : Objects.requireNonNull(
                spec.controlFrames.translationFrame().get(clock),
                "SpatialControlFrames.translationFrame().get(clock) returned null"
        );

        Pose2d fieldToRobot = est.toPose2d();
        Pose2d fieldToTranslationFrame = fieldToRobot.then(robotToTranslationFrame);
        if (fieldToTranslationFrameAnchor == null) {
            fieldToTranslationFrameAnchor = fieldToTranslationFrame;
        }

        DriveGuidanceSpec.RobotRelativePoint target =
                (DriveGuidanceSpec.RobotRelativePoint) spec.translationTarget;
        Pose2d fieldToTargetPoint = fieldToTranslationFrameAnchor.then(
                new Pose2d(target.forwardInches, target.leftInches, 0.0)
        );
        Pose2d robotToTargetPoint = fieldToRobot.inverse().then(fieldToTargetPoint);

        return new TranslationSolve(
                true,
                robotToTargetPoint.xInches - robotToTranslationFrame.xInches,
                robotToTargetPoint.yInches - robotToTranslationFrame.yInches,
                false,
                Double.NaN
        );
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
