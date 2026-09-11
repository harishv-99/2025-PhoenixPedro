package edu.ftcsushi.fw.spatial;

import java.util.Objects;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.time.LoopTimestamp;

/**
 * Solved translation relationship from one spatial solve lane.
 *
 * <p>The solution keeps both common views of the same relationship:</p>
 * <ul>
 *   <li>{@link #robotToTargetPoint}: the target point in robot coordinates</li>
 *   <li>{@link #translationFrameToTargetPoint}: the same point expressed in the sampled
 *       translation frame's coordinates</li>
 * </ul>
 *
 * <p>This lets a drivetrain, an extension planner, or a manipulator planner reuse the same solve
 * result without duplicating geometry. The solution carries quality and an epoch-safe measurement
 * timestamp so consumers can apply consistent gates.</p>
 */
public final class TranslationSolution {

    public final Pose2d robotToTargetPoint;
    public final Pose2d translationFrameToTargetPoint;
    public final boolean hasRangeInches;
    public final double rangeInches;
    /** Lane-specific score, or NaN when no calibrated score is supplied; not pickup probability. */
    public final double quality;
    /** Oldest required evidence; remembered sightings are not refreshed by a new robot pose. */
    public final LoopTimestamp timestamp;
    /** Pose evidence used for a field solve; unavailable for direct observed-point feedback. */
    public final LoopTimestamp robotPoseTimestamp;
    /** Original target sighting, or unavailable for an authored fixed target. */
    public final LoopTimestamp targetObservationTimestamp;
    /**
     * True when target age constrains this result, including remembered locations; not visibility.
     * False for an explicit committed destination or an authored fixed target.
     */
    public final boolean liveTarget;

    /**
     * Creates a translation solution.
     */
    public TranslationSolution(Pose2d robotToTargetPoint,
                               Pose2d translationFrameToTargetPoint,
                               boolean hasRangeInches,
                               double rangeInches,
                               double quality,
                               LoopTimestamp timestamp) {
        this(robotToTargetPoint, translationFrameToTargetPoint, hasRangeInches, rangeInches,
                quality, timestamp, timestamp, LoopTimestamp.unavailable(), false);
    }

    private TranslationSolution(Pose2d robotToTargetPoint, Pose2d translationFrameToTargetPoint,
                                boolean hasRangeInches, double rangeInches, double quality,
                                LoopTimestamp timestamp, LoopTimestamp robotPoseTimestamp,
                                LoopTimestamp targetObservationTimestamp, boolean liveTarget) {
        this.robotToTargetPoint = Objects.requireNonNull(robotToTargetPoint, "robotToTargetPoint");
        this.translationFrameToTargetPoint = Objects.requireNonNull(translationFrameToTargetPoint,
                "translationFrameToTargetPoint");
        this.hasRangeInches = hasRangeInches;
        this.rangeInches = rangeInches;
        this.quality = quality;
        this.timestamp = Objects.requireNonNull(timestamp, "timestamp");
        this.robotPoseTimestamp = robotPoseTimestamp;
        this.targetObservationTimestamp = targetObservationTimestamp;
        this.liveTarget = liveTarget;
    }

    /** Adds provenance without treating a new pose or committed goal as a new target sighting. */
    TranslationSolution withTargetEvidence(LoopTimestamp observation, boolean requiresSightingAge) {
        LoopTimestamp effective = timestamp;
        double difference = timestamp.secondsSince(observation);
        if (requiresSightingAge && Double.isFinite(difference) && difference > 0.0) effective = observation;
        return new TranslationSolution(robotToTargetPoint, translationFrameToTargetPoint,
                hasRangeInches, rangeInches, quality, effective, robotPoseTimestamp,
                observation, requiresSightingAge);
    }

    /** A direct observation solves in its capture frame without a field-pose estimate. */
    TranslationSolution withDirectObservationEvidence(LoopTimestamp observation) {
        return new TranslationSolution(robotToTargetPoint, translationFrameToTargetPoint,
                hasRangeInches, rangeInches, quality, observation, LoopTimestamp.unavailable(),
                observation, true);
    }

    /**
     * Returns target +X component in robot coordinates, in inches.
     */
    public double robotForwardInches() {
        return robotToTargetPoint.xInches;
    }

    /** Returns target +Y component in robot coordinates, in inches. */
    public double robotLeftInches() {
        return robotToTargetPoint.yInches;
    }

    /** Returns target +X component in the translation frame, in inches. */
    public double frameForwardInches() {
        return translationFrameToTargetPoint.xInches;
    }

    /** Returns target +Y component in the translation frame, in inches. */
    public double frameLeftInches() {
        return translationFrameToTargetPoint.yInches;
    }

    /** Returns planar distance from the translation frame to the target point, in inches. */
    public double frameDistanceInches() {
        return Math.hypot(translationFrameToTargetPoint.xInches, translationFrameToTargetPoint.yInches);
    }

    @Override
    public String toString() {
        return "TranslationSolution{robotToTargetPoint=" + robotToTargetPoint
                + ", translationFrameToTargetPoint=" + translationFrameToTargetPoint
                + ", hasRangeInches=" + hasRangeInches
                + ", rangeInches=" + rangeInches
                + ", quality=" + quality
                + ", timestamp=" + timestamp + '}';
    }
}
