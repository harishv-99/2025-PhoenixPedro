package edu.ftcphoenix.fw.spatial;

import java.util.Objects;

import edu.ftcphoenix.fw.core.geometry.Pose2d;

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
 * result without duplicating geometry. The solution carries quality and timing metadata so consumers
 * can apply consistent gates.</p>
 */
public final class TranslationSolution {

    public final Pose2d robotToTargetPoint;
    public final Pose2d translationFrameToTargetPoint;
    public final boolean hasRangeInches;
    public final double rangeInches;
    public final double quality;
    public final double ageSec;
    public final double timestampSec;

    /**
     * Creates a translation solution without an explicit timestamp.
     */
    public TranslationSolution(Pose2d robotToTargetPoint,
                               Pose2d translationFrameToTargetPoint,
                               boolean hasRangeInches,
                               double rangeInches,
                               double quality,
                               double ageSec) {
        this(robotToTargetPoint, translationFrameToTargetPoint, hasRangeInches, rangeInches,
                quality, ageSec, Double.NaN);
    }

    /**
     * Creates a translation solution.
     */
    public TranslationSolution(Pose2d robotToTargetPoint,
                               Pose2d translationFrameToTargetPoint,
                               boolean hasRangeInches,
                               double rangeInches,
                               double quality,
                               double ageSec,
                               double timestampSec) {
        this.robotToTargetPoint = Objects.requireNonNull(robotToTargetPoint, "robotToTargetPoint");
        this.translationFrameToTargetPoint = Objects.requireNonNull(translationFrameToTargetPoint,
                "translationFrameToTargetPoint");
        this.hasRangeInches = hasRangeInches;
        this.rangeInches = rangeInches;
        this.quality = quality;
        this.ageSec = ageSec;
        this.timestampSec = timestampSec;
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
                + ", ageSec=" + ageSec
                + ", timestampSec=" + timestampSec + '}';
    }
}
