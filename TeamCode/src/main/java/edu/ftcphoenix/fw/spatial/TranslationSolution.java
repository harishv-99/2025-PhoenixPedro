package edu.ftcphoenix.fw.spatial;

import java.util.Objects;

import edu.ftcphoenix.fw.core.geometry.Pose2d;

/**
 * Solved translation relationship from one spatial solve lane.
 *
 * <p>{@link TranslationSolution} keeps both common views of the same relationship:</p>
 * <ul>
 *   <li>{@link #robotToTargetPoint}: the target point in robot coordinates</li>
 *   <li>{@link #translationFrameToTargetPoint}: the same point expressed in the sampled
 *       translation frame's coordinates</li>
 * </ul>
 *
 * <p>This lets different consumers reuse the same solve result without re-running the geometry.
 * A drivetrain may care about robot-axis errors, while a mechanism planner may care about the
 * controlled frame's local coordinates.</p>
 */
public final class TranslationSolution {

    public final Pose2d robotToTargetPoint;
    public final Pose2d translationFrameToTargetPoint;
    public final boolean hasRangeInches;
    public final double rangeInches;
    public final double quality;
    public final double ageSec;

    public TranslationSolution(Pose2d robotToTargetPoint,
                               Pose2d translationFrameToTargetPoint,
                               boolean hasRangeInches,
                               double rangeInches,
                               double quality,
                               double ageSec) {
        this.robotToTargetPoint = Objects.requireNonNull(robotToTargetPoint, "robotToTargetPoint");
        this.translationFrameToTargetPoint = Objects.requireNonNull(translationFrameToTargetPoint,
                "translationFrameToTargetPoint");
        this.hasRangeInches = hasRangeInches;
        this.rangeInches = rangeInches;
        this.quality = quality;
        this.ageSec = ageSec;
    }

    public double robotForwardInches() {
        return robotToTargetPoint.xInches;
    }

    public double robotLeftInches() {
        return robotToTargetPoint.yInches;
    }

    public double frameForwardInches() {
        return translationFrameToTargetPoint.xInches;
    }

    public double frameLeftInches() {
        return translationFrameToTargetPoint.yInches;
    }

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
                + ", ageSec=" + ageSec + '}';
    }
}
