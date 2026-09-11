package edu.ftcsushi.fw.sensing.observation;

import java.util.Objects;
import java.util.function.ToDoubleFunction;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.source.ScalarSource;

/**
 * Geometric ranking rules for {@link TargetSelections#fromVisibleObjects}.
 *
 * <p>Factory calls validate configuration without sampling observations, bearing inputs or custom
 * cost functions. Every rule ranks one observed frame, never accumulated sightings. Equal finite
 * costs use the selector's deterministic geometric ordering, not input order or invented IDs.</p>
 */
public final class TargetSelectionPolicies {
    private TargetSelectionPolicies() { }

    /** Chooses minimum planar distance from the robot origin at capture, not camera 3D range. */
    public static TargetSelectionPolicy nearestToRobot() {
        return new TargetSelectionPolicy("nearest robot-at-capture origin", (frame, clock) -> observation ->
                Math.hypot(observation.forwardInches, observation.leftInches));
    }

    /**
     * Chooses minimum planar distance from an explicit fixed robot-at-capture control origin.
     * The immutable frame must have finite inch/radian components; heading does not change distance.
     */
    public static TargetSelectionPolicy nearestToControlFrame(Pose2d robotToControlFrame) {
        Objects.requireNonNull(robotToControlFrame, "robotToControlFrame");
        finite("robotToControlFrame.xInches", robotToControlFrame.xInches);
        finite("robotToControlFrame.yInches", robotToControlFrame.yInches);
        finite("robotToControlFrame.headingRad", robotToControlFrame.headingRad);
        return new TargetSelectionPolicy("nearest fixed control origin at capture", (frame, clock) -> observation ->
                Math.hypot(observation.forwardInches - robotToControlFrame.xInches,
                        observation.leftInches - robotToControlFrame.yInches));
    }

    /**
     * Chooses the nearest available field point within an inclusive finite non-negative radius.
     * Field coordinates must already come from the observation's capture-time pose lookup;
     * missing field geometry excludes a candidate. All arguments are in field inches.
     */
    public static TargetSelectionPolicy nearFieldPoint(double fieldXInches, double fieldYInches,
                                                      double maxDistanceInches) {
        finite("fieldXInches", fieldXInches);
        finite("fieldYInches", fieldYInches);
        nonnegative("maxDistanceInches", maxDistanceInches);
        return new TargetSelectionPolicy("nearest field point within radius", (frame, clock) -> observation -> {
            if (!observation.hasFieldPosition()) return Double.NaN;
            double distance = Math.hypot(observation.fieldXInches - fieldXInches,
                    observation.fieldYInches - fieldYInches);
            return distance <= maxDistanceInches ? distance : Double.NaN;
        });
    }

    /**
     * Chooses minimum angular difference from an explicit robot-frame bearing intent, radians.
     * The intent must describe the same capture frame as the observation; this does not compensate
     * current driver input for robot motion. Reads the borrowed source once per successful selector
     * cycle with an accepted nonempty frame. Non-finite intent yields no selection; reset never
     * resets that source. A failed selector calculation may read it again on a same-cycle retry.
     */
    public static TargetSelectionPolicy nearestBearingRad(ScalarSource robotBearingIntent) {
        Objects.requireNonNull(robotBearingIntent, "robotBearingIntent");
        return new TargetSelectionPolicy("nearest robot-frame bearing", (frame, clock) -> {
            double angle = robotBearingIntent.getAsDouble(clock);
            double wrapped = Math.atan2(Math.sin(angle), Math.cos(angle));
            return observation -> {
                double difference = observation.bearingRad - wrapped;
                return Math.abs(Math.atan2(Math.sin(difference), Math.cos(difference)));
            };
        });
    }

    /**
     * Chooses the candidate with most other positioned candidates within an inclusive finite
     * non-negative radius, in inches. Counts one frame's observations, not proven physical objects
     * or expected intake yield. The published ranking metric is the negative neighbor count.
     */
    public static TargetSelectionPolicy mostNeighborsWithinInches(double radiusInches) {
        nonnegative("radiusInches", radiusInches);
        return new TargetSelectionPolicy("most neighboring observations in one frame", (frame, clock) -> observation -> {
            int neighbors = 0;
            for (TargetObservation2d other : frame.observations()) {
                if (other != observation && other.hasPosition()
                        && Math.hypot(other.forwardInches - observation.forwardInches,
                        other.leftInches - observation.leftInches) <= radiusInches) neighbors++;
            }
            return -neighbors;
        });
    }

    /**
     * Chooses the lowest finite custom cost. Non-finite cost excludes a candidate. The function
     * must be a pure value operation with no mutation or external effects. An exception propagates
     * without caching a partial selection and may be retried in the same cycle.
     */
    public static TargetSelectionPolicy lowestCost(ToDoubleFunction<TargetObservation2d> cost) {
        Objects.requireNonNull(cost, "cost");
        return new TargetSelectionPolicy("lowest custom cost", (frame, clock) -> cost);
    }

    /** Validate one explicit physical-coordinate or geometric configuration value. */
    private static void finite(String name, double value) {
        if (!Double.isFinite(value)) throw new IllegalArgumentException(name + " must be finite");
    }

    /** Validate an inclusive non-negative geometric radius. */
    private static void nonnegative(String name, double value) {
        finite(name, value);
        if (value < 0) throw new IllegalArgumentException(name + " must be >= 0");
    }
}
