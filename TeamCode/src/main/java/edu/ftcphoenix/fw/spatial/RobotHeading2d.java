package edu.ftcphoenix.fw.spatial;

import edu.ftcphoenix.fw.core.geometry.Pose2d;

/**
 * A robot-aware heading error function.
 *
 * <p>This is the heading counterpart to {@link RobotZone2d}:
 * instead of answering “is the robot inside a region?”, it answers
 * “how far (in radians) is the robot turned away from a desired heading or bearing?”.</p>
 *
 * <p><b>Sign convention:</b> the returned value is a signed error in radians wrapped to
 * [-π, +π]. A positive value means “rotate CCW” (turn left) to reduce the error.</p>
 *
 * <p>This interface is intentionally controller-free. It is used by the spatial predicate
 * layer (see {@link HeadingLatch}) and can also be used directly for telemetry and gating.</p>
 */
@FunctionalInterface
public interface RobotHeading2d {

    /**
     * Compute the signed heading error (radians) for the given robot pose.
     *
     * @param fieldToRobot robot pose in the field frame
     * @return signed heading error in radians wrapped to [-π, +π]
     */
    double errorRad(Pose2d fieldToRobot);

    /**
     * Convenience: check whether the heading error is within the provided tolerance.
     *
     * @param fieldToRobot robot pose in the field frame
     * @param tolRad       absolute tolerance in radians
     * @return true if |error| <= tolRad
     */
    default boolean within(Pose2d fieldToRobot, double tolRad) {
        double e = errorRad(fieldToRobot);
        return Double.isFinite(e) && Math.abs(e) <= tolRad;
    }
}
