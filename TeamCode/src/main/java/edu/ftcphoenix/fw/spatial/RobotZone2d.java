package edu.ftcphoenix.fw.spatial;

import edu.ftcphoenix.fw.core.geometry.Pose2d;

/**
 * A robot-aware "zone" predicate with a signed-distance output.
 *
 * <p>Unlike {@link Region2d} (which is purely geometric in a frame), a {@code RobotZone2d}
 * answers a question about the robot at a pose. Examples:</p>
 * <ul>
 *   <li>"Is the robot's shooter point inside the shooting zone?"</li>
 *   <li>"Does the robot rectangle overlap the zone at all?"</li>
 *   <li>"Is the entire robot footprint fully inside the parking box?"</li>
 * </ul>
 *
 * <p><b>Signed distance convention</b> matches {@link Region2d}:</p>
 * <ul>
 *   <li>Positive = satisfied / inside.</li>
 *   <li>Zero = on boundary.</li>
 *   <li>Negative = not satisfied / outside.</li>
 * </ul>
 */
@FunctionalInterface
public interface RobotZone2d {

    /**
     * Signed distance (inches) for the configured rule at the given robot pose.
     */
    double signedDistanceInches(Pose2d fieldToRobot);

    /**
     * Convenience: return true if {@link #signedDistanceInches(Pose2d)} is non-negative.
     */
    default boolean contains(Pose2d fieldToRobot) {
        return signedDistanceInches(fieldToRobot) >= 0.0;
    }
}
