package edu.ftcphoenix.fw.spatial;

import edu.ftcphoenix.fw.core.geometry.Pose2d;

/**
 * A 2D region on the floor plane.
 *
 * <p>Regions are used for <b>spatial predicates</b>: answering yes/no questions like:
 * “Is the robot in the shooting zone?” or “Is the intake point inside a safe area?”</p>
 *
 * <p><b>Signed distance convention</b></p>
 * <ul>
 *   <li>Positive values mean the point is <b>inside</b> the region.</li>
 *   <li>Zero means “on the boundary”.</li>
 *   <li>Negative means the point is <b>outside</b> the region.</li>
 * </ul>
 *
 * <p>The signed distance lets you add hysteresis around a boundary (see {@link ZoneLatch}).
 * For robot-aware checks ("footprint overlaps zone", "robot fully inside"), see
 * {@link RobotZones2d}.</p>
 */
public interface Region2d {

    /**
     * Signed distance to the region boundary for a point.
     *
     * @param xInches X coordinate in the region’s frame
     * @param yInches Y coordinate in the region’s frame
     * @return signed distance in inches (positive inside)
     */
    double signedDistanceInches(double xInches, double yInches);

    /**
     * Convenience overload for a pose.
     */
    default double signedDistanceInches(Pose2d pose) {
        return signedDistanceInches(pose.xInches, pose.yInches);
    }

    /**
     * Returns true if the point is inside (or on) the region boundary.
     */
    default boolean contains(double xInches, double yInches) {
        return signedDistanceInches(xInches, yInches) >= 0.0;
    }

    /**
     * Returns true if the pose’s translation is inside (or on) the region boundary.
     */
    default boolean contains(Pose2d pose) {
        return contains(pose.xInches, pose.yInches);
    }
}
