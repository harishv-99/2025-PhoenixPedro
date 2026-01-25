package edu.ftcphoenix.fw.spatial;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.math.MathUtil;

/**
 * Geometry helpers for common 2D “where am I / where is the target” questions.
 *
 * <p>This class is intentionally <b>controller-free</b>. It does not produce motor or drive
 * commands. It only computes angles, errors, and frame transforms.</p>
 *
 * <p><b>Layering note:</b> In Phoenix, we try to separate:</p>
 * <ul>
 *   <li><b>Spatial math</b> (this class): pure geometry and errors</li>
 *   <li><b>Spatial predicates</b> (see {@link Region2d}): yes/no decisions like “in zone?”</li>
 *   <li><b>Controllers</b> (DriveGuidance overlays, DriveGuidanceTask/GoToPoseTasks): produce drive commands from errors</li>
 * </ul>
 */
public final class SpatialMath2d {

    private SpatialMath2d() {
    }

    /**
     * Bearing from {@code from} to the point ({@code x}, {@code y}) in the same frame.
     *
     * <p>The result is a field/robot-frame angle (radians) measured CCW from +X.</p>
     */
    public static double bearingToPointRad(Pose2d from, double xInches, double yInches) {
        double dx = xInches - from.xInches;
        double dy = yInches - from.yInches;
        return Math.atan2(dy, dx);
    }

    /**
     * Bearing of a 2D vector using Phoenix conventions.
     *
     * <p>Convention recap:</p>
     * <ul>
     *   <li>+X / {@code forwardInches} is forward.</li>
     *   <li>+Y / {@code leftInches} is left.</li>
     *   <li>0 rad means straight ahead; + is left/CCW.</li>
     * </ul>
     */
    public static double bearingRadOfVector(double forwardInches, double leftInches) {
        return Math.atan2(leftInches, forwardInches);
    }

    /**
     * Resolve a point expressed in an <b>anchor's</b> coordinate frame into the parent frame.
     *
     * <p>Example: if {@code robotToTag} is a pose from robot→tag, and you have a desired
     * aim/approach point expressed in the tag frame (forward/left), this returns robot→point.</p>
     */
    public static Pose2d anchorRelativePointInches(Pose2d frameToAnchorPose,
                                                   double anchorForwardInches,
                                                   double anchorLeftInches) {
        return frameToAnchorPose.then(new Pose2d(anchorForwardInches, anchorLeftInches, 0.0));
    }

    /**
     * Convenience: bearing from the parent-frame origin to an anchor-relative point.
     */
    public static double bearingRadToAnchorRelativePoint(Pose2d frameToAnchorPose,
                                                         double anchorForwardInches,
                                                         double anchorLeftInches) {
        Pose2d frameToPoint = anchorRelativePointInches(frameToAnchorPose, anchorForwardInches, anchorLeftInches);
        return bearingRadOfVector(frameToPoint.xInches, frameToPoint.yInches);
    }

    /**
     * Signed bearing error needed for {@code fromHeadingRad} to point at ({@code x}, {@code y}).
     *
     * @return error in [-pi, +pi], where + means “rotate CCW”.
     */
    public static double bearingErrorToPointRad(Pose2d from, double fromHeadingRad,
                                                double xInches, double yInches) {
        double desired = bearingToPointRad(from, xInches, yInches);
        return Pose2d.wrapToPi(desired - fromHeadingRad);
    }

    /**
     * Translation error (dx, dy) from {@code from} to ({@code x}, {@code y}) in the same frame.
     */
    public static Pose2d translationError(Pose2d from, double xInches, double yInches) {
        return new Pose2d(xInches - from.xInches, yInches - from.yInches, 0.0);
    }

    /**
     * Linear interpolation helper for geometry values.
     */
    public static double lerp(double a, double b, double t) {
        return MathUtil.lerp(a, b, t);
    }
}
