package edu.ftcphoenix.fw.core.geometry;

import edu.ftcphoenix.fw.core.math.MathUtil;

/**
 * Immutable 2D pose in a right-handed, robot/field-friendly coordinate system.
 *
 * <p>A {@code Pose2d} represents a position on the floor plane plus a heading
 * (orientation) about the vertical axis. It is used throughout the framework
 * to describe the pose of the robot, AprilTags in the field layout, and other
 * objects constrained to the floor plane.</p>
 *
 * <h2>Coordinate conventions</h2>
 *
 * <ul>
 *   <li><strong>Units:</strong> distances are in inches, angles are in radians.</li>
 *   <li><strong>Axes:</strong>
 *     <ul>
 *       <li>{@code xInches} and {@code yInches} are the X/Y translation components of the
 *           pose in <em>whatever frame</em> you are working in.</li>
 *       <li>{@code Pose2d} itself does not assign a universal real-world meaning to +X/+Y.
 *           That meaning comes from the coordinate frame the pose is expressed in.</li>
 *       <li><b>Robot/mechanism frames</b> in Phoenix are typically expressed as +X forward,
 *           +Y left (+Z up in 3D).</li>
 *       <li><b>Field-centric poses</b> in Phoenix are expressed in the <b>FTC Field Coordinate
 *           System</b> for the current season (see {@code TagLayout} and {@code PoseEstimate}).</li>
 *     </ul>
 *   </li>
 *   <li><strong>Heading:</strong>
 *     <ul>
 *       <li>{@code headingRad} is measured CCW from +X, following the right-hand rule about +Z.</li>
 *       <li>{@code headingRad = 0} -> "facing +X".</li>
 *       <li>{@code headingRad = +pi/2} -> "facing +Y".</li>
 *     </ul>
 *   </li>
 * </ul>
 */
public final class Pose2d {

    /**
     * X position on the floor plane, in inches (frame +X).
     */
    public final double xInches;

    /**
     * Y position on the floor plane, in inches (frame +Y).
     */
    public final double yInches;

    /**
     * Heading about the vertical axis, in radians, measured CCW from +X.
     *
     * <p>Callers may provide any real value; no normalization is performed in
     * the constructor. Use {@link #wrapToPi(double)} if you need a canonical
     * angle in the range [-pi, +pi].</p>
     */
    public final double headingRad;

    /**
     * A constant zero pose (x=0, y=0, heading=0).
     *
     * <p>This is safe to share as a singleton because {@code Pose2d} is immutable.</p>
     */
    private static final Pose2d ZERO = new Pose2d(0.0, 0.0, 0.0);

    /**
     * Returns the zero pose (x=0, y=0, heading=0).
     */
    public static Pose2d zero() {
        return ZERO;
    }

    /**
     * Constructs a new immutable 2D pose.
     *
     * @param xInches    X position on the floor plane, in inches (frame +X)
     * @param yInches    Y position on the floor plane, in inches (frame +Y)
     * @param headingRad heading about the vertical axis, in radians
     */
    public Pose2d(double xInches, double yInches, double headingRad) {
        this.xInches = xInches;
        this.yInches = yInches;
        this.headingRad = headingRad;
    }

    /**
     * Euclidean length of the translation component in inches.
     *
     * <p>This is {@code sqrt(x^2 + y^2)} and ignores {@link #headingRad}.</p>
     */
    public double translationNormInches() {
        return Math.hypot(xInches, yInches);
    }

    /**
     * Compose this transform with {@code next} (apply {@code this}, then {@code next}).
     *
     * <p>If this pose is A→B and {@code next} is B→C, the result is A→C.</p>
     *
     * <p>This is the 2D equivalent of {@link Pose3d#then(Pose3d)} and is
     * extremely useful when working with:</p>
     *
     * <ul>
     *   <li>Field→robot, robot→camera, camera→target chains, and</li>
     *   <li>Field layouts (e.g., field→tag, tag→goal-point).</li>
     * </ul>
     *
     * <p><b>Important:</b> Unlike some pose libraries, Phoenix does not normalize
     * the resulting heading. Call {@link #normalizedHeading()} if you want a
     * canonical heading in [-π, +π].</p>
     *
     * @param next transform to apply after this one (non-null)
     * @return composed transform
     */
    public Pose2d then(Pose2d next) {
        if (next == null) {
            throw new IllegalArgumentException("next is required");
        }

        // 2D rigid transform composition:
        // p_out = p_this + R(heading_this) * p_next
        double cos = Math.cos(this.headingRad);
        double sin = Math.sin(this.headingRad);

        double xOut = this.xInches + cos * next.xInches - sin * next.yInches;
        double yOut = this.yInches + sin * next.xInches + cos * next.yInches;
        double headingOut = this.headingRad + next.headingRad;
        return new Pose2d(xOut, yOut, headingOut);
    }

    /**
     * Return the inverse transform.
     *
     * <p>If this pose is A→B, the inverse is B→A.</p>
     *
     * <p>This is the 2D equivalent of {@link Pose3d#inverse()}.</p>
     */
    public Pose2d inverse() {
        // Inverse of 2D rigid transform:
        // R_inv = R(-heading)
        // t_inv = -R_inv * t
        double cos = Math.cos(this.headingRad);
        double sin = Math.sin(this.headingRad);

        // R^T * t (since R is orthonormal):
        double xRt = cos * this.xInches + sin * this.yInches;
        double yRt = -sin * this.xInches + cos * this.yInches;

        return new Pose2d(-xRt, -yRt, -this.headingRad);
    }

    /**
     * Creates a new pose with the same heading but different translation.
     */
    public Pose2d withTranslation(double xInches, double yInches) {
        return new Pose2d(xInches, yInches, headingRad);
    }

    /**
     * Creates a new pose with the same translation but different heading.
     */
    public Pose2d withHeading(double headingRad) {
        return new Pose2d(xInches, yInches, headingRad);
    }

    /**
     * Returns a new {@code Pose2d} with the same x/y but heading wrapped
     * into the range [-pi, +pi].
     *
     * @return a new {@code Pose2d} whose heading is normalized to [-pi, +pi]
     */
    public Pose2d normalizedHeading() {
        return new Pose2d(xInches, yInches, wrapToPi(headingRad));
    }

    /**
     * Compute the Euclidean distance in the floor plane to another pose.
     *
     * <p>This ignores heading and only considers (x, y):</p>
     *
     * <pre>
     * sqrt((other.xInches - xInches)^2 + (other.yInches - yInches)^2)
     * </pre>
     *
     * @param other another pose in the same field frame
     * @return straight-line distance between this pose and {@code other}, in inches
     */
    public double distanceTo(Pose2d other) {
        double dx = other.xInches - this.xInches;
        double dy = other.yInches - this.yInches;
        return Math.hypot(dx, dy);
    }

    /**
     * Compute the smallest signed heading error from this pose's heading
     * to another pose's heading.
     *
     * <p>The result is normalized to the range [-pi, +pi] and represents the
     * rotation needed to go from {@code this.headingRad} to
     * {@code other.headingRad}:</p>
     *
     * <pre>
     * headingErrorTo(other) = wrapToPi(other.headingRad - this.headingRad)
     * </pre>
     *
     * <p>Positive values indicate that {@code other} is rotated CCW (to the
     * left) relative to this pose; negative values indicate CW.</p>
     *
     * @param other another pose in the same frame
     * @return heading error (other - this), wrapped to [-pi, +pi]
     */
    public double headingErrorTo(Pose2d other) {
        return wrapToPi(other.headingRad - this.headingRad);
    }

    /**
     * Normalize an angle in radians into the range [-pi, +pi].
     *
     * <p>This method exists for convenience in pose/math code, but the canonical
     * implementation lives in {@link MathUtil#wrapToPi(double)} so all framework
     * code shares identical behavior.</p>
     *
     * @param angleRad angle in radians (any real value)
     * @return equivalent angle in radians, wrapped to [-pi, +pi]
     */
    public static double wrapToPi(double angleRad) {
        return MathUtil.wrapToPi(angleRad);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        return "Pose2d{" +
                "xInches=" + xInches +
                ", yInches=" + yInches +
                ", headingRad=" + headingRad +
                '}';
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean equals(Object o) {
        if (!(o instanceof Pose2d)) return false;
        Pose2d other = (Pose2d) o;
        return Double.compare(this.xInches, other.xInches) == 0
                && Double.compare(this.yInches, other.yInches) == 0
                && Double.compare(this.headingRad, other.headingRad) == 0;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public int hashCode() {
        long bits = 7L;
        bits = 31L * bits + Double.doubleToLongBits(xInches);
        bits = 31L * bits + Double.doubleToLongBits(yInches);
        bits = 31L * bits + Double.doubleToLongBits(headingRad);
        return (int) (bits ^ (bits >>> 32));
    }
}
