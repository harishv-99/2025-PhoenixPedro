package edu.ftcphoenix.fw.spatial;

import edu.ftcphoenix.fw.core.math.MathUtil;

/**
 * A convex circular region in 2D.
 */
public final class CircleRegion2d implements ConvexRegion2d {

    /**
     * Circle center X in inches.
     */
    public final double centerXInches;

    /**
     * Circle center Y in inches.
     */
    public final double centerYInches;

    /**
     * Circle radius in inches (non-negative).
     */
    public final double radiusInches;

    /**
     * Creates a circular region centered at the supplied field coordinates.
     */
    public CircleRegion2d(double centerXInches, double centerYInches, double radiusInches) {
        this.centerXInches = centerXInches;
        this.centerYInches = centerYInches;
        this.radiusInches = Math.max(0.0, radiusInches);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public double signedDistanceInches(double xInches, double yInches) {
        double dx = xInches - centerXInches;
        double dy = yInches - centerYInches;
        double r = Math.hypot(dx, dy);
        return radiusInches - r;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        return "CircleRegion2d{" +
                "centerXInches=" + MathUtil.fmt(centerXInches) +
                ", centerYInches=" + MathUtil.fmt(centerYInches) +
                ", radiusInches=" + MathUtil.fmt(radiusInches) +
                '}';
    }
}
