package edu.ftcphoenix.fw.spatial;

import edu.ftcphoenix.fw.core.math.MathUtil;

/**
 * Convex axis-aligned rectangular region (AABB).
 *
 * <p>The rectangle sides are aligned with the region's X/Y axes. This is a common and
 * very fast special-case of a convex polygon.</p>
 */
public final class AxisAlignedBoxRegion2d implements ConvexRegion2d {

    /**
     * Min X (inches).
     */
    public final double minXInches;

    /**
     * Max X (inches).
     */
    public final double maxXInches;

    /**
     * Min Y (inches).
     */
    public final double minYInches;

    /**
     * Max Y (inches).
     */
    public final double maxYInches;

    /**
     * Creates an axis-aligned box region from two X bounds and two Y bounds.
     *
     * <p>The constructor tolerates either ordering and normalizes the stored min/max values.</p>
     */
    public AxisAlignedBoxRegion2d(double minXInches, double maxXInches, double minYInches, double maxYInches) {
        this.minXInches = Math.min(minXInches, maxXInches);
        this.maxXInches = Math.max(minXInches, maxXInches);
        this.minYInches = Math.min(minYInches, maxYInches);
        this.maxYInches = Math.max(minYInches, maxYInches);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public double signedDistanceInches(double xInches, double yInches) {
        // If inside, return +min distance to a wall.
        boolean inside = (xInches >= minXInches && xInches <= maxXInches && yInches >= minYInches && yInches <= maxYInches);
        if (inside) {
            double dx = Math.min(xInches - minXInches, maxXInches - xInches);
            double dy = Math.min(yInches - minYInches, maxYInches - yInches);
            return Math.min(dx, dy);
        }

        // Outside: distance to the box.
        double clampedX = MathUtil.clamp(xInches, minXInches, maxXInches);
        double clampedY = MathUtil.clamp(yInches, minYInches, maxYInches);
        double dx = xInches - clampedX;
        double dy = yInches - clampedY;
        double dist = Math.hypot(dx, dy);
        return -dist;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        return "AxisAlignedBoxRegion2d{" +
                "minXInches=" + MathUtil.fmt(minXInches) +
                ", maxXInches=" + MathUtil.fmt(maxXInches) +
                ", minYInches=" + MathUtil.fmt(minYInches) +
                ", maxYInches=" + MathUtil.fmt(maxYInches) +
                '}';
    }
}
