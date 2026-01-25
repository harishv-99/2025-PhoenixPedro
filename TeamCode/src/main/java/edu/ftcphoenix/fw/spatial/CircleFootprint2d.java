package edu.ftcphoenix.fw.spatial;

import edu.ftcphoenix.fw.core.math.MathUtil;

/**
 * Robot footprint modeled as a circle.
 *
 * <p>This is a useful approximation when you want simpler math than a rectangle, or when you
 * just want a conservative "near the zone" check.</p>
 */
public final class CircleFootprint2d implements RobotFootprint2d {

    /**
     * Radius in inches (non-negative).
     */
    public final double radiusInches;

    public CircleFootprint2d(double radiusInches) {
        this.radiusInches = Math.max(0.0, radiusInches);
    }

    @Override
    public String toString() {
        return "CircleFootprint2d{" +
                "radiusInches=" + MathUtil.fmt(radiusInches) +
                '}';
    }
}
