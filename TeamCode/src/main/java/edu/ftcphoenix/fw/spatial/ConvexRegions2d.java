package edu.ftcphoenix.fw.spatial;

import java.util.Objects;

import edu.ftcphoenix.fw.core.math.MathUtil;

/**
 * Factory and helpers for <b>convex</b> 2D regions.
 */
public final class ConvexRegions2d {

    private ConvexRegions2d() {
        // utility
    }

    /**
     * Axis-aligned rectangle (AABB) region.
     */
    public static ConvexRegion2d aabb(double minXInches, double maxXInches, double minYInches, double maxYInches) {
        return new AxisAlignedBoxRegion2d(minXInches, maxXInches, minYInches, maxYInches);
    }

    /**
     * Circle region.
     */
    public static ConvexRegion2d circle(double centerXInches, double centerYInches, double radiusInches) {
        return new CircleRegion2d(centerXInches, centerYInches, radiusInches);
    }

    /**
     * Convex polygon region.
     *
     * <p>Vertices may be CW or CCW; the implementation will normalize to CCW.</p>
     */
    public static ConvexRegion2d convexPolygon(double... xyPairsInches) {
        return new ConvexPolygonRegion2d(xyPairsInches);
    }

    /**
     * Wrap a region so its signed distance is clamped to a symmetric range.
     *
     * <p>This is sometimes useful for telemetry/readability when you only care about a small band
     * around the boundary.</p>
     */
    public static ConvexRegion2d clamped(final ConvexRegion2d region, final double clampAbsInches) {
        Objects.requireNonNull(region, "region");
        final double c = Math.abs(clampAbsInches);
        return (x, y) -> MathUtil.clamp(region.signedDistanceInches(x, y), -c, c);
    }
}
