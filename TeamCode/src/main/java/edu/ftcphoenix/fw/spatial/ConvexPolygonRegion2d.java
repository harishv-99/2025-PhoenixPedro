package edu.ftcphoenix.fw.spatial;

import java.util.Arrays;

import edu.ftcphoenix.fw.core.math.MathUtil;

/**
 * Convex polygon region with a signed-distance function.
 *
 * <h2>Assumptions</h2>
 * <ul>
 *   <li>The polygon is <b>convex</b>.</li>
 *   <li>Vertices are provided in consistent winding (CW or CCW). Phoenix will normalize to CCW.</li>
 *   <li>The polygon is not self-intersecting.</li>
 * </ul>
 *
 * <p>Signed distance convention: positive inside, negative outside.</p>
 */
public final class ConvexPolygonRegion2d implements ConvexRegion2d {

    private final double[] xs;
    private final double[] ys;
    private final int n;

    /**
     * Create a convex polygon from vertex arrays.
     *
     * @param xInches vertex x array (length >= 3)
     * @param yInches vertex y array (same length as x)
     */
    public ConvexPolygonRegion2d(double[] xInches, double[] yInches) {
        if (xInches == null || yInches == null) {
            throw new IllegalArgumentException("vertex arrays are required");
        }
        if (xInches.length != yInches.length) {
            throw new IllegalArgumentException("x/y arrays must have the same length");
        }
        if (xInches.length < 3) {
            throw new IllegalArgumentException("convex polygon requires at least 3 vertices");
        }

        this.n = xInches.length;
        this.xs = Arrays.copyOf(xInches, n);
        this.ys = Arrays.copyOf(yInches, n);

        // Normalize winding to CCW for consistent half-plane tests.
        if (signedArea2() < 0.0) {
            reverseInPlace(this.xs);
            reverseInPlace(this.ys);
        }
    }

    /**
     * Convenience constructor using interleaved x/y pairs.
     *
     * <p>Example: {@code new ConvexPolygonRegion2d(0,0, 10,0, 10,5, 0,5)}</p>
     */
    public ConvexPolygonRegion2d(double... xyPairsInches) {
        if (xyPairsInches == null) {
            throw new IllegalArgumentException("xyPairsInches is required");
        }
        if (xyPairsInches.length < 6 || (xyPairsInches.length % 2) != 0) {
            throw new IllegalArgumentException("expected an even number of values (x0,y0,x1,y1,...) with at least 3 vertices");
        }
        int count = xyPairsInches.length / 2;
        double[] x = new double[count];
        double[] y = new double[count];
        for (int i = 0; i < count; i++) {
            x[i] = xyPairsInches[2 * i];
            y[i] = xyPairsInches[2 * i + 1];
        }
        this.n = count;
        this.xs = x;
        this.ys = y;
        if (signedArea2() < 0.0) {
            reverseInPlace(this.xs);
            reverseInPlace(this.ys);
        }
    }

    @Override
    public double signedDistanceInches(double xInches, double yInches) {
        // Half-plane distances for inside test.
        double minHalfPlane = Double.POSITIVE_INFINITY;
        boolean inside = true;

        for (int i = 0; i < n; i++) {
            int j = (i + 1) % n;
            double xi = xs[i];
            double yi = ys[i];
            double xj = xs[j];
            double yj = ys[j];

            // Edge vector and inward unit normal (left normal for CCW polygon).
            double ex = xj - xi;
            double ey = yj - yi;
            double nx = -ey;
            double ny = ex;
            double nl = Math.hypot(nx, ny);
            if (nl <= 1e-9) {
                // Degenerate edge; ignore.
                continue;
            }
            nx /= nl;
            ny /= nl;

            double dx = xInches - xi;
            double dy = yInches - yi;
            double d = dx * nx + dy * ny; // signed distance to supporting line, positive inside.

            if (d < 0.0) {
                inside = false;
            }
            if (d < minHalfPlane) {
                minHalfPlane = d;
            }
        }

        if (inside) {
            // Inside: min distance to any edge.
            return minHalfPlane;
        }

        // Outside: Euclidean distance to polygon boundary (segments).
        double minDistSq = Double.POSITIVE_INFINITY;
        for (int i = 0; i < n; i++) {
            int j = (i + 1) % n;
            double distSq = distSqPointToSegment(xInches, yInches, xs[i], ys[i], xs[j], ys[j]);
            if (distSq < minDistSq) {
                minDistSq = distSq;
            }
        }
        double dist = Math.sqrt(minDistSq);
        return -dist;
    }

    private double signedArea2() {
        // 2x polygon area (shoelace). Positive for CCW.
        double sum = 0.0;
        for (int i = 0; i < n; i++) {
            int j = (i + 1) % n;
            sum += xs[i] * ys[j] - xs[j] * ys[i];
        }
        return sum;
    }

    private static void reverseInPlace(double[] arr) {
        for (int i = 0, j = arr.length - 1; i < j; i++, j--) {
            double tmp = arr[i];
            arr[i] = arr[j];
            arr[j] = tmp;
        }
    }

    private static double distSqPointToSegment(
            double px, double py,
            double ax, double ay,
            double bx, double by
    ) {
        double abx = bx - ax;
        double aby = by - ay;
        double apx = px - ax;
        double apy = py - ay;

        double abLenSq = abx * abx + aby * aby;
        if (abLenSq <= 1e-12) {
            // Degenerate segment.
            double dx = px - ax;
            double dy = py - ay;
            return dx * dx + dy * dy;
        }

        double t = (apx * abx + apy * aby) / abLenSq;
        t = MathUtil.clamp(t, 0.0, 1.0);

        double cx = ax + t * abx;
        double cy = ay + t * aby;
        double dx = px - cx;
        double dy = py - cy;
        return dx * dx + dy * dy;
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder("ConvexPolygonRegion2d{");
        sb.append("n=").append(n);
        sb.append(", vertices=");
        for (int i = 0; i < n; i++) {
            sb.append("(")
                    .append(MathUtil.fmt(xs[i]))
                    .append(",")
                    .append(MathUtil.fmt(ys[i]))
                    .append(")");
            if (i != n - 1) sb.append(",");
        }
        sb.append('}');
        return sb.toString();
    }
}
