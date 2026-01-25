package edu.ftcphoenix.fw.util;

import java.util.Objects;
import java.util.function.DoubleUnaryOperator;
import java.util.ArrayList;
import java.util.List;
import java.util.Arrays;

import edu.ftcphoenix.fw.debug.DebugSink;

/**
 * Simple immutable 1D lookup table with linear interpolation.
 *
 * <p>Typical usage:</p>
 * <ul>
 *   <li>Distance (inches) → shooter velocity (rad/s).</li>
 *   <li>Distance (inches) → arm angle (rad).</li>
 * </ul>
 *
 * <p>Semantics:</p>
 * <ul>
 *   <li>x-values must be strictly increasing.</li>
 *   <li>Values below the first x clamp to the first y.</li>
 *   <li>Values above the last x clamp to the last y.</li>
 *   <li>Values in-between are linearly interpolated.</li>
 * </ul>
 */
public final class InterpolatingTable1D implements DoubleUnaryOperator {

    private final double[] xs;
    private final double[] ys;

    private InterpolatingTable1D(double[] xs, double[] ys) {
        this.xs = xs;
        this.ys = ys;
    }

    /**
     * Create a table from sorted x-values and corresponding y-values.
     *
     * <p>Preconditions:</p>
     * <ul>
     *   <li>{@code xs.length == ys.length}</li>
     *   <li>{@code xs.length >= 1}</li>
     *   <li>{@code xs} must be strictly increasing (no duplicates).</li>
     * </ul>
     *
     * @param xs sorted x-values
     * @param ys corresponding y-values
     * @return a new {@link InterpolatingTable1D}
     */
    public static InterpolatingTable1D ofSorted(double[] xs, double[] ys) {
        Objects.requireNonNull(xs, "xs is required");
        Objects.requireNonNull(ys, "ys is required");
        if (xs.length != ys.length) {
            throw new IllegalArgumentException("xs and ys must have same length");
        }
        if (xs.length == 0) {
            throw new IllegalArgumentException("xs/ys must contain at least one point");
        }

        double[] xsCopy = xs.clone();
        double[] ysCopy = ys.clone();

        // Verify strict monotonicity
        for (int i = 1; i < xsCopy.length; i++) {
            if (!(xsCopy[i] > xsCopy[i - 1])) {
                throw new IllegalArgumentException("xs must be strictly increasing");
            }
        }

        return new InterpolatingTable1D(xsCopy, ysCopy);
    }

    /**
     * Create a table from arbitrary x-values and corresponding y-values.
     *
     * <p>The points are sorted by x internally. Duplicate x-values are not
     * allowed.</p>
     *
     * @param xs unsorted x-values
     * @param ys corresponding y-values
     * @return a new {@link InterpolatingTable1D}
     */
    public static InterpolatingTable1D ofUnsorted(double[] xs, double[] ys) {
        Objects.requireNonNull(xs, "xs is required");
        Objects.requireNonNull(ys, "ys is required");
        if (xs.length != ys.length) {
            throw new IllegalArgumentException("xs and ys must have same length");
        }
        if (xs.length == 0) {
            throw new IllegalArgumentException("xs/ys must contain at least one point");
        }

        int n = xs.length;
        double[] xsCopy = xs.clone();
        double[] ysCopy = ys.clone();

        // Sort by xs, keeping ys aligned via index indirection.
        Integer[] indices = new Integer[n];
        for (int i = 0; i < n; i++) {
            indices[i] = i;
        }
        Arrays.sort(indices, (i, j) -> Double.compare(xsCopy[i], xsCopy[j]));

        double[] sortedX = new double[n];
        double[] sortedY = new double[n];
        for (int i = 0; i < n; i++) {
            sortedX[i] = xsCopy[indices[i]];
            sortedY[i] = ysCopy[indices[i]];
        }

        return ofSorted(sortedX, sortedY);
    }

    /**
     * Convenience factory: build from sorted (x,y) pairs.
     *
     * <p>Example:</p>
     * <pre>
     * InterpolatingTable1D table = InterpolatingTable1D.ofSortedPairs(
     *      24.0, 180.0,
     *      30.0, 190.0,
     *      36.0, 205.0,
     *      42.0, 220.0
     * );
     * </pre>
     *
     * @param xsAndYs flattened pairs: x0, y0, x1, y1, ...
     */
    public static InterpolatingTable1D ofSortedPairs(double... xsAndYs) {
        Objects.requireNonNull(xsAndYs, "xsAndYs is required");
        if (xsAndYs.length == 0 || xsAndYs.length % 2 != 0) {
            throw new IllegalArgumentException(
                    "xsAndYs must contain an even number of values (x0, y0, x1, y1, ...)");
        }
        int n = xsAndYs.length / 2;
        double[] xs = new double[n];
        double[] ys = new double[n];
        int idx = 0;
        for (int i = 0; i < n; i++) {
            xs[i] = xsAndYs[idx++];
            ys[i] = xsAndYs[idx++];
        }
        return ofSorted(xs, ys);
    }

    /**
     * Builder for readable table declarations in robot code.
     *
     * <pre>{@code
     * InterpolatingTable1D table = InterpolatingTable1D.builder()
     *         .add(24.0, 180.0)
     *         .add(30.0, 190.0)
     *         .add(36.0, 205.0)
     *         .add(42.0, 220.0)
     *         .buildSorted();
     * }</pre>
     */
    public static Builder builder() {
        return new Builder();
    }

    /**
     * Evaluate the table at the given x using linear interpolation
     * with clamping to the end values.
     *
     * @param x query x-value
     * @return interpolated y-value
     */
    public double interpolate(double x) {
        int n = xs.length;
        if (n == 1) {
            return ys[0];
        }

        // Clamp below/above range
        if (x <= xs[0]) {
            return ys[0];
        }
        int last = n - 1;
        if (x >= xs[last]) {
            return ys[last];
        }

        // Binary search for segment
        int idx = Arrays.binarySearch(xs, x);
        if (idx >= 0) {
            // Exact match
            return ys[idx];
        }

        // Insertion point of first element greater than x
        int insertionPoint = -idx - 1;
        int i0 = insertionPoint - 1;
        int i1 = insertionPoint;

        double x0 = xs[i0];
        double x1 = xs[i1];
        double y0 = ys[i0];
        double y1 = ys[i1];

        if (x1 == x0) {
            // Should not happen if xs is strictly increasing, but guard anyway.
            return y0;
        }

        double t = (x - x0) / (x1 - x0);
        // Use shared interpolation helper for consistency.
        return MathUtil.lerp(y0, y1, t);
    }

    /**
     * Functional interface integration: treat this table as a DoubleUnaryOperator.
     */
    @Override
    public double applyAsDouble(double operand) {
        return interpolate(operand);
    }

    /**
     * @return number of calibration points.
     */
    public int size() {
        return xs.length;
    }

    /**
     * @return defensive copy of x-samples.
     */
    public double[] xs() {
        return xs.clone();
    }

    /**
     * @return defensive copy of y-samples.
     */
    public double[] ys() {
        return ys.clone();
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder("InterpolatingTable1D{");
        for (int i = 0; i < xs.length; i++) {
            if (i > 0) sb.append(", ");
            sb.append('(').append(xs[i]).append(", ").append(ys[i]).append(')');
        }
        sb.append('}');
        return sb.toString();
    }

    /**
     * Builder for InterpolatingTable1D.
     *
     * <p>Note: x-values must be added in strictly increasing order.
     * This is enforced when {@link #buildSorted()} is called.</p>
     */
    public static final class Builder {
        private final List<Double> xs = new ArrayList<>();
        private final List<Double> ys = new ArrayList<>();

        public Builder add(double x, double y) {
            xs.add(x);
            ys.add(y);
            return this;
        }

        /**
         * Build a table from the added points, assuming they are already
         * sorted by x and strictly increasing.
         */
        public InterpolatingTable1D buildSorted() {
            int n = xs.size();
            if (n == 0) {
                throw new IllegalStateException("No points added to table");
            }

            double[] xsArr = new double[n];
            double[] ysArr = new double[n];
            for (int i = 0; i < n; i++) {
                xsArr[i] = xs.get(i);
                ysArr[i] = ys.get(i);
            }
            return InterpolatingTable1D.ofSorted(xsArr, ysArr);
        }
    }

    /**
     * Emit a small summary of this table (size and range).
     *
     * @param dbg    debug sink (may be {@code null}; if null, no output is produced)
     * @param prefix base key prefix, e.g. "lookup.shooterRpm"
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "table" : prefix;
        int n = xs.length;
        dbg.addData(p + ".size", n)
                .addData(p + ".xMin", xs[0])
                .addData(p + ".xMax", xs[n - 1]);
    }
}
