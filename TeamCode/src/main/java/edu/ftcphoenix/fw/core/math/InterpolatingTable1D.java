package edu.ftcphoenix.fw.core.math;

import java.util.Arrays;
import java.util.Objects;
import java.util.function.DoubleUnaryOperator;

import edu.ftcphoenix.fw.core.debug.DebugSink;

/**
 * Immutable one-dimensional calibration table with linear interpolation.
 *
 * <p>Typical usage:</p>
 * <ul>
 *   <li>Distance (inches) → shooter velocity (rad/s).</li>
 *   <li>Distance (inches) → arm angle (rad).</li>
 * </ul>
 *
 * <p>Semantics:</p>
 * <ul>
 *   <li>Every authored x- and y-value must be finite.</li>
 *   <li>Sorted x-values must be strictly increasing.</li>
 *   <li>Finite queries below or above the authored range clamp to the corresponding y-value.</li>
 *   <li>Finite queries inside the range produce a finite, piecewise-linear result.</li>
 *   <li>A non-finite runtime query returns {@link Double#NaN} rather than inventing an endpoint
 *       value.</li>
 * </ul>
 *
 * <p>Factories defensively capture their input. Construction validates authored calibration facts;
 * a non-finite query instead represents unavailable runtime evidence.</p>
 */
public final class InterpolatingTable1D implements DoubleUnaryOperator {

    private final double[] xs;
    private final double[] ys;

    private InterpolatingTable1D(double[] xs, double[] ys) {
        this.xs = xs;
        this.ys = ys;
    }

    /**
     * Creates a table from already-sorted x-values and their corresponding y-values.
     *
     * <p>Preconditions:</p>
     * <ul>
     *   <li>{@code xs.length == ys.length}</li>
     *   <li>{@code xs.length >= 1}</li>
     *   <li>Every x- and y-value is finite.</li>
     *   <li>{@code xs} is strictly increasing (no duplicates, including signed zero).</li>
     * </ul>
     *
     * <p>Both arrays are copied before they are retained.</p>
     *
     * @param xs sorted finite x-values
     * @param ys corresponding finite y-values
     * @return an independent immutable table
     * @throws NullPointerException if either array is {@code null}
     * @throws IllegalArgumentException if the arrays have different or zero lengths, a value is
     *                                  non-finite, or the x-values are not strictly increasing
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
        validateFiniteSamples(xsCopy, ysCopy);

        for (int i = 1; i < xsCopy.length; i++) {
            if (!(xsCopy[i] > xsCopy[i - 1])) {
                throw sortedOrderFailure(i - 1, xsCopy[i - 1], i, xsCopy[i]);
            }
        }

        return new InterpolatingTable1D(xsCopy, ysCopy);
    }

    /**
     * Creates a table from arbitrarily ordered x-values and their corresponding y-values.
     *
     * <p>The captured samples are sorted by x internally while preserving each x/y pair. Every
     * value must be finite, and duplicate x-values are rejected. Validation errors identify the
     * original authored array indices, not positions after sorting.</p>
     *
     * @param xs finite x-values in any order
     * @param ys corresponding finite y-values
     * @return an independent immutable table ordered by x
     * @throws NullPointerException if either array is {@code null}
     * @throws IllegalArgumentException if the arrays have different or zero lengths, a value is
     *                                  non-finite, or two x-values are duplicates
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
        validateFiniteSamples(xsCopy, ysCopy);

        // Sort by xs, keeping ys aligned via index indirection.
        Integer[] indices = new Integer[n];
        for (int i = 0; i < n; i++) {
            indices[i] = i;
        }
        Arrays.sort(indices, (i, j) -> Double.compare(xsCopy[i], xsCopy[j]));

        double[] sortedX = new double[n];
        double[] sortedY = new double[n];
        for (int i = 0; i < n; i++) {
            int authoredIndex = indices[i];
            sortedX[i] = xsCopy[authoredIndex];
            sortedY[i] = ysCopy[authoredIndex];
            if (i > 0 && !(sortedX[i] > sortedX[i - 1])) {
                int previousAuthoredIndex = indices[i - 1];
                throw duplicateFailure(previousAuthoredIndex, sortedX[i - 1],
                        authoredIndex, sortedX[i]);
            }
        }

        return new InterpolatingTable1D(sortedX, sortedY);
    }

    /**
     * Creates a table from sorted, flattened {@code (x, y)} pairs.
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
     * <p>The supplied varargs array is captured into independent x/y arrays. Every value must be
     * finite, and x-values must be strictly increasing.</p>
     *
     * @param xsAndYs flattened pairs: x0, y0, x1, y1, ...
     * @return an independent immutable table
     * @throws NullPointerException if {@code xsAndYs} is {@code null}
     * @throws IllegalArgumentException if the input is empty or contains an incomplete odd value,
     *                                  a value is non-finite, or the x-values are not strictly
     *                                  increasing
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
     * Evaluates this table at {@code x} using finite linear interpolation and endpoint clamping.
     *
     * <p>A finite query below or above the calibration range returns the exact corresponding
     * endpoint y-value. An exact authored x-value returns its exact y-value. Any other finite query
     * returns a finite value between the surrounding y-values. A non-finite query is unavailable
     * runtime evidence and returns {@link Double#NaN}, including for a one-point table.</p>
     *
     * @param x query x-value
     * @return a finite interpolated or clamped y-value, or {@link Double#NaN} when {@code x} is
     *         non-finite
     */
    public double interpolate(double x) {
        if (!Double.isFinite(x)) {
            return Double.NaN;
        }

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

        double t = interpolationFraction(x, x0, x1);
        return interpolateFinite(y0, y1, t);
    }

    /**
     * Evaluates this table through the standard {@link DoubleUnaryOperator} adapter.
     *
     * @param operand query x-value
     * @return the same result as {@link #interpolate(double)}, including {@link Double#NaN} for a
     *         non-finite query
     */
    @Override
    public double applyAsDouble(double operand) {
        return interpolate(operand);
    }

    /**
     * {@inheritDoc}
     */
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

    /** Validates finite samples at their original authored indices. */
    private static void validateFiniteSamples(double[] xs, double[] ys) {
        for (int i = 0; i < xs.length; i++) {
            if (!Double.isFinite(xs[i])) {
                throw nonFiniteFailure("x", i, xs[i]);
            }
            if (!Double.isFinite(ys[i])) {
                throw nonFiniteFailure("y", i, ys[i]);
            }
        }
    }

    /** Creates an actionable diagnostic for one non-finite authored component. */
    private static IllegalArgumentException nonFiniteFailure(String component,
                                                             int authoredIndex,
                                                             double value) {
        return new IllegalArgumentException(component + " at authored index " + authoredIndex
                + " must be finite, but was " + value
                + "; provide a finite " + component + "-value");
    }

    /** Creates an actionable diagnostic for adjacent authored x-values that are not ordered. */
    private static IllegalArgumentException sortedOrderFailure(int previousAuthoredIndex,
                                                               double previousValue,
                                                               int authoredIndex,
                                                               double value) {
        return new IllegalArgumentException("x-values must be strictly increasing: x at authored "
                + "index " + authoredIndex + " was " + value + " but must be greater than x at "
                + "authored index " + previousAuthoredIndex + " (" + previousValue + "); reorder "
                + "or remove the conflicting x-value");
    }

    /** Creates an actionable diagnostic that retains both pre-sort authored indices. */
    private static IllegalArgumentException duplicateFailure(int firstAuthoredIndex,
                                                             double firstValue,
                                                             int secondAuthoredIndex,
                                                             double secondValue) {
        return new IllegalArgumentException("duplicate x-values at authored indices "
                + firstAuthoredIndex + " (" + firstValue + ") and " + secondAuthoredIndex + " ("
                + secondValue + "); provide one sample per unique x-value");
    }

    /**
     * Computes the finite in-segment fraction without overflowing an opposite-sign x span.
     */
    private static double interpolationFraction(double x, double x0, double x1) {
        double span = x1 - x0;
        double fraction;
        if (Double.isFinite(span)) {
            fraction = (x - x0) / span;
        } else {
            fraction = (x * 0.5 - x0 * 0.5) / (x1 * 0.5 - x0 * 0.5);
        }

        // The query is already proven interior; clamp only floating-point rounding drift.
        return Math.max(0.0, Math.min(1.0, fraction));
    }

    /**
     * Interpolates finite endpoints without overflowing a valid convex result.
     */
    private static double interpolateFinite(double y0, double y1, double fraction) {
        double delta = y1 - y0;
        if (!Double.isFinite(delta)) {
            return (1.0 - fraction) * y0 + fraction * y1;
        }
        if (fraction <= 0.5) {
            return y0 + fraction * delta;
        }
        return y1 - (1.0 - fraction) * delta;
    }
}
