package edu.ftcsushi.fw.core.math;

import java.util.Arrays;
import java.util.Objects;
import java.util.function.DoubleBinaryOperator;

import edu.ftcsushi.fw.core.debug.DebugSink;

/**
 * Immutable two-input calibration table with bilinear interpolation on a rectangular grid.
 *
 * <p>Each row belongs to one first-axis coordinate; each column belongs to one second-axis
 * coordinate. Thus {@code values[xIndex][yIndex]} is the output measured at
 * {@code (xs[xIndex], ys[yIndex])}. Coordinate names x/y describe input order, not mandatory
 * spatial axes. The robot owns units, physical calibration and the meaning of each output.</p>
 *
 * <pre>
 * InterpolatingTable2D speedByOffset = InterpolatingTable2D.ofSorted(
 *         new double[] {24.0, 48.0},       // Forward inches: rows
 *         new double[] {-12.0, 0.0, 12.0}, // Left inches: columns
 *         new double[][] {
 *                 {3100.0, 3000.0, 3100.0},
 *                 {3500.0, 3400.0, 3500.0}});
 * double speed = speedByOffset.interpolate(36.0, 6.0); // 3250.0 ticks/second
 * </pre>
 *
 * <p>The numbers above are illustrative, not tuning recommendations. Finite inputs clamp
 * independently to the grid boundaries; inside a cell, interpolation blends the surrounding
 * outputs along the second axis, then along the first. One singleton axis reduces to 1D
 * interpolation, and two singleton axes produce a constant for finite inputs. Any non-finite
 * input returns {@link Double#NaN}, including along singleton axes.</p>
 *
 * <p>Queries are pure and allocate no per-query objects. This table has no clock, hardware,
 * freshness, fitting, missing-cell inference, extrapolation or physical acceptance policy.
 * A finite clamped result does not establish that the query is in a calibrated operating region.</p>
 *
 * @see InterpolatingTable1D
 */
public final class InterpolatingTable2D implements DoubleBinaryOperator {
    private final double[] xs;
    private final double[] ys;
    private final double[][] values;

    /** Retains only factory-owned, validated axis and row copies. */
    private InterpolatingTable2D(double[] xs, double[] ys, double[][] values) {
        this.xs = xs;
        this.ys = ys;
        this.values = values;
    }

    /**
     * Creates an independent table from sorted axes and a complete rectangular output grid.
     *
     * <p>Axes must be nonempty, finite and strictly increasing; signed zeros count as duplicate
     * coordinates. Each grid row corresponds to the first axis and must contain exactly one
     * finite output for every second-axis coordinate. Outputs need not be monotonic. Both axes,
     * the outer grid and every row are defensively copied; callers must not mutate inputs
     * concurrently with construction.</p>
     *
     * @param xs first-axis coordinates in robot-owned units; determines row order
     * @param ys second-axis coordinates in robot-owned units; determines column order
     * @param values outputs indexed as values[firstAxisIndex][secondAxisIndex]
     * @return immutable defensive capture of both axes and the complete grid
     * @throws NullPointerException if an axis, the grid or any row is null
     * @throws IllegalArgumentException if axes are empty, coordinates are non-finite or not
     *                                  strictly increasing, grid dimensions disagree with the
     *                                  axes, or any output is non-finite
     */
    public static InterpolatingTable2D ofSorted(double[] xs, double[] ys, double[][] values) {
        Objects.requireNonNull(xs, "xs is required");
        Objects.requireNonNull(ys, "ys is required");
        Objects.requireNonNull(values, "values is required");
        if (xs.length == 0) {
            throw new IllegalArgumentException("xs must contain at least one coordinate");
        }
        if (ys.length == 0) {
            throw new IllegalArgumentException("ys must contain at least one coordinate");
        }
        if (values.length != xs.length) {
            throw new IllegalArgumentException("values row count " + values.length
                    + " must match xs length " + xs.length + "; supply one row per x coordinate");
        }
        double[] xsCopy = xs.clone();
        double[] ysCopy = ys.clone();
        requireFiniteAxis(xsCopy, "x");
        requireFiniteAxis(ysCopy, "y");
        InterpolationMath.requireIncreasing(xsCopy, "x");
        InterpolationMath.requireIncreasing(ysCopy, "y");

        double[][] copy = new double[values.length][];
        for (int i = 0; i < copy.length; i++) {
            double[] row = Objects.requireNonNull(values[i], "values[" + i + "] row is required");
            if (row.length != ysCopy.length) {
                throw new IllegalArgumentException("values[" + i + "] row length " + row.length
                        + " must match ys length " + ysCopy.length
                        + "; supply one output per y coordinate");
            }
            copy[i] = row.clone();
            for (int j = 0; j < copy[i].length; j++) {
                if (!Double.isFinite(copy[i][j])) {
                    throw new IllegalArgumentException("values[" + i + "][" + j
                            + "] at authored row " + i + ", column " + j
                            + " must be finite, but was " + copy[i][j]
                            + "; provide a finite output value");
                }
            }
        }
        return new InterpolatingTable2D(xsCopy, ysCopy, copy);
    }

    /**
     * Returns the bilinearly interpolated output for two input coordinates.
     *
     * <p>Finite inputs clamp independently to their authored axis bounds. At an exact or clamped
     * grid node the authored output is returned, including its signed-zero bits. Elsewhere the
     * result is finite and between the surrounding outputs, even for extreme finite magnitudes.
     * A non-finite value in either input returns NaN; it is never treated as a boundary sample.</p>
     *
     * @param x first-axis query in the same units as xs
     * @param y second-axis query in the same units as ys
     * @return interpolated or clamped output, or NaN if either input is non-finite
     */
    public double interpolate(double x, double y) {
        if (!Double.isFinite(x) || !Double.isFinite(y)) {
            return Double.NaN;
        }
        int first = InterpolationMath.lowerIndex(xs, x);
        int second = InterpolationMath.lowerIndex(ys, y);
        double low = interpolateRow(first, second, y);
        if (first == xs.length - 1 || x <= xs[first]) {
            return low;
        }
        return InterpolationMath.blend(low, interpolateRow(first + 1, second, y),
                InterpolationMath.fraction(x, xs[first], xs[first + 1]));
    }

    /** Blends one captured row along the second axis, preserving exact boundary/node values. */
    private double interpolateRow(int row, int lowerColumn, double y) {
        if (lowerColumn == ys.length - 1 || y <= ys[lowerColumn]) {
            return values[row][lowerColumn];
        }
        return InterpolationMath.blend(values[row][lowerColumn], values[row][lowerColumn + 1],
                InterpolationMath.fraction(y, ys[lowerColumn], ys[lowerColumn + 1]));
    }

    /**
     * Standard Java functional adapter; ordinary table lookups use interpolate.
     *
     * @param left first-axis input in the same units as xs
     * @param right second-axis input in the same units as ys
     * @return exactly {@link #interpolate(double, double)}, including unavailable results
     */
    @Override
    public double applyAsDouble(double left, double right) {
        return interpolate(left, right);
    }

    /** Returns a passive description with both ordered axes and their row-first output grid. */
    @Override
    public String toString() {
        return "InterpolatingTable2D{xs=" + Arrays.toString(xs) + ", ys=" + Arrays.toString(ys)
                + ", values=" + Arrays.deepToString(values) + '}';
    }

    /**
     * Publishes only the total sample count and authored input bounds; never evaluates a query.
     *
     * @param dbg destination, or null for no action
     * @param prefix diagnostic key prefix; null or empty uses table
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) return;
        String p = (prefix == null || prefix.isEmpty()) ? "table" : prefix;
        dbg.addData(p + ".size", Long.valueOf((long) xs.length * ys.length))
                .addData(p + ".xMin", xs[0])
                .addData(p + ".xMax", xs[xs.length - 1])
                .addData(p + ".yMin", ys[0])
                .addData(p + ".yMax", ys[ys.length - 1]);
    }

    /** Reports the original axis and index before retaining invalid authored data. */
    private static void requireFiniteAxis(double[] axis, String name) {
        for (int i = 0; i < axis.length; i++) {
            if (!Double.isFinite(axis[i])) {
                throw new IllegalArgumentException(name + " at authored index " + i
                        + " must be finite, but was " + axis[i] + "; provide a finite "
                        + name + "-value");
            }
        }
    }
}
