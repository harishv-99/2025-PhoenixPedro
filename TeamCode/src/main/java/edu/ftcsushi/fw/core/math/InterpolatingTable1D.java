package edu.ftcsushi.fw.core.math;

import java.util.Objects;
import java.util.function.DoubleUnaryOperator;

import edu.ftcsushi.fw.core.debug.DebugSink;

/**
 * Immutable one-input calibration table with linear interpolation.
 *
 * <p>For example, robot code can supply distance in inches and corresponding flywheel velocities
 * in encoder ticks per second. The table owns only the numbers; the robot owns units, physical
 * calibration, freshness and permission to command a mechanism.</p>
 *
 * <pre>
 * InterpolatingTable1D speedByDistance = InterpolatingTable1D.ofSorted(
 *         new double[] {24.0, 48.0},
 *         new double[] {3000.0, 3400.0});
 * double speed = speedByDistance.interpolate(36.0); // 3200.0
 * </pre>
 *
 * <p>The numbers above are illustrative, not tuning recommendations. Construction copies the
 * arrays and requires finite values and strictly increasing input
 * coordinates. Finite queries clamp to the authored range; other interior queries blend the
 * surrounding outputs. A non-finite query returns {@link Double#NaN}, even for a one-point table.
 * A finite result proves a calculation, not physical accuracy or operation inside a calibrated
 * region. Queries are pure, allocate no per-query objects, and have no clock or hardware effects.</p>
 *
 * @see InterpolatingTable2D
 */
public final class InterpolatingTable1D implements DoubleUnaryOperator {

    private final double[] xs;
    private final double[] ys;

    /** Retains arrays already copied and validated by the sole factory. */
    private InterpolatingTable1D(double[] xs, double[] ys) {
        this.xs = xs;
        this.ys = ys;
    }

    /**
     * Creates an independent table from sorted input coordinates and corresponding output values.
     *
     * <p>Both arrays must be nonempty, have the same length, and contain only finite values.
     * Coordinates must be strictly increasing; signed zeros are duplicate coordinates. Output
     * values need not be monotonic. A one-point table is valid and constant for finite queries.</p>
     *
     * @param xs sorted finite input coordinates, in robot-owned units
     * @param values corresponding finite output values, in robot-owned units
     * @return an immutable defensive capture of both arrays
     * @throws NullPointerException if either array is null
     * @throws IllegalArgumentException if lengths differ, arrays are empty, a value is non-finite,
     *                                  or coordinates are not strictly increasing
     */
    public static InterpolatingTable1D ofSorted(double[] xs, double[] values) {
        Objects.requireNonNull(xs, "xs is required");
        Objects.requireNonNull(values, "values is required");
        if (xs.length != values.length) {
            throw new IllegalArgumentException("xs and values must have same length");
        }
        if (xs.length == 0) {
            throw new IllegalArgumentException("xs/values must contain at least one point");
        }
        double[] xsCopy = xs.clone();
        double[] valuesCopy = values.clone();
        for (int i = 0; i < xsCopy.length; i++) {
            requireFinite("x", i, xsCopy[i]);
            requireFinite("value", i, valuesCopy[i]);
        }
        InterpolationMath.requireIncreasing(xsCopy, "x");
        return new InterpolatingTable1D(xsCopy, valuesCopy);
    }

    /**
     * Returns the linearly interpolated output at one input coordinate.
     *
     * <p>Finite queries outside the input range return the exact endpoint value. An exact node
     * returns its authored output, including its signed-zero bits. Other finite queries return a
     * finite value between the surrounding outputs, including at extreme finite magnitudes.
     * Non-finite input returns {@link Double#NaN} instead of inventing an endpoint.</p>
     *
     * @param x input coordinate in the same units as the authored xs
     * @return interpolated or clamped output, or NaN for a non-finite query
     */
    public double interpolate(double x) {
        if (!Double.isFinite(x)) {
            return Double.NaN;
        }
        int lower = InterpolationMath.lowerIndex(xs, x);
        if (lower == xs.length - 1 || x <= xs[lower]) {
            return ys[lower];
        }
        double fraction = InterpolationMath.fraction(x, xs[lower], xs[lower + 1]);
        return InterpolationMath.blend(ys[lower], ys[lower + 1], fraction);
    }

    /**
     * Standard Java functional adapter; ordinary table lookups use {@link #interpolate(double)}.
     *
     * @param operand input coordinate in the authored input units
     * @return exactly the same result as interpolate, including NaN for unavailable input
     */
    @Override
    public double applyAsDouble(double operand) {
        return interpolate(operand);
    }

    /** Returns a passive description containing the authored input/output pairs. */
    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder("InterpolatingTable1D{");
        for (int i = 0; i < xs.length; i++) {
            if (i > 0) sb.append(", ");
            sb.append('(').append(xs[i]).append(", ").append(ys[i]).append(')');
        }
        return sb.append('}').toString();
    }

    /**
     * Publishes only authored size and input bounds; never samples or evaluates a query.
     *
     * @param dbg destination, or null for no action
     * @param prefix diagnostic key prefix; null or empty uses table
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "table" : prefix;
        dbg.addData(p + ".size", xs.length)
                .addData(p + ".xMin", xs[0])
                .addData(p + ".xMax", xs[xs.length - 1]);
    }

    /** Reports a non-finite authored value before it becomes retained table state. */
    private static void requireFinite(String component, int index, double value) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(component + " at authored index " + index
                    + " must be finite, but was " + value
                    + "; provide a finite authored number");
        }
    }
}
