package edu.ftcsushi.fw.actuation;

/**
 * Legal scalar travel range for a mechanism in the same units as its {@link Plant} target.
 *
 * <p>A valid range has exactly one of four explicit shapes: finite {@link #bounded(double, double)},
 * finite-lower-bound {@link #boundedFrom(double)}, finite-upper-bound {@link #boundedTo(double)},
 * or {@link #unbounded()}. Caller-supplied bounds must be finite. Invalid ranges are useful before
 * homing/calibration is complete; a target planner should refuse to command a mechanism when its
 * range is invalid. Plant construction requires a valid configured range; a position Plant that
 * still needs a reference publishes a temporary invalid runtime range itself. Every legal target
 * remains finite, including for one-sided or unbounded shapes.</p>
 */
public final class ScalarRange {

    public final boolean valid;
    public final double minValue;
    public final double maxValue;
    public final String reason;

    private ScalarRange(boolean valid, double minValue, double maxValue, String reason) {
        this.valid = valid;
        this.minValue = minValue;
        this.maxValue = maxValue;
        this.reason = reason;
    }

    /**
     * Returns the valid range with neither a lower nor an upper bound.
     */
    public static ScalarRange unbounded() {
        return new ScalarRange(true, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, "unbounded");
    }

    /**
     * Returns a valid range with inclusive finite lower and upper bounds.
     *
     * <p>Equal endpoints are allowed. Supplied endpoint values, including the sign bit of zero, are
     * retained without normalization.</p>
     *
     * @param minValue finite inclusive lower bound
     * @param maxValue finite inclusive upper bound
     * @throws IllegalArgumentException if either endpoint is non-finite or {@code minValue} is
     *                                  greater than {@code maxValue}
     */
    public static ScalarRange bounded(double minValue, double maxValue) {
        requireFinite("bounded", "minValue", minValue);
        requireFinite("bounded", "maxValue", maxValue);
        if (minValue > maxValue) {
            throw new IllegalArgumentException(
                    "ScalarRange.bounded requires minValue <= maxValue, but minValue="
                            + minValue + " and maxValue=" + maxValue);
        }
        return new ScalarRange(true, minValue, maxValue, "bounded");
    }

    /**
     * Returns a valid range extending inclusively from a finite lower bound with no upper bound.
     *
     * <p>The supplied endpoint value, including the sign bit of zero, is retained without
     * normalization.</p>
     *
     * @param minValue finite inclusive lower bound
     * @throws IllegalArgumentException if {@code minValue} is non-finite
     */
    public static ScalarRange boundedFrom(double minValue) {
        requireFinite("boundedFrom", "minValue", minValue);
        return new ScalarRange(true, minValue, Double.POSITIVE_INFINITY, "boundedFrom");
    }

    /**
     * Returns a valid range extending inclusively to a finite upper bound with no lower bound.
     *
     * <p>The supplied endpoint value, including the sign bit of zero, is retained without
     * normalization.</p>
     *
     * @param maxValue finite inclusive upper bound
     * @throws IllegalArgumentException if {@code maxValue} is non-finite
     */
    public static ScalarRange boundedTo(double maxValue) {
        requireFinite("boundedTo", "maxValue", maxValue);
        return new ScalarRange(true, Double.NEGATIVE_INFINITY, maxValue, "boundedTo");
    }

    /**
     * Returns an invalid range with an actionable reason, commonly used before homing.
     */
    public static ScalarRange invalid(String reason) {
        return new ScalarRange(false, Double.NaN, Double.NaN, reason != null ? reason : "invalid range");
    }

    /**
     * Returns true if {@code value} is finite and lies inside this valid range.
     */
    public boolean contains(double value) {
        return valid && Double.isFinite(value) && value >= minValue && value <= maxValue;
    }

    /**
     * Clamps a finite value into this valid range.
     *
     * @return the clamped finite value, or {@link Double#NaN} if this range is invalid or
     *         {@code value} is non-finite
     */
    public double clamp(double value) {
        if (!valid || !Double.isFinite(value)) {
            return Double.NaN;
        }
        return Math.max(minValue, Math.min(maxValue, value));
    }

    /**
     * Returns true only for the canonical valid range from {@link #unbounded()}.
     */
    public boolean isUnbounded() {
        return valid
                && minValue == Double.NEGATIVE_INFINITY
                && maxValue == Double.POSITIVE_INFINITY;
    }

    /**
     * Returns the finite center of the range without overflowing finite endpoints, or NaN if this
     * range is invalid or either side is unbounded.
     */
    public double center() {
        if (!valid || !Double.isFinite(minValue) || !Double.isFinite(maxValue)) {
            return Double.NaN;
        }
        if (minValue < 0.0 && maxValue >= 0.0) {
            return (minValue + maxValue) / 2.0;
        }
        return minValue + (maxValue - minValue) / 2.0;
    }

    private static void requireFinite(String factory, String argument, double value) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(
                    "ScalarRange." + factory + " requires finite " + argument
                            + ", but " + argument + "=" + value);
        }
    }

    @Override
    public String toString() {
        return "ScalarRange{valid=" + valid + ", minValue=" + minValue
                + ", maxValue=" + maxValue + ", reason='" + reason + "'}";
    }
}
