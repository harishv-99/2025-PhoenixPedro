package edu.ftcphoenix.fw.actuation;

import edu.ftcphoenix.fw.core.source.Source;

/**
 * Legal scalar travel range for a mechanism in the same units as its {@link Plant} target.
 *
 * <p>A range may be unbounded, bounded, half-bounded, or invalid. Invalid ranges are useful before
 * homing/calibration is complete; a setpoint planner should refuse to command a mechanism when its
 * range is invalid.</p>
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
     * Returns an unbounded valid range.
     */
    public static ScalarRange unbounded() {
        return new ScalarRange(true, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, "unbounded");
    }

    /**
     * Returns a valid bounded range.
     */
    public static ScalarRange bounded(double minValue, double maxValue) {
        if (minValue > maxValue) {
            throw new IllegalArgumentException("ScalarRange minValue must be <= maxValue");
        }
        return new ScalarRange(true, minValue, maxValue, "bounded");
    }

    /**
     * Returns a valid range with only a lower bound.
     */
    public static ScalarRange minOnly(double minValue) {
        return new ScalarRange(true, minValue, Double.POSITIVE_INFINITY, "minOnly");
    }

    /**
     * Returns a valid range with only an upper bound.
     */
    public static ScalarRange maxOnly(double maxValue) {
        return new ScalarRange(true, Double.NEGATIVE_INFINITY, maxValue, "maxOnly");
    }

    /**
     * Returns an invalid range with an actionable reason, commonly used before homing.
     */
    public static ScalarRange invalid(String reason) {
        return new ScalarRange(false, Double.NaN, Double.NaN, reason != null ? reason : "invalid range");
    }

    /**
     * Returns a source that always reports an unbounded range.
     */
    public static Source<ScalarRange> unboundedSource() {
        return clock -> ScalarRange.unbounded();
    }

    /**
     * Returns true if {@code value} lies inside this valid range.
     */
    public boolean contains(double value) {
        return valid && value >= minValue && value <= maxValue;
    }

    /**
     * Clamps a value into this valid range.
     */
    public double clamp(double value) {
        if (!valid) {
            return Double.NaN;
        }
        return Math.max(minValue, Math.min(maxValue, value));
    }

    /**
     * Returns true when neither side has a finite bound.
     */
    public boolean isUnbounded() {
        return valid && !Double.isFinite(minValue) && !Double.isFinite(maxValue);
    }

    /**
     * Returns the finite center of the range, or NaN if either side is unbounded.
     */
    public double center() {
        return valid && Double.isFinite(minValue) && Double.isFinite(maxValue)
                ? 0.5 * (minValue + maxValue)
                : Double.NaN;
    }

    @Override
    public String toString() {
        return "ScalarRange{valid=" + valid + ", minValue=" + minValue
                + ", maxValue=" + maxValue + ", reason='" + reason + "'}";
    }
}
