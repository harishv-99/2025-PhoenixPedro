package edu.ftcsushi.fw.core.math;

import java.util.Arrays;

/** Shared finite arithmetic for the immutable calibration-table family; not a public math API. */
final class InterpolationMath {
    private InterpolationMath() { }

    /** Validates order after the owning factory has already established finite coordinates. */
    static void requireIncreasing(double[] axis, String name) {
        for (int i = 1; i < axis.length; i++) {
            if (!(axis[i] > axis[i - 1])) {
                throw new IllegalArgumentException(name + "-values must be strictly increasing: "
                        + name + " at authored index " + i + " was " + axis[i]
                        + " but must be greater than " + name + " at authored index " + (i - 1)
                        + " (" + axis[i - 1] + "); reorder or remove the conflicting " + name
                        + "-value");
            }
        }
    }

    /** Finds a finite query's lower node, clamping to an endpoint without allocating a bracket. */
    static int lowerIndex(double[] axis, double query) {
        if (query <= axis[0]) return 0;
        int last = axis.length - 1;
        if (query >= axis[last]) return last;
        int found = Arrays.binarySearch(axis, query);
        return found >= 0 ? found : -found - 2;
    }

    /** Computes a clamped finite fraction without overflowing an opposite-sign coordinate span. */
    static double fraction(double query, double low, double high) {
        if (query <= low) return 0.0;
        if (query >= high) return 1.0;
        double span = high - low;
        double result = Double.isFinite(span)
                ? (query - low) / span
                : (query * 0.5 - low * 0.5) / (high * 0.5 - low * 0.5);
        return Math.max(0.0, Math.min(1.0, result));
    }

    /** Blends finite endpoints without overflowing a valid convex result or altering exact nodes. */
    static double blend(double low, double high, double fraction) {
        if (fraction <= 0.0) return low;
        if (fraction >= 1.0) return high;
        double delta = high - low;
        if (!Double.isFinite(delta)) {
            return (1.0 - fraction) * low + fraction * high;
        }
        return fraction <= 0.5
                ? low + fraction * delta
                : high - (1.0 - fraction) * delta;
    }
}
