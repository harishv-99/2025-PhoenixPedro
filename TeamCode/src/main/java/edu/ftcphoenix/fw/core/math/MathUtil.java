package edu.ftcphoenix.fw.core.math;

import java.util.Locale;

/**
 * Small collection of math helpers commonly used in robotics code.
 *
 * <p>This class is intentionally minimal. The goal is to centralize a few
 * patterns that appear throughout the framework (clamping, deadband, angle wrapping)
 * so that:</p>
 *
 * <ul>
 *   <li>Behavior is consistent everywhere.</li>
 *   <li>Students have a single, obvious place to look for these utilities.</li>
 *   <li>We avoid re-implementing the same logic in multiple packages.</li>
 * </ul>
 *
 * <p>As the framework evolves, new helpers should only be added here when:</p>
 * <ul>
 *   <li>They are used in multiple places, and</li>
 *   <li>Their behavior can be specified clearly and kept simple.</li>
 * </ul>
 */
public final class MathUtil {

    private MathUtil() {
        // utility holder; not instantiable
    }

    /**
     * Clamp a value to the closed interval [{@code min}, {@code max}].
     *
     * <p>If {@code min} is greater than {@code max}, the arguments are
     * automatically swapped so that the interval is well-defined.</p>
     *
     * <p>Examples:</p>
     * <pre>{@code
     * double x = MathUtil.clamp(1.5, 0.0, 1.0);   // -> 1.0
     * double y = MathUtil.clamp(-0.5, 0.0, 1.0);  // -> 0.0
     * double z = MathUtil.clamp(0.5, 1.0, 0.0);   // -> 0.5 (min/max swapped)
     * }</pre>
     *
     * @param value value to clamp
     * @param min   lower bound of the interval
     * @param max   upper bound of the interval
     * @return {@code value} clamped to [{@code min}, {@code max}]
     */
    public static double clamp(double value, double min, double max) {
        if (Double.isNaN(value)) {
            return value;
        }
        if (min > max) {
            double tmp = min;
            min = max;
            max = tmp;
        }
        if (value < min) {
            return min;
        }
        if (value > max) {
            return max;
        }
        return value;
    }

    /**
     * Clamps an integer to [min, max] (inclusive).
     *
     * <p>If min > max, this swaps them to keep the method robust.</p>
     */
    public static int clamp(int value, int min, int max) {
        if (min > max) {
            int t = min;
            min = max;
            max = t;
        }
        return Math.max(min, Math.min(max, value));
    }

    /**
     * Clamp a value to the symmetric interval [-{@code maxAbs}, +{@code maxAbs}].
     *
     * <p>This is a common pattern when limiting a normalized power or command
     * that should not exceed a given magnitude.</p>
     *
     * @param value  value to clamp
     * @param maxAbs maximum allowed absolute value (must be non-negative)
     * @return {@code value} clamped to [-{@code maxAbs}, +{@code maxAbs}]
     */
    public static double clampAbs(double value, double maxAbs) {
        if (maxAbs < 0.0) {
            maxAbs = -maxAbs;
        }
        return clamp(value, -maxAbs, +maxAbs);
    }

    /**
     * Simple linear interpolation between {@code a} and {@code b}.
     *
     * <p>No clamping is applied to {@code t}:</p>
     * <ul>
     *   <li>{@code t = 0} → returns {@code a}.</li>
     *   <li>{@code t = 1} → returns {@code b}.</li>
     *   <li>{@code t < 0} or {@code t > 1} → extrapolates linearly.</li>
     * </ul>
     *
     * <p>Callers that require strictly in-range interpolation should clamp
     * {@code t} first (for example, using {@link #clamp(double, double, double)}).</p>
     *
     * @param a first value (t = 0)
     * @param b second value (t = 1)
     * @param t blend factor (typically in [0, 1])
     * @return interpolated value {@code a + t * (b - a)}
     */
    public static double lerp(double a, double b, double t) {
        return a + t * (b - a);
    }

    /**
     * Clamp a value to the interval [0, 1].
     *
     * <p>This is a small convenience wrapper around {@link #clamp(double, double, double)}
     * for values that are conceptually fractions or normalized parameters, such as
     * trigger values, servo positions, and scale factors.</p>
     *
     * @param value value to clamp
     * @return {@code value} clamped to [0, 1]
     */
    public static double clamp01(double value) {
        return clamp(value, 0.0, 1.0);
    }

    /**
     * Apply a symmetric deadband around zero.
     *
     * <p>If {@code |value|} is less than or equal to {@code deadband}, this
     * returns 0. Otherwise it returns {@code value} unchanged.</p>
     *
     * <p>This is useful for joystick processing (ignoring tiny stick noise)
     * or for ignoring very small error values in controllers.</p>
     *
     * <p>Examples:</p>
     * <pre>{@code
     * MathUtil.deadband(0.01, 0.05);  // -> 0.0
     * MathUtil.deadband(0.10, 0.05);  // -> 0.10
     * MathUtil.deadband(-0.03, 0.05); // -> 0.0
     * }</pre>
     *
     * @param value    input value
     * @param deadband non-negative deadband radius around zero
     * @return 0 if |value| <= deadband; otherwise {@code value}
     */
    public static double deadband(double value, double deadband) {
        if (deadband < 0.0) {
            deadband = -deadband;
        }
        if (Math.abs(value) <= deadband) {
            return 0.0;
        }
        return value;
    }

    /**
     * Normalize an angle in radians into the range [-π, +π].
     *
     * <p>This is the canonical "wrap heading" helper used by controllers and pose logic.
     * A common pattern is:</p>
     *
     * <pre>{@code
     * double error = MathUtil.wrapToPi(targetHeading - currentHeading);
     * }</pre>
     *
     * <p>Implementation notes:</p>
     * <ul>
     *   <li>Equivalent angles are wrapped to a canonical representative.</li>
     *   <li>{@code +π} is allowed; {@code -π} is mapped to {@code +π} by the chosen interval rules.</li>
     *   <li>{@code NaN} propagates naturally.</li>
     * </ul>
     *
     * @param angleRad angle in radians (any real value)
     * @return equivalent angle in radians, wrapped to [-π, +π]
     */
    public static double wrapToPi(double angleRad) {
        double a = angleRad % (2.0 * Math.PI);
        if (a <= -Math.PI) {
            a += 2.0 * Math.PI;
        } else if (a > Math.PI) {
            a -= 2.0 * Math.PI;
        }
        return a;
    }

    /**
     * Format a double for debug output.
     *
     * <p>This helper is intentionally small and stable across devices by forcing
     * {@link Locale#US}. It is primarily used for {@code toString()} methods and
     * simple telemetry/debug messages where a consistent number of decimals is
     * convenient.</p>
     *
     * <p>This is <strong>not</strong> intended to be a general-purpose formatting
     * library for user-facing UI.</p>
     *
     * @param value value to format
     * @return value formatted to 3 decimal places using {@link Locale#US}
     */
    public static String fmt(double value) {
        return String.format(Locale.US, "%.3f", value);
    }
}
