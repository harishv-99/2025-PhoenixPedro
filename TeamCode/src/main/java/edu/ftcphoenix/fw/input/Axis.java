package edu.ftcphoenix.fw.input;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.ftcphoenix.fw.core.math.MathUtil;

/**
 * Continuous input in [-1..+1] or [0..1].
 *
 * <h2>What an Axis represents</h2>
 * <p>
 * {@code Axis} is a small abstraction for "something that returns a double each loop":
 * a stick, a trigger, a synthesized value from multiple buttons, etc.
 * </p>
 *
 * <p>
 * It is intentionally tiny:
 * </p>
 *
 * <ul>
 *   <li>{@link #get()} – sample the current value.</li>
 *   <li>Default helpers for deadband, scaling, clamping, etc.</li>
 *   <li>Static helpers to construct axes from raw suppliers and buttons.</li>
 * </ul>
 *
 * <h2>Typical usage</h2>
 *
 * <pre>{@code
 * Gamepads pads = Gamepads.create(gamepad1, gamepad2);
 * LoopClock clock = new LoopClock();
 *
 * void loop(double runtimeSec) {
 *     clock.update(runtimeSec);
 *     pads.update(clock); // updates registered button edges (idempotent by clock.cycle())
 *
 *     Axis lx = pads.p1().leftX();
 *     Axis ly = pads.p1().leftY();
 *
 *     double strafe = lx.get();
 *     double forward = ly.get(); // Phoenix gamepad convention: pushing up is positive
 * }
 * }</pre>
 *
 * <h2>Why not just use doubles?</h2>
 *
 * <p>
 * The main benefits of {@code Axis} are:
 * </p>
 *
 * <ul>
 *   <li>It can be composed (deadband, scaling, signed-from-triggers).</li>
 *   <li>You can pass it around as "an input source" instead of wiring raw gamepad fields everywhere.</li>
 *   <li>You can build higher-level helpers on top (e.g. drive shaping) without caring where the data comes from.</li>
 * </ul>
 */
public interface Axis {

    /**
     * Sample the current value of this axis.
     *
     * <p>
     * The returned value is typically in [-1, +1] or [0, 1] depending on what
     * the axis represents (stick vs trigger), but this is not enforced; it is
     * acceptable for synthesized axes to use other ranges as long as callers
     * know what to expect.
     * </p>
     */
    double get();

    // ------------------------------------------------------------------------
    // Common per-axis transforms
    // ------------------------------------------------------------------------

    /**
     * Return a new {@link Axis} that is this axis clamped to a given range.
     *
     * <p>
     * This is a simple pass-through wrapper; it does not store any state.
     * </p>
     */
    default Axis clamped(double min, double max) {
        final Axis self = this;
        return () -> MathUtil.clamp(self.get(), min, max);
    }

    /**
     * Return a new {@link Axis} scaled by a constant factor.
     *
     * <p>
     * For example, to reduce a stick's sensitivity to 50%:
     * </p>
     *
     * <pre>{@code
     * Axis slow = pads.p1().leftY().scaled(0.5);
     * }</pre>
     */
    default Axis scaled(double factor) {
        final Axis self = this;
        return () -> self.get() * factor;
    }

    /**
     * Return a new {@link Axis} with its sign inverted.
     *
     * <p>
     * Equivalent to {@code scaled(-1.0)}.
     * </p>
     */
    default Axis inverted() {
        return scaled(-1.0);
    }

    /**
     * Apply a deadband to this axis, returning a new axis.
     *
     * <p>
     * Values within {@code [-deadband, +deadband]} are treated as 0.0.
     * </p>
     *
     * <p>
     * This is useful for joystick center wobble or accidental small motions.
     * </p>
     *
     * <p>
     * Note: This method treats the deadband as centered around 0.
     * </p>
     */
    default Axis deadband(double deadband) {
        final Axis self = this;
        final double db = Math.abs(deadband);

        return () -> {
            double v = self.get();
            return (Math.abs(v) <= db) ? 0.0 : v;
        };
    }

    /**
     * Apply a deadband centered at 0 and renormalize the remaining range back to the
     * provided {@code [min, max]}.
     *
     * <p>
     * This is useful when you want to remove small inputs (e.g., stick drift) but still
     * reach full scale at the extremes.
     * </p>
     *
     * <p>
     * The caller provides {@code min} and {@code max} explicitly so {@code Axis} does not
     * need to assume any particular range.
     * </p>
     *
     * <p>
     * Behavior (deadband centered at 0):
     * </p>
     * <ul>
     *   <li>If {@code |v| <= deadband} → 0</li>
     *   <li>If {@code v > deadband} → linearly map {@code [deadband..max]} to {@code [0..max]}</li>
     *   <li>If {@code v < -deadband} → linearly map {@code [min..-deadband]} to {@code [min..0]}</li>
     * </ul>
     */
    default Axis deadbandNormalized(double deadband, double min, double max) {
        final Axis self = this;
        final double db = Math.abs(deadband);

        return () -> {
            double v = MathUtil.clamp(self.get(), min, max);

            if (Math.abs(v) <= db) {
                return 0.0;
            }

            // Positive side: [db..max] -> [0..max]
            if (v > 0.0) {
                if (max <= db) {
                    return MathUtil.clamp(v, min, max);
                }
                double t = (v - db) / (max - db);   // db -> 0, max -> 1
                double out = t * max;               // 0 -> 0, 1 -> max
                return MathUtil.clamp(out, min, max);
            }

            // Negative side: [min..-db] -> [min..0]
            if (min >= -db) {
                return MathUtil.clamp(v, min, max);
            }
            double t = (v + db) / (min + db);       // -db -> 0, min -> 1 (denominator is negative)
            double out = t * min;                   // 0 -> 0, 1 -> min
            return MathUtil.clamp(out, min, max);
        };
    }

    /**
     * Shape this axis with the same "deadband → normalize → pow(expo)" behavior used by
     * drive-stick shaping.
     *
     * <p>
     * This method:
     * </p>
     * <ol>
     *   <li>Clamps the raw value to {@code [min, max]}.</li>
     *   <li>Applies a deadband centered at 0.</li>
     *   <li>Normalizes the remaining magnitude to {@code [0, 1]} based on the corresponding side's
     *       full-scale magnitude (positive side uses {@code max}, negative uses {@code -min}).</li>
     *   <li>Applies {@code shaped = pow(norm, expo)} (with {@code expo < 1} treated as {@code 1}).</li>
     *   <li>Scales back to the original side's full-scale range.</li>
     * </ol>
     *
     * <p>
     * The caller provides {@code min} and {@code max} explicitly so {@code Axis} does not assume
     * a specific range like [-1, +1] or [0, 1].
     * </p>
     */
    default Axis shaped(double deadband, double expo, double min, double max) {
        final Axis self = this;
        final double db = Math.abs(deadband);
        final double e = Math.max(1.0, expo);

        return () -> {
            double v = MathUtil.clamp(self.get(), min, max);

            if (Math.abs(v) <= db) {
                return 0.0;
            }

            // Positive side shaping.
            if (v > 0.0) {
                double sideMax = max;
                if (sideMax <= db) {
                    return MathUtil.clamp(v, min, max);
                }
                double norm = (v - db) / (sideMax - db);         // db -> 0, max -> 1
                norm = MathUtil.clamp(norm, 0.0, 1.0);
                double shaped = Math.pow(norm, e) * sideMax;
                return MathUtil.clamp(shaped, min, max);
            }

            // Negative side shaping.
            double sideMag = -min; // magnitude of negative full-scale
            if (sideMag <= db) {
                return MathUtil.clamp(v, min, max);
            }
            double norm = ((-v) - db) / (sideMag - db);          // |v| in (db..sideMag] -> (0..1]
            norm = MathUtil.clamp(norm, 0.0, 1.0);
            double shapedMag = Math.pow(norm, e) * sideMag;
            double shaped = -shapedMag;
            return MathUtil.clamp(shaped, min, max);
        };
    }

    /**
     * Convert this axis into a {@link Button} by thresholding.
     *
     * <p>
     * This lets you use the <b>same method</b> for both "positive direction"
     * and "negative direction" checks on signed axes:
     * </p>
     *
     * <pre>{@code
     * Axis axial = player.leftY(); // [-1, +1]
     *
     * Button forward = axial.asButton(+0.5); // pressed when value >= +0.5
     * Button back    = axial.asButton(-0.5); // pressed when value <= -0.5
     * }</pre>
     *
     * <p>
     * For positive-only axes (e.g., triggers in [0, 1]), you would typically
     * use a positive threshold (e.g., {@code 0.5}). Passing a negative threshold
     * for such axes is allowed but not meaningful; it would almost always result
     * in the button being "not pressed".
     * </p>
     *
     * @param threshold cutoff value at or beyond which the button is "held";
     *                  positive thresholds use {@code >=}, negative thresholds
     *                  use {@code <=}
     * @return a stateful, registered {@link Button} view of this axis
     */
    default Button asButton(final double threshold) {
        final Axis self = this;
        final BooleanSupplier raw;
        if (threshold >= 0.0) {
            raw = () -> self.get() >= threshold;
        } else {
            raw = () -> self.get() <= threshold;
        }
        return Button.of(raw);
    }

    // ------------------------------------------------------------------------
    // Factories
    // ------------------------------------------------------------------------

    /**
     * Create an axis from a raw supplier.
     */
    static Axis of(DoubleSupplier raw) {
        return raw::getAsDouble;
    }

    /**
     * Create a signed axis from two buttons (negative and positive).
     *
     * <p>Returns:</p>
     * <ul>
     *   <li>-1 when {@code negative} is held and {@code positive} is not</li>
     *   <li>+1 when {@code positive} is held and {@code negative} is not</li>
     *   <li>0 when neither or both are held</li>
     * </ul>
     */
    static Axis signedFromButtons(Button negative, Button positive) {
        return () -> {
            boolean neg = negative != null && negative.isHeld();
            boolean pos = positive != null && positive.isHeld();
            if (neg == pos) return 0.0;
            return pos ? 1.0 : -1.0;
        };
    }

    /**
     * Create a [0..1] axis from a single button.
     *
     * <p>Returns 1 when held, otherwise 0.</p>
     */
    static Axis fromButton(Button button) {
        return () -> (button != null && button.isHeld()) ? 1.0 : 0.0;
    }

    /**
     * Generic difference combinator: {@code positive - negative}.
     *
     * <p>
     * This method makes no assumptions about ranges and does not clamp.
     * </p>
     */
    static Axis difference(Axis negative, Axis positive) {
        return () -> {
            double neg = (negative != null) ? negative.get() : 0.0;
            double pos = (positive != null) ? positive.get() : 0.0;
            return pos - neg;
        };
    }

    /**
     * Create a normalized signed axis from two axes by specifying each axis's expected range.
     *
     * <p>
     * Each input is normalized into {@code [0, 1]} using its provided {@code [min, max]} range,
     * and the output is {@code posNorm - negNorm}, which is naturally in {@code [-1, +1]}.
     * </p>
     *
     * <p>
     * This keeps range assumptions explicit while still producing a standard signed control axis.
     * </p>
     */
    static Axis signedNormalizedFromAxes(
            Axis negativeAxis, double negativeMin, double negativeMax,
            Axis positiveAxis, double positiveMin, double positiveMax
    ) {
        return () -> {
            double negRaw = (negativeAxis != null) ? negativeAxis.get() : 0.0;
            double posRaw = (positiveAxis != null) ? positiveAxis.get() : 0.0;

            double negDen = (negativeMax - negativeMin);
            double posDen = (positiveMax - positiveMin);

            double negNorm = (negDen == 0.0) ? 0.0 : (negRaw - negativeMin) / negDen;
            double posNorm = (posDen == 0.0) ? 0.0 : (posRaw - positiveMin) / posDen;

            negNorm = MathUtil.clamp(negNorm, 0.0, 1.0);
            posNorm = MathUtil.clamp(posNorm, 0.0, 1.0);

            return MathUtil.clamp(posNorm - negNorm, -1.0, 1.0);
        };
    }

    /**
     * Convenience helper for the common "two triggers → signed axis" use-case.
     *
     * <p>
     * Assumes each trigger axis is approximately in {@code [0, 1]} and returns
     * {@code positiveTrigger - negativeTrigger} (in {@code [-1, +1]}).
     * </p>
     */
    static Axis signedFromTriggers(Axis negativeTrigger, Axis positiveTrigger) {
        return signedNormalizedFromAxes(
                negativeTrigger, 0.0, 1.0,
                positiveTrigger, 0.0, 1.0
        );
    }

    /**
     * Create an axis that reports the 2D magnitude of ({@code x}, {@code y}): {@code sqrt(x^2 + y^2)}.
     *
     * <p>This is commonly used for "stick moved" / "stick idle" checks, radial deadbands,
     * and any UI behavior that depends on how far a 2D control has moved.</p>
     *
     * <p>Null axes are treated as 0.</p>
     */
    static Axis magnitude(Axis x, Axis y) {
        return () -> {
            double xv = (x != null) ? x.get() : 0.0;
            double yv = (y != null) ? y.get() : 0.0;
            return Math.hypot(xv, yv);
        };
    }

    /**
     * Create an axis that reports the squared 2D magnitude of ({@code x}, {@code y}): {@code x^2 + y^2}.
     *
     * <p>Use this if you want to avoid a {@code sqrt} and you're comparing to a squared threshold.</p>
     *
     * <p>Null axes are treated as 0.</p>
     */
    static Axis magnitudeSquared(Axis x, Axis y) {
        return () -> {
            double xv = (x != null) ? x.get() : 0.0;
            double yv = (y != null) ? y.get() : 0.0;
            return xv * xv + yv * yv;
        };
    }
}