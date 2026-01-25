package edu.ftcphoenix.fw.input;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.ftcphoenix.fw.util.MathUtil;

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
 *
 * void loop(double dtSec) {
 *     pads.update(dtSec); // updates all buttons
 *
 *     Axis lx = pads.p1().leftX();
 *     Axis ly = pads.p1().leftY();
 *
 *     double strafe = lx.get();
 *     double forward = -ly.get(); // if you prefer up = +1 in your math
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
     * Apply a deadband to this axis, returning a new axis.
     *
     * <p>
     * Values within {@code [-deadband, +deadband]} are treated as 0.0.
     * </p>
     *
     * <p>
     * This is useful for joystick center wobble or accidental small motions.
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
            // Positive threshold: held when value >= threshold.
            raw = () -> self.get() >= threshold;
        } else {
            // Negative threshold: held when value <= threshold.
            raw = () -> self.get() <= threshold;
        }
        // Wrap in a stateful, registered Button with edge + level semantics.
        return Button.of(raw);
    }

    // ------------------------------------------------------------------------
    // Factory helpers
    // ------------------------------------------------------------------------

    /**
     * Factory for an {@link Axis} backed by a {@link DoubleSupplier}.
     *
     * <p>
     * This is the most generic way to adapt arbitrary numeric sources into
     * the {@code Axis} interface.
     * </p>
     *
     * <p>
     * The supplied value is used directly; the factory does not clamp or scale.
     * Calling code is responsible for choosing an appropriate range or applying
     * transformations (e.g., via {@link #clamped(double, double)} or
     * {@link #scaled(double)}).
     * </p>
     */
    static Axis of(final DoubleSupplier supplier) {
        if (supplier == null) {
            throw new IllegalArgumentException("supplier is required");
        }
        return supplier::getAsDouble;
    }

    /**
     * Create an axis from a constant value.
     *
     * <p>
     * This is rarely used in production code, but is handy for testing or
     * temporarily wiring "always on" behavior.
     * </p>
     */
    static Axis constant(double value) {
        return () -> value;
    }

    /**
     * Create an axis in {0.0, 1.0} from a raw {@link BooleanSupplier}.
     *
     * <p>
     * When the supplier returns {@code true}, the axis returns {@code 1.0}.
     * Otherwise it returns {@code 0.0}.
     * </p>
     */
    static Axis fromBooleanSupplier(final BooleanSupplier supplier) {
        if (supplier == null) {
            throw new IllegalArgumentException("supplier is required");
        }
        return () -> supplier.getAsBoolean() ? 1.0 : 0.0;
    }

    /**
     * Create an axis in {0.0, 1.0} from a {@link Button}.
     *
     * <p>
     * The returned axis uses {@link Button#isHeld()} (level semantics),
     * <b>not</b> {@link Button#onPress()} / {@link Button#onRelease()}.
     * </p>
     *
     * @param button source button; must not be {@code null}
     * @return axis that is 1.0 when the button is held, 0.0 otherwise
     */
    static Axis fromButton(final Button button) {
        if (button == null) {
            throw new IllegalArgumentException("Button is required");
        }
        return new Axis() {
            @Override
            public double get() {
                return button.isHeld() ? 1.0 : 0.0;
            }
        };
    }

    /**
     * Create an axis from a {@link BooleanSupplier}, returning 1.0 when the
     * supplier is {@code true}, 0.0 when it is {@code false}.
     *
     * <p>
     * This is a convenience alias for {@link #fromBooleanSupplier(BooleanSupplier)}.
     * </p>
     */
    static Axis fromBoolean(BooleanSupplier supplier) {
        return fromBooleanSupplier(supplier);
    }

    // ------------------------------------------------------------------------
    // Combinators: build signed axes from positive axes / buttons
    // ------------------------------------------------------------------------

    /**
     * Build a signed axis in [-1, +1] from two "positive" axes (typically triggers).
     *
     * <p>
     * Intended usage is to combine a forward and backward trigger:
     * </p>
     *
     * <pre>{@code
     * Axis forward = player.rightTrigger(); // [0,1]
     * Axis back    = player.leftTrigger();  // [0,1]
     *
     * Axis axial = Axis.signedFromPositivePair(forward, back);
     * }</pre>
     *
     * <p>
     * Internally, both inputs are clamped to [0, 1] via
     * {@link MathUtil#clamp01(double)} before subtraction, so accidental misuse
     * (e.g., passing in a stick axis in [-1, +1]) is somewhat contained.
     * </p>
     *
     * @param positive "forward" axis (e.g., right trigger)
     * @param negative "backward" axis (e.g., left trigger)
     * @return signed axis in [-1, +1] as (positive - negative)
     */
    static Axis signedFromPositivePair(final Axis positive, final Axis negative) {
        if (positive == null) {
            throw new IllegalArgumentException("positive axis is required");
        }
        if (negative == null) {
            throw new IllegalArgumentException("negative axis is required");
        }

        return new Axis() {
            @Override
            public double get() {
                double pos = MathUtil.clamp01(positive.get());
                double neg = MathUtil.clamp01(negative.get());
                return pos - neg; // in [-1, +1]
            }
        };
    }

    /**
     * Build a signed axis in [-1, +1] from two {@link Button}s.
     *
     * <p>
     * This is analogous to {@link #signedFromPositivePair(Axis, Axis)} but
     * uses buttons instead of continuous axes.
     * </p>
     *
     * <p>
     * When {@code positive} is held and {@code negative} is not, the axis is +1.0.<br>
     * When {@code negative} is held and {@code positive} is not, the axis is -1.0.<br>
     * When both are held, the axis is 0.0 (they cancel).<br>
     * When neither is held, the axis is 0.0.
     * </p>
     *
     * @param positive "forward" button
     * @param negative "backward" button
     * @return signed axis in [-1, +1] derived from the two buttons
     */
    static Axis signedFromButtonsPair(final Button positive, final Button negative) {
        if (negative == null) {
            throw new IllegalArgumentException("negative button is required");
        }
        if (positive == null) {
            throw new IllegalArgumentException("positive button is required");
        }

        return new Axis() {
            @Override
            public double get() {
                double pos = positive.isHeld() ? 1.0 : 0.0;
                double neg = negative.isHeld() ? 1.0 : 0.0;
                return pos - neg; // in [-1, +1]
            }
        };
    }
}
