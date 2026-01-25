package edu.ftcphoenix.fw.tools.tester.ui;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleUnaryOperator;

import edu.ftcphoenix.fw.core.math.MathUtil;
import edu.ftcphoenix.fw.input.Axis;
import edu.ftcphoenix.fw.input.Button;
import edu.ftcphoenix.fw.input.binding.Bindings;

/**
 * Reusable "scalar target" controller for tester UIs.
 *
 * <p>Designed to avoid duplicating the same input bindings across many testers:
 * power testers (DcMotor / CRServo), position testers (Servo), and other target-based testers.</p>
 *
 * <h2>What it owns</h2>
 * <ul>
 *   <li>A target value within a range [{@code min}, {@code max}]</li>
 *   <li>Fine/coarse step sizing</li>
 *   <li>Optional enable/arm toggle</li>
 *   <li>Optional invert toggle</li>
 *   <li>Optional analog axis override</li>
 *   <li>Optional disabled behavior (apply a disabled value vs hold last applied)</li>
 * </ul>
 *
 * <h2>Invert behavior</h2>
 * Default invert is "reflect across the midpoint" of the range:
 * <pre>
 * inverted(v) = (min + max) - v
 * </pre>
 * which yields:
 * <ul>
 *   <li>[-1,+1] → -v (power sign flip)</li>
 *   <li>[0,1]   → 1-v (servo position flip)</li>
 * </ul>
 */
public final class ScalarTuner {

    /**
     * How {@link #applied()} behaves when disabled.
     */
    public enum DisabledBehavior {
        /**
         * When disabled, {@link #applied()} returns {@link #disabledValue} (after invert, if enabled).
         * This is ideal for power outputs (disabled => 0).
         */
        APPLY_DISABLED_VALUE,

        /**
         * When disabled, {@link #applied()} returns the last value previously returned while enabled.
         * This is ideal for servos where "disabled" should mean "don't move it anymore".
         *
         * <p>Note: while disabled, invert toggles do not affect the held output.</p>
         */
        HOLD_LAST_APPLIED
    }

    private final String label;

    private double min;
    private double max;

    private double fineStep;
    private double coarseStep;

    private boolean enabledSupported = true;
    private boolean invertSupported = true;

    private boolean enabled = false;
    private boolean inverted = false;
    private boolean fine = true;

    private double target;

    private double disabledValue = 0.0;
    private DisabledBehavior disabledBehavior = DisabledBehavior.APPLY_DISABLED_VALUE;

    // Track last output value actually "applied" so HOLD_LAST_APPLIED can work.
    private double lastApplied;

    // Optional axis override (e.g., stick or trigger)
    private Axis axis = null;
    private double axisDeadband = 0.08;
    private DoubleUnaryOperator axisMap = v -> v; // raw axis -> target range

    // Inversion mapping (defaults to midpoint reflection)
    private DoubleUnaryOperator invertFn = v -> (min + max) - v;

    /**
     * Create a scalar tuner.
     *
     * <p>A {@code ScalarTuner} is a small helper for TeleOp test screens where you want to adjust a
     * continuous target (like motor power or servo position) using gamepad buttons and/or an axis.</p>
     *
     * @param label         display label used in telemetry (for example {@code "Power"} or {@code "Position"})
     * @param min           minimum target value (inclusive)
     * @param max           maximum target value (inclusive)
     * @param fineStep      increment used in fine mode (absolute value)
     * @param coarseStep    increment used in coarse mode (absolute value)
     * @param initialTarget initial target value (will be clamped into {@code [min, max]})
     */
    public ScalarTuner(String label,
                       double min,
                       double max,
                       double fineStep,
                       double coarseStep,
                       double initialTarget) {
        this.label = (label == null) ? "Value" : label;
        this.min = min;
        this.max = max;
        this.fineStep = Math.abs(fineStep);
        this.coarseStep = Math.abs(coarseStep);

        this.target = MathUtil.clamp(initialTarget, min, max);

        // Default disabled value is 0, clamped.
        this.disabledValue = MathUtil.clamp(0.0, min, max);

        // Default invert function depends on min/max (midpoint reflection).
        this.invertFn = v -> (this.min + this.max) - v;

        // Start with a safe "last applied" value.
        this.lastApplied = this.disabledValue;
    }

    // ---------------------------------------------------------------------------------------------
    // Configuration
    // ---------------------------------------------------------------------------------------------

    /**
     * Enable/disable support for the "enabled" toggle.
     *
     * <p>If enable is not supported, the tuner is always considered enabled.</p>
     *
     * @param supported whether this tuner should expose an enable/disable toggle
     * @return this tuner for chaining
     */
    public ScalarTuner setEnableSupported(boolean supported) {
        this.enabledSupported = supported;
        if (!supported) {
            this.enabled = true;
        }
        return this;
    }

    /**
     * Enable/disable support for the "invert" toggle.
     *
     * <p>Invert is useful for quickly flipping power sign or reflecting a servo range without
     * changing the rest of your test code.</p>
     *
     * @param supported whether this tuner should expose an invert toggle
     * @return this tuner for chaining
     */
    public ScalarTuner setInvertSupported(boolean supported) {
        this.invertSupported = supported;
        if (!supported) {
            this.inverted = false;
        }
        return this;
    }

    /**
     * Set the value used when disabled (for {@link DisabledBehavior#APPLY_DISABLED_VALUE}).
     *
     * @param value disabled output value (will be clamped into {@code [min, max]})
     * @return this tuner for chaining
     */
    public ScalarTuner setDisabledValue(double value) {
        this.disabledValue = MathUtil.clamp(value, min, max);
        // Keep lastApplied safe if we haven't applied anything yet.
        if (disabledBehavior == DisabledBehavior.APPLY_DISABLED_VALUE && !isEnabled()) {
            this.lastApplied = this.disabledValue;
        }
        return this;
    }

    /**
     * Set how {@link #applied()} behaves when disabled.
     *
     * <p>Defaults to {@link DisabledBehavior#APPLY_DISABLED_VALUE} for safety.</p>
     *
     * @param behavior disabled behavior to use (nullable; ignored if null)
     * @return this tuner for chaining
     */
    public ScalarTuner setDisabledBehavior(DisabledBehavior behavior) {
        if (behavior != null) {
            this.disabledBehavior = behavior;
            if (behavior == DisabledBehavior.APPLY_DISABLED_VALUE && !isEnabled()) {
                this.lastApplied = this.disabledValue;
            }
        }
        return this;
    }

    /**
     * Override the invert mapping if you need something special.
     *
     * <p>By default, invert reflects across the midpoint of the range.</p>
     *
     * @param invertFn function that maps an applied value to its inverted form (nullable; ignored if null)
     * @return this tuner for chaining
     */
    public ScalarTuner setInvertFn(DoubleUnaryOperator invertFn) {
        if (invertFn != null) {
            this.invertFn = invertFn;
        }
        return this;
    }

    /**
     * Attach an analog axis override.
     *
     * <p>If the axis magnitude exceeds {@code deadband}, the axis value (after mapping)
     * becomes the new target.</p>
     *
     * @param axis     axis to read (for example {@code pads.p1().leftY()})
     * @param deadband deadband applied to the raw axis
     * @param axisMap  maps raw axis value to the tuner target domain (nullable; identity if null)
     * @return this tuner for chaining
     */
    public ScalarTuner attachAxis(Axis axis, double deadband, DoubleUnaryOperator axisMap) {
        this.axis = axis;
        this.axisDeadband = Math.abs(deadband);
        this.axisMap = (axisMap == null) ? (v -> v) : axisMap;
        return this;
    }

    // ---------------------------------------------------------------------------------------------
    // State access
    // ---------------------------------------------------------------------------------------------

    /**
     * Return whether the tuner is currently enabled.
     *
     * @return {@code true} if enabled
     */
    public boolean isEnabled() {
        return enabledSupported ? enabled : true;
    }

    /**
     * Return whether the tuner is currently inverted.
     *
     * <p>If invert support is disabled via {@link #setInvertSupported(boolean)}, this will return {@code false}.</p>
     *
     * @return {@code true} if inverted
     */
    public boolean isInverted() {
        return invertSupported && inverted;
    }

    /**
     * Return whether the tuner is currently in fine step mode.
     *
     * @return {@code true} if in fine mode, {@code false} if in coarse mode
     */
    public boolean isFine() {
        return fine;
    }

    /**
     * Return the current raw target value.
     *
     * @return current target (clamped into {@code [min, max]})
     */
    public double target() {
        return target;
    }

    /**
     * Return the current step size (fine or coarse).
     *
     * @return current step size
     */
    public double step() {
        return fine ? fineStep : coarseStep;
    }

    /**
     * The last value returned by {@link #applied()} while enabled (or disabledValue before first apply).
     */
    public double lastApplied() {
        return lastApplied;
    }

    /**
     * Set the raw target value.
     *
     * @param target new target value (will be clamped into {@code [min, max]})
     */
    public void setTarget(double target) {
        this.target = MathUtil.clamp(target, min, max);
    }

    /**
     * Increase the target by the current step size.
     */
    public void inc() {
        setTarget(target + step());
    }

    /**
     * Decrease the target by the current step size.
     */
    public void dec() {
        setTarget(target - step());
    }

    /**
     * Set the target to the configured disabled value.
     */
    public void zero() {
        setTarget(disabledValue);
    }

    /**
     * Toggle between fine and coarse step sizes.
     */
    public void toggleFine() {
        fine = !fine;
    }

    /**
     * Toggle the enabled state (only if enable support is enabled).
     */
    public void toggleEnabled() {
        if (!enabledSupported) return;
        enabled = !enabled;
    }

    /**
     * Toggle the inverted state (only if invert support is enabled).
     */
    public void toggleInvert() {
        if (!invertSupported) return;
        inverted = !inverted;
    }

    /**
     * Update the target from the attached axis override (if configured).
     *
     * <p>Call once per loop after inputs are updated.</p>
     *
     * @param active optional gate; if provided and it returns {@code false}, the axis override is ignored
     */
    public void updateFromAxis(BooleanSupplier active) {
        if (axis == null) return;
        if (active != null && !active.getAsBoolean()) return;

        double raw = axis.get();
        if (Math.abs(raw) <= axisDeadband) return;

        double mapped = axisMap.applyAsDouble(raw);
        setTarget(mapped);
    }

    /**
     * Return the value you should apply to a device right now.
     *
     * <p>Includes enable/disable + invert behavior.</p>
     *
     * @return applied output value (clamped into {@code [min, max]})
     */
    public double applied() {
        if (!isEnabled() && disabledBehavior == DisabledBehavior.HOLD_LAST_APPLIED) {
            // Hold means "do not change output" — ignore invert toggles while disabled.
            return lastApplied;
        }

        double v = isEnabled() ? target : disabledValue;

        if (isInverted()) {
            v = invertFn.applyAsDouble(v);
        }

        v = MathUtil.clamp(v, min, max);

        // Update lastApplied any time we are returning an "active" output (including disabled APPLY_DISABLED_VALUE).
        lastApplied = v;
        return v;
    }

    // ---------------------------------------------------------------------------------------------
    // Bindings
    // ---------------------------------------------------------------------------------------------

    /**
     * Bind a standard set of controls to manipulate this tuner.
     *
     * <p>All actions are gated by {@code active}. Pass something like {@code () -> ready}
     * so menu inputs don't conflict with your test controls.</p>
     *
     * @param bindings     bindings registry to attach to
     * @param enableToggle button that toggles enabled/disabled (nullable)
     * @param invertToggle button that toggles invert (nullable)
     * @param fineToggle   button that toggles fine/coarse (nullable)
     * @param incButton    button that increments the target (nullable)
     * @param decButton    button that decrements the target (nullable)
     * @param zeroButton   button that resets to the disabled value (nullable)
     * @param active       optional gating predicate; if null, actions are always allowed
     */
    public void bind(Bindings bindings,
                     Button enableToggle,
                     Button invertToggle,
                     Button fineToggle,
                     Button incButton,
                     Button decButton,
                     Button zeroButton,
                     BooleanSupplier active) {

        BooleanSupplier ok = (active == null) ? () -> true : active;

        if (enableToggle != null) {
            bindings.onPress(enableToggle, () -> {
                if (!ok.getAsBoolean()) return;
                toggleEnabled();
            });
        }

        if (invertToggle != null) {
            bindings.onPress(invertToggle, () -> {
                if (!ok.getAsBoolean()) return;
                toggleInvert();
            });
        }

        if (fineToggle != null) {
            bindings.onPress(fineToggle, () -> {
                if (!ok.getAsBoolean()) return;
                toggleFine();
            });
        }

        if (incButton != null) {
            bindings.onPress(incButton, () -> {
                if (!ok.getAsBoolean()) return;
                inc();
            });
        }

        if (decButton != null) {
            bindings.onPress(decButton, () -> {
                if (!ok.getAsBoolean()) return;
                dec();
            });
        }

        if (zeroButton != null) {
            bindings.onPress(zeroButton, () -> {
                if (!ok.getAsBoolean()) return;
                zero();
            });
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Telemetry
    // ---------------------------------------------------------------------------------------------

    /**
     * Render this tuner's current state into FTC {@link Telemetry}.
     *
     * @param t telemetry sink
     */
    public void render(Telemetry t) {
        t.addLine(String.format(Locale.US,
                "%s: target=%.3f applied=%.3f last=%.3f range=[%.3f, %.3f]",
                label, target(), applied(), lastApplied(), min, max));

        t.addLine(String.format(Locale.US,
                "Enabled=%s Invert=%s Step=%s (%.3f) DisabledBehavior=%s",
                isEnabled() ? "ON" : "OFF",
                isInverted() ? "ON" : "OFF",
                fine ? "FINE" : "COARSE",
                step(),
                disabledBehavior.name()));
    }
}
