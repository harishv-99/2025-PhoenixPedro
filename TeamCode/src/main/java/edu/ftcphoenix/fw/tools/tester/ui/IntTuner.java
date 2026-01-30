package edu.ftcphoenix.fw.tools.tester.ui;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;
import java.util.function.BooleanSupplier;

import edu.ftcphoenix.fw.core.math.MathUtil;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.input.binding.Bindings;

/**
 * Reusable "integer target" controller for tester UIs.
 *
 * <p>This is the integer sibling of {@link ScalarTuner}. It exists to avoid duplicating
 * the same bindings/telemetry for things like:
 * <ul>
 *   <li>encoder position targets (ticks)</li>
 *   <li>velocity targets (ticks/sec or RPM-scaled ints)</li>
 *   <li>index/selection values</li>
 * </ul>
 *
 * <h2>What it owns</h2>
 * <ul>
 *   <li>An integer {@code target} clamped to [{@code min}, {@code max}]</li>
 *   <li>Fine/coarse step sizing</li>
 *   <li>Optional enabled/arm toggle (if you want it)</li>
 *   <li>Optional analog axis "nudge" that changes the target continuously</li>
 * </ul>
 */
public final class IntTuner {

    private final String label;

    private int min;
    private int max;

    private int fineStep;
    private int coarseStep;

    private boolean enabledSupported = true;
    private boolean enabled = false;

    private boolean fine = true;

    private int target;
    private int disabledValue;

    // Optional axis nudge
    private ScalarSource axis = null;
    private double axisDeadband = 0.08;

    // ticks/sec at full deflection for fine vs coarse
    private double axisFineRatePerSec = 0.0;
    private double axisCoarseRatePerSec = 0.0;

    // carry for fractional tick accumulation
    private double axisCarry = 0.0;

    /**
     * Create an integer tuner.
     *
     * <p>An {@code IntTuner} is a small helper for TeleOp test screens where you want to adjust an
     * integer target (like encoder ticks) using gamepad buttons/axes.</p>
     *
     * @param label display label used in telemetry (for example {@code "TargetTicks"})
     * @param min minimum target value (inclusive)
     * @param max maximum target value (inclusive)
     * @param fineStep increment used in fine mode (absolute value; must be {@code >= 1})
     * @param coarseStep increment used in coarse mode (absolute value; must be {@code >= 1})
     * @param initialTarget initial target value (will be clamped into {@code [min, max]})
     */
    public IntTuner(String label,
                    int min,
                    int max,
                    int fineStep,
                    int coarseStep,
                    int initialTarget) {

        this.label = (label == null) ? "Value" : label;

        // Robust if min/max are swapped.
        if (min > max) {
            int t = min;
            min = max;
            max = t;
        }

        this.min = min;
        this.max = max;

        this.fineStep = Math.max(1, Math.abs(fineStep));
        this.coarseStep = Math.max(1, Math.abs(coarseStep));

        this.target = MathUtil.clamp(initialTarget, min, max);

        // Default disabled value is 0 clamped into range.
        this.disabledValue = MathUtil.clamp(0, min, max);
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
    public IntTuner setEnableSupported(boolean supported) {
        this.enabledSupported = supported;
        if (!supported) {
            this.enabled = true;
        }
        return this;
    }

    /**
     * Set the value returned by {@link #applied()} when the tuner is disabled.
     *
     * @param value disabled output value (will be clamped into {@code [min, max]})
     * @return this tuner for chaining
     */
    public IntTuner setDisabledValue(int value) {
        this.disabledValue = MathUtil.clamp(value, min, max);
        return this;
    }

    /**
     * Attach an analog axis that continuously adjusts the target.
     *
     * <p>Example: for encoder targets, you might want:
     * <ul>
     *   <li>fineRate = 250 ticks/sec</li>
     *   <li>coarseRate = 1500 ticks/sec</li>
     * </ul>
     * so holding the stick moves the target smoothly.</p>
     *
     * @param axis              axis to read (e.g., gamepads.p1().leftY())
     * @param deadband          raw axis deadband
     * @param fineRatePerSec    ticks/sec at full deflection in fine mode
     * @param coarseRatePerSec  ticks/sec at full deflection in coarse mode
     */
    public IntTuner attachAxisNudge(ScalarSource axis,
                                    double deadband,
                                    double fineRatePerSec,
                                    double coarseRatePerSec) {
        this.axis = axis;
        this.axisDeadband = Math.abs(deadband);
        this.axisFineRatePerSec = Math.max(0.0, fineRatePerSec);
        this.axisCoarseRatePerSec = Math.max(0.0, coarseRatePerSec);
        return this;
    }

    // ---------------------------------------------------------------------------------------------
    // State access
    // ---------------------------------------------------------------------------------------------

    /**
     * Return whether this tuner is currently enabled.
     *
     * <p>If enable support is disabled via {@link #setEnableSupported(boolean)}, this always returns {@code true}.</p>
     *
     * @return {@code true} if enabled
     */
    public boolean isEnabled() {
        return enabledSupported ? enabled : true;
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
    public int target() {
        return target;
    }

    /**
     * Return the current step size (fine or coarse).
     *
     * @return current step increment (always {@code >= 1})
     */
    public int step() {
        return fine ? fineStep : coarseStep;
    }

    /**
     * Set the raw target value.
     *
     * @param target new target value (will be clamped into {@code [min, max]})
     */
    public void setTarget(int target) {
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
     * Set the target to the configured disabled value (convenient "zero"/reset button).
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
     * Apply the enabled gate to the target (if you use enabled semantics).
     * If you don't want enabled semantics, call {@link #setEnableSupported(boolean)} with false.
     */
    /**
     * Return the value you should apply to the system under test.
     *
     * <p>If enabled support is active and the tuner is disabled, this returns the configured
     * disabled value. Otherwise it returns {@link #target()}.</p>
     *
     * @return applied output value
     */
    public int applied() {
        return isEnabled() ? target : disabledValue;
    }

    /**
     * Update the target from the attached axis nudge, if configured.
     *
     * @param dtSec time since last loop (seconds)
     * @param active optional gate; if provided and it returns {@code false}, axis nudge is ignored
     */
    public void updateFromAxis(LoopClock clock, BooleanSupplier active) {
        if (axis == null) return;
        if (active != null && !active.getAsBoolean()) return;
        double dtSec = (clock != null) ? clock.dtSec() : 0.0;


        double raw = axis.getAsDouble(clock);
        if (Math.abs(raw) <= axisDeadband) {
            axisCarry = 0.0;
            return;
        }

        double rate = fine ? axisFineRatePerSec : axisCoarseRatePerSec;
        if (rate <= 0.0) return;

        axisCarry += raw * rate * Math.max(0.0, dtSec);

        int delta = 0;
        if (axisCarry >= 1.0) {
            delta = (int) Math.floor(axisCarry);
        } else if (axisCarry <= -1.0) {
            delta = (int) Math.ceil(axisCarry); // negative
        }

        if (delta != 0) {
            axisCarry -= delta;
            setTarget(target + delta);
        }
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
     * @param bindings bindings registry to attach to
     * @param enableToggle button that toggles enabled/disabled (nullable)
     * @param fineToggle button that toggles fine/coarse (nullable)
     * @param incButton button that increments the target (nullable)
     * @param decButton button that decrements the target (nullable)
     * @param zeroButton button that resets to the disabled value (nullable)
     * @param active optional gating predicate; if null, actions are always allowed
     */
    public void bind(Bindings bindings,
                     BooleanSource enableToggle,
                     BooleanSource fineToggle,
                     BooleanSource incButton,
                     BooleanSource decButton,
                     BooleanSource zeroButton,
                     BooleanSupplier active) {

        BooleanSupplier ok = (active == null) ? () -> true : active;

        if (enableToggle != null) {
            bindings.onRise(enableToggle, () -> {
                if (!ok.getAsBoolean()) return;
                toggleEnabled();
            });
        }

        if (fineToggle != null) {
            bindings.onRise(fineToggle, () -> {
                if (!ok.getAsBoolean()) return;
                toggleFine();
            });
        }

        if (incButton != null) {
            bindings.onRise(incButton, () -> {
                if (!ok.getAsBoolean()) return;
                inc();
            });
        }

        if (decButton != null) {
            bindings.onRise(decButton, () -> {
                if (!ok.getAsBoolean()) return;
                dec();
            });
        }

        if (zeroButton != null) {
            bindings.onRise(zeroButton, () -> {
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
                "%s: target=%d applied=%d range=[%d, %d]",
                label, target(), applied(), min, max));

        t.addLine(String.format(Locale.US,
                "Enabled=%s Step=%s (%d)",
                isEnabled() ? "ON" : "OFF",
                fine ? "FINE" : "COARSE",
                step()));
    }
}
