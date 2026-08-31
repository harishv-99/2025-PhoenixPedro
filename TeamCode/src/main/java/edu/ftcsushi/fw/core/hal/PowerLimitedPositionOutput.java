package edu.ftcsushi.fw.core.hal;

/**
 * Position-command channel that submits each target and normalized maximum-output-power magnitude
 * together as one paired logical command.
 *
 * <p>This is a narrower, evidence-bearing extension of {@link PositionOutput}. It is appropriate
 * only when the concrete actuator accepts both a native position target and a normalized
 * power-like effort limit for that command. Standard servos and arbitrary position sinks should
 * remain plain {@code PositionOutput}s rather than inventing this capability.</p>
 *
 * <p>The magnitude is command policy, not feedback: it does not report measured power, voltage,
 * current, torque, motion, or target arrival. A normally returning command establishes only the
 * position and magnitude submitted through this configured output seam.</p>
 */
public interface PowerLimitedPositionOutput extends PositionOutput {

    /**
     * Command a position with the full normalized output-power magnitude {@code 1.0}.
     *
     * <p>This preserves the ordinary {@link PositionOutput} command while giving it one precise
     * meaning on the richer capability. Use {@link #setPosition(double, double)} when a smaller
     * per-command magnitude is required.</p>
     */
    @Override
    default void setPosition(double position) {
        setPosition(position, 1.0);
    }

    /**
     * Submit one native position command and its normalized maximum-output-power magnitude together
     * through one call.
     *
     * <p>Implementations must validate the complete pair before changing their command cache or
     * invoking the underlying output. The magnitude domain is the inclusive {@code [0.0, 1.0]}
     * range. Both arguments must be finite. A normal return means the complete pair was submitted
     * through this seam; it does not prove physical response.</p>
     *
     * <p>This is not a transactional hardware guarantee. An adapter may perform sequential effects
     * after prevalidation; if a later effect throws, earlier device state may already have changed.
     * In that case its cached submitted pair must become unavailable rather than claim a complete
     * successful command.</p>
     *
     * @param position native position command
     * @param maximumOutputPowerMagnitude finite normalized magnitude in {@code [0.0, 1.0]}
     * @throws IllegalArgumentException if either argument is non-finite or the magnitude lies
     *                                  outside {@code [0.0, 1.0]}
     */
    void setPosition(double position, double maximumOutputPowerMagnitude);

    /**
     * Return the maximum-output power magnitude paired with the cached position command.
     *
     * <p>This is the submitted command limit, not applied-power or physical feedback. Before a
     * successful command, or when an implementation can no longer report a truthful complete
     * pair, it should return {@link Double#NaN}.</p>
     *
     * @return cached normalized maximum-output power magnitude, or {@code NaN} when unavailable
     */
    double getCommandedMaximumOutputPowerMagnitude();

    /**
     * Perform this power-limited position output's explicit natural stop.
     *
     * <p>Unlike a general set-and-hold {@link PositionOutput}, a power-limited device must decide
     * how its controller and effort are made safe. Implementations therefore cannot inherit the
     * default position-hold behavior.</p>
     */
    @Override
    void stop();
}
