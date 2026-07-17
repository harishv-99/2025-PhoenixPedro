package edu.ftcphoenix.fw.core.hal;

/**
 * Generic command-only power channel for an actuator.
 *
 * <p>{@code PowerOutput} is the lowest-level scalar command seam used by Phoenix plants when the
 * actuation lever is a normalized power-like signal. Typical examples include FTC DC motors driven
 * with {@code setPower(-1..+1)} and continuous-rotation servos treated as motors. Direct and
 * framework-regulated Phoenix Plants enforce the finite normalized range before calling this
 * boundary. Raw callers remain responsible for honoring the command domain; any adapter saturation
 * is defense in depth rather than command policy.</p>
 *
 * <p>This device-neutral interface does not choose or own an FTC motor run mode. Concrete FTC
 * adapters may own SDK state required by their command semantics. In particular,
 * {@code FtcHardware.motorPower(...)} means raw/open-loop FTC motor power: construction does not
 * acquire a run mode, each explicit {@link #setPower(double)} command conditionally establishes
 * {@code RUN_WITHOUT_ENCODER}, and {@link #stop()} sends zero without acquiring or restoring a run
 * mode. That adapter never resets the motor encoder. Device-managed FTC position and velocity
 * outputs own their corresponding run modes instead.</p>
 *
 * <p>This is a command seam, not an acknowledgement or measurement channel. A normally returning
 * call establishes only what was submitted to this configured top-level output. It does not prove
 * that a composite wrote every child or that hardware accepted, produced, or physically followed
 * the command.</p>
 *
 * <h2>Typical usage</h2>
 *
 * <pre>{@code
 * PowerOutput intake = FtcHardware.motorPower(hardwareMap, "intake", Direction.FORWARD);
 * intake.setPower(0.75);
 * }</pre>
 */
public interface PowerOutput {

    /**
     * Command the actuator with a normalized power value.
     *
     * <p>The exact physical interpretation depends on the implementation, but {@code -1.0} is full
     * reverse, {@code 0.0} is neutral, and {@code +1.0} is full forward in the logical command
     * domain. Callers should supply a finite value in the inclusive normalized range. Behavior for
     * an out-of-contract raw call is implementation-specific; adapters may add defensive
     * saturation, but callers must not rely on it as policy or applied-command readback.</p>
     *
     * @param power normalized power request, usually in {@code [-1.0, +1.0]}
     */
    void setPower(double power);

    /**
     * Return this implementation's cached seam-level command.
     *
     * <p>This is not a physical measurement or acknowledgement. A wrapper or composite may
     * transform one top-level request into different child commands, so this value also does not
     * imply per-child or atomic group truth. Consult the implementation's contract for when and how
     * its cache is updated.</p>
     *
     * @return cached seam-level power command reported by this implementation
     */
    double getCommandedPower();

    /**
     * Convenience helper that stops the output using the implementation's natural zero command.
     *
     * <p>The default behavior is equivalent to {@code setPower(0.0)}. Implementations may override
     * this when zero has a more specific meaning, such as brake versus coast. Normal return is a
     * top-level seam fact, not proof that every child or physical actuator stopped.</p>
     *
     * <p>A concrete implementation may distinguish an explicit zero command from lifecycle stop.
     * For example, {@code FtcHardware.motorPower(...).setPower(0.0)} still asserts raw/open-loop
     * command ownership, while its overridden {@code stop()} writes zero without acquiring or
     * restoring an FTC motor run mode.</p>
     */
    default void stop() {
        setPower(0.0);
    }
}
