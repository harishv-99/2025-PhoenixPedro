package edu.ftcphoenix.fw.actuation;

import java.util.Objects;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.controller.*;
import edu.ftcphoenix.fw.debug.DebugSink;

/**
 * A {@link Plant} decorator that rate-limits how quickly the target can
 * change before it is applied to an underlying plant.
 *
 * <p>Think of this as:</p>
 *
 * <pre>{@code
 * // raw plant (no smoothing)
 * Plant shooter = Plants.velocity(
 *     hw.flywheelMotor(),
 *     /* maxRps = * / 50.0
 * );
 *
 * // rate-limited wrapper: clamp changes to +/- 30 units per second
 * Plant smoothShooter = new RateLimitedPlant(shooter, 30.0);
 *
 * // wrap in a controller that lets you change shooter "modes"
 * enum ShooterMode { OFF, LOW, HIGH }
 *
 * GoalController<ShooterMode> shooterCtrl =
 *     GoalController.forEnum(
 *         shooter,
 *         ShooterMode.class,
 *         ShooterMode.OFF,
 *         GoalController.target(ShooterMode.OFF,  0.0),
 *         GoalController.target(ShooterMode.LOW, 30.0),
 *         GoalController.target(ShooterMode.HIGH,45.0)
 *     );
 * }</pre>
 *
 * <p>Typical use cases:</p>
 * <ul>
 *   <li>Soft-starting intake or drive power to avoid brownouts.</li>
 *   <li>Slewing shooter velocity between modes to reduce overshoot.</li>
 *   <li>Applying a low-pass / "slew rate" filter on any scalar target.</li>
 * </ul>
 */
class RateLimitedPlant implements Plant {

    private final Plant inner;

    /**
     * Maximum allowed increase per second (in plant units).
     */
    private final double maxUpPerSec;

    /**
     * Maximum allowed decrease per second (in plant units, positive).
     */
    private final double maxDownPerSec;

    /**
     * Last target value actually applied to the inner plant.
     */
    private double currentTarget;

    /**
     * Commanded target (what callers <em>want</em> the plant to be at).
     */
    private double desiredTarget;

    /**
     * Construct a {@link RateLimitedPlant} with separate up/down limits.
     *
     * @param inner         plant to wrap
     * @param maxUpPerSec   maximum increase per second (target units per second)
     *                      must be {@code >= 0}
     * @param maxDownPerSec maximum decrease per second (target units per second,
     *                      positive). The actual downward rate is
     *                      {@code -maxDownPerSec}.
     */
    public RateLimitedPlant(Plant inner,
                            double maxUpPerSec,
                            double maxDownPerSec) {

        this.inner = Objects.requireNonNull(inner, "inner plant is required");

        if (maxUpPerSec < 0.0) {
            throw new IllegalArgumentException(
                    "maxUpPerSec must be >= 0, got " + maxUpPerSec);
        }
        if (maxDownPerSec < 0.0) {
            throw new IllegalArgumentException(
                    "maxDownPerSec must be >= 0, got " + maxDownPerSec);
        }

        this.maxUpPerSec = maxUpPerSec;
        this.maxDownPerSec = maxDownPerSec;

        double initial = inner.getTarget();
        this.currentTarget = initial;
        this.desiredTarget = initial;
    }

    /**
     * Construct a {@link RateLimitedPlant} with a symmetric rate limit for
     * both increases and decreases.
     *
     * @param inner          plant to wrap
     * @param maxDeltaPerSec maximum absolute rate of change per second
     *                       (target units per second, must be >= 0)
     */
    public RateLimitedPlant(Plant inner,
                            double maxDeltaPerSec) {
        this(inner, maxDeltaPerSec, maxDeltaPerSec);
    }

    // ---------------------------------------------------------------------
    // Plant implementation
    // ---------------------------------------------------------------------

    /**
     * Set the desired target. This does not immediately change the target
     * applied to the inner plant; instead the value is slewed towards this
     * desired target at the configured rate limits on each {@link #update}.
     */
    @Override
    public void setTarget(double target) {
        this.desiredTarget = target;
    }

    /**
     * @return the last target value that was actually applied to the inner
     * plant after rate limiting.
     */
    @Override
    public double getTarget() {
        return currentTarget;
    }

    /**
     * @return the commanded target as most recently set via {@link #setTarget(double)}.
     */
    public double getCommandedTarget() {
        return desiredTarget;
    }

    @Override
    public void stop() {
        inner.stop();
        // After a stop, treat the inner plant's target as our new baseline.
        currentTarget = inner.getTarget();
        desiredTarget = currentTarget;
    }


    @Override
    public void update(double dtSec) {
        if (dtSec < 0.0) {
            dtSec = 0.0;
        }

        double maxUp = maxUpPerSec * dtSec;
        double maxDown = maxDownPerSec * dtSec;

        double delta = desiredTarget - currentTarget;

        if (delta > maxUp) {
            delta = maxUp;
        } else if (delta < -maxDown) {
            delta = -maxDown;
        }

        currentTarget += delta;

        inner.setTarget(currentTarget);
        inner.update(dtSec);
    }

    @Override
    public void reset() {
        inner.reset();
        double t = inner.getTarget();
        this.currentTarget = t;
        this.desiredTarget = t;
    }

    @Override
    public boolean atSetpoint() {
        // Defer to the inner plant's definition of at-setpoint, based on the
        // rate-limited target we are actually feeding it.
        return inner.atSetpoint();
    }

    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "rate" : prefix;
        dbg.addData(p + ".desiredTarget", desiredTarget)
                .addData(p + ".currentTarget", currentTarget)
                .addData(p + ".maxUpPerSec", maxUpPerSec)
                .addData(p + ".maxDownPerSec", maxDownPerSec);
        inner.debugDump(dbg, p + ".inner");
    }

    // ---------------------------------------------------------------------
    // Introspection
    // ---------------------------------------------------------------------

    /**
     * @return the wrapped inner plant.
     */
    public Plant getInner() {
        return inner;
    }

    public double getMaxUpPerSec() {
        return maxUpPerSec;
    }

    public double getMaxDownPerSec() {
        return maxDownPerSec;
    }
}
