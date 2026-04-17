package edu.ftcphoenix.fw.actuation;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.math.MathUtil;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * A {@link Plant} decorator that rate-limits target changes before forwarding them to an inner
 * plant.
 */
public class RateLimitedPlant implements Plant {

    private final Plant inner;
    private final double maxUpPerSec;
    private final double maxDownPerSec;

    private double currentTarget;
    private double desiredTarget;

    public RateLimitedPlant(Plant inner, double maxUpPerSec, double maxDownPerSec) {
        this.inner = Objects.requireNonNull(inner, "inner");
        if (maxUpPerSec < 0.0) {
            throw new IllegalArgumentException("maxUpPerSec must be >= 0, got " + maxUpPerSec);
        }
        if (maxDownPerSec < 0.0) {
            throw new IllegalArgumentException("maxDownPerSec must be >= 0, got " + maxDownPerSec);
        }
        this.maxUpPerSec = maxUpPerSec;
        this.maxDownPerSec = maxDownPerSec;
        this.currentTarget = inner.getTarget();
        this.desiredTarget = currentTarget;
    }

    public RateLimitedPlant(Plant inner, double maxDeltaPerSec) {
        this(inner, maxDeltaPerSec, maxDeltaPerSec);
    }

    @Override
    public void setTarget(double target) {
        desiredTarget = target;
    }

    @Override
    public double getTarget() {
        return currentTarget;
    }

    public double getCommandedTarget() {
        return desiredTarget;
    }

    @Override
    public void update(LoopClock clock) {
        double dtSec = Math.max(0.0, clock.dtSec());
        double delta = desiredTarget - currentTarget;
        double limitedDelta = delta;
        if (delta > 0.0) {
            limitedDelta = Math.min(delta, maxUpPerSec * dtSec);
        } else if (delta < 0.0) {
            limitedDelta = Math.max(delta, -maxDownPerSec * dtSec);
        }
        currentTarget += limitedDelta;
        currentTarget = MathUtil.clamp(currentTarget,
                Math.min(currentTarget, desiredTarget),
                Math.max(currentTarget, desiredTarget));
        inner.setTarget(currentTarget);
        inner.update(clock);
    }

    @Override
    public void reset() {
        inner.reset();
        currentTarget = inner.getTarget();
        desiredTarget = currentTarget;
    }

    @Override
    public void stop() {
        inner.stop();
        currentTarget = inner.getTarget();
        desiredTarget = currentTarget;
    }

    @Override
    public boolean atSetpoint() {
        return inner.atSetpoint();
    }

    @Override
    public boolean hasFeedback() {
        return inner.hasFeedback();
    }

    @Override
    public double getMeasurement() {
        return inner.getMeasurement();
    }

    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "plant" : prefix;
        dbg.addData(p + ".desiredTarget", desiredTarget)
                .addData(p + ".currentTarget", currentTarget)
                .addData(p + ".maxUpPerSec", maxUpPerSec)
                .addData(p + ".maxDownPerSec", maxDownPerSec);
        inner.debugDump(dbg, p + ".inner");
    }
}
