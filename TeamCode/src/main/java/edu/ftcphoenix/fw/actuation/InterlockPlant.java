package edu.ftcphoenix.fw.actuation;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * A {@link Plant} decorator that can temporarily block target updates while still allowing the
 * wrapped plant to keep updating.
 *
 * <p>This is useful for simple safety interlocks such as "only move the slide when the wrist is in
 * a safe zone". When blocked, the inner plant keeps its last accepted target.</p>
 */
public final class InterlockPlant implements Plant {

    /**
     * Predicate queried each loop to decide whether new targets may pass through.
     */
    public interface Gate {
        boolean allowsTarget(double target);
    }

    private final Plant inner;
    private final Gate gate;
    private double desiredTarget;
    private double appliedTarget;

    public InterlockPlant(Plant inner, Gate gate) {
        this.inner = Objects.requireNonNull(inner, "inner");
        this.gate = Objects.requireNonNull(gate, "gate");
        this.desiredTarget = inner.getTarget();
        this.appliedTarget = desiredTarget;
    }

    @Override
    public void setTarget(double target) {
        desiredTarget = target;
    }

    @Override
    public double getTarget() {
        return appliedTarget;
    }

    public double getDesiredTarget() {
        return desiredTarget;
    }

    @Override
    public void update(LoopClock clock) {
        if (gate.allowsTarget(desiredTarget)) {
            appliedTarget = desiredTarget;
            inner.setTarget(appliedTarget);
        }
        inner.update(clock);
    }

    @Override
    public void reset() {
        inner.reset();
        desiredTarget = inner.getTarget();
        appliedTarget = desiredTarget;
    }

    @Override
    public void stop() {
        inner.stop();
        desiredTarget = inner.getTarget();
        appliedTarget = desiredTarget;
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
                .addData(p + ".appliedTarget", appliedTarget);
        inner.debugDump(dbg, p + ".inner");
    }
}
