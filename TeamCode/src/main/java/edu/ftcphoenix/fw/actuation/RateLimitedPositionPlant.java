package edu.ftcphoenix.fw.actuation;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Position-preserving rate limiter for {@link PositionPlant} targets.
 *
 * <p>{@link RateLimitedPlant} is intentionally generic and therefore returns only {@link Plant}.
 * This wrapper keeps the richer {@link PositionPlant} metadata available to planners and
 * calibration tasks while limiting target changes in plant units.</p>
 */
public final class RateLimitedPositionPlant implements PositionPlant {
    private final PositionPlant inner;
    private final double maxUpPerSec;
    private final double maxDownPerSec;
    private double desiredTarget;
    private double appliedTarget;
    private double lastSec = Double.NaN;

    /**
     * Creates a symmetric position rate limiter in plant units per second.
     */
    public RateLimitedPositionPlant(PositionPlant inner, double maxDeltaPerSec) {
        this(inner, maxDeltaPerSec, maxDeltaPerSec);
    }

    /**
     * Creates an asymmetric position rate limiter in plant units per second.
     */
    public RateLimitedPositionPlant(PositionPlant inner, double maxUpPerSec, double maxDownPerSec) {
        this.inner = Objects.requireNonNull(inner, "inner");
        if (maxUpPerSec < 0.0 || maxDownPerSec < 0.0) {
            throw new IllegalArgumentException("rate limits must be >= 0");
        }
        this.maxUpPerSec = maxUpPerSec;
        this.maxDownPerSec = maxDownPerSec;
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

    /**
     * Returns the unclamped target requested by callers.
     */
    public double getDesiredTarget() {
        return desiredTarget;
    }

    @Override
    public void update(LoopClock clock) {
        if (clock != null) {
            double now = clock.nowSec();
            if (!Double.isFinite(lastSec)) {
                appliedTarget = desiredTarget;
            } else {
                double dt = Math.max(0.0, now - lastSec);
                double delta = desiredTarget - appliedTarget;
                double limit = delta >= 0.0 ? maxUpPerSec * dt : maxDownPerSec * dt;
                if (Math.abs(delta) <= limit) {
                    appliedTarget = desiredTarget;
                } else {
                    appliedTarget += Math.copySign(limit, delta);
                }
            }
            lastSec = now;
        }
        inner.setTarget(appliedTarget);
        inner.update(clock);
    }

    @Override
    public void reset() {
        inner.reset();
        lastSec = Double.NaN;
        desiredTarget = inner.getTarget();
        appliedTarget = desiredTarget;
    }

    @Override
    public void stop() {
        inner.stop();
        desiredTarget = inner.getTarget();
        appliedTarget = desiredTarget;
        lastSec = Double.NaN;
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
    public Topology topology() {
        return inner.topology();
    }

    @Override
    public double period() {
        return inner.period();
    }

    @Override
    public ScalarRange targetRange() {
        return inner.targetRange();
    }

    @Override
    public Source<ScalarRange> targetRangeSource() {
        return inner.targetRangeSource();
    }

    @Override
    public ScalarSource positionSource() {
        return inner.positionSource();
    }

    @Override
    public boolean isReferenced() {
        return inner.isReferenced();
    }

    @Override
    public String referenceStatus() {
        return inner.referenceStatus();
    }

    @Override
    public void establishReferenceAt(double plantPosition) {
        inner.establishReferenceAt(plantPosition);
    }

    @Override
    public void establishReferenceAt(double plantPosition, LoopClock clock) {
        inner.establishReferenceAt(plantPosition, clock);
    }

    @Override
    public boolean supportsCalibrationSearch() {
        return inner.supportsCalibrationSearch();
    }

    @Override
    public void beginCalibrationSearch(double power) {
        inner.beginCalibrationSearch(power);
    }

    @Override
    public void endCalibrationSearch(boolean stopOutput) {
        inner.endCalibrationSearch(stopOutput);
    }

    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "rateLimitedPositionPlant" : prefix;
        dbg.addData(p + ".desiredTarget", desiredTarget)
                .addData(p + ".appliedTarget", appliedTarget)
                .addData(p + ".maxUpPerSec", maxUpPerSec)
                .addData(p + ".maxDownPerSec", maxDownPerSec);
        inner.debugDump(dbg, p + ".inner");
    }
}
