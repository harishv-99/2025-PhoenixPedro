package edu.ftcphoenix.fw.core.control;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Loop-clocked scalar slew-rate limiter.
 *
 * <p>A slew-rate limiter shapes how quickly a value may change. It does not predict a future
 * timestep; on each sample it uses the time that actually elapsed since the previous accepted
 * sample and moves the output toward the input by at most {@code rate * dt}.</p>
 *
 * <p>The limiter is idempotent for a single {@link LoopClock#cycle()}: multiple calls with the
 * same cycle return the same output and do not advance the limiter twice. That makes it safe to
 * use inside source graphs that may be sampled by multiple consumers in one robot loop.</p>
 */
public final class SlewRateLimiter {
    private final double maxUpPerSec;
    private final double maxDownPerSec;

    private boolean initialized;
    private long lastCycle = Long.MIN_VALUE;
    private double lastSec = Double.NaN;
    private double lastInput;
    private double lastOutput;
    private double lastDelta;
    private double lastDtSec;
    private boolean lastLimited;

    /**
     * Create a symmetric limiter.
     */
    public SlewRateLimiter(double maxDeltaPerSec) {
        this(maxDeltaPerSec, maxDeltaPerSec);
    }

    /**
     * Create an asymmetric limiter.
     */
    public SlewRateLimiter(double maxUpPerSec, double maxDownPerSec) {
        if (!Double.isFinite(maxUpPerSec) || maxUpPerSec <= 0.0) {
            throw new IllegalArgumentException("maxUpPerSec must be finite and > 0, got " + maxUpPerSec);
        }
        if (!Double.isFinite(maxDownPerSec) || maxDownPerSec <= 0.0) {
            throw new IllegalArgumentException("maxDownPerSec must be finite and > 0, got " + maxDownPerSec);
        }
        this.maxUpPerSec = maxUpPerSec;
        this.maxDownPerSec = maxDownPerSec;
    }

    /**
     * Sample the limiter for this loop.
     *
     * <p>The first sample initializes directly to {@code input}. If you need hardware-safe
     * startup from a known current value, call {@link #reset(double, LoopClock)} first.</p>
     */
    public double calculate(double input, LoopClock clock) {
        if (clock == null) {
            initialized = true;
            lastCycle = Long.MIN_VALUE;
            lastSec = Double.NaN;
            lastInput = input;
            lastOutput = input;
            lastDelta = 0.0;
            lastDtSec = 0.0;
            lastLimited = false;
            return input;
        }

        long cycle = clock.cycle();
        if (cycle == lastCycle) {
            return lastOutput;
        }
        lastCycle = cycle;
        lastInput = input;

        double now = clock.nowSec();
        if (!initialized || !Double.isFinite(lastSec)) {
            initialized = true;
            lastSec = now;
            lastOutput = input;
            lastDelta = 0.0;
            lastDtSec = 0.0;
            lastLimited = false;
            return lastOutput;
        }

        double dt = Math.max(0.0, now - lastSec);
        lastSec = now;
        lastDtSec = dt;

        double delta = input - lastOutput;
        if (dt <= 0.0) {
            lastDelta = 0.0;
            lastLimited = Math.abs(delta) > 1e-9;
            return lastOutput;
        }

        double limitedDelta = delta;
        if (delta > 0.0) {
            limitedDelta = Math.min(delta, maxUpPerSec * dt);
        } else if (delta < 0.0) {
            limitedDelta = Math.max(delta, -maxDownPerSec * dt);
        }

        lastLimited = Math.abs(limitedDelta - delta) > 1e-9;
        lastDelta = limitedDelta;
        lastOutput += limitedDelta;
        return lastOutput;
    }

    /**
     * Reset so the next sample initializes directly to its input.
     */
    public void reset() {
        initialized = false;
        lastCycle = Long.MIN_VALUE;
        lastSec = Double.NaN;
        lastInput = 0.0;
        lastOutput = 0.0;
        lastDelta = 0.0;
        lastDtSec = 0.0;
        lastLimited = false;
    }

    /**
     * Reset to a known current output.
     */
    public void reset(double value, LoopClock clock) {
        initialized = true;
        lastCycle = clock != null ? clock.cycle() : Long.MIN_VALUE;
        lastSec = clock != null ? clock.nowSec() : Double.NaN;
        lastInput = value;
        lastOutput = value;
        lastDelta = 0.0;
        lastDtSec = 0.0;
        lastLimited = false;
    }

    /**
     * Most recent input value.
     */
    public double getLastInput() {
        return lastInput;
    }

    /**
     * Most recent output value.
     */
    public double getLastOutput() {
        return lastOutput;
    }

    /**
     * Most recent output change.
     */
    public double getLastDelta() {
        return lastDelta;
    }

    /**
     * Most recent elapsed time.
     */
    public double getLastDtSec() {
        return lastDtSec;
    }

    /**
     * Whether the most recent sample had to be limited.
     */
    public boolean wasLimited() {
        return lastLimited;
    }

    /**
     * Positive rate limit in units/sec.
     */
    public double maxUpPerSec() {
        return maxUpPerSec;
    }

    /**
     * Negative rate limit magnitude in units/sec.
     */
    public double maxDownPerSec() {
        return maxDownPerSec;
    }

    /**
     * Emit limiter state for debug telemetry.
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) return;
        String p = (prefix == null || prefix.isEmpty()) ? "slew" : prefix;
        dbg.addData(p + ".class", "SlewRateLimiter")
                .addData(p + ".maxUpPerSec", maxUpPerSec)
                .addData(p + ".maxDownPerSec", maxDownPerSec)
                .addData(p + ".initialized", initialized)
                .addData(p + ".lastInput", lastInput)
                .addData(p + ".lastOutput", lastOutput)
                .addData(p + ".lastDelta", lastDelta)
                .addData(p + ".lastDtSec", lastDtSec)
                .addData(p + ".lastLimited", lastLimited);
    }
}
