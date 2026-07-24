package edu.ftcphoenix.fw.testing;

import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Test-only controller that advances a real {@link LoopClock} deterministically.
 *
 * <p>This keeps tests on the production clock contract without adding another clock abstraction to
 * robot code. One call to {@link #nextCycle(double)} represents one OpMode loop cycle.</p>
 */
public final class ManualLoopClock {

    private final LoopClock clock = new LoopClock();
    private double nowSec;

    /**
     * Start at time zero. The constructor's explicit reset owns the first cycle identity.
     */
    public ManualLoopClock() {
        this(0.0);
    }

    /**
     * Start at the supplied absolute time. The constructor's explicit reset owns the first cycle
     * identity.
     *
     * @param initialTimeSec initial absolute time in seconds; must be finite
     */
    public ManualLoopClock(double initialTimeSec) {
        requireFinite(initialTimeSec, "initialTimeSec");
        nowSec = initialTimeSec;
        clock.reset(initialTimeSec);
    }

    /**
     * Return the real framework clock driven by this fixture.
     */
    public LoopClock clock() {
        return clock;
    }

    /**
     * Advance absolute time and the real clock by exactly one cycle.
     *
     * @param dtSec elapsed time in seconds; must be finite and non-negative
     * @return the advanced framework clock
     */
    public LoopClock nextCycle(double dtSec) {
        requireFinite(dtSec, "dtSec");
        if (dtSec < 0.0) {
            throw new IllegalArgumentException("dtSec must be >= 0, got " + dtSec);
        }

        double nextSec = nowSec + dtSec;
        requireFinite(nextSec, "resulting time");
        nowSec = nextSec;
        clock.update(nowSec);
        return clock;
    }

    private static void requireFinite(double value, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite, got " + value);
        }
    }
}
