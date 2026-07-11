package edu.ftcphoenix.fw.core.control;

import org.junit.Test;

import edu.ftcphoenix.fw.core.time.LoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/** Verifies defensive loop-time handling for target rate limiting. */
public final class SlewRateLimiterTest {

    private static final double EPSILON = 1e-12;

    @Test
    public void nonFiniteTimeHoldsOutputUntilFiniteBaselineReturns() {
        SlewRateLimiter limiter = new SlewRateLimiter(1.0);
        LoopClock clock = new LoopClock();
        clock.reset(0.0);

        assertEquals(10.0, limiter.calculate(10.0, clock), EPSILON);

        clock.update(Double.NaN);
        assertEquals(10.0, limiter.calculate(20.0, clock), EPSILON);
        assertTrue(limiter.wasLimited());

        clock.update(1.0);
        assertEquals(10.0, limiter.calculate(20.0, clock), EPSILON);
        assertTrue(limiter.wasLimited());

        clock.update(2.0);
        assertEquals(11.0, limiter.calculate(20.0, clock), EPSILON);
        assertTrue(limiter.wasLimited());
    }
}
