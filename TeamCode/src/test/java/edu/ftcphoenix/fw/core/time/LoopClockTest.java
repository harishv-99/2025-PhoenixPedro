package edu.ftcphoenix.fw.core.time;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/** Verifies the deterministic heartbeat contract used by framework tests and robot loops. */
public final class LoopClockTest {

    private static final double EPSILON = 1e-12;

    @Test
    public void firstImplicitUpdateStartsAtCycleOne() {
        LoopClock clock = new LoopClock();

        clock.update(4.0);
        assertEquals(4.0, clock.nowSec(), EPSILON);
        assertEquals(0.0, clock.dtSec(), EPSILON);
        assertEquals(1L, clock.cycle());

        clock.update(4.025);
        assertEquals(4.025, clock.nowSec(), EPSILON);
        assertEquals(0.025, clock.dtSec(), EPSILON);
        assertEquals(2L, clock.cycle());
    }

    @Test
    public void explicitResetAdvancesCycleImmediatelyAndNeverAliasesOldCacheIdentity() {
        LoopClock clock = new LoopClock();
        assertEquals(0L, clock.cycle());

        clock.reset(10.0);
        assertEquals(10.0, clock.nowSec(), EPSILON);
        assertEquals(0.0, clock.dtSec(), EPSILON);
        assertEquals(1L, clock.cycle());

        clock.update(10.025);
        assertEquals(2L, clock.cycle());

        long beforeReset = clock.cycle();
        clock.reset(10.025);
        assertEquals(beforeReset + 1L, clock.cycle());
        assertEquals(10.025, clock.nowSec(), EPSILON);
        assertEquals(0.0, clock.dtSec(), EPSILON);

        clock.reset(10.025);
        assertEquals(beforeReset + 2L, clock.cycle());
        assertEquals(0.0, clock.dtSec(), EPSILON);
        assertTrue(clock.cycle() > beforeReset);
    }
}
