package edu.ftcphoenix.fw.core.time;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies clock identity, reset-epoch invalidation, and centralized freshness semantics. */
public final class LoopTimestampTest {

    private static final double EPSILON = 1e-12;

    @Test
    public void uninitializedClockCannotCreateApparentlyValidTimestamp() {
        LoopClock clock = new LoopClock();

        expectIllegalState("initialized", clock::nowTimestamp);
        expectIllegalState("initialized", () -> clock.timestampSecondsAgo(0.0));
    }

    @Test
    public void nonFiniteClockTimeCannotCreateApparentlyValidTimestamp() {
        LoopClock clock = startedAt(1.0);
        for (double invalidNow : new double[]{
                Double.NaN,
                Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY
        }) {
            clock.update(invalidNow);
            expectIllegalState("non-finite", clock::nowTimestamp);
            expectIllegalState("non-finite", () -> clock.timestampSecondsAgo(0.0));
        }
    }

    @Test
    public void clockFactoriesCreateCurrentAndDelayedTimestamps() {
        LoopClock clock = startedAt(10.0);

        LoopTimestamp now = clock.nowTimestamp();
        LoopTimestamp delayed = clock.timestampSecondsAgo(0.25);

        assertTrue(now.isAvailable());
        assertTrue(delayed.isAvailable());
        assertEquals(0.0, now.ageSec(clock), EPSILON);
        assertEquals(0.25, delayed.ageSec(clock), EPSILON);
        assertEquals(0.25, now.secondsSince(delayed), EPSILON);
        assertEquals(-0.25, delayed.secondsSince(now), EPSILON);
    }

    @Test
    public void unavailableFailsClosedWithoutPretendingToBeFresh() {
        LoopTimestamp unavailable = LoopTimestamp.unavailable();
        LoopClock clock = startedAt(1.0);

        assertFalse(unavailable.isAvailable());
        assertTrue(Double.isNaN(unavailable.ageSec(clock)));
        assertFalse(unavailable.isFresh(clock, 100.0));
        assertTrue(Double.isNaN(unavailable.secondsSince(clock.nowTimestamp())));
    }

    @Test
    public void freshnessBoundaryIsInclusiveAndAgesAsClockAdvances() {
        LoopClock clock = startedAt(5.0);
        LoopTimestamp captured = clock.nowTimestamp();

        clock.update(5.5);
        assertTrue(captured.isFresh(clock, 0.5));
        assertEquals(0.5, captured.ageSec(clock), EPSILON);

        clock.update(5.500001);
        assertFalse(captured.isFresh(clock, 0.5));
    }

    @Test
    public void tinyFutureDifferenceIsCurrentButMaterialFutureFailsClosed() {
        LoopClock clock = startedAt(10.0);
        LoopTimestamp captured = clock.nowTimestamp();

        clock.update(10.0 - 0.5e-6);
        assertEquals(0.0, captured.ageSec(clock), EPSILON);
        assertTrue(captured.isFresh(clock, 0.0));

        clock.update(10.0 - 2.0e-6);
        assertTrue(Double.isNaN(captured.ageSec(clock)));
        assertFalse(captured.isFresh(clock, 100.0));
    }

    @Test
    public void resetInvalidatesCapturedTimestampEvenAtSameTime() {
        LoopClock clock = startedAt(3.0);
        LoopTimestamp beforeReset = clock.nowTimestamp();

        clock.reset(3.0);
        LoopTimestamp afterReset = clock.nowTimestamp();

        assertTrue(beforeReset.isAvailable());
        assertTrue(Double.isNaN(beforeReset.ageSec(clock)));
        assertFalse(beforeReset.isFresh(clock, 10.0));
        assertTrue(Double.isNaN(afterReset.secondsSince(beforeReset)));
        assertEquals(0.0, afterReset.ageSec(clock), EPSILON);
    }

    @Test
    public void differentClockUseFailsWithActionableError() {
        LoopClock first = startedAt(2.0);
        LoopClock second = startedAt(2.0);
        LoopTimestamp firstTimestamp = first.nowTimestamp();
        LoopTimestamp secondTimestamp = second.nowTimestamp();

        expectIllegalArgument("different LoopClock",
                () -> firstTimestamp.ageSec(second));
        expectIllegalArgument("different LoopClock",
                () -> firstTimestamp.isFresh(second, 1.0));
        expectIllegalArgument("different LoopClock",
                () -> firstTimestamp.secondsSince(secondTimestamp));
    }

    @Test
    public void timestampFactoriesAndFreshnessRejectInvalidArguments() {
        LoopClock clock = startedAt(4.0);
        double[] invalidAges = {
                -0.001,
                Double.NaN,
                Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY
        };
        for (double ageSec : invalidAges) {
            expectIllegalArgument("ageSec", () -> clock.timestampSecondsAgo(ageSec));
            expectIllegalArgument("maxAgeSec", () -> clock.nowTimestamp().isFresh(clock, ageSec));
        }
    }

    @Test
    public void pairwiseDurationsRequireTheCurrentSharedEpoch() {
        LoopClock clock = startedAt(0.0);
        LoopTimestamp first = clock.nowTimestamp();
        clock.update(0.2);
        LoopTimestamp second = clock.nowTimestamp();

        assertEquals(0.2, second.secondsSince(first), EPSILON);

        clock.reset(100.0);
        assertTrue(Double.isNaN(second.secondsSince(first)));
    }

    private static LoopClock startedAt(double nowSec) {
        LoopClock clock = new LoopClock();
        clock.reset(nowSec);
        return clock;
    }

    private static void expectIllegalArgument(String messageFragment, Runnable action) {
        try {
            action.run();
            fail("Expected IllegalArgumentException containing " + messageFragment);
        } catch (IllegalArgumentException expected) {
            assertTrue("Expected message containing " + messageFragment + ", got: "
                            + expected.getMessage(),
                    expected.getMessage() != null
                            && expected.getMessage().contains(messageFragment));
        }
    }

    private static void expectIllegalState(String messageFragment, Runnable action) {
        try {
            action.run();
            fail("Expected IllegalStateException containing " + messageFragment);
        } catch (IllegalStateException expected) {
            assertTrue("Expected message containing " + messageFragment + ", got: "
                            + expected.getMessage(),
                    expected.getMessage() != null
                            && expected.getMessage().contains(messageFragment));
        }
    }
}
