package edu.ftcphoenix.fw.core.source;

import org.junit.Test;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies reset-safe timing in generic stateful source helpers. */
public final class SourceTimestampTest {

    @Test
    public void holdLastValidDoesNotRejuvenateValueAcrossSameTimeReset() {
        MutableSource raw = new MutableSource("ready");
        ManualLoopClock time = new ManualLoopClock(7.0);
        Source<String> held = raw.holdLastValid(
                value -> !"unknown".equals(value),
                1.0,
                "fallback"
        );

        assertEquals("ready", held.get(time.clock()));
        raw.value = "unknown";
        time.clock().reset(7.0);

        assertEquals("fallback", held.get(time.clock()));

        time.nextCycle(0.0);
        raw.value = "new";
        assertEquals("new", held.get(time.clock()));
    }

    @Test
    public void holdLastValidRejectsNonFiniteOrNegativeDuration() {
        MutableSource raw = new MutableSource("ready");
        for (double duration : new double[]{
                -0.01,
                Double.NaN,
                Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY
        }) {
            try {
                raw.holdLastValid(value -> true, duration, "fallback");
                fail("Expected invalid maxHoldSec " + duration + " to be rejected");
            } catch (IllegalArgumentException expected) {
                assertTrue(expected.getMessage().contains("maxHoldSec"));
            }
        }
    }

    private static final class MutableSource implements Source<String> {
        private String value;

        private MutableSource(String value) {
            this.value = value;
        }

        @Override
        public String get(LoopClock clock) {
            return value;
        }
    }
}
