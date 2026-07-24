package edu.ftcphoenix.fw.core.source;

import org.junit.Test;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;

import static org.junit.Assert.assertEquals;

/** Verifies the typed historical-time seam and its explicit current-only fallback. */
public final class TimeAwareSourcesTest {

    private static final double EPSILON = 1e-12;

    @Test
    public void defaultCurrentSampleUsesClockOwnedTimestamp() {
        LoopClock clock = startedAt(4.0);
        final LoopTimestamp[] requested = {LoopTimestamp.unavailable()};
        TimeAwareSource<String> source = (sampleClock, timestamp) -> {
            requested[0] = timestamp;
            return "value";
        };

        assertEquals("value", source.get(clock));
        assertEquals(0.0, requested[0].ageSec(clock), EPSILON);
    }

    @Test
    public void fixedAndCurrentOnlyExposeTheirDocumentedHistoricalBehavior() {
        LoopClock clock = startedAt(10.0);
        LoopTimestamp old = clock.timestampSecondsAgo(2.0);

        assertEquals("fixed", TimeAwareSources.fixed("fixed").getAt(clock, old));

        final int[] samples = {0};
        Source<String> current = sampleClock -> "current-" + ++samples[0];
        TimeAwareSource<String> currentOnly = TimeAwareSources.currentOnly(current);

        assertEquals("current-1", currentOnly.getAt(clock, old));
        assertEquals("current-2", currentOnly.get(clock));
    }

    private static LoopClock startedAt(double nowSec) {
        LoopClock clock = new LoopClock();
        clock.reset(nowSec);
        return clock;
    }
}
