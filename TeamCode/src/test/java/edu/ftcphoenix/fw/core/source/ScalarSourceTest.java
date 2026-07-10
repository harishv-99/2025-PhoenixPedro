package edu.ftcphoenix.fw.core.source;

import org.junit.Test;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;

/** Verifies representative per-cycle behavior for scalar source composition. */
public final class ScalarSourceTest {

    private static final double EPSILON = 1e-12;

    @Test
    public void memoizedSourceSamplesUpstreamOncePerCycle() {
        final int[] sampleCount = {0};
        final int[] resetCount = {0};
        ScalarSource upstream = new ScalarSource() {
            @Override
            public double getAsDouble(LoopClock clock) {
                sampleCount[0]++;
                return sampleCount[0];
            }

            @Override
            public void reset() {
                resetCount[0]++;
            }
        };

        ManualLoopClock manualClock = new ManualLoopClock(5.0);
        ScalarSource memoized = upstream.memoized();

        assertEquals(1.0, memoized.getAsDouble(manualClock.clock()), EPSILON);
        assertEquals(1.0, memoized.getAsDouble(manualClock.clock()), EPSILON);
        assertEquals(1, sampleCount[0]);

        manualClock.nextCycle(0.02);
        assertEquals(2.0, memoized.getAsDouble(manualClock.clock()), EPSILON);
        assertEquals(2, sampleCount[0]);

        memoized.reset();
        assertEquals(1, resetCount[0]);
        assertEquals(3.0, memoized.getAsDouble(manualClock.clock()), EPSILON);
        assertEquals(3, sampleCount[0]);
    }
}
