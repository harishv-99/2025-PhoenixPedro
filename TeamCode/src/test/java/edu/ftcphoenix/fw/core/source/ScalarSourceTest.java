package edu.ftcphoenix.fw.core.source;

import org.junit.Test;

import java.util.LinkedHashMap;
import java.util.Map;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

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

    @Test
    public void ratePerSecondUsesAcceptedElapsedTimeAndCachesEachCycle() {
        MutableScalarSource position = new MutableScalarSource(10.0);
        ManualLoopClock manualClock = new ManualLoopClock(5.0);
        ScalarSource rate = position.ratePerSecond();

        assertEquals(0.0, rate.getAsDouble(manualClock.clock()), EPSILON);
        assertEquals(1, position.sampleCount);

        manualClock.nextCycle(0.5);
        position.value = 12.0;
        assertEquals(4.0, rate.getAsDouble(manualClock.clock()), EPSILON);

        position.value = 100.0;
        assertEquals(4.0, rate.getAsDouble(manualClock.clock()), EPSILON);
        assertEquals(2, position.sampleCount);

        manualClock.nextCycle(0.25);
        manualClock.nextCycle(0.25);
        position.value = 13.0;
        assertEquals(2.0, rate.getAsDouble(manualClock.clock()), EPSILON);
        assertEquals(3, position.sampleCount);
    }

    @Test
    public void ratePerSecondPreservesCallerPositionUnitsAndDirection() {
        MutableScalarSource positionInches = new MutableScalarSource(3.0);
        ManualLoopClock manualClock = new ManualLoopClock();
        ScalarSource velocityCentimetersPerSec = positionInches
                .scaled(2.54)
                .ratePerSecond();

        assertEquals(0.0, velocityCentimetersPerSec.getAsDouble(manualClock.clock()), EPSILON);

        manualClock.nextCycle(0.5);
        positionInches.value = 2.0;
        assertEquals(-5.08,
                velocityCentimetersPerSec.getAsDouble(manualClock.clock()),
                EPSILON);
    }

    @Test
    public void ratePerSecondDoesNotConsumePositionWhenNoTimeElapsed() {
        MutableScalarSource position = new MutableScalarSource(0.0);
        ManualLoopClock manualClock = new ManualLoopClock();
        ScalarSource rate = position.ratePerSecond();

        assertEquals(0.0, rate.getAsDouble(manualClock.clock()), EPSILON);

        manualClock.nextCycle(1.0);
        position.value = 2.0;
        assertEquals(2.0, rate.getAsDouble(manualClock.clock()), EPSILON);

        manualClock.nextCycle(0.0);
        position.value = 5.0;
        assertEquals(2.0, rate.getAsDouble(manualClock.clock()), EPSILON);

        manualClock.nextCycle(1.0);
        position.value = 7.0;
        assertEquals(5.0, rate.getAsDouble(manualClock.clock()), EPSILON);
    }

    @Test
    public void ratePerSecondRebaselinesWhenTimeRegresses() {
        MutableScalarSource position = new MutableScalarSource(10.0);
        LoopClock clock = new LoopClock();
        clock.reset(10.0);
        ScalarSource rate = position.ratePerSecond();

        assertEquals(0.0, rate.getAsDouble(clock), EPSILON);
        clock.update(11.0);
        position.value = 12.0;
        assertEquals(2.0, rate.getAsDouble(clock), EPSILON);

        clock.update(5.0);
        position.value = 100.0;
        assertEquals(0.0, rate.getAsDouble(clock), EPSILON);

        clock.update(5.5);
        position.value = 101.0;
        assertEquals(2.0, rate.getAsDouble(clock), EPSILON);
    }

    @Test
    public void ratePerSecondRejectsNonFiniteSamplesWithoutPoisoningBaseline() {
        MutableScalarSource position = new MutableScalarSource(Double.NaN);
        ManualLoopClock manualClock = new ManualLoopClock();
        ScalarSource rate = position.ratePerSecond();

        assertTrue(Double.isNaN(rate.getAsDouble(manualClock.clock())));

        manualClock.nextCycle(1.0);
        position.value = 10.0;
        assertEquals(0.0, rate.getAsDouble(manualClock.clock()), EPSILON);

        manualClock.nextCycle(1.0);
        position.value = 12.0;
        assertEquals(2.0, rate.getAsDouble(manualClock.clock()), EPSILON);

        manualClock.clock().update(Double.NaN);
        position.value = 14.0;
        assertTrue(Double.isNaN(rate.getAsDouble(manualClock.clock())));

        manualClock.nextCycle(1.0);
        assertEquals(2.0, rate.getAsDouble(manualClock.clock()), EPSILON);

        manualClock.nextCycle(1.0);
        position.value = Double.POSITIVE_INFINITY;
        assertTrue(Double.isNaN(rate.getAsDouble(manualClock.clock())));

        manualClock.nextCycle(1.0);
        position.value = 18.0;
        assertEquals(2.0, rate.getAsDouble(manualClock.clock()), EPSILON);
    }

    @Test
    public void ratePerSecondRejectsNonFiniteCalculationWithoutPoisoningBaseline() {
        MutableScalarSource position = new MutableScalarSource(-Double.MAX_VALUE);
        ManualLoopClock manualClock = new ManualLoopClock();
        ScalarSource rate = position.ratePerSecond();

        assertEquals(0.0, rate.getAsDouble(manualClock.clock()), EPSILON);

        manualClock.nextCycle(1.0);
        position.value = Double.MAX_VALUE;
        assertTrue(Double.isNaN(rate.getAsDouble(manualClock.clock())));

        manualClock.nextCycle(1.0);
        position.value = 0.0;
        assertEquals(Double.MAX_VALUE / 2.0, rate.getAsDouble(manualClock.clock()), 0.0);
    }

    @Test
    public void ratePerSecondResetClearsStateAndPropagatesUpstream() {
        MutableScalarSource position = new MutableScalarSource(1.0);
        ManualLoopClock manualClock = new ManualLoopClock();
        ScalarSource rate = position.ratePerSecond();

        assertEquals(0.0, rate.getAsDouble(manualClock.clock()), EPSILON);
        manualClock.nextCycle(1.0);
        position.value = 3.0;
        assertEquals(2.0, rate.getAsDouble(manualClock.clock()), EPSILON);

        rate.reset();
        assertEquals(1, position.resetCount);
        assertEquals(0.0, rate.getAsDouble(manualClock.clock()), EPSILON);
        assertEquals(3, position.sampleCount);
    }

    @Test
    public void ratePerSecondDebugDumpReportsLiveStateAndDelegates() {
        MutableScalarSource position = new MutableScalarSource(1.0);
        ManualLoopClock manualClock = new ManualLoopClock();
        ScalarSource rate = position.ratePerSecond();
        rate.getAsDouble(manualClock.clock());

        rate.debugDump(null, "ignored");
        CapturingDebugSink debug = new CapturingDebugSink();
        rate.debugDump(debug, null);

        assertEquals("RatePerSecondScalar", debug.data.get("ratePerSecond.class"));
        assertEquals(Boolean.TRUE, debug.data.get("ratePerSecond.hasBaseline"));
        assertEquals(1.0, number(debug, "ratePerSecond.lastAcceptedValue"), EPSILON);
        assertEquals(0.0, number(debug, "ratePerSecond.lastFiniteRatePerSec"), EPSILON);
        assertEquals("MutableScalarSource", debug.data.get("ratePerSecond.src.class"));
        assertFalse(debug.data.containsKey("ignored.class"));
    }

    private static double number(CapturingDebugSink debug, String key) {
        return ((Number) debug.data.get(key)).doubleValue();
    }

    private static final class MutableScalarSource implements ScalarSource {
        private double value;
        private int sampleCount;
        private int resetCount;

        private MutableScalarSource(double value) {
            this.value = value;
        }

        @Override
        public double getAsDouble(LoopClock clock) {
            sampleCount++;
            return value;
        }

        @Override
        public void reset() {
            resetCount++;
        }

        @Override
        public void debugDump(DebugSink dbg, String prefix) {
            if (dbg == null) return;
            String p = (prefix == null || prefix.isEmpty()) ? "mutableScalar" : prefix;
            dbg.addData(p + ".class", "MutableScalarSource");
        }
    }

    private static final class CapturingDebugSink implements DebugSink {
        private final Map<String, Object> data = new LinkedHashMap<>();

        @Override
        public DebugSink addData(String key, Object value) {
            data.put(key, value);
            return this;
        }

        @Override
        public DebugSink addLine(String text) {
            return this;
        }
    }
}
