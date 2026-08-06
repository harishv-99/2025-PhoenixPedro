package edu.ftcphoenix.fw.core.source;

import org.junit.Test;

import java.util.LinkedHashMap;
import java.util.Map;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

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
    public void memoizedSourceRetriesFailureWithoutReturningZeroOrAStaleValue() {
        ManualLoopClock time = new ManualLoopClock();
        RuntimeException expected = new IllegalStateException("scalar sample failed");
        int[] attempts = {0};
        ScalarSource memoized = ScalarSource.of(() -> {
            int attempt = ++attempts[0];
            if (attempt == 1 || attempt == 3) {
                throw expected;
            }
            return attempt == 2 ? 4.5 : 8.5;
        }).memoized();

        assertSame(expected, captureFailure(() -> memoized.getAsDouble(time.clock())));
        assertEquals(4.5, memoized.getAsDouble(time.clock()), EPSILON);
        assertEquals(4.5, memoized.getAsDouble(time.clock()), EPSILON);

        time.nextCycle(0.02);
        assertSame(expected, captureFailure(() -> memoized.getAsDouble(time.clock())));
        assertEquals(8.5, memoized.getAsDouble(time.clock()), EPSILON);
        assertEquals(8.5, memoized.getAsDouble(time.clock()), EPSILON);
        assertEquals(4, attempts[0]);
    }

    @Test
    public void scalarMemoizedSourceRejectsReentryThenRecovers() {
        LoopClock clock = new ManualLoopClock().clock();
        ScalarSource[] memoizedRef = new ScalarSource[1];
        boolean[] reenter = {true};
        ScalarSource raw = sampleClock -> reenter[0]
                ? memoizedRef[0].getAsDouble(sampleClock)
                : 6.0;
        memoizedRef[0] = raw.memoized();

        RuntimeException failure =
                captureFailure(() -> memoizedRef[0].getAsDouble(clock));
        assertTrue(failure instanceof IllegalStateException);
        assertTrue(failure.getMessage().contains("sampling cycle"));

        reenter[0] = false;
        assertEquals(6.0, memoizedRef[0].getAsDouble(clock), EPSILON);
        assertEquals(6.0, memoizedRef[0].getAsDouble(clock), EPSILON);
    }

    @Test
    public void holdLastValidDoesNotRejuvenateValueAcrossSameTimeReset() {
        MutableScalarSource raw = new MutableScalarSource(8.0);
        ManualLoopClock time = new ManualLoopClock(4.0);
        ScalarSource held = raw.holdLastValid(Double::isFinite, 1.0, -1.0);

        assertEquals(8.0, held.getAsDouble(time.clock()), EPSILON);
        raw.value = Double.NaN;
        time.clock().reset(4.0);

        assertEquals(-1.0, held.getAsDouble(time.clock()), EPSILON);

        time.nextCycle(0.0);
        raw.value = 9.0;
        assertEquals(9.0, held.getAsDouble(time.clock()), EPSILON);
    }

    @Test
    public void holdLastValidRejectsNonFiniteOrNegativeDuration() {
        for (double duration : new double[]{
                -0.01,
                Double.NaN,
                Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY
        }) {
            try {
                ScalarSource.constant(1.0).holdLastFinite(duration, -1.0);
                fail("Expected invalid maxHoldSec " + duration + " to be rejected");
            } catch (IllegalArgumentException expected) {
                assertTrue(expected.getMessage().contains("maxHoldSec"));
            }
        }
    }

    @Test
    public void holdLastValidRetriesPredicateFailureWithoutChangingHistory() {
        ManualLoopClock time = new ManualLoopClock();
        MutableScalarSource raw = new MutableScalarSource(8.0);
        boolean[] predicateFails = {false};
        RuntimeException expected = new IllegalStateException("predicate failed");
        ScalarSource held = raw.holdLastValid(value -> {
            if (predicateFails[0]) {
                throw expected;
            }
            return Double.isFinite(value);
        }, 1.0, -1.0);

        assertEquals(8.0, held.getAsDouble(time.clock()), EPSILON);
        time.nextCycle(0.25);
        raw.value = Double.NaN;
        predicateFails[0] = true;
        assertSame(expected, captureFailure(() -> held.getAsDouble(time.clock())));

        predicateFails[0] = false;
        assertEquals(8.0, held.getAsDouble(time.clock()), EPSILON);
        assertEquals(8.0, held.getAsDouble(time.clock()), EPSILON);
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
    public void ratePerSecondRebaselinesAcrossSameTimeReset() {
        MutableScalarSource position = new MutableScalarSource(10.0);
        LoopClock clock = new LoopClock();
        clock.reset(5.0);
        ScalarSource rate = position.ratePerSecond();

        assertEquals(0.0, rate.getAsDouble(clock), EPSILON);
        clock.update(6.0);
        position.value = 12.0;
        assertEquals(2.0, rate.getAsDouble(clock), EPSILON);

        clock.reset(6.0);
        position.value = 100.0;
        assertEquals(0.0, rate.getAsDouble(clock), EPSILON);

        clock.update(6.5);
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
    public void ratePerSecondRetriesTimestampFailureFromItsLastPublishedBaseline() {
        MutableScalarSource position = new MutableScalarSource(10.0);
        LoopClock firstClock = new LoopClock();
        firstClock.reset(0.0);
        ScalarSource rate = position.ratePerSecond();

        assertEquals(0.0, rate.getAsDouble(firstClock), EPSILON);

        LoopClock wrongClock = new LoopClock();
        wrongClock.reset(0.0);
        wrongClock.update(1.0);
        position.value = 12.0;
        RuntimeException wrongClockFailure =
                captureFailure(() -> rate.getAsDouble(wrongClock));
        assertTrue(wrongClockFailure instanceof IllegalArgumentException);

        firstClock.update(1.0);
        assertEquals(2.0, rate.getAsDouble(firstClock), EPSILON);
        assertEquals(2.0, rate.getAsDouble(firstClock), EPSILON);
        assertEquals(3, position.sampleCount);
    }

    @Test
    public void hysteresisAndHoldUnlessRetryWithoutPublishingPhantomState() {
        ManualLoopClock time = new ManualLoopClock();
        double[] value = {0.0};
        boolean[] fail = {false};
        RuntimeException expected = new IllegalStateException("value failed");
        ScalarSource raw = ScalarSource.of(() -> {
            if (fail[0]) {
                throw expected;
            }
            return value[0];
        });
        BooleanSource above = raw.hysteresisAbove(10.0, 5.0);
        ScalarSource held = raw.holdLastUnless(BooleanSource.constant(true), -1.0);

        assertFalse(above.getAsBoolean(time.clock()));
        assertEquals(0.0, held.getAsDouble(time.clock()), EPSILON);

        time.nextCycle(0.02);
        value[0] = 12.0;
        fail[0] = true;
        assertSame(expected, captureFailure(() -> above.getAsBoolean(time.clock())));
        assertSame(expected, captureFailure(() -> held.getAsDouble(time.clock())));

        fail[0] = false;
        assertTrue(above.getAsBoolean(time.clock()));
        assertTrue(above.getAsBoolean(time.clock()));
        assertEquals(12.0, held.getAsDouble(time.clock()), EPSILON);
        assertEquals(12.0, held.getAsDouble(time.clock()), EPSILON);
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

    private static RuntimeException captureFailure(Runnable action) {
        try {
            action.run();
            fail("Expected source operation to fail");
            return null;
        } catch (RuntimeException failure) {
            return failure;
        }
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
