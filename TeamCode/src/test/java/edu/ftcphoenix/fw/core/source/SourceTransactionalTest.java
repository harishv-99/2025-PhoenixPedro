package edu.ftcphoenix.fw.core.source;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies successful-only publication for the generic stateful source decorators. */
public final class SourceTransactionalTest {

    @Test
    public void memoizedRetriesFailuresAndRepeatsTheExactSuccessfulValue() {
        ManualLoopClock time = new ManualLoopClock();
        RuntimeException expected = new IllegalStateException("transient sample failure");
        Object first = new Object();
        Object second = new Object();
        int[] attempts = {0};

        Source<Object> memoized = Source.of(clock -> {
            int attempt = ++attempts[0];
            if (attempt == 1 || attempt == 3) {
                throw expected;
            }
            return attempt == 2 ? first : second;
        }).memoized();

        assertSame(expected, captureFailure(() -> memoized.get(time.clock())));
        assertSame(first, memoized.get(time.clock()));
        assertSame(first, memoized.get(time.clock()));

        time.nextCycle(0.02);
        assertSame(expected, captureFailure(() -> memoized.get(time.clock())));
        assertSame(second, memoized.get(time.clock()));
        assertSame(second, memoized.get(time.clock()));
        assertEquals(4, attempts[0]);
    }

    @Test
    public void memoizedRejectsSamplingAndResetReentryThenRecovers() {
        LoopClock clock = new ManualLoopClock().clock();
        AtomicReference<Source<String>> memoizedRef = new AtomicReference<>();
        int[] mode = {1};

        Source<String> raw = Source.of(sampleClock -> {
            if (mode[0] == 1) {
                return memoizedRef.get().get(sampleClock);
            }
            if (mode[0] == 2) {
                memoizedRef.get().reset();
            }
            return "ready";
        });
        Source<String> memoized = raw.memoized();
        memoizedRef.set(memoized);

        RuntimeException recursiveSample =
                captureFailure(() -> memoized.get(clock));
        assertTrue(recursiveSample instanceof IllegalStateException);
        assertTrue(recursiveSample.getMessage().contains("sampling cycle"));

        mode[0] = 2;
        RuntimeException resetDuringSample =
                captureFailure(() -> memoized.get(clock));
        assertTrue(resetDuringSample instanceof IllegalStateException);
        assertTrue(resetDuringSample.getMessage().contains("while sampling"));

        mode[0] = 0;
        assertEquals("ready", memoized.get(clock));
        assertEquals("ready", memoized.get(clock));
    }

    @Test
    public void failedOrRecursiveChildResetKeepsThePublishedCache() {
        LoopClock clock = new ManualLoopClock().clock();
        AtomicReference<Source<Object>> memoizedRef = new AtomicReference<>();
        Object first = new Object();
        Object second = new Object();
        Object[] rawValue = {first};
        int[] resetMode = {1};
        int[] samples = {0};

        Source<Object> raw = new Source<Object>() {
            @Override
            public Object get(LoopClock sampleClock) {
                samples[0]++;
                return rawValue[0];
            }

            @Override
            public void reset() {
                if (resetMode[0] == 1) {
                    memoizedRef.get().get(clock);
                } else if (resetMode[0] == 2) {
                    memoizedRef.get().reset();
                }
            }
        };
        Source<Object> memoized = raw.memoized();
        memoizedRef.set(memoized);

        assertSame(first, memoized.get(clock));
        rawValue[0] = second;

        RuntimeException samplingDuringReset = captureFailure(memoized::reset);
        assertTrue(samplingDuringReset instanceof IllegalStateException);
        assertTrue(samplingDuringReset.getMessage().contains("reset is in progress"));
        assertSame(first, memoized.get(clock));

        resetMode[0] = 2;
        RuntimeException recursiveReset = captureFailure(memoized::reset);
        assertTrue(recursiveReset instanceof IllegalStateException);
        assertTrue(recursiveReset.getMessage().contains("recursively"));
        assertSame(first, memoized.get(clock));
        assertEquals(1, samples[0]);

        resetMode[0] = 0;
        memoized.reset();
        assertSame(second, memoized.get(clock));
        assertEquals(2, samples[0]);
    }

    @Test
    public void accumulatePublishesOnlyAfterReducerSuccess() {
        ManualLoopClock time = new ManualLoopClock();
        int[] sample = {2};
        boolean[] failReducer = {false};
        RuntimeException expected = new IllegalArgumentException("reducer failed");
        List<Integer> priorStates = new ArrayList<>();

        Source<Integer> accumulated = Source.of(clock -> sample[0]).accumulate(
                (prior, current) -> {
                    priorStates.add(prior);
                    if (failReducer[0]) {
                        throw expected;
                    }
                    return prior + current;
                },
                0
        );

        assertEquals(Integer.valueOf(2), accumulated.get(time.clock()));

        time.nextCycle(0.02);
        sample[0] = 3;
        failReducer[0] = true;
        assertSame(expected, captureFailure(() -> accumulated.get(time.clock())));

        failReducer[0] = false;
        assertEquals(Integer.valueOf(5), accumulated.get(time.clock()));
        assertEquals(Integer.valueOf(5), accumulated.get(time.clock()));
        assertEquals(Arrays.asList(0, 2, 2), priorStates);
    }

    @Test
    public void accumulateUntilDoesNotPublishAResetBeforeTheWholeSampleSucceeds() {
        ManualLoopClock time = new ManualLoopClock();
        boolean[] resetNow = {false};
        int[] sample = {2};
        boolean[] failSample = {false};
        RuntimeException expected = new IllegalStateException("sample failed");

        Source<Integer> raw = Source.of(clock -> {
            if (failSample[0]) {
                throw expected;
            }
            return sample[0];
        });
        Source<Integer> accumulated = raw.accumulateUntil(
                BooleanSource.of(() -> resetNow[0]),
                Integer::sum,
                0
        );

        assertEquals(Integer.valueOf(2), accumulated.get(time.clock()));

        time.nextCycle(0.02);
        resetNow[0] = true;
        sample[0] = 3;
        failSample[0] = true;
        assertSame(expected, captureFailure(() -> accumulated.get(time.clock())));

        resetNow[0] = false;
        failSample[0] = false;
        assertEquals(Integer.valueOf(5), accumulated.get(time.clock()));

        time.nextCycle(0.02);
        resetNow[0] = true;
        sample[0] = 4;
        assertEquals(Integer.valueOf(4), accumulated.get(time.clock()));
    }

    @Test
    public void accumulateUntilResetsChildrenInOrderAndClearsLocalStateOnlyAfterSuccess() {
        LoopClock clock = new ManualLoopClock().clock();
        List<String> events = new ArrayList<>();
        int[] sample = {2};
        boolean[] resetFails = {true};
        RuntimeException expected = new IllegalStateException("reset signal failed");
        int[] sampleCount = {0};

        Source<Integer> raw = new Source<Integer>() {
            @Override
            public Integer get(LoopClock ignored) {
                sampleCount[0]++;
                return sample[0];
            }

            @Override
            public void reset() {
                events.add("source");
            }
        };
        BooleanSource reset = new BooleanSource() {
            @Override
            public boolean getAsBoolean(LoopClock ignored) {
                return false;
            }

            @Override
            public void reset() {
                events.add("reset");
                if (resetFails[0]) {
                    throw expected;
                }
            }
        };
        Source<Integer> accumulated = raw.accumulateUntil(reset, Integer::sum, 0);

        assertEquals(Integer.valueOf(2), accumulated.get(clock));
        sample[0] = 7;
        assertSame(expected, captureFailure(accumulated::reset));
        assertEquals(Arrays.asList("source", "reset"), events);
        assertEquals(Integer.valueOf(2), accumulated.get(clock));
        assertEquals(1, sampleCount[0]);

        events.clear();
        resetFails[0] = false;
        accumulated.reset();
        assertEquals(Arrays.asList("source", "reset"), events);
        assertEquals(Integer.valueOf(7), accumulated.get(clock));
        assertEquals(2, sampleCount[0]);
    }

    @Test
    public void holdLastValidRetriesPredicateFailureWithoutChangingHistory() {
        ManualLoopClock time = new ManualLoopClock();
        String[] raw = {"valid"};
        boolean[] predicateFails = {false};
        RuntimeException expected = new IllegalStateException("predicate failed");
        Source<String> held = Source.of(clock -> raw[0]).holdLastValid(
                value -> {
                    if (predicateFails[0]) {
                        throw expected;
                    }
                    return !"invalid".equals(value);
                },
                1.0,
                "fallback"
        );

        assertEquals("valid", held.get(time.clock()));
        time.nextCycle(0.25);
        raw[0] = "invalid";
        predicateFails[0] = true;
        assertSame(expected, captureFailure(() -> held.get(time.clock())));

        predicateFails[0] = false;
        assertEquals("valid", held.get(time.clock()));
        assertEquals("valid", held.get(time.clock()));
    }

    private static RuntimeException captureFailure(Runnable action) {
        try {
            action.run();
            fail("Expected operation to fail");
            return null;
        } catch (RuntimeException failure) {
            return failure;
        }
    }
}
