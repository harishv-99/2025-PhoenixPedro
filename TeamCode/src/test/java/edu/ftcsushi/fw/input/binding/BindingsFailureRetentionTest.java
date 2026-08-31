package edu.ftcsushi.fw.input.binding;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Regression coverage for effectful per-cycle failure and reentry semantics. */
public final class BindingsFailureRetentionTest {

    @Test
    public void nullClockFailsWithoutClaimingAnUpdateAttempt() {
        Bindings bindings = new Bindings();
        ManualLoopClock manualClock = new ManualLoopClock();
        int[] calls = {0};
        bindings.whileHigh(BooleanSource.constant(true), () -> calls[0]++);

        try {
            bindings.update(null);
            fail("expected a null clock to fail");
        } catch (NullPointerException expected) {
            assertTrue(expected.getMessage().contains("clock is required"));
        }

        bindings.update(manualClock.clock());
        assertEquals(1, calls[0]);
    }

    @Test
    public void activationFailureIsRetainedWithoutResamplingAndRetriesNextCycle() {
        Bindings bindings = new Bindings();
        ManualLoopClock manualClock = new ManualLoopClock();
        RuntimeException failure = new RuntimeException("activation failure");
        boolean[] shouldFail = {true};
        int[] activationSamples = {0};
        int[] contextualCalls = {0};

        Bindings.ControlContext context = bindings.contextWhen(clock -> {
            activationSamples[0]++;
            if (shouldFail[0]) {
                throw failure;
            }
            return true;
        }, Bindings.ActivationPolicy.ACCEPT_CURRENT);
        context.whileHigh(BooleanSource.constant(true), () -> contextualCalls[0]++);

        assertSame(failure, captureRuntime(() -> bindings.update(manualClock.clock())));
        assertSame(failure, captureRuntime(() -> bindings.update(manualClock.clock())));
        assertEquals(1, activationSamples[0]);
        assertEquals(0, contextualCalls[0]);

        shouldFail[0] = false;
        bindings.update(manualClock.nextCycle(0.02));
        assertEquals(2, activationSamples[0]);
        assertEquals(0, contextualCalls[0]);
        bindings.update(manualClock.nextCycle(0.02));
        assertEquals(1, contextualCalls[0]);
    }

    @Test
    public void sourceFailureDoesNotReplayEarlierEffectsAndRetriesNextCycle() {
        Bindings bindings = new Bindings();
        ManualLoopClock manualClock = new ManualLoopClock();
        RuntimeException failure = new RuntimeException("source failure");
        boolean[] shouldFail = {true};
        int[] beforeCalls = {0};
        int[] sourceSamples = {0};
        int[] failedBindingCalls = {0};
        int[] afterCalls = {0};

        bindings.whileHigh(BooleanSource.constant(true), () -> beforeCalls[0]++);
        bindings.whileHigh(clock -> {
            sourceSamples[0]++;
            if (shouldFail[0]) {
                throw failure;
            }
            return true;
        }, () -> failedBindingCalls[0]++);
        bindings.whileHigh(BooleanSource.constant(true), () -> afterCalls[0]++);

        assertSame(failure, captureRuntime(() -> bindings.update(manualClock.clock())));
        assertSame(failure, captureRuntime(() -> bindings.update(manualClock.clock())));
        assertEquals(1, beforeCalls[0]);
        assertEquals(1, sourceSamples[0]);
        assertEquals(0, failedBindingCalls[0]);
        assertEquals(0, afterCalls[0]);

        shouldFail[0] = false;
        bindings.update(manualClock.nextCycle(0.02));
        assertEquals(2, beforeCalls[0]);
        assertEquals(2, sourceSamples[0]);
        assertEquals(1, failedBindingCalls[0]);
        assertEquals(1, afterCalls[0]);
    }

    @Test
    public void callbackFailureAtEachDeclarationPositionStopsAndRetainsTraversal() {
        for (int failurePosition = 0; failurePosition < 3; failurePosition++) {
            Bindings bindings = new Bindings();
            ManualLoopClock manualClock = new ManualLoopClock();
            final int failingPosition = failurePosition;
            RuntimeException failure = new RuntimeException(
                    "callback failure at " + failurePosition);
            int[] calls = {0, 0, 0};

            for (int callbackPosition = 0; callbackPosition < 3; callbackPosition++) {
                final int position = callbackPosition;
                bindings.whileHigh(BooleanSource.constant(true), () -> {
                    calls[position]++;
                    if (position == failingPosition) {
                        throw failure;
                    }
                });
            }

            assertSame(failure, captureRuntime(() -> bindings.update(manualClock.clock())));
            assertSame(failure, captureRuntime(() -> bindings.update(manualClock.clock())));
            for (int position = 0; position < 3; position++) {
                assertEquals(
                        "callback position " + position + " for failure " + failurePosition,
                        position <= failurePosition ? 1 : 0,
                        calls[position]);
            }
        }
    }

    @Test
    public void contextualCallbackFailureAtEachDeclarationPositionStopsAndRetainsTraversal() {
        for (int failurePosition = 0; failurePosition < 3; failurePosition++) {
            Bindings bindings = new Bindings();
            ManualLoopClock manualClock = new ManualLoopClock();
            final int failingPosition = failurePosition;
            RuntimeException failure = new RuntimeException(
                    "context callback failure at " + failurePosition);
            int[] activationSamples = {0};
            int[] calls = {0, 0, 0};

            Bindings.ControlContext context = bindings.contextWhen(clock -> {
                activationSamples[0]++;
                return true;
            }, Bindings.ActivationPolicy.ACCEPT_CURRENT);
            for (int callbackPosition = 0; callbackPosition < 3; callbackPosition++) {
                final int position = callbackPosition;
                context.whileHigh(BooleanSource.constant(true), () -> {
                    calls[position]++;
                    if (position == failingPosition) {
                        throw failure;
                    }
                });
            }

            bindings.update(manualClock.clock()); // Establish contextual baselines.
            manualClock.nextCycle(0.02);
            assertSame(failure, captureRuntime(() -> bindings.update(manualClock.clock())));
            assertSame(failure, captureRuntime(() -> bindings.update(manualClock.clock())));
            assertEquals("same-cycle retry must not resample activation", 2, activationSamples[0]);
            for (int position = 0; position < 3; position++) {
                assertEquals(
                        "context callback position " + position + " for failure "
                                + failurePosition,
                        position <= failurePosition ? 1 : 0,
                        calls[position]);
            }
        }
    }

    @Test
    public void failedRootAndContextRiseEventsStayConsumedUntilAFreshEdge() {
        assertFailedRiseIsConsumed(false);
        assertFailedRiseIsConsumed(true);
    }

    @Test
    public void failedToggleConsumerDoesNotRollBackItsCommittedState() {
        Bindings bindings = new Bindings();
        ManualLoopClock manualClock = new ManualLoopClock();
        RuntimeException failure = new RuntimeException("toggle consumer failure");
        boolean[] button = {false};
        List<Boolean> acceptedValues = new ArrayList<>();

        bindings.toggleOnRise(BooleanSource.of(() -> button[0]), enabled -> {
            acceptedValues.add(enabled);
            if (acceptedValues.size() == 1) {
                throw failure;
            }
        });

        bindings.update(manualClock.clock()); // Establish the false edge/toggle baseline.
        button[0] = true;
        assertSame(failure, captureRuntime(
                () -> bindings.update(manualClock.nextCycle(0.02))));
        assertSame(failure, captureRuntime(() -> bindings.update(manualClock.clock())));

        bindings.update(manualClock.nextCycle(0.02)); // Held true does not replay the toggle.
        button[0] = false;
        bindings.update(manualClock.nextCycle(0.02));
        button[0] = true;
        bindings.update(manualClock.nextCycle(0.02));

        assertEquals(Arrays.asList(true, false), acceptedValues);
    }

    @Test
    public void failedContextualScalarConsumerRetainsInitializationAndArmingState() {
        Bindings bindings = new Bindings();
        ManualLoopClock manualClock = new ManualLoopClock();
        RuntimeException failure = new RuntimeException("context scalar consumer failure");
        int[] sourceSamples = {0};
        List<Double> acceptedValues = new ArrayList<>();
        Bindings.ControlContext context = bindings.contextWhen(
                BooleanSource.constant(true), Bindings.ActivationPolicy.ACCEPT_CURRENT);

        context.copyEachCycle(clock -> {
            sourceSamples[0]++;
            return 0.75;
        }, value -> {
            acceptedValues.add(value);
            if (acceptedValues.size() == 1) {
                throw failure;
            }
        });

        assertSame(failure, captureRuntime(() -> bindings.update(manualClock.clock())));
        assertSame(failure, captureRuntime(() -> bindings.update(manualClock.clock())));
        bindings.update(manualClock.nextCycle(0.02));

        assertEquals(2, sourceSamples[0]);
        assertEquals(Arrays.asList(0.0, 0.75), acceptedValues);
    }

    @Test
    public void nextCycleStartsFromStateReachedBeforeCallbackFailure() {
        Bindings bindings = new Bindings();
        ManualLoopClock manualClock = new ManualLoopClock();
        RuntimeException failure = new RuntimeException("mirror consumer failure");
        int[] mirrorCalls = {0};
        int[] downstreamCalls = {0};

        bindings.mirrorOnChange(BooleanSource.constant(true), value -> {
            mirrorCalls[0]++;
            throw failure;
        });
        bindings.whileHigh(BooleanSource.constant(true), () -> downstreamCalls[0]++);

        assertSame(failure, captureRuntime(() -> bindings.update(manualClock.clock())));
        assertSame(failure, captureRuntime(() -> bindings.update(manualClock.clock())));
        assertEquals(1, mirrorCalls[0]);
        assertEquals(0, downstreamCalls[0]);

        bindings.update(manualClock.nextCycle(0.02));
        assertEquals("committed mirror state must not be rolled back", 1, mirrorCalls[0]);
        assertEquals(1, downstreamCalls[0]);
    }

    @Test
    public void caughtSameClockReentryFailsBeforeDedupeAndOuterUpdateSucceeds() {
        Bindings bindings = new Bindings();
        ManualLoopClock manualClock = new ManualLoopClock();
        LoopClock clock = manualClock.clock();
        RuntimeException[] caught = {null};
        int[] firstCalls = {0};
        int[] lastCalls = {0};

        bindings.whileHigh(BooleanSource.constant(true), () -> {
            firstCalls[0]++;
            caught[0] = captureRuntime(() -> bindings.update(clock));
        });
        bindings.whileHigh(BooleanSource.constant(true), () -> lastCalls[0]++);

        bindings.update(clock);
        assertTrue(caught[0] instanceof IllegalStateException);
        assertTrue(caught[0].getMessage().contains("cannot run recursively"));
        assertTrue(caught[0].getMessage().contains("owning loop heartbeat"));
        assertEquals(1, firstCalls[0]);
        assertEquals(1, lastCalls[0]);

        bindings.update(clock);
        assertEquals(1, firstCalls[0]);
        assertEquals(1, lastCalls[0]);
    }

    @Test
    public void escapingDifferentClockReentryIsRetainedByTheOuterCycle() {
        Bindings bindings = new Bindings();
        ManualLoopClock ownerClock = new ManualLoopClock();
        ManualLoopClock otherClock = new ManualLoopClock();
        otherClock.nextCycle(0.02);
        otherClock.nextCycle(0.02);
        int[] firstCalls = {0};
        int[] lastCalls = {0};

        bindings.whileHigh(BooleanSource.constant(true), () -> {
            firstCalls[0]++;
            bindings.update(otherClock.clock());
        });
        bindings.whileHigh(BooleanSource.constant(true), () -> lastCalls[0]++);

        RuntimeException firstFailure = captureRuntime(
                () -> bindings.update(ownerClock.clock()));
        assertTrue(firstFailure instanceof IllegalStateException);
        assertTrue(firstFailure.getMessage().contains("cannot run recursively"));
        assertSame(firstFailure, captureRuntime(() -> bindings.update(ownerClock.clock())));
        assertEquals(1, firstCalls[0]);
        assertEquals(0, lastCalls[0]);

        RuntimeException nextCycleFailure = captureRuntime(
                () -> bindings.update(ownerClock.nextCycle(0.02)));
        assertNotSame(firstFailure, nextCycleFailure);
        assertEquals(2, firstCalls[0]);
        assertEquals(0, lastCalls[0]);
    }

    @Test
    public void clearResetsFailureAndAllowsSameCycleRebuild() {
        Bindings bindings = new Bindings();
        ManualLoopClock manualClock = new ManualLoopClock();
        RuntimeException failure = new RuntimeException("old graph failure");
        int[] oldCalls = {0};
        int[] rebuiltCalls = {0};

        bindings.whileHigh(BooleanSource.constant(true), () -> {
            oldCalls[0]++;
            throw failure;
        });
        assertSame(failure, captureRuntime(() -> bindings.update(manualClock.clock())));

        bindings.clear();
        bindings.whileHigh(BooleanSource.constant(true), () -> rebuiltCalls[0]++);
        bindings.update(manualClock.clock());
        bindings.update(manualClock.clock());

        assertEquals(1, oldCalls[0]);
        assertEquals(1, rebuiltCalls[0]);
    }

    @Test
    public void errorIsOutsideRetentionButItsClaimedCycleIsNotReplayed() {
        Bindings bindings = new Bindings();
        ManualLoopClock manualClock = new ManualLoopClock();
        AssertionError error = new AssertionError("fatal failure");
        int[] firstCalls = {0};
        int[] lastCalls = {0};

        bindings.whileHigh(BooleanSource.constant(true), () -> {
            firstCalls[0]++;
            throw error;
        });
        bindings.whileHigh(BooleanSource.constant(true), () -> lastCalls[0]++);

        assertSame(error, captureError(() -> bindings.update(manualClock.clock())));
        bindings.update(manualClock.clock());
        assertEquals(1, firstCalls[0]);
        assertEquals(0, lastCalls[0]);

        assertSame(error, captureError(
                () -> bindings.update(manualClock.nextCycle(0.02))));
        assertEquals(2, firstCalls[0]);
        assertEquals(0, lastCalls[0]);
    }

    private static void assertFailedRiseIsConsumed(boolean contextual) {
        Bindings bindings = new Bindings();
        ManualLoopClock manualClock = new ManualLoopClock();
        RuntimeException failure = new RuntimeException(
                contextual ? "context rise failure" : "root rise failure");
        boolean[] button = {false};
        int[] calls = {0};
        CallbackBindings surface = contextual
                ? bindings.contextWhen(
                        BooleanSource.constant(true), Bindings.ActivationPolicy.ACCEPT_CURRENT)
                : bindings;

        surface.onRise(BooleanSource.of(() -> button[0]), () -> {
            calls[0]++;
            if (calls[0] == 1) {
                throw failure;
            }
        });

        bindings.update(manualClock.clock()); // Establish the false edge baseline.
        button[0] = true;
        assertSame(failure, captureRuntime(
                () -> bindings.update(manualClock.nextCycle(0.02))));
        assertSame(failure, captureRuntime(() -> bindings.update(manualClock.clock())));
        bindings.update(manualClock.nextCycle(0.02)); // Held true must not manufacture a retry.
        assertEquals(1, calls[0]);

        button[0] = false;
        bindings.update(manualClock.nextCycle(0.02));
        button[0] = true;
        bindings.update(manualClock.nextCycle(0.02));
        assertEquals(2, calls[0]);
    }

    private static RuntimeException captureRuntime(Runnable operation) {
        try {
            operation.run();
            fail("expected a RuntimeException");
            return null;
        } catch (RuntimeException expected) {
            return expected;
        }
    }

    private static Error captureError(Runnable operation) {
        try {
            operation.run();
            fail("expected an Error");
            return null;
        } catch (Error expected) {
            return expected;
        }
    }
}
