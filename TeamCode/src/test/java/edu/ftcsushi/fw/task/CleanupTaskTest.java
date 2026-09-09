package edu.ftcsushi.fw.task;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/**
 * Software lifecycle evidence for the public terminal-cleanup factory.
 *
 * <p>The wrapper, clock, runners, and parent compositions remain real. Scripted children expose
 * callback order and failures; these tests do not model actuator motion or physical recovery.</p>
 */
public final class CleanupTaskTest {

    private static final List<TaskOutcome> TERMINAL_OUTCOMES = Arrays.asList(
            TaskOutcome.SUCCESS, TaskOutcome.TIMEOUT, TaskOutcome.CANCELLED, TaskOutcome.UNKNOWN);

    @Test
    public void constructionRejectsNullWithoutRunningEitherArgument() {
        ProbeTask child = new ProbeTask();
        AtomicInteger cleanup = new AtomicInteger();

        RuntimeException missingChild = expectRuntime(
                () -> Tasks.withCleanup(null, cleanup::incrementAndGet));
        RuntimeException missingCleanup = expectRuntime(() -> Tasks.withCleanup(child, null));

        assertContains(missingChild, "child");
        assertContains(missingCleanup, "cleanup");
        assertEquals(0, child.callbackCount());
        assertEquals(0, cleanup.get());
    }

    @Test
    public void constructionAndCachedQueriesDoNotObserveOrStartTheChild() {
        ProbeTask child = new ProbeTask();
        AtomicInteger cleanup = new AtomicInteger();
        Task decorated = Tasks.withCleanup(child, cleanup::incrementAndGet);

        assertFalse(decorated.isComplete());
        assertEquals(TaskOutcome.NOT_DONE, decorated.getOutcome());
        decorated.debugDump(new RecordingDebugSink(), "task");

        assertEquals(0, child.callbackCount());
        assertEquals(0, child.debugCount);
        assertEquals(0, cleanup.get());
    }

    @Test
    public void prestartCancelAndInvalidUpdateDoNotConsumeTheFutureCleanupScope() {
        ManualLoopClock clock = new ManualLoopClock();
        ProbeTask child = new ProbeTask();
        AtomicInteger cleanup = new AtomicInteger();
        Task decorated = Tasks.withCleanup(child, cleanup::incrementAndGet);

        decorated.cancel();
        decorated.cancel();
        assertContains(expectRuntime(() -> decorated.update(clock.clock())), "before start");
        assertFalse(decorated.isComplete());
        assertEquals(TaskOutcome.NOT_DONE, decorated.getOutcome());
        assertEquals(0, child.callbackCount());
        assertEquals(0, cleanup.get());

        decorated.start(clock.clock());
        decorated.cancel();
        assertEquals(1, child.startCount);
        assertEquals(1, child.cancelCount);
        assertEquals(1, cleanup.get());
    }

    @Test
    public void invalidStartClockConsumesStartButDoesNotArmCleanup() {
        ProbeTask child = new ProbeTask();
        AtomicInteger cleanup = new AtomicInteger();
        Task decorated = Tasks.withCleanup(child, cleanup::incrementAndGet);

        assertContains(expectRuntime(() -> decorated.start(null)), "LoopClock");
        decorated.cancel();
        assertContains(expectRuntime(() -> decorated.start(new LoopClock())), "single-use");
        assertEquals(0, child.callbackCount());
        assertEquals(0, cleanup.get());
    }

    @Test
    public void invalidActiveUpdateClockFailsAndCleansTheAlreadyArmedScope() {
        ManualLoopClock clock = new ManualLoopClock();
        ProbeTask child = new ProbeTask();
        AtomicInteger cleanup = new AtomicInteger();
        Task decorated = Tasks.withCleanup(child, cleanup::incrementAndGet);
        decorated.start(clock.clock());

        RuntimeException failure = expectRuntime(() -> decorated.update(null));

        assertContains(failure, "LoopClock");
        assertEquals(1, child.cancelCount);
        assertEquals(1, cleanup.get());
        assertRetainedFailure(decorated, failure, clock);
    }

    @Test
    public void everyNaturalOutcomeIsPreservedAfterImmediateOrUpdatedCompletion() {
        for (TaskOutcome outcome : TERMINAL_OUTCOMES) {
            for (boolean duringStart : new boolean[]{true, false}) {
                ManualLoopClock clock = new ManualLoopClock();
                ProbeTask child = new ProbeTask();
                AtomicInteger cleanup = new AtomicInteger();
                if (duringStart) {
                    child.startHook = () -> child.finish(outcome);
                } else {
                    child.updateHook = () -> child.finish(outcome);
                }
                Task decorated = Tasks.withCleanup(child, cleanup::incrementAndGet);

                decorated.start(clock.clock());
                if (!duringStart) {
                    assertEquals(0, cleanup.get());
                    assertFalse(decorated.isComplete());
                    decorated.update(clock.clock());
                }

                assertTrue(decorated.isComplete());
                assertEquals(outcome, decorated.getOutcome());
                assertEquals(1, child.outcomeCount);
                assertEquals(1, cleanup.get());
                assertEquals(0, child.cancelCount);
                int observedCallbacks = child.callbackCount();
                decorated.cancel();
                decorated.update(clock.clock());
                decorated.update(clock.nextCycle(1.0));
                assertEquals(outcome, decorated.getOutcome());
                assertEquals(observedCallbacks, child.callbackCount());
                assertEquals(1, cleanup.get());
            }
        }
    }

    @Test
    public void terminalOutcomeIsCapturedBeforeCleanupAndNeverResampled() {
        ManualLoopClock clock = new ManualLoopClock();
        ProbeTask child = new ProbeTask();
        child.startHook = () -> child.finish(TaskOutcome.TIMEOUT);
        Task decorated = Tasks.withCleanup(child, () -> child.finish(TaskOutcome.SUCCESS));

        decorated.start(clock.clock());
        assertEquals(TaskOutcome.TIMEOUT, decorated.getOutcome());
        assertEquals(TaskOutcome.TIMEOUT, decorated.getOutcome());
        assertEquals(1, child.outcomeCount);
    }

    @Test
    public void completionBetweenCyclesIsObservedBeforeAnotherChildUpdate() {
        ManualLoopClock clock = new ManualLoopClock();
        ProbeTask child = new ProbeTask();
        AtomicInteger cleanup = new AtomicInteger();
        Task decorated = Tasks.withCleanup(child, cleanup::incrementAndGet);
        decorated.start(clock.clock());
        decorated.update(clock.clock());

        child.finish(TaskOutcome.SUCCESS);
        decorated.update(clock.nextCycle(0.1));

        assertEquals(1, child.updateCount);
        assertEquals(1, cleanup.get());
        assertEquals(TaskOutcome.SUCCESS, decorated.getOutcome());
    }

    @Test
    public void alreadyCompleteNoopStillArmsAndCleansOnlyWhenWrapperStarts() {
        AtomicInteger cleanup = new AtomicInteger();
        Task decorated = Tasks.withCleanup(Tasks.noop(), cleanup::incrementAndGet);

        assertFalse(decorated.isComplete());
        decorated.cancel();
        assertEquals(0, cleanup.get());
        decorated.start(new ManualLoopClock().clock());

        assertTrue(decorated.isComplete());
        assertEquals(TaskOutcome.SUCCESS, decorated.getOutcome());
        assertEquals(1, cleanup.get());
    }

    @Test
    public void cancellationClaimsTerminalityBeforeChildThenCleanupAndRunsEachOnce() {
        ManualLoopClock clock = new ManualLoopClock();
        ProbeTask child = new ProbeTask();
        child.cancelOutcome = TaskOutcome.TIMEOUT;
        List<String> order = new ArrayList<>();
        AtomicReference<Task> owner = new AtomicReference<>();
        child.cancelHook = () -> {
            assertTrue(owner.get().isComplete());
            order.add("child cancel");
            owner.get().cancel();
        };
        Task decorated = Tasks.withCleanup(child, () -> {
            assertTrue(owner.get().isComplete());
            order.add("cleanup");
            owner.get().cancel();
        });
        owner.set(decorated);
        decorated.start(clock.clock());

        decorated.cancel();
        decorated.cancel();
        decorated.update(clock.clock());

        assertEquals(Arrays.asList("child cancel", "cleanup"), order);
        assertEquals(1, child.cancelCount);
        assertEquals(TaskOutcome.CANCELLED, decorated.getOutcome());
    }

    @Test
    public void reentrantCancellationFromEachChildCallbackCannotReopenTheWrapper() {
        for (CallbackPoint point : CallbackPoint.values()) {
            ManualLoopClock clock = new ManualLoopClock();
            ProbeTask child = new ProbeTask();
            AtomicInteger cleanup = new AtomicInteger();
            Task decorated = Tasks.withCleanup(child, cleanup::incrementAndGet);
            if (point == CallbackPoint.OUTCOME) {
                child.startHook = () -> child.finish(TaskOutcome.SUCCESS);
            }
            child.hook(point, decorated::cancel);

            decorated.start(clock.clock());
            if (!decorated.isComplete()) {
                decorated.update(clock.clock());
            }

            assertTrue(point.name(), decorated.isComplete());
            assertEquals(point.name(), TaskOutcome.CANCELLED, decorated.getOutcome());
            assertEquals(point.name(), 1, cleanup.get());
            int expectedCancellationCalls = point == CallbackPoint.OUTCOME ? 0 : 1;
            assertEquals(point.name(), expectedCancellationCalls, child.cancelCount);
            decorated.cancel();
            assertEquals(expectedCancellationCalls, child.cancelCount);
        }
    }

    @Test
    public void reentrantCancelDuringNaturalCleanupDoesNotRelabelTheChosenOutcome() {
        for (TaskOutcome outcome : TERMINAL_OUTCOMES) {
            ManualLoopClock clock = new ManualLoopClock();
            ProbeTask child = new ProbeTask();
            child.startHook = () -> child.finish(outcome);
            AtomicInteger cleanup = new AtomicInteger();
            AtomicReference<Task> owner = new AtomicReference<>();
            Task decorated = Tasks.withCleanup(child, () -> {
                cleanup.incrementAndGet();
                assertTrue(owner.get().isComplete());
                owner.get().cancel();
            });
            owner.set(decorated);

            decorated.start(clock.clock());

            assertEquals(outcome, decorated.getOutcome());
            assertEquals(1, cleanup.get());
            assertEquals(0, child.cancelCount);
        }
    }

    @Test
    public void firstUpdateCanShareStartCycleButRepeatedUpdatesDoNotAdvanceTheChild() {
        ManualLoopClock clock = new ManualLoopClock();
        ProbeTask child = new ProbeTask();
        Task decorated = Tasks.withCleanup(child, () -> { });

        decorated.start(clock.clock());
        decorated.update(clock.clock());
        decorated.update(clock.clock());
        assertEquals(1, child.updateCount);
        decorated.update(clock.nextCycle(0.1));
        decorated.update(clock.clock());
        assertEquals(2, child.updateCount);
        assertEquals(TaskOutcome.NOT_DONE, decorated.getOutcome());
    }

    @Test
    public void activeReentrantUpdatesAreNoOpsFromStartUpdateAndObservationCallbacks() {
        for (CallbackPoint point : CallbackPoint.values()) {
            ManualLoopClock clock = new ManualLoopClock();
            ProbeTask child = new ProbeTask();
            AtomicInteger cleanup = new AtomicInteger();
            Task decorated = Tasks.withCleanup(child, cleanup::incrementAndGet);
            child.hook(point, () -> decorated.update(clock.clock()));

            decorated.start(clock.clock());
            decorated.update(clock.clock());
            assertEquals(point.name(), 1, child.startCount);
            assertEquals(point.name(), 1, child.updateCount);
            child.finish(TaskOutcome.SUCCESS);
            decorated.update(clock.nextCycle(0.1));

            assertEquals(point.name(), TaskOutcome.SUCCESS, decorated.getOutcome());
            assertEquals(point.name(), 1, child.updateCount);
            assertEquals(point.name(), 1, cleanup.get());
        }
    }

    @Test
    public void reentrantStartIsRejectedWithoutPoisoningOrRepeatingTheActiveAttempt() {
        ManualLoopClock clock = new ManualLoopClock();
        ProbeTask child = new ProbeTask();
        AtomicInteger cleanup = new AtomicInteger();
        Task decorated = Tasks.withCleanup(child, cleanup::incrementAndGet);
        child.startHook = () -> assertContains(
                expectRuntime(() -> decorated.start(clock.clock())), "single-use");

        decorated.start(clock.clock());
        assertFalse(decorated.isComplete());
        assertEquals(1, child.startCount);
        assertEquals(0, child.cancelCount);
        assertEquals(0, cleanup.get());
        child.finish(TaskOutcome.SUCCESS);
        decorated.update(clock.clock());

        assertEquals(TaskOutcome.SUCCESS, decorated.getOutcome());
        assertEquals(1, cleanup.get());
        assertContains(expectRuntime(() -> decorated.start(clock.clock())), "single-use");
        assertEquals(TaskOutcome.SUCCESS, decorated.getOutcome());
        assertEquals(1, child.startCount);
        assertEquals(1, cleanup.get());
    }

    @Test
    public void repeatedStartDuringCleanupIsRejectedWithoutReplacingTheCapturedOutcome() {
        ManualLoopClock clock = new ManualLoopClock();
        AtomicReference<Task> owner = new AtomicReference<>();
        AtomicInteger cleanup = new AtomicInteger();
        Task decorated = Tasks.withCleanup(Tasks.noop(), () -> {
            cleanup.incrementAndGet();
            assertContains(expectRuntime(() -> owner.get().start(clock.clock())), "single-use");
        });
        owner.set(decorated);

        decorated.start(clock.clock());

        assertEquals(TaskOutcome.SUCCESS, decorated.getOutcome());
        assertEquals(1, cleanup.get());
    }

    @Test
    public void childTimerKeepsItsOwnStartBoundaryAfterALargePrestartGap() {
        ManualLoopClock clock = new ManualLoopClock();
        AtomicInteger cleanup = new AtomicInteger();
        Task decorated = Tasks.withCleanup(Tasks.waitForSeconds(0.25), cleanup::incrementAndGet);
        clock.nextCycle(10.0);

        decorated.start(clock.clock());
        decorated.update(clock.clock());
        assertFalse(decorated.isComplete());
        assertEquals(0, cleanup.get());
        decorated.update(clock.nextCycle(0.125));
        assertFalse(decorated.isComplete());
        decorated.update(clock.nextCycle(0.125));

        assertEquals(TaskOutcome.SUCCESS, decorated.getOutcome());
        assertEquals(1, cleanup.get());
        assertEquals(10.25, clock.clock().nowSec(), 0.0);
    }

    @Test
    public void successfulImmediateCleanupPrecedesSameCallbackSequenceContinuation() {
        List<String> order = new ArrayList<>();
        Task graph = Tasks.sequence(
                Tasks.withCleanup(Tasks.runOnce(() -> order.add("body")),
                        () -> order.add("cleanup")),
                Tasks.runOnce(() -> order.add("next")));

        graph.start(new ManualLoopClock().clock());

        assertEquals(Arrays.asList("body", "cleanup", "next"), order);
        assertEquals(TaskOutcome.SUCCESS, graph.getOutcome());
    }

    @Test
    public void childLifecycleFailuresAreRetainedAndCleanupIsNotRetried() {
        for (CallbackPoint point : CallbackPoint.values()) {
            ManualLoopClock clock = new ManualLoopClock();
            ProbeTask child = new ProbeTask();
            AtomicInteger cleanup = new AtomicInteger();
            RuntimeException failure = new RuntimeException(point + " failed");
            Task decorated = Tasks.withCleanup(child, cleanup::incrementAndGet);
            if (point == CallbackPoint.OUTCOME) {
                child.startHook = () -> child.finish(TaskOutcome.SUCCESS);
            }
            child.hook(point, () -> { throw failure; });

            RuntimeException thrown = expectRuntime(() -> {
                decorated.start(clock.clock());
                decorated.update(clock.clock());
            });

            assertSame(point.name(), failure, thrown);
            assertTrue(point.name(), decorated.isComplete());
            assertEquals(point.name(), 1, cleanup.get());
            if (point != CallbackPoint.OUTCOME) {
                assertEquals(point.name(), 1, child.cancelCount);
            }
            int callbacksAfterFailure = child.callbackCount();
            assertRetainedFailure(decorated, failure, clock);
            assertEquals(point.name(), callbacksAfterFailure, child.callbackCount());
            assertEquals(point.name(), 1, cleanup.get());
        }
    }

    @Test
    public void failedStartCannotBeRetriedEvenAfterTheChildStopsThrowing() {
        ManualLoopClock clock = new ManualLoopClock();
        ProbeTask child = new ProbeTask();
        RuntimeException failure = new RuntimeException("partially acquired start");
        child.startHook = () -> { throw failure; };
        AtomicInteger cleanup = new AtomicInteger();
        Task decorated = Tasks.withCleanup(child, cleanup::incrementAndGet);
        assertSame(failure, expectRuntime(() -> decorated.start(clock.clock())));
        child.startHook = null;

        assertContains(expectRuntime(() -> decorated.start(clock.clock())), "single-use");

        assertSame(failure, expectRuntime(decorated::getOutcome));
        assertEquals(1, child.startCount);
        assertEquals(1, child.cancelCount);
        assertEquals(1, cleanup.get());
    }

    @Test
    public void malformedNaturalOutcomeIsNotConvertedIntoSuccessOrUnknown() {
        for (TaskOutcome malformed : Arrays.asList(null, TaskOutcome.NOT_DONE)) {
            ManualLoopClock clock = new ManualLoopClock();
            ProbeTask child = new ProbeTask();
            child.startHook = () -> child.finish(malformed);
            AtomicInteger cleanup = new AtomicInteger();
            Task decorated = Tasks.withCleanup(child, cleanup::incrementAndGet);

            RuntimeException failure = expectRuntime(() -> decorated.start(clock.clock()));

            assertTrue(failure instanceof IllegalStateException);
            assertContains(failure, String.valueOf(malformed));
            assertEquals(1, cleanup.get());
            assertRetainedFailure(decorated, failure, clock);
        }
    }

    @Test
    public void childCancelRuntimeFailureStillRunsCleanupAndStaysExceptional() {
        ManualLoopClock clock = new ManualLoopClock();
        ProbeTask child = new ProbeTask();
        List<String> order = new ArrayList<>();
        RuntimeException failure = new RuntimeException("child cancel failed");
        child.cancelHook = () -> {
            order.add("child cancel");
            throw failure;
        };
        Task decorated = Tasks.withCleanup(child, () -> order.add("cleanup"));
        decorated.start(clock.clock());

        assertSame(failure, expectRuntime(decorated::cancel));

        assertEquals(Arrays.asList("child cancel", "cleanup"), order);
        assertRetainedFailure(decorated, failure, clock);
        assertEquals(1, child.cancelCount);
    }

    @Test
    public void nonterminalCancellationIsAnErrorButDoesNotSkipCleanup() {
        ManualLoopClock clock = new ManualLoopClock();
        ProbeTask child = new ProbeTask();
        child.completeOnCancel = false;
        AtomicInteger cleanup = new AtomicInteger();
        Task decorated = Tasks.withCleanup(child, cleanup::incrementAndGet);
        decorated.start(clock.clock());

        RuntimeException failure = expectRuntime(decorated::cancel);

        assertTrue(failure instanceof IllegalStateException);
        assertContains(failure, "terminal");
        assertEquals(1, child.cancelCount);
        assertEquals(1, cleanup.get());
        assertRetainedFailure(decorated, failure, clock);
        assertEquals(1, child.cancelCount);
    }

    @Test
    public void malformedCancelledOutcomeIsAnErrorButDoesNotSkipCleanup() {
        for (TaskOutcome malformed : Arrays.asList(null, TaskOutcome.NOT_DONE)) {
            ManualLoopClock clock = new ManualLoopClock();
            ProbeTask child = new ProbeTask();
            child.cancelOutcome = malformed;
            AtomicInteger cleanup = new AtomicInteger();
            Task decorated = Tasks.withCleanup(child, cleanup::incrementAndGet);
            decorated.start(clock.clock());

            RuntimeException failure = expectRuntime(decorated::cancel);

            assertTrue(failure instanceof IllegalStateException);
            assertContains(failure, String.valueOf(malformed));
            assertEquals(1, cleanup.get());
            assertRetainedFailure(decorated, failure, clock);
        }
    }

    @Test
    public void naturalCleanupFailureNeverPublishesTheChildsSuccess() {
        ManualLoopClock clock = new ManualLoopClock();
        RuntimeException failure = new RuntimeException("cleanup failed");
        AtomicInteger cleanup = new AtomicInteger();
        Task decorated = Tasks.withCleanup(Tasks.noop(), () -> {
            cleanup.incrementAndGet();
            throw failure;
        });

        assertSame(failure, expectRuntime(() -> decorated.start(clock.clock())));

        assertRetainedFailure(decorated, failure, clock);
        assertEquals(1, cleanup.get());
    }

    @Test
    public void lifecycleFailureKeepsLaterCancelAndCleanupFailuresSuppressedInOrder() {
        ManualLoopClock clock = new ManualLoopClock();
        ProbeTask child = new ProbeTask();
        RuntimeException primary = new RuntimeException("update failed");
        RuntimeException cancelFailure = new RuntimeException("cancel failed");
        RuntimeException cleanupFailure = new RuntimeException("cleanup failed");
        child.updateHook = () -> { throw primary; };
        child.cancelHook = () -> { throw cancelFailure; };
        AtomicInteger cleanup = new AtomicInteger();
        Task decorated = Tasks.withCleanup(child, () -> {
            cleanup.incrementAndGet();
            throw cleanupFailure;
        });
        decorated.start(clock.clock());

        assertSame(primary, expectRuntime(() -> decorated.update(clock.clock())));

        assertEquals(2, primary.getSuppressed().length);
        assertSame(cancelFailure, primary.getSuppressed()[0]);
        assertSame(cleanupFailure, primary.getSuppressed()[1]);
        assertRetainedFailure(decorated, primary, clock);
        assertEquals(2, primary.getSuppressed().length);
        assertEquals(1, child.cancelCount);
        assertEquals(1, cleanup.get());
    }

    @Test
    public void oneExceptionReusedByEveryHookIsNeverSelfSuppressed() {
        ManualLoopClock clock = new ManualLoopClock();
        ProbeTask child = new ProbeTask();
        RuntimeException failure = new RuntimeException("same exception identity");
        child.updateHook = () -> { throw failure; };
        child.cancelHook = () -> { throw failure; };
        AtomicInteger cleanup = new AtomicInteger();
        Task decorated = Tasks.withCleanup(child, () -> {
            cleanup.incrementAndGet();
            throw failure;
        });
        decorated.start(clock.clock());

        assertSame(failure, expectRuntime(() -> decorated.update(clock.clock())));

        assertEquals(0, failure.getSuppressed().length);
        assertRetainedFailure(decorated, failure, clock);
        assertEquals(1, cleanup.get());
    }

    @Test
    public void swallowedOutcomeReadDuringCleanupStillPoisonsThePendingResult() {
        ManualLoopClock clock = new ManualLoopClock();
        AtomicReference<Task> owner = new AtomicReference<>();
        AtomicReference<RuntimeException> observed = new AtomicReference<>();
        AtomicInteger cleanup = new AtomicInteger();
        Task decorated = Tasks.withCleanup(Tasks.noop(), () -> {
            cleanup.incrementAndGet();
            assertTrue(owner.get().isComplete());
            observed.set(expectRuntime(owner.get()::getOutcome));
            assertSame(observed.get(), expectRuntime(() -> owner.get().update(clock.clock())));
            owner.get().cancel();
        });
        owner.set(decorated);

        RuntimeException failure = expectRuntime(() -> decorated.start(clock.clock()));

        assertSame(observed.get(), failure);
        assertTrue(failure instanceof IllegalStateException);
        assertContains(failure, "cleanup");
        assertRetainedFailure(decorated, failure, clock);
        assertEquals(1, cleanup.get());
    }

    @Test
    public void cleanupFailurePreventsBothSequencePoliciesAndClearsRunnerBacklog() {
        for (boolean onCompletion : new boolean[]{false, true}) {
            for (boolean duringStart : new boolean[]{false, true}) {
                ManualLoopClock clock = new ManualLoopClock();
                ProbeTask child = new ProbeTask();
                if (duringStart) {
                    child.startHook = () -> child.finish(TaskOutcome.SUCCESS);
                } else {
                    child.updateHook = () -> child.finish(TaskOutcome.SUCCESS);
                }
                RuntimeException failure = new RuntimeException("cleanup failed");
                Task decorated = Tasks.withCleanup(child, () -> { throw failure; });
                AtomicInteger continuation = new AtomicInteger();
                AtomicInteger backlog = new AtomicInteger();
                Task next = Tasks.runOnce(continuation::incrementAndGet);
                Task graph = onCompletion
                        ? Tasks.sequenceOnCompletion(decorated, next)
                        : Tasks.sequence(decorated, next);
                TaskRunner runner = new TaskRunner();
                runner.enqueue(graph);
                runner.enqueue(Tasks.runOnce(backlog::incrementAndGet));

                assertSame(failure, expectRuntime(() -> runner.update(clock.clock())));
                runner.update(clock.nextCycle(0.1));
                graph.update(clock.clock());

                assertTrue(runner.isIdle());
                assertEquals(0, continuation.get());
                assertEquals(0, backlog.get());
                assertSame(failure, expectRuntime(decorated::getOutcome));
            }
        }
    }

    @Test
    public void reentrantParentUpdateCannotStartContinuationBeforeCleanupHasSettled() {
        for (boolean onCompletion : new boolean[]{false, true}) {
            ManualLoopClock clock = new ManualLoopClock();
            AtomicReference<Task> parent = new AtomicReference<>();
            AtomicReference<RuntimeException> observed = new AtomicReference<>();
            AtomicInteger continuation = new AtomicInteger();
            AtomicInteger cleanup = new AtomicInteger();
            Task decorated = Tasks.withCleanup(Tasks.noop(), () -> {
                cleanup.incrementAndGet();
                observed.set(expectRuntime(() -> parent.get().update(clock.clock())));
                assertEquals(0, continuation.get());
            });
            Task next = Tasks.runOnce(continuation::incrementAndGet);
            Task graph = onCompletion
                    ? Tasks.sequenceOnCompletion(decorated, next)
                    : Tasks.sequence(decorated, next);
            parent.set(graph);

            RuntimeException failure = expectRuntime(() -> graph.start(clock.clock()));

            assertSame(observed.get(), failure);
            assertSame(failure, expectRuntime(decorated::getOutcome));
            graph.update(clock.nextCycle(0.1));
            assertEquals(0, continuation.get());
            assertEquals(1, cleanup.get());
        }
    }

    @Test
    public void nestedCleanupScopesFinishInsideOutExactlyOnce() {
        for (boolean cancelled : new boolean[]{false, true}) {
            ManualLoopClock clock = new ManualLoopClock();
            ProbeTask child = new ProbeTask();
            List<String> order = new ArrayList<>();
            Task inner = Tasks.withCleanup(child, () -> order.add("inner"));
            Task outer = Tasks.withCleanup(inner, () -> order.add("outer"));
            outer.start(clock.clock());

            if (cancelled) {
                outer.cancel();
            } else {
                child.finish(TaskOutcome.TIMEOUT);
                outer.update(clock.clock());
            }
            outer.cancel();

            assertEquals(Arrays.asList("inner", "outer"), order);
            assertEquals(cancelled ? TaskOutcome.CANCELLED : TaskOutcome.TIMEOUT,
                    outer.getOutcome());
            assertEquals(cancelled ? 1 : 0, child.cancelCount);
        }
    }

    @Test
    public void nestedCleanupFailureStillRunsTheOuterCleanupWithoutRepeatingInnerWork() {
        ManualLoopClock clock = new ManualLoopClock();
        RuntimeException primary = new RuntimeException("inner cleanup failed");
        RuntimeException secondary = new RuntimeException("outer cleanup failed");
        List<String> order = new ArrayList<>();
        Task inner = Tasks.withCleanup(Tasks.noop(), () -> {
            order.add("inner");
            throw primary;
        });
        Task outer = Tasks.withCleanup(inner, () -> {
            order.add("outer");
            throw secondary;
        });

        assertSame(primary, expectRuntime(() -> outer.start(clock.clock())));

        assertEquals(Arrays.asList("inner", "outer"), order);
        assertEquals(1, primary.getSuppressed().length);
        assertSame(secondary, primary.getSuppressed()[0]);
        assertRetainedFailure(outer, primary, clock);
        assertEquals(Arrays.asList("inner", "outer"), order);
    }

    @Test
    public void discardedQueuedScopeNeverAcquiresOrCleansAnything() {
        ManualLoopClock clock = new ManualLoopClock();
        ProbeTask activeChild = new ProbeTask();
        ProbeTask pendingChild = new ProbeTask();
        AtomicInteger activeCleanup = new AtomicInteger();
        AtomicInteger pendingCleanup = new AtomicInteger();
        Task active = Tasks.withCleanup(activeChild, activeCleanup::incrementAndGet);
        Task pending = Tasks.withCleanup(pendingChild, pendingCleanup::incrementAndGet);
        TaskRunner runner = new TaskRunner();
        runner.enqueue(active);
        runner.enqueue(pending);
        runner.update(clock.clock());

        runner.cancelAndClear();
        runner.update(clock.nextCycle(0.1));

        assertTrue(runner.isIdle());
        assertEquals(1, activeCleanup.get());
        assertEquals(1, activeChild.cancelCount);
        assertEquals(0, pendingChild.callbackCount());
        assertEquals(0, pendingCleanup.get());
        assertFalse(pending.isComplete());
        assertEquals(TaskOutcome.NOT_DONE, pending.getOutcome());
    }

    @Test
    public void freshRepeatedScopesEachReceiveTheirOwnCleanup() {
        ManualLoopClock clock = new ManualLoopClock();
        List<Task> scopes = new ArrayList<>();
        AtomicInteger body = new AtomicInteger();
        AtomicInteger cleanup = new AtomicInteger();
        Task repeated = Tasks.repeatWhileSuccessful("three cleanup scopes", 3,
                BooleanSource.constant(true), () -> {
                    Task scope = Tasks.withCleanup(Tasks.runOnce(body::incrementAndGet),
                            cleanup::incrementAndGet);
                    scopes.add(scope);
                    return scope;
                });

        repeated.start(clock.clock());
        repeated.update(clock.clock());
        repeated.update(clock.nextCycle(0.1));
        repeated.update(clock.nextCycle(0.1));

        assertEquals(TaskOutcome.SUCCESS, repeated.getOutcome());
        assertEquals(3, body.get());
        assertEquals(3, cleanup.get());
        assertEquals(3, scopes.size());
        assertNotSame(scopes.get(0), scopes.get(1));
        assertNotSame(scopes.get(1), scopes.get(2));
    }

    @Test
    public void aFreshWrapperDoesNotMakeAnAlreadyUsedChildReusable() {
        ManualLoopClock clock = new ManualLoopClock();
        AtomicInteger body = new AtomicInteger();
        AtomicInteger firstCleanup = new AtomicInteger();
        AtomicInteger secondCleanup = new AtomicInteger();
        Task child = Tasks.runOnce(body::incrementAndGet);
        Task first = Tasks.withCleanup(child, firstCleanup::incrementAndGet);
        Task second = Tasks.withCleanup(child, secondCleanup::incrementAndGet);
        first.start(clock.clock());

        RuntimeException failure = expectRuntime(() -> second.start(clock.clock()));

        assertContains(failure, "single-use");
        assertEquals(1, body.get());
        assertEquals(1, firstCleanup.get());
        assertEquals(1, secondCleanup.get());
        assertEquals(TaskOutcome.SUCCESS, first.getOutcome());
        assertRetainedFailure(second, failure, clock);
    }

    @Test
    public void diagnosticsAfterFailureReadOnlyCachedWrapperState() {
        ManualLoopClock clock = new ManualLoopClock();
        ProbeTask child = new ProbeTask();
        RuntimeException failure = new RuntimeException("update failed");
        child.updateHook = () -> { throw failure; };
        AtomicInteger cleanup = new AtomicInteger();
        Task decorated = Tasks.withCleanup(child, cleanup::incrementAndGet);
        decorated.start(clock.clock());
        assertSame(failure, expectRuntime(() -> decorated.update(clock.clock())));
        int callbacks = child.callbackCount();
        child.debugHook = () -> { throw new AssertionError("must not inspect child diagnostics"); };
        RecordingDebugSink debug = new RecordingDebugSink();

        decorated.debugDump(debug, "failed");
        decorated.debugDump(null, null);

        assertFalse(debug.values.isEmpty());
        assertEquals(callbacks, child.callbackCount());
        assertEquals(0, child.debugCount);
        assertEquals(1, cleanup.get());
        assertSame(failure, expectRuntime(decorated::getOutcome));
    }

    @Test
    public void normalDiagnosticsDelegateButPendingCleanupDoesNotInspectTheChild() {
        ManualLoopClock clock = new ManualLoopClock();
        ProbeTask child = new ProbeTask();
        AtomicReference<Task> owner = new AtomicReference<>();
        Task decorated = Tasks.withCleanup(child, () -> {
            assertTrue(owner.get().isComplete());
            int callbacks = child.callbackCount();
            owner.get().debugDump(new RecordingDebugSink(), "pending");
            assertEquals(callbacks, child.callbackCount());
            assertEquals(1, child.debugCount);
        });
        owner.set(decorated);
        decorated.start(clock.clock());
        int activeCallbacks = child.callbackCount();

        decorated.debugDump(new RecordingDebugSink(), "active");

        assertEquals(1, child.debugCount);
        assertEquals(activeCallbacks, child.callbackCount());
        child.finish(TaskOutcome.TIMEOUT);
        decorated.update(clock.clock());
        int terminalCallbacks = child.callbackCount();
        decorated.debugDump(new RecordingDebugSink(), "terminal");
        assertEquals(2, child.debugCount);
        assertEquals(terminalCallbacks, child.callbackCount());
        assertEquals(TaskOutcome.TIMEOUT, decorated.getOutcome());
    }

    @Test
    public void childDiagnosticRuntimeFailureIsReportedWithoutPoisoningTaskOutcome() {
        ManualLoopClock clock = new ManualLoopClock();
        ProbeTask child = new ProbeTask();
        RuntimeException diagnosticFailure = new RuntimeException("optional diagnostic failed");
        child.debugHook = () -> { throw diagnosticFailure; };
        AtomicInteger cleanup = new AtomicInteger();
        Task decorated = Tasks.withCleanup(child, cleanup::incrementAndGet);
        decorated.start(clock.clock());
        int callbacks = child.callbackCount();
        RecordingDebugSink debug = new RecordingDebugSink();

        decorated.debugDump(debug, "active");

        assertTrue(debug.keys.contains("active.childDiagnosticsUnavailable"));
        assertTrue(debug.values.contains("RuntimeException: optional diagnostic failed"));
        assertEquals(callbacks, child.callbackCount());
        assertEquals(0, cleanup.get());
        assertFalse(decorated.isComplete());
        assertEquals(TaskOutcome.NOT_DONE, decorated.getOutcome());
        child.finish(TaskOutcome.SUCCESS);
        decorated.update(clock.clock());
        assertEquals(TaskOutcome.SUCCESS, decorated.getOutcome());
        assertEquals(1, cleanup.get());
    }

    @Test
    public void childDiagnosticErrorIsNotCaughtAsOptionalRuntimeFailure() {
        ManualLoopClock clock = new ManualLoopClock();
        ProbeTask child = new ProbeTask();
        AssertionError error = new AssertionError("diagnostic Error");
        child.debugHook = () -> { throw error; };
        AtomicInteger cleanup = new AtomicInteger();
        Task decorated = Tasks.withCleanup(child, cleanup::incrementAndGet);
        decorated.start(clock.clock());

        assertSame(error, expectError(
                () -> decorated.debugDump(new RecordingDebugSink(), "active")));

        assertEquals(0, cleanup.get());
        assertEquals(TaskOutcome.NOT_DONE, decorated.getOutcome());
        decorated.cancel();
        assertEquals(1, cleanup.get());
    }

    @Test
    public void javaErrorsFromChildLifecyclePassThroughWithoutRuntimeRecovery() {
        for (CallbackPoint point : CallbackPoint.values()) {
            ManualLoopClock clock = new ManualLoopClock();
            ProbeTask child = new ProbeTask();
            AssertionError error = new AssertionError(point + " is not a RuntimeException");
            AtomicInteger cleanup = new AtomicInteger();
            if (point == CallbackPoint.OUTCOME) {
                child.startHook = () -> child.finish(TaskOutcome.SUCCESS);
            }
            child.hook(point, () -> { throw error; });
            Task decorated = Tasks.withCleanup(child, cleanup::incrementAndGet);

            Error observed = expectError(() -> {
                decorated.start(clock.clock());
                decorated.update(clock.clock());
            });

            assertSame(point.name(), error, observed);
            assertEquals(point.name(), 0, child.cancelCount);
            assertEquals(point.name(), 0, cleanup.get());
        }
    }

    @Test
    public void javaErrorsFromCancelAndCleanupPassThroughUntranslated() {
        ManualLoopClock clock = new ManualLoopClock();
        ProbeTask child = new ProbeTask();
        AssertionError cancelError = new AssertionError("cancel Error");
        child.cancelHook = () -> { throw cancelError; };
        Task cancelled = Tasks.withCleanup(child, () -> { });
        cancelled.start(clock.clock());
        assertSame(cancelError, expectError(cancelled::cancel));

        AssertionError cleanupError = new AssertionError("cleanup Error");
        AtomicInteger cleanup = new AtomicInteger();
        Task natural = Tasks.withCleanup(Tasks.noop(), () -> {
            cleanup.incrementAndGet();
            throw cleanupError;
        });
        assertSame(cleanupError, expectError(() -> natural.start(clock.clock())));
        assertEquals(1, cleanup.get());
    }

    /** Verify stable failure identity before cycle guards, without rearming terminal cleanup. */
    private static void assertRetainedFailure(Task task,
                                              RuntimeException failure,
                                              ManualLoopClock clock) {
        assertTrue(task.isComplete());
        assertSame(failure, expectRuntime(task::getOutcome));
        assertSame(failure, expectRuntime(() -> task.update(clock.clock())));
        task.cancel();
        task.cancel();
        assertSame(failure, expectRuntime(() -> task.update(clock.nextCycle(0.1))));
        assertSame(failure, expectRuntime(task::getOutcome));
    }

    /** Select one child lifecycle callback without adding a production test seam. */
    private enum CallbackPoint {
        START, UPDATE, COMPLETION, OUTCOME
    }

    /**
     * A cooperative child with explicit observation counters and independently scripted callbacks.
     * Deliberately malformed outcomes and cancellation are configured only in named negative tests.
     */
    private static final class ProbeTask implements Task {
        private int startCount;
        private int updateCount;
        private int cancelCount;
        private int completionCount;
        private int outcomeCount;
        private int debugCount;
        private boolean started;
        private boolean complete;
        private boolean completeOnCancel = true;
        private TaskOutcome outcome = TaskOutcome.NOT_DONE;
        private TaskOutcome cancelOutcome = TaskOutcome.CANCELLED;
        private Runnable startHook;
        private Runnable updateHook;
        private Runnable cancelHook;
        private Runnable completionHook;
        private Runnable outcomeHook;
        private Runnable debugHook;

        @Override
        public void start(LoopClock clock) {
            if (started) {
                throw new IllegalStateException("ProbeTask is single-use");
            }
            started = true;
            startCount++;
            run(startHook);
        }

        @Override
        public void update(LoopClock clock) {
            if (!started) {
                throw new IllegalStateException("ProbeTask update before start");
            }
            updateCount++;
            run(updateHook);
        }

        @Override
        public void cancel() {
            cancelCount++;
            if (!started || complete) {
                return;
            }
            if (completeOnCancel) {
                finish(cancelOutcome);
            }
            run(cancelHook);
        }

        @Override
        public boolean isComplete() {
            completionCount++;
            run(completionHook);
            return complete;
        }

        @Override
        public TaskOutcome getOutcome() {
            outcomeCount++;
            run(outcomeHook);
            return outcome;
        }

        @Override
        public void debugDump(DebugSink dbg, String prefix) {
            debugCount++;
            run(debugHook);
        }

        /** Publish an authored terminal result; null/NOT_DONE are intentional negative probes. */
        private void finish(TaskOutcome terminalOutcome) {
            complete = true;
            outcome = terminalOutcome;
        }

        /** Install one callback so the real wrapper encounters the selected lifecycle boundary. */
        private void hook(CallbackPoint point, Runnable action) {
            switch (point) {
                case START:
                    startHook = action;
                    break;
                case UPDATE:
                    updateHook = action;
                    break;
                case COMPLETION:
                    completionHook = action;
                    break;
                case OUTCOME:
                    outcomeHook = action;
                    break;
                default:
                    throw new AssertionError(point);
            }
        }

        /** Count child entry points; cached wrapper reads must leave this unchanged. */
        private int callbackCount() {
            return startCount + updateCount + cancelCount + completionCount + outcomeCount;
        }

        private static void run(Runnable callback) {
            if (callback != null) {
                callback.run();
            }
        }
    }

    /** A value-only sink that does not call into either the wrapper or its child. */
    private static final class RecordingDebugSink implements DebugSink {
        private final List<String> keys = new ArrayList<>();
        private final List<Object> values = new ArrayList<>();

        @Override
        public DebugSink addData(String key, Object value) {
            keys.add(key);
            values.add(value);
            return this;
        }

        @Override
        public DebugSink addLine(String text) {
            values.add(text);
            return this;
        }
    }

    private static RuntimeException expectRuntime(Runnable action) {
        try {
            action.run();
            fail("Expected RuntimeException");
            return null;
        } catch (RuntimeException failure) {
            return failure;
        }
    }

    private static Error expectError(Runnable action) {
        try {
            action.run();
        } catch (Error error) {
            return error;
        }
        throw new AssertionError("Expected the scripted Java Error");
    }

    private static void assertContains(RuntimeException failure, String text) {
        assertTrue(String.valueOf(failure.getMessage()),
                failure.getMessage() != null && failure.getMessage().contains(text));
    }
}
