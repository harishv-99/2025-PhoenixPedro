package edu.ftcsushi.fw.task;

import org.junit.Test;

import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/**
 * Maintainer evidence for the shared Task lifetime, independent of any one timed phase policy.
 *
 * <p>The base, shared clock, and parent compositions remain real. A small subclass supplies
 * observable synchronous hooks and test-owned resource handles. No test claims physical cleanup,
 * thread safety, vendor acquisition behavior, or recovery from Java Error.</p>
 */
public final class AbstractTaskTest {
    private static final List<TaskOutcome> TERMINAL_OUTCOMES = Arrays.asList(
            TaskOutcome.SUCCESS, TaskOutcome.TIMEOUT, TaskOutcome.CANCELLED, TaskOutcome.UNKNOWN);

    @Test
    public void baseIsOneAdvancedExtensionSurfaceWithFinalLifecycle() throws Exception {
        assertTrue(Modifier.isPublic(AbstractTask.class.getModifiers()));
        assertTrue(Modifier.isAbstract(AbstractTask.class.getModifiers()));
        assertEquals(1, AbstractTask.class.getDeclaredConstructors().length);
        assertTrue(Modifier.isProtected(AbstractTask.class.getDeclaredConstructor(String.class)
                .getModifiers()));
        for (String method : Arrays.asList("cancel", "isComplete", "getOutcome")) {
            assertTrue(method, Modifier.isFinal(AbstractTask.class.getMethod(method).getModifiers()));
        }
        for (String method : Arrays.asList("start", "update")) {
            assertTrue(method, Modifier.isFinal(AbstractTask.class.getMethod(method, LoopClock.class)
                    .getModifiers()));
        }
        assertTrue(Modifier.isFinal(AbstractTask.class.getMethod(
                "debugDump", DebugSink.class, String.class).getModifiers()));
        assertFalse(Modifier.isFinal(AbstractTask.class.getMethod("getDebugName").getModifiers()));
    }

    @Test
    public void constructionValidatesNameWithoutCallingHooks() {
        for (String name : Arrays.asList(null, "", "  ")) {
            assertContains(expectRuntime(() -> new Probe(name)), "name");
        }
        Probe task = new Probe();
        assertFalse(task.started());
        assertFalse(task.active());
        assertFalse(task.isComplete());
        assertEquals(TaskOutcome.NOT_DONE, task.getOutcome());
        assertEquals(0, task.effectCount());
    }

    @Test
    public void prestartCancelAndUpdateDoNotArmOrConsumeStart() {
        Probe task = new Probe();
        ManualLoopClock clock = new ManualLoopClock();
        task.cancel();
        task.cancel();
        assertContains(expectRuntime(() -> task.update(clock.clock())), "before start");
        assertEquals(0, task.effectCount());
        task.start(clock.clock());
        task.cancel();
        assertEquals(Arrays.asList("start", "beforeCancel", "cancel", "finish"), task.events);
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
    }

    @Test
    public void rejectedClockConsumesStartButDoesNotArmEndingHooks() {
        Probe task = new Probe();
        assertContains(expectRuntime(() -> task.start(null)), "LoopClock");
        task.cancel();
        assertContains(expectRuntime(() -> task.start(new LoopClock())), "single-use");
        assertFalse(task.started());
        assertFalse(task.isComplete());
        assertEquals(TaskOutcome.NOT_DONE, task.getOutcome());
        assertEquals(0, task.effectCount());
    }

    @Test
    public void lifetimeIsArmedBeforeStartHookAndSecondStartCannotRepeatEffects() {
        Probe task = new Probe();
        ManualLoopClock clock = new ManualLoopClock();
        task.startAction = suppliedClock -> {
            assertSame(clock.clock(), suppliedClock);
            assertTrue(task.started());
            assertTrue(task.active());
            assertContains(expectRuntime(() -> task.start(suppliedClock)), "single-use");
        };
        task.start(clock.clock());
        assertContains(expectRuntime(() -> task.start(clock.clock())), "single-use");
        assertFalse(task.failed());
        task.cancel();
        assertContains(expectRuntime(() -> task.start(clock.clock())), "single-use");
        assertEquals(1, task.starts);
        assertEquals(1, task.finishes);
    }

    @Test
    public void firstUpdateMayShareStartCycleButLaterUpdatesAreClaimedOnce() {
        Probe task = new Probe();
        ManualLoopClock clock = new ManualLoopClock();
        clock.nextCycle(4.0);
        task.startAction = c -> task.update(c);
        task.updateAction = c -> {
            assertSame(clock.clock(), c);
            task.update(c);
        };
        task.start(clock.clock());
        assertEquals(0, task.updates);
        task.update(clock.clock());
        task.update(clock.clock());
        assertEquals(1, task.updates);
        clock.nextCycle(0.1);
        task.update(clock.clock());
        assertEquals(2, task.updates);
        assertFalse(task.failed());
    }

    @Test
    public void activeOutcomeIsNotDoneAndDoesNotObserveDomainState() {
        Probe task = started();
        task.observeAction = () -> task.finishWith(TaskOutcome.SUCCESS);
        assertEquals(TaskOutcome.NOT_DONE, task.getOutcome());
        assertFalse(task.isComplete());
        assertEquals(0, task.observations);
        task.observeStatus();
        assertEquals(TaskOutcome.SUCCESS, task.getOutcome());
        assertEquals(1, task.observations);
    }

    @Test
    public void naturalOutcomesAreExactAndFinishOnceWithoutCancellation() {
        for (TaskOutcome outcome : TERMINAL_OUTCOMES) {
            for (boolean duringStart : Arrays.asList(false, true)) {
                Probe task = new Probe();
                ManualLoopClock clock = new ManualLoopClock();
                Consumer<LoopClock> finish = c -> task.finishWith(outcome);
                if (duringStart) {
                    task.startAction = finish;
                } else {
                    task.updateAction = finish;
                }
                task.start(clock.clock());
                task.update(clock.clock());
                task.cancel();
                clock.nextCycle(1.0);
                task.update(clock.clock());
                assertTrue(task.isComplete());
                assertFalse(task.active());
                assertEquals(outcome, task.getOutcome());
                assertEquals(0, task.cancels);
                assertEquals(0, task.failures);
                assertEquals(1, task.finishes);
            }
        }
    }

    @Test
    public void cancellationClaimsEndingBeforeAbortAndFinishCallbacks() {
        Probe task = started();
        task.cancelAction = () -> {
            assertTrue(task.isComplete());
            assertFalse(task.active());
            task.cancel();
            task.update(new LoopClock());
        };
        task.finishAction = () -> {
            assertTrue(task.isComplete());
            assertFalse(task.active());
            task.cancel();
        };
        task.cancel();
        task.cancel();
        assertEquals(Arrays.asList("start", "beforeCancel", "cancel", "finish"), task.events);
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
    }

    @Test
    public void beforeCancelCanPreserveAlreadyTerminalEvidenceWithoutAbortingIt() {
        for (TaskOutcome outcome : TERMINAL_OUTCOMES) {
            Probe task = started();
            task.beforeCancelAction = () -> {
                task.cancel();
                task.finishWith(outcome);
            };
            task.cancel();
            assertEquals(outcome, task.getOutcome());
            assertEquals(1, task.beforeCancels);
            assertEquals(0, task.cancels);
            assertEquals(1, task.finishes);
        }
    }

    @Test
    public void malformedNaturalOutcomesFailClosedInsteadOfPublishingAnEnum() {
        for (TaskOutcome malformed : Arrays.asList(null, TaskOutcome.NOT_DONE)) {
            Probe task = new Probe();
            ManualLoopClock clock = new ManualLoopClock();
            task.updateAction = c -> task.finishWith(malformed);
            task.start(clock.clock());
            RuntimeException failure = expectRuntime(() -> task.update(clock.clock()));
            assertContains(failure, "cannot finish");
            assertFailed(task, failure, clock.clock());
            assertEquals(1, task.cancels);
            assertEquals(1, task.finishes);
        }
    }

    @Test
    public void invalidUpdateClockIsAnArmedFailureNotAReusableAttempt() {
        Probe task = started();
        RuntimeException failure = expectRuntime(() -> task.update(null));
        assertContains(failure, "LoopClock");
        assertFailed(task, failure, new LoopClock());
        assertEquals(0, task.updates);
        assertEquals(1, task.cancels);
        assertEquals(1, task.finishes);
    }

    @Test
    public void startAndUpdateFailuresUseDefaultAbortAndRetainExactFailure() {
        for (boolean atStart : Arrays.asList(false, true)) {
            Probe task = new Probe();
            ManualLoopClock clock = new ManualLoopClock();
            RuntimeException original = new IllegalArgumentException("behavior failed");
            Consumer<LoopClock> throwing = c -> { throw original; };
            if (atStart) {
                task.startAction = throwing;
                assertSame(original, expectRuntime(() -> task.start(clock.clock())));
            } else {
                task.updateAction = throwing;
                task.start(clock.clock());
                assertSame(original, expectRuntime(() -> task.update(clock.clock())));
            }
            assertFailed(task, original, clock.clock());
            assertEquals(1, task.failures);
            assertEquals(1, task.cancels);
            assertEquals(1, task.finishes);
            assertContains(expectRuntime(() -> task.start(clock.clock())), "single-use");
        }
    }

    @Test
    public void retainedFailureWinsOverSameCycleAndReentrantUpdateGuards() {
        Probe task = new Probe();
        ManualLoopClock clock = new ManualLoopClock();
        RuntimeException original = new IllegalArgumentException("update");
        task.updateAction = c -> { throw original; };
        task.finishAction = () -> assertSame(original,
                expectRuntime(() -> task.update(clock.clock())));
        task.start(clock.clock());
        assertSame(original, expectRuntime(() -> task.update(clock.clock())));
        assertSame(original, expectRuntime(() -> task.update(clock.clock())));
        assertEquals(1, task.updates);
        assertEquals(1, task.finishes);
    }

    @Test
    public void specializedFailureHookReplacesDefaultCancelPolicy() {
        Probe task = new Probe();
        RuntimeException original = new IllegalStateException("vendor failure");
        task.failureAction = failure -> {
            assertSame(original, failure);
            assertTrue(task.isComplete());
            assertFalse(task.active());
            task.cancel();
        };
        task.startAction = c -> { throw original; };
        assertSame(original, expectRuntime(() -> task.start(new LoopClock())));
        assertEquals(1, task.failures);
        assertEquals(0, task.cancels);
        assertEquals(1, task.finishes);
    }

    @Test
    public void lifecycleFailureKeepsFlatAbortThenFinishSuppressionOrder() {
        Probe task = new Probe();
        RuntimeException original = new IllegalStateException("update");
        RuntimeException abort = new IllegalArgumentException("abort");
        RuntimeException finish = new IllegalArgumentException("finish");
        task.updateAction = c -> { throw original; };
        task.cancelAction = () -> { throw abort; };
        task.finishAction = () -> { throw finish; };
        task.start(new LoopClock());
        assertSame(original, expectRuntime(() -> task.update(new LoopClock())));
        assertEquals(Arrays.asList(abort, finish), Arrays.asList(original.getSuppressed()));
        assertEquals(0, abort.getSuppressed().length);
        assertEquals(1, task.cancels);
        assertEquals(1, task.finishes);
    }

    @Test
    public void cancelFailureIsVisibleToFinishAndDoesNotSkipIt() {
        Probe task = started();
        RuntimeException abort = new IllegalArgumentException("cancel");
        RuntimeException finish = new IllegalStateException("finish");
        task.cancelAction = () -> { throw abort; };
        task.finishAction = () -> {
            assertSame(abort, expectRuntime(task::getOutcome));
            throw finish;
        };
        assertSame(abort, expectRuntime(task::cancel));
        assertEquals(Arrays.asList(finish), Arrays.asList(abort.getSuppressed()));
        assertFailed(task, abort, new LoopClock());
        assertEquals(0, task.failures);
        assertEquals(1, task.cancels);
        assertEquals(1, task.finishes);
    }

    @Test
    public void beforeCancelFailureUsesFailurePolicyAndFinish() {
        Probe task = started();
        RuntimeException original = new IllegalStateException("status observation");
        task.beforeCancelAction = () -> { throw original; };
        assertSame(original, expectRuntime(task::cancel));
        assertEquals(Arrays.asList("start", "beforeCancel", "failure", "cancel", "finish"),
                task.events);
        assertFailed(task, original, new LoopClock());
    }

    @Test
    public void naturalFinishFailureNeverBecomesSuccessOrRunsAbortAfterTheFact() {
        Probe task = new Probe();
        RuntimeException original = new IllegalStateException("finish");
        task.startAction = c -> task.finishWith(TaskOutcome.SUCCESS);
        task.finishAction = () -> { throw original; };
        assertSame(original, expectRuntime(() -> task.start(new LoopClock())));
        assertFailed(task, original, new LoopClock());
        assertEquals(0, task.cancels);
        assertEquals(0, task.failures);
        assertEquals(1, task.finishes);
    }

    @Test
    public void theSamePrimaryFailureIsNeverSuppressedOntoItself() {
        Probe task = new Probe();
        RuntimeException original = new IllegalStateException("same object");
        task.startAction = c -> { throw original; };
        task.cancelAction = () -> { throw original; };
        task.finishAction = () -> { throw original; };
        assertSame(original, expectRuntime(() -> task.start(new LoopClock())));
        assertEquals(0, original.getSuppressed().length);
    }

    @Test
    public void outcomeReadDuringFinishPoisonsResultEvenWhenCallbackCatchesIt() {
        Probe task = new Probe();
        AtomicReference<RuntimeException> inspection = new AtomicReference<>();
        task.startAction = c -> task.finishWith(TaskOutcome.SUCCESS);
        task.finishAction = () -> {
            assertTrue(task.isComplete());
            inspection.set(expectRuntime(task::getOutcome));
        };
        RuntimeException thrown = expectRuntime(() -> task.start(new LoopClock()));
        assertSame(inspection.get(), thrown);
        assertContains(thrown, "outcome");
        assertFailed(task, thrown, new LoopClock());
        assertEquals(1, task.finishes);
    }

    @Test
    public void outcomeRemainsPendingAfterReentrantCancelUntilOuterStartReturns() {
        Probe task = new Probe();
        AtomicReference<RuntimeException> inspection = new AtomicReference<>();
        task.startAction = c -> {
            task.cancel();
            assertEquals(1, task.finishes);
            assertTrue(task.isComplete());
            inspection.set(expectRuntime(task::getOutcome));
        };
        assertSame(inspectionAfterStart(task, inspection), expectRuntime(task::getOutcome));
        assertEquals(1, task.cancels);
        assertEquals(1, task.finishes);
    }

    @Test
    public void outerUpdateCannotPublishOutcomeAfterNestedCancelEither() {
        Probe task = new Probe();
        AtomicReference<RuntimeException> inspection = new AtomicReference<>();
        task.updateAction = c -> {
            task.cancel();
            inspection.set(expectRuntime(task::typedOutcome));
        };
        task.start(new LoopClock());
        RuntimeException failure = expectRuntime(() -> task.update(new LoopClock()));
        assertSame(inspection.get(), failure);
        assertFailed(task, failure, new LoopClock());
    }

    @Test
    public void lateAcquisitionIsReleasedBeforePendingCancellationSettles() {
        Probe task = new Probe();
        AtomicInteger released = new AtomicInteger();
        task.startAction = c -> {
            Runnable handle = acquireAfterCancellation(task, released);
            assertTrue(task.isComplete());
            assertFalse(task.active());
            handle.run();
            task.events.add("lateRelease");
        };
        task.start(new LoopClock());
        assertEquals(1, released.get());
        assertEquals(Arrays.asList("start", "beforeCancel", "cancel", "finish", "lateRelease"),
                task.events);
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertEquals(1, task.finishes);
    }

    @Test
    public void lateReleaseFailureOverridesNoSettledResultAndDoesNotRepeatEndingHooks() {
        Probe task = new Probe();
        RuntimeException original = new IllegalStateException("late acquired handle release");
        task.startAction = c -> {
            task.cancel();
            task.events.add("lateRelease");
            throw original;
        };
        assertSame(original, expectRuntime(() -> task.start(new LoopClock())));
        assertFailed(task, original, new LoopClock());
        assertEquals(1, task.cancels);
        assertEquals(0, task.failures);
        assertEquals(1, task.finishes);
    }

    @Test
    public void lateAcquisitionCanRefineOnlyItsStillPendingStartCancellation() {
        for (TaskOutcome outcome : TERMINAL_OUTCOMES) {
            Probe task = new Probe();
            task.startAction = c -> {
                task.cancel();
                task.acquiredResult(outcome);
            };
            task.start(new LoopClock());
            assertEquals(outcome, task.getOutcome());
            assertEquals(1, task.cancels);
            assertEquals(1, task.finishes);
            assertContains(expectRuntime(() -> task.acquiredResult(TaskOutcome.UNKNOWN)), "guarded");
            assertEquals(outcome, task.getOutcome());
        }
    }

    @Test
    public void completionOutsideGuardedHooksRejectsWithoutEndingOrPoisoningTheAttempt() {
        for (boolean acquisition : Arrays.asList(false, true)) {
            Probe task = started();
            Runnable completeOutsideHook = () -> {
                if (acquisition) {
                    task.acquiredResult(TaskOutcome.SUCCESS);
                } else {
                    task.finishWith(TaskOutcome.SUCCESS);
                }
            };

            assertContains(expectRuntime(completeOutsideHook), "guarded");
            assertTrue(task.active());
            assertFalse(task.isComplete());
            assertFalse(task.failed());
            assertEquals(TaskOutcome.NOT_DONE, task.getOutcome());
            assertEquals(0, task.cancels);
            assertEquals(0, task.finishes);

            task.observeAction = () -> task.finishWith(TaskOutcome.TIMEOUT);
            task.observeStatus();
            assertEquals(TaskOutcome.TIMEOUT, task.getOutcome());
            assertContains(expectRuntime(completeOutsideHook), "guarded");
            assertEquals(TaskOutcome.TIMEOUT, task.getOutcome());
            assertEquals(1, task.finishes);
        }
    }

    @Test
    public void acquisitionRefinementCannotRewriteNaturalEndingOrUpdateCancellation() {
        for (boolean natural : Arrays.asList(false, true)) {
            Probe task = new Probe();
            if (natural) {
                task.startAction = c -> {
                    task.finishWith(TaskOutcome.SUCCESS);
                    task.acquiredResult(TaskOutcome.TIMEOUT);
                };
                RuntimeException failure = expectRuntime(() -> task.start(new LoopClock()));
                assertContains(failure, "pending");
                assertFailed(task, failure, new LoopClock());
            } else {
                task.updateAction = c -> {
                    task.cancel();
                    task.acquiredResult(TaskOutcome.SUCCESS);
                };
                task.start(new LoopClock());
                RuntimeException failure = expectRuntime(() -> task.update(new LoopClock()));
                assertContains(failure, "pending");
                assertFailed(task, failure, new LoopClock());
            }
            assertEquals(1, task.finishes);
        }
    }

    @Test
    public void guardedObservationDoesNotConsumeUpdatesOrCacheSameCycleStatus() {
        Probe task = new Probe();
        ManualLoopClock clock = new ManualLoopClock();
        task.observeStatus();
        assertEquals(0, task.observations);
        task.start(clock.clock());
        task.observeStatus();
        task.update(clock.clock());
        assertEquals(1, task.updates);
        task.observeAction = () -> task.finishWith(TaskOutcome.TIMEOUT);
        task.observeStatus();
        assertEquals(TaskOutcome.TIMEOUT, task.getOutcome());
        assertEquals(2, task.observations);
        task.observeStatus();
        assertEquals(2, task.observations);
    }

    @Test
    public void recursiveObservationAndUpdatesInsideObservationAreInert() {
        Probe task = new Probe();
        ManualLoopClock clock = new ManualLoopClock();
        task.startAction = c -> task.observeStatus();
        task.start(clock.clock());
        task.observeAction = () -> {
            task.observeStatus();
            task.update(clock.clock());
        };
        task.observeStatus();
        assertEquals(1, task.observations);
        assertEquals(0, task.updates);
        task.update(clock.clock());
        assertEquals(1, task.updates);
    }

    @Test
    public void observationFailureUsesTheSameRetainedFailureAndAbortPolicy() {
        Probe task = started();
        RuntimeException original = new IllegalStateException("execution status");
        task.observeAction = () -> { throw original; };
        assertSame(original, expectRuntime(task::observeStatus));
        assertSame(original, expectRuntime(task::observeStatus));
        assertFailed(task, original, new LoopClock());
        assertEquals(1, task.observations);
        assertEquals(1, task.cancels);
        assertEquals(1, task.finishes);
    }

    @Test
    public void bothSequencePoliciesWaitForFinishAndRefuseAThrowingEnding() {
        for (boolean completionPolicy : Arrays.asList(false, true)) {
            Probe task = new Probe();
            AtomicInteger later = new AtomicInteger();
            RuntimeException original = new IllegalStateException("finish");
            task.startAction = c -> task.finishWith(TaskOutcome.SUCCESS);
            task.finishAction = () -> { throw original; };
            Task sequence = sequence(completionPolicy, task, Tasks.runOnce(later::incrementAndGet));
            assertSame(original, expectRuntime(() -> sequence.start(new LoopClock())));
            assertEquals(0, later.get());
            assertEquals(1, task.finishes);
        }
    }

    @Test
    public void reentrantParentCannotAdvanceWhileCompletedChildCallbackIsStillRunning() {
        for (boolean completionPolicy : Arrays.asList(false, true)) {
            Probe task = new Probe();
            ManualLoopClock clock = new ManualLoopClock();
            AtomicInteger later = new AtomicInteger();
            AtomicReference<Task> parent = new AtomicReference<>();
            AtomicReference<RuntimeException> inspection = new AtomicReference<>();
            task.updateAction = c -> {
                task.finishWith(TaskOutcome.SUCCESS);
                inspection.set(expectRuntime(() -> parent.get().update(c)));
            };
            parent.set(sequence(completionPolicy, task, Tasks.runOnce(later::incrementAndGet)));
            parent.get().start(clock.clock());
            RuntimeException thrown = expectRuntime(() -> parent.get().update(clock.clock()));
            assertSame(inspection.get(), thrown);
            assertEquals(0, later.get());
            assertEquals(1, task.finishes);
        }
    }

    @Test
    public void nestedCleanupRunsInsideOutBeforeAParentContinuation() {
        List<String> events = new ArrayList<>();
        Probe task = new Probe();
        task.startAction = c -> task.finishWith(TaskOutcome.SUCCESS);
        task.finishAction = () -> events.add("leafFinish");
        Task inner = Tasks.withCleanup(task, () -> events.add("innerCleanup"));
        Task outer = Tasks.withCleanup(inner, () -> events.add("outerCleanup"));
        Task sequence = Tasks.sequence(outer, Tasks.runOnce(() -> events.add("continuation")));
        sequence.start(new LoopClock());
        assertEquals(Arrays.asList("leafFinish", "innerCleanup", "outerCleanup", "continuation"),
                events);
        assertEquals(TaskOutcome.SUCCESS, sequence.getOutcome());
    }

    @Test
    public void cleanupWrapperCanBeCancelledFromRealTimedChildStart() {
        assertReentrantChildCancellation(true, true);
    }

    @Test
    public void cleanupWrapperCanBeCancelledFromRealTimedChildUpdate() {
        assertReentrantChildCancellation(true, false);
    }

    @Test
    public void timeoutWrapperCanBeCancelledFromRealTimedChildStart() {
        assertReentrantChildCancellation(false, true);
    }

    @Test
    public void timeoutWrapperCanBeCancelledFromRealTimedChildUpdate() {
        assertReentrantChildCancellation(false, false);
    }

    @Test
    public void queuedDiscardBeforeStartHasNoEndingAndARepeatNeedsFreshInstances() {
        Probe discarded = new Probe();
        TaskRunner runner = new TaskRunner();
        runner.enqueue(discarded);
        runner.cancelAndClear();
        assertEquals(0, discarded.effectCount());
        assertEquals(TaskOutcome.NOT_DONE, discarded.getOutcome());

        Probe first = new Probe();
        Probe second = new Probe();
        first.start(new LoopClock());
        first.cancel();
        second.start(new LoopClock());
        second.cancel();
        assertEquals(1, first.finishes);
        assertEquals(1, second.finishes);
        assertContains(expectRuntime(() -> first.start(new LoopClock())), "single-use");
    }

    @Test
    public void cachedDiagnosticsRemainReadableDuringPendingEndingAndAfterFailure() {
        Probe task = new Probe();
        RuntimeException original = new IllegalStateException("update");
        RecordingDebugSink during = new RecordingDebugSink();
        task.startAction = c -> {
            task.cancel();
            task.debugDump(during, "task");
            throw original;
        };
        assertSame(original, expectRuntime(() -> task.start(new LoopClock())));
        assertEquals(false, during.rows.get("task.outcomeAvailable"));
        RecordingDebugSink after = new RecordingDebugSink();
        task.debugDump(after, "task");
        assertEquals(true, after.rows.get("task.hasLifecycleFailure"));
        assertEquals(false, after.rows.get("task.outcomeAvailable"));
        assertEquals(1, task.cancels);
        assertEquals(1, task.finishes);
    }

    @Test
    public void remainingTimeReadInsideFinishFailsClosedButDiagnosticsStayReadable() {
        AtomicReference<RunForSecondsTask> task = new AtomicReference<>();
        AtomicReference<RuntimeException> inspection = new AtomicReference<>();
        AtomicInteger starts = new AtomicInteger();
        AtomicInteger updates = new AtomicInteger();
        AtomicInteger finishes = new AtomicInteger();
        RecordingDebugSink during = new RecordingDebugSink();
        task.set(new RunForSecondsTask(0.0, starts::incrementAndGet,
                c -> updates.incrementAndGet(), () -> {
                    finishes.incrementAndGet();
                    task.get().debugDump(during, "timer");
                    inspection.set(expectRuntime(task.get()::getRemainingSec));
                }));

        RuntimeException failure = expectRuntime(() -> task.get().start(new LoopClock()));
        assertSame(inspection.get(), failure);
        assertContains(failure, "outcome");
        assertEquals(false, during.rows.get("timer.outcomeAvailable"));
        assertEquals(0.0, (double) during.rows.get("timer.remainingSec"), 0.0);
        assertSame(failure, expectRuntime(task.get()::getRemainingSec));
        assertSame(failure, expectRuntime(task.get()::getOutcome));
        RecordingDebugSink after = new RecordingDebugSink();
        task.get().debugDump(after, "timer");
        assertEquals(true, after.rows.get("timer.hasLifecycleFailure"));
        assertEquals(0.0, (double) after.rows.get("timer.remainingSec"), 0.0);
        assertEquals(1, starts.get());
        assertEquals(0, updates.get());
        assertEquals(1, finishes.get());
    }

    @Test
    public void remainingTimeStillPendingAfterFinishReturnsInsideOuterStartCallback() {
        AtomicReference<RunForSecondsTask> task = new AtomicReference<>();
        AtomicReference<RuntimeException> inspection = new AtomicReference<>();
        AtomicInteger finishes = new AtomicInteger();
        task.set(new RunForSecondsTask(1.0, () -> {
            task.get().cancel();
            assertEquals(1, finishes.get());
            inspection.set(expectRuntime(task.get()::getRemainingSec));
        }, null, finishes::incrementAndGet));

        RuntimeException failure = expectRuntime(() -> task.get().start(new LoopClock()));
        assertSame(inspection.get(), failure);
        assertSame(failure, expectRuntime(task.get()::getRemainingSec));
        assertEquals(1, finishes.get());
    }

    @Test
    public void remainingTimeRethrowsEachCallbackFailureWithoutDebugRetryingTheCallback() {
        for (String phase : Arrays.asList("start", "update", "finish")) {
            RuntimeException original = new IllegalStateException(phase);
            AtomicInteger starts = new AtomicInteger();
            AtomicInteger updates = new AtomicInteger();
            AtomicInteger finishes = new AtomicInteger();
            RunForSecondsTask task = new RunForSecondsTask(1.0,
                    () -> {
                        starts.incrementAndGet();
                        if (phase.equals("start")) {
                            throw original;
                        }
                    },
                    c -> {
                        updates.incrementAndGet();
                        if (phase.equals("update")) {
                            throw original;
                        }
                    },
                    () -> {
                        finishes.incrementAndGet();
                        if (phase.equals("finish")) {
                            throw original;
                        }
                    });
            ManualLoopClock clock = new ManualLoopClock();
            assertEquals(0.0, task.getRemainingSec(), 0.0);
            if (phase.equals("start")) {
                assertSame(original, expectRuntime(() -> task.start(clock.clock())));
            } else {
                task.start(clock.clock());
                assertEquals(1.0, task.getRemainingSec(), 0.0);
                if (phase.equals("finish")) {
                    clock.nextCycle(1.0);
                }
                assertSame(original, expectRuntime(() -> task.update(clock.clock())));
            }

            assertSame(original, expectRuntime(task::getRemainingSec));
            assertSame(original, expectRuntime(task::getOutcome));
            assertSame(original, expectRuntime(() -> task.update(clock.clock())));
            RecordingDebugSink sink = new RecordingDebugSink();
            task.debugDump(sink, "timer");
            task.debugDump(sink, "timer");
            assertEquals(true, sink.rows.get("timer.hasLifecycleFailure"));
            assertEquals(0.0, (double) sink.rows.get("timer.remainingSec"), 0.0);
            assertEquals(1, starts.get());
            assertEquals(phase.equals("start") ? 0 : 1, updates.get());
            assertEquals(1, finishes.get());
        }
    }

    @Test
    public void diagnosticRuntimeExceptionIsReportedButDoesNotPoisonLifecycle() {
        Probe task = started();
        task.debugAction = () -> { throw new IllegalStateException("optional view"); };
        RecordingDebugSink sink = new RecordingDebugSink();
        task.debugDump(sink, "task");
        assertTrue(sink.rows.containsKey("task.diagnosticsUnavailable"));
        assertFalse(task.failed());
        assertEquals(TaskOutcome.NOT_DONE, task.getOutcome());
        task.cancel();
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
    }

    @Test
    public void javaErrorsPropagateWithoutRuntimeRecoveryOrLaterEndingActions() {
        for (String phase : Arrays.asList("start", "update", "cancel", "finish", "observe", "debug")) {
            Probe task = new Probe();
            AssertionError original = new AssertionError("intentional " + phase);
            Runnable throwing = () -> { throw original; };
            if (phase.equals("start")) {
                task.startAction = c -> throwing.run();
                expectError(original, () -> task.start(new LoopClock()));
            } else {
                task.start(new LoopClock());
                switch (phase) {
                    case "update":
                        task.updateAction = c -> throwing.run();
                        expectError(original, () -> task.update(new LoopClock()));
                        break;
                    case "cancel":
                        task.cancelAction = throwing;
                        expectError(original, task::cancel);
                        break;
                    case "finish":
                        task.finishAction = throwing;
                        expectError(original, task::cancel);
                        break;
                    case "observe":
                        task.observeAction = throwing;
                        expectError(original, task::observeStatus);
                        break;
                    default:
                        task.debugAction = throwing;
                        expectError(original, () -> task.debugDump(new RecordingDebugSink(), "task"));
                }
            }
            assertFalse(phase, task.failed());
            assertEquals(phase, 0, task.failures);
            assertEquals(phase, phase.equals("cancel") || phase.equals("finish") ? 1 : 0,
                    task.cancels);
            assertEquals(phase, phase.equals("finish") ? 1 : 0, task.finishes);
        }
    }

    private static RuntimeException inspectionAfterStart(
            Probe task, AtomicReference<RuntimeException> inspection) {
        RuntimeException thrown = expectRuntime(() -> task.start(new LoopClock()));
        assertSame(inspection.get(), thrown);
        return thrown;
    }

    /**
     * A real timed child keeps its result pending while its own callback is still on the stack.
     * The enclosing wrapper may request cancellation immediately, but must wait for that child
     * callback to return before consuming its terminal outcome. Counter changes after cancel are
     * test observations of callback return, not another actuator command after cancellation.
     */
    private static void assertReentrantChildCancellation(boolean cleanupWrapper, boolean atStart) {
        ManualLoopClock clock = new ManualLoopClock();
        AtomicReference<Task> parent = new AtomicReference<>();
        AtomicInteger starts = new AtomicInteger();
        AtomicInteger updates = new AtomicInteger();
        AtomicInteger returnedFromCancel = new AtomicInteger();
        AtomicInteger childFinishes = new AtomicInteger();
        AtomicInteger parentCleanups = new AtomicInteger();
        Runnable cancelParent = () -> {
            parent.get().cancel();
            assertTrue(parent.get().isComplete());
            returnedFromCancel.incrementAndGet();
        };
        Task child = new RunForSecondsTask(1.0,
                () -> {
                    starts.incrementAndGet();
                    if (atStart) {
                        cancelParent.run();
                    }
                },
                c -> {
                    updates.incrementAndGet();
                    if (!atStart) {
                        cancelParent.run();
                    }
                },
                childFinishes::incrementAndGet);
        parent.set(cleanupWrapper
                ? Tasks.withCleanup(child, parentCleanups::incrementAndGet)
                : Tasks.withTimeout(child, 2.0));

        parent.get().start(clock.clock());
        if (!atStart) {
            assertFalse(parent.get().isComplete());
            parent.get().update(clock.clock());
        }

        assertEquals(1, returnedFromCancel.get());
        assertTrue(child.isComplete());
        assertEquals(TaskOutcome.CANCELLED, child.getOutcome());
        assertEquals(TaskOutcome.CANCELLED, parent.get().getOutcome());
        parent.get().cancel();
        parent.get().update(clock.clock());
        parent.get().update(clock.nextCycle(0.1));
        assertEquals(1, starts.get());
        assertEquals(atStart ? 0 : 1, updates.get());
        assertEquals(1, childFinishes.get());
        assertEquals(cleanupWrapper ? 1 : 0, parentCleanups.get());
    }

    private static Runnable acquireAfterCancellation(Probe task, AtomicInteger releases) {
        task.cancel();
        return releases::incrementAndGet;
    }

    private static Task sequence(boolean completionPolicy, Task first, Task second) {
        return completionPolicy ? Tasks.sequenceOnCompletion(first, second) : Tasks.sequence(first, second);
    }

    private static Probe started() {
        Probe task = new Probe();
        task.start(new LoopClock());
        return task;
    }

    private static void assertFailed(Probe task, RuntimeException expected, LoopClock clock) {
        assertTrue(task.isComplete());
        assertTrue(task.failed());
        assertFalse(task.active());
        assertSame(expected, expectRuntime(task::getOutcome));
        assertSame(expected, expectRuntime(task::typedOutcome));
        assertSame(expected, expectRuntime(() -> task.update(clock)));
        task.cancel();
    }

    private static RuntimeException expectRuntime(Runnable operation) {
        try {
            operation.run();
        } catch (RuntimeException failure) {
            return failure;
        }
        fail("Expected RuntimeException");
        return null;
    }

    private static void expectError(Error expected, Runnable operation) {
        Error observed = null;
        try {
            operation.run();
        } catch (Error failure) {
            observed = failure;
        }
        assertSame(expected, observed);
    }

    private static void assertContains(RuntimeException failure, String expected) {
        assertTrue(failure.getMessage(), failure.getMessage().contains(expected));
    }

    /** Domain hooks expose effects while the production base alone owns lifecycle state. */
    private static final class Probe extends AbstractTask {
        final List<String> events = new ArrayList<>();
        Consumer<LoopClock> startAction = c -> { };
        Consumer<LoopClock> updateAction = c -> { };
        Runnable beforeCancelAction = () -> { };
        Runnable cancelAction = () -> { };
        Consumer<RuntimeException> failureAction;
        Runnable finishAction = () -> { };
        Runnable observeAction = () -> { };
        Runnable debugAction = () -> { };
        int starts;
        int updates;
        int beforeCancels;
        int cancels;
        int failures;
        int finishes;
        int observations;

        Probe() { this("probe"); }
        Probe(String name) { super(name); }
        boolean started() { return isStarted(); }
        boolean active() { return isActive(); }
        boolean failed() { return hasFailure(); }
        void finishWith(TaskOutcome outcome) { complete(outcome); }
        void acquiredResult(TaskOutcome outcome) { completeAfterAcquisition(outcome); }
        TaskOutcome typedOutcome() { requireOutcomeAvailable(); return getOutcome(); }
        int effectCount() { return starts + updates + beforeCancels + cancels + failures + finishes; }

        void observeStatus() {
            observe(() -> {
                observations++;
                observeAction.run();
            });
        }

        @Override protected void onStart(LoopClock clock) {
            starts++;
            events.add("start");
            startAction.accept(clock);
        }

        @Override protected void onUpdate(LoopClock clock) {
            updates++;
            events.add("update");
            updateAction.accept(clock);
        }

        @Override protected void onBeforeCancel() {
            beforeCancels++;
            events.add("beforeCancel");
            beforeCancelAction.run();
        }

        @Override protected void onCancel() {
            cancels++;
            events.add("cancel");
            cancelAction.run();
        }

        @Override protected void onFailure(RuntimeException failure) {
            failures++;
            events.add("failure");
            if (failureAction == null) {
                super.onFailure(failure);
            } else {
                failureAction.accept(failure);
            }
        }

        @Override protected void onFinish() {
            finishes++;
            events.add("finish");
            finishAction.run();
        }

        @Override protected void debugState(DebugSink dbg, String prefix) {
            dbg.addData(prefix + ".domainStarts", starts);
            debugAction.run();
        }
    }

    private static final class RecordingDebugSink implements DebugSink {
        final Map<String, Object> rows = new LinkedHashMap<>();
        @Override public DebugSink addData(String caption, Object value) {
            rows.put(caption, value);
            return this;
        }
        @Override public DebugSink addLine(String text) {
            return this;
        }
    }
}
