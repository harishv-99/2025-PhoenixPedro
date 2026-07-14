package edu.ftcphoenix.fw.task;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the deadline-owned parallel composition exposed by {@link Tasks}. */
public final class ParallelDeadlineTaskTest {

    @Test
    public void validatesInputsAliasesAndDefensiveCopy() {
        ProbeTask deadline = new ProbeTask("deadline");
        ProbeTask companion = new ProbeTask("companion");

        assertContains(
                expectIllegalArgument(() -> Tasks.parallelDeadline(null)),
                "deadline",
                "not be null");
        assertContains(
                expectIllegalArgument(
                        () -> Tasks.parallelDeadline(deadline, (Task[]) null)),
                "companions",
                "not be null");
        assertContains(
                expectIllegalArgument(() -> Tasks.parallelDeadline(deadline, companion, null)),
                "index 1",
                "not be null");
        assertContains(
                expectIllegalArgument(() -> Tasks.parallelDeadline(deadline, deadline)),
                "deadline",
                "index 0",
                "distinct Task instances");
        assertContains(
                expectIllegalArgument(
                        () -> Tasks.parallelDeadline(deadline, companion, companion)),
                "index 0",
                "index 1",
                "fresh task");

        ProbeTask retained = new ProbeTask("retained");
        ProbeTask replacement = new ProbeTask("replacement");
        Task[] supplied = {retained};
        Task group = Tasks.parallelDeadline(new ProbeTask("copiedDeadline"), supplied);
        supplied[0] = replacement;

        group.start(new ManualLoopClock().clock());

        assertEquals(1, retained.startCount);
        assertEquals(0, replacement.startCount);
        group.cancel();
    }

    @Test
    public void zeroCompanionsPreservesEveryValidDeadlineOutcome() {
        for (TaskOutcome terminal : Arrays.asList(
                TaskOutcome.SUCCESS,
                TaskOutcome.TIMEOUT,
                TaskOutcome.CANCELLED,
                TaskOutcome.UNKNOWN)) {
            ManualLoopClock manualClock = new ManualLoopClock();
            ProbeTask deadline = new ProbeTask("deadline-" + terminal);
            Task group = Tasks.parallelDeadline(deadline);

            group.start(manualClock.clock());
            assertFalse(group.isComplete());
            assertEquals(TaskOutcome.NOT_DONE, group.getOutcome());

            deadline.finish(terminal);
            group.update(manualClock.clock());

            assertTrue(group.isComplete());
            assertEquals(terminal, group.getOutcome());
            assertEquals(0, deadline.updateCount);
            assertEquals(0, deadline.cancelInvocations);
        }
    }

    @Test
    public void startsDeadlineFirstAndImmediateCompletionSkipsCompanions() {
        ManualLoopClock manualClock = new ManualLoopClock();
        List<String> events = new ArrayList<>();
        ProbeTask deadline = new ProbeTask("deadline", events);
        ProbeTask first = new ProbeTask("first", events);
        ProbeTask second = new ProbeTask("second", events);
        Task group = Tasks.parallelDeadline(deadline, first, second);

        group.start(manualClock.clock());

        assertEquals(
                Arrays.asList("deadline.start", "first.start", "second.start"),
                events);

        events.clear();
        group.update(manualClock.clock());

        assertEquals(
                Arrays.asList("deadline.update", "first.update", "second.update"),
                events);
        group.cancel();

        ProbeTask immediate = new ProbeTask("immediate");
        immediate.startHook = () -> immediate.finish(TaskOutcome.SUCCESS);
        ProbeTask skipped = new ProbeTask("skipped");
        Task immediateGroup = Tasks.parallelDeadline(immediate, skipped);

        immediateGroup.start(new ManualLoopClock().clock());

        assertTrue(immediateGroup.isComplete());
        assertEquals(TaskOutcome.SUCCESS, immediateGroup.getOutcome());
        assertEquals(0, skipped.startCount);
        assertEquals(0, skipped.updateCount);
        assertEquals(0, skipped.cancelInvocations);
    }

    @Test
    public void completionDuringCompanionStartCleansItAndSkipsLaterCompanions() {
        ManualLoopClock manualClock = new ManualLoopClock();
        ProbeTask deadline = new ProbeTask("deadline");
        ProbeTask completingStart = new ProbeTask("completingStart");
        ProbeTask skipped = new ProbeTask("skipped");
        completingStart.startHook = () -> deadline.finish(TaskOutcome.SUCCESS);
        Task group = Tasks.parallelDeadline(deadline, completingStart, skipped);

        group.start(manualClock.clock());

        assertTrue(group.isComplete());
        assertEquals(TaskOutcome.SUCCESS, group.getOutcome());
        assertEquals(0, deadline.cancelInvocations);
        assertEquals(1, completingStart.startCount);
        assertEquals(1, completingStart.cleanupTransitions);
        assertEquals(0, skipped.startCount);
        assertEquals(0, skipped.cancelInvocations);
    }

    @Test
    public void completionBeforeOrDuringDeadlineUpdatePreventsCompanionUpdate() {
        ManualLoopClock manualClock = new ManualLoopClock();
        ProbeTask completedBetweenCycles = new ProbeTask("betweenCycles");
        ProbeTask skipped = new ProbeTask("skipped");
        Task group = Tasks.parallelDeadline(completedBetweenCycles, skipped);
        group.start(manualClock.clock());

        completedBetweenCycles.finish(TaskOutcome.SUCCESS);
        group.update(manualClock.nextCycle(0.02));

        assertEquals(0, completedBetweenCycles.updateCount);
        assertEquals(0, skipped.updateCount);
        assertEquals(1, skipped.cleanupTransitions);

        ManualLoopClock sameCycleClock = new ManualLoopClock();
        List<String> events = new ArrayList<>();
        ProbeTask completesOnUpdate = new ProbeTask("deadline", events);
        completesOnUpdate.updateHook =
                () -> completesOnUpdate.finish(TaskOutcome.SUCCESS);
        ProbeTask neverUpdated = new ProbeTask("companion", events);
        Task sameCycleGroup = Tasks.parallelDeadline(completesOnUpdate, neverUpdated);
        TaskRunner runner = new TaskRunner();
        runner.enqueue(sameCycleGroup);

        runner.update(sameCycleClock.clock());

        assertTrue(sameCycleGroup.isComplete());
        assertSame(sameCycleGroup, runner.currentTaskOrNull());
        assertEquals(1, completesOnUpdate.updateCount);
        assertEquals(0, neverUpdated.updateCount);
        assertEquals(1, neverUpdated.cleanupTransitions);
        assertTrue(events.indexOf("companion.cancel")
                < events.indexOf("companion.update") || !events.contains("companion.update"));
        runner.update(sameCycleClock.nextCycle(0.02));
        assertTrue(runner.isIdle());
    }

    @Test
    public void earlyCompanionCompletionDoesNotEndOrOverrideGroup() {
        ManualLoopClock manualClock = new ManualLoopClock();
        ProbeTask deadline = new ProbeTask("deadline");
        ProbeTask early = new ProbeTask("early");
        early.startHook = () -> early.finish(TaskOutcome.TIMEOUT);
        ProbeTask active = new ProbeTask("active");
        Task group = Tasks.parallelDeadline(deadline, early, active);

        group.start(manualClock.clock());
        group.update(manualClock.clock());

        assertFalse(group.isComplete());
        assertEquals(TaskOutcome.NOT_DONE, group.getOutcome());
        assertEquals(0, early.updateCount);
        assertEquals(1, active.updateCount);

        deadline.finish(TaskOutcome.SUCCESS);
        group.update(manualClock.nextCycle(0.02));

        assertEquals(TaskOutcome.SUCCESS, group.getOutcome());
        assertEquals(1, early.cancelInvocations);
        assertEquals(0, early.cleanupTransitions);
        assertEquals(1, active.cleanupTransitions);
    }

    @Test
    public void reentrantCompletionAndCancellationStopLaterCallbacks() {
        ManualLoopClock manualClock = new ManualLoopClock();
        ProbeTask deadline = new ProbeTask("deadline");
        ProbeTask first = new ProbeTask("first");
        ProbeTask later = new ProbeTask("later");
        first.updateHook = () -> deadline.finish(TaskOutcome.SUCCESS);
        Task group = Tasks.parallelDeadline(deadline, first, later);
        group.start(manualClock.clock());

        group.update(manualClock.clock());

        assertEquals(TaskOutcome.SUCCESS, group.getOutcome());
        assertEquals(1, first.updateCount);
        assertEquals(0, later.updateCount);
        assertEquals(1, later.cleanupTransitions);

        AtomicReference<Task> startGroupRef = new AtomicReference<>();
        ProbeTask startDeadline = new ProbeTask("startDeadline");
        ProbeTask cancellingStart = new ProbeTask("cancellingStart");
        ProbeTask neverStarted = new ProbeTask("neverStarted");
        cancellingStart.startHook = () -> startGroupRef.get().cancel();
        Task startGroup = Tasks.parallelDeadline(
                startDeadline,
                cancellingStart,
                neverStarted);
        startGroupRef.set(startGroup);

        startGroup.start(manualClock.clock());

        assertEquals(TaskOutcome.CANCELLED, startGroup.getOutcome());
        assertEquals(0, neverStarted.startCount);
        assertEquals(0, neverStarted.cancelInvocations);

        AtomicReference<Task> outcomeGroupRef = new AtomicReference<>();
        ProbeTask outcomeDeadline = new ProbeTask("outcomeDeadline");
        ProbeTask outcomeCompanion = new ProbeTask("outcomeCompanion");
        Task outcomeGroup = Tasks.parallelDeadline(outcomeDeadline, outcomeCompanion);
        outcomeGroupRef.set(outcomeGroup);
        outcomeDeadline.outcomeHook = () -> outcomeGroupRef.get().cancel();
        outcomeGroup.start(manualClock.clock());
        outcomeDeadline.finish(TaskOutcome.SUCCESS);

        outcomeGroup.update(manualClock.clock());

        assertEquals(TaskOutcome.CANCELLED, outcomeGroup.getOutcome());
        assertEquals(1, outcomeCompanion.cleanupTransitions);
    }

    @Test
    public void directCancellationIsTerminalBestEffortAndIdempotent() {
        ManualLoopClock manualClock = new ManualLoopClock();
        ProbeTask deadline = new ProbeTask("deadline");
        ProbeTask first = new ProbeTask("first");
        ProbeTask last = new ProbeTask("last");
        RuntimeException firstFailure = new RuntimeException("deadline cleanup");
        RuntimeException secondFailure = new RuntimeException("first cleanup");
        deadline.cancelFailure = firstFailure;
        first.cancelFailure = secondFailure;
        AtomicReference<Task> groupRef = new AtomicReference<>();
        Runnable assertTerminal = () -> {
            assertTrue(groupRef.get().isComplete());
            assertEquals(TaskOutcome.CANCELLED, groupRef.get().getOutcome());
        };
        deadline.cancelHook = assertTerminal;
        first.cancelHook = assertTerminal;
        last.cancelHook = assertTerminal;
        Task group = Tasks.parallelDeadline(deadline, first, last);
        groupRef.set(group);
        group.start(manualClock.clock());

        RuntimeException thrown = expectRuntime(group::cancel);

        assertSame(firstFailure, thrown);
        assertEquals(1, thrown.getSuppressed().length);
        assertSame(secondFailure, thrown.getSuppressed()[0]);
        assertEquals(1, deadline.cancelInvocations);
        assertEquals(1, first.cancelInvocations);
        assertEquals(1, last.cancelInvocations);
        group.cancel();
        group.update(manualClock.clock());
        assertEquals(1, deadline.cancelInvocations);
        assertEquals(1, first.cancelInvocations);
        assertEquals(1, last.cancelInvocations);
    }

    @Test
    public void naturalCompletionCleansCompanionsOnlyAndPreservesOutcomeOnFailure() {
        ManualLoopClock manualClock = new ManualLoopClock();
        ProbeTask deadline = new ProbeTask("deadline");
        ProbeTask first = new ProbeTask("first");
        ProbeTask second = new ProbeTask("second");
        ProbeTask last = new ProbeTask("last");
        RuntimeException firstFailure = new RuntimeException("first cleanup");
        RuntimeException secondFailure = new RuntimeException("second cleanup");
        first.cancelFailure = firstFailure;
        second.cancelFailure = secondFailure;
        Task group = Tasks.parallelDeadline(deadline, first, second, last);
        group.start(manualClock.clock());
        deadline.finish(TaskOutcome.TIMEOUT);

        RuntimeException thrown = expectRuntime(() -> group.update(manualClock.clock()));

        assertSame(firstFailure, thrown);
        assertEquals(1, thrown.getSuppressed().length);
        assertSame(secondFailure, thrown.getSuppressed()[0]);
        assertTrue(group.isComplete());
        assertEquals(TaskOutcome.TIMEOUT, group.getOutcome());
        assertEquals(0, deadline.cancelInvocations);
        assertEquals(1, first.cancelInvocations);
        assertEquals(1, second.cancelInvocations);
        assertEquals(1, last.cancelInvocations);
        group.cancel();
        group.update(manualClock.clock());
        assertEquals(1, first.cancelInvocations);
        assertEquals(1, second.cancelInvocations);
        assertEquals(1, last.cancelInvocations);
    }

    @Test
    public void runnerFailStopCleansOnlyStartAttemptedChildren() {
        ManualLoopClock manualClock = new ManualLoopClock();
        ProbeTask deadline = new ProbeTask("deadline");
        ProbeTask first = new ProbeTask("first");
        ProbeTask failing = new ProbeTask("failing");
        ProbeTask skipped = new ProbeTask("skipped");
        RuntimeException startFailure = new RuntimeException("start failed");
        failing.startFailure = startFailure;
        Task group = Tasks.parallelDeadline(deadline, first, failing, skipped);
        TaskRunner runner = new TaskRunner();
        runner.enqueue(group);

        RuntimeException thrown = expectRuntime(() -> runner.update(manualClock.clock()));

        assertSame(startFailure, thrown);
        assertTrue(runner.isIdle());
        assertEquals(TaskOutcome.CANCELLED, group.getOutcome());
        assertEquals(1, deadline.cleanupTransitions);
        assertEquals(1, first.cleanupTransitions);
        assertEquals(1, failing.cleanupTransitions);
        assertEquals(0, skipped.startCount);
        assertEquals(0, skipped.cancelInvocations);

        assertContains(expectIllegalState(() -> group.start(manualClock.clock())), "single-use");
        assertEquals(1, deadline.startCount);
        assertEquals(1, first.startCount);
        assertEquals(1, failing.startCount);
    }

    @Test
    public void deadlineStartFailureCleansDeadlineAndSkipsCompanions() {
        ManualLoopClock manualClock = new ManualLoopClock();
        ProbeTask deadline = new ProbeTask("deadline");
        ProbeTask skipped = new ProbeTask("skipped");
        RuntimeException startFailure = new RuntimeException("deadline start failed");
        RuntimeException cleanupFailure = new RuntimeException("deadline cleanup failed");
        deadline.startFailure = startFailure;
        deadline.cancelFailure = cleanupFailure;
        Task group = Tasks.parallelDeadline(deadline, skipped);
        TaskRunner runner = new TaskRunner();
        runner.enqueue(group);

        RuntimeException thrown = expectRuntime(() -> runner.update(manualClock.clock()));

        assertSame(startFailure, thrown);
        assertEquals(1, thrown.getSuppressed().length);
        assertSame(cleanupFailure, thrown.getSuppressed()[0]);
        assertTrue(runner.isIdle());
        assertEquals(TaskOutcome.CANCELLED, group.getOutcome());
        assertEquals(1, deadline.startCount);
        assertEquals(1, deadline.cancelInvocations);
        assertEquals(1, deadline.cleanupTransitions);
        assertEquals(0, skipped.startCount);
        assertEquals(0, skipped.cancelInvocations);
    }

    @Test
    public void lifecycleFailuresFailStopWithoutCleanupQueries() {
        ManualLoopClock manualClock = new ManualLoopClock();
        ProbeTask deadline = new ProbeTask("deadline");
        ProbeTask failingQuery = new ProbeTask("query");
        ProbeTask later = new ProbeTask("later");
        RuntimeException queryFailure = new RuntimeException("query failed");
        failingQuery.completeFailure = queryFailure;
        Task group = Tasks.parallelDeadline(deadline, failingQuery, later);
        TaskRunner runner = new TaskRunner();
        runner.enqueue(group);

        RuntimeException thrown = expectRuntime(() -> runner.update(manualClock.clock()));

        assertSame(queryFailure, thrown);
        assertTrue(runner.isIdle());
        assertEquals(1, failingQuery.completeChecks);
        assertEquals(1, deadline.cancelInvocations);
        assertEquals(1, failingQuery.cancelInvocations);
        assertEquals(1, later.cancelInvocations);

        ProbeTask updateDeadline = new ProbeTask("updateDeadline");
        ProbeTask failingUpdate = new ProbeTask("failingUpdate");
        ProbeTask updateLater = new ProbeTask("updateLater");
        RuntimeException updateFailure = new RuntimeException("update failed");
        failingUpdate.updateFailure = updateFailure;
        Task updateGroup = Tasks.parallelDeadline(
                updateDeadline,
                failingUpdate,
                updateLater);
        TaskRunner updateRunner = new TaskRunner();
        updateRunner.enqueue(updateGroup);

        assertSame(
                updateFailure,
                expectRuntime(() -> updateRunner.update(new ManualLoopClock().clock())));
        assertTrue(updateRunner.isIdle());
        assertEquals(1, updateDeadline.cleanupTransitions);
        assertEquals(1, failingUpdate.cleanupTransitions);
        assertEquals(1, updateLater.cleanupTransitions);
    }

    @Test
    public void malformedOrThrowingDeadlineOutcomeFailsClosed() {
        OutcomeFailureResult nullResult = runOutcomeFailure(null, null, null, null);
        assertContains(nullResult.thrown, "reported null", "lifecycle contract");
        assertTrue(nullResult.runner.isIdle());
        assertEquals(TaskOutcome.CANCELLED, nullResult.group.getOutcome());
        assertEquals(1, nullResult.companions[0].cleanupTransitions);

        OutcomeFailureResult notDoneResult =
                runOutcomeFailure(TaskOutcome.NOT_DONE, null, null, null);
        assertContains(notDoneResult.thrown, "NOT_DONE", "lifecycle contract");
        assertTrue(notDoneResult.runner.isIdle());
        assertEquals(1, notDoneResult.companions[0].cleanupTransitions);

        RuntimeException outcomeFailure = new RuntimeException("outcome failed");
        RuntimeException cleanupFailure = new RuntimeException("cleanup one");
        RuntimeException laterCleanupFailure = new RuntimeException("cleanup two");
        OutcomeFailureResult throwingResult = runOutcomeFailure(
                TaskOutcome.SUCCESS,
                outcomeFailure,
                cleanupFailure,
                laterCleanupFailure);

        assertSame(outcomeFailure, throwingResult.thrown);
        assertEquals(1, outcomeFailure.getSuppressed().length);
        assertSame(cleanupFailure, outcomeFailure.getSuppressed()[0]);
        assertEquals(1, cleanupFailure.getSuppressed().length);
        assertSame(laterCleanupFailure, cleanupFailure.getSuppressed()[0]);
        assertTrue(throwingResult.runner.isIdle());
        assertEquals(TaskOutcome.CANCELLED, throwingResult.group.getOutcome());
        assertEquals(1, throwingResult.deadline.cancelInvocations);
        assertEquals(1, throwingResult.companions[0].cancelInvocations);
        assertEquals(1, throwingResult.companions[1].cancelInvocations);
    }

    @Test
    public void preStartTerminalAndSingleUseLifecycleIsActionable() {
        ManualLoopClock manualClock = new ManualLoopClock();
        ProbeTask deadline = new ProbeTask("deadline");
        ProbeTask companion = new ProbeTask("companion");
        Task group = Tasks.parallelDeadline(deadline, companion);

        group.cancel();
        assertFalse(group.isComplete());
        assertEquals(0, deadline.cancelInvocations);
        assertEquals(0, companion.cancelInvocations);
        assertContains(
                expectIllegalState(() -> group.update(manualClock.clock())),
                "Tasks.parallelDeadline",
                "before start",
                "TaskRunner");

        group.start(manualClock.clock());
        assertFalse(group.isComplete());
        assertContains(
                expectIllegalState(() -> group.start(manualClock.clock())),
                "single-use",
                "Supplier<Task>",
                "OutputTaskFactory");
        assertEquals(1, deadline.startCount);
        assertEquals(1, companion.startCount);
        group.cancel();
        group.update(manualClock.clock());
        assertEquals(TaskOutcome.CANCELLED, group.getOutcome());

        ProbeTask completedDeadline = new ProbeTask("completedDeadline");
        completedDeadline.startHook =
                () -> completedDeadline.finish(TaskOutcome.SUCCESS);
        Task completedGroup = Tasks.parallelDeadline(completedDeadline);
        completedGroup.start(manualClock.clock());
        assertContains(
                expectIllegalState(() -> completedGroup.start(manualClock.clock())),
                "single-use",
                "fresh task");
        assertEquals(1, completedDeadline.startCount);
    }

    @Test
    public void hiddenNestedAliasFailsAtLeafAndCleansItOnce() {
        ManualLoopClock manualClock = new ManualLoopClock();
        AtomicInteger startEffects = new AtomicInteger();
        AtomicInteger cleanupEffects = new AtomicInteger();
        Task sharedLeaf = new RunForSecondsTask(
                1.0,
                startEffects::incrementAndGet,
                null,
                cleanupEffects::incrementAndGet);
        Task deadline = Tasks.sequence(sharedLeaf);
        Task companion = Tasks.sequence(sharedLeaf);
        Task group = Tasks.parallelDeadline(deadline, companion);
        TaskRunner runner = new TaskRunner();
        runner.enqueue(group);

        IllegalStateException thrown =
                expectIllegalState(() -> runner.update(manualClock.clock()));

        assertContains(thrown, "RunForSecondsTask", "single-use");
        assertEquals(1, startEffects.get());
        assertEquals(1, cleanupEffects.get());
        assertEquals(TaskOutcome.CANCELLED, group.getOutcome());
        assertTrue(runner.isIdle());
    }

    @Test
    public void debugDumpUsesStableDeadlineAndCompanionPrefixes() {
        ProbeTask deadline = new ProbeTask("deadline");
        ProbeTask companion = new ProbeTask("companion");
        Task group = Tasks.parallelDeadline(deadline, companion);
        group.start(new ManualLoopClock().clock());
        CapturingDebugSink sink = new CapturingDebugSink();

        group.debugDump(sink, "auto.collect");

        assertEquals(Boolean.TRUE, sink.values.get("auto.collect.started"));
        assertEquals(Boolean.FALSE, sink.values.get("auto.collect.complete"));
        assertEquals(TaskOutcome.NOT_DONE, sink.values.get("auto.collect.outcome"));
        assertEquals(Boolean.TRUE,
                sink.values.get("auto.collect.deadlineStartAttempted"));
        assertEquals(Double.valueOf(1.0), sink.values.get("auto.collect.companionCount"));
        assertEquals("deadline", sink.values.get("auto.collect.deadline.name"));
        assertEquals(Boolean.TRUE,
                sink.values.get("auto.collect.companion0.startAttempted"));
        assertEquals("companion", sink.values.get("auto.collect.companion0.name"));
        group.cancel();
    }

    private static OutcomeFailureResult runOutcomeFailure(
            TaskOutcome reportedOutcome,
            RuntimeException outcomeFailure,
            RuntimeException firstCleanupFailure,
            RuntimeException secondCleanupFailure) {
        ManualLoopClock manualClock = new ManualLoopClock();
        ProbeTask deadline = new ProbeTask("deadline");
        ProbeTask first = new ProbeTask("first");
        ProbeTask second = new ProbeTask("second");
        first.cancelFailure = firstCleanupFailure;
        second.cancelFailure = secondCleanupFailure;
        Task group = Tasks.parallelDeadline(deadline, first, second);
        TaskRunner runner = new TaskRunner();
        runner.enqueue(group);
        runner.update(manualClock.clock());

        deadline.finish(reportedOutcome);
        deadline.outcomeFailure = outcomeFailure;
        RuntimeException thrown =
                expectRuntime(() -> runner.update(manualClock.nextCycle(0.02)));
        return new OutcomeFailureResult(
                thrown,
                deadline,
                new ProbeTask[]{first, second},
                group,
                runner);
    }

    private static void assertContains(RuntimeException failure, String... fragments) {
        String message = failure.getMessage();
        for (String fragment : fragments) {
            assertTrue(
                    "Expected message to contain '" + fragment + "' but was: " + message,
                    message != null && message.contains(fragment));
        }
    }

    private static IllegalArgumentException expectIllegalArgument(Runnable action) {
        try {
            action.run();
            fail("Expected IllegalArgumentException");
            return null;
        } catch (IllegalArgumentException expected) {
            return expected;
        }
    }

    private static IllegalStateException expectIllegalState(Runnable action) {
        try {
            action.run();
            fail("Expected IllegalStateException");
            return null;
        } catch (IllegalStateException expected) {
            return expected;
        }
    }

    private static RuntimeException expectRuntime(Runnable action) {
        try {
            action.run();
            fail("Expected RuntimeException");
            return null;
        } catch (RuntimeException expected) {
            return expected;
        }
    }

    private static final class OutcomeFailureResult {
        private final RuntimeException thrown;
        private final ProbeTask deadline;
        private final ProbeTask[] companions;
        private final Task group;
        private final TaskRunner runner;

        private OutcomeFailureResult(RuntimeException thrown,
                                     ProbeTask deadline,
                                     ProbeTask[] companions,
                                     Task group,
                                     TaskRunner runner) {
            this.thrown = thrown;
            this.deadline = deadline;
            this.companions = companions;
            this.group = group;
            this.runner = runner;
        }
    }

    private static final class ProbeTask implements Task {
        private final String name;
        private final List<String> events;

        private int startCount;
        private int updateCount;
        private int completeChecks;
        private int outcomeChecks;
        private int cancelInvocations;
        private int cleanupTransitions;
        private boolean started;
        private boolean complete;
        private TaskOutcome outcome = TaskOutcome.NOT_DONE;

        private Runnable startHook;
        private Runnable updateHook;
        private Runnable completeHook;
        private Runnable outcomeHook;
        private Runnable cancelHook;
        private RuntimeException startFailure;
        private RuntimeException updateFailure;
        private RuntimeException completeFailure;
        private RuntimeException outcomeFailure;
        private RuntimeException cancelFailure;

        private ProbeTask(String name) {
            this(name, new ArrayList<>());
        }

        private ProbeTask(String name, List<String> events) {
            this.name = name;
            this.events = events;
        }

        @Override
        public void start(LoopClock clock) {
            startCount++;
            started = true;
            events.add(name + ".start");
            if (startHook != null) {
                startHook.run();
            }
            if (startFailure != null) {
                throw startFailure;
            }
        }

        @Override
        public void update(LoopClock clock) {
            updateCount++;
            events.add(name + ".update");
            if (updateHook != null) {
                updateHook.run();
            }
            if (updateFailure != null) {
                throw updateFailure;
            }
        }

        @Override
        public void cancel() {
            cancelInvocations++;
            events.add(name + ".cancel");
            if (!started || complete) {
                return;
            }
            complete = true;
            outcome = TaskOutcome.CANCELLED;
            cleanupTransitions++;
            if (cancelHook != null) {
                cancelHook.run();
            }
            if (cancelFailure != null) {
                throw cancelFailure;
            }
        }

        @Override
        public boolean isComplete() {
            completeChecks++;
            if (completeHook != null) {
                completeHook.run();
            }
            if (completeFailure != null) {
                throw completeFailure;
            }
            return complete;
        }

        @Override
        public TaskOutcome getOutcome() {
            outcomeChecks++;
            if (outcomeHook != null) {
                outcomeHook.run();
            }
            if (outcomeFailure != null) {
                throw outcomeFailure;
            }
            return outcome;
        }

        @Override
        public String getDebugName() {
            return name;
        }

        private void finish(TaskOutcome terminalOutcome) {
            complete = true;
            outcome = terminalOutcome;
        }
    }

    private static final class CapturingDebugSink implements DebugSink {
        private final Map<String, Object> values = new LinkedHashMap<>();

        @Override
        public DebugSink addData(String key, Object value) {
            values.put(key, value);
            return this;
        }

        @Override
        public DebugSink addLine(String text) {
            return this;
        }
    }
}
