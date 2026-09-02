package edu.ftcsushi.fw.task;

import org.junit.Test;

import java.util.Arrays;
import java.util.concurrent.atomic.AtomicReference;

import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Outcome, continuation, and lifecycle matrix for generic Task composition. */
public final class OutcomeAwareCompositionTest {

    @Test
    public void ordinarySequenceAdvancesOnlyAfterExactSuccessAndCachesOutcome() {
        for (TaskOutcome terminal : terminalOutcomes()) {
            ManualLoopClock clock = new ManualLoopClock();
            ProbeTask first = ProbeTask.immediate("first-" + terminal, terminal);
            ProbeTask second = ProbeTask.immediate("second", TaskOutcome.SUCCESS);
            Task sequence = Tasks.sequence(first, second);

            sequence.start(clock.clock());

            assertTrue(sequence.isComplete());
            assertEquals(terminal, sequence.getOutcome());
            assertEquals(terminal == TaskOutcome.SUCCESS ? 1 : 0, second.startCount);
            assertEquals(1, first.outcomeCalls);

            // Terminal wrapper observations use the cached child result.
            assertEquals(terminal, sequence.getOutcome());
            assertEquals(1, first.outcomeCalls);
        }
    }

    @Test
    public void ordinarySequenceAppliesEveryOutcomeAfterBetweenCycleCompletion() {
        for (TaskOutcome terminal : terminalOutcomes()) {
            ManualLoopClock clock = new ManualLoopClock();
            ProbeTask first = new ProbeTask("first-" + terminal);
            ProbeTask second = ProbeTask.immediate("second", TaskOutcome.SUCCESS);
            Task sequence = Tasks.sequence(first, second);
            sequence.start(clock.clock());

            first.finish(terminal);
            sequence.update(clock.nextCycle(0.02));

            assertEquals(0, first.updateCount);
            assertEquals(terminal == TaskOutcome.SUCCESS ? 1 : 0, second.startCount);
            assertEquals(terminal, sequence.getOutcome());
            assertEquals(1, first.outcomeCalls);
        }
    }

    @Test
    public void sequenceObservesBetweenCycleCompletionBeforeUpdateAndHandsOffSameCallback() {
        ManualLoopClock clock = new ManualLoopClock();
        ProbeTask first = new ProbeTask("first");
        ProbeTask second = new ProbeTask("second");
        Task sequence = Tasks.sequence(first, second);

        sequence.start(clock.clock());
        first.finish(TaskOutcome.SUCCESS);
        sequence.update(clock.nextCycle(0.02));

        assertEquals(0, first.updateCount);
        assertEquals(1, second.startCount);
        assertEquals(0, second.updateCount);
        assertFalse(sequence.isComplete());

        second.finish(TaskOutcome.TIMEOUT);
        sequence.update(clock.nextCycle(0.02));

        assertEquals(0, second.updateCount);
        assertTrue(sequence.isComplete());
        assertEquals(TaskOutcome.TIMEOUT, sequence.getOutcome());
    }

    @Test
    public void completionSequenceRunsRepairStepsAndRetainsFirstNonSuccess() {
        ManualLoopClock clock = new ManualLoopClock();
        ProbeTask timedOut = ProbeTask.immediate("timedOut", TaskOutcome.TIMEOUT);
        ProbeTask cancelled = ProbeTask.immediate("cancelled", TaskOutcome.CANCELLED);
        ProbeTask repair = new ProbeTask("repair");
        Task sequence = Tasks.sequenceOnCompletion(timedOut, cancelled, repair);

        sequence.start(clock.clock());

        assertEquals(1, timedOut.startCount);
        assertEquals(1, cancelled.startCount);
        assertEquals(1, repair.startCount);
        assertFalse(sequence.isComplete());

        repair.finish(TaskOutcome.SUCCESS);
        sequence.update(clock.nextCycle(0.02));

        assertTrue(sequence.isComplete());
        assertEquals(TaskOutcome.TIMEOUT, sequence.getOutcome());
        assertEquals(1, timedOut.outcomeCalls);
        assertEquals(1, cancelled.outcomeCalls);
        assertEquals(1, repair.outcomeCalls);

        Task empty = Tasks.sequenceOnCompletion();
        empty.start(clock.clock());
        assertEquals(TaskOutcome.SUCCESS, empty.getOutcome());
    }

    @Test
    public void completionSequenceAdvancesAfterEveryValidTerminalOutcome() {
        for (TaskOutcome terminal : terminalOutcomes()) {
            ManualLoopClock clock = new ManualLoopClock();
            ProbeTask first = ProbeTask.immediate("first-" + terminal, terminal);
            ProbeTask later = ProbeTask.immediate("later", TaskOutcome.SUCCESS);
            Task sequence = Tasks.sequenceOnCompletion(first, later);

            sequence.start(clock.clock());

            assertTrue(sequence.isComplete());
            assertEquals(terminal, sequence.getOutcome());
            assertEquals(1, later.startCount);
            assertEquals(1, first.outcomeCalls);
            assertEquals(1, later.outcomeCalls);
        }
    }

    @Test
    public void completionSequenceAppliesEveryOutcomeAfterBetweenCycleCompletion() {
        for (TaskOutcome terminal : terminalOutcomes()) {
            ManualLoopClock clock = new ManualLoopClock();
            ProbeTask first = new ProbeTask("first-" + terminal);
            ProbeTask later = ProbeTask.immediate("later", TaskOutcome.SUCCESS);
            Task sequence = Tasks.sequenceOnCompletion(first, later);
            sequence.start(clock.clock());

            first.finish(terminal);
            sequence.update(clock.nextCycle(0.02));

            assertEquals(0, first.updateCount);
            assertEquals(1, later.startCount);
            assertEquals(terminal, sequence.getOutcome());
            assertEquals(1, first.outcomeCalls);
            assertEquals(1, later.outcomeCalls);
        }
    }

    @Test
    public void directCancellationOfEitherSequenceSkipsEveryLaterChild() {
        for (boolean continueOnCompletion : Arrays.asList(false, true)) {
            ManualLoopClock clock = new ManualLoopClock();
            ProbeTask active = new ProbeTask("active");
            ProbeTask later = ProbeTask.immediate("later", TaskOutcome.SUCCESS);
            Task sequence = continueOnCompletion
                    ? Tasks.sequenceOnCompletion(active, later)
                    : Tasks.sequence(active, later);

            sequence.start(clock.clock());
            sequence.cancel();
            sequence.cancel();

            assertTrue(sequence.isComplete());
            assertEquals(TaskOutcome.CANCELLED, sequence.getOutcome());
            assertEquals(1, active.cancelCount);
            assertEquals(0, later.startCount);
        }
    }

    @Test
    public void sequenceMalformedOutcomeAndLifecycleFailureFailClosed() {
        for (TaskOutcome malformed : Arrays.asList(null, TaskOutcome.NOT_DONE)) {
            ManualLoopClock clock = new ManualLoopClock();
            ProbeTask child = ProbeTask.immediate("malformed", malformed);
            ProbeTask later = ProbeTask.immediate("later", TaskOutcome.SUCCESS);
            Task sequence = Tasks.sequenceOnCompletion(child, later);

            IllegalStateException failure = expectIllegalState(
                    () -> sequence.start(clock.clock()));

            assertContains(failure, "sequenceOnCompletion", "index 0",
                    String.valueOf(malformed), "lifecycle contract");
            assertEquals(TaskOutcome.CANCELLED, sequence.getOutcome());
            assertEquals(0, later.startCount);
            assertEquals(1, child.outcomeCalls);
        }

        ManualLoopClock clock = new ManualLoopClock();
        RuntimeException updateFailure = new RuntimeException("update failed");
        RuntimeException cleanupFailure = new RuntimeException("cleanup failed");
        ProbeTask active = new ProbeTask("active");
        active.updateFailure = updateFailure;
        active.cancelFailure = cleanupFailure;
        ProbeTask later = ProbeTask.immediate("later", TaskOutcome.SUCCESS);
        Task sequence = Tasks.sequenceOnCompletion(active, later);
        sequence.start(clock.clock());

        RuntimeException observed = expectRuntime(() -> sequence.update(clock.clock()));

        assertSame(updateFailure, observed);
        assertEquals(1, observed.getSuppressed().length);
        assertSame(cleanupFailure, observed.getSuppressed()[0]);
        assertEquals(TaskOutcome.CANCELLED, sequence.getOutcome());
        assertEquals(0, later.startCount);
    }

    @Test
    public void reentrantSequenceCancellationWinsDuringOutcomeAndSuccessorStart() {
        ManualLoopClock clock = new ManualLoopClock();
        AtomicReference<Task> outcomeSequenceRef = new AtomicReference<>();
        ProbeTask outcomeChild = ProbeTask.immediate("outcomeChild", TaskOutcome.SUCCESS);
        outcomeChild.outcomeHook = () -> outcomeSequenceRef.get().cancel();
        ProbeTask outcomeLater = ProbeTask.immediate("outcomeLater", TaskOutcome.SUCCESS);
        Task outcomeSequence = Tasks.sequence(outcomeChild, outcomeLater);
        outcomeSequenceRef.set(outcomeSequence);

        outcomeSequence.start(clock.clock());

        assertEquals(TaskOutcome.CANCELLED, outcomeSequence.getOutcome());
        assertEquals(0, outcomeLater.startCount);

        AtomicReference<Task> startSequenceRef = new AtomicReference<>();
        ProbeTask first = ProbeTask.immediate("first", TaskOutcome.SUCCESS);
        ProbeTask successor = new ProbeTask("successor");
        successor.startHook = () -> startSequenceRef.get().cancel();
        ProbeTask finalChild = ProbeTask.immediate("final", TaskOutcome.SUCCESS);
        Task startSequence = Tasks.sequence(first, successor, finalChild);
        startSequenceRef.set(startSequence);

        startSequence.start(clock.clock());

        assertEquals(TaskOutcome.CANCELLED, startSequence.getOutcome());
        assertEquals(1, successor.startCount);
        assertEquals(1, successor.cancelCount);
        assertEquals(0, finalChild.startCount);
    }

    @Test
    public void reentrantSequenceCancellationWinsDuringCompletionObservation() {
        for (boolean continueOnCompletion : Arrays.asList(false, true)) {
            ManualLoopClock clock = new ManualLoopClock();
            AtomicReference<Task> sequenceRef = new AtomicReference<>();
            ProbeTask first = ProbeTask.immediate("first", TaskOutcome.SUCCESS);
            first.completeHook = () -> sequenceRef.get().cancel();
            ProbeTask later = ProbeTask.immediate("later", TaskOutcome.SUCCESS);
            Task sequence = continueOnCompletion
                    ? Tasks.sequenceOnCompletion(first, later)
                    : Tasks.sequence(first, later);
            sequenceRef.set(sequence);

            sequence.start(clock.clock());

            assertEquals(TaskOutcome.CANCELLED, sequence.getOutcome());
            assertEquals(0, later.startCount);
        }
    }

    @Test
    public void parallelAllWaitsForEveryChildAndAggregatesExactOutcomes() {
        assertParallelOutcome(TaskOutcome.SUCCESS,
                TaskOutcome.SUCCESS, TaskOutcome.SUCCESS);
        assertParallelOutcome(TaskOutcome.TIMEOUT,
                TaskOutcome.SUCCESS, TaskOutcome.TIMEOUT, TaskOutcome.TIMEOUT);
        assertParallelOutcome(TaskOutcome.CANCELLED,
                TaskOutcome.SUCCESS, TaskOutcome.CANCELLED);
        assertParallelOutcome(TaskOutcome.UNKNOWN,
                TaskOutcome.SUCCESS, TaskOutcome.UNKNOWN, TaskOutcome.UNKNOWN);
        assertParallelOutcome(TaskOutcome.UNKNOWN,
                TaskOutcome.TIMEOUT, TaskOutcome.CANCELLED);

        ManualLoopClock clock = new ManualLoopClock();
        ProbeTask timedOut = ProbeTask.immediate("timedOut", TaskOutcome.TIMEOUT);
        ProbeTask active = new ProbeTask("active");
        Task parallel = Tasks.parallelAll(timedOut, active);
        parallel.start(clock.clock());

        assertFalse(parallel.isComplete());
        assertEquals(TaskOutcome.NOT_DONE, parallel.getOutcome());
        assertEquals(1, timedOut.outcomeCalls);

        active.finish(TaskOutcome.TIMEOUT);
        parallel.update(clock.nextCycle(0.02));

        assertEquals(0, active.updateCount);
        assertEquals(TaskOutcome.TIMEOUT, parallel.getOutcome());
        assertEquals(1, timedOut.outcomeCalls);
        assertEquals(1, active.outcomeCalls);
    }

    @Test
    public void parallelAllAggregatesEveryOutcomeAfterBetweenCycleCompletion() {
        assertParallelOutcomeAfterUpdate(TaskOutcome.SUCCESS,
                TaskOutcome.SUCCESS, TaskOutcome.SUCCESS);
        assertParallelOutcomeAfterUpdate(TaskOutcome.TIMEOUT,
                TaskOutcome.SUCCESS, TaskOutcome.TIMEOUT, TaskOutcome.TIMEOUT);
        assertParallelOutcomeAfterUpdate(TaskOutcome.CANCELLED,
                TaskOutcome.SUCCESS, TaskOutcome.CANCELLED);
        assertParallelOutcomeAfterUpdate(TaskOutcome.UNKNOWN,
                TaskOutcome.SUCCESS, TaskOutcome.UNKNOWN, TaskOutcome.UNKNOWN);
        assertParallelOutcomeAfterUpdate(TaskOutcome.UNKNOWN,
                TaskOutcome.TIMEOUT, TaskOutcome.CANCELLED);
    }

    @Test
    public void parallelMalformedOutcomeCancelsStartedSiblingsAndPreservesPrimaryFailure() {
        for (TaskOutcome malformed : Arrays.asList(null, TaskOutcome.NOT_DONE)) {
            ManualLoopClock clock = new ManualLoopClock();
            RuntimeException cleanupFailure = new RuntimeException("cleanup failed");
            ProbeTask active = new ProbeTask("active");
            active.cancelFailure = cleanupFailure;
            ProbeTask invalid = ProbeTask.immediate("invalid", malformed);
            Task parallel = Tasks.parallelAll(active, invalid);

            IllegalStateException failure = expectIllegalState(
                    () -> parallel.start(clock.clock()));

            assertContains(failure, "parallelAll", "index 1", String.valueOf(malformed),
                    "lifecycle contract");
            assertEquals(1, failure.getSuppressed().length);
            assertSame(cleanupFailure, failure.getSuppressed()[0]);
            assertEquals(1, active.cancelCount);
            assertEquals(TaskOutcome.CANCELLED, parallel.getOutcome());
            assertEquals(1, invalid.outcomeCalls);
        }
    }

    @Test
    public void parallelReentrantCancellationDuringOutcomeWins() {
        ManualLoopClock clock = new ManualLoopClock();
        AtomicReference<Task> parallelRef = new AtomicReference<>();
        ProbeTask first = ProbeTask.immediate("first", TaskOutcome.SUCCESS);
        first.outcomeHook = () -> parallelRef.get().cancel();
        ProbeTask second = ProbeTask.immediate("second", TaskOutcome.SUCCESS);
        Task parallel = Tasks.parallelAll(first, second);
        parallelRef.set(parallel);

        parallel.start(clock.clock());

        assertTrue(parallel.isComplete());
        assertEquals(TaskOutcome.CANCELLED, parallel.getOutcome());
        assertEquals(1, first.outcomeCalls);
        assertEquals(0, second.outcomeCalls);
    }

    @Test
    public void directParallelCancellationAttemptsEveryChildAndAggregatesCleanupFailures() {
        ManualLoopClock clock = new ManualLoopClock();
        AtomicReference<Task> reentrantRef = new AtomicReference<>();
        ProbeTask cancellingStart = new ProbeTask("cancellingStart");
        cancellingStart.startHook = () -> reentrantRef.get().cancel();
        ProbeTask neverStarted = new ProbeTask("neverStarted");
        Task reentrant = Tasks.parallelAll(cancellingStart, neverStarted);
        reentrantRef.set(reentrant);

        reentrant.start(clock.clock());

        assertEquals(TaskOutcome.CANCELLED, reentrant.getOutcome());
        assertEquals(0, neverStarted.startCount);
        assertEquals(1, neverStarted.cancelInvocations);
        assertEquals(0, neverStarted.cancelCount);

        RuntimeException firstFailure = new RuntimeException("first cleanup");
        RuntimeException secondFailure = new RuntimeException("second cleanup");
        ProbeTask first = new ProbeTask("first");
        first.cancelFailure = firstFailure;
        ProbeTask second = new ProbeTask("second");
        second.cancelFailure = secondFailure;
        ProbeTask third = new ProbeTask("third");
        Task parallel = Tasks.parallelAll(first, second, third);
        parallel.start(clock.clock());

        RuntimeException observed = expectRuntime(parallel::cancel);

        assertSame(firstFailure, observed);
        assertEquals(1, observed.getSuppressed().length);
        assertSame(secondFailure, observed.getSuppressed()[0]);
        assertEquals(1, first.cancelCount);
        assertEquals(1, second.cancelCount);
        assertEquals(1, third.cancelCount);
        assertEquals(TaskOutcome.CANCELLED, parallel.getOutcome());
    }

    @Test
    public void branchSelectsOnlyExactSuccessOrTimeoutAndCachesTerminalResults() {
        for (TaskOutcome moveOutcome : terminalOutcomes()) {
            ManualLoopClock clock = new ManualLoopClock();
            ProbeTask move = ProbeTask.immediate("move", moveOutcome);
            ProbeTask success = ProbeTask.immediate("success", TaskOutcome.SUCCESS);
            ProbeTask timeout = ProbeTask.immediate("timeout", TaskOutcome.SUCCESS);
            Task branch = Tasks.branchOnOutcome(move, success, timeout);

            branch.start(clock.clock());
            branch.update(clock.clock());

            if (moveOutcome == TaskOutcome.SUCCESS || moveOutcome == TaskOutcome.TIMEOUT) {
                ProbeTask selected = moveOutcome == TaskOutcome.SUCCESS ? success : timeout;
                ProbeTask skipped = moveOutcome == TaskOutcome.SUCCESS ? timeout : success;
                assertEquals(1, selected.startCount);
                assertEquals(0, selected.updateCount);
                assertEquals(0, skipped.startCount);
                assertFalse(branch.isComplete());

                branch.update(clock.nextCycle(0.02));
                assertEquals(TaskOutcome.SUCCESS, branch.getOutcome());
                assertEquals(1, selected.outcomeCalls);
            } else {
                assertTrue(branch.isComplete());
                assertEquals(moveOutcome, branch.getOutcome());
                assertEquals(0, success.startCount);
                assertEquals(0, timeout.startCount);
            }

            assertEquals(1, move.outcomeCalls);
            branch.getOutcome();
            assertEquals(1, move.outcomeCalls);
        }
    }

    @Test
    public void branchRetainsAnyValidSelectedBranchOutcomeAndRejectsMalformedEvidence() {
        for (TaskOutcome selectedOutcome : terminalOutcomes()) {
            ManualLoopClock clock = new ManualLoopClock();
            ProbeTask selected = ProbeTask.immediate("selected", selectedOutcome);
            Task branch = Tasks.branchOnOutcome(
                    ProbeTask.immediate("move", TaskOutcome.SUCCESS),
                    selected,
                    ProbeTask.immediate("timeout", TaskOutcome.SUCCESS));

            branch.start(clock.clock());
            branch.update(clock.clock());
            branch.update(clock.nextCycle(0.02));

            assertEquals(selectedOutcome, branch.getOutcome());
            assertEquals(1, selected.outcomeCalls);
        }

        for (TaskOutcome malformed : Arrays.asList(null, TaskOutcome.NOT_DONE)) {
            ManualLoopClock clock = new ManualLoopClock();
            ProbeTask move = ProbeTask.immediate("move", malformed);
            ProbeTask success = ProbeTask.immediate("success", TaskOutcome.SUCCESS);
            ProbeTask timeout = ProbeTask.immediate("timeout", TaskOutcome.SUCCESS);
            Task branch = Tasks.branchOnOutcome(move, success, timeout);
            branch.start(clock.clock());

            IllegalStateException failure = expectIllegalState(() -> branch.update(clock.clock()));

            assertContains(failure, "branchOnOutcome", "move", String.valueOf(malformed),
                    "lifecycle contract");
            assertEquals(TaskOutcome.CANCELLED, branch.getOutcome());
            assertEquals(0, success.startCount);
            assertEquals(0, timeout.startCount);
        }

        for (TaskOutcome malformed : Arrays.asList(null, TaskOutcome.NOT_DONE)) {
            ManualLoopClock clock = new ManualLoopClock();
            ProbeTask selected = ProbeTask.immediate("selected", malformed);
            ProbeTask timeout = ProbeTask.immediate("timeout", TaskOutcome.SUCCESS);
            Task branch = Tasks.branchOnOutcome(
                    ProbeTask.immediate("move", TaskOutcome.SUCCESS),
                    selected,
                    timeout);
            branch.start(clock.clock());
            branch.update(clock.clock());

            IllegalStateException failure = expectIllegalState(
                    () -> branch.update(clock.nextCycle(0.02)));

            assertContains(failure, "branchOnOutcome", "selected branch",
                    String.valueOf(malformed), "lifecycle contract");
            assertEquals(TaskOutcome.CANCELLED, branch.getOutcome());
            assertEquals(0, timeout.startCount);
            assertEquals(1, selected.outcomeCalls);
        }
    }

    /** Assert one all-immediate parallel aggregate and its one-query outcome cache. */
    private static void assertParallelOutcome(TaskOutcome expected, TaskOutcome... outcomes) {
        ManualLoopClock clock = new ManualLoopClock();
        ProbeTask[] children = new ProbeTask[outcomes.length];
        for (int i = 0; i < outcomes.length; i++) {
            children[i] = ProbeTask.immediate("child" + i, outcomes[i]);
        }
        Task parallel = Tasks.parallelAll(children);

        parallel.start(clock.clock());

        assertTrue(parallel.isComplete());
        assertEquals(expected, parallel.getOutcome());
        assertEquals(expected, parallel.getOutcome());
        for (ProbeTask child : children) {
            assertEquals(1, child.startCount);
            assertEquals(1, child.outcomeCalls);
        }
    }

    /** Assert one between-cycle parallel aggregate and its one-query child outcome cache. */
    private static void assertParallelOutcomeAfterUpdate(TaskOutcome expected,
                                                         TaskOutcome... outcomes) {
        ManualLoopClock clock = new ManualLoopClock();
        ProbeTask[] children = new ProbeTask[outcomes.length];
        for (int i = 0; i < outcomes.length; i++) {
            children[i] = new ProbeTask("child" + i);
        }
        Task parallel = Tasks.parallelAll(children);
        parallel.start(clock.clock());

        for (int i = 0; i < outcomes.length; i++) {
            children[i].finish(outcomes[i]);
        }
        parallel.update(clock.nextCycle(0.02));

        assertTrue(parallel.isComplete());
        assertEquals(expected, parallel.getOutcome());
        for (ProbeTask child : children) {
            assertEquals(0, child.updateCount);
            assertEquals(1, child.outcomeCalls);
        }
    }

    /** Every valid terminal Task outcome in enum-independent teaching order. */
    private static Iterable<TaskOutcome> terminalOutcomes() {
        return Arrays.asList(
                TaskOutcome.SUCCESS,
                TaskOutcome.TIMEOUT,
                TaskOutcome.CANCELLED,
                TaskOutcome.UNKNOWN);
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

    private static void assertContains(Throwable failure, String... snippets) {
        String message = failure.getMessage();
        for (String snippet : snippets) {
            assertTrue("Expected message to contain '" + snippet + "' but was: " + message,
                    message != null && message.contains(snippet));
        }
    }

    /** Small controllable Task that exposes lifecycle counts without hiding terminal outcomes. */
    private static final class ProbeTask implements Task {
        private final String name;
        private int startCount;
        private int updateCount;
        private int cancelCount;
        private int cancelInvocations;
        private int outcomeCalls;
        private boolean started;
        private boolean complete;
        private TaskOutcome outcome = TaskOutcome.NOT_DONE;
        private Runnable startHook;
        private Runnable updateHook;
        private Runnable completeHook;
        private Runnable outcomeHook;
        private RuntimeException updateFailure;
        private RuntimeException cancelFailure;

        private ProbeTask(String name) {
            this.name = name;
        }

        private static ProbeTask immediate(String name, TaskOutcome outcome) {
            ProbeTask task = new ProbeTask(name);
            task.startHook = () -> task.finish(outcome);
            return task;
        }

        private void finish(TaskOutcome terminalOutcome) {
            complete = true;
            outcome = terminalOutcome;
        }

        @Override
        public void start(LoopClock clock) {
            startCount++;
            started = true;
            if (startHook != null) {
                startHook.run();
            }
        }

        @Override
        public void update(LoopClock clock) {
            updateCount++;
            if (updateFailure != null) {
                throw updateFailure;
            }
            if (updateHook != null) {
                updateHook.run();
            }
        }

        @Override
        public void cancel() {
            cancelInvocations++;
            if (!started || complete) {
                return;
            }
            cancelCount++;
            complete = true;
            outcome = TaskOutcome.CANCELLED;
            if (cancelFailure != null) {
                throw cancelFailure;
            }
        }

        @Override
        public boolean isComplete() {
            if (completeHook != null) {
                completeHook.run();
            }
            return complete;
        }

        @Override
        public TaskOutcome getOutcome() {
            outcomeCalls++;
            if (outcomeHook != null) {
                outcomeHook.run();
            }
            return outcome;
        }

        @Override
        public String getDebugName() {
            return name;
        }
    }
}
