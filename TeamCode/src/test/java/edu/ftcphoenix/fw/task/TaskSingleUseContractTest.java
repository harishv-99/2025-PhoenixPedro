package edu.ftcphoenix.fw.task;

import org.junit.Test;

import java.lang.reflect.Modifier;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Supplier;

import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the core framework's fail-fast, single-use Task lifecycle contract. */
public final class TaskSingleUseContractTest {

    @Test
    public void primitiveTasksRejectSecondStartWhileActive() {
        ManualLoopClock manualClock = new ManualLoopClock();
        LoopClock clock = manualClock.clock();

        assertActiveThenRejects(
                new RunForSecondsTask(1.0, null, null, null),
                clock,
                "RunForSecondsTask");
        assertActiveThenRejects(
                new WaitUntilTask(BooleanSource.constant(false)),
                clock,
                "WaitUntilTask");
        assertActiveThenRejects(
                new OutputForSecondsTask("activeOutput", 0.5, 1.0),
                clock,
                "activeOutput");
        assertActiveThenRejects(
                new GatedOutputUntilTask(
                        "activeGate",
                        BooleanSource.constant(false),
                        BooleanSource.constant(false),
                        ScalarSource.constant(0.5),
                        0.0,
                        0.0,
                        1.0,
                        0.0),
                clock,
                "activeGate");
    }

    @Test
    public void primitiveTasksRejectSecondStartAfterCompletion() {
        ManualLoopClock manualClock = new ManualLoopClock();
        LoopClock clock = manualClock.clock();

        Task instant = new InstantTask(() -> { });
        instant.start(clock);
        assertTrue(instant.isComplete());
        assertSingleUseStartRejected(instant, clock, "InstantTask");

        Task noop = Tasks.noop();
        noop.start(clock);
        assertTrue(noop.isComplete());
        assertSingleUseStartRejected(noop, clock, "Tasks.noop");

        Task timed = new RunForSecondsTask(0.0, null, null, null);
        timed.start(clock);
        assertTrue(timed.isComplete());
        assertSingleUseStartRejected(timed, clock, "RunForSecondsTask");

        Task wait = new WaitUntilTask(BooleanSource.constant(true));
        wait.start(clock);
        wait.update(clock);
        assertTrue(wait.isComplete());
        assertSingleUseStartRejected(wait, clock, "WaitUntilTask");

        Task output = new OutputForSecondsTask("terminalOutput", 0.5, 0.0);
        output.start(clock);
        assertTrue(output.isComplete());
        assertSingleUseStartRejected(output, clock, "terminalOutput");

        Task gated = new GatedOutputUntilTask(
                "terminalGate",
                BooleanSource.constant(true),
                BooleanSource.constant(true),
                ScalarSource.constant(0.5),
                0.0,
                0.0,
                1.0,
                0.0);
        gated.start(clock);
        gated.update(clock);
        assertTrue(gated.isComplete());
        assertSingleUseStartRejected(gated, clock, "terminalGate");
    }

    @Test
    public void startAttemptIsConsumedBeforeUserActionThrows() {
        ManualLoopClock manualClock = new ManualLoopClock();
        LoopClock clock = manualClock.clock();

        AtomicInteger instantCalls = new AtomicInteger();
        InstantTask instant = new InstantTask(() -> {
            instantCalls.incrementAndGet();
            throw new ExpectedStartFailure();
        });
        expectStartFailure(() -> instant.start(clock));
        assertEquals(1, instantCalls.get());
        assertSingleUseStartRejected(instant, clock, "InstantTask");
        assertEquals(1, instantCalls.get());

        AtomicInteger runCalls = new AtomicInteger();
        RunForSecondsTask run = new RunForSecondsTask(
                1.0,
                () -> {
                    runCalls.incrementAndGet();
                    throw new ExpectedStartFailure();
                },
                null,
                null);
        expectStartFailure(() -> run.start(clock));
        assertEquals(1, runCalls.get());
        assertSingleUseStartRejected(run, clock, "RunForSecondsTask");
        assertEquals(1, runCalls.get());
    }

    @Test
    public void compositesRejectSecondStartBeforePartiallyActiveChildrenSeeSideEffects() {
        ManualLoopClock manualClock = new ManualLoopClock();
        LoopClock clock = manualClock.clock();

        CountingIncompleteTask sequenceChild = new CountingIncompleteTask("sequenceChild");
        Task sequence = Tasks.sequence(sequenceChild);
        sequence.start(clock);
        assertEquals(1, sequenceChild.startCount);
        assertSingleUseStartRejected(sequence, clock, "Tasks.sequence");
        assertEquals(1, sequenceChild.startCount);

        CountingIncompleteTask parallelChild = new CountingIncompleteTask("parallelChild");
        Task parallel = Tasks.parallelAll(parallelChild);
        parallel.start(clock);
        assertEquals(1, parallelChild.startCount);
        assertSingleUseStartRejected(parallel, clock, "Tasks.parallelAll");
        assertEquals(1, parallelChild.startCount);

        CountingIncompleteTask move = new CountingIncompleteTask("move");
        Task branch = Tasks.branchOnOutcome(move, Tasks.noop(), Tasks.noop());
        branch.start(clock);
        assertEquals(1, move.startCount);
        assertSingleUseStartRejected(branch, clock, "BranchOnOutcome(move)");
        assertEquals(1, move.startCount);
    }

    @Test
    public void compositesRejectSecondStartAfterCompletion() {
        ManualLoopClock manualClock = new ManualLoopClock();
        LoopClock clock = manualClock.clock();

        Task sequence = Tasks.sequence(Tasks.noop(), Tasks.runOnce(() -> { }));
        sequence.start(clock);
        assertTrue(sequence.isComplete());
        assertSingleUseStartRejected(sequence, clock, "Tasks.sequence");

        Task parallel = Tasks.parallelAll(Tasks.noop(), Tasks.runOnce(() -> { }));
        parallel.start(clock);
        assertTrue(parallel.isComplete());
        assertSingleUseStartRejected(parallel, clock, "Tasks.parallelAll");

        Task branch = Tasks.branchOnOutcome(
                Tasks.noop(),
                Tasks.runOnce(() -> { }),
                Tasks.runOnce(() -> { }));
        branch.start(clock);
        branch.update(clock);
        branch.update(clock);
        assertTrue(branch.isComplete());
        assertSingleUseStartRejected(branch, clock, "BranchOnOutcome(Tasks.noop)");
    }

    @Test
    public void compositeStartAttemptIsConsumedBeforeChildStartThrows() {
        ManualLoopClock manualClock = new ManualLoopClock();
        LoopClock clock = manualClock.clock();

        ThrowingStartTask sequenceChild = new ThrowingStartTask("sequenceThrow");
        Task sequence = Tasks.sequence(sequenceChild);
        expectStartFailure(() -> sequence.start(clock));
        assertEquals(1, sequenceChild.startCount);
        assertSingleUseStartRejected(sequence, clock, "Tasks.sequence");
        assertEquals(1, sequenceChild.startCount);

        ThrowingStartTask parallelChild = new ThrowingStartTask("parallelThrow");
        Task parallel = Tasks.parallelAll(parallelChild);
        expectStartFailure(() -> parallel.start(clock));
        assertEquals(1, parallelChild.startCount);
        assertSingleUseStartRejected(parallel, clock, "Tasks.parallelAll");
        assertEquals(1, parallelChild.startCount);

        ThrowingStartTask move = new ThrowingStartTask("branchThrow");
        Task branch = Tasks.branchOnOutcome(move, Tasks.noop(), Tasks.runOnce(() -> { }));
        expectStartFailure(() -> branch.start(clock));
        assertEquals(1, move.startCount);
        assertSingleUseStartRejected(branch, clock, "BranchOnOutcome(branchThrow)");
        assertEquals(1, move.startCount);
    }

    @Test
    public void cancelBeforeStartIsNoOpAndDoesNotConsumeFirstStart() {
        ManualLoopClock manualClock = new ManualLoopClock();
        LoopClock clock = manualClock.clock();

        AtomicInteger instantActions = new AtomicInteger();
        Task instant = new InstantTask(instantActions::incrementAndGet);
        instant.cancel();
        assertFalse(instant.isComplete());
        instant.start(clock);
        assertEquals(1, instantActions.get());
        assertEquals(TaskOutcome.SUCCESS, instant.getOutcome());
        assertSingleUseStartRejected(instant, clock, "InstantTask");

        Task wait = new WaitUntilTask(BooleanSource.constant(false));
        wait.cancel();
        assertFalse(wait.isComplete());
        wait.start(clock);
        assertFalse(wait.isComplete());
        assertSingleUseStartRejected(wait, clock, "WaitUntilTask");

        AtomicInteger sequenceActions = new AtomicInteger();
        Task sequence = Tasks.sequence(Tasks.runOnce(sequenceActions::incrementAndGet));
        sequence.cancel();
        sequence.start(clock);
        assertEquals(1, sequenceActions.get());
        assertEquals(TaskOutcome.SUCCESS, sequence.getOutcome());
        assertSingleUseStartRejected(sequence, clock, "Tasks.sequence");

        AtomicInteger parallelActions = new AtomicInteger();
        Task parallel = Tasks.parallelAll(Tasks.runOnce(parallelActions::incrementAndGet));
        parallel.cancel();
        parallel.start(clock);
        assertEquals(1, parallelActions.get());
        assertEquals(TaskOutcome.SUCCESS, parallel.getOutcome());
        assertSingleUseStartRejected(parallel, clock, "Tasks.parallelAll");
    }

    @Test
    public void terminalUpdatesAndRepeatedCancellationRemainSafe() {
        ManualLoopClock manualClock = new ManualLoopClock();
        LoopClock clock = manualClock.clock();
        AtomicInteger finishCalls = new AtomicInteger();

        Task[] tasks = {
                new RunForSecondsTask(1.0, null, null, finishCalls::incrementAndGet),
                new WaitUntilTask(BooleanSource.constant(false)),
                new OutputForSecondsTask("cancelOutput", 0.5, 1.0),
                new GatedOutputUntilTask(
                        "cancelGate",
                        BooleanSource.constant(false),
                        BooleanSource.constant(false),
                        ScalarSource.constant(0.5),
                        0.0,
                        0.0,
                        1.0,
                        0.0),
                Tasks.sequence(new CountingIncompleteTask("sequenceCancelChild")),
                Tasks.parallelAll(new CountingIncompleteTask("parallelCancelChild")),
                Tasks.branchOnOutcome(
                        new CountingIncompleteTask("branchCancelMove"),
                        Tasks.noop(),
                        Tasks.runOnce(() -> { }))
        };

        for (Task task : tasks) {
            task.start(clock);
            task.cancel();
            task.cancel();
            task.update(clock);
            assertTrue(task.getDebugName(), task.isComplete());
            assertEquals(task.getDebugName(), TaskOutcome.CANCELLED, task.getOutcome());
        }
        assertEquals(1, finishCalls.get());
    }

    @Test
    public void compositeConstructionRejectsDirectDuplicateChildren() {
        Task shared = Tasks.noop();

        assertTrue(expectIllegalArgument(
                () -> Tasks.sequence((Task[]) null)).getMessage().contains("tasks"));
        assertTrue(expectIllegalArgument(
                () -> Tasks.sequence(Tasks.noop(), null)).getMessage().contains("index 1"));
        assertTrue(expectIllegalArgument(
                () -> Tasks.parallelAll((Task[]) null)).getMessage().contains("tasks"));
        assertTrue(expectIllegalArgument(
                () -> Tasks.parallelAll(Tasks.noop(), null)).getMessage().contains("index 1"));

        IllegalArgumentException sequenceError =
                expectIllegalArgument(() -> Tasks.sequence(shared, shared));
        assertTrue(sequenceError.getMessage().contains("Tasks.sequence"));
        assertTrue(sequenceError.getMessage().contains("index 0"));
        assertTrue(sequenceError.getMessage().contains("index 1"));

        IllegalArgumentException parallelError =
                expectIllegalArgument(() -> Tasks.parallelAll(shared, shared));
        assertTrue(parallelError.getMessage().contains("Tasks.parallelAll"));

        IllegalArgumentException branchError =
                expectIllegalArgument(() -> Tasks.branchOnOutcome(shared, shared, Tasks.noop()));
        assertTrue(branchError.getMessage().contains("move"));
        assertTrue(branchError.getMessage().contains("onSuccess"));
    }

    @Test
    public void compositionImplementationsRemainPackagePrivate() {
        assertFalse(Modifier.isPublic(SequenceTask.class.getModifiers()));
        assertFalse(Modifier.isPublic(ParallelAllTask.class.getModifiers()));
        assertFalse(Modifier.isPublic(ParallelDeadlineTask.class.getModifiers()));
    }

    @Test
    public void nestedCompositeAliasFailsAtLeafWithoutRepeatingLeafSideEffect() {
        ManualLoopClock manualClock = new ManualLoopClock();
        AtomicInteger leafCalls = new AtomicInteger();
        Task sharedLeaf = Tasks.runOnce(leafCalls::incrementAndGet);
        Task firstWrapper = Tasks.sequence(sharedLeaf);
        Task secondWrapper = Tasks.parallelAll(sharedLeaf);
        Task outer = Tasks.sequence(firstWrapper, secondWrapper);

        IllegalStateException error =
                expectIllegalState(() -> outer.start(manualClock.clock()));

        assertActionable(error, "InstantTask");
        assertEquals(1, leafCalls.get());
        assertSingleUseStartRejected(outer, manualClock.clock(), "Tasks.sequence");
        assertEquals(1, leafCalls.get());
    }

    @Test
    public void publicCompositionFactoriesCopyArraysAndAllowEmptyGroups() {
        ManualLoopClock manualClock = new ManualLoopClock();
        CountingIncompleteTask sequenceChild = new CountingIncompleteTask("sequenceChild");
        CountingIncompleteTask sequenceReplacement =
                new CountingIncompleteTask("sequenceReplacement");
        Task[] sequenceChildren = {sequenceChild};
        Task sequence = Tasks.sequence(sequenceChildren);
        sequenceChildren[0] = sequenceReplacement;

        sequence.start(manualClock.clock());
        assertEquals(1, sequenceChild.startCount);
        assertEquals(0, sequenceReplacement.startCount);
        sequence.cancel();

        CountingIncompleteTask parallelChild = new CountingIncompleteTask("parallelChild");
        CountingIncompleteTask parallelReplacement =
                new CountingIncompleteTask("parallelReplacement");
        Task[] parallelChildren = {parallelChild};
        Task parallel = Tasks.parallelAll(parallelChildren);
        parallelChildren[0] = parallelReplacement;

        parallel.start(manualClock.clock());
        assertEquals(1, parallelChild.startCount);
        assertEquals(0, parallelReplacement.startCount);
        parallel.cancel();

        Task emptySequence = Tasks.sequence();
        emptySequence.start(manualClock.clock());
        assertTrue(emptySequence.isComplete());
        assertEquals(TaskOutcome.SUCCESS, emptySequence.getOutcome());

        Task emptyParallel = Tasks.parallelAll();
        emptyParallel.start(manualClock.clock());
        assertTrue(emptyParallel.isComplete());
        assertEquals(TaskOutcome.SUCCESS, emptyParallel.getOutcome());
    }

    @Test
    public void supplierMacroBuildsFreshFactoryOnlyCompositionGraphs() {
        ManualLoopClock manualClock = new ManualLoopClock();
        AtomicInteger sequenceActions = new AtomicInteger();
        AtomicInteger companionActions = new AtomicInteger();
        Supplier<Task> macro = () -> Tasks.sequence(
                Tasks.runOnce(sequenceActions::incrementAndGet),
                Tasks.parallelDeadline(
                        Tasks.waitForSeconds(0.01),
                        Tasks.runOnce(companionActions::incrementAndGet)));

        Task first = macro.get();
        Task second = macro.get();

        assertNotSame(first, second);
        first.start(manualClock.clock());
        second.start(manualClock.clock());
        assertEquals(2, sequenceActions.get());
        assertEquals(2, companionActions.get());

        LoopClock completionClock = manualClock.nextCycle(0.02);
        first.update(completionClock);
        second.update(completionClock);

        assertTrue(first.isComplete());
        assertTrue(second.isComplete());
        assertEquals(TaskOutcome.SUCCESS, first.getOutcome());
        assertEquals(TaskOutcome.SUCCESS, second.getOutcome());
    }

    @Test
    public void factoryOnlyCompositionsExposePublicDebugNamesToTaskRunner() {
        Task[] compositions = {
                Tasks.sequence(new WaitUntilTask(BooleanSource.constant(false))),
                Tasks.parallelAll(new WaitUntilTask(BooleanSource.constant(false))),
                Tasks.parallelDeadline(
                        new WaitUntilTask(BooleanSource.constant(false)),
                        new WaitUntilTask(BooleanSource.constant(false)))
        };
        String[] publicNames = {
                "Tasks.sequence(...)",
                "Tasks.parallelAll(...)",
                "Tasks.parallelDeadline(...)"
        };
        String[] hiddenNames = {
                "SequenceTask",
                "ParallelAllTask",
                "ParallelDeadlineTask"
        };

        for (int i = 0; i < compositions.length; i++) {
            Task task = compositions[i];
            TaskRunner runner = new TaskRunner();
            assertEquals(publicNames[i], task.getDebugName());

            runner.enqueue(task);
            IllegalStateException duplicate = expectIllegalState(() -> runner.enqueue(task));

            assertTrue(duplicate.getMessage(), duplicate.getMessage().contains(publicNames[i]));
            assertFalse(duplicate.getMessage(), duplicate.getMessage().contains(hiddenNames[i]));
        }
    }

    @Test
    public void taskRunnerRejectsDuplicateQueuedCurrentAndCompletedIdentity() {
        ManualLoopClock manualClock = new ManualLoopClock();

        Task queued = new WaitUntilTask(BooleanSource.constant(false));
        TaskRunner queuedRunner = new TaskRunner();
        queuedRunner.enqueue(queued);
        IllegalStateException queuedError =
                expectIllegalState(() -> queuedRunner.enqueue(queued));
        assertActionable(queuedError, "WaitUntilTask");
        assertEquals(1, queuedRunner.queuedCount());

        Task current = new WaitUntilTask(BooleanSource.constant(false));
        TaskRunner currentRunner = new TaskRunner();
        currentRunner.enqueue(current);
        currentRunner.update(manualClock.clock());
        assertSame(current, currentRunner.currentTaskOrNull());
        IllegalStateException currentError =
                expectIllegalState(() -> currentRunner.enqueue(current));
        assertActionable(currentError, "WaitUntilTask");

        Task completed = new WaitUntilTask(BooleanSource.constant(true));
        TaskRunner completedRunner = new TaskRunner();
        completedRunner.enqueue(completed);
        completedRunner.update(manualClock.clock());
        assertTrue(completed.isComplete());
        assertSame(completed, completedRunner.currentTaskOrNull());
        IllegalStateException completedError =
                expectIllegalState(() -> completedRunner.enqueue(completed));
        assertActionable(completedError, "WaitUntilTask");
    }

    @Test
    public void taskRunnerKeepsNoPermanentIdentityHistoryButTaskStillRejectsReuse() {
        ManualLoopClock manualClock = new ManualLoopClock();
        TaskRunner runner = new TaskRunner();
        Task task = Tasks.runOnce(() -> { });

        runner.enqueue(task);
        runner.update(manualClock.clock());
        assertNull(runner.currentTaskOrNull());
        assertTrue(runner.isIdle());

        runner.enqueue(task);
        assertEquals(1, runner.queuedCount());
        manualClock.nextCycle(0.02);
        IllegalStateException error =
                expectIllegalState(() -> runner.update(manualClock.clock()));
        assertActionable(error, "InstantTask");
    }

    @Test
    public void sameTaskMayBeQueuedAcrossRunnersButSecondRunnerCannotStartIt() {
        ManualLoopClock manualClock = new ManualLoopClock();
        Task shared = new WaitUntilTask(BooleanSource.constant(false));
        TaskRunner first = new TaskRunner();
        TaskRunner second = new TaskRunner();

        first.enqueue(shared);
        second.enqueue(shared);
        first.update(manualClock.clock());
        assertSame(shared, first.currentTaskOrNull());

        IllegalStateException error =
                expectIllegalState(() -> second.update(manualClock.clock()));
        assertActionable(error, "WaitUntilTask");
    }

    @Test
    public void outputFactoryCreatesFreshTasksThatCanStartIndependently() {
        ManualLoopClock manualClock = new ManualLoopClock();
        OutputTaskFactory factory = Tasks.outputPulse("freshPulse")
                .startImmediately()
                .runOutput(0.75)
                .forSeconds(0.1)
                .build();

        OutputTask first = factory.create();
        OutputTask second = factory.create();
        assertNotSame(first, second);

        first.start(manualClock.clock());
        second.start(manualClock.clock());
        first.update(manualClock.clock());
        second.update(manualClock.clock());
        assertFalse(first.isComplete());
        assertFalse(second.isComplete());
    }

    @Test
    public void repeatedBacklogUsesFreshFactoryTasks() {
        ManualLoopClock manualClock = new ManualLoopClock();
        OutputTaskRunner runner = new OutputTaskRunner(0.0);
        AtomicInteger factoryCalls = new AtomicInteger();
        Supplier<OutputTask> factory = () -> new OutputForSecondsTask(
                "backlog-" + factoryCalls.incrementAndGet(),
                0.5,
                0.0);

        runner.ensureBacklog(manualClock.clock(), 1, factory);
        assertEquals(1, factoryCalls.get());
        runner.update(manualClock.clock());
        assertTrue(runner.isIdle());

        manualClock.nextCycle(0.02);
        runner.ensureBacklog(manualClock.clock(), 1, factory);
        assertEquals(2, factoryCalls.get());
        runner.update(manualClock.clock());
        assertTrue(runner.isIdle());
    }

    @Test
    public void backlogRejectsFactoryThatReturnsSameQueuedTask() {
        ManualLoopClock manualClock = new ManualLoopClock();
        OutputTaskRunner runner = new OutputTaskRunner(0.0);
        OutputTask shared = new OutputForSecondsTask("sharedBacklog", 0.5, 1.0);

        IllegalStateException error = expectIllegalState(
                () -> runner.ensureBacklog(manualClock.clock(), 2, () -> shared));

        assertActionable(error, "sharedBacklog");
        assertEquals(1, runner.queuedCount());
        assertEquals(1, runner.backlogCount());
    }

    private static void assertActiveThenRejects(Task task,
                                                LoopClock clock,
                                                String expectedName) {
        task.start(clock);
        assertFalse(expectedName, task.isComplete());
        assertSingleUseStartRejected(task, clock, expectedName);
    }

    private static void assertSingleUseStartRejected(Task task,
                                                     LoopClock clock,
                                                     String expectedName) {
        IllegalStateException error = expectIllegalState(() -> task.start(clock));
        assertActionable(error, expectedName);
    }

    private static void assertActionable(IllegalStateException error, String expectedName) {
        assertTrue(error.getMessage(), error.getMessage().contains(expectedName));
        assertTrue(error.getMessage(), error.getMessage().contains("fresh task"));
        assertTrue(error.getMessage(), error.getMessage().contains("Supplier<Task>"));
        assertTrue(error.getMessage(), error.getMessage().contains("OutputTaskFactory"));
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

    private static IllegalArgumentException expectIllegalArgument(Runnable action) {
        try {
            action.run();
            fail("Expected IllegalArgumentException");
            return null;
        } catch (IllegalArgumentException expected) {
            return expected;
        }
    }

    private static void expectStartFailure(Runnable action) {
        try {
            action.run();
            fail("Expected test start failure");
        } catch (ExpectedStartFailure expected) {
            // Expected test fixture failure.
        }
    }

    /** Incomplete fixture used to observe composite child lifecycle calls. */
    private static final class CountingIncompleteTask implements Task {
        private final String name;
        private int startCount;
        private boolean cancelled;

        private CountingIncompleteTask(String name) {
            this.name = name;
        }

        @Override
        public void start(LoopClock clock) {
            startCount++;
        }

        @Override
        public void update(LoopClock clock) {
            // Remain active until cancelled.
        }

        @Override
        public void cancel() {
            cancelled = true;
        }

        @Override
        public boolean isComplete() {
            return cancelled;
        }

        @Override
        public TaskOutcome getOutcome() {
            return cancelled ? TaskOutcome.CANCELLED : TaskOutcome.NOT_DONE;
        }

        @Override
        public String getDebugName() {
            return name;
        }
    }

    /** Fixture whose first child start side effect deliberately fails. */
    private static final class ThrowingStartTask implements Task {
        private final String name;
        private int startCount;

        private ThrowingStartTask(String name) {
            this.name = name;
        }

        @Override
        public void start(LoopClock clock) {
            startCount++;
            throw new ExpectedStartFailure();
        }

        @Override
        public void update(LoopClock clock) {
            // Never reached in these tests.
        }

        @Override
        public boolean isComplete() {
            return false;
        }

        @Override
        public TaskOutcome getOutcome() {
            return TaskOutcome.NOT_DONE;
        }

        @Override
        public String getDebugName() {
            return name;
        }
    }

    /** Distinct unchecked exception used only to exercise failed first-start attempts. */
    private static final class ExpectedStartFailure extends RuntimeException {
    }
}
