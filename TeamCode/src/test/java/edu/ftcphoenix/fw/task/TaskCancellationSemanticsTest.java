package edu.ftcphoenix.fw.task;

import org.junit.Test;

import java.util.concurrent.atomic.AtomicInteger;

import edu.ftcphoenix.fw.actuation.ScalarTasks;
import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.source.ScalarTarget;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the core Task and runner cancellation/failure lifecycle contract. */
public final class TaskCancellationSemanticsTest {

    private static final double EPSILON = 1e-12;
    private static final DebugSink NOOP_DEBUG_SINK = new DebugSink() {
        @Override
        public DebugSink addData(String key, Object value) {
            return this;
        }

        @Override
        public DebugSink addLine(String text) {
            return this;
        }
    };

    private static final class ActiveCapturingDebugSink implements DebugSink {
        private Object activeValue;

        @Override
        public DebugSink addData(String key, Object value) {
            if ("active.active".equals(key)) {
                activeValue = value;
            }
            return this;
        }

        @Override
        public DebugSink addLine(String text) {
            return this;
        }
    }

    @Test
    public void builtInsIgnorePreStartCancelAndRejectPreStartUpdate() {
        ManualLoopClock manualClock = new ManualLoopClock();
        LoopClock clock = manualClock.clock();
        Task[] tasks = {
                new InstantTask(() -> { }),
                new RunForSecondsTask(1.0, null, null, null),
                new WaitUntilTask(BooleanSource.constant(false)),
                new OutputForSecondsTask("directOutput", 0.5, 1.0),
                new GatedOutputUntilTask(
                        "gatedOutput",
                        BooleanSource.constant(false),
                        BooleanSource.constant(false),
                        ScalarSource.constant(0.5),
                        0.0,
                        0.0,
                        1.0,
                        0.0),
                Tasks.sequence(new WaitUntilTask(BooleanSource.constant(false))),
                Tasks.parallelAll(new WaitUntilTask(BooleanSource.constant(false))),
                Tasks.parallelDeadline(
                        new WaitUntilTask(BooleanSource.constant(false)),
                        new WaitUntilTask(BooleanSource.constant(false))),
                Tasks.branchOnOutcome(
                        new WaitUntilTask(BooleanSource.constant(false)),
                        Tasks.noop(),
                        Tasks.noop())
        };
        String[] updateFailureLabels = {
                "InstantTask",
                "RunForSecondsTask",
                "WaitUntilTask",
                "directOutput",
                "gatedOutput",
                "Tasks.sequence(...)",
                "Tasks.parallelAll(...)",
                "Tasks.parallelDeadline(...)",
                "BranchOnOutcome"
        };

        for (int i = 0; i < tasks.length; i++) {
            Task task = tasks[i];
            task.cancel();
            assertFalse(task.getDebugName(), task.isComplete());

            IllegalStateException failure = expectIllegalState(() -> task.update(clock));
            assertTrue(
                    failure.getMessage(),
                    failure.getMessage().contains(updateFailureLabels[i]));
            assertTrue(failure.getMessage(), failure.getMessage().contains("before start"));
            assertTrue(failure.getMessage(), failure.getMessage().contains("TaskRunner"));

            task.start(clock);
            if (!task.isComplete()) {
                task.cancel();
            }
        }
    }

    @Test
    public void noopRemainsAnAlwaysSuccessfulDirectUpdateSpecialCase() {
        ManualLoopClock manualClock = new ManualLoopClock();
        Task noop = Tasks.noop();

        noop.cancel();
        noop.update(manualClock.clock());

        assertTrue(noop.isComplete());
        assertEquals(TaskOutcome.SUCCESS, noop.getOutcome());
        noop.start(manualClock.clock());
    }

    @Test
    public void activeCancelIsTerminalBeforeCleanupAndRepeatedCancelIsNoOp() {
        ManualLoopClock manualClock = new ManualLoopClock();
        AtomicInteger finishCalls = new AtomicInteger();
        RunForSecondsTask timed = new RunForSecondsTask(
                1.0,
                null,
                null,
                finishCalls::incrementAndGet);

        timed.start(manualClock.clock());
        timed.cancel();
        timed.cancel();
        timed.update(manualClock.clock());

        assertTrue(timed.isComplete());
        assertEquals(TaskOutcome.CANCELLED, timed.getOutcome());
        assertEquals(1, finishCalls.get());

        RecordingTask child = new RecordingTask("sequenceChild");
        Task sequence = Tasks.sequence(child);
        sequence.start(manualClock.clock());
        sequence.cancel();
        sequence.cancel();

        assertTrue(sequence.isComplete());
        assertEquals(TaskOutcome.CANCELLED, sequence.getOutcome());
        assertEquals(1, child.cancelCount);
    }

    @Test
    public void terminalCancelDoesNotReplaceSuccessfulOutcomeOrRepeatCleanup() {
        ManualLoopClock manualClock = new ManualLoopClock();
        AtomicInteger finishCalls = new AtomicInteger();
        RunForSecondsTask task = new RunForSecondsTask(
                0.0,
                null,
                null,
                finishCalls::incrementAndGet);

        task.start(manualClock.clock());
        task.cancel();
        task.cancel();
        task.update(manualClock.clock());

        assertEquals(TaskOutcome.SUCCESS, task.getOutcome());
        assertEquals(1, finishCalls.get());
    }

    @Test
    public void cancelCurrentCancelsOnlyCurrentAndLeavesFollowUpQueued() {
        ManualLoopClock manualClock = new ManualLoopClock();
        TaskRunner runner = new TaskRunner();
        RecordingTask active = new RecordingTask("active");
        RecordingTask followUp = new RecordingTask("followUp");
        runner.enqueue(active);
        runner.enqueue(followUp);
        runner.update(manualClock.clock());

        assertTrue(runner.cancelCurrent());

        assertEquals(1, active.cancelCount);
        assertEquals(0, followUp.cancelCount);
        assertEquals(1, runner.queuedCount());
        assertFalse(runner.hasActiveTask());

        runner.update(manualClock.clock());
        assertEquals(1, followUp.startCount);
    }

    @Test
    public void cancelAndClearCancelsActiveButNeverTouchesQueuedPreStartTask() {
        ManualLoopClock manualClock = new ManualLoopClock();
        TaskRunner runner = new TaskRunner();
        RecordingTask active = new RecordingTask("active");
        RecordingTask queued = new RecordingTask("queued");
        runner.enqueue(active);
        runner.enqueue(queued);
        runner.update(manualClock.clock());

        runner.cancelAndClear();

        assertTrue(runner.isIdle());
        assertEquals(1, active.cancelCount);
        assertEquals(0, queued.startCount);
        assertEquals(0, queued.cancelCount);
    }

    @Test
    public void cancelAndClearEndsEmptyWhenCancelThrows() {
        ManualLoopClock manualClock = new ManualLoopClock();
        TaskRunner runner = new TaskRunner();
        RuntimeException cancelFailure = new RuntimeException("cancel failed");
        ThrowingCancelTask active = new ThrowingCancelTask("active", cancelFailure);
        RecordingTask queued = new RecordingTask("queued");
        runner.enqueue(active);
        runner.enqueue(queued);
        runner.update(manualClock.clock());

        RuntimeException observed = expectRuntime(runner::cancelAndClear);

        assertSame(cancelFailure, observed);
        assertTrue(runner.isIdle());
        assertEquals(0, queued.startCount);
        assertEquals(0, queued.cancelCount);
    }

    @Test
    public void cancelAndClearDiscardsWorkEnqueuedReentrantlyByCancelHook() {
        ManualLoopClock manualClock = new ManualLoopClock();
        TaskRunner runner = new TaskRunner();
        RecordingTask reentrant = new RecordingTask("reentrant");
        ReentrantCancelTask active = new ReentrantCancelTask(
                runner,
                manualClock.clock(),
                reentrant);
        runner.enqueue(active);
        runner.update(manualClock.clock());

        runner.cancelAndClear();

        assertTrue(runner.isIdle());
        assertEquals(1, active.cancelCount);
        assertEquals(0, reentrant.startCount);
        assertEquals(0, reentrant.cancelCount);
    }

    @Test
    public void failedCancelCurrentAlsoDiscardsFollowUpWork() {
        ManualLoopClock manualClock = new ManualLoopClock();
        TaskRunner runner = new TaskRunner();
        RuntimeException cancelFailure = new RuntimeException("cancel failed");
        runner.enqueue(new ThrowingCancelTask("active", cancelFailure));
        runner.enqueue(new RecordingTask("mustNotRun"));
        runner.update(manualClock.clock());

        RuntimeException observed = expectRuntime(runner::cancelCurrent);

        assertSame(cancelFailure, observed);
        assertTrue(runner.isIdle());
    }

    @Test
    public void cancelCurrentSkipsATerminalCurrentTaskAndKeepsItsFollowUp() {
        ManualLoopClock manualClock = new ManualLoopClock();
        TaskRunner runner = new TaskRunner();
        RecordingTask completed = new RecordingTask("completed");
        RecordingTask followUp = new RecordingTask("followUp");
        runner.enqueue(completed);
        runner.enqueue(followUp);
        runner.update(manualClock.clock());
        completed.complete = true;

        assertFalse(runner.cancelCurrent());

        assertEquals(0, completed.cancelCount);
        assertEquals(1, runner.queuedCount());
        runner.update(manualClock.clock());
        assertEquals(1, followUp.startCount);
    }

    @Test
    public void lifecycleFailuresFailStopAndRunnerCanBeReusedInSameCycle() {
        for (FailurePoint point : FailurePoint.values()) {
            ManualLoopClock manualClock = new ManualLoopClock();
            TaskRunner runner = new TaskRunner();
            RuntimeException lifecycleFailure = new RuntimeException(point.name());
            LifecycleFailureTask failing = new LifecycleFailureTask(point, lifecycleFailure, null);
            RecordingTask queued = new RecordingTask("queued");
            runner.enqueue(failing);
            runner.enqueue(queued);

            RuntimeException observed = expectRuntime(() -> runner.update(manualClock.clock()));

            assertSame(point.name(), lifecycleFailure, observed);
            assertTrue(point.name(), runner.isIdle());
            assertEquals(point.name(), 1, failing.cancelCount);
            assertEquals(point.name(), 0, queued.startCount);
            assertEquals(point.name(), 0, queued.cancelCount);

            AtomicInteger replacementRuns = new AtomicInteger();
            runner.enqueue(Tasks.runOnce(replacementRuns::incrementAndGet));
            runner.update(manualClock.clock());
            assertEquals(point.name(), 1, replacementRuns.get());
        }
    }

    @Test
    public void lifecycleFailurePreservesOriginalAndSuppressesCleanupFailure() {
        ManualLoopClock manualClock = new ManualLoopClock();
        TaskRunner runner = new TaskRunner();
        RuntimeException lifecycleFailure = new RuntimeException("start failed");
        RuntimeException cleanupFailure = new RuntimeException("cleanup failed");
        runner.enqueue(new LifecycleFailureTask(
                FailurePoint.START,
                lifecycleFailure,
                cleanupFailure));

        RuntimeException observed = expectRuntime(() -> runner.update(manualClock.clock()));

        assertSame(lifecycleFailure, observed);
        assertEquals(1, observed.getSuppressed().length);
        assertSame(cleanupFailure, observed.getSuppressed()[0]);
        assertTrue(runner.isIdle());
    }

    @Test
    public void failStopDiscardsWorkEnqueuedReentrantlyByCleanup() {
        ManualLoopClock manualClock = new ManualLoopClock();
        TaskRunner runner = new TaskRunner();
        RecordingTask reentrant = new RecordingTask("reentrant");
        RuntimeException lifecycleFailure = new RuntimeException("start failed");
        ReentrantCleanupFailureTask failing = new ReentrantCleanupFailureTask(
                runner,
                manualClock.clock(),
                reentrant,
                lifecycleFailure);
        runner.enqueue(failing);

        RuntimeException observed = expectRuntime(() -> runner.update(manualClock.clock()));

        assertSame(lifecycleFailure, observed);
        assertEquals(1, failing.cancelCount);
        assertEquals(0, reentrant.startCount);
        assertEquals(0, reentrant.cancelCount);
        assertTrue(runner.isIdle());
    }

    @Test
    public void lifecycleCallbackCannotStartAReplacementBeforeItsFailureIsCleanedUp() {
        ManualLoopClock manualClock = new ManualLoopClock();
        TaskRunner runner = new TaskRunner();
        RecordingTask replacement = new RecordingTask("replacement");
        RuntimeException lifecycleFailure = new RuntimeException("update failed after abort");
        SelfAbortingFailureTask failing = new SelfAbortingFailureTask(
                runner,
                manualClock.clock(),
                lifecycleFailure);
        runner.enqueue(failing);
        runner.enqueue(replacement);

        RuntimeException observed = expectRuntime(() -> runner.update(manualClock.clock()));

        assertSame(lifecycleFailure, observed);
        assertEquals(1, failing.cancelCount);
        assertEquals(0, replacement.startCount);
        assertEquals(0, replacement.cancelCount);
        assertTrue(runner.isIdle());
    }

    @Test
    public void compositeCleanupDoesNotRepeatAFailedChildCompletionQuery() {
        ManualLoopClock manualClock = new ManualLoopClock();

        LifecycleFailureTask sequenceChild = completionFailureTask("sequence child completion");
        assertCompositeFailureCancelsChild(
                Tasks.sequence(sequenceChild), sequenceChild, manualClock.clock());

        LifecycleFailureTask parallelChild = completionFailureTask("parallel child completion");
        assertCompositeFailureCancelsChild(
                Tasks.parallelAll(parallelChild), parallelChild, manualClock.clock());

        LifecycleFailureTask branchChild = completionFailureTask("branch child completion");
        assertCompositeFailureCancelsChild(
                Tasks.branchOnOutcome(branchChild, Tasks.noop(), Tasks.noop()),
                branchChild,
                manualClock.clock());
    }

    @Test
    public void parallelCancellationAttemptsEveryChildAndSuppressesLaterFailures() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RuntimeException firstFailure = new RuntimeException("first cleanup");
        RuntimeException secondFailure = new RuntimeException("second cleanup");
        ThrowingCancelTask first = new ThrowingCancelTask("first", firstFailure);
        ThrowingCancelTask second = new ThrowingCancelTask("second", secondFailure);
        RecordingTask third = new RecordingTask("third");
        Task parallel = Tasks.parallelAll(first, second, third);
        parallel.start(manualClock.clock());

        RuntimeException observed = expectRuntime(parallel::cancel);

        assertSame(firstFailure, observed);
        assertEquals(1, observed.getSuppressed().length);
        assertSame(secondFailure, observed.getSuppressed()[0]);
        assertEquals(1, first.cancelCount);
        assertEquals(1, second.cancelCount);
        assertEquals(1, third.cancelCount);
        assertTrue(parallel.isComplete());
        assertEquals(TaskOutcome.CANCELLED, parallel.getOutcome());
        parallel.cancel();
        assertEquals(1, third.cancelCount);
    }

    @Test
    public void reentrantStartCancellationStaysTerminalAndSkipsLaterChildren() {
        ManualLoopClock manualClock = new ManualLoopClock();

        TaskRunner parallelRunner = new TaskRunner();
        RecordingTask laterChild = new RecordingTask("laterChild");
        Task parallel = Tasks.parallelAll(
                Tasks.runOnce(parallelRunner::cancelCurrent),
                laterChild);
        parallelRunner.enqueue(parallel);
        parallelRunner.update(manualClock.clock());
        assertTrue(parallelRunner.isIdle());
        assertTrue(parallel.isComplete());
        assertEquals(TaskOutcome.CANCELLED, parallel.getOutcome());
        assertEquals(0, laterChild.startCount);

        TaskRunner branchRunner = new TaskRunner();
        Task selected = Tasks.runOnce(branchRunner::cancelCurrent);
        Task branch = Tasks.branchOnOutcome(Tasks.noop(), selected, Tasks.noop());
        branchRunner.enqueue(branch);
        branchRunner.update(manualClock.clock());
        assertTrue(branchRunner.isIdle());
        assertTrue(branch.isComplete());
        assertEquals(TaskOutcome.CANCELLED, branch.getOutcome());
        assertEquals(TaskOutcome.CANCELLED, selected.getOutcome());

        TaskRunner scalarRunner = new TaskRunner();
        CancellingScalarTarget target = new CancellingScalarTarget(scalarRunner);
        Task scalarSet = ScalarTasks.set(target, 1.0);
        scalarRunner.enqueue(scalarSet);
        scalarRunner.update(manualClock.clock());
        assertTrue(scalarRunner.isIdle());
        assertEquals(1, target.setCount);
        assertEquals(TaskOutcome.CANCELLED, scalarSet.getOutcome());
    }

    @Test
    public void reentrantGateCancellationCannotReopenAnOutputTask() {
        ManualLoopClock manualClock = new ManualLoopClock();
        TaskRunner runner = new TaskRunner();
        BooleanSource cancellingGate = clock -> {
            runner.cancelCurrent();
            return true;
        };
        OutputTask output = new GatedOutputUntilTask(
                "reentrantGate",
                cancellingGate,
                BooleanSource.constant(false),
                ScalarSource.constant(0.75),
                -0.25,
                0.1,
                1.0,
                0.0);
        runner.enqueue(output);

        runner.update(manualClock.clock());

        assertTrue(runner.isIdle());
        assertTrue(output.isComplete());
        assertEquals(TaskOutcome.CANCELLED, output.getOutcome());
        assertEquals(-0.25, output.getOutput(), EPSILON);
    }

    @Test
    public void throwingTimedCleanupLeavesTaskTerminalAndRunsOnce() {
        ManualLoopClock manualClock = new ManualLoopClock();
        AtomicInteger finishCalls = new AtomicInteger();
        RuntimeException cleanupFailure = new RuntimeException("finish failed");
        Task timed = new RunForSecondsTask(
                1.0,
                null,
                null,
                () -> {
                    finishCalls.incrementAndGet();
                    throw cleanupFailure;
                });
        timed.start(manualClock.clock());

        RuntimeException observed = expectRuntime(timed::cancel);

        assertSame(cleanupFailure, observed);
        assertTrue(timed.isComplete());
        assertEquals(TaskOutcome.CANCELLED, timed.getOutcome());
        timed.cancel();
        assertEquals(1, finishCalls.get());
    }

    @Test
    public void outputAbortInvalidatesAlreadySampledValueInSameCycle() {
        ManualLoopClock manualClock = new ManualLoopClock();
        OutputTaskRunner runner = new OutputTaskRunner(-0.25);
        runner.enqueue(new OutputForSecondsTask("active", 0.75, 1.0));
        BooleanSource active = runner.activeSource();
        runner.update(manualClock.clock());
        assertEquals(0.75, runner.output(manualClock.clock()), EPSILON);
        assertTrue(active.getAsBoolean(manualClock.clock()));

        runner.cancelAndClear();

        assertTrue(runner.isIdle());
        assertEquals(-0.25, runner.output(manualClock.clock()), EPSILON);
        ActiveCapturingDebugSink debug = new ActiveCapturingDebugSink();
        active.debugDump(debug, "active");
        assertEquals(Boolean.FALSE, debug.activeValue);
        assertFalse(active.getAsBoolean(manualClock.clock()));
    }

    @Test
    public void outputCancelCurrentInvalidatesSameCycleOutputAndActiveSource() {
        ManualLoopClock manualClock = new ManualLoopClock();
        OutputTaskRunner runner = new OutputTaskRunner(-0.25);
        runner.enqueue(new OutputForSecondsTask("active", 0.75, 1.0));
        BooleanSource active = runner.activeSource();
        assertEquals(0.75, runner.getAsDouble(manualClock.clock()), EPSILON);
        assertTrue(active.getAsBoolean(manualClock.clock()));

        assertTrue(runner.cancelCurrent());

        assertTrue(runner.isIdle());
        assertEquals(-0.25, runner.output(manualClock.clock()), EPSILON);
        assertFalse(active.getAsBoolean(manualClock.clock()));
    }

    @Test
    public void outputLifecycleFailureInvalidatesValueSampledBeforeFailedUpdate() {
        ManualLoopClock manualClock = new ManualLoopClock();
        OutputTaskRunner runner = new OutputTaskRunner(-0.25);
        RuntimeException updateFailure = new RuntimeException("output update failed");
        FailingOutputTask task = new FailingOutputTask(updateFailure, null);
        runner.enqueue(task);
        runner.update(manualClock.clock());
        assertEquals(0.75, runner.output(manualClock.clock()), EPSILON);

        manualClock.nextCycle(0.02);
        assertEquals(0.75, runner.output(manualClock.clock()), EPSILON);
        RuntimeException observed = expectRuntime(() -> runner.update(manualClock.clock()));

        assertSame(updateFailure, observed);
        assertTrue(runner.isIdle());
        assertEquals(-0.25, runner.output(manualClock.clock()), EPSILON);
    }

    @Test
    public void activeSourceCanRetrySameCycleAfterLifecycleFailure() {
        ManualLoopClock manualClock = new ManualLoopClock();
        OutputTaskRunner runner = new OutputTaskRunner(0.0);
        RuntimeException updateFailure = new RuntimeException("output update failed");
        runner.enqueue(new FailingOutputTask(updateFailure, null));
        BooleanSource active = runner.activeSource();
        assertTrue(active.getAsBoolean(manualClock.clock()));

        manualClock.nextCycle(0.02);
        RuntimeException observed =
                expectRuntime(() -> active.getAsBoolean(manualClock.clock()));

        assertSame(updateFailure, observed);
        assertFalse(active.getAsBoolean(manualClock.clock()));
        assertTrue(runner.isIdle());
    }

    @Test
    public void outputCancelFailureStillLeavesQueueEmptyAndOutputIdle() {
        ManualLoopClock manualClock = new ManualLoopClock();
        OutputTaskRunner runner = new OutputTaskRunner(-0.25);
        RuntimeException cancelFailure = new RuntimeException("output cancel failed");
        FailingOutputTask task = new FailingOutputTask(null, cancelFailure);
        runner.enqueue(task);
        runner.update(manualClock.clock());
        assertEquals(0.75, runner.output(manualClock.clock()), EPSILON);

        RuntimeException observed = expectRuntime(runner::cancelAndClear);

        assertSame(cancelFailure, observed);
        assertTrue(runner.isIdle());
        assertEquals(-0.25, runner.output(manualClock.clock()), EPSILON);
    }

    @Test
    public void failedOutputCancelCurrentClearsFollowUpsAndEverySameCycleCache() {
        ManualLoopClock manualClock = new ManualLoopClock();
        OutputTaskRunner runner = new OutputTaskRunner(-0.25);
        RuntimeException cancelFailure = new RuntimeException("output cancel failed");
        runner.enqueue(new FailingOutputTask(null, cancelFailure));
        runner.enqueue(new OutputForSecondsTask("mustNotRun", 1.0, 1.0));
        BooleanSource active = runner.activeSource();
        assertEquals(0.75, runner.getAsDouble(manualClock.clock()), EPSILON);
        assertTrue(active.getAsBoolean(manualClock.clock()));

        RuntimeException observed = expectRuntime(runner::cancelCurrent);

        assertSame(cancelFailure, observed);
        assertTrue(runner.isIdle());
        assertEquals(-0.25, runner.output(manualClock.clock()), EPSILON);
        assertFalse(active.getAsBoolean(manualClock.clock()));
    }

    @Test
    public void failedActiveSourceResetClearsItsLocalMemoInTheSameCycle() {
        ManualLoopClock manualClock = new ManualLoopClock();
        OutputTaskRunner runner = new OutputTaskRunner(0.0);
        RuntimeException cancelFailure = new RuntimeException("reset cancel failed");
        runner.enqueue(new FailingOutputTask(null, cancelFailure));
        BooleanSource active = runner.activeSource();
        assertTrue(active.getAsBoolean(manualClock.clock()));

        RuntimeException observed = expectRuntime(active::reset);

        assertSame(cancelFailure, observed);
        assertTrue(runner.isIdle());
        assertFalse(active.getAsBoolean(manualClock.clock()));
    }

    @Test
    public void debugDumpReportsLifecycleFailureAfterFailStopWithoutThrowing() {
        ManualLoopClock manualClock = new ManualLoopClock();
        TaskRunner runner = new TaskRunner();
        ToggleCompletionFailureTask failing = new ToggleCompletionFailureTask("debugFailure");
        RecordingTask queued = new RecordingTask("queued");
        runner.enqueue(failing);
        runner.enqueue(queued);
        runner.update(manualClock.clock());
        failing.failCompletionQuery = true;

        runner.debugDump(NOOP_DEBUG_SINK, "tasks");

        assertTrue(runner.isIdle());
        assertEquals(1, failing.cancelCount);
        assertEquals(0, queued.startCount);

        OutputTaskRunner outputRunner = new OutputTaskRunner(-0.25);
        ToggleCompletionFailureOutputTask failingOutput =
                new ToggleCompletionFailureOutputTask();
        outputRunner.enqueue(failingOutput);
        outputRunner.update(manualClock.clock());
        assertEquals(0.75, outputRunner.output(manualClock.clock()), EPSILON);
        failingOutput.failCompletionQuery = true;

        outputRunner.debugDump(NOOP_DEBUG_SINK, "output");

        assertTrue(outputRunner.isIdle());
        assertEquals(1, failingOutput.cancelCount);
        assertEquals(-0.25, outputRunner.output(manualClock.clock()), EPSILON);
    }

    private enum FailurePoint {
        START,
        UPDATE,
        IS_COMPLETE
    }

    private static LifecycleFailureTask completionFailureTask(String message) {
        return new LifecycleFailureTask(
                FailurePoint.IS_COMPLETE,
                new RuntimeException(message),
                null);
    }

    private static void assertCompositeFailureCancelsChild(Task composite,
                                                           LifecycleFailureTask child,
                                                           LoopClock clock) {
        TaskRunner runner = new TaskRunner();
        runner.enqueue(composite);

        RuntimeException observed = expectRuntime(() -> runner.update(clock));

        assertSame(child.lifecycleFailure, observed);
        assertEquals(1, child.cancelCount);
        assertTrue(runner.isIdle());
    }

    /** Incomplete task with observable lifecycle calls. */
    private static class RecordingTask implements Task {
        private final String name;
        private int startCount;
        private int updateCount;
        int cancelCount;
        private boolean started;
        private boolean complete;

        private RecordingTask(String name) {
            this.name = name;
        }

        @Override
        public void start(LoopClock clock) {
            started = true;
            startCount++;
        }

        @Override
        public void update(LoopClock clock) {
            if (!started) {
                throw new AssertionError("fixture updated before start");
            }
            updateCount++;
        }

        @Override
        public void cancel() {
            if (!started || complete) {
                return;
            }
            complete = true;
            cancelCount++;
        }

        @Override
        public boolean isComplete() {
            return complete;
        }

        @Override
        public TaskOutcome getOutcome() {
            return complete ? TaskOutcome.CANCELLED : TaskOutcome.NOT_DONE;
        }

        @Override
        public String getDebugName() {
            return name;
        }
    }

    /** Active task whose cancellation hook fails after establishing terminal state. */
    private static final class ThrowingCancelTask extends RecordingTask {
        private final RuntimeException failure;

        private ThrowingCancelTask(String name, RuntimeException failure) {
            super(name);
            this.failure = failure;
        }

        @Override
        public void cancel() {
            super.cancel();
            throw failure;
        }
    }

    /** Cancellation fixture that tries to enqueue and start work on the runner being aborted. */
    private static final class ReentrantCancelTask extends RecordingTask {
        private final TaskRunner runner;
        private final LoopClock clock;
        private final Task reentrantTask;

        private ReentrantCancelTask(TaskRunner runner, LoopClock clock, Task reentrantTask) {
            super("reentrantCancel");
            this.runner = runner;
            this.clock = clock;
            this.reentrantTask = reentrantTask;
        }

        @Override
        public void cancel() {
            super.cancel();
            runner.enqueue(reentrantTask);
            runner.update(clock);
            runner.cancelAndClear();
        }
    }

    /** Failed-start cleanup that tries to create and advance replacement work. */
    private static final class ReentrantCleanupFailureTask implements Task {
        private final TaskRunner runner;
        private final LoopClock clock;
        private final Task reentrantTask;
        private final RuntimeException failure;
        private boolean started;
        private boolean complete;
        private int cancelCount;

        private ReentrantCleanupFailureTask(TaskRunner runner,
                                            LoopClock clock,
                                            Task reentrantTask,
                                            RuntimeException failure) {
            this.runner = runner;
            this.clock = clock;
            this.reentrantTask = reentrantTask;
            this.failure = failure;
        }

        @Override
        public void start(LoopClock ignored) {
            started = true;
            throw failure;
        }

        @Override
        public void update(LoopClock ignored) {
        }

        @Override
        public void cancel() {
            if (!started || complete) {
                return;
            }
            complete = true;
            cancelCount++;
            runner.enqueue(reentrantTask);
            runner.update(clock);
        }

        @Override
        public boolean isComplete() {
            return complete;
        }

        @Override
        public TaskOutcome getOutcome() {
            return complete ? TaskOutcome.CANCELLED : TaskOutcome.NOT_DONE;
        }
    }

    /** Active task that detaches itself, tries a nested update, and then fails its outer update. */
    private static final class SelfAbortingFailureTask implements Task {
        private final TaskRunner runner;
        private final LoopClock clock;
        private final RuntimeException failure;
        private boolean started;
        private boolean complete;
        private int cancelCount;

        private SelfAbortingFailureTask(TaskRunner runner,
                                        LoopClock clock,
                                        RuntimeException failure) {
            this.runner = runner;
            this.clock = clock;
            this.failure = failure;
        }

        @Override
        public void start(LoopClock ignored) {
            started = true;
        }

        @Override
        public void update(LoopClock ignored) {
            runner.cancelCurrent();
            runner.update(clock);
            throw failure;
        }

        @Override
        public void cancel() {
            if (!started || complete) {
                return;
            }
            complete = true;
            cancelCount++;
        }

        @Override
        public boolean isComplete() {
            return complete;
        }

        @Override
        public TaskOutcome getOutcome() {
            return complete ? TaskOutcome.CANCELLED : TaskOutcome.NOT_DONE;
        }
    }

    /** Scalar target that cancels the Task currently writing it. */
    private static final class CancellingScalarTarget implements ScalarTarget {
        private final TaskRunner runner;
        private int setCount;
        private double value;

        private CancellingScalarTarget(TaskRunner runner) {
            this.runner = runner;
        }

        @Override
        public void set(double value) {
            setCount++;
            this.value = value;
            runner.cancelCurrent();
        }

        @Override
        public double get() {
            return value;
        }
    }

    /** Active Task whose completion query can be switched to a failure after startup. */
    private static final class ToggleCompletionFailureTask implements Task {
        private final String name;
        private boolean started;
        private boolean complete;
        private boolean failCompletionQuery;
        private int cancelCount;

        private ToggleCompletionFailureTask(String name) {
            this.name = name;
        }

        @Override
        public void start(LoopClock clock) {
            started = true;
        }

        @Override
        public void update(LoopClock clock) {
        }

        @Override
        public void cancel() {
            if (!started || complete) {
                return;
            }
            complete = true;
            cancelCount++;
        }

        @Override
        public boolean isComplete() {
            if (failCompletionQuery) {
                throw new RuntimeException("debug completion failed");
            }
            return complete;
        }

        @Override
        public TaskOutcome getOutcome() {
            return complete ? TaskOutcome.CANCELLED : TaskOutcome.NOT_DONE;
        }

        @Override
        public String getDebugName() {
            return name;
        }
    }

    /** Output equivalent of ToggleCompletionFailureTask for queue diagnostics. */
    private static final class ToggleCompletionFailureOutputTask implements OutputTask {
        private boolean started;
        private boolean complete;
        private boolean failCompletionQuery;
        private int cancelCount;

        @Override
        public void start(LoopClock clock) {
            started = true;
        }

        @Override
        public void update(LoopClock clock) {
        }

        @Override
        public void cancel() {
            if (!started || complete) {
                return;
            }
            complete = true;
            cancelCount++;
        }

        @Override
        public boolean isComplete() {
            if (failCompletionQuery) {
                throw new RuntimeException("output debug completion failed");
            }
            return complete;
        }

        @Override
        public TaskOutcome getOutcome() {
            return complete ? TaskOutcome.CANCELLED : TaskOutcome.NOT_DONE;
        }

        @Override
        public double getOutput() {
            return 0.75;
        }
    }

    /** Task that fails at one selected runner lifecycle boundary. */
    private static final class LifecycleFailureTask implements Task {
        private final FailurePoint failurePoint;
        private final RuntimeException lifecycleFailure;
        private final RuntimeException cleanupFailure;
        private boolean started;
        private int cancelCount;

        private LifecycleFailureTask(FailurePoint failurePoint,
                                     RuntimeException lifecycleFailure,
                                     RuntimeException cleanupFailure) {
            this.failurePoint = failurePoint;
            this.lifecycleFailure = lifecycleFailure;
            this.cleanupFailure = cleanupFailure;
        }

        @Override
        public void start(LoopClock clock) {
            started = true;
            if (failurePoint == FailurePoint.START) {
                throw lifecycleFailure;
            }
        }

        @Override
        public void update(LoopClock clock) {
            if (failurePoint == FailurePoint.UPDATE) {
                throw lifecycleFailure;
            }
        }

        @Override
        public void cancel() {
            if (!started) {
                return;
            }
            cancelCount++;
            if (cleanupFailure != null) {
                throw cleanupFailure;
            }
        }

        @Override
        public boolean isComplete() {
            if (started && failurePoint == FailurePoint.IS_COMPLETE) {
                throw lifecycleFailure;
            }
            return false;
        }

        @Override
        public TaskOutcome getOutcome() {
            return TaskOutcome.NOT_DONE;
        }
    }

    /** Output task that fails on its second update or from cancellation, as configured. */
    private static final class FailingOutputTask implements OutputTask {
        private final RuntimeException updateFailure;
        private final RuntimeException cancelFailure;
        private boolean started;
        private boolean complete;
        private int updateCount;

        private FailingOutputTask(RuntimeException updateFailure, RuntimeException cancelFailure) {
            this.updateFailure = updateFailure;
            this.cancelFailure = cancelFailure;
        }

        @Override
        public void start(LoopClock clock) {
            started = true;
        }

        @Override
        public void update(LoopClock clock) {
            updateCount++;
            if (updateFailure != null && updateCount >= 2) {
                throw updateFailure;
            }
        }

        @Override
        public void cancel() {
            if (!started || complete) {
                return;
            }
            complete = true;
            if (cancelFailure != null) {
                throw cancelFailure;
            }
        }

        @Override
        public boolean isComplete() {
            return complete;
        }

        @Override
        public TaskOutcome getOutcome() {
            return complete ? TaskOutcome.CANCELLED : TaskOutcome.NOT_DONE;
        }

        @Override
        public double getOutput() {
            return 0.75;
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
}
