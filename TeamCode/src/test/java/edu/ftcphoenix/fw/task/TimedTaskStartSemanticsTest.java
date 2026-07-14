package edu.ftcphoenix.fw.task;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/** Verifies that core timed tasks measure only intervals that occur after their own start. */
public final class TimedTaskStartSemanticsTest {

    private static final double EPSILON = 1e-12;

    @Test
    public void runForSecondsIgnoresLargePreStartDeltaAndFinishesAtExactBoundary() {
        ManualLoopClock manualClock = new ManualLoopClock();
        manualClock.nextCycle(4.0);

        List<String> callbacks = new ArrayList<>();
        RunForSecondsTask task = new RunForSecondsTask(
                0.125,
                () -> callbacks.add("start"),
                clock -> callbacks.add("update"),
                () -> callbacks.add("finish"));
        TaskRunner runner = new TaskRunner();
        runner.enqueue(task);

        runner.update(manualClock.clock());
        assertFalse(task.isComplete());
        assertEquals(Arrays.asList("start", "update"), callbacks);
        assertEquals(0.125, task.getRemainingSec(), EPSILON);

        runner.update(manualClock.clock());
        assertEquals(Arrays.asList("start", "update"), callbacks);

        manualClock.nextCycle(0.0625);
        runner.update(manualClock.clock());
        assertFalse(task.isComplete());
        assertEquals(0.0625, task.getRemainingSec(), EPSILON);

        manualClock.nextCycle(0.0625);
        runner.update(manualClock.clock());
        assertTrue(task.isComplete());
        assertEquals(TaskOutcome.SUCCESS, task.getOutcome());
        assertEquals(Arrays.asList("start", "update", "update", "update", "finish"), callbacks);
    }

    @Test
    public void zeroDurationRunFinishesImmediatelyWithoutUpdateCallback() {
        ManualLoopClock manualClock = new ManualLoopClock();
        List<String> callbacks = new ArrayList<>();
        RunForSecondsTask task = new RunForSecondsTask(
                0.0,
                () -> callbacks.add("start"),
                clock -> callbacks.add("update"),
                () -> callbacks.add("finish"));
        TaskRunner runner = new TaskRunner();
        runner.enqueue(task);

        runner.update(manualClock.clock());

        assertTrue(task.isComplete());
        assertEquals(Arrays.asList("start", "finish"), callbacks);
    }

    @Test
    public void waitTimeoutStartsAtTaskStartAndConditionWinsAtBoundary() {
        ManualLoopClock manualClock = new ManualLoopClock();
        manualClock.nextCycle(3.0);
        AtomicBoolean ready = new AtomicBoolean(false);
        WaitUntilTask task = new WaitUntilTask(BooleanSource.of(ready::get), 0.125);
        TaskRunner runner = new TaskRunner();
        runner.enqueue(task);

        runner.update(manualClock.clock());
        assertFalse(task.isComplete());

        ready.set(true);
        manualClock.nextCycle(0.125);
        runner.update(manualClock.clock());

        assertTrue(task.isComplete());
        assertEquals(TaskOutcome.SUCCESS, task.getOutcome());
        assertFalse(task.isTimedOut());
    }

    @Test
    public void waitTimesOutAtInclusiveBoundaryWhenConditionIsFalse() {
        ManualLoopClock manualClock = new ManualLoopClock();
        WaitUntilTask task = new WaitUntilTask(BooleanSource.constant(false), 0.125);
        TaskRunner runner = new TaskRunner();
        runner.enqueue(task);

        runner.update(manualClock.clock());
        assertFalse(task.isComplete());

        manualClock.nextCycle(0.125);
        runner.update(manualClock.clock());

        assertTrue(task.isComplete());
        assertEquals(TaskOutcome.TIMEOUT, task.getOutcome());
        assertTrue(task.isTimedOut());
    }

    @Test
    public void zeroTimeoutStillSamplesConditionBeforeTimingOut() {
        ManualLoopClock manualClock = new ManualLoopClock();
        WaitUntilTask task = new WaitUntilTask(BooleanSource.constant(true), 0.0);
        TaskRunner runner = new TaskRunner();
        runner.enqueue(task);

        runner.update(manualClock.clock());

        assertEquals(TaskOutcome.SUCCESS, task.getOutcome());
    }

    @Test
    public void positiveDirectOutputShorterThanLoopIsVisibleForOneCycle() {
        ManualLoopClock manualClock = new ManualLoopClock();
        manualClock.nextCycle(2.0);
        OutputForSecondsTask task = new OutputForSecondsTask("short", 0.75, 0.02);
        OutputTaskRunner queue = new OutputTaskRunner(0.0);
        queue.enqueue(task);

        queue.update(manualClock.clock());
        assertFalse(task.isComplete());
        assertTrue(queue.hasActiveTask());
        assertEquals(0.75, queue.output(manualClock.clock()), EPSILON);

        queue.update(manualClock.clock());
        assertTrue(queue.hasActiveTask());
        assertEquals(0.75, queue.output(manualClock.clock()), EPSILON);

        manualClock.nextCycle(0.10);
        queue.update(manualClock.clock());
        assertTrue(task.isComplete());
        assertFalse(queue.hasActiveTask());
        assertEquals(0.0, queue.output(manualClock.clock()), EPSILON);
    }

    @Test
    public void zeroDurationDirectOutputStaysIdle() {
        ManualLoopClock manualClock = new ManualLoopClock();
        OutputForSecondsTask task = new OutputForSecondsTask("zero", 1.0, 0.0);
        OutputTaskRunner queue = new OutputTaskRunner(-0.25);
        queue.enqueue(task);

        queue.update(manualClock.clock());

        assertTrue(task.isComplete());
        assertFalse(queue.hasActiveTask());
        assertEquals(-0.25, queue.output(manualClock.clock()), EPSILON);
    }

    @Test
    public void positiveGuidedPulsePublishesWhenGateOpensBeforeShortDurationElapses() {
        ManualLoopClock manualClock = new ManualLoopClock();
        manualClock.nextCycle(1.0);
        OutputTask task = Tasks.outputPulse("shortPulse")
                .startImmediately()
                .runOutput(0.9)
                .forSeconds(0.02)
                .buildTask();
        OutputTaskRunner queue = new OutputTaskRunner(0.0);
        queue.enqueue(task);

        queue.update(manualClock.clock());
        assertFalse(task.isComplete());
        assertTrue(queue.hasActiveTask());
        assertEquals(0.9, queue.output(manualClock.clock()), EPSILON);

        manualClock.nextCycle(0.10);
        queue.update(manualClock.clock());
        assertTrue(task.isComplete());
        assertEquals(TaskOutcome.SUCCESS, task.getOutcome());
        assertEquals(0.0, queue.output(manualClock.clock()), EPSILON);
    }

    @Test
    public void zeroDurationGuidedPulseNeverPublishesRunOutput() {
        ManualLoopClock manualClock = new ManualLoopClock();
        OutputTask task = Tasks.outputPulse("zeroPulse")
                .startImmediately()
                .runOutput(0.9)
                .forSeconds(0.0)
                .buildTask();
        OutputTaskRunner queue = new OutputTaskRunner(0.0);
        queue.enqueue(task);

        queue.update(manualClock.clock());

        assertTrue(task.isComplete());
        assertEquals(TaskOutcome.SUCCESS, task.getOutcome());
        assertFalse(queue.hasActiveTask());
        assertEquals(0.0, queue.output(manualClock.clock()), EPSILON);
    }

    @Test
    public void zeroDurationGuidedPulseRemainsIdleThroughConfiguredCooldown() {
        ManualLoopClock manualClock = new ManualLoopClock();
        OutputTask task = Tasks.outputPulse("zeroPulseWithCooldown")
                .startImmediately()
                .runOutput(0.9)
                .forSeconds(0.0)
                .cooldownSec(0.10)
                .buildTask();
        OutputTaskRunner queue = new OutputTaskRunner(0.0);
        queue.enqueue(task);

        queue.update(manualClock.clock());

        assertFalse(task.isComplete());
        assertTrue(queue.hasActiveTask());
        assertEquals(0.0, queue.output(manualClock.clock()), EPSILON);

        manualClock.nextCycle(0.10);
        queue.update(manualClock.clock());

        assertTrue(task.isComplete());
        assertEquals(TaskOutcome.SUCCESS, task.getOutcome());
        assertEquals(0.0, queue.output(manualClock.clock()), EPSILON);
    }

    @Test
    public void alreadyDoneSensorPulseWithNoMinimumCompletesWithoutOutput() {
        ManualLoopClock manualClock = new ManualLoopClock();
        final int[] runOutputSamples = {0};
        ScalarSource runOutput = new ScalarSource() {
            @Override
            public double getAsDouble(LoopClock clock) {
                runOutputSamples[0]++;
                return 0.9;
            }
        };
        OutputTask task = Tasks.outputPulse("alreadyDone")
                .startImmediately()
                .runOutput(runOutput)
                .until(BooleanSource.constant(true))
                .maxRunSec(1.0)
                .buildTask();
        OutputTaskRunner queue = new OutputTaskRunner(0.0);
        queue.enqueue(task);

        queue.update(manualClock.clock());

        assertTrue(task.isComplete());
        assertEquals(TaskOutcome.SUCCESS, task.getOutcome());
        assertEquals(0, runOutputSamples[0]);
        assertEquals(0.0, queue.output(manualClock.clock()), EPSILON);
    }

    @Test
    public void gatedRunAndCooldownUseTheirOwnExactBoundaryAnchors() {
        ManualLoopClock manualClock = new ManualLoopClock();
        manualClock.nextCycle(5.0);
        OutputTask task = new GatedOutputUntilTask(
                "boundedPulse",
                BooleanSource.constant(true),
                BooleanSource.constant(false),
                ScalarSource.constant(0.8),
                0.0,
                0.0,
                0.125,
                0.125);
        OutputTaskRunner queue = new OutputTaskRunner(0.0);
        queue.enqueue(task);

        queue.update(manualClock.clock());
        assertFalse(task.isComplete());
        assertEquals(0.8, queue.output(manualClock.clock()), EPSILON);

        manualClock.nextCycle(0.125);
        queue.update(manualClock.clock());
        assertFalse(task.isComplete());
        assertEquals(TaskOutcome.NOT_DONE, task.getOutcome());
        assertTrue(queue.hasActiveTask());
        assertEquals(0.0, queue.output(manualClock.clock()), EPSILON);

        manualClock.nextCycle(0.125);
        queue.update(manualClock.clock());
        assertTrue(task.isComplete());
        assertEquals(TaskOutcome.TIMEOUT, task.getOutcome());
        assertEquals(0.0, queue.output(manualClock.clock()), EPSILON);
    }

    @Test
    public void publicSequenceChildDoesNotConsumePreStartDelta() {
        ManualLoopClock manualClock = new ManualLoopClock();
        manualClock.nextCycle(3.0);
        AtomicBoolean followUpRan = new AtomicBoolean(false);
        Task sequence = Tasks.sequence(
                Tasks.waitForSeconds(0.125),
                Tasks.runOnce(() -> followUpRan.set(true)));
        TaskRunner runner = new TaskRunner();
        runner.enqueue(sequence);

        runner.update(manualClock.clock());
        assertFalse(sequence.isComplete());
        assertFalse(followUpRan.get());

        manualClock.nextCycle(0.125);
        runner.update(manualClock.clock());
        assertTrue(sequence.isComplete());
        assertTrue(followUpRan.get());
    }

    @Test
    public void publicParallelChildrenMeasureFromSharedStart() {
        ManualLoopClock manualClock = new ManualLoopClock();
        manualClock.nextCycle(3.0);
        Task shorter = Tasks.waitForSeconds(0.125);
        Task longer = Tasks.waitForSeconds(0.25);
        Task parallel = Tasks.parallelAll(shorter, longer);
        TaskRunner runner = new TaskRunner();
        runner.enqueue(parallel);

        runner.update(manualClock.clock());
        assertFalse(shorter.isComplete());
        assertFalse(longer.isComplete());

        manualClock.nextCycle(0.125);
        runner.update(manualClock.clock());
        assertTrue(shorter.isComplete());
        assertFalse(longer.isComplete());

        manualClock.nextCycle(0.125);
        runner.update(manualClock.clock());
        assertTrue(parallel.isComplete());
    }
}
