package edu.ftcsushi.fw.actuation;

import org.junit.Test;

import edu.ftcsushi.fw.core.source.ScalarTarget;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.task.TaskRunner;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the explicit cancellation policy required by feedback-aware Plant moves. */
public final class ScalarTasksCancellationTest {

    @Test
    public void cancelToWritesOnceOnlyForActiveCancellation() {
        ManualLoopClock manualClock = new ManualLoopClock();
        CountingFeedbackPlant plant = new CountingFeedbackPlant();
        Task move = ScalarTasks.set(plant.command, 5.0)
                .untilReachedBy(plant)
                .cancelTo(-1.0)
                .build();

        move.cancel();
        assertFalse(move.isComplete());
        assertEquals(0, plant.command.setCount);

        move.start(manualClock.clock());
        assertEquals(1, plant.command.setCount);
        assertEquals(5.0, plant.command.value, 0.0);

        move.cancel();
        assertTrue(move.isComplete());
        assertEquals(TaskOutcome.CANCELLED, move.getOutcome());
        assertEquals(2, plant.command.setCount);
        assertEquals(-1.0, plant.command.value, 0.0);

        move.cancel();
        move.update(manualClock.clock());
        assertEquals(2, plant.command.setCount);
        assertEquals(-1.0, plant.command.value, 0.0);
    }

    @Test
    public void leaveRequestOnCancelPerformsNoCancellationWrite() {
        ManualLoopClock manualClock = new ManualLoopClock();
        CountingFeedbackPlant plant = new CountingFeedbackPlant();
        Task move = ScalarTasks.set(plant.command, 6.0)
                .untilReachedBy(plant)
                .leaveRequestOnCancel()
                .build();

        move.start(manualClock.clock());
        move.cancel();
        move.cancel();

        assertTrue(move.isComplete());
        assertEquals(TaskOutcome.CANCELLED, move.getOutcome());
        assertEquals(1, plant.command.setCount);
        assertEquals(6.0, plant.command.value, 0.0);
    }

    @Test
    public void cancelToRequiresAFinitePlantUnitTarget() {
        CountingFeedbackPlant plant = new CountingFeedbackPlant();

        assertFiniteFailure(() -> ScalarTasks.set(plant.command, 1.0)
                .untilReachedBy(plant).cancelTo(Double.NaN));
        assertFiniteFailure(() -> ScalarTasks.set(plant.command, 1.0)
                .untilReachedBy(plant).cancelTo(Double.POSITIVE_INFINITY));
        assertFiniteFailure(() -> ScalarTasks.set(plant.command, 1.0)
                .untilReachedBy(plant).cancelTo(Double.NEGATIVE_INFINITY));
    }

    @Test
    public void throwingCancellationTargetLeavesTaskTerminalAndIsNotRetried() {
        ManualLoopClock manualClock = new ManualLoopClock();
        ThrowingScalarTarget target = new ThrowingScalarTarget(-2.0);
        CountingFeedbackPlant plant = new CountingFeedbackPlant(target);
        Task move = ScalarTasks.set(target, 7.0)
                .untilReachedBy(plant)
                .cancelTo(-2.0)
                .build();

        move.start(manualClock.clock());
        assertSame(target.failure, expectRuntime(move::cancel));
        assertRetainedFailure(move, target.failure, manualClock);
        assertEquals(2, target.setCount());
    }

    @Test
    public void failedMoveStartStillAppliesItsCancellationRequestOnce() {
        ManualLoopClock manualClock = new ManualLoopClock();
        ThrowingScalarTarget target = new ThrowingScalarTarget(7.0);
        CountingFeedbackPlant plant = new CountingFeedbackPlant(target);
        Task move = ScalarTasks.set(target, 7.0)
                .untilReachedBy(plant)
                .cancelTo(-2.0)
                .build();

        assertSame(target.failure, expectRuntime(() -> move.start(manualClock.clock())));
        assertRetainedFailure(move, target.failure, manualClock);
        assertEquals(2, target.setCount());
        assertEquals(-2.0, target.get(), 0.0);
    }

    @Test
    public void directFeedbackUpdatesSampleOncePerCycleAndIgnoreRecursiveUpdate() {
        ManualLoopClock time = new ManualLoopClock();
        CountingFeedbackPlant plant = new CountingFeedbackPlant();
        Task move = ScalarTasks.set(plant.command, 5.0).untilReachedBy(plant)
                .leaveRequestOnCancel().build();
        plant.onAtTarget = () -> move.update(time.clock());

        move.start(time.clock());
        move.update(time.clock());
        plant.reached = true;
        move.update(time.clock());

        assertFalse(move.isComplete());
        assertEquals(1, plant.atTargetCount);
        move.update(time.nextCycle(0.02));
        assertEquals(TaskOutcome.SUCCESS, move.getOutcome());
        assertEquals(2, plant.atTargetCount);
        assertEquals(1, plant.command.setCount());
    }

    @Test
    public void feedbackFailureKeepsFirstExceptionAndAttemptsCancellationTargetOnce() {
        ManualLoopClock time = new ManualLoopClock();
        ThrowingScalarTarget target = new ThrowingScalarTarget(-1.0);
        CountingFeedbackPlant plant = new CountingFeedbackPlant(target);
        RuntimeException feedbackFailure = new IllegalStateException("feedback unavailable");
        plant.feedbackFailure = feedbackFailure;
        Task move = ScalarTasks.set(target, 5.0).untilReachedBy(plant)
                .cancelTo(-1.0).build();

        move.start(time.clock());
        assertSame(feedbackFailure, expectRuntime(() -> move.update(time.clock())));
        assertRetainedFailure(move, feedbackFailure, time);

        assertEquals(1, feedbackFailure.getSuppressed().length);
        assertSame(target.failure, feedbackFailure.getSuppressed()[0]);
        assertEquals(1, plant.atTargetCount);
        assertEquals(2, target.setCount());
    }

    @Test
    public void feedbackFailureRespectsLeaveRequestPolicy() {
        ManualLoopClock time = new ManualLoopClock();
        CountingFeedbackPlant plant = new CountingFeedbackPlant();
        RuntimeException failure = new IllegalStateException("feedback unavailable");
        plant.feedbackFailure = failure;
        Task move = ScalarTasks.set(plant.command, 5.0).untilReachedBy(plant)
                .leaveRequestOnCancel().build();

        move.start(time.clock());
        assertSame(failure, expectRuntime(() -> move.update(time.clock())));
        assertRetainedFailure(move, failure, time);
        assertEquals(1, plant.command.setCount());
        assertEquals(5.0, plant.command.get(), 0.0);
    }

    @Test
    public void timedStartFailureStillAttemptsThenValueOnce() {
        ManualLoopClock time = new ManualLoopClock();
        ThrowingScalarTarget target = new ThrowingScalarTarget(5.0);
        Task timed = ScalarTasks.set(target, 5.0).forSeconds(1.0).then(0.0).build();

        assertSame(target.failure, expectRuntime(() -> timed.start(time.clock())));
        assertRetainedFailure(timed, target.failure, time);
        assertEquals(2, target.setCount());
        assertEquals(0.0, target.get(), 0.0);
    }

    @Test
    public void timedEndingFailureCannotBecomeSuccessOrRepeatEndingWrite() {
        ManualLoopClock time = new ManualLoopClock();
        ThrowingScalarTarget target = new ThrowingScalarTarget(0.0);
        Task timed = ScalarTasks.set(target, 5.0).forSeconds(0.1).then(0.0).build();

        timed.start(time.clock());
        timed.update(time.clock());
        assertSame(target.failure, expectRuntime(() -> timed.update(time.nextCycle(0.1))));
        assertRetainedFailure(timed, target.failure, time);
        assertEquals(4, target.setCount());
    }

    @Test
    public void reentrantCancellationFromFeedbackInitialWritePreservesCancellationOutcome() {
        ManualLoopClock manualClock = new ManualLoopClock();
        TaskRunner runner = new TaskRunner();
        ReentrantCancelScalarTarget target = new ReentrantCancelScalarTarget(runner, 8.0);
        CountingFeedbackPlant plant = new CountingFeedbackPlant(target);
        Task move = ScalarTasks.set(target, 8.0)
                .untilReachedBy(plant)
                .cancelTo(-1.0)
                .build();
        runner.enqueue(move);

        runner.update(manualClock.clock());

        assertTrue(runner.isIdle());
        assertTrue(move.isComplete());
        assertEquals(TaskOutcome.CANCELLED, move.getOutcome());
        assertEquals(2, target.setCount());
        assertEquals(-1.0, target.get(), 0.0);
    }

    @Test
    public void successfulMoveLeavesRequestAndNeverUsesCancellationTarget() {
        ManualLoopClock manualClock = new ManualLoopClock();
        CountingFeedbackPlant plant = new CountingFeedbackPlant();
        plant.reached = true;
        Task move = ScalarTasks.set(plant.command, 8.0)
                .untilReachedBy(plant)
                .cancelTo(-1.0)
                .build();

        move.start(manualClock.clock());
        move.update(manualClock.clock());
        move.cancel();

        assertEquals(TaskOutcome.SUCCESS, move.getOutcome());
        assertEquals(1, plant.command.setCount);
        assertEquals(8.0, plant.command.value, 0.0);
    }

    @Test
    public void timedOutMoveLeavesRequestAndNeverUsesCancellationTarget() {
        ManualLoopClock manualClock = new ManualLoopClock();
        CountingFeedbackPlant plant = new CountingFeedbackPlant();
        Task move = ScalarTasks.set(plant.command, 9.0)
                .untilReachedBy(plant)
                .cancelTo(-1.0)
                .timeout(0.1)
                .build();

        move.start(manualClock.clock());
        manualClock.nextCycle(0.11);
        move.update(manualClock.clock());
        move.cancel();

        assertEquals(TaskOutcome.TIMEOUT, move.getOutcome());
        assertEquals(1, plant.command.setCount);
        assertEquals(9.0, plant.command.value, 0.0);
    }

    @Test
    public void timedWriteAppliesThenValueOnceWhenActivelyCancelled() {
        ManualLoopClock manualClock = new ManualLoopClock();
        CountingFeedbackPlant plant = new CountingFeedbackPlant();
        Task write = ScalarTasks.set(plant.command, 4.0)
                .forSeconds(1.0)
                .then(0.0)
                .build();

        write.cancel();
        assertEquals(0, plant.command.setCount);

        write.start(manualClock.clock());
        write.cancel();
        write.cancel();

        assertEquals(TaskOutcome.CANCELLED, write.getOutcome());
        assertEquals(2, plant.command.setCount);
        assertEquals(0.0, plant.command.value, 0.0);
    }

    @Test
    public void timedNaturalThenReentrantCancellationPreservesSuccessOutcome() {
        ManualLoopClock manualClock = new ManualLoopClock();
        TaskRunner runner = new TaskRunner();
        ReentrantCancelScalarTarget target = new ReentrantCancelScalarTarget(runner, 0.0);
        Task write = ScalarTasks.set(target, 4.0)
                .forSeconds(0.1)
                .then(0.0)
                .build();
        runner.enqueue(write);

        runner.update(manualClock.clock());
        assertFalse(write.isComplete());

        manualClock.nextCycle(0.11);
        runner.update(manualClock.clock());

        assertTrue(runner.isIdle());
        assertTrue(write.isComplete());
        assertEquals(TaskOutcome.SUCCESS, write.getOutcome());
        assertEquals(4, target.setCount());
        assertEquals(0.0, target.get(), 0.0);
    }

    private static void assertFiniteFailure(Runnable action) {
        try {
            action.run();
            fail("expected a finite-value validation failure");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("cancellation target"));
            assertTrue(expected.getMessage().contains("finite"));
        }
    }

    /** Failure reads remain diagnostic-only: they must not repeat target or feedback effects. */
    private static void assertRetainedFailure(Task task, RuntimeException failure,
                                              ManualLoopClock time) {
        assertTrue(task.isComplete());
        assertSame(failure, expectRuntime(task::getOutcome));
        assertSame(failure, expectRuntime(() -> task.update(time.clock())));
        assertSame(failure, expectRuntime(() -> task.update(time.nextCycle(0.02))));
        task.cancel();
        task.cancel();
    }

    private static RuntimeException expectRuntime(Runnable action) {
        try {
            action.run();
            fail("expected a lifecycle failure");
            return null;
        } catch (RuntimeException expected) {
            return expected;
        }
    }

    private static class CountingScalarTarget implements ScalarTarget {
        private int setCount;
        private double value;

        @Override
        public void set(double value) {
            setCount++;
            this.value = value;
        }

        @Override
        public double get() {
            return value;
        }

        int setCount() {
            return setCount;
        }
    }

    private static final class ThrowingScalarTarget extends CountingScalarTarget {
        private final double throwingValue;
        private final RuntimeException failure =
                new IllegalStateException("test target rejected cancellation request");

        private ThrowingScalarTarget(double throwingValue) {
            this.throwingValue = throwingValue;
        }

        @Override
        public void set(double value) {
            super.set(value);
            if (value == throwingValue) {
                throw failure;
            }
        }
    }

    private static final class ReentrantCancelScalarTarget extends CountingScalarTarget {
        private final TaskRunner runner;
        private final double cancellingValue;

        private ReentrantCancelScalarTarget(TaskRunner runner, double cancellingValue) {
            this.runner = runner;
            this.cancellingValue = cancellingValue;
        }

        @Override
        public void set(double value) {
            super.set(value);
            if (value == cancellingValue) {
                runner.cancelCurrent();
            }
        }
    }

    private static final class CountingFeedbackPlant implements Plant {
        private final CountingScalarTarget command;
        private boolean reached;
        private int atTargetCount;
        private Runnable onAtTarget;
        private RuntimeException feedbackFailure;

        private CountingFeedbackPlant() {
            this(new CountingScalarTarget());
        }

        private CountingFeedbackPlant(CountingScalarTarget command) {
            this.command = command;
        }

        @Override
        public void update(LoopClock clock) {
        }

        @Override
        public double getRequestedTarget() {
            return command.get();
        }

        @Override
        public double getAppliedTarget() {
            return command.get();
        }

        @Override
        public PlantTargetStatus getTargetStatus() {
            return PlantTargetStatus.ACCEPTED;
        }

        @Override
        public boolean hasFeedback() {
            return true;
        }

        @Override
        public boolean atTarget(double target) {
            atTargetCount++;
            if (onAtTarget != null) {
                onAtTarget.run();
            }
            if (feedbackFailure != null) {
                throw feedbackFailure;
            }
            return reached;
        }

        @Override
        public boolean hasCommandTarget() {
            return true;
        }

        @Override
        public ScalarTarget commandTarget() {
            return command;
        }

        @Override
        public void stop() {
        }
    }
}
