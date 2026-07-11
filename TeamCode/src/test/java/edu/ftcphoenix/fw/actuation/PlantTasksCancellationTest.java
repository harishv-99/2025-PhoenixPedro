package edu.ftcphoenix.fw.actuation;

import org.junit.Test;

import edu.ftcphoenix.fw.core.source.ScalarTarget;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;
import edu.ftcphoenix.fw.task.TaskRunner;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the explicit cancellation policy required by feedback-aware Plant moves. */
public final class PlantTasksCancellationTest {

    @Test
    public void cancelToWritesOnceOnlyForActiveCancellation() {
        ManualLoopClock manualClock = new ManualLoopClock();
        CountingFeedbackPlant plant = new CountingFeedbackPlant();
        Task move = PlantTasks.move(plant)
                .to(5.0)
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
    public void leaveTargetOnCancelPerformsNoCancellationWrite() {
        ManualLoopClock manualClock = new ManualLoopClock();
        CountingFeedbackPlant plant = new CountingFeedbackPlant();
        Task move = PlantTasks.move(plant)
                .to(6.0)
                .leaveTargetOnCancel()
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

        assertFiniteFailure(() -> PlantTasks.move(plant).to(1.0).cancelTo(Double.NaN));
        assertFiniteFailure(() -> PlantTasks.move(plant).to(1.0).cancelTo(Double.POSITIVE_INFINITY));
        assertFiniteFailure(() -> PlantTasks.move(plant).to(1.0).cancelTo(Double.NEGATIVE_INFINITY));
    }

    @Test
    public void throwingCancellationTargetLeavesTaskTerminalAndIsNotRetried() {
        ManualLoopClock manualClock = new ManualLoopClock();
        ThrowingScalarTarget target = new ThrowingScalarTarget(-2.0);
        CountingFeedbackPlant plant = new CountingFeedbackPlant(target);
        Task move = PlantTasks.move(plant)
                .to(7.0)
                .cancelTo(-2.0)
                .build();

        move.start(manualClock.clock());
        try {
            move.cancel();
            fail("expected the cancellation target write to fail");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("test target"));
        }

        assertTrue(move.isComplete());
        assertEquals(TaskOutcome.CANCELLED, move.getOutcome());
        assertEquals(2, target.setCount());

        move.cancel();
        move.update(manualClock.clock());
        assertEquals(2, target.setCount());
    }

    @Test
    public void failedMoveStartStillAppliesItsCancellationRequestOnce() {
        ManualLoopClock manualClock = new ManualLoopClock();
        ThrowingScalarTarget target = new ThrowingScalarTarget(7.0);
        CountingFeedbackPlant plant = new CountingFeedbackPlant(target);
        Task move = PlantTasks.move(plant)
                .to(7.0)
                .cancelTo(-2.0)
                .build();

        try {
            move.start(manualClock.clock());
            fail("expected the move request to fail");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("test target"));
        }

        move.cancel();
        move.cancel();
        assertTrue(move.isComplete());
        assertEquals(TaskOutcome.CANCELLED, move.getOutcome());
        assertEquals(2, target.setCount());
        assertEquals(-2.0, target.get(), 0.0);
    }

    @Test
    public void successfulMoveUsesThenTargetAndNeverCancellationTarget() {
        ManualLoopClock manualClock = new ManualLoopClock();
        CountingFeedbackPlant plant = new CountingFeedbackPlant();
        plant.reached = true;
        Task move = PlantTasks.move(plant)
                .to(8.0)
                .cancelTo(-1.0)
                .thenTarget(2.0)
                .build();

        move.start(manualClock.clock());
        move.update(manualClock.clock());
        move.cancel();

        assertEquals(TaskOutcome.SUCCESS, move.getOutcome());
        assertEquals(2, plant.command.setCount);
        assertEquals(2.0, plant.command.value, 0.0);
    }

    @Test
    public void timedOutMoveUsesThenTargetAndNeverCancellationTarget() {
        ManualLoopClock manualClock = new ManualLoopClock();
        CountingFeedbackPlant plant = new CountingFeedbackPlant();
        Task move = PlantTasks.move(plant)
                .to(9.0)
                .cancelTo(-1.0)
                .timeout(0.1)
                .thenTarget(3.0)
                .build();

        move.start(manualClock.clock());
        manualClock.nextCycle(0.11);
        move.update(manualClock.clock());
        move.cancel();

        assertEquals(TaskOutcome.TIMEOUT, move.getOutcome());
        assertEquals(2, plant.command.setCount);
        assertEquals(3.0, plant.command.value, 0.0);
    }

    @Test
    public void timedWriteAppliesThenValueOnceWhenActivelyCancelled() {
        ManualLoopClock manualClock = new ManualLoopClock();
        CountingFeedbackPlant plant = new CountingFeedbackPlant();
        Task write = PlantTasks.write(plant)
                .to(4.0)
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
    public void reentrantCancellationFromThenTargetPreservesCancellationOutcome() {
        ManualLoopClock manualClock = new ManualLoopClock();
        TaskRunner runner = new TaskRunner();
        ReentrantCancelScalarTarget target = new ReentrantCancelScalarTarget(runner, 2.0);
        CountingFeedbackPlant plant = new CountingFeedbackPlant(target);
        plant.reached = true;
        Task move = PlantTasks.move(plant)
                .to(8.0)
                .cancelTo(-1.0)
                .thenTarget(2.0)
                .build();
        runner.enqueue(move);

        runner.update(manualClock.clock());

        assertTrue(runner.isIdle());
        assertTrue(move.isComplete());
        assertEquals(TaskOutcome.CANCELLED, move.getOutcome());
        assertEquals(3, target.setCount());
        assertEquals(-1.0, target.get(), 0.0);
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

        private ThrowingScalarTarget(double throwingValue) {
            this.throwingValue = throwingValue;
        }

        @Override
        public void set(double value) {
            super.set(value);
            if (value == throwingValue) {
                throw new IllegalStateException("test target rejected cancellation request");
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
            return reached;
        }

        @Override
        public boolean hasWritableTarget() {
            return true;
        }

        @Override
        public ScalarTarget writableTarget() {
            return command;
        }

        @Override
        public void stop() {
        }
    }
}
