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
import static org.junit.Assert.assertTrue;

/** Verifies that feedback-move timing begins at the event each interval describes. */
public final class ScalarTasksTimingTest {

    @Test
    public void timedWriteRemainsRequestedForItsSubLoopStartCycle() {
        ManualLoopClock manualClock = new ManualLoopClock();
        manualClock.nextCycle(1.0);
        FakeFeedbackPlant plant = new FakeFeedbackPlant();
        Task write = ScalarTasks.set(plant.command, 5.0)
                .forSeconds(0.02)
                .then(-1.0)
                .build();
        TaskRunner runner = new TaskRunner();
        runner.enqueue(write);

        runner.update(manualClock.clock());

        assertFalse(write.isComplete());
        assertEquals(5.0, plant.command.get(), 0.0);

        manualClock.nextCycle(0.10);
        runner.update(manualClock.clock());

        assertTrue(write.isComplete());
        assertEquals(-1.0, plant.command.get(), 0.0);
    }

    @Test
    public void timeoutStartsWhenMoveStartsNotAtBeginningOfLoopInterval() {
        ManualLoopClock manualClock = new ManualLoopClock();
        FakeFeedbackPlant plant = new FakeFeedbackPlant();
        Task move = ScalarTasks.set(plant.command, 8.0)
                .untilReachedBy(plant)
                .leaveRequestOnCancel()
                .timeout(0.10)
                .build();

        manualClock.nextCycle(1.0);
        move.start(manualClock.clock());
        move.update(manualClock.clock());

        assertFalse(move.isComplete());
        assertEquals(8.0, plant.command.get(), 0.0);

        manualClock.nextCycle(0.09);
        move.update(manualClock.clock());
        assertFalse(move.isComplete());

        manualClock.nextCycle(0.02);
        move.update(manualClock.clock());
        assertTrue(move.isComplete());
        assertEquals(TaskOutcome.TIMEOUT, move.getOutcome());
        assertEquals(8.0, plant.command.get(), 0.0);
    }

    @Test
    public void stablePeriodStartsWhenTargetIsFirstObservedReached() {
        ManualLoopClock manualClock = new ManualLoopClock();
        FakeFeedbackPlant plant = new FakeFeedbackPlant();
        Task move = ScalarTasks.set(plant.command, 12.0)
                .untilReachedBy(plant)
                .leaveRequestOnCancel()
                .stableFor(0.10)
                .timeout(1.0)
                .build();

        manualClock.nextCycle(0.75);
        move.start(manualClock.clock());
        move.update(manualClock.clock());

        manualClock.nextCycle(0.20);
        plant.reached = true;
        move.update(manualClock.clock());
        assertFalse("pre-observation time must not satisfy stableFor", move.isComplete());

        manualClock.nextCycle(0.09);
        move.update(manualClock.clock());
        assertFalse(move.isComplete());

        manualClock.nextCycle(0.02);
        move.update(manualClock.clock());
        assertTrue(move.isComplete());
        assertEquals(TaskOutcome.SUCCESS, move.getOutcome());
        assertEquals(12.0, plant.command.get(), 0.0);
    }

    @Test
    public void losingTargetResetsStablePeriod() {
        ManualLoopClock manualClock = new ManualLoopClock();
        FakeFeedbackPlant plant = new FakeFeedbackPlant();
        plant.reached = true;
        Task move = ScalarTasks.set(plant.command, 4.0)
                .untilReachedBy(plant)
                .leaveRequestOnCancel()
                .stableFor(0.10)
                .build();

        move.start(manualClock.clock());
        move.update(manualClock.clock());

        manualClock.nextCycle(0.06);
        move.update(manualClock.clock());
        assertFalse(move.isComplete());

        manualClock.nextCycle(0.02);
        plant.reached = false;
        move.update(manualClock.clock());

        manualClock.nextCycle(0.06);
        plant.reached = true;
        move.update(manualClock.clock());
        assertFalse(move.isComplete());

        manualClock.nextCycle(0.06);
        move.update(manualClock.clock());
        assertFalse(move.isComplete());

        manualClock.nextCycle(0.05);
        move.update(manualClock.clock());
        assertTrue(move.isComplete());
        assertEquals(TaskOutcome.SUCCESS, move.getOutcome());
    }

    @Test
    public void stableSuccessWinsWhenSuccessAndTimeoutBecomeDueTogether() {
        ManualLoopClock manualClock = new ManualLoopClock();
        FakeFeedbackPlant plant = new FakeFeedbackPlant();
        plant.reached = true;
        Task move = ScalarTasks.set(plant.command, 6.0)
                .untilReachedBy(plant)
                .leaveRequestOnCancel()
                .stableFor(0.10)
                .timeout(0.10)
                .build();

        move.start(manualClock.clock());
        move.update(manualClock.clock());
        manualClock.nextCycle(0.11);
        move.update(manualClock.clock());

        assertTrue(move.isComplete());
        assertEquals(TaskOutcome.SUCCESS, move.getOutcome());
        assertEquals(6.0, plant.command.get(), 0.0);
    }

    @Test
    public void cancellationKeepsRequestedTargetAndDoesNotApplyCompletionTarget() {
        ManualLoopClock manualClock = new ManualLoopClock();
        FakeFeedbackPlant plant = new FakeFeedbackPlant();
        Task move = ScalarTasks.set(plant.command, 5.0)
                .untilReachedBy(plant)
                .leaveRequestOnCancel()
                .timeout(1.0)
                .build();

        move.start(manualClock.clock());
        move.update(manualClock.clock());
        move.cancel();

        assertTrue(move.isComplete());
        assertEquals(TaskOutcome.CANCELLED, move.getOutcome());
        assertEquals(5.0, plant.command.get(), 0.0);
    }

    private static final class FakeFeedbackPlant implements Plant {
        private final ScalarTarget command = ScalarTarget.create(0.0);
        private boolean reached;

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
