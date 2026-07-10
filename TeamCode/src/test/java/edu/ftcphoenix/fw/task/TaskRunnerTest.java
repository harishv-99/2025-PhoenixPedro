package edu.ftcphoenix.fw.task;

import org.junit.Test;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;

/** Verifies representative queue behavior without fixing later task-policy decisions in place. */
public final class TaskRunnerTest {

    @Test
    public void repeatedUpdateInSameCycleDoesNotAdvanceTaskTwice() {
        ManualLoopClock manualClock = new ManualLoopClock();
        TaskRunner runner = new TaskRunner();
        CountingTask task = new CountingTask();
        runner.enqueue(task);

        runner.update(manualClock.clock());
        int updatesAfterFirstCall = task.updateCount;
        assertEquals(1, task.startCount);

        runner.update(manualClock.clock());
        assertEquals(1, task.startCount);
        assertEquals(updatesAfterFirstCall, task.updateCount);

        manualClock.nextCycle(0.02);
        runner.update(manualClock.clock());
        assertEquals(updatesAfterFirstCall + 1, task.updateCount);
    }

    private static final class CountingTask implements Task {
        private int startCount;
        private int updateCount;

        @Override
        public void start(LoopClock clock) {
            startCount++;
        }

        @Override
        public void update(LoopClock clock) {
            updateCount++;
        }

        @Override
        public boolean isComplete() {
            return false;
        }

        @Override
        public TaskOutcome getOutcome() {
            return TaskOutcome.NOT_DONE;
        }
    }
}
