package edu.ftcphoenix.fw.actuation;

import org.junit.Test;

import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarTarget;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies single-use enforcement for task implementations owned by the actuation package. */
public final class ActuationTaskSingleUseTest {

    @Test
    public void scalarSetRejectsTerminalRestartWithoutRepeatingWriteAndFreshTaskWorks() {
        ManualLoopClock manualClock = new ManualLoopClock();
        CountingScalarTarget target = new CountingScalarTarget();
        Task first = ScalarTasks.set(target, 4.0);

        first.start(manualClock.clock());
        assertEquals(1, target.setCount);
        assertEquals(4.0, target.value, 0.0);

        assertSingleUseFailure(() -> first.start(manualClock.clock()), "ScalarTasks.set");
        assertEquals(1, target.setCount);

        Task fresh = ScalarTasks.set(target, 4.0);
        fresh.start(manualClock.clock());
        assertEquals(2, target.setCount);
        assertEquals(TaskOutcome.SUCCESS, fresh.getOutcome());
    }

    @Test
    public void moveRejectsRestartWhileActiveAndAfterCancellationWithoutRepeatingWrite() {
        ManualLoopClock manualClock = new ManualLoopClock();
        CountingFeedbackPlant plant = new CountingFeedbackPlant();
        Task move = PlantTasks.move(plant).to(8.0).timeout(1.0).build();

        move.start(manualClock.clock());
        assertFalse(move.isComplete());
        assertEquals(1, plant.command.setCount);

        assertSingleUseFailure(() -> move.start(manualClock.clock()), "PlantTasks.move");
        assertEquals(1, plant.command.setCount);
        assertFalse(move.isComplete());

        move.cancel();
        assertEquals(TaskOutcome.CANCELLED, move.getOutcome());
        assertSingleUseFailure(() -> move.start(manualClock.clock()), "PlantTasks.move");
        assertEquals(1, plant.command.setCount);
        assertEquals(TaskOutcome.CANCELLED, move.getOutcome());
    }

    @Test
    public void freshMoveTaskCanRequestTheSameTargetAgain() {
        ManualLoopClock manualClock = new ManualLoopClock();
        CountingFeedbackPlant plant = new CountingFeedbackPlant();

        Task first = PlantTasks.move(plant).to(3.0).build();
        first.start(manualClock.clock());
        first.cancel();

        Task fresh = PlantTasks.move(plant).to(3.0).build();
        fresh.start(manualClock.clock());

        assertEquals(2, plant.command.setCount);
        assertEquals(3.0, plant.command.value, 0.0);
        assertFalse(fresh.isComplete());
    }

    @Test
    public void movePreservesCancelBeforeFirstStartBehavior() {
        ManualLoopClock manualClock = new ManualLoopClock();
        CountingFeedbackPlant plant = new CountingFeedbackPlant();
        Task move = PlantTasks.move(plant).to(5.0).build();

        move.cancel();
        assertEquals(TaskOutcome.CANCELLED, move.getOutcome());

        move.start(manualClock.clock());
        assertEquals(1, plant.command.setCount);
        assertFalse(move.isComplete());
        assertEquals(TaskOutcome.NOT_DONE, move.getOutcome());
    }

    @Test
    public void calibrationSearchRejectsActiveAndTerminalRestartWithoutRepeatingHardwareEffects() {
        ManualLoopClock manualClock = new ManualLoopClock();
        CountingPositionPlant plant = new CountingPositionPlant();
        CountingBooleanSource condition = new CountingBooleanSource();
        Task search = calibrationSearch(plant, condition);

        search.start(manualClock.clock());
        assertEquals(1, plant.beginSearchCount);
        assertEquals(1, condition.resetCount);

        assertSingleUseFailure(() -> search.start(manualClock.clock()), "PositionCalibrationTasks.search");
        assertEquals(1, plant.beginSearchCount);
        assertEquals(1, condition.resetCount);

        search.cancel();
        assertEquals(1, plant.endSearchCount);
        assertEquals(TaskOutcome.CANCELLED, search.getOutcome());

        assertSingleUseFailure(() -> search.start(manualClock.clock()), "PositionCalibrationTasks.search");
        assertEquals(1, plant.beginSearchCount);
        assertEquals(1, plant.endSearchCount);
        assertEquals(1, condition.resetCount);
    }

    @Test
    public void freshCalibrationSearchCanStartAgain() {
        ManualLoopClock manualClock = new ManualLoopClock();
        CountingPositionPlant plant = new CountingPositionPlant();
        CountingBooleanSource condition = new CountingBooleanSource();

        Task first = calibrationSearch(plant, condition);
        first.start(manualClock.clock());
        first.cancel();

        Task fresh = calibrationSearch(plant, condition);
        fresh.start(manualClock.clock());

        assertEquals(2, plant.beginSearchCount);
        assertEquals(2, condition.resetCount);
        assertFalse(fresh.isComplete());
    }

    @Test
    public void calibrationSearchPreservesCancelBeforeFirstStartEffects() {
        ManualLoopClock manualClock = new ManualLoopClock();
        CountingPositionPlant plant = new CountingPositionPlant();
        CountingBooleanSource condition = new CountingBooleanSource();
        Task search = calibrationSearch(plant, condition);

        search.cancel();
        assertEquals(1, plant.endSearchCount);
        assertEquals(TaskOutcome.CANCELLED, search.getOutcome());

        search.start(manualClock.clock());
        assertEquals(1, plant.beginSearchCount);
        assertEquals(1, condition.resetCount);
        assertFalse(search.isComplete());
        assertEquals(TaskOutcome.NOT_DONE, search.getOutcome());
    }

    private static Task calibrationSearch(CountingPositionPlant plant,
                                          CountingBooleanSource condition) {
        return PositionCalibrationTasks.search(plant)
                .withPower(-0.2)
                .until(condition)
                .establishReferenceAt(0.0)
                .stopAfterReference()
                .neverTimeout()
                .build();
    }

    private static void assertSingleUseFailure(Runnable action, String expectedTaskName) {
        try {
            action.run();
            fail("expected a repeated start to fail");
        } catch (IllegalStateException expected) {
            String message = expected.getMessage();
            assertTrue(message.contains(expectedTaskName));
            assertTrue(message.contains("single-use"));
            assertTrue(message.contains("fresh Task"));
        }
    }

    private static final class CountingScalarTarget implements ScalarTarget {
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
    }

    private static final class CountingFeedbackPlant implements Plant {
        private final CountingScalarTarget command = new CountingScalarTarget();

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
            return false;
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

    private static final class CountingBooleanSource implements BooleanSource {
        private int resetCount;

        @Override
        public boolean getAsBoolean(LoopClock clock) {
            return false;
        }

        @Override
        public void reset() {
            resetCount++;
        }
    }

    private static final class CountingPositionPlant implements PositionPlant {
        private final CountingScalarTarget command = new CountingScalarTarget();
        private int beginSearchCount;
        private int endSearchCount;

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
        public Topology topology() {
            return Topology.LINEAR;
        }

        @Override
        public double period() {
            return Double.NaN;
        }

        @Override
        public ScalarRange targetRange() {
            return ScalarRange.unbounded();
        }

        @Override
        public boolean isReferenced() {
            return false;
        }

        @Override
        public String referenceStatus() {
            return "not referenced";
        }

        @Override
        public void establishReferenceAt(double plantPosition) {
        }

        @Override
        public boolean supportsCalibrationSearch() {
            return true;
        }

        @Override
        public void beginCalibrationSearch(double power) {
            beginSearchCount++;
        }

        @Override
        public void endCalibrationSearch(boolean stopOutput) {
            endSearchCount++;
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
