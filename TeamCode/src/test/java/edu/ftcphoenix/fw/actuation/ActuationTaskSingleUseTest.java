package edu.ftcphoenix.fw.actuation;

import org.junit.Test;

import edu.ftcphoenix.fw.core.source.BooleanSource;
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

/** Verifies single-use enforcement for task implementations owned by the actuation package. */
public final class ActuationTaskSingleUseTest {

    @Test
    public void scalarSetCancelBeforeStartIsNoOpAndUpdateBeforeStartFails() {
        ManualLoopClock manualClock = new ManualLoopClock();
        CountingScalarTarget target = new CountingScalarTarget();
        Task set = ScalarTasks.set(target, 2.0);

        set.cancel();
        assertFalse(set.isComplete());
        assertEquals(TaskOutcome.NOT_DONE, set.getOutcome());
        assertEquals(0, target.setCount);
        assertUpdateBeforeStartFailure(set, manualClock.clock(), "ScalarTasks.set");

        set.start(manualClock.clock());
        set.cancel();
        assertTrue(set.isComplete());
        assertEquals(TaskOutcome.SUCCESS, set.getOutcome());
        assertEquals(1, target.setCount);
    }

    @Test
    public void scalarSetFailedStartCanBeCancelledWithoutRetryingTheWrite() {
        ManualLoopClock manualClock = new ManualLoopClock();
        ThrowingScalarTarget target = new ThrowingScalarTarget(3.0);
        Task set = ScalarTasks.set(target, 3.0);

        try {
            set.start(manualClock.clock());
            fail("expected the target write to fail");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("test target"));
        }

        set.cancel();
        set.cancel();
        assertTrue(set.isComplete());
        assertEquals(TaskOutcome.CANCELLED, set.getOutcome());
        assertEquals(1, target.setCount);
    }

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
        Task move = PlantTasks.move(plant).to(8.0).cancelTo(0.0).timeout(1.0).build();

        move.start(manualClock.clock());
        assertFalse(move.isComplete());
        assertEquals(1, plant.command.setCount);

        assertSingleUseFailure(() -> move.start(manualClock.clock()), "PlantTasks.move");
        assertEquals(1, plant.command.setCount);
        assertFalse(move.isComplete());

        move.cancel();
        assertEquals(TaskOutcome.CANCELLED, move.getOutcome());
        move.cancel();
        assertSingleUseFailure(() -> move.start(manualClock.clock()), "PlantTasks.move");
        assertEquals(2, plant.command.setCount);
        assertEquals(0.0, plant.command.value, 0.0);
        assertEquals(TaskOutcome.CANCELLED, move.getOutcome());
    }

    @Test
    public void freshMoveTaskCanRequestTheSameTargetAgain() {
        ManualLoopClock manualClock = new ManualLoopClock();
        CountingFeedbackPlant plant = new CountingFeedbackPlant();

        Task first = PlantTasks.move(plant).to(3.0).leaveTargetOnCancel().build();
        first.start(manualClock.clock());
        first.cancel();

        Task fresh = PlantTasks.move(plant).to(3.0).leaveTargetOnCancel().build();
        fresh.start(manualClock.clock());

        assertEquals(2, plant.command.setCount);
        assertEquals(3.0, plant.command.value, 0.0);
        assertFalse(fresh.isComplete());
    }

    @Test
    public void moveCancelBeforeFirstStartIsNoOp() {
        ManualLoopClock manualClock = new ManualLoopClock();
        CountingFeedbackPlant plant = new CountingFeedbackPlant();
        Task move = PlantTasks.move(plant).to(5.0).cancelTo(0.0).build();

        assertUpdateBeforeStartFailure(move, manualClock.clock(), "PlantTasks.move");
        move.cancel();
        assertFalse(move.isComplete());
        assertEquals(TaskOutcome.NOT_DONE, move.getOutcome());
        assertEquals(0, plant.command.setCount);

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
        search.cancel();
        assertEquals(1, plant.endSearchCount);

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
    public void calibrationSearchCancelBeforeFirstStartIsNoOp() {
        ManualLoopClock manualClock = new ManualLoopClock();
        CountingPositionPlant plant = new CountingPositionPlant();
        CountingBooleanSource condition = new CountingBooleanSource();
        Task search = calibrationSearch(plant, condition);

        assertUpdateBeforeStartFailure(search, manualClock.clock(), "PositionCalibrationTasks.search");
        search.cancel();
        assertEquals(0, plant.endSearchCount);
        assertFalse(search.isComplete());
        assertEquals(TaskOutcome.NOT_DONE, search.getOutcome());

        search.start(manualClock.clock());
        assertEquals(1, plant.beginSearchCount);
        assertEquals(1, condition.resetCount);
        assertFalse(search.isComplete());
        assertEquals(TaskOutcome.NOT_DONE, search.getOutcome());
    }

    @Test
    public void calibrationFailedStartStillAllowsOneCleanupAttempt() {
        ManualLoopClock manualClock = new ManualLoopClock();
        CountingPositionPlant plant = new CountingPositionPlant();
        plant.throwOnBeginSearch = true;
        CountingBooleanSource condition = new CountingBooleanSource();
        Task search = calibrationSearch(plant, condition);

        try {
            search.start(manualClock.clock());
            fail("expected calibration start to fail");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("test calibration"));
        }

        search.cancel();
        search.cancel();
        assertTrue(search.isComplete());
        assertEquals(TaskOutcome.CANCELLED, search.getOutcome());
        assertEquals(1, plant.beginSearchCount);
        assertEquals(1, plant.endSearchCount);
    }

    @Test
    public void calibrationConditionResetFailureDoesNotReleaseAnUnstartedSearch() {
        ManualLoopClock manualClock = new ManualLoopClock();
        CountingPositionPlant plant = new CountingPositionPlant();
        CountingBooleanSource condition = new CountingBooleanSource();
        condition.throwOnReset = true;
        Task search = calibrationSearch(plant, condition);

        try {
            search.start(manualClock.clock());
            fail("expected condition reset to fail");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("test condition"));
        }

        search.cancel();
        search.cancel();
        assertTrue(search.isComplete());
        assertEquals(TaskOutcome.CANCELLED, search.getOutcome());
        assertEquals(0, plant.beginSearchCount);
        assertEquals(0, plant.endSearchCount);
    }

    @Test
    public void throwingCalibrationCleanupLeavesSearchTerminalAndIsNotRetried() {
        ManualLoopClock manualClock = new ManualLoopClock();
        CountingPositionPlant plant = new CountingPositionPlant();
        plant.throwOnEndSearch = true;
        Task search = calibrationSearch(plant, new CountingBooleanSource());
        search.start(manualClock.clock());

        try {
            search.cancel();
            fail("expected calibration cleanup to fail");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("test calibration cleanup"));
        }

        assertTrue(search.isComplete());
        assertEquals(TaskOutcome.CANCELLED, search.getOutcome());
        search.cancel();
        assertEquals(1, plant.endSearchCount);
    }

    @Test
    public void reentrantCancellationDuringConditionResetDoesNotStartTheSearch() {
        ManualLoopClock manualClock = new ManualLoopClock();
        TaskRunner runner = new TaskRunner();
        CountingPositionPlant plant = new CountingPositionPlant();
        BooleanSource condition = new BooleanSource() {
            @Override
            public boolean getAsBoolean(LoopClock clock) {
                return false;
            }

            @Override
            public void reset() {
                runner.cancelCurrent();
            }
        };
        Task search = calibrationSearch(plant, condition);
        runner.enqueue(search);

        runner.update(manualClock.clock());

        assertTrue(runner.isIdle());
        assertTrue(search.isComplete());
        assertEquals(TaskOutcome.CANCELLED, search.getOutcome());
        assertEquals(0, plant.beginSearchCount);
        assertEquals(0, plant.endSearchCount);
    }

    private static Task calibrationSearch(CountingPositionPlant plant,
                                          BooleanSource condition) {
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

    private static void assertUpdateBeforeStartFailure(Task task,
                                                       LoopClock clock,
                                                       String expectedTaskName) {
        try {
            task.update(clock);
            fail("expected update before start to fail");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains(expectedTaskName));
            assertTrue(expected.getMessage().contains("before start"));
            assertTrue(expected.getMessage().contains("TaskRunner"));
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

    private static final class ThrowingScalarTarget implements ScalarTarget {
        private final double throwingValue;
        private int setCount;

        private ThrowingScalarTarget(double throwingValue) {
            this.throwingValue = throwingValue;
        }

        @Override
        public void set(double value) {
            setCount++;
            if (value == throwingValue) {
                throw new IllegalStateException("test target rejected write");
            }
        }

        @Override
        public double get() {
            return 0.0;
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
        private boolean throwOnReset;

        @Override
        public boolean getAsBoolean(LoopClock clock) {
            return false;
        }

        @Override
        public void reset() {
            resetCount++;
            if (throwOnReset) {
                throw new IllegalStateException("test condition reset failure");
            }
        }
    }

    private static final class CountingPositionPlant implements PositionPlant {
        private final CountingScalarTarget command = new CountingScalarTarget();
        private int beginSearchCount;
        private int endSearchCount;
        private boolean throwOnBeginSearch;
        private boolean throwOnEndSearch;

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
            if (throwOnBeginSearch) {
                throw new IllegalStateException("test calibration start failure");
            }
        }

        @Override
        public void endCalibrationSearch(boolean stopOutput) {
            endSearchCount++;
            if (throwOnEndSearch) {
                throw new IllegalStateException("test calibration cleanup failure");
            }
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
