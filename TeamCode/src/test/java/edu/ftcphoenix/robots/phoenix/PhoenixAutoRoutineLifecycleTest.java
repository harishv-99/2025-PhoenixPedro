package edu.ftcphoenix.robots.phoenix;

import org.junit.Test;

import java.lang.reflect.Method;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies Phoenix's one-root autonomous installation and exact-START lifecycle. */
public final class PhoenixAutoRoutineLifecycleTest {

    @Test
    public void publicSurfaceHasOneInstallPathAndNoRunnerEscapeHatch() throws Exception {
        Method install = PhoenixRobot.class.getMethod("installAutoRoutine", Task.class);
        assertNotNull(install);

        assertNoPublicMethod("enqueueAuto", Task.class);
        assertNoPublicMethod("autoRunner");
    }

    @Test
    public void installRejectsNullSecondAndLateRoots() {
        PhoenixRobot.AutoRoutineLifecycle lifecycle =
                new PhoenixRobot.AutoRoutineLifecycle();

        try {
            lifecycle.install(null);
            fail("Expected a null Auto root to be rejected");
        } catch (NullPointerException expected) {
            assertTrue(expected.getMessage().contains("routine is required"));
        }

        RecordingTask first = new RecordingTask(false, null);
        lifecycle.install(first);
        try {
            lifecycle.install(new RecordingTask(false, null));
            fail("Expected a second Auto root to be rejected");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("already has an installed routine"));
        }

        PhoenixRobot.AutoRoutineLifecycle lateLifecycle =
                new PhoenixRobot.AutoRoutineLifecycle();
        lateLifecycle.markStartBoundary();
        try {
            lateLifecycle.install(new RecordingTask(false, null));
            fail("Expected installation after FTC START to be rejected");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("INIT-only"));
        }
    }

    @Test
    public void startRequiresInstalledRootAndRejectsRepeat() {
        LoopClock clock = new LoopClock();
        clock.reset(7.25);
        PhoenixRobot.AutoRoutineLifecycle empty =
                new PhoenixRobot.AutoRoutineLifecycle();

        try {
            empty.start(clock);
            fail("Expected Auto start without a root to fail");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("without an installed routine"));
        }

        PhoenixRobot.AutoRoutineLifecycle lifecycle =
                new PhoenixRobot.AutoRoutineLifecycle();
        RecordingTask root = new RecordingTask(false, null);
        lifecycle.install(root);

        try {
            lifecycle.start(clock);
            fail("Expected Auto start before the shared clock boundary to fail");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("startAny(runtime)"));
        }

        lifecycle.markStartBoundary();
        lifecycle.start(clock);

        assertEquals(1, root.starts);
        assertEquals(1, root.updates);
        assertEquals(7.25, root.startedAtSec, 0.0);
        assertEquals(7.25, root.updatedAtSec, 0.0);
        assertEquals(0L, root.startedCycle);
        assertEquals(0L, root.updatedCycle);

        try {
            lifecycle.start(clock);
            fail("Expected repeated Auto start to fail");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("already started"));
        }
        assertEquals(1, root.starts);
    }

    @Test
    public void completedRootRemainsAvailableAfterRunnerDetachesIt() {
        LoopClock clock = new LoopClock();
        clock.reset(3.0);
        PhoenixRobot.AutoRoutineLifecycle lifecycle =
                new PhoenixRobot.AutoRoutineLifecycle();
        RecordingTask root = new RecordingTask(true, null);
        lifecycle.install(root);
        lifecycle.markStartBoundary();
        lifecycle.start(clock);

        assertSame(root, lifecycle.installedRoutine());
        assertEquals(TaskOutcome.SUCCESS, root.getOutcome());

        clock.update(3.1);
        lifecycle.update(clock);
        lifecycle.cancelAndClear();

        assertSame(root, lifecycle.installedRoutine());
        assertEquals(0, root.cancels);
        assertEquals(1, root.updates);
    }

    @Test
    public void startFailureFailsClosedButRetainsRootAndCannotRestart() {
        LoopClock clock = new LoopClock();
        clock.reset(11.0);
        RuntimeException updateFailure = new IllegalStateException("root update failed");
        PhoenixRobot.AutoRoutineLifecycle lifecycle =
                new PhoenixRobot.AutoRoutineLifecycle();
        RecordingTask root = new RecordingTask(false, updateFailure);
        lifecycle.install(root);
        lifecycle.markStartBoundary();

        try {
            lifecycle.start(clock);
            fail("Expected the root update failure");
        } catch (RuntimeException actual) {
            assertSame(updateFailure, actual);
        }

        assertSame(root, lifecycle.installedRoutine());
        assertEquals(1, root.starts);
        assertEquals(1, root.updates);
        assertEquals(1, root.cancels);
        clock.update(11.1);
        lifecycle.update(clock);
        assertEquals(1, root.updates);

        try {
            lifecycle.start(clock);
            fail("Expected a failed root not to be restarted");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("already started"));
        }
        lifecycle.cancelAndClear();
        assertEquals(1, root.cancels);
    }

    @Test
    public void shutdownCancelsTheActiveRootOnceAndKeepsNoPendingWork() {
        LoopClock clock = new LoopClock();
        clock.reset(15.0);
        PhoenixRobot.AutoRoutineLifecycle lifecycle =
                new PhoenixRobot.AutoRoutineLifecycle();
        RecordingTask root = new RecordingTask(false, null);
        lifecycle.install(root);
        lifecycle.markStartBoundary();
        lifecycle.start(clock);

        lifecycle.cancelAndClear();
        lifecycle.cancelAndClear();
        clock.update(15.1);
        lifecycle.update(clock);

        assertSame(root, lifecycle.installedRoutine());
        assertEquals(1, root.cancels);
        assertEquals(1, root.updates);
        assertEquals(TaskOutcome.CANCELLED, root.getOutcome());
    }

    private static void assertNoPublicMethod(String name, Class<?>... parameterTypes) {
        try {
            PhoenixRobot.class.getMethod(name, parameterTypes);
            fail("Expected PhoenixRobot." + name + " not to be public");
        } catch (NoSuchMethodException expected) {
            // Expected: the TaskRunner remains private behind installAutoRoutine(...).
        }
    }

    private static final class RecordingTask implements Task {
        private final boolean completeOnUpdate;
        private final RuntimeException updateFailure;
        private int starts;
        private int updates;
        private int cancels;
        private boolean started;
        private boolean complete;
        private boolean cancelled;
        private double startedAtSec = Double.NaN;
        private double updatedAtSec = Double.NaN;
        private long startedCycle = Long.MIN_VALUE;
        private long updatedCycle = Long.MIN_VALUE;

        private RecordingTask(boolean completeOnUpdate, RuntimeException updateFailure) {
            this.completeOnUpdate = completeOnUpdate;
            this.updateFailure = updateFailure;
        }

        @Override
        public void start(LoopClock clock) {
            starts++;
            started = true;
            startedAtSec = clock.nowSec();
            startedCycle = clock.cycle();
        }

        @Override
        public void update(LoopClock clock) {
            updates++;
            updatedAtSec = clock.nowSec();
            updatedCycle = clock.cycle();
            if (updateFailure != null) {
                throw updateFailure;
            }
            if (completeOnUpdate) {
                complete = true;
            }
        }

        @Override
        public void cancel() {
            if (!started || complete) {
                return;
            }
            cancels++;
            cancelled = true;
            complete = true;
        }

        @Override
        public boolean isComplete() {
            return complete;
        }

        @Override
        public TaskOutcome getOutcome() {
            if (!complete) {
                return TaskOutcome.NOT_DONE;
            }
            return cancelled ? TaskOutcome.CANCELLED : TaskOutcome.SUCCESS;
        }
    }
}
