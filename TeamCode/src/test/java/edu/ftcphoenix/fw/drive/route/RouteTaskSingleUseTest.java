package edu.ftcphoenix.fw.drive.route;

import org.junit.Test;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies single-use lifecycle enforcement at the external-route task boundary. */
public final class RouteTaskSingleUseTest {

    @Test
    public void secondStartWhileActiveFailsBeforeFollowingAgain() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingFollower follower = new RecordingFollower();
        RouteTask<String> task = new RouteTask<>("outbound", follower, "route-a", null);

        task.start(manualClock.clock());
        assertEquals(1, follower.followCount);

        IllegalStateException failure = expectSecondStartFailure(task, manualClock.clock());

        assertActionable(failure, "outbound", "RouteTasks.follow");
        assertEquals(1, follower.followCount);
        assertEquals(0, follower.cancelCount);
        assertFalse(task.isComplete());
    }

    @Test
    public void secondStartAfterTerminalCancelHasNoRepeatedFollowerSideEffect() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingFollower follower = new RecordingFollower();
        RouteTask<String> task = new RouteTask<>("return", follower, "route-b", null);

        task.start(manualClock.clock());
        task.cancel();
        assertTrue(task.isComplete());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertEquals(1, follower.followCount);
        assertEquals(1, follower.cancelCount);

        IllegalStateException failure = expectSecondStartFailure(task, manualClock.clock());

        assertActionable(failure, "return", "Supplier<Task>");
        assertEquals(1, follower.followCount);
        assertEquals(1, follower.cancelCount);
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
    }

    @Test
    public void freshFactoryTasksCanFollowIndependently() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingFollower follower = new RecordingFollower();
        Task first = RouteTasks.follow("first", follower, "route", null);
        Task second = RouteTasks.follow("second", follower, "route", null);

        first.start(manualClock.clock());
        first.cancel();
        second.start(manualClock.clock());

        assertEquals(2, follower.followCount);
        assertFalse(second.isComplete());
    }

    @Test
    public void cancelBeforeFirstStartKeepsExistingRouteBehavior() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingFollower follower = new RecordingFollower();
        RouteTask<String> task = new RouteTask<>("cancelBeforeStart", follower, "route", null);

        task.cancel();
        assertTrue(task.isComplete());
        assertEquals(1, follower.cancelCount);

        task.start(manualClock.clock());

        assertEquals(1, follower.followCount);
        assertFalse(task.isComplete());
        assertEquals(TaskOutcome.NOT_DONE, task.getOutcome());
    }

    private static IllegalStateException expectSecondStartFailure(Task task, LoopClock clock) {
        try {
            task.start(clock);
            fail("Expected a second start to fail");
            return null;
        } catch (IllegalStateException expected) {
            return expected;
        }
    }

    private static void assertActionable(IllegalStateException failure,
                                         String taskName,
                                         String freshTaskHint) {
        assertTrue(failure.getMessage().contains(taskName));
        assertTrue(failure.getMessage().contains("single-use"));
        assertTrue(failure.getMessage().contains(freshTaskHint));
    }

    private static final class RecordingFollower implements RouteFollower<String> {
        private int followCount;
        private int cancelCount;
        private boolean busy = true;

        @Override
        public void follow(String route) {
            followCount++;
            busy = true;
        }

        @Override
        public boolean isBusy() {
            return busy;
        }

        @Override
        public void cancel() {
            cancelCount++;
            busy = false;
        }
    }
}
