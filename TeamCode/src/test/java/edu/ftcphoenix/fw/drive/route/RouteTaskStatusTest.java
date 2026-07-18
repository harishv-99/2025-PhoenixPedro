package edu.ftcphoenix.fw.drive.route;

import org.junit.Test;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.task.TaskOutcome;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies truthful per-execution route status and RouteTask outcome mapping. */
public final class RouteTaskStatusTest {

    @Test
    public void factoryReturnsTypedTaskAndExposesActiveExecution() {
        RecordingFollower follower = new RecordingFollower();
        RouteTask<String> task =
                RouteTasks.followWithoutTaskTimeout("outbound", follower, "route-a");

        assertEquals(RouteStatus.NOT_STARTED, task.getRouteStatus());

        task.start(new ManualLoopClock().clock());

        assertEquals(RouteStatus.ACTIVE, task.getRouteStatus());
        assertEquals(TaskOutcome.NOT_DONE, task.getOutcome());
        assertFalse(task.isComplete());
        assertNotNull(follower.current);
    }

    @Test
    public void completedMapsToSuccess() {
        assertTerminalMapping(RouteStatus.COMPLETED, TaskOutcome.SUCCESS);
    }

    @Test
    public void followerTimeoutOrStallMapsToTimeout() {
        assertTerminalMapping(RouteStatus.FOLLOWER_TIMEOUT_OR_STALL, TaskOutcome.TIMEOUT);
    }

    @Test
    public void abnormalTerminalStatusesMapFailClosedWithoutLosingPreciseStatus() {
        RouteStatus[] statuses = {
                RouteStatus.INTERRUPTED,
                RouteStatus.REPLACED,
                RouteStatus.CANCELLED,
                RouteStatus.FAILED,
                RouteStatus.UNKNOWN_TERMINAL
        };

        for (RouteStatus status : statuses) {
            assertTerminalMapping(status, TaskOutcome.CANCELLED);
        }
    }

    @Test
    public void activeStatusRemainsNonterminal() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingFollower follower = new RecordingFollower();
        RouteTask<String> task =
                RouteTasks.followWithoutTaskTimeout("activeStatus", follower, "route");
        task.start(manualClock.clock());

        task.update(manualClock.clock());

        assertEquals(RouteStatus.ACTIVE, task.getRouteStatus());
        assertEquals(TaskOutcome.NOT_DONE, task.getOutcome());
        assertFalse(task.isComplete());
    }

    @Test
    public void statusGetterObservesHeartbeatTerminalBeforeTaskUpdate() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingFollower follower = new RecordingFollower();
        RouteTask<String> task =
                RouteTasks.followWithoutTaskTimeout("heartbeatTerminal", follower, "route");
        task.start(manualClock.clock());
        follower.current.integrationStatus = RouteStatus.INTERRUPTED;

        assertEquals(RouteStatus.INTERRUPTED, task.getRouteStatus());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertTrue(task.isComplete());

        task.cancel();
        assertEquals(0, follower.current.cancelCount);
        assertEquals(RouteStatus.INTERRUPTED, task.getRouteStatus());
    }

    @Test
    public void followerTerminalStatusWinsAtExactTaskTimeoutBoundary() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingFollower follower = new RecordingFollower();
        RouteTask<String> task =
                RouteTasks.follow("terminalBeforeTimeout", follower, "route", 0.05);
        task.start(manualClock.clock());
        follower.statusOnUpdate = RouteStatus.COMPLETED;

        manualClock.nextCycle(0.05);
        task.update(manualClock.clock());

        assertEquals(RouteStatus.COMPLETED, task.getRouteStatus());
        assertEquals(TaskOutcome.SUCCESS, task.getOutcome());
        assertEquals(0, follower.current.cancelCount);
    }

    @Test
    public void activeRouteTimesOutAtExactTaskTimeoutBoundary() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingFollower follower = new RecordingFollower();
        RouteTask<String> task =
                RouteTasks.follow("exactTimeoutBoundary", follower, "route", 0.05);
        task.start(manualClock.clock());

        manualClock.nextCycle(0.05);
        task.update(manualClock.clock());

        assertEquals(RouteStatus.TASK_TIMEOUT, task.getRouteStatus());
        assertEquals(TaskOutcome.TIMEOUT, task.getOutcome());
        assertTrue(task.isComplete());
        assertEquals(1, follower.current.cancelCount);
    }

    @Test
    public void taskTimeoutRetainsTaskReasonAndCancelsOnlyItsExecution() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingFollower follower = new RecordingFollower();
        RouteTask<String> task =
                RouteTasks.follow("taskTimeout", follower, "route", 0.05);
        task.start(manualClock.clock());
        RecordingExecution execution = follower.current;

        manualClock.nextCycle(0.10);
        task.update(manualClock.clock());
        task.cancel();

        assertEquals(RouteStatus.TASK_TIMEOUT, task.getRouteStatus());
        assertEquals(TaskOutcome.TIMEOUT, task.getOutcome());
        assertTrue(task.isComplete());
        assertEquals(RouteStatus.TASK_TIMEOUT, execution.status());
        assertEquals(1, execution.cancelCount);
    }

    @Test
    public void returnedNotStartedExecutionFailsFastAndFailsClosed() {
        RecordingFollower follower = new RecordingFollower();
        RecordingExecution execution = new RecordingExecution(follower);
        execution.integrationStatus = RouteStatus.NOT_STARTED;
        follower.current = execution;
        RouteTask<String> task = RouteTasks.followWithoutTaskTimeout(
                "notStartedContractViolation",
                route -> execution,
                "route");

        try {
            task.start(new ManualLoopClock().clock());
            fail("expected NOT_STARTED execution to fail");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("notStartedContractViolation"));
            assertTrue(expected.getMessage().contains("synchronously"));
            assertTrue(expected.getMessage().contains("ACTIVE"));
        }

        assertEquals(RouteStatus.FAILED, task.getRouteStatus());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertTrue(task.isComplete());
        assertEquals(RouteStatus.FAILED, execution.status());
        assertEquals(1, execution.cancelCount);
    }

    @Test
    public void timeoutStartsAtTaskBoundaryInsteadOfChargingPriorLoopInterval() {
        ManualLoopClock manualClock = new ManualLoopClock();
        manualClock.nextCycle(1.0);
        RecordingFollower follower = new RecordingFollower();
        RouteTask<String> task =
                RouteTasks.follow("timeoutBoundary", follower, "route", 0.05);

        task.start(manualClock.clock());
        task.update(manualClock.clock());
        assertFalse(task.isComplete());

        manualClock.nextCycle(0.06);
        task.update(manualClock.clock());
        assertEquals(RouteStatus.TASK_TIMEOUT, task.getRouteStatus());
        assertEquals(TaskOutcome.TIMEOUT, task.getOutcome());
    }

    @Test
    public void activeTaskCancellationIsTerminalAndIdempotent() {
        RecordingFollower follower = new RecordingFollower();
        RouteTask<String> task =
                RouteTasks.followWithoutTaskTimeout("activeCancellation", follower, "route");
        task.start(new ManualLoopClock().clock());
        RecordingExecution execution = follower.current;

        task.cancel();
        task.cancel();

        assertEquals(RouteStatus.CANCELLED, task.getRouteStatus());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertTrue(task.isComplete());
        assertEquals(RouteStatus.CANCELLED, execution.status());
        assertEquals(1, execution.cancelCount);
    }

    @Test
    public void taskCancellationPreservesCompletionObservedSinceLastUpdate() {
        RecordingFollower follower = new RecordingFollower();
        RouteTask<String> task =
                RouteTasks.followWithoutTaskTimeout("preserveCompletion", follower, "route");
        task.start(new ManualLoopClock().clock());
        RecordingExecution execution = follower.current;
        execution.integrationStatus = RouteStatus.COMPLETED;

        task.cancel();

        assertEquals(RouteStatus.COMPLETED, task.getRouteStatus());
        assertEquals(TaskOutcome.SUCCESS, task.getOutcome());
        assertTrue(task.isComplete());
        assertEquals(RouteStatus.COMPLETED, execution.status());
        assertEquals(0, execution.cancelCount);
    }

    @Test
    public void taskCancellationPreservesReplacementObservedSinceLastUpdate() {
        RecordingFollower follower = new RecordingFollower();
        RouteTask<String> oldTask =
                RouteTasks.followWithoutTaskTimeout("old", follower, "route-a");
        RouteTask<String> newTask =
                RouteTasks.followWithoutTaskTimeout("new", follower, "route-b");
        oldTask.start(new ManualLoopClock().clock());
        RecordingExecution oldExecution = follower.current;
        newTask.start(new ManualLoopClock().clock());
        RecordingExecution newExecution = follower.current;

        oldTask.cancel();

        assertEquals(RouteStatus.REPLACED, oldTask.getRouteStatus());
        assertEquals(TaskOutcome.CANCELLED, oldTask.getOutcome());
        assertEquals(0, oldExecution.cancelCount);
        assertSame(newExecution, follower.current);
        assertEquals(RouteStatus.ACTIVE, newExecution.status());
    }

    @Test
    public void taskCancellationStatusFailureFailsClosedBeforeRethrow() {
        RecordingFollower follower = new RecordingFollower();
        RouteTask<String> task =
                RouteTasks.followWithoutTaskTimeout("cancellationStatusFailure", follower, "route");
        task.start(new ManualLoopClock().clock());
        RecordingExecution execution = follower.current;
        IllegalStateException statusFailure = new IllegalStateException("cancel status failure");
        execution.statusFailure = statusFailure;

        try {
            task.cancel();
            fail("expected cancellation status failure");
        } catch (IllegalStateException actual) {
            assertSame(statusFailure, actual.getCause());
        }

        assertEquals(RouteStatus.FAILED, task.getRouteStatus());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertEquals(RouteStatus.FAILED, execution.status());
        assertEquals(1, execution.cancelCount);
    }

    @Test
    public void executionCancellationIsNoOpForTerminalStates() {
        RecordingFollower owner = new RecordingFollower();
        RecordingExecution execution = new RecordingExecution(owner);
        execution.integrationStatus = RouteStatus.COMPLETED;

        execution.cancel();
        execution.cancel();
        assertEquals(RouteStatus.COMPLETED, execution.status());
        assertEquals(0, execution.cancelCount);
    }

    @Test
    public void replacementBeatsOldTaskTimeoutAndOldCleanupCannotStopNewRoute() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingFollower follower = new RecordingFollower();
        RouteTask<String> oldTask = RouteTasks.follow("old", follower, "route-a", 0.05);
        RouteTask<String> newTask =
                RouteTasks.followWithoutTaskTimeout("new", follower, "route-b");
        oldTask.start(manualClock.clock());
        RecordingExecution oldExecution = follower.current;

        newTask.start(manualClock.clock());
        RecordingExecution newExecution = follower.current;
        manualClock.nextCycle(0.10);
        oldTask.update(manualClock.clock());
        oldTask.cancel();
        oldExecution.cancel();

        assertEquals(RouteStatus.REPLACED, oldTask.getRouteStatus());
        assertEquals(TaskOutcome.CANCELLED, oldTask.getOutcome());
        assertEquals(0, oldExecution.cancelCount);
        assertEquals(0, follower.updateCount);
        assertSame(newExecution, follower.current);
        assertEquals(RouteStatus.ACTIVE, newExecution.status());
        assertEquals(0, newExecution.cancelCount);
        assertFalse(newTask.isComplete());
    }

    @Test
    public void oldTaskReadsItsOwnCompletedExecutionInsteadOfNewRouteState() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingFollower follower = new RecordingFollower();
        RouteTask<String> oldTask =
                RouteTasks.followWithoutTaskTimeout("old", follower, "route-a");
        RouteTask<String> newTask =
                RouteTasks.followWithoutTaskTimeout("new", follower, "route-b");
        oldTask.start(manualClock.clock());
        RecordingExecution oldExecution = follower.current;
        oldExecution.integrationStatus = RouteStatus.COMPLETED;
        newTask.start(manualClock.clock());
        RecordingExecution newExecution = follower.current;

        oldTask.update(manualClock.clock());

        assertEquals(RouteStatus.COMPLETED, oldTask.getRouteStatus());
        assertEquals(TaskOutcome.SUCCESS, oldTask.getOutcome());
        assertEquals(RouteStatus.ACTIVE, newExecution.status());
        assertFalse(newTask.isComplete());
    }

    @Test
    public void updateFailureRefreshesFailedExecutionStatusBeforeRethrow() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingFollower follower = new RecordingFollower();
        RouteTask<String> task =
                RouteTasks.followWithoutTaskTimeout("updateFailure", follower, "route");
        task.start(manualClock.clock());
        IllegalStateException updateFailure = new IllegalStateException("test update failure");
        follower.statusBeforeUpdateFailure = RouteStatus.FAILED;
        follower.updateFailure = updateFailure;

        try {
            task.update(manualClock.clock());
            fail("expected update failure");
        } catch (IllegalStateException actual) {
            assertSame(updateFailure, actual);
        }

        assertTrue(task.isComplete());
        assertEquals(RouteStatus.FAILED, task.getRouteStatus());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertEquals(0, follower.current.cancelCount);
    }

    @Test
    public void updateFailureDoesNotOverwriteExplicitTerminalCompletion() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingFollower follower = new RecordingFollower();
        RouteTask<String> task =
                RouteTasks.followWithoutTaskTimeout("completedThenThrow", follower, "route");
        task.start(manualClock.clock());
        RecordingExecution execution = follower.current;
        IllegalStateException updateFailure = new IllegalStateException("late update failure");
        follower.statusBeforeUpdateFailure = RouteStatus.COMPLETED;
        follower.updateFailure = updateFailure;

        try {
            task.update(manualClock.clock());
            fail("expected update failure");
        } catch (IllegalStateException actual) {
            assertSame(updateFailure, actual);
        }

        assertTrue(task.isComplete());
        assertEquals(RouteStatus.COMPLETED, task.getRouteStatus());
        assertEquals(TaskOutcome.SUCCESS, task.getOutcome());
        assertEquals(RouteStatus.COMPLETED, execution.status());
        assertEquals(0, execution.cancelCount);
    }

    @Test
    public void updateFailureWithActiveExecutionMarksTaskFailedBeforeCleanup() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingFollower follower = new RecordingFollower();
        RouteTask<String> task =
                RouteTasks.followWithoutTaskTimeout("activeUpdateFailure", follower, "route");
        task.start(manualClock.clock());
        RecordingExecution execution = follower.current;
        execution.taskToObserveDuringCancel = task;
        IllegalStateException updateFailure = new IllegalStateException("active update failure");
        follower.updateFailure = updateFailure;

        try {
            task.update(manualClock.clock());
            fail("expected update failure");
        } catch (IllegalStateException actual) {
            assertSame(updateFailure, actual);
        }

        assertEquals(RouteStatus.FAILED, task.getRouteStatus());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertTrue(task.isComplete());
        assertEquals(RouteStatus.FAILED, execution.taskStatusObservedDuringCancel);
        assertEquals(RouteStatus.FAILED, execution.status());
        assertEquals(1, execution.cancelCount);
    }

    @Test
    public void updateFailurePreservesOriginalAndSuppressesStatusReadFailure() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingFollower follower = new RecordingFollower();
        RouteTask<String> task =
                RouteTasks.followWithoutTaskTimeout("statusReadFailure", follower, "route");
        task.start(manualClock.clock());
        IllegalStateException updateFailure = new IllegalStateException("update failure");
        IllegalStateException statusFailure = new IllegalStateException("status failure");
        follower.updateFailure = updateFailure;
        follower.statusFailureBeforeUpdateFailure = statusFailure;

        try {
            task.update(manualClock.clock());
            fail("expected update failure");
        } catch (IllegalStateException actual) {
            assertSame(updateFailure, actual);
        }

        assertEquals(RouteStatus.FAILED, task.getRouteStatus());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertTrue(task.isComplete());
        assertEquals(1, updateFailure.getSuppressed().length);
        assertSame(statusFailure, updateFailure.getSuppressed()[0].getCause());
        assertEquals(RouteStatus.FAILED, follower.current.status());
        assertEquals(1, follower.current.cancelCount);
    }

    @Test
    public void initialStatusReadFailureFailsClosedBeforeRethrow() {
        RecordingFollower follower = new RecordingFollower();
        RecordingExecution execution = new RecordingExecution(follower);
        IllegalStateException statusFailure = new IllegalStateException("initial status failure");
        execution.statusFailure = statusFailure;
        follower.current = execution;
        RouteTask<String> task = RouteTasks.followWithoutTaskTimeout(
                "initialStatusFailure",
                route -> execution,
                "route");

        try {
            task.start(new ManualLoopClock().clock());
            fail("expected initial status failure");
        } catch (IllegalStateException actual) {
            assertTrue(actual.getMessage().contains("initialStatusFailure"));
            assertSame(statusFailure, actual.getCause());
        }

        assertEquals(RouteStatus.FAILED, task.getRouteStatus());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertTrue(task.isComplete());
        assertEquals(RouteStatus.FAILED, execution.status());
        assertEquals(1, execution.cancelCount);
    }

    @Test
    public void executionCancelStatusFailurePreservesFailureAndFailsClosed() {
        RecordingFollower follower = new RecordingFollower();
        RecordingExecution execution = new RecordingExecution(follower);
        IllegalStateException statusFailure = new IllegalStateException("cancel status failure");
        IllegalStateException cleanupFailure = new IllegalStateException("cancel cleanup failure");
        execution.statusFailure = statusFailure;
        execution.cancelFailure = cleanupFailure;
        follower.current = execution;

        try {
            execution.cancel();
            fail("expected cancellation status failure");
        } catch (IllegalStateException actual) {
            assertSame(statusFailure, actual);
        }

        assertEquals(RouteStatus.FAILED, execution.status());
        assertEquals(1, execution.cancelCount);
        assertEquals(1, statusFailure.getSuppressed().length);
        assertSame(cleanupFailure, statusFailure.getSuppressed()[0]);
    }

    @Test
    public void updateFailurePreservesCleanupFailureAsSuppressed() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingFollower follower = new RecordingFollower();
        RouteTask<String> task =
                RouteTasks.followWithoutTaskTimeout("cleanupFailure", follower, "route");
        task.start(manualClock.clock());
        RecordingExecution execution = follower.current;
        IllegalStateException updateFailure = new IllegalStateException("update failure");
        IllegalStateException cleanupFailure = new IllegalStateException("cleanup failure");
        follower.updateFailure = updateFailure;
        execution.cancelFailure = cleanupFailure;

        try {
            task.update(manualClock.clock());
            fail("expected update failure");
        } catch (IllegalStateException actual) {
            assertSame(updateFailure, actual);
        }

        assertEquals(RouteStatus.FAILED, task.getRouteStatus());
        assertEquals(RouteStatus.FAILED, execution.status());
        assertEquals(1, execution.cancelCount);
        assertEquals(1, updateFailure.getSuppressed().length);
        assertSame(cleanupFailure, updateFailure.getSuppressed()[0]);
    }

    @Test
    public void timeoutRetainsStatusWhenCleanupThrowsAndDoesNotRetry() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingFollower follower = new RecordingFollower();
        RouteTask<String> task =
                RouteTasks.follow("timeoutCleanupFailure", follower, "route", 0.05);
        task.start(manualClock.clock());
        RecordingExecution execution = follower.current;
        IllegalStateException cleanupFailure = new IllegalStateException("timeout cleanup failure");
        execution.cancelFailure = cleanupFailure;

        manualClock.nextCycle(0.10);
        try {
            task.update(manualClock.clock());
            fail("expected timeout cleanup failure");
        } catch (IllegalStateException actual) {
            assertSame(cleanupFailure, actual);
        }

        assertTrue(task.isComplete());
        assertEquals(RouteStatus.TASK_TIMEOUT, task.getRouteStatus());
        assertEquals(TaskOutcome.TIMEOUT, task.getOutcome());
        assertEquals(RouteStatus.TASK_TIMEOUT, execution.status());
        assertEquals(1, execution.cancelCount);

        task.update(manualClock.clock());
        task.cancel();
        execution.cancel();
        assertEquals(1, execution.cancelCount);
    }

    @Test
    public void nullExecutionFailsFastWithAdapterAuthorGuidance() {
        RouteFollower<String> follower = route -> null;
        RouteTask<String> task =
                RouteTasks.followWithoutTaskTimeout("nullExecution", follower, "route");

        try {
            task.start(new ManualLoopClock().clock());
            fail("expected null execution to fail");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("nullExecution"));
            assertTrue(expected.getMessage().contains("RouteExecution"));
            assertTrue(expected.getMessage().contains("returned null"));
        }
    }

    @Test
    public void nullStatusFailsFastWithAdapterAuthorGuidance() {
        RouteExecution nullStatusExecution = new RouteExecution() {
            @Override
            protected RouteStatus integrationStatus() {
                return null;
            }

            @Override
            protected void cancelActive() {
                // no-op test fixture
            }
        };
        RouteFollower<String> follower = route -> nullStatusExecution;
        RouteTask<String> task =
                RouteTasks.followWithoutTaskTimeout("nullStatus", follower, "route");

        try {
            task.start(new ManualLoopClock().clock());
            fail("expected null status to fail");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("nullStatus"));
            assertTrue(expected.getMessage().contains("RouteStatus"));
            assertTrue(expected.getMessage().contains("returned null"));
        }
    }

    private static void assertTerminalMapping(RouteStatus status, TaskOutcome expectedOutcome) {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingFollower follower = new RecordingFollower();
        RouteTask<String> task = RouteTasks.followWithoutTaskTimeout(
                "mapping-" + status,
                follower,
                "route");
        task.start(manualClock.clock());
        follower.current.integrationStatus = status;

        task.update(manualClock.clock());

        assertEquals(status, task.getRouteStatus());
        assertEquals(expectedOutcome, task.getOutcome());
        assertTrue(task.isComplete());
        assertEquals(0, follower.current.cancelCount);
    }

    private static final class RecordingFollower implements RouteFollower<String> {
        private RecordingExecution current;
        private RouteStatus statusOnUpdate;
        private RouteStatus statusBeforeUpdateFailure;
        private RuntimeException statusFailureBeforeUpdateFailure;
        private RuntimeException updateFailure;
        private int updateCount;

        @Override
        public void update(LoopClock clock) {
            updateCount++;
            if (updateFailure != null) {
                if (statusBeforeUpdateFailure != null) {
                    current.integrationStatus = statusBeforeUpdateFailure;
                }
                if (statusFailureBeforeUpdateFailure != null) {
                    current.statusFailure = statusFailureBeforeUpdateFailure;
                }
                throw updateFailure;
            }
            if (statusOnUpdate != null && current != null
                    && current.integrationStatus == RouteStatus.ACTIVE) {
                current.integrationStatus = statusOnUpdate;
                statusOnUpdate = null;
            }
        }

        @Override
        public RouteExecution follow(String route) {
            if (current != null && current.integrationStatus == RouteStatus.ACTIVE) {
                current.integrationStatus = RouteStatus.REPLACED;
            }
            current = new RecordingExecution(this);
            return current;
        }
    }

    private static final class RecordingExecution extends RouteExecution {
        private final RecordingFollower owner;
        private RouteStatus integrationStatus = RouteStatus.ACTIVE;
        private int cancelCount;
        private RuntimeException statusFailure;
        private RuntimeException cancelFailure;
        private RouteTask<?> taskToObserveDuringCancel;
        private RouteStatus taskStatusObservedDuringCancel;

        RecordingExecution(RecordingFollower owner) {
            this.owner = owner;
        }

        @Override
        protected RouteStatus integrationStatus() {
            if (statusFailure != null) {
                throw statusFailure;
            }
            return integrationStatus;
        }

        @Override
        protected void cancelActive() {
            if (owner.current != this
                    || (integrationStatus != RouteStatus.ACTIVE
                    && integrationStatus != RouteStatus.NOT_STARTED)) {
                return;
            }
            if (taskToObserveDuringCancel != null) {
                taskStatusObservedDuringCancel = taskToObserveDuringCancel.getRouteStatus();
            }
            integrationStatus = RouteStatus.CANCELLED;
            cancelCount++;
            if (cancelFailure != null) {
                throw cancelFailure;
            }
        }
    }
}
