package edu.ftcphoenix.fw.drive.route;

import org.junit.Test;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;
import edu.ftcphoenix.fw.task.Tasks;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies that dynamic routes are resolved exactly once at their RouteTask start boundary. */
public final class RouteTaskStartTimeConstructionTest {

    @Test
    public void factoryIsNotSampledByConstructionStatusDebugOrPreStartCancellation() {
        ManualLoopClock manualClock = new ManualLoopClock();
        MutableRouteInputs inputs = new MutableRouteInputs(1, "init-left");
        RecordingFollower follower = new RecordingFollower();
        RouteTask<RouteSnapshot> task = RouteTasks.followBuiltAtStartWithoutTaskTimeout(
                "liveReturn",
                follower,
                inputs::build
        );

        assertEquals(0, inputs.buildCount);
        assertEquals(RouteStatus.NOT_STARTED, task.getRouteStatus());
        assertEquals(TaskOutcome.NOT_DONE, task.getOutcome());
        assertFalse(task.isComplete());
        task.debugDump(new CapturingDebugSink(), "route");
        try {
            task.update(manualClock.clock());
            fail("expected update before start to fail");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains("before start"));
        }
        assertEquals(0, inputs.buildCount);
        task.cancel();

        assertEquals(0, inputs.buildCount);
        assertEquals(0, inputs.poseReadCount);
        assertEquals(0, inputs.visionReadCount);
        assertEquals(RouteStatus.NOT_STARTED, task.getRouteStatus());
        assertEquals(0, follower.followCount);

        task.start(manualClock.clock());

        assertEquals(1, inputs.buildCount);
        assertEquals(1, follower.followCount);
        assertEquals(RouteStatus.ACTIVE, task.getRouteStatus());
    }

    @Test
    public void directStartSamplesLatestPoseAndVisionExactlyOnce() {
        ManualLoopClock manualClock = new ManualLoopClock();
        MutableRouteInputs inputs = new MutableRouteInputs(1, "init-left");
        RecordingFollower follower = new RecordingFollower();
        RouteTask<RouteSnapshot> task = RouteTasks.followBuiltAtStartWithoutTaskTimeout(
                "latestInputs",
                follower,
                inputs::build
        );

        inputs.poseVersion = 7;
        inputs.visionSelection = "live-center";
        task.start(manualClock.clock());
        RouteSnapshot resolved = follower.lastRoute;

        assertNotNull(resolved);
        assertEquals(7, resolved.poseVersion);
        assertEquals("live-center", resolved.visionSelection);
        assertEquals(1, inputs.buildCount);
        assertEquals(1, inputs.poseReadCount);
        assertEquals(1, inputs.visionReadCount);
        assertSame(resolved, follower.lastRoute);

        inputs.poseVersion = 9;
        inputs.visionSelection = "later-right";
        manualClock.nextCycle(0.02);
        task.update(manualClock.clock());
        task.debugDump(new CapturingDebugSink(), "route");

        assertEquals(1, inputs.buildCount);
        assertEquals(1, inputs.poseReadCount);
        assertEquals(1, inputs.visionReadCount);
        assertSame(resolved, follower.lastRoute);
    }

    @Test
    public void sequenceSamplesWhenDynamicChildActuallyStartsInLaterLoop() {
        ManualLoopClock manualClock = new ManualLoopClock();
        AtomicBoolean releaseFirstChild = new AtomicBoolean(false);
        MutableRouteInputs inputs = new MutableRouteInputs(2, "init-left");
        RecordingFollower follower = new RecordingFollower();
        final long[] sampledCycle = {Long.MIN_VALUE};
        RouteTask<RouteSnapshot> dynamicRoute =
                RouteTasks.followBuiltAtStartWithoutTaskTimeout(
                        "sequenceReturn",
                        follower,
                        () -> {
                            sampledCycle[0] = manualClock.clock().cycle();
                            return inputs.build();
                        }
                );
        Task sequence = Tasks.sequence(
                Tasks.waitUntil(releaseFirstChild::get),
                dynamicRoute
        );

        sequence.start(manualClock.clock());
        assertEquals(0, inputs.buildCount);

        inputs.poseVersion = 12;
        inputs.visionSelection = "sub-loop-right";
        releaseFirstChild.set(true);
        manualClock.nextCycle(0.02);
        sequence.update(manualClock.clock());

        assertEquals(1, inputs.buildCount);
        assertEquals(manualClock.clock().cycle(), sampledCycle[0]);
        assertEquals(12, follower.lastRoute.poseVersion);
        assertEquals("sub-loop-right", follower.lastRoute.visionSelection);
        assertEquals(RouteStatus.ACTIVE, dynamicRoute.getRouteStatus());
        assertFalse(sequence.isComplete());
    }

    @Test
    public void eagerFollowStillUsesTheConcreteRouteSuppliedAtConstruction() {
        RecordingFollower follower = new RecordingFollower();
        RouteSnapshot eagerRoute = new RouteSnapshot(3, "fixed");
        RouteTask<RouteSnapshot> task = RouteTasks.followWithoutTaskTimeout(
                "fixedOutbound",
                follower,
                eagerRoute
        );

        task.start(new ManualLoopClock().clock());

        assertSame(eagerRoute, follower.lastRoute);
        assertEquals(1, follower.followCount);
        assertEquals(RouteStatus.ACTIVE, task.getRouteStatus());
        CapturingDebugSink debug = new CapturingDebugSink();
        task.debugDump(debug, "route");
        assertEquals("eager", debug.data.get("route.routeSource"));
        assertEquals("RouteSnapshot", debug.data.get("route.routeClass"));
    }

    @Test
    public void nullFactoryIsRejectedBeforeATaskCanBeBuilt() {
        RecordingFollower follower = new RecordingFollower();

        try {
            RouteTasks.followBuiltAtStartWithoutTaskTimeout(
                    "missingFactory",
                    follower,
                    (Supplier<? extends RouteSnapshot>) null
            );
            fail("expected a null route factory to be rejected");
        } catch (RuntimeException expected) {
            String message = String.valueOf(expected.getMessage()).toLowerCase();
            assertTrue(message, message.contains("factory") || message.contains("supplier"));
        }

        assertEquals(0, follower.followCount);
    }

    @Test
    public void nullFactoryResultFailsClosedWithoutTouchingFollower() {
        RecordingFollower follower = new RecordingFollower();
        final int[] factoryCalls = {0};
        RouteTask<RouteSnapshot> task = RouteTasks.followBuiltAtStartWithoutTaskTimeout(
                "nullRouteResult",
                follower,
                () -> {
                    factoryCalls[0]++;
                    return null;
                }
        );

        try {
            task.start(new ManualLoopClock().clock());
            fail("expected a null route result to fail");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains("nullRouteResult"));
            assertTrue(expected.getMessage(), expected.getMessage().contains("followBuiltAtStart"));
            assertTrue(expected.getMessage(), expected.getMessage().contains("returned null"));
        }

        assertEquals(1, factoryCalls[0]);
        assertEquals(0, follower.followCount);
        assertEquals(RouteStatus.FAILED, task.getRouteStatus());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertTrue(task.isComplete());
        CapturingDebugSink debug = new CapturingDebugSink();
        task.debugDump(debug, "route");
        assertEquals("builtAtStart", debug.data.get("route.routeSource"));
        assertEquals("pending", debug.data.get("route.routeClass"));
        assertEquals(1, factoryCalls[0]);
    }

    @Test
    public void throwingFactoryFailsClosedWithTaskNamedErrorAndOriginalCause() {
        RecordingFollower follower = new RecordingFollower();
        IllegalStateException factoryFailure = new IllegalStateException("vision snapshot missing");
        RouteTask<RouteSnapshot> task = RouteTasks.followBuiltAtStartWithoutTaskTimeout(
                "throwingReturnFactory",
                follower,
                () -> {
                    throw factoryFailure;
                }
        );

        try {
            task.start(new ManualLoopClock().clock());
            fail("expected route factory failure");
        } catch (IllegalStateException actual) {
            assertTrue(actual.getMessage(), actual.getMessage().contains("throwingReturnFactory"));
            assertTrue(actual.getMessage(), actual.getMessage().contains("vision snapshot missing"));
            assertSame(factoryFailure, actual.getCause());
        }

        assertEquals(0, follower.followCount);
        assertEquals(RouteStatus.FAILED, task.getRouteStatus());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertTrue(task.isComplete());
    }

    @Test
    public void secondStartAfterFactoryFailureCannotResampleFactory() {
        RecordingFollower follower = new RecordingFollower();
        final int[] factoryCalls = {0};
        RouteTask<RouteSnapshot> task = RouteTasks.followBuiltAtStartWithoutTaskTimeout(
                "singleUseFactoryFailure",
                follower,
                () -> {
                    factoryCalls[0]++;
                    throw new IllegalStateException("cannot build route");
                }
        );
        ManualLoopClock manualClock = new ManualLoopClock();

        try {
            task.start(manualClock.clock());
            fail("expected first factory failure");
        } catch (IllegalStateException expected) {
            assertEquals(1, factoryCalls[0]);
        }

        try {
            task.start(manualClock.clock());
            fail("expected second start to fail single-use validation");
        } catch (IllegalStateException expected) {
            String message = expected.getMessage();
            assertTrue(message, message.contains("single-use") || message.contains("already"));
        }

        assertEquals(1, factoryCalls[0]);
        assertEquals(0, follower.followCount);
        assertEquals(RouteStatus.FAILED, task.getRouteStatus());
    }

    @Test
    public void activeAndRepeatedCancellationCancelResolvedExecutionOnce() {
        RecordingFollower follower = new RecordingFollower();
        MutableRouteInputs inputs = new MutableRouteInputs(4, "center");
        RouteTask<RouteSnapshot> task = RouteTasks.followBuiltAtStartWithoutTaskTimeout(
                "cancelDynamicRoute",
                follower,
                inputs::build
        );
        task.start(new ManualLoopClock().clock());
        RecordingExecution execution = follower.current;

        task.cancel();
        task.cancel();

        assertEquals(1, inputs.buildCount);
        assertEquals(1, execution.cancelCount);
        assertEquals(RouteStatus.CANCELLED, task.getRouteStatus());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertTrue(task.isComplete());
    }

    @Test
    public void reentrantCancellationDuringFactoryPreventsFollowerStart() {
        RecordingFollower follower = new RecordingFollower();
        AtomicReference<RouteTask<RouteSnapshot>> taskRef = new AtomicReference<>();
        final int[] factoryCalls = {0};
        RouteTask<RouteSnapshot> task = RouteTasks.followBuiltAtStartWithoutTaskTimeout(
                "cancelDuringFactory",
                follower,
                () -> {
                    factoryCalls[0]++;
                    taskRef.get().cancel();
                    return new RouteSnapshot(5, "cancelled");
                }
        );
        taskRef.set(task);

        task.start(new ManualLoopClock().clock());

        assertEquals(1, factoryCalls[0]);
        assertEquals(0, follower.followCount);
        assertEquals(RouteStatus.CANCELLED, task.getRouteStatus());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertTrue(task.isComplete());
    }

    @Test
    public void reentrantCancellationDuringFollowerStartCancelsReturnedExecutionOnce() {
        AtomicReference<RouteTask<RouteSnapshot>> taskRef = new AtomicReference<>();
        AtomicReference<RecordingExecution> executionRef = new AtomicReference<>();
        final int[] followCalls = {0};
        RouteFollower<RouteSnapshot> follower = route -> {
            followCalls[0]++;
            RecordingExecution execution = new RecordingExecution();
            executionRef.set(execution);
            taskRef.get().cancel();
            return execution;
        };
        RouteTask<RouteSnapshot> task = RouteTasks.followBuiltAtStartWithoutTaskTimeout(
                "cancelDuringFollow",
                follower,
                () -> new RouteSnapshot(6, "cancelled-during-follow")
        );
        taskRef.set(task);

        task.start(new ManualLoopClock().clock());

        RecordingExecution execution = executionRef.get();
        assertNotNull(execution);
        assertEquals(1, followCalls[0]);
        assertEquals(1, execution.cancelCount);
        assertEquals(RouteStatus.CANCELLED, execution.status());
        assertEquals(RouteStatus.CANCELLED, task.getRouteStatus());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertTrue(task.isComplete());

        task.cancel();
        assertEquals(1, execution.cancelCount);
    }

    @Test
    public void terminalExecutionReturnedAfterReentrantCancelKeepsTruthfulStatus() {
        AtomicReference<RouteTask<RouteSnapshot>> taskRef = new AtomicReference<>();
        AtomicReference<RecordingExecution> executionRef = new AtomicReference<>();
        RouteFollower<RouteSnapshot> follower = route -> {
            RecordingExecution execution = new RecordingExecution();
            execution.integrationStatus = RouteStatus.COMPLETED;
            executionRef.set(execution);
            taskRef.get().cancel();
            return execution;
        };
        RouteTask<RouteSnapshot> task = RouteTasks.followBuiltAtStartWithoutTaskTimeout(
                "completeDuringFollow",
                follower,
                () -> new RouteSnapshot(7, "complete-during-follow")
        );
        taskRef.set(task);

        task.start(new ManualLoopClock().clock());

        RecordingExecution execution = executionRef.get();
        assertNotNull(execution);
        assertEquals(0, execution.cancelCount);
        assertEquals(RouteStatus.COMPLETED, execution.status());
        assertEquals(RouteStatus.COMPLETED, task.getRouteStatus());
        assertEquals(TaskOutcome.SUCCESS, task.getOutcome());
        assertTrue(task.isComplete());
    }

    @Test
    public void followerFailureAfterReentrantCancelKeepsCancellationTerminal() {
        AtomicReference<RouteTask<RouteSnapshot>> taskRef = new AtomicReference<>();
        IllegalStateException followFailure = new IllegalStateException("route start failed closed");
        RouteFollower<RouteSnapshot> follower = route -> {
            taskRef.get().cancel();
            throw followFailure;
        };
        RouteTask<RouteSnapshot> task = RouteTasks.followBuiltAtStartWithoutTaskTimeout(
                "cancelThenFailFollow",
                follower,
                () -> new RouteSnapshot(8, "cancel-then-fail")
        );
        taskRef.set(task);

        try {
            task.start(new ManualLoopClock().clock());
            fail("expected follower start failure");
        } catch (IllegalStateException actual) {
            assertSame(followFailure, actual);
        }

        assertEquals(RouteStatus.CANCELLED, task.getRouteStatus());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertTrue(task.isComplete());
        task.cancel();
        assertEquals(RouteStatus.CANCELLED, task.getRouteStatus());
    }

    @Test
    public void taskTimeoutRetainsTaskReasonAndCancelsResolvedExecutionOnce() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingFollower follower = new RecordingFollower();
        RouteTask<RouteSnapshot> task = RouteTasks.followBuiltAtStart(
                "dynamicTimeout",
                follower,
                () -> new RouteSnapshot(6, "timeout"),
                0.05
        );
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
    public void integrationTerminalStatusesRemainPreciseForBuiltAtStartRoutes() {
        RouteStatus[] terminalStatuses = {
                RouteStatus.COMPLETED,
                RouteStatus.FOLLOWER_TIMEOUT_OR_STALL,
                RouteStatus.INTERRUPTED,
                RouteStatus.REPLACED,
                RouteStatus.CANCELLED,
                RouteStatus.FAILED,
                RouteStatus.UNKNOWN_TERMINAL
        };

        for (RouteStatus terminalStatus : terminalStatuses) {
            RecordingFollower follower = new RecordingFollower();
            RouteTask<RouteSnapshot> task =
                    RouteTasks.followBuiltAtStartWithoutTaskTimeout(
                            "dynamic-" + terminalStatus,
                            follower,
                            () -> new RouteSnapshot(8, terminalStatus.name())
                    );
            ManualLoopClock manualClock = new ManualLoopClock();
            task.start(manualClock.clock());
            RecordingExecution execution = follower.current;
            execution.integrationStatus = terminalStatus;

            task.update(manualClock.clock());
            task.cancel();

            assertEquals(terminalStatus, task.getRouteStatus());
            assertEquals(expectedOutcome(terminalStatus), task.getOutcome());
            assertTrue(task.isComplete());
            assertEquals(terminalStatus, execution.status());
            assertEquals(0, execution.cancelCount);
        }
    }

    @Test
    public void debugShowsPendingAndResolvedRouteWithoutAdditionalSampling() {
        MutableRouteInputs inputs = new MutableRouteInputs(10, "debug");
        RecordingFollower follower = new RecordingFollower();
        RouteTask<RouteSnapshot> task = RouteTasks.followBuiltAtStartWithoutTaskTimeout(
                "debugDynamicRoute",
                follower,
                inputs::build
        );
        CapturingDebugSink before = new CapturingDebugSink();

        task.debugDump(before, "route");

        assertEquals(0, inputs.buildCount);
        assertEquals("builtAtStart", before.data.get("route.routeSource"));
        assertEquals("pending", before.data.get("route.routeClass"));
        assertEquals("none", before.data.get("route.timeoutSec"));

        task.start(new ManualLoopClock().clock());
        CapturingDebugSink after = new CapturingDebugSink();
        task.debugDump(after, "route");
        task.debugDump(after, "route");

        assertEquals(1, inputs.buildCount);
        assertEquals("builtAtStart", after.data.get("route.routeSource"));
        assertEquals("RouteSnapshot", after.data.get("route.routeClass"));
        assertEquals("none", after.data.get("route.timeoutSec"));
    }

    @Test
    public void debugKeepsStableTimeoutKeyForBoundedRoute() {
        RecordingFollower follower = new RecordingFollower();
        RouteTask<RouteSnapshot> task = RouteTasks.follow(
                "debugBoundedRoute",
                follower,
                new RouteSnapshot(11, "bounded"),
                2.5
        );
        CapturingDebugSink debug = new CapturingDebugSink();

        task.debugDump(debug, "route");

        assertEquals(Double.valueOf(2.5), debug.data.get("route.timeoutSec"));
    }

    private static TaskOutcome expectedOutcome(RouteStatus terminalStatus) {
        if (terminalStatus == RouteStatus.COMPLETED) {
            return TaskOutcome.SUCCESS;
        }
        if (terminalStatus == RouteStatus.FOLLOWER_TIMEOUT_OR_STALL
                || terminalStatus == RouteStatus.TASK_TIMEOUT) {
            return TaskOutcome.TIMEOUT;
        }
        return TaskOutcome.CANCELLED;
    }

    private static final class MutableRouteInputs {
        private int poseVersion;
        private String visionSelection;
        private int buildCount;
        private int poseReadCount;
        private int visionReadCount;

        MutableRouteInputs(int poseVersion, String visionSelection) {
            this.poseVersion = poseVersion;
            this.visionSelection = visionSelection;
        }

        RouteSnapshot build() {
            buildCount++;
            poseReadCount++;
            int sampledPoseVersion = poseVersion;
            visionReadCount++;
            String sampledVisionSelection = visionSelection;
            return new RouteSnapshot(sampledPoseVersion, sampledVisionSelection);
        }
    }

    private static final class RouteSnapshot {
        private final int poseVersion;
        private final String visionSelection;

        RouteSnapshot(int poseVersion, String visionSelection) {
            this.poseVersion = poseVersion;
            this.visionSelection = visionSelection;
        }
    }

    private static final class RecordingFollower implements RouteFollower<RouteSnapshot> {
        private RouteSnapshot lastRoute;
        private RecordingExecution current;
        private int followCount;
        private int updateCount;

        @Override
        public RouteExecution follow(RouteSnapshot route) {
            lastRoute = route;
            followCount++;
            current = new RecordingExecution();
            return current;
        }

        @Override
        public void update(LoopClock clock) {
            updateCount++;
        }
    }

    private static final class RecordingExecution extends RouteExecution {
        private RouteStatus integrationStatus = RouteStatus.ACTIVE;
        private int cancelCount;

        @Override
        protected RouteStatus integrationStatus() {
            return integrationStatus;
        }

        @Override
        protected void cancelActive() {
            integrationStatus = RouteStatus.CANCELLED;
            cancelCount++;
        }
    }

    private static final class CapturingDebugSink implements DebugSink {
        private final Map<String, Object> data = new LinkedHashMap<>();

        @Override
        public DebugSink addData(String key, Object value) {
            data.put(key, value);
            return this;
        }

        @Override
        public DebugSink addLine(String text) {
            return this;
        }
    }
}
