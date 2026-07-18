package edu.ftcphoenix.fw.integrations.pedro;

import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.paths.callbacks.PathCallback;

import org.junit.Test;

import java.util.Arrays;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidance;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidancePlan;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidanceTask;
import edu.ftcphoenix.fw.drive.route.RouteExecution;
import edu.ftcphoenix.fw.drive.route.RouteStatus;
import edu.ftcphoenix.fw.drive.route.RouteTask;
import edu.ftcphoenix.fw.drive.route.RouteTasks;
import edu.ftcphoenix.fw.localization.AbsolutePoseEstimator;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.task.TaskOutcome;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/**
 * Version-pinned integration coverage for Phoenix's Pedro 2.1.2 lifecycle owner.
 *
 * <p>The tests use Pedro's real {@link Follower}, path, callback, and controller code. Only its
 * hardware-facing {@link Localizer} and {@link Drivetrain} boundaries are replaced with small JVM
 * fakes, so {@link RecordingLocalizer#updateCount} counts real Pedro heartbeats, including the
 * hidden update performed by {@link Follower#startTeleopDrive()}.</p>
 */
public final class PedroPathingDriveAdapterTest {

    private static final double EPSILON = 1e-9;
    private static final Pose START = new Pose(0.0, 0.0, 0.0);
    private static final Pose MID = new Pose(6.0, 0.0, 0.0);
    private static final Pose END = new Pose(12.0, 0.0, 0.0);

    @Test
    public void rootAndRouteTaskUpdatesAdvanceFollowerOncePerCycle() {
        Fixture fixture = new Fixture(START);
        RouteTask<PathChain> task = RouteTasks.followWithoutTaskTimeout(
                "deduplicated route",
                fixture.adapter,
                lineRoute()
        );

        task.start(fixture.clock.clock());
        fixture.adapter.update(fixture.clock.clock());
        task.update(fixture.clock.clock());

        assertEquals(1, fixture.localizer.updateCount);
        assertEquals(1, fixture.drivetrain.updateConstantsCount);
        assertFalse(task.isComplete());
        assertEquals(RouteStatus.ACTIVE, task.getRouteStatus());

        fixture.clock.nextCycle(0.02);
        task.update(fixture.clock.clock());
        fixture.adapter.update(fixture.clock.clock());

        assertEquals(2, fixture.localizer.updateCount);
        assertEquals(2, fixture.drivetrain.updateConstantsCount);
        assertFalse(task.isComplete());
    }

    @Test
    public void holdEndContinuesAfterRouteTaskCompletesAndOnlyRootKeepsUpdating() {
        Fixture fixture = new Fixture(END);
        RouteTask<PathChain> task = RouteTasks.follow(
                "hold-end route",
                fixture.adapter,
                lineRoute(),
                1.0
        );

        task.start(fixture.clock.clock());
        fixture.adapter.update(fixture.clock.clock());
        task.update(fixture.clock.clock());

        assertTrue(task.isComplete());
        assertEquals(TaskOutcome.SUCCESS, task.getOutcome());
        assertEquals(RouteStatus.COMPLETED, task.getRouteStatus());
        assertEquals(RouteStatus.COMPLETED, fixture.adapter.getLatestRouteStatus());
        assertFalse(fixture.follower.isBusy());
        assertEquals(1, fixture.localizer.updateCount);
        assertZero(fixture.drivetrain.lastDrivePowers);

        int runCountAfterCompletion = fixture.drivetrain.runDriveCount;
        fixture.drivetrain.clearDriveObservation();
        fixture.localizer.pose = new Pose(END.getX() - 1.0, END.getY(), END.getHeading());
        fixture.clock.nextCycle(0.02);
        fixture.adapter.update(fixture.clock.clock());

        assertEquals(2, fixture.localizer.updateCount);
        assertEquals(runCountAfterCompletion + 1, fixture.drivetrain.runDriveCount);
        assertTrue(fixture.drivetrain.sawNonZeroDrive);
        assertFalse(fixture.follower.isBusy());
    }

    @Test
    public void explicitNoHoldOverridesPedrosDefaultHold() {
        Fixture fixture = new Fixture(END);

        RouteExecution execution = fixture.adapter.follow(lineRoute(), false);
        fixture.adapter.update(fixture.clock.clock());

        assertEquals(RouteStatus.COMPLETED, execution.status());
        assertFalse(fixture.follower.isBusy());
        assertZero(fixture.drivetrain.lastDrivePowers);

        int runCountAfterCompletion = fixture.drivetrain.runDriveCount;
        fixture.drivetrain.clearDriveObservation();
        fixture.localizer.pose = new Pose(END.getX() - 1.0, END.getY(), END.getHeading());
        fixture.clock.nextCycle(0.02);
        fixture.adapter.update(fixture.clock.clock());

        assertEquals(runCountAfterCompletion, fixture.drivetrain.runDriveCount);
        assertFalse(fixture.drivetrain.sawNonZeroDrive);
        assertZero(fixture.drivetrain.lastDrivePowers);
    }

    @Test
    public void naturalCompletionAtPedroParametricThresholdDoesNotRequireExactEndpoint() {
        Fixture fixture = new Fixture(new Pose(11.9, 0.0, 0.0));
        RouteExecution execution = fixture.adapter.follow(parametricThresholdRoute(), false);

        fixture.adapter.update(fixture.clock.clock());

        assertEquals(RouteStatus.COMPLETED, execution.status());
        assertEquals(RouteStatus.COMPLETED, fixture.adapter.getLatestRouteStatus());
        assertFalse(fixture.follower.isBusy());
        assertZero(fixture.drivetrain.lastDrivePowers);
    }

    @Test
    public void routeToManualTransitionCountsPedrosHiddenUpdateAsTheHeartbeat() {
        Fixture fixture = new Fixture(END);
        DriveSignal requested = new DriveSignal(0.50, -0.25, 0.20);

        RouteExecution execution = fixture.adapter.follow(lineRoute(), true);
        fixture.adapter.update(fixture.clock.clock());
        assertEquals(RouteStatus.COMPLETED, execution.status());
        assertFalse(fixture.follower.isBusy());
        assertEquals(1, fixture.localizer.updateCount);

        fixture.adapter.drive(requested);
        assertZero(fixture.drivetrain.lastDrivePowers);

        fixture.clock.nextCycle(0.02);
        fixture.adapter.update(fixture.clock.clock());
        fixture.adapter.update(fixture.clock.clock());

        assertEquals(2, fixture.localizer.updateCount);
        assertEquals(1, fixture.drivetrain.startTeleopDriveCount);
        assertTrue(fixture.follower.isTeleopDrive());
        assertZero(fixture.drivetrain.lastDrivePowers);

        fixture.drivetrain.clearDriveObservation();
        fixture.clock.nextCycle(0.02);
        fixture.adapter.update(fixture.clock.clock());

        assertEquals(3, fixture.localizer.updateCount);
        assertEquals(1, fixture.drivetrain.startTeleopDriveCount);
        assertTrue(fixture.drivetrain.sawNonZeroDrive);
    }

    @Test
    public void guidanceSharesRootHeartbeatAndLeavesLaterScoringWaitStopped() {
        Fixture fixture = new Fixture(START);
        MutablePhoenixPoseEstimator estimator = new MutablePhoenixPoseEstimator();
        estimator.setPose(0.0, fixture.clock.clock().nowSec());
        DriveGuidancePlan plan = DriveGuidance.plan()
                .translateTo()
                    .fieldPointInches(12.0, 0.0)
                .solveWith()
                    .localizationOnlyWithDefaults(estimator)
                .build();
        DriveGuidanceTask.Config config = new DriveGuidanceTask.Config();
        config.positionTolInches = 0.25;
        config.timeoutSec = 5.0;
        DriveGuidanceTask task = new DriveGuidanceTask(fixture.adapter, plan, config);

        // Phoenix's root heartbeat precedes the runner that starts and updates guidance.
        fixture.adapter.update(fixture.clock.clock());
        task.start(fixture.clock.clock());
        task.update(fixture.clock.clock());
        assertEquals(1, fixture.localizer.updateCount);
        assertFalse(task.isComplete());

        fixture.clock.nextCycle(0.02);
        fixture.adapter.update(fixture.clock.clock());
        task.update(fixture.clock.clock());
        assertEquals(2, fixture.localizer.updateCount);
        assertEquals(1, fixture.drivetrain.startTeleopDriveCount);
        assertZero(fixture.drivetrain.lastDrivePowers);

        fixture.drivetrain.clearDriveObservation();
        fixture.clock.nextCycle(0.02);
        fixture.adapter.update(fixture.clock.clock());
        task.update(fixture.clock.clock());
        assertEquals(3, fixture.localizer.updateCount);
        assertTrue(fixture.drivetrain.sawNonZeroDrive);

        estimator.setPose(12.0, fixture.clock.clock().nowSec());
        fixture.clock.nextCycle(0.02);
        fixture.adapter.update(fixture.clock.clock());
        task.update(fixture.clock.clock());

        assertEquals(4, fixture.localizer.updateCount);
        assertTrue(task.isComplete());
        assertEquals(TaskOutcome.SUCCESS, task.getOutcome());
        assertZero(fixture.drivetrain.lastDrivePowers);

        fixture.drivetrain.clearDriveObservation();
        fixture.clock.nextCycle(0.02);
        fixture.adapter.update(fixture.clock.clock());
        fixture.clock.nextCycle(0.02);
        fixture.adapter.update(fixture.clock.clock());

        assertEquals(6, fixture.localizer.updateCount);
        assertFalse(fixture.drivetrain.sawNonZeroDrive);
        assertZero(fixture.drivetrain.lastDrivePowers);
    }

    @Test
    public void stopIsImmediateStableAndIdempotentAcrossLaterHeartbeats() {
        Fixture fixture = new Fixture(START);

        enterNonZeroManualDrive(fixture);
        assertTrue(fixture.drivetrain.sawNonZeroDrive);

        fixture.adapter.stop();
        assertZero(fixture.drivetrain.lastDrivePowers);
        assertFalse(fixture.follower.isTeleopDrive());
        int updateCountAtStop = fixture.localizer.updateCount;

        fixture.adapter.stop();
        fixture.adapter.stop();

        assertEquals(updateCountAtStop, fixture.localizer.updateCount);
        assertZero(fixture.drivetrain.lastDrivePowers);

        fixture.drivetrain.clearDriveObservation();
        fixture.clock.nextCycle(0.02);
        fixture.adapter.update(fixture.clock.clock());
        assertEquals(updateCountAtStop + 1, fixture.localizer.updateCount);
        assertZero(fixture.drivetrain.lastDrivePowers);
        assertFalse(fixture.drivetrain.sawNonZeroDrive);

        fixture.clock.nextCycle(0.02);
        fixture.adapter.update(fixture.clock.clock());
        fixture.adapter.stop();
        assertEquals(updateCountAtStop + 2, fixture.localizer.updateCount);
        assertZero(fixture.drivetrain.lastDrivePowers);
        assertFalse(fixture.drivetrain.sawNonZeroDrive);
    }

    @Test
    public void endpointCancelCannotResurrectRetainedRouteOrHold() {
        Fixture fixture = new Fixture(END);

        RouteExecution execution = fixture.adapter.follow(lineRoute(), true);
        execution.cancel();

        assertEquals(RouteStatus.CANCELLED, execution.status());
        assertFalse(fixture.follower.isBusy());
        assertEquals(0, fixture.localizer.updateCount);
        assertZero(fixture.drivetrain.lastDrivePowers);

        fixture.localizer.pose = new Pose(END.getX() - 1.0, END.getY(), END.getHeading());
        fixture.drivetrain.clearDriveObservation();
        fixture.adapter.update(fixture.clock.clock());
        fixture.clock.nextCycle(0.02);
        fixture.adapter.update(fixture.clock.clock());

        assertEquals(2, fixture.localizer.updateCount);
        assertFalse(fixture.follower.isBusy());
        assertFalse(fixture.drivetrain.sawNonZeroDrive);
        assertZero(fixture.drivetrain.lastDrivePowers);
    }

    @Test
    public void routeTimeoutStopsImmediatelyWithoutASecondTaskHeartbeat() {
        Fixture fixture = new Fixture(START);
        RouteTask<PathChain> task = RouteTasks.follow(
                "timed route",
                fixture.adapter,
                lineRoute(),
                0.10
        );

        task.start(fixture.clock.clock());
        fixture.adapter.update(fixture.clock.clock());
        task.update(fixture.clock.clock());
        assertEquals(1, fixture.localizer.updateCount);
        assertFalse(task.isComplete());

        fixture.clock.nextCycle(0.11);
        fixture.adapter.update(fixture.clock.clock());
        task.update(fixture.clock.clock());

        assertTrue(task.isComplete());
        assertEquals(TaskOutcome.TIMEOUT, task.getOutcome());
        assertEquals(RouteStatus.TASK_TIMEOUT, task.getRouteStatus());
        assertEquals(2, fixture.localizer.updateCount);
        assertFalse(fixture.follower.isBusy());
        assertZero(fixture.drivetrain.lastDrivePowers);

        fixture.drivetrain.clearDriveObservation();
        fixture.clock.nextCycle(0.02);
        fixture.adapter.update(fixture.clock.clock());

        assertEquals(3, fixture.localizer.updateCount);
        assertFalse(fixture.drivetrain.sawNonZeroDrive);
        assertZero(fixture.drivetrain.lastDrivePowers);
    }

    @Test
    public void impossibleEndpointConstraintsReportFollowerTimeoutInsteadOfCompletion() {
        Fixture fixture = new Fixture(END);
        RouteExecution execution = fixture.adapter.follow(endpointTimeoutRoute(), false);

        fixture.adapter.update(fixture.clock.clock());

        assertEquals(RouteStatus.FOLLOWER_TIMEOUT_OR_STALL, execution.status());
        assertEquals(
                RouteStatus.FOLLOWER_TIMEOUT_OR_STALL,
                fixture.adapter.getLatestRouteStatus()
        );
        assertFalse(fixture.follower.isBusy());
        assertZero(fixture.drivetrain.lastDrivePowers);
    }

    @Test
    public void endpointClassificationUsesConstraintsMutatedBySameHeartbeatCallback() {
        Fixture fixture = new Fixture(END);
        PathChain route = lineRoute();
        Path finalPath = route.getPath(route.size() - 1);
        int[] callbackCount = {0};
        route.setCallbacks(new OneShotCallback(() -> {
            callbackCount[0]++;
            finalPath.setVelocityConstraint(-1.0);
            finalPath.setTranslationalConstraint(-1.0);
            finalPath.setHeadingConstraint(-1.0);
            finalPath.setTimeoutConstraint(-1.0);
        }));
        RouteExecution execution = fixture.adapter.follow(route, false);

        fixture.adapter.update(fixture.clock.clock());

        assertEquals(1, callbackCount[0]);
        assertEquals(RouteStatus.FOLLOWER_TIMEOUT_OR_STALL, execution.status());
        assertFalse(fixture.follower.isBusy());
        assertZero(fixture.drivetrain.lastDrivePowers);
    }

    @Test
    public void endpointClassificationUsesHeadingGoalCachedByPedroBeforeCallback() {
        Fixture fixture = new Fixture(END);
        PathChain route = lineRoute();
        Path finalPath = route.getPath(route.size() - 1);
        finalPath.setConstantHeadingInterpolation(Math.PI / 2.0);
        finalPath.setTimeoutConstraint(-1.0);
        int[] callbackCount = {0};
        route.setCallbacks(new OneShotCallback(() -> {
            callbackCount[0]++;
            finalPath.setConstantHeadingInterpolation(0.0);
        }));
        RouteExecution execution = fixture.adapter.follow(route, false);

        fixture.adapter.update(fixture.clock.clock());

        assertEquals(1, callbackCount[0]);
        assertEquals(RouteStatus.FOLLOWER_TIMEOUT_OR_STALL, execution.status());
        assertFalse(fixture.follower.isBusy());
        assertZero(fixture.drivetrain.lastDrivePowers);
    }

    @Test
    public void intermediateStallStopsTheWholeRouteInsteadOfSkippingAPath() {
        Fixture fixture = new Fixture(START, immediateStallConstants());
        RouteExecution execution = fixture.adapter.follow(twoSegmentRoute(), false);

        fixture.adapter.update(fixture.clock.clock());

        assertEquals(RouteStatus.FOLLOWER_TIMEOUT_OR_STALL, execution.status());
        assertFalse(fixture.follower.isBusy());
        assertZero(fixture.drivetrain.lastDrivePowers);
    }

    @Test
    public void finalPathStallReportsFollowerTimeoutWithoutWaitingForEndpointTimeout() {
        Fixture fixture = new Fixture(MID, immediateStallConstants());
        RouteExecution execution = fixture.adapter.follow(lineRoute(), false);

        fixture.adapter.update(fixture.clock.clock());

        assertEquals(RouteStatus.FOLLOWER_TIMEOUT_OR_STALL, execution.status());
        assertFalse(fixture.follower.isBusy());
        assertZero(fixture.drivetrain.lastDrivePowers);
    }

    @Test
    public void manualTakeoverInterruptsOnlyTheActiveRoute() {
        Fixture fixture = new Fixture(START);
        RouteExecution execution = fixture.adapter.follow(lineRoute(), false);

        fixture.adapter.drive(new DriveSignal(0.40, 0.10, -0.20));

        assertEquals(RouteStatus.INTERRUPTED, execution.status());
        assertFalse(fixture.follower.isBusy());
        assertZero(fixture.drivetrain.lastDrivePowers);

        execution.cancel();
        assertEquals(RouteStatus.INTERRUPTED, execution.status());
    }

    @Test
    public void replacementRetainsOldStatusAndOldCancelCannotStopNewRoute() {
        Fixture fixture = new Fixture(START);
        PathChain firstRoute = lineRoute();
        PathChain secondRoute = lineRoute();
        RouteExecution first = fixture.adapter.follow(firstRoute, false);

        RouteExecution second = fixture.adapter.follow(secondRoute, false);

        assertEquals(RouteStatus.REPLACED, first.status());
        assertEquals(RouteStatus.ACTIVE, second.status());
        assertEquals(RouteStatus.ACTIVE, fixture.adapter.getLatestRouteStatus());
        assertSame(secondRoute, fixture.follower.getCurrentPathChain());
        assertTrue(fixture.follower.isBusy());

        int breakCountAfterReplacement = fixture.drivetrain.breakFollowingCount;
        first.cancel();

        assertEquals(RouteStatus.REPLACED, first.status());
        assertEquals(RouteStatus.ACTIVE, second.status());
        assertEquals(RouteStatus.ACTIVE, fixture.adapter.getLatestRouteStatus());
        assertEquals(breakCountAfterReplacement, fixture.drivetrain.breakFollowingCount);
        assertSame(secondRoute, fixture.follower.getCurrentPathChain());
        assertTrue(fixture.follower.isBusy());

        fixture.localizer.pose = END;
        fixture.clock.nextCycle(0.02);
        fixture.adapter.update(fixture.clock.clock());

        assertEquals(RouteStatus.REPLACED, first.status());
        assertEquals(RouteStatus.COMPLETED, second.status());
        assertEquals(RouteStatus.COMPLETED, fixture.adapter.getLatestRouteStatus());
    }

    @Test
    public void unsupportedRawFollowerBreakReportsUnknownTerminal() {
        Fixture fixture = new Fixture(START);
        RouteExecution execution = fixture.adapter.follow(lineRoute(), false);

        fixture.follower.breakFollowing();
        fixture.adapter.update(fixture.clock.clock());

        assertEquals(RouteStatus.UNKNOWN_TERMINAL, execution.status());
        assertFalse(fixture.follower.isBusy());
        assertZero(fixture.drivetrain.lastDrivePowers);
    }

    @Test
    public void routeUpdateFailureRetainsFailedStatusAndFailsClosed() {
        Fixture fixture = new Fixture(START);
        RouteExecution execution = fixture.adapter.follow(lineRoute(), false);
        RuntimeException expected = new IllegalStateException("test route update failure");
        fixture.localizer.throwOnNextUpdate = expected;

        try {
            fixture.adapter.update(fixture.clock.clock());
            fail("Expected the route heartbeat to propagate the localizer failure");
        } catch (RuntimeException actual) {
            assertSame(expected, actual);
        }

        assertEquals(RouteStatus.FAILED, execution.status());
        assertEquals(RouteStatus.FAILED, fixture.adapter.getLatestRouteStatus());
        assertFalse(fixture.follower.isBusy());
        assertZero(fixture.drivetrain.lastDrivePowers);
    }

    @Test
    public void snapshotGetterFailureFailsClosedAndDoesNotStrandHeartbeatGuard() {
        RecordingLocalizer localizer = new RecordingLocalizer(START);
        RecordingDrivetrain drivetrain = new RecordingDrivetrain();
        SnapshotThrowingFollower follower = new SnapshotThrowingFollower(localizer, drivetrain);
        PedroPathingDriveAdapter adapter = new PedroPathingDriveAdapter(follower);
        localizer.clearObservations();
        drivetrain.clearObservations();
        ManualLoopClock clock = new ManualLoopClock();
        RouteExecution execution = adapter.follow(lineRoute(), false);
        RuntimeException snapshotFailure = new IllegalStateException("test snapshot failure");
        follower.throwOnNextCurrentPath = snapshotFailure;

        try {
            adapter.update(clock.clock());
            fail("Expected snapshot getter failure");
        } catch (RuntimeException actual) {
            assertSame(snapshotFailure, actual);
        }

        assertEquals(RouteStatus.FAILED, execution.status());
        assertFalse(follower.isBusy());
        assertEquals(0, localizer.updateCount);
        assertZero(drivetrain.lastDrivePowers);

        adapter.update(clock.clock());
        assertEquals(0, localizer.updateCount);

        clock.nextCycle(0.02);
        adapter.update(clock.clock());
        assertEquals(1, localizer.updateCount);
        assertZero(drivetrain.lastDrivePowers);
    }

    @Test
    public void nextRouteReplacesStableStoppedManualMode() {
        Fixture fixture = new Fixture(START);

        enterNonZeroManualDrive(fixture);
        fixture.adapter.stop();
        fixture.clock.nextCycle(0.02);
        fixture.adapter.update(fixture.clock.clock());
        assertTrue(fixture.follower.isTeleopDrive());
        assertZero(fixture.drivetrain.lastDrivePowers);

        PathChain nextRoute = lineRoute();
        RouteExecution execution = fixture.adapter.follow(nextRoute, false);

        assertEquals(RouteStatus.ACTIVE, execution.status());
        assertFalse(fixture.follower.isTeleopDrive());
        assertTrue(fixture.follower.isBusy());
        assertSame(nextRoute, fixture.follower.getCurrentPathChain());

        int startTeleopCount = fixture.drivetrain.startTeleopDriveCount;
        int runDriveCount = fixture.drivetrain.runDriveCount;
        fixture.clock.nextCycle(0.02);
        fixture.adapter.update(fixture.clock.clock());

        assertEquals(startTeleopCount, fixture.drivetrain.startTeleopDriveCount);
        assertEquals(runDriveCount + 1, fixture.drivetrain.runDriveCount);
        assertTrue(fixture.follower.isBusy());
    }

    @Test
    public void partialRouteStartFailureFailsClosedAndKeepsLaterHeartbeatZero() {
        Fixture fixture = new Fixture(START);
        enterNonZeroManualDrive(fixture);
        RuntimeException startFailure = new IllegalStateException("test route-start break failure");
        fixture.drivetrain.throwOnNextBreak = startFailure;

        try {
            fixture.adapter.follow(lineRoute(), false);
            fail("Expected the partial Pedro route start to fail");
        } catch (RuntimeException actual) {
            assertSame(startFailure, actual);
        }

        assertFalse(fixture.follower.isBusy());
        assertFalse(fixture.follower.isTeleopDrive());
        assertEquals(RouteStatus.FAILED, fixture.adapter.getLatestRouteStatus());
        assertZero(fixture.drivetrain.lastDrivePowers);

        fixture.drivetrain.clearDriveObservation();
        fixture.clock.nextCycle(0.02);
        fixture.adapter.update(fixture.clock.clock());

        assertFalse(fixture.drivetrain.sawNonZeroDrive);
        assertZero(fixture.drivetrain.lastDrivePowers);
    }

    @Test
    public void invalidReplacementFailsClosedAndStopsActiveNonzeroRoute() {
        Fixture fixture = new Fixture(START);
        RouteExecution activeExecution = fixture.adapter.follow(lineRoute(), false);
        fixture.drivetrain.forceNextRunDrive = new double[]{0.70, 0.0, 0.0, 0.0};
        fixture.adapter.update(fixture.clock.clock());
        assertTrue(fixture.drivetrain.sawNonZeroDrive);

        try {
            fixture.adapter.follow(new PathChain(), false);
            fail("Expected an empty replacement route to fail");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("at least one Path"));
        }

        assertEquals(RouteStatus.FAILED, activeExecution.status());
        assertEquals(RouteStatus.FAILED, fixture.adapter.getLatestRouteStatus());
        assertFalse(fixture.follower.isBusy());
        assertZero(fixture.drivetrain.lastDrivePowers);

        fixture.drivetrain.clearDriveObservation();
        fixture.clock.nextCycle(0.02);
        fixture.adapter.update(fixture.clock.clock());
        assertFalse(fixture.drivetrain.sawNonZeroDrive);
        assertZero(fixture.drivetrain.lastDrivePowers);
    }

    @Test
    public void callbackInitializationFailureRetainsFailedStatusAndStopsPartialStart() {
        Fixture fixture = new Fixture(START);
        RuntimeException startFailure = new IllegalStateException(
                "test callback initialization failure"
        );
        PathChain route = lineRoute();
        route.setCallbacks(new InitializeCallback(() -> {
            throw startFailure;
        }));

        try {
            fixture.adapter.follow(route, false);
            fail("Expected callback initialization to fail the route start");
        } catch (RuntimeException actual) {
            assertSame(startFailure, actual);
        }

        assertEquals(RouteStatus.FAILED, fixture.adapter.getLatestRouteStatus());
        assertFalse(fixture.follower.isBusy());
        assertZero(fixture.drivetrain.lastDrivePowers);

        fixture.drivetrain.clearDriveObservation();
        fixture.adapter.update(fixture.clock.clock());
        assertFalse(fixture.drivetrain.sawNonZeroDrive);
        assertZero(fixture.drivetrain.lastDrivePowers);
    }

    @Test
    public void postStartValidationGetterFailureRetainsFailedStatusAndStopsRoute() {
        RecordingLocalizer localizer = new RecordingLocalizer(START);
        RecordingDrivetrain drivetrain = new RecordingDrivetrain();
        StartValidationThrowingFollower follower = new StartValidationThrowingFollower(
                localizer,
                drivetrain
        );
        PedroPathingDriveAdapter adapter = new PedroPathingDriveAdapter(follower);
        localizer.clearObservations();
        drivetrain.clearObservations();
        RuntimeException validationFailure = new IllegalStateException(
                "test post-start validation failure"
        );
        follower.validationFailure = validationFailure;

        try {
            adapter.follow(lineRoute(), false);
            fail("Expected post-start validation getter failure");
        } catch (RuntimeException actual) {
            assertSame(validationFailure, actual);
        }

        assertEquals(RouteStatus.FAILED, adapter.getLatestRouteStatus());
        assertFalse(follower.isBusy());
        assertZero(drivetrain.lastDrivePowers);
    }

    @Test
    public void callbackUpdateReentryDoesNotAdvanceFollowerTwice() {
        Fixture fixture = new Fixture(START);
        int[] callbackCount = {0};
        PathChain route = lineRoute();
        route.setCallbacks(new OneShotCallback(() -> {
            callbackCount[0]++;
            fixture.adapter.update(fixture.clock.clock());
        }));

        RouteExecution execution = fixture.adapter.follow(route, false);
        fixture.adapter.update(fixture.clock.clock());
        fixture.adapter.update(fixture.clock.clock());

        assertEquals(RouteStatus.ACTIVE, execution.status());
        assertEquals(1, callbackCount[0]);
        assertEquals(1, fixture.localizer.updateCount);
        assertEquals(1, fixture.drivetrain.updateConstantsCount);
    }

    @Test
    public void callbackInitializationUpdateReentryDefersWithoutConsumingCycle() {
        Fixture fixture = new Fixture(START);
        int[] callbackCount = {0};
        PathChain route = lineRoute();
        route.setCallbacks(new InitializeCallback(() -> {
            callbackCount[0]++;
            fixture.adapter.update(fixture.clock.clock());
        }));

        RouteExecution execution = fixture.adapter.follow(route, false);

        assertEquals(1, callbackCount[0]);
        assertEquals(0, fixture.localizer.updateCount);
        assertEquals(RouteStatus.ACTIVE, execution.status());

        fixture.adapter.update(fixture.clock.clock());
        fixture.adapter.update(fixture.clock.clock());

        assertEquals(1, fixture.localizer.updateCount);
        assertEquals(1, fixture.drivetrain.updateConstantsCount);
        assertEquals(RouteStatus.ACTIVE, execution.status());
    }

    @Test
    public void callbackInitializationStopOverridesOuterRouteStart() {
        Fixture fixture = new Fixture(END);
        PathChain route = lineRoute();
        route.setCallbacks(new InitializeCallback(fixture.adapter::stop));

        RouteExecution execution = fixture.adapter.follow(route, true);

        assertEquals(RouteStatus.INTERRUPTED, execution.status());
        assertFalse(fixture.follower.isBusy());
        assertZero(fixture.drivetrain.lastDrivePowers);

        fixture.drivetrain.clearDriveObservation();
        fixture.adapter.update(fixture.clock.clock());

        assertEquals(1, fixture.drivetrain.startTeleopDriveCount);
        assertTrue(fixture.follower.isTeleopDrive());
        assertFalse(fixture.drivetrain.sawNonZeroDrive);
        assertZero(fixture.drivetrain.lastDrivePowers);

        fixture.localizer.pose = new Pose(END.getX() - 1.0, END.getY(), END.getHeading());
        fixture.clock.nextCycle(0.02);
        fixture.adapter.update(fixture.clock.clock());

        assertFalse(fixture.drivetrain.sawNonZeroDrive);
        assertZero(fixture.drivetrain.lastDrivePowers);
    }

    @Test
    public void callbackInitializationRouteStartIsRejectedAndFailsOuterStartClosed() {
        Fixture fixture = new Fixture(START);
        PathChain route = lineRoute();
        route.setCallbacks(new InitializeCallback(
                () -> fixture.adapter.follow(lineRoute(), false)
        ));

        try {
            fixture.adapter.follow(route, false);
            fail("Expected callback-initializer route replacement to be rejected");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("enqueue a fresh RouteTask"));
        }

        assertEquals(RouteStatus.FAILED, fixture.adapter.getLatestRouteStatus());
        assertFalse(fixture.follower.isBusy());
        assertZero(fixture.drivetrain.lastDrivePowers);
    }

    @Test
    public void callbackRouteStartIsRejectedAndFailsClosed() {
        Fixture fixture = new Fixture(START);
        PathChain route = lineRoute();
        route.setCallbacks(new OneShotCallback(
                () -> fixture.adapter.follow(lineRoute(), false)
        ));
        RouteExecution execution = fixture.adapter.follow(route, false);

        try {
            fixture.adapter.update(fixture.clock.clock());
            fail("Expected callback-time route replacement to be rejected");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("enqueue a fresh RouteTask"));
        }

        assertEquals(1, fixture.localizer.updateCount);
        assertEquals(RouteStatus.FAILED, execution.status());
        assertFalse(fixture.follower.isBusy());
        assertZero(fixture.drivetrain.lastDrivePowers);

        fixture.adapter.update(fixture.clock.clock());
        assertEquals(1, fixture.localizer.updateCount);

        fixture.clock.nextCycle(0.02);
        fixture.adapter.update(fixture.clock.clock());
        assertZero(fixture.drivetrain.lastDrivePowers);
    }

    @Test
    public void reentrantStopWinsAfterEnclosingPedroUpdateWritesAgain() {
        Fixture fixture = new Fixture(START);
        PathChain route = lineRoute();
        route.setCallbacks(new OneShotCallback(fixture.adapter::stop));

        RouteExecution execution = fixture.adapter.follow(route, false);
        int breakCountBeforeHeartbeat = fixture.drivetrain.breakFollowingCount;
        fixture.drivetrain.forceNextRunDrive = new double[]{0.70, 0.0, 0.0, 0.0};
        fixture.drivetrain.clearDriveObservation();

        fixture.adapter.update(fixture.clock.clock());

        assertTrue(fixture.drivetrain.sawNonZeroDrive);
        assertZero(fixture.drivetrain.lastDrivePowers);
        assertEquals(RouteStatus.INTERRUPTED, execution.status());
        assertEquals(
                breakCountBeforeHeartbeat + 2,
                fixture.drivetrain.breakFollowingCount
        );
        assertFalse(fixture.follower.isBusy());

        fixture.drivetrain.clearDriveObservation();
        fixture.clock.nextCycle(0.02);
        fixture.adapter.update(fixture.clock.clock());
        assertFalse(fixture.drivetrain.sawNonZeroDrive);
        assertZero(fixture.drivetrain.lastDrivePowers);
    }

    @Test
    public void thrownVendorUpdateFailsClosedAndConsumesTheCurrentCycle() {
        Fixture fixture = new Fixture(START);
        enterNonZeroManualDrive(fixture);
        assertTrue(fixture.drivetrain.sawNonZeroDrive);

        RuntimeException expected = new IllegalStateException("test Pedro update failure");
        fixture.localizer.throwOnNextUpdate = expected;
        fixture.clock.nextCycle(0.02);

        try {
            fixture.adapter.update(fixture.clock.clock());
            fail("Expected the real Pedro heartbeat to propagate the localizer failure");
        } catch (RuntimeException actual) {
            assertSame(expected, actual);
        }

        assertZero(fixture.drivetrain.lastDrivePowers);
        assertFalse(fixture.follower.isBusy());
        assertFalse(fixture.follower.isTeleopDrive());
        int updateCountAfterFailure = fixture.localizer.updateCount;

        fixture.adapter.update(fixture.clock.clock());
        assertEquals(updateCountAfterFailure, fixture.localizer.updateCount);

        fixture.drivetrain.clearDriveObservation();
        fixture.clock.nextCycle(0.02);
        fixture.adapter.update(fixture.clock.clock());

        assertEquals(updateCountAfterFailure + 1, fixture.localizer.updateCount);
        assertFalse(fixture.drivetrain.sawNonZeroDrive);
        assertZero(fixture.drivetrain.lastDrivePowers);
    }

    @Test
    public void updatePreservesStopFailureAndLaterOwnerStopRetriesPhysicalZero() {
        Fixture fixture = new Fixture(START);
        enterNonZeroManualDrive(fixture);

        RuntimeException updateFailure = new IllegalStateException("test update failure");
        RuntimeException firstStopFailure = new IllegalArgumentException("test stop failure");
        fixture.localizer.throwOnNextUpdate = updateFailure;
        fixture.drivetrain.throwOnNextBreak = firstStopFailure;
        fixture.clock.nextCycle(0.02);

        try {
            fixture.adapter.update(fixture.clock.clock());
            fail("Expected the Pedro update failure");
        } catch (RuntimeException actual) {
            assertSame(updateFailure, actual);
            assertEquals(1, actual.getSuppressed().length);
            assertSame(firstStopFailure, actual.getSuppressed()[0]);
        }

        assertTrue(containsNonZero(fixture.drivetrain.lastDrivePowers));

        // Phoenix's later owner-level stop must retry even though failure cleanup already staged stop.
        fixture.adapter.stop();
        assertZero(fixture.drivetrain.lastDrivePowers);
        assertFalse(fixture.follower.isBusy());
    }

    private static void enterNonZeroManualDrive(Fixture fixture) {
        fixture.adapter.drive(new DriveSignal(0.60, -0.20, 0.15));
        fixture.adapter.update(fixture.clock.clock());
        assertZero(fixture.drivetrain.lastDrivePowers);

        fixture.drivetrain.clearDriveObservation();
        fixture.clock.nextCycle(0.02);
        fixture.adapter.update(fixture.clock.clock());
        assertTrue(fixture.drivetrain.sawNonZeroDrive);
    }

    private static PathChain lineRoute() {
        return lineRoute(testConstraints());
    }

    private static PathChain lineRoute(PathConstraints constraints) {
        Path path = new Path(new BezierLine(START, END), constraints);
        path.setConstantHeadingInterpolation(0.0);
        return new PathChain(constraints, path);
    }

    private static PathChain twoSegmentRoute() {
        PathConstraints constraints = testConstraints();
        Path first = new Path(new BezierLine(START, MID), constraints);
        Path second = new Path(new BezierLine(MID, END), constraints);
        first.setConstantHeadingInterpolation(0.0);
        second.setConstantHeadingInterpolation(0.0);
        return new PathChain(constraints, first, second);
    }

    private static PathChain endpointTimeoutRoute() {
        PathConstraints impossibleEndpointConstraints = new PathConstraints(
                0.99,
                -1.0,
                -1.0,
                -1.0,
                -1.0,
                1.0,
                10,
                1.0
        );
        return lineRoute(impossibleEndpointConstraints);
    }

    private static PathChain parametricThresholdRoute() {
        PathConstraints thresholdConstraints = new PathConstraints(
                0.99,
                0.10,
                0.20,
                0.01,
                10_000.0,
                1.0,
                10,
                1.0
        );
        return lineRoute(thresholdConstraints);
    }

    private static PathConstraints testConstraints() {
        return new PathConstraints(
                0.99,
                0.10,
                0.10,
                0.01,
                0.0,
                1.0,
                10,
                1.0
        );
    }

    private static FollowerConstants testFollowerConstants() {
        return new FollowerConstants().automaticHoldEnd(true);
    }

    private static FollowerConstants immediateStallConstants() {
        FollowerConstants constants = testFollowerConstants();
        constants.stuckVelocity = 1.0;
        constants.stuckTValueLow = -1.0;
        constants.stuckTValueHigh = 2.0;
        constants.stuckTimeout = -1.0;
        return constants;
    }

    private static void assertZero(double[] powers) {
        assertArrayEquals(new double[]{0.0, 0.0, 0.0, 0.0}, powers, EPSILON);
    }

    private static boolean containsNonZero(double[] values) {
        for (double value : values) {
            if (Math.abs(value) > EPSILON) {
                return true;
            }
        }
        return false;
    }

    /** Real Pedro follower with one injectable read failure at the adapter snapshot boundary. */
    private static final class SnapshotThrowingFollower extends Follower {
        RuntimeException throwOnNextCurrentPath;

        SnapshotThrowingFollower(RecordingLocalizer localizer,
                                 RecordingDrivetrain drivetrain) {
            super(testFollowerConstants(), localizer, drivetrain, testConstraints());
        }

        @Override
        public Path getCurrentPath() {
            RuntimeException failure = throwOnNextCurrentPath;
            throwOnNextCurrentPath = null;
            if (failure != null) {
                throw failure;
            }
            return super.getCurrentPath();
        }
    }

    /** Real Pedro follower that fails only in the adapter's post-start validation read. */
    private static final class StartValidationThrowingFollower extends Follower {
        RuntimeException validationFailure;
        private RuntimeException throwOnNextCurrentPathChain;

        StartValidationThrowingFollower(RecordingLocalizer localizer,
                                        RecordingDrivetrain drivetrain) {
            super(testFollowerConstants(), localizer, drivetrain, testConstraints());
        }

        @Override
        public void followPath(PathChain route, boolean holdEnd) {
            super.followPath(route, holdEnd);
            throwOnNextCurrentPathChain = validationFailure;
        }

        @Override
        public PathChain getCurrentPathChain() {
            RuntimeException failure = throwOnNextCurrentPathChain;
            throwOnNextCurrentPathChain = null;
            if (failure != null) {
                throw failure;
            }
            return super.getCurrentPathChain();
        }
    }

    private static final class Fixture {
        final ManualLoopClock clock = new ManualLoopClock();
        final RecordingLocalizer localizer;
        final RecordingDrivetrain drivetrain = new RecordingDrivetrain();
        final Follower follower;
        final PedroPathingDriveAdapter adapter;

        Fixture(Pose initialPose) {
            this(initialPose, testFollowerConstants());
        }

        Fixture(Pose initialPose, FollowerConstants constants) {
            localizer = new RecordingLocalizer(initialPose);
            follower = new Follower(
                    constants,
                    localizer,
                    drivetrain,
                    testConstraints()
            );
            adapter = new PedroPathingDriveAdapter(follower);

            // Follower construction deliberately resets its drivetrain once.
            localizer.clearObservations();
            drivetrain.clearObservations();
        }
    }

    private static final class RecordingLocalizer implements Localizer {
        Pose pose;
        Pose velocity = new Pose();
        int updateCount;
        RuntimeException throwOnNextUpdate;

        RecordingLocalizer(Pose initialPose) {
            pose = initialPose;
        }

        void clearObservations() {
            updateCount = 0;
            throwOnNextUpdate = null;
        }

        @Override
        public Pose getPose() {
            return pose;
        }

        @Override
        public Pose getVelocity() {
            return velocity;
        }

        @Override
        public Vector getVelocityVector() {
            return velocity.getAsVector();
        }

        @Override
        public void setStartPose(Pose setStart) {
            pose = setStart;
        }

        @Override
        public void setPose(Pose setPose) {
            pose = setPose;
        }

        @Override
        public void update() {
            updateCount++;
            RuntimeException failure = throwOnNextUpdate;
            throwOnNextUpdate = null;
            if (failure != null) {
                throw failure;
            }
        }

        @Override
        public double getTotalHeading() {
            return pose.getHeading();
        }

        @Override
        public double getForwardMultiplier() {
            return 1.0;
        }

        @Override
        public double getLateralMultiplier() {
            return 1.0;
        }

        @Override
        public double getTurningMultiplier() {
            return 1.0;
        }

        @Override
        public void resetIMU() {
            // No IMU in the pure-JVM fixture.
        }

        @Override
        public double getIMUHeading() {
            return pose.getHeading();
        }

        @Override
        public boolean isNAN() {
            return Double.isNaN(pose.getX())
                    || Double.isNaN(pose.getY())
                    || Double.isNaN(pose.getHeading());
        }
    }

    private static final class MutablePhoenixPoseEstimator implements AbsolutePoseEstimator {
        private PoseEstimate estimate = PoseEstimate.noPose(0.0);

        void setPose(double fieldXInches, double nowSec) {
            estimate = new PoseEstimate(
                    new Pose3d(fieldXInches, 0.0, 0.0, 0.0, 0.0, 0.0),
                    true,
                    1.0,
                    0.0,
                    nowSec
            );
        }

        @Override
        public void update(LoopClock clock) {
            // The test controls snapshots directly.
        }

        @Override
        public PoseEstimate getEstimate() {
            return estimate;
        }
    }

    private static final class RecordingDrivetrain extends Drivetrain {
        int updateConstantsCount;
        int breakFollowingCount;
        int runDriveCount;
        int startTeleopDriveCount;
        double xVelocity;
        double yVelocity;
        double[] lastDrivePowers = new double[4];
        double[] forceNextRunDrive;
        RuntimeException throwOnNextBreak;
        boolean sawNonZeroDrive;

        RecordingDrivetrain() {
            setMaxPowerScaling(1.0);
            setNominalVoltage(12.0);
        }

        void clearObservations() {
            updateConstantsCount = 0;
            breakFollowingCount = 0;
            runDriveCount = 0;
            startTeleopDriveCount = 0;
            xVelocity = 0.0;
            yVelocity = 0.0;
            forceNextRunDrive = null;
            throwOnNextBreak = null;
            clearDriveObservation();
        }

        void clearDriveObservation() {
            Arrays.fill(lastDrivePowers, 0.0);
            sawNonZeroDrive = false;
        }

        @Override
        public double[] calculateDrive(Vector correctivePower,
                                       Vector headingPower,
                                       Vector pathingPower,
                                       double robotHeading) {
            return new double[]{
                    correctivePower.getMagnitude(),
                    headingPower.getMagnitude(),
                    pathingPower.getMagnitude(),
                    0.0
            };
        }

        @Override
        public void updateConstants() {
            updateConstantsCount++;
        }

        @Override
        public void breakFollowing() {
            breakFollowingCount++;
            RuntimeException failure = throwOnNextBreak;
            throwOnNextBreak = null;
            if (failure != null) {
                throw failure;
            }
            Arrays.fill(lastDrivePowers, 0.0);
        }

        @Override
        public void runDrive(double[] drivePowers) {
            runDriveCount++;
            if (forceNextRunDrive != null) {
                lastDrivePowers = forceNextRunDrive.clone();
                forceNextRunDrive = null;
            } else {
                lastDrivePowers = drivePowers.clone();
            }
            sawNonZeroDrive |= containsNonZero(lastDrivePowers);
        }

        @Override
        public void startTeleopDrive() {
            startTeleopDriveCount++;
        }

        @Override
        public void startTeleopDrive(boolean brakeMode) {
            startTeleopDriveCount++;
        }

        @Override
        public double xVelocity() {
            return xVelocity;
        }

        @Override
        public double yVelocity() {
            return yVelocity;
        }

        @Override
        public void setXVelocity(double xMovement) {
            xVelocity = xMovement;
        }

        @Override
        public void setYVelocity(double yMovement) {
            yVelocity = yMovement;
        }

        @Override
        public double getVoltage() {
            return 12.0;
        }

        @Override
        public String debugString() {
            return "RecordingDrivetrain";
        }
    }

    private static final class OneShotCallback implements PathCallback {
        private final Runnable action;
        private boolean fired;

        OneShotCallback(Runnable action) {
            this.action = action;
        }

        @Override
        public boolean run() {
            fired = true;
            action.run();
            return true;
        }

        @Override
        public boolean isReady() {
            return !fired;
        }

        @Override
        public int getPathIndex() {
            return 0;
        }

        @Override
        public void reset() {
            fired = false;
        }
    }

    private static final class InitializeCallback implements PathCallback {
        private final Runnable action;

        InitializeCallback(Runnable action) {
            this.action = action;
        }

        @Override
        public void initialize() {
            action.run();
        }

        @Override
        public boolean run() {
            return true;
        }

        @Override
        public boolean isReady() {
            return false;
        }

        @Override
        public int getPathIndex() {
            return 0;
        }
    }
}
