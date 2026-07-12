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
import edu.ftcphoenix.fw.drive.route.RouteTask;
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
    private static final Pose END = new Pose(12.0, 0.0, 0.0);

    @Test
    public void rootAndRouteTaskUpdatesAdvanceFollowerOncePerCycle() {
        Fixture fixture = new Fixture(START);
        RouteTask.Config config = new RouteTask.Config();
        config.timeoutSec = 0.0;
        RouteTask<PathChain> task = new RouteTask<>(
                "deduplicated route",
                fixture.adapter,
                lineRoute(),
                config
        );

        task.start(fixture.clock.clock());
        fixture.adapter.update(fixture.clock.clock());
        task.update(fixture.clock.clock());

        assertEquals(1, fixture.localizer.updateCount);
        assertEquals(1, fixture.drivetrain.updateConstantsCount);
        assertFalse(task.isComplete());

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
        RouteTask.Config config = new RouteTask.Config();
        config.timeoutSec = 1.0;
        RouteTask<PathChain> task = new RouteTask<>(
                "hold-end route",
                fixture.adapter,
                lineRoute(),
                config
        );

        task.start(fixture.clock.clock());
        fixture.adapter.update(fixture.clock.clock());
        task.update(fixture.clock.clock());

        assertTrue(task.isComplete());
        assertEquals(TaskOutcome.SUCCESS, task.getOutcome());
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

        fixture.adapter.follow(lineRoute(), false);
        fixture.adapter.update(fixture.clock.clock());

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
    public void routeToManualTransitionCountsPedrosHiddenUpdateAsTheHeartbeat() {
        Fixture fixture = new Fixture(END);
        DriveSignal requested = new DriveSignal(0.50, -0.25, 0.20);

        fixture.adapter.follow(lineRoute(), true);
        fixture.adapter.update(fixture.clock.clock());
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
        fixture.adapter.cancel();
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

        fixture.adapter.follow(lineRoute(), true);
        fixture.adapter.cancel();

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
        RouteTask.Config config = new RouteTask.Config();
        config.timeoutSec = 0.10;
        RouteTask<PathChain> task = new RouteTask<>(
                "timed route",
                fixture.adapter,
                lineRoute(),
                config
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
    public void nextRouteReplacesStableStoppedManualMode() {
        Fixture fixture = new Fixture(START);

        enterNonZeroManualDrive(fixture);
        fixture.adapter.stop();
        fixture.clock.nextCycle(0.02);
        fixture.adapter.update(fixture.clock.clock());
        assertTrue(fixture.follower.isTeleopDrive());
        assertZero(fixture.drivetrain.lastDrivePowers);

        PathChain nextRoute = lineRoute();
        fixture.adapter.follow(nextRoute, false);

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
        assertZero(fixture.drivetrain.lastDrivePowers);

        fixture.drivetrain.clearDriveObservation();
        fixture.clock.nextCycle(0.02);
        fixture.adapter.update(fixture.clock.clock());

        assertFalse(fixture.drivetrain.sawNonZeroDrive);
        assertZero(fixture.drivetrain.lastDrivePowers);
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

        fixture.adapter.follow(route, false);
        fixture.adapter.update(fixture.clock.clock());
        fixture.adapter.update(fixture.clock.clock());

        assertEquals(1, callbackCount[0]);
        assertEquals(1, fixture.localizer.updateCount);
        assertEquals(1, fixture.drivetrain.updateConstantsCount);
    }

    @Test
    public void callbackInitializationStopOverridesOuterRouteStart() {
        Fixture fixture = new Fixture(END);
        PathChain route = lineRoute();
        route.setCallbacks(new InitializeCallback(fixture.adapter::stop));

        fixture.adapter.follow(route, true);

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
    public void callbackRouteStartIsRejectedAndFailsClosed() {
        Fixture fixture = new Fixture(START);
        PathChain route = lineRoute();
        route.setCallbacks(new OneShotCallback(
                () -> fixture.adapter.follow(lineRoute(), false)
        ));
        fixture.adapter.follow(route, false);

        try {
            fixture.adapter.update(fixture.clock.clock());
            fail("Expected callback-time route replacement to be rejected");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("enqueue a fresh RouteTask"));
        }

        assertEquals(1, fixture.localizer.updateCount);
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

        fixture.adapter.follow(route, false);
        int breakCountBeforeHeartbeat = fixture.drivetrain.breakFollowingCount;
        fixture.drivetrain.forceNextRunDrive = new double[]{0.70, 0.0, 0.0, 0.0};
        fixture.drivetrain.clearDriveObservation();

        fixture.adapter.update(fixture.clock.clock());

        assertTrue(fixture.drivetrain.sawNonZeroDrive);
        assertZero(fixture.drivetrain.lastDrivePowers);
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
        Path path = new Path(new BezierLine(START, END), testConstraints());
        path.setConstantHeadingInterpolation(0.0);
        return new PathChain(testConstraints(), path);
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

    private static final class Fixture {
        final ManualLoopClock clock = new ManualLoopClock();
        final RecordingLocalizer localizer;
        final RecordingDrivetrain drivetrain = new RecordingDrivetrain();
        final Follower follower;
        final PedroPathingDriveAdapter adapter;

        Fixture(Pose initialPose) {
            localizer = new RecordingLocalizer(initialPose);
            follower = new Follower(
                    new FollowerConstants().automaticHoldEnd(true),
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
