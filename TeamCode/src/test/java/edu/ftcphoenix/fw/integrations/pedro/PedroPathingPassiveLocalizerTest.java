package edu.ftcphoenix.fw.integrations.pedro;

import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;

import org.junit.Test;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

public final class PedroPathingPassiveLocalizerTest {

    private static final double EPSILON = 1e-9;

    @Test
    public void pinnedFollowerConsumesExactlyOneExpectedConstructorReset() {
        FakePredictorAccess access = new FakePredictorAccess();
        PedroPathingPassiveLocalizer localizer = passiveLocalizer(access);

        new Follower(
                new FollowerConstants(),
                localizer,
                new FakeDrivetrain(),
                PathConstraints.defaultConstraints.copy()
        );
        localizer.completeFollowerConstruction();

        assertFailsContaining(localizer::resetIMU, "Raw Pedro resetIMU");

        PedroPathingPassiveLocalizer missingReset = passiveLocalizer(
                new FakePredictorAccess()
        );
        assertFailsContaining(
                missingReset::completeFollowerConstruction,
                "exactly once"
        );
    }

    @Test
    public void realFollowerHeartbeatConsumesOneExactCycleSnapshot() {
        Fixture fixture = new Fixture();
        fixture.access.sample = sampled(
                fixture.clock.clock().cycle(),
                fixture.clock.clock().nowSec(),
                new Pose2d(10.0, -5.0, 0.25),
                3.0,
                4.0,
                0.5,
                7.25
        );
        int readsBeforeHeartbeat = fixture.access.readCount;

        fixture.adapter.update(fixture.clock.clock());
        fixture.adapter.update(fixture.clock.clock());

        assertEquals(readsBeforeHeartbeat + 1, fixture.access.readCount);
        Pose pedroPose = fixture.follower.getPose();
        assertEquals(77.0, pedroPose.getX(), EPSILON);
        assertEquals(82.0, pedroPose.getY(), EPSILON);
        assertEquals(0.25 + Math.PI / 2.0, pedroPose.getHeading(), EPSILON);

        Pose pedroVelocity = fixture.localizer.getVelocity();
        assertEquals(-4.0, pedroVelocity.getX(), EPSILON);
        assertEquals(3.0, pedroVelocity.getY(), EPSILON);
        assertEquals(0.5, pedroVelocity.getHeading(), EPSILON);
        assertEquals(7.25, fixture.localizer.getTotalHeading(), EPSILON);
        assertFalse(fixture.localizer.isNAN());
    }

    @Test
    public void staleAndUnavailableSamplesFailClosedWithActionableErrors() {
        Fixture stale = new Fixture();
        stale.access.sample = sampled(
                stale.clock.clock().cycle() + 1,
                stale.clock.clock().nowSec(),
                Pose2d.zero(),
                0.0,
                0.0,
                0.0,
                0.0
        );
        int breaksBeforeStaleUpdate = stale.drivetrain.breakCount;
        assertFailsContaining(
                () -> stale.adapter.update(stale.clock.clock()),
                "Update Phoenix localization"
        );
        assertTrue(stale.drivetrain.breakCount > breaksBeforeStaleUpdate);

        Fixture noPose = new Fixture();
        noPose.access.sample = new PedroPathingPassiveLocalizer.Sample(
                Pose2d.zero(),
                false,
                false,
                noPose.clock.clock().cycle(),
                noPose.clock.clock().nowSec(),
                0.0,
                0.0,
                0.0,
                0.0
        );
        assertFailsContaining(
                () -> noPose.adapter.update(noPose.clock.clock()),
                "pose is unavailable"
        );

        Fixture noVelocity = new Fixture();
        noVelocity.access.sample = new PedroPathingPassiveLocalizer.Sample(
                Pose2d.zero(),
                true,
                false,
                noVelocity.clock.clock().cycle(),
                noVelocity.clock.clock().nowSec(),
                0.0,
                0.0,
                0.0,
                0.0
        );
        assertFailsContaining(
                () -> noVelocity.adapter.update(noVelocity.clock.clock()),
                "physical velocity is unavailable"
        );
    }

    @Test
    public void startAndCorrectionRebasesPreservePhysicalVelocityAndHeading() {
        FakePredictorAccess access = new FakePredictorAccess();
        access.sample = sampled(
                -1L,
                2.0,
                new Pose2d(4.0, 5.0, 0.2),
                6.0,
                -2.0,
                0.75,
                9.0
        );
        PedroPathingPassiveLocalizer localizer = passiveLocalizer(access);
        FakeDrivetrain drivetrain = new FakeDrivetrain();
        Follower follower = new Follower(
                new FollowerConstants(),
                localizer,
                drivetrain,
                PathConstraints.defaultConstraints.copy()
        );
        localizer.completeFollowerConstruction();

        Pose pedroStart = new Pose(80.0, 90.0, Math.PI);
        follower.setStartingPose(pedroStart);

        assertEquals(1, access.setPoseCount);
        assertEquals(6.0, access.sample.phoenixFieldVelocityXInchesPerSec, EPSILON);
        assertEquals(-2.0, access.sample.phoenixFieldVelocityYInchesPerSec, EPSILON);
        assertEquals(0.75, access.sample.angularVelocityRadPerSec, EPSILON);
        assertEquals(9.0, access.sample.totalHeadingRad, EPSILON);
        assertEquals(2.0, localizer.getVelocity().getX(), EPSILON);
        assertEquals(6.0, localizer.getVelocity().getY(), EPSILON);
        assertEquals(9.0, localizer.getTotalHeading(), EPSILON);

        ManualLoopClock clock = new ManualLoopClock(2.0);
        access.sample = sampled(
                clock.clock().cycle(),
                clock.clock().nowSec(),
                access.sample.phoenixFieldToRobotPose,
                6.0,
                -2.0,
                0.75,
                9.0
        );
        access.rebase(new Pose2d(-12.0, 18.0, -0.4));

        PedroPathingDriveAdapter adapter = new PedroPathingDriveAdapter(
                follower,
                localizer::prepareForHeartbeat
        );
        adapter.update(clock.clock());

        Pose correctedPedroPose = follower.getPose();
        assertEquals(54.0, correctedPedroPose.getX(), EPSILON);
        assertEquals(60.0, correctedPedroPose.getY(), EPSILON);
        assertEquals(-0.4 + Math.PI / 2.0, correctedPedroPose.getHeading(), EPSILON);
        assertEquals(2.0, localizer.getVelocity().getX(), EPSILON);
        assertEquals(6.0, localizer.getVelocity().getY(), EPSILON);
        assertEquals(0.75, localizer.getVelocity().getHeading(), EPSILON);
        assertEquals(9.0, localizer.getTotalHeading(), EPSILON);

        assertFailsContaining(
                () -> localizer.requireStartingPoseAllowed(pedroStart),
                "before the first heartbeat"
        );
    }

    @Test
    public void rawPedroPoseMutationAndUnownedUpdateAreRejected() {
        Fixture fixture = new Fixture();

        assertFailsContaining(
                () -> fixture.localizer.setPose(new Pose()),
                "Phoenix localization owner"
        );
        assertFailsContaining(fixture.localizer::update, "owned PedroPathingDriveAdapter");
        assertFailsContaining(fixture.localizer::resetIMU, "coordinated reset");
    }

    @Test
    public void runtimePathBuilderUsesCopiedConfiguredConstraintsInsteadOfPedroGlobalDefault() {
        Fixture fixture = new Fixture();
        PathConstraints configured = new PathConstraints(0.91, 42.0, 1.3, 0.8);

        PathChain path = PedroPathingRuntime.newPathBuilder(fixture.follower, configured)
                .addPath(new BezierLine(new Pose(), new Pose(12.0, 0.0, 0.0)))
                .setBrakingStart(0.35)
                .build();

        assertEquals(0.91, path.getPath(0).getPathEndTValueConstraint(), EPSILON);
        assertEquals(42.0, path.getPath(0).getPathEndTimeoutConstraint(), EPSILON);
        assertEquals(1.3, path.getPath(0).getBrakingStrength(), EPSILON);
        assertEquals(0.35, path.getPath(0).getBrakingStartMultiplier(), EPSILON);
        assertEquals(0.8, configured.getBrakingStart(), EPSILON);
    }

    @Test
    public void runtimePathBuilderPreservesMultiplePathsAndPerPathOverrides() {
        Fixture fixture = new Fixture();
        PathConstraints firstDefaults = new PathConstraints(0.91, 42.0, 1.3, 0.8);
        PathConstraints secondDefaults = new PathConstraints(0.87, 24.0, 1.5, 0.6);

        PathChain path = PedroPathingRuntime.newPathBuilder(fixture.follower, firstDefaults)
                .addPath(new BezierLine(new Pose(), new Pose(12.0, 0.0, 0.0)))
                .setTValueConstraint(0.83)
                .setConstraints(secondDefaults)
                .addPath(new BezierLine(
                        new Pose(12.0, 0.0, 0.0),
                        new Pose(24.0, 0.0, 0.0)
                ))
                .setTimeoutConstraint(17.0)
                .setBrakingStart(0.35)
                .build();

        assertEquals(2, path.size());
        assertEquals(0.83, path.getPath(0).getPathEndTValueConstraint(), EPSILON);
        assertEquals(42.0, path.getPath(0).getPathEndTimeoutConstraint(), EPSILON);
        assertEquals(1.3, path.getPath(0).getBrakingStrength(), EPSILON);
        assertEquals(0.87, path.getPath(1).getPathEndTValueConstraint(), EPSILON);
        assertEquals(17.0, path.getPath(1).getPathEndTimeoutConstraint(), EPSILON);
        assertEquals(1.5, path.getPath(1).getBrakingStrength(), EPSILON);
        assertEquals(0.35, path.getPath(0).getBrakingStartMultiplier(), EPSILON);
        assertEquals(0.35, path.getPath(1).getBrakingStartMultiplier(), EPSILON);
        assertEquals(0.8, firstDefaults.getBrakingStart(), EPSILON);
        assertEquals(0.6, secondDefaults.getBrakingStart(), EPSILON);
    }

    private static PedroPathingPassiveLocalizer passiveLocalizer(
            FakePredictorAccess access) {
        return new PedroPathingPassiveLocalizer(
                access,
                PedroFieldTransform.decodeInvertedFtc()
        );
    }

    private static PedroPathingPassiveLocalizer.Sample sampled(long cycle,
                                                               double timestampSec,
                                                               Pose2d pose,
                                                               double velocityX,
                                                               double velocityY,
                                                               double angularVelocity,
                                                               double totalHeading) {
        return new PedroPathingPassiveLocalizer.Sample(
                pose,
                true,
                true,
                cycle,
                timestampSec,
                velocityX,
                velocityY,
                angularVelocity,
                totalHeading
        );
    }

    private static void assertFailsContaining(Runnable action, String expectedMessage) {
        try {
            action.run();
            fail("Expected failure containing: " + expectedMessage);
        } catch (IllegalStateException expected) {
            assertTrue(
                    "Expected message containing '" + expectedMessage + "' but got: "
                            + expected.getMessage(),
                    expected.getMessage().contains(expectedMessage)
            );
        }
    }

    private static final class Fixture {
        final FakePredictorAccess access = new FakePredictorAccess();
        final PedroPathingPassiveLocalizer localizer = passiveLocalizer(access);
        final FakeDrivetrain drivetrain = new FakeDrivetrain();
        final Follower follower = new Follower(
                new FollowerConstants(),
                localizer,
                drivetrain,
                PathConstraints.defaultConstraints.copy()
        );
        final PedroPathingDriveAdapter adapter;
        final ManualLoopClock clock = new ManualLoopClock();

        Fixture() {
            localizer.completeFollowerConstruction();
            follower.setStartingPose(new Pose(72.0, 72.0, Math.PI / 2.0));
            adapter = new PedroPathingDriveAdapter(
                    follower,
                    localizer::prepareForHeartbeat
            );
        }
    }

    private static final class FakePredictorAccess
            implements PedroPathingPassiveLocalizer.PredictorAccess {
        PedroPathingPassiveLocalizer.Sample sample =
                PedroPathingPassiveLocalizer.Sample.unavailable();
        int readCount;
        int setPoseCount;

        @Override
        public PedroPathingPassiveLocalizer.Sample currentSnapshot() {
            readCount++;
            return sample;
        }

        @Override
        public void setPose(Pose2d phoenixFieldToRobotPose) {
            setPoseCount++;
            rebase(phoenixFieldToRobotPose);
        }

        void rebase(Pose2d phoenixFieldToRobotPose) {
            sample = new PedroPathingPassiveLocalizer.Sample(
                    phoenixFieldToRobotPose,
                    true,
                    sample.hasVelocity,
                    sample.cycle,
                    sample.timestampSec,
                    sample.phoenixFieldVelocityXInchesPerSec,
                    sample.phoenixFieldVelocityYInchesPerSec,
                    sample.angularVelocityRadPerSec,
                    sample.totalHeadingRad
            );
        }
    }

    private static final class FakeDrivetrain extends Drivetrain {
        int breakCount;

        @Override
        public double[] calculateDrive(Vector correctivePower,
                                       Vector headingPower,
                                       Vector pathingPower,
                                       double robotHeading) {
            return new double[] {0.0, 0.0, 0.0, 0.0};
        }

        @Override
        public void updateConstants() {
            // Nothing to update in the fake boundary.
        }

        @Override
        public void breakFollowing() {
            breakCount++;
        }

        @Override
        public void runDrive(double[] drivePowers) {
            // No hardware in the JVM test.
        }

        @Override
        public void startTeleopDrive() {
            // No hardware in the JVM test.
        }

        @Override
        public void startTeleopDrive(boolean brakeMode) {
            // No hardware in the JVM test.
        }

        @Override
        public double xVelocity() {
            return 80.0;
        }

        @Override
        public double yVelocity() {
            return 65.0;
        }

        @Override
        public void setXVelocity(double xMovement) {
            // No hardware in the JVM test.
        }

        @Override
        public void setYVelocity(double yMovement) {
            // No hardware in the JVM test.
        }

        @Override
        public double getVoltage() {
            return 12.0;
        }

        @Override
        public String debugString() {
            return "FakeDrivetrain";
        }
    }
}
