package edu.ftcphoenix.fw.integrations.pedro;

import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;

import org.junit.Test;

import java.lang.reflect.Constructor;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.ftc.localization.PinpointOdometryPredictor;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;
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
                fixture.clock.clock().nowTimestamp(),
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
                stale.clock.clock().nowTimestamp(),
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
        noPose.access.statusSummary = "CALIBRATING";
        noPose.access.sample = new PedroPathingPassiveLocalizer.Sample(
                Pose2d.zero(),
                false,
                false,
                noPose.clock.clock().cycle(),
                noPose.clock.clock().nowTimestamp(),
                0.0,
                0.0,
                0.0,
                0.0
        );
        assertFailsContaining(
                () -> noPose.adapter.update(noPose.clock.clock()),
                "pose is unavailable",
                "Pinpoint lastDeviceStatus=CALIBRATING"
        );

        Fixture noVelocity = new Fixture();
        noVelocity.access.sample = new PedroPathingPassiveLocalizer.Sample(
                Pose2d.zero(),
                true,
                false,
                noVelocity.clock.clock().cycle(),
                noVelocity.clock.clock().nowTimestamp(),
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
    public void sameCycleSampleWithPriorEpochTimestampFailsClosedAfterReset() {
        Fixture fixture = new Fixture();
        LoopTimestamp priorEpochTimestamp = fixture.clock.clock().nowTimestamp();

        // Keep the exact same numeric clock value, and make cycle identity look current. The
        // retained timestamp's reset epoch must still prevent Pedro from consuming the sample.
        fixture.clock.clock().reset(fixture.clock.clock().nowSec());
        fixture.access.sample = sampled(
                fixture.clock.clock().cycle(),
                priorEpochTimestamp,
                Pose2d.zero(),
                0.0,
                0.0,
                0.0,
                0.0
        );
        int breaksBeforeUpdate = fixture.drivetrain.breakCount;

        assertFailsContaining(
                () -> fixture.adapter.update(fixture.clock.clock()),
                "current LoopClock reset epoch"
        );
        assertTrue(fixture.drivetrain.breakCount > breaksBeforeUpdate);
    }

    @Test
    public void repeatedPreHeartbeatStartAndCorrectionRebasesPreservePhysicalVelocityAndHeading() {
        FakePredictorAccess access = new FakePredictorAccess();
        ManualLoopClock initialClock = new ManualLoopClock(2.0);
        access.sample = sampled(
                -1L,
                initialClock.clock().nowTimestamp(),
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
        follower.setStartingPose(pedroStart);

        assertEquals(2, access.setPoseCount);
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
                clock.clock().nowTimestamp(),
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

    @Test
    public void runtimePathBuilderRejectsObjectConstraintDraftsBeforeMutation() {
        Fixture fixture = new Fixture();
        PathConstraints defaults = new PathConstraints(0.91, 42.0, 1.3, 0.8);
        Path first = new Path(
                new BezierLine(new Pose(), new Pose(12.0, 0.0, 0.0)),
                defaults
        );
        PathBuilder builder = PedroPathingRuntime.newPathBuilder(fixture.follower, defaults)
                .addPath(first);

        assertNullContaining(() -> builder.setConstraints(null), "pathConstraints");
        assertNullContaining(() -> builder.setConstraintsForAll(null), "pathConstraints");
        assertNullContaining(() -> builder.setConstraintsForLast(null), "pathConstraints");

        PathConstraints invalid = defaults.copy();
        invalid.setHeadingConstraint(Double.POSITIVE_INFINITY);
        assertInvalidContaining(() -> builder.setConstraints(invalid), "headingConstraint");
        assertInvalidContaining(() -> builder.setConstraintsForAll(invalid), "headingConstraint");
        assertInvalidContaining(() -> builder.setConstraintsForLast(invalid), "headingConstraint");

        PathConstraints invalidSearchLimit = defaults.copy();
        invalidSearchLimit.setBEZIER_CURVE_SEARCH_LIMIT(0);
        assertInvalidContaining(
                () -> builder.setConstraints(invalidSearchLimit),
                "BEZIER_CURVE_SEARCH_LIMIT"
        );
        assertInvalidContaining(
                () -> builder.setConstraintsForAll(invalidSearchLimit),
                "BEZIER_CURVE_SEARCH_LIMIT"
        );
        assertInvalidContaining(
                () -> builder.setConstraintsForLast(invalidSearchLimit),
                "BEZIER_CURVE_SEARCH_LIMIT"
        );

        assertEquals(0.007, first.getPathEndHeadingConstraint(), EPSILON);
        assertEquals(
                defaults.getBEZIER_CURVE_SEARCH_LIMIT(),
                first.getConstraints().getBEZIER_CURVE_SEARCH_LIMIT()
        );
        PathChain built = builder
                .addPath(new BezierLine(
                        new Pose(12.0, 0.0, 0.0),
                        new Pose(24.0, 0.0, 0.0)
                ))
                .build();
        assertEquals(0.91, built.getPath(0).getPathEndTValueConstraint(), EPSILON);
        assertEquals(0.91, built.getPath(1).getPathEndTValueConstraint(), EPSILON);
    }

    @Test
    public void runtimePathBuilderRejectsEveryScalarConstraintBeforeMutation() {
        Fixture fixture = new Fixture();
        PathConstraints defaults = new PathConstraints(0.91, 42.0, 1.3, 0.8);
        Path path = new Path(
                new BezierLine(new Pose(), new Pose(12.0, 0.0, 0.0)),
                defaults
        );
        PathBuilder builder = PedroPathingRuntime.newPathBuilder(fixture.follower, defaults)
                .addPath(path);

        assertInvalidContaining(() -> builder.setBrakingStrength(0.0), "brakingStrength");
        assertInvalidContaining(() -> builder.setBrakingStart(-1.0), "brakingStart");
        assertInvalidContaining(() -> builder.setVelocityConstraint(-1.0), "velocityConstraint");
        assertInvalidContaining(
                () -> builder.setTranslationalConstraint(-1.0),
                "translationalConstraint"
        );
        assertInvalidContaining(() -> builder.setHeadingConstraint(-1.0), "headingConstraint");
        assertInvalidContaining(() -> builder.setTValueConstraint(0.0), "tValueConstraint");
        assertInvalidContaining(() -> builder.setTimeoutConstraint(-1.0), "timeoutConstraint");

        assertEquals(1.3, path.getBrakingStrength(), EPSILON);
        assertEquals(0.1, path.getPathEndVelocityConstraint(), EPSILON);
        assertEquals(0.1, path.getPathEndTranslationalConstraint(), EPSILON);
        assertEquals(0.007, path.getPathEndHeadingConstraint(), EPSILON);
        assertEquals(0.91, path.getPathEndTValueConstraint(), EPSILON);
        assertEquals(42.0, path.getPathEndTimeoutConstraint(), EPSILON);
        assertEquals(0.8, defaults.getBrakingStart(), EPSILON);
        PathChain built = builder.build();
        assertEquals(0.8, built.getPath(0).getBrakingStartMultiplier(), EPSILON);
    }

    @Test
    public void runtimePathBuilderChecksLaterBrakingArithmeticAndGlobalRollback() {
        FakePredictorAccess access = new FakePredictorAccess();
        PedroPathingPassiveLocalizer localizer = passiveLocalizer(access);
        FollowerConstants constants = new FollowerConstants();
        constants.forwardZeroPowerAcceleration = -1.0;
        constants.lateralZeroPowerAcceleration = -100.0;
        Follower follower = new Follower(
                constants,
                localizer,
                new FakeDrivetrain(),
                PathConstraints.defaultConstraints.copy()
        );
        localizer.completeFollowerConstruction();
        PathConstraints defaults = new PathConstraints(0.91, 42.0, 1.3, 0.8);
        Path path = new Path(
                new BezierLine(new Pose(), new Pose(12.0, 0.0, 0.0)),
                defaults
        );
        PathBuilder builder = PedroPathingRuntime.newPathBuilder(follower, defaults)
                .addPath(path);

        assertInvalidContaining(
                () -> builder.setBrakingStrength(Double.MAX_VALUE),
                "must remain finite"
        );
        double forwardAcceleration = Math.abs(
                follower.getConstants().forwardZeroPowerAcceleration
        );
        double doubledOnlyOverflow =
                (Double.MAX_VALUE / (forwardAcceleration * 4.0)) * 0.75;
        assertInvalidContaining(
                () -> builder.setBrakingStrength(doubledOnlyOverflow),
                "2.0 *",
                "forwardZeroPowerAcceleration",
                "brakingStrength"
        );
        double lateralAcceleration = Math.abs(
                follower.getConstants().lateralZeroPowerAcceleration
        );
        double lateralOnlyOverflow =
                (Double.MAX_VALUE / (lateralAcceleration * 4.0)) * 1.05;
        assertInvalidContaining(
                () -> builder.setBrakingStrength(lateralOnlyOverflow),
                "lateralZeroPowerAcceleration",
                "brakingStrength",
                "must remain finite"
        );
        double lateralDoubledOnlyOverflow =
                (Double.MAX_VALUE / (lateralAcceleration * 4.0)) * 0.75;
        assertInvalidContaining(
                () -> builder.setBrakingStrength(lateralDoubledOnlyOverflow),
                "2.0 *",
                "lateralZeroPowerAcceleration",
                "brakingStrength"
        );
        assertInvalidContaining(
                () -> builder.setBrakingStart(Double.MAX_VALUE),
                "times"
        );
        assertInvalidContaining(
                () -> builder.setGlobalDeceleration(Double.MAX_VALUE),
                "times"
        );

        PathChain built = builder.build();
        assertEquals(PathChain.DecelerationType.LAST_PATH, built.getDecelerationType());
        assertEquals(1.3, built.getPath(0).getBrakingStrength(), EPSILON);
        assertEquals(0.8, built.getPath(0).getBrakingStartMultiplier(), EPSILON);
    }

    @Test
    public void runtimePathBuilderChecksLaterLateralStoppingDistanceBeforeMutation() {
        FakePredictorAccess access = new FakePredictorAccess();
        PedroPathingPassiveLocalizer localizer = passiveLocalizer(access);
        Follower follower = new Follower(
                new FollowerConstants(),
                localizer,
                new FakeDrivetrain(Double.MIN_VALUE, 1e154),
                PathConstraints.defaultConstraints.copy()
        );
        localizer.completeFollowerConstruction();
        PathConstraints defaults = new PathConstraints(0.91, 42.0, 1.3, 0.8);
        PathBuilder builder = PedroPathingRuntime.newPathBuilder(follower, defaults)
                .addPath(new BezierLine(new Pose(), new Pose(12.0, 0.0, 0.0)));

        assertInvalidContaining(
                () -> builder.setBrakingStart(1e10),
                "yVelocity",
                "brakingStart",
                "times"
        );

        assertEquals(0.8, builder.build().getPath(0).getBrakingStartMultiplier(), EPSILON);
    }

    @Test
    public void runtimeCurrentPedroPoseIsDefensiveAndNeverPollsPredictor() throws Exception {
        FakePredictorAccess access = new FakePredictorAccess();
        PedroPathingPassiveLocalizer localizer = passiveLocalizer(access);
        CoordinateSystem coordinateSystem = new CoordinateSystem() {
            @Override
            public Pose convertToPedro(Pose pose) {
                return pose;
            }

            @Override
            public Pose convertFromPedro(Pose pose) {
                return pose;
            }
        };
        Pose cached = new Pose(18.25, -7.5, 0.375, coordinateSystem);
        CountingPoseFollower follower = new CountingPoseFollower(
                localizer,
                new FakeDrivetrain(),
                cached
        );
        localizer.completeFollowerConstruction();
        Constructor<PedroPathingRuntime> constructor = PedroPathingRuntime.class
                .getDeclaredConstructor(
                        PinpointOdometryPredictor.class,
                        PedroPathingPassiveLocalizer.class,
                        Follower.class,
                        PedroPathingDriveAdapter.class,
                        PathConstraints.class
                );
        constructor.setAccessible(true);
        PedroPathingRuntime runtime = constructor.newInstance(
                null,
                localizer,
                follower,
                null,
                PathConstraints.defaultConstraints.copy()
        );
        int predictorReadsBefore = access.readCount;

        Pose first = runtime.currentPedroPose();
        Pose second = runtime.currentPedroPose();

        assertEquals(2, follower.poseReadCount);
        assertNotSame(cached, first);
        assertNotSame(first, second);
        assertEquals(cached.getX(), first.getX(), 0.0);
        assertEquals(cached.getY(), first.getY(), 0.0);
        assertEquals(cached.getHeading(), first.getHeading(), 0.0);
        assertSame(cached.getCoordinateSystem(), first.getCoordinateSystem());
        assertEquals(cached.getX(), second.getX(), 0.0);
        assertEquals(cached.getY(), second.getY(), 0.0);
        assertEquals(cached.getHeading(), second.getHeading(), 0.0);
        assertSame(cached.getCoordinateSystem(), second.getCoordinateSystem());
        assertEquals(predictorReadsBefore, access.readCount);
    }

    private static PedroPathingPassiveLocalizer passiveLocalizer(
            FakePredictorAccess access) {
        return new PedroPathingPassiveLocalizer(
                access,
                PedroFieldTransform.decodeInvertedFtc()
        );
    }

    private static PedroPathingPassiveLocalizer.Sample sampled(long cycle,
                                                               LoopTimestamp timestamp,
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
                timestamp,
                velocityX,
                velocityY,
                angularVelocity,
                totalHeading
        );
    }

    private static void assertFailsContaining(Runnable action, String... expectedMessages) {
        try {
            action.run();
            fail("Expected failure containing: " + java.util.Arrays.toString(expectedMessages));
        } catch (IllegalStateException expected) {
            for (String expectedMessage : expectedMessages) {
                assertTrue(
                        "Expected message containing '" + expectedMessage + "' but got: "
                                + expected.getMessage(),
                        expected.getMessage().contains(expectedMessage)
                );
            }
        }
    }

    private static void assertInvalidContaining(Runnable action, String... expectedMessages) {
        try {
            action.run();
            fail("Expected invalid value containing: "
                    + java.util.Arrays.toString(expectedMessages));
        } catch (IllegalArgumentException expected) {
            for (String expectedMessage : expectedMessages) {
                assertTrue(
                        "Expected message containing '" + expectedMessage + "' but got: "
                                + expected.getMessage(),
                        expected.getMessage().contains(expectedMessage)
                );
            }
        }
    }

    private static void assertNullContaining(Runnable action, String... expectedMessages) {
        try {
            action.run();
            fail("Expected null value containing: "
                    + java.util.Arrays.toString(expectedMessages));
        } catch (NullPointerException expected) {
            for (String expectedMessage : expectedMessages) {
                assertTrue(
                        "Expected message containing '" + expectedMessage + "' but got: "
                                + expected.getMessage(),
                        expected.getMessage().contains(expectedMessage)
                );
            }
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
        String statusSummary = "unavailable";
        int readCount;
        int setPoseCount;

        @Override
        public PedroPathingPassiveLocalizer.Sample currentSnapshot() {
            readCount++;
            return sample;
        }

        @Override
        public String lastDeviceStatusSummary() {
            return statusSummary;
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
                    sample.timestamp,
                    sample.phoenixFieldVelocityXInchesPerSec,
                    sample.phoenixFieldVelocityYInchesPerSec,
                    sample.angularVelocityRadPerSec,
                    sample.totalHeadingRad
            );
        }
    }

    private static final class CountingPoseFollower extends Follower {
        private Pose cachedPose;
        int poseReadCount;

        CountingPoseFollower(PedroPathingPassiveLocalizer localizer,
                             Drivetrain drivetrain,
                             Pose cachedPose) {
            super(
                    new FollowerConstants(),
                    localizer,
                    drivetrain,
                    PathConstraints.defaultConstraints.copy()
            );
            this.cachedPose = cachedPose;
        }

        @Override
        public Pose getPose() {
            if (cachedPose == null) {
                return super.getPose();
            }
            poseReadCount++;
            return cachedPose;
        }
    }

    private static final class FakeDrivetrain extends Drivetrain {
        int breakCount;
        double xVelocity;
        double yVelocity;

        FakeDrivetrain() {
            this(80.0, 65.0);
        }

        FakeDrivetrain(double xVelocity, double yVelocity) {
            this.xVelocity = xVelocity;
            this.yVelocity = yVelocity;
        }

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
            return "FakeDrivetrain";
        }
    }
}
