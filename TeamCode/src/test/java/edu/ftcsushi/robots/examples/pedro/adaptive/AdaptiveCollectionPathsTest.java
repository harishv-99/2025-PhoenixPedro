package edu.ftcsushi.robots.examples.pedro.adaptive;

import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.paths.callbacks.ParametricCallback;
import com.pedropathing.paths.callbacks.PathCallback;
import com.pedropathing.util.FiniteRunAction;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.concurrent.atomic.AtomicInteger;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.drive.route.RouteFollower;
import edu.ftcsushi.fw.integrations.pedro.PedroFieldTransform;
import edu.ftcsushi.fw.integrations.pedro.PedroPathingDriveAdapter;
import edu.ftcsushi.fw.integrations.pedro.PedroPathingRuntime;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Focused geometry, callback, and configuration proof for the adaptive path example. */
public final class AdaptiveCollectionPathsTest {

    private static final double EPSILON = 1e-9;

    @Test
    public void publicSurfaceHasOneRuntimeConfigConstructorAndFreshDefaults() throws Exception {
        int publicConstructors = 0;
        for (Constructor<?> constructor : AdaptiveCollectionPaths.class.getDeclaredConstructors()) {
            if (Modifier.isPublic(constructor.getModifiers())) {
                publicConstructors++;
            }
        }
        assertEquals(1, publicConstructors);
        assertTrue(Modifier.isPublic(
                AdaptiveCollectionPaths.class.getDeclaredConstructor(
                        PedroPathingRuntime.class,
                        AdaptiveCollectionPaths.Config.class
                ).getModifiers()
        ));

        AdaptiveCollectionPaths.Config first = AdaptiveCollectionPaths.Config.defaults();
        AdaptiveCollectionPaths.Config second = AdaptiveCollectionPaths.Config.defaults();
        assertNotSame(first, second);
        assertSame(PedroFieldTransform.decodeInvertedFtc(), first.fieldTransform);
        assertNotSame(first.fallbackFieldToRobotPose, second.fallbackFieldToRobotPose);
        assertNotSame(first.returnFieldToRobotPose, second.returnFieldToRobotPose);
        assertTrue(first.safeToLeavePathT > 0.0);
        assertTrue(first.safeToLeavePathT < first.nearEndPathT);
        assertTrue(first.nearEndPathT < 1.0);
    }

    @Test
    public void selectedAndUnavailableCollectionUseLivePoseAndExplicitSushiTargets()
            throws Exception {
        RuntimeFixture fixture = runtimeFixture(new Pose(10.0, 20.0, 0.25));
        AdaptiveCollectionPaths.Config config = AdaptiveCollectionPaths.Config.defaults();
        config.collectionFieldXInches = 18.0;
        config.collectionFieldHeadingRad = -0.40;
        config.fallbackFieldToRobotPose = new Pose2d(-12.0, 8.0, 0.65);
        config.returnFieldToRobotPose = new Pose2d(-30.0, -14.0, -0.75);
        config.safeToLeavePathT = 0.35;
        config.nearEndPathT = 0.82;
        AdaptiveCollectionPaths paths = new AdaptiveCollectionPaths(fixture.runtime, config);

        AdaptiveCollectionPaths.Milestones selectedMilestones = paths.newMilestones();
        PathChain selected = paths.buildCollectionFromCurrentPose(
                AdaptiveCollectionVisionService.Decision.selectedForHardwareNeutralTest(7.5),
                selectedMilestones
        );

        assertSame(fixture.adapter, paths.routeFollower());
        assertEquals(1, fixture.follower.poseReadCount);
        assertPath(
                selected,
                new Pose(10.0, 20.0, 0.25),
                config.fieldTransform.sushiFieldToPedroPose(
                        new Pose2d(18.0, 7.5, -0.40)
                )
        );
        assertEquals(2, selected.getCallbacks().size());
        assertFalse(selectedMilestones.safeToLeave());
        assertFalse(selectedMilestones.nearEnd());

        fixture.follower.cachedPose = new Pose(30.0, 40.0, 0.50);
        AdaptiveCollectionPaths.Milestones fallbackMilestones = paths.newMilestones();
        PathChain fallback = paths.buildCollectionFromCurrentPose(
                AdaptiveCollectionVisionService.Decision.unavailableForHardwareNeutralTest(
                        AdaptiveCollectionVisionService.UnavailableReason.ZERO_DETECTIONS
                ),
                fallbackMilestones
        );

        assertEquals(2, fixture.follower.poseReadCount);
        assertPath(
                fallback,
                new Pose(30.0, 40.0, 0.50),
                config.fieldTransform.sushiFieldToPedroPose(
                        new Pose2d(-12.0, 8.0, 0.65)
                )
        );
        assertNotSame(selectedMilestones, fallbackMilestones);
        assertFalse(fallbackMilestones.safeToLeave());
        assertFalse(fallbackMilestones.nearEnd());
        assertFalse(fixture.follower.isBusy());
    }

    @Test
    public void returnSamplesLaterPoseAndOwnerRetainsDefensiveConfigSnapshot() throws Exception {
        RuntimeFixture fixture = runtimeFixture(new Pose(2.0, 3.0, 0.10));
        AdaptiveCollectionPaths.Config config = AdaptiveCollectionPaths.Config.defaults();
        config.collectionFieldXInches = 22.0;
        config.collectionFieldHeadingRad = 0.30;
        config.fallbackFieldToRobotPose = new Pose2d(22.0, -9.0, 0.30);
        config.returnFieldToRobotPose = new Pose2d(-24.0, 16.0, -0.60);
        config.safeToLeavePathT = 0.30;
        config.nearEndPathT = 0.80;
        AdaptiveCollectionPaths paths = new AdaptiveCollectionPaths(fixture.runtime, config);

        config.fieldTransform = null;
        config.collectionFieldXInches = Double.NaN;
        config.collectionFieldHeadingRad = Double.NaN;
        config.fallbackFieldToRobotPose = new Pose2d(Double.NaN, 0.0, 0.0);
        config.returnFieldToRobotPose = new Pose2d(99.0, 99.0, 2.0);
        config.safeToLeavePathT = 0.01;
        config.nearEndPathT = 0.02;
        fixture.follower.cachedPose = new Pose(44.0, 55.0, 0.70);

        PathChain returnRoute = paths.buildReturnFromCurrentPose();
        assertEquals(1, fixture.follower.poseReadCount);
        assertPath(
                returnRoute,
                new Pose(44.0, 55.0, 0.70),
                PedroFieldTransform.decodeInvertedFtc().sushiFieldToPedroPose(
                        new Pose2d(-24.0, 16.0, -0.60)
                )
        );
        assertEquals(0, returnRoute.getCallbacks().size());
        assertFalse(fixture.follower.isBusy());
    }

    @Test
    public void nativeCallbacksOnlyLatchFreshSemanticMilestonesAtCapturedThresholds()
            throws Exception {
        RuntimeFixture fixture = runtimeFixture(new Pose(4.0, 6.0, 0.20));
        AdaptiveCollectionPaths.Config config = AdaptiveCollectionPaths.Config.defaults();
        config.safeToLeavePathT = 0.25;
        config.nearEndPathT = 0.88;
        AdaptiveCollectionPaths paths = new AdaptiveCollectionPaths(fixture.runtime, config);
        config.safeToLeavePathT = 0.40;
        config.nearEndPathT = 0.60;

        AdaptiveCollectionPaths.Milestones milestones = paths.newMilestones();
        PathChain route = paths.buildCollectionFromCurrentPose(
                AdaptiveCollectionVisionService.Decision.selectedForHardwareNeutralTest(3.0),
                milestones
        );
        assertEquals(0.25, callbackThreshold(route.getCallbacks().get(0)), 0.0);
        assertEquals(0.88, callbackThreshold(route.getCallbacks().get(1)), 0.0);

        assertTrue(route.getCallbacks().get(0).run());
        assertTrue(milestones.safeToLeave());
        assertFalse(milestones.nearEnd());
        assertFalse(fixture.follower.isBusy());

        assertTrue(route.getCallbacks().get(1).run());
        assertTrue(milestones.safeToLeave());
        assertTrue(milestones.nearEnd());
        assertFalse(fixture.follower.isBusy());

        AdaptiveCollectionPaths.Milestones fresh = paths.newMilestones();
        assertNotSame(milestones, fresh);
        assertFalse(fresh.safeToLeave());
        assertFalse(fresh.nearEnd());
    }

    @Test
    public void coincidentTranslationUsesPedroPointAndDisplacedTranslationUsesLine() {
        Pose start = new Pose(17.0, -4.0, 0.30);
        assertTrue(AdaptiveCollectionPaths.curveFrom(
                start,
                new Pose(17.0, -4.0, 1.10)
        ) instanceof BezierPoint);
        assertTrue(AdaptiveCollectionPaths.curveFrom(
                start,
                new Pose(18.0, -4.0, 1.10)
        ) instanceof BezierLine);

        assertCurveRejected(
                new Pose(Double.NaN, -4.0, 0.30),
                new Pose(18.0, -4.0, 1.10),
                "sampledPedroStartPose.x must be finite"
        );
        CoordinateSystem wrongCoordinates = new CoordinateSystem() {
            @Override
            public Pose convertToPedro(Pose pose) {
                return pose;
            }

            @Override
            public Pose convertFromPedro(Pose pose) {
                return pose;
            }
        };
        assertCurveRejected(
                start,
                new Pose(18.0, -4.0, 1.10, wrongCoordinates),
                "must be explicitly tagged with PedroCoordinates"
        );
    }

    @Test
    public void configurationRejectsInvalidGeometryAndMilestoneOrdering() throws Exception {
        RuntimeFixture fixture = runtimeFixture(new Pose());

        assertConfigRejected(fixture.runtime, c -> c.fieldTransform = null, "fieldTransform");
        assertConfigRejected(
                fixture.runtime,
                c -> c.collectionFieldXInches = Double.NaN,
                "collectionFieldXInches must be finite"
        );
        assertConfigRejected(
                fixture.runtime,
                c -> c.collectionFieldHeadingRad = Double.POSITIVE_INFINITY,
                "collectionFieldHeadingRad must be finite"
        );
        assertConfigRejected(
                fixture.runtime,
                c -> c.fallbackFieldToRobotPose = null,
                "fallbackFieldToRobotPose"
        );
        assertConfigRejected(
                fixture.runtime,
                c -> c.fallbackFieldToRobotPose = new Pose2d(0.0, Double.NaN, 0.0),
                "fallbackFieldToRobotPose.yInches must be finite"
        );
        assertConfigRejected(
                fixture.runtime,
                c -> c.returnFieldToRobotPose = new Pose2d(0.0, 0.0, Double.NEGATIVE_INFINITY),
                "returnFieldToRobotPose.headingRad must be finite"
        );
        assertConfigRejected(
                fixture.runtime,
                c -> c.safeToLeavePathT = 0.0,
                "safeToLeavePathT must be > 0 and < 1"
        );
        assertConfigRejected(
                fixture.runtime,
                c -> c.nearEndPathT = 1.0,
                "nearEndPathT must be > 0 and < 1"
        );
        assertConfigRejected(
                fixture.runtime,
                c -> {
                    c.safeToLeavePathT = 0.8;
                    c.nearEndPathT = 0.8;
                },
                "safeToLeavePathT must be less than nearEndPathT"
        );

        AdaptiveCollectionPaths.Config positiveBoundary =
                AdaptiveCollectionPaths.Config.defaults();
        positiveBoundary.safeToLeavePathT = Double.MIN_VALUE;
        positiveBoundary.nearEndPathT = Math.nextDown(1.0);
        new AdaptiveCollectionPaths(fixture.runtime, positiveBoundary);
    }

    @Test
    public void buildFailureAndNullResultsNeverStartTheFollower() {
        AtomicInteger followCount = new AtomicInteger();
        RouteFollower<PathChain> follower = route -> {
            followCount.incrementAndGet();
            throw new AssertionError("Path construction must not follow a route");
        };
        RuntimeException constructionFailure = new RuntimeException("bad geometry");
        AdaptiveCollectionPaths throwing = new AdaptiveCollectionPaths(
                follower,
                (decision, milestones) -> {
                    throw constructionFailure;
                },
                PathChain::new
        );
        try {
            throwing.buildCollectionFromCurrentPose(
                    AdaptiveCollectionVisionService.Decision.selectedForHardwareNeutralTest(0.0),
                    throwing.newMilestones()
            );
            fail("Expected construction failure");
        } catch (RuntimeException expected) {
            assertSame(constructionFailure, expected);
        }
        assertEquals(0, followCount.get());

        AdaptiveCollectionPaths nullBuilders = new AdaptiveCollectionPaths(
                follower,
                (decision, milestones) -> null,
                () -> null
        );
        assertBuildRejected(
                () -> nullBuilders.buildCollectionFromCurrentPose(
                        AdaptiveCollectionVisionService.Decision.selectedForHardwareNeutralTest(0.0),
                        nullBuilders.newMilestones()
                ),
                "collectionRouteBuilder returned null"
        );
        assertBuildRejected(
                nullBuilders::buildReturnFromCurrentPose,
                "returnRouteBuilder returned null"
        );
        assertEquals(0, followCount.get());
    }

    private static void assertPath(PathChain route, Pose expectedStart, Pose expectedTarget) {
        assertEquals(1, route.size());
        Path path = route.firstPath();
        assertPose(expectedStart, path.getFirstControlPoint());
        assertPose(expectedTarget, path.getLastControlPoint());
        assertEquals(expectedStart.getHeading(), path.getHeadingGoal(0.0), EPSILON);
        assertEquals(expectedTarget.getHeading(), path.getHeadingGoal(1.0), EPSILON);
        assertSame(PedroCoordinates.INSTANCE, path.getFirstControlPoint().getCoordinateSystem());
        assertSame(PedroCoordinates.INSTANCE, path.getLastControlPoint().getCoordinateSystem());
    }

    private static void assertPose(Pose expected, Pose actual) {
        assertEquals(expected.getX(), actual.getX(), EPSILON);
        assertEquals(expected.getY(), actual.getY(), EPSILON);
        assertEquals(expected.getHeading(), actual.getHeading(), EPSILON);
    }

    private static double callbackThreshold(PathCallback callback) throws Exception {
        assertTrue(callback instanceof FiniteRunAction);
        Field wrappedField = FiniteRunAction.class.getDeclaredField("callback");
        wrappedField.setAccessible(true);
        Object wrapped = wrappedField.get(callback);
        assertTrue(wrapped instanceof ParametricCallback);
        return ((ParametricCallback) wrapped).getStartCondition();
    }

    private static void assertConfigRejected(PedroPathingRuntime runtime,
                                             ConfigEdit edit,
                                             String expectedMessagePart) {
        AdaptiveCollectionPaths.Config config = AdaptiveCollectionPaths.Config.defaults();
        edit.apply(config);
        try {
            new AdaptiveCollectionPaths(runtime, config);
            fail("Expected invalid adaptive path configuration");
        } catch (RuntimeException expected) {
            assertTrue(
                    "Expected message containing '" + expectedMessagePart + "' but was '"
                            + expected.getMessage() + "'",
                    expected.getMessage().contains(expectedMessagePart)
            );
        }
    }

    private static void assertCurveRejected(Pose start,
                                            Pose target,
                                            String expectedMessagePart) {
        try {
            AdaptiveCollectionPaths.curveFrom(start, target);
            fail("Expected invalid Pedro curve endpoint");
        } catch (RuntimeException expected) {
            assertTrue(expected.getMessage().contains(expectedMessagePart));
        }
    }

    private static void assertBuildRejected(BuildCall call, String expectedMessagePart) {
        try {
            call.run();
            fail("Expected invalid adaptive route build");
        } catch (RuntimeException expected) {
            assertTrue(expected.getMessage().contains(expectedMessagePart));
        }
    }

    private static RuntimeFixture runtimeFixture(Pose initialPose) throws Exception {
        PathConstraints constraints = PathConstraints.defaultConstraints.copy();
        CountingFollower follower = new CountingFollower(
                new FakeLocalizer(initialPose),
                new FakeDrivetrain(),
                initialPose,
                constraints.copy()
        );
        PedroPathingDriveAdapter adapter = new PedroPathingDriveAdapter(follower);
        PedroPathingRuntime runtime = null;
        for (Constructor<?> constructor : PedroPathingRuntime.class.getDeclaredConstructors()) {
            if (constructor.getParameterTypes().length == 5) {
                constructor.setAccessible(true);
                runtime = (PedroPathingRuntime) constructor.newInstance(
                        null,
                        null,
                        follower,
                        adapter,
                        constraints
                );
                break;
            }
        }
        if (runtime == null) {
            fail("PedroPathingRuntime five-role constructor was not found");
        }
        return new RuntimeFixture(runtime, follower, adapter);
    }

    private interface ConfigEdit {
        void apply(AdaptiveCollectionPaths.Config config);
    }

    private interface BuildCall {
        void run();
    }

    private static final class RuntimeFixture {
        final PedroPathingRuntime runtime;
        final CountingFollower follower;
        final PedroPathingDriveAdapter adapter;

        private RuntimeFixture(PedroPathingRuntime runtime,
                               CountingFollower follower,
                               PedroPathingDriveAdapter adapter) {
            this.runtime = runtime;
            this.follower = follower;
            this.adapter = adapter;
        }
    }

    private static final class CountingFollower extends Follower {
        Pose cachedPose;
        int poseReadCount;

        private CountingFollower(Localizer localizer,
                                 Drivetrain drivetrain,
                                 Pose cachedPose,
                                 PathConstraints constraints) {
            super(new FollowerConstants(), localizer, drivetrain, constraints);
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

    private static final class FakeLocalizer implements Localizer {
        private Pose pose;

        private FakeLocalizer(Pose pose) {
            this.pose = pose;
        }

        @Override
        public Pose getPose() {
            return pose;
        }

        @Override
        public Pose getVelocity() {
            return new Pose();
        }

        @Override
        public Vector getVelocityVector() {
            return getVelocity().getAsVector();
        }

        @Override
        public void setStartPose(Pose startPose) {
            pose = startPose;
        }

        @Override
        public void setPose(Pose nextPose) {
            pose = nextPose;
        }

        @Override
        public void update() {
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
        }

        @Override
        public double getIMUHeading() {
            return pose.getHeading();
        }

        @Override
        public boolean isNAN() {
            return false;
        }
    }

    private static final class FakeDrivetrain extends Drivetrain {
        private FakeDrivetrain() {
            setMaxPowerScaling(1.0);
            setNominalVoltage(12.0);
        }

        @Override
        public double[] calculateDrive(Vector correctivePower,
                                       Vector headingPower,
                                       Vector pathingPower,
                                       double robotHeading) {
            return new double[4];
        }

        @Override
        public void updateConstants() {
        }

        @Override
        public void breakFollowing() {
        }

        @Override
        public void runDrive(double[] drivePowers) {
        }

        @Override
        public void startTeleopDrive() {
        }

        @Override
        public void startTeleopDrive(boolean brakeMode) {
        }

        @Override
        public double xVelocity() {
            return 0.0;
        }

        @Override
        public double yVelocity() {
            return 0.0;
        }

        @Override
        public void setXVelocity(double xMovement) {
        }

        @Override
        public void setYVelocity(double yMovement) {
        }

        @Override
        public double getVoltage() {
            return 12.0;
        }

        @Override
        public String debugString() {
            return "AdaptiveCollectionPathsTest.FakeDrivetrain";
        }
    }
}
