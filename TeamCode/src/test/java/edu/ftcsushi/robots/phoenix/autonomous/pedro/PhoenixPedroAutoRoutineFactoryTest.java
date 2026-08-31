package edu.ftcsushi.robots.phoenix.autonomous.pedro;

import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathConstraints;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;

import edu.ftcsushi.fw.drive.DriveCommandSink;
import edu.ftcsushi.fw.drive.guidance.DriveGuidanceTask;
import edu.ftcsushi.fw.integrations.pedro.PedroPathingDriveAdapter;
import edu.ftcsushi.fw.integrations.pedro.PedroPathingRuntime;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.Tasks;
import edu.ftcsushi.robots.phoenix.PhoenixAutoConfig;
import edu.ftcsushi.robots.phoenix.PhoenixCapabilities;
import edu.ftcsushi.robots.phoenix.autonomous.PhoenixAutoSpec;
import edu.ftcsushi.robots.phoenix.autonomous.PhoenixAutoStrategyId;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the public routine factory keeps every strategy on one fresh bounded policy graph. */
public final class PhoenixPedroAutoRoutineFactoryTest {

    @Test
    public void everyStrategyBuildReturnsFreshCoordinatorAndFreshRoleTasks() throws Exception {
        for (PhoenixAutoStrategyId strategy : PhoenixAutoStrategyId.values()) {
            PhoenixPedroAutoContext context = contextFor(strategy);

            Task first = PhoenixPedroAutoRoutineFactory.build(context);
            Task second = PhoenixPedroAutoRoutineFactory.build(context);

            assertTrue(strategy.toString(), first instanceof PhoenixPedroAutoRoutineTask);
            assertTrue(strategy.toString(), second instanceof PhoenixPedroAutoRoutineTask);
            assertNotSame(strategy.toString(), first, second);
            Object firstPrePark = role(first, "prePark");
            Object secondPrePark = role(second, "prePark");
            assertTrue(strategy.toString(), firstPrePark instanceof PhoenixPedroPreParkTask);
            assertTrue(strategy.toString(), secondPrePark instanceof PhoenixPedroPreParkTask);
            assertNotSame(strategy + " prePark", firstPrePark, secondPrePark);
            assertNotSame(
                    strategy + " boundedPrePark",
                    role(first, "boundedPrePark"),
                    role(second, "boundedPrePark")
            );
            assertNotSame(
                    strategy + " outboundRoute",
                    preParkRole(firstPrePark, "outboundRoute"),
                    preParkRole(secondPrePark, "outboundRoute")
            );
            assertNotSame(
                    strategy + " scoringAttempt",
                    preParkRole(firstPrePark, "scoringAttempt"),
                    preParkRole(secondPrePark, "scoringAttempt")
            );
            assertNotSame(
                    strategy + " returnOrParkRoute",
                    role(first, "returnOrParkRoute"),
                    role(second, "returnOrParkRoute")
            );
            assertTrue(
                    first.getDebugName(),
                    first.getDebugName().contains(expectedRoutineName(strategy))
            );
        }
    }

    @Test
    public void invalidMatchTakeoverThresholdFailsBeforeRoutineConstruction() throws Exception {
        double[] invalid = {
                0.0,
                -1.0,
                Double.NaN,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY
        };
        for (double value : invalid) {
            PhoenixAutoConfig autoConfig = PhoenixAutoConfig.defaults();
            autoConfig.parkTakeoverElapsedSec = value;
            PhoenixPedroAutoContext context = contextFor(
                    PhoenixAutoStrategyId.SAFE_PRELOAD,
                    autoConfig
            );

            try {
                PhoenixPedroAutoRoutineFactory.build(context);
                fail("expected invalid park takeover threshold " + value);
            } catch (IllegalArgumentException expected) {
                assertTrue(expected.getMessage().contains("parkTakeoverElapsedSec"));
                assertTrue(expected.getMessage().contains("finite"));
                assertTrue(expected.getMessage().contains("> 0"));
            }
        }
    }

    @Test
    public void invalidRouteTimeoutFailsAfterValidParkPolicy() throws Exception {
        double[] invalid = {
                0.0,
                -1.0,
                Double.NaN,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY
        };
        for (double value : invalid) {
            PhoenixAutoConfig autoConfig = PhoenixAutoConfig.defaults();
            autoConfig.routeTimeoutSec = value;
            PhoenixPedroAutoContext context = contextFor(
                    PhoenixAutoStrategyId.SAFE_PRELOAD,
                    autoConfig
            );

            try {
                PhoenixPedroAutoRoutineFactory.build(context);
                fail("expected invalid route timeout " + value);
            } catch (IllegalArgumentException expected) {
                assertTrue(expected.getMessage().contains("PhoenixAutoConfig.routeTimeoutSec"));
                assertTrue(expected.getMessage().contains("finite"));
                assertTrue(expected.getMessage().contains("> 0"));
            }
        }
    }

    @Test
    public void routinePolicyValidationUsesPhoenixAutoConfigSourceOrder() throws Exception {
        PhoenixAutoConfig autoConfig = PhoenixAutoConfig.defaults();
        autoConfig.parkTakeoverElapsedSec = 0.0;
        autoConfig.routeTimeoutSec = 0.0;
        PhoenixPedroAutoContext context = contextFor(
                PhoenixAutoStrategyId.SAFE_PRELOAD,
                autoConfig
        );

        try {
            PhoenixPedroAutoRoutineFactory.build(context);
            fail("expected first invalid routine-policy field");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains(
                    "PhoenixAutoConfig.parkTakeoverElapsedSec"
            ));
        }
    }

    @Test
    public void pathFactoryRequiresAndValidatesItsActiveAutoSlice() throws Exception {
        try {
            contextFor(PhoenixAutoStrategyId.PEDRO_INTEGRATION_TEST, null);
            fail("expected explicit path Auto policy requirement");
        } catch (NullPointerException expected) {
            assertTrue(expected.getMessage().contains("autoConfig"));
        }

        for (double invalid : new double[]{
                0.0,
                -1.0,
                Double.NaN,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY
        }) {
            PhoenixAutoConfig autoConfig = PhoenixAutoConfig.defaults();
            autoConfig.pedroIntegrationTestDistanceIn = invalid;
            try {
                contextFor(PhoenixAutoStrategyId.PEDRO_INTEGRATION_TEST, autoConfig);
                fail("expected invalid integration distance " + invalid);
            } catch (IllegalArgumentException expected) {
                assertTrue(expected.getMessage().contains(
                        "PhoenixAutoConfig.pedroIntegrationTestDistanceIn"
                ));
                assertTrue(expected.getMessage().contains("finite"));
                assertTrue(expected.getMessage().contains("> 0"));
            }
        }
    }

    @Test
    public void pathFactoryCapturesOnlyItsDistanceBeforeLaterSourceMutation() throws Exception {
        PhoenixAutoConfig source = PhoenixAutoConfig.defaults();
        source.pedroIntegrationTestDistanceIn = 18.25;
        PhoenixPedroAutoContext context = contextFor(
                PhoenixAutoStrategyId.PEDRO_INTEGRATION_TEST,
                source
        );

        source.pedroIntegrationTestDistanceIn = 99.0;
        Field capturedDistance = PhoenixPedroPathFactory.class.getDeclaredField(
                "pedroIntegrationTestDistanceIn"
        );
        capturedDistance.setAccessible(true);
        assertEquals(18.25, capturedDistance.getDouble(context.pathFactory()), 0.0);
        for (Field field : PhoenixPedroPathFactory.class.getDeclaredFields()) {
            assertTrue(
                    "path factory must not retain the broad Auto policy",
                    field.getType() != PhoenixAutoConfig.class
            );
        }
    }

    @Test
    public void contextSnapshotsAutoPolicyAndReturnsOnlyDefensiveCopies() throws Exception {
        PhoenixAutoConfig source = PhoenixAutoConfig.defaults();
        source.parkTakeoverElapsedSec = 23.5;
        source.routeTimeoutSec = 3.25;
        PhoenixPedroAutoContext context = contextFor(
                PhoenixAutoStrategyId.PRELOAD_AND_PARK,
                source
        );

        source.parkTakeoverElapsedSec = 1.0;
        source.routeTimeoutSec = 1.0;
        PhoenixAutoConfig first = context.autoConfig();
        assertEquals(23.5, first.parkTakeoverElapsedSec, 0.0);
        assertEquals(3.25, first.routeTimeoutSec, 0.0);

        first.parkTakeoverElapsedSec = 2.0;
        first.routeTimeoutSec = 2.0;
        PhoenixAutoConfig second = context.autoConfig();
        assertNotSame(first, second);
        assertEquals(23.5, second.parkTakeoverElapsedSec, 0.0);
        assertEquals(3.25, second.routeTimeoutSec, 0.0);

        try {
            PhoenixPedroAutoContext.class.getMethod("profile");
            fail("context must not export the broad Phoenix profile");
        } catch (NoSuchMethodException expected) {
            // Exact narrow context surface.
        }
    }

    @Test
    public void contextRequiresAnExplicitAutoPolicy() throws Exception {
        PhoenixPedroAutoContext valid = contextFor(PhoenixAutoStrategyId.SAFE_PRELOAD);
        try {
            new PhoenixPedroAutoContext(
                    valid.spec(),
                    null,
                    valid.capabilities(),
                    valid.driveAdapter(),
                    valid.pathFactory(),
                    valid.paths()
            );
            fail("context must not invent a default Auto policy");
        } catch (NullPointerException expected) {
            assertTrue(expected.getMessage().contains("autoConfig"));
        }
    }

    @Test
    public void routineCapturesPositiveBoundaryValuesFromOneContextSnapshot() throws Exception {
        PhoenixAutoConfig autoConfig = PhoenixAutoConfig.defaults();
        autoConfig.parkTakeoverElapsedSec = Double.MIN_VALUE;
        autoConfig.routeTimeoutSec = Double.MIN_VALUE;
        PhoenixPedroAutoContext context = contextFor(
                PhoenixAutoStrategyId.PEDRO_INTEGRATION_TEST,
                autoConfig
        );

        Task routine = PhoenixPedroAutoRoutineFactory.build(context);
        assertEquals(
                Double.MIN_VALUE,
                ((Double) role(routine, "parkTakeoverElapsedSec")).doubleValue(),
                0.0
        );
        Object prePark = role(routine, "prePark");
        assertEquals(
                Double.MIN_VALUE,
                ((Double) routeRole(preParkRole(prePark, "outboundRoute"), "taskTimeoutSec"))
                        .doubleValue(),
                0.0
        );
        assertEquals(
                Double.MIN_VALUE,
                ((Double) routeRole(role(routine, "returnOrParkRoute"), "taskTimeoutSec"))
                        .doubleValue(),
                0.0
        );
    }

    private static PhoenixPedroAutoContext contextFor(PhoenixAutoStrategyId strategy)
            throws Exception {
        return contextFor(strategy, PhoenixAutoConfig.defaults());
    }

    private static PhoenixPedroAutoContext contextFor(PhoenixAutoStrategyId strategy,
                                                       PhoenixAutoConfig autoConfig)
            throws Exception {
        PathConstraints constraints = PathConstraints.defaultConstraints.copy();
        Follower follower = new Follower(
                new FollowerConstants().automaticHoldEnd(true),
                new FactoryLocalizer(new Pose()),
                new FactoryDrivetrain(),
                constraints.copy()
        );
        PedroPathingDriveAdapter adapter = new PedroPathingDriveAdapter(follower);
        PedroPathingRuntime runtime = runtimeForPathConstruction(
                follower,
                adapter,
                constraints.copy()
        );
        PhoenixCapabilities capabilities = new PhoenixCapabilities(
                new FactoryScoring(),
                new FactoryTargeting()
        );
        PhoenixAutoSpec spec = PhoenixAutoSpec.builder()
                .strategy(strategy)
                .build();
        PhoenixPedroPathFactory pathFactory = new PhoenixPedroPathFactory(runtime, autoConfig);
        PhoenixPedroPathFactory.Paths paths = pathFactory.build(spec, capabilities);
        return new PhoenixPedroAutoContext(
                spec,
                autoConfig,
                capabilities,
                adapter,
                pathFactory,
                paths
        );
    }

    /** Construct only the path-building portion of the otherwise hardware-owned Pedro runtime. */
    private static PedroPathingRuntime runtimeForPathConstruction(Follower follower,
                                                                  PedroPathingDriveAdapter adapter,
                                                                  PathConstraints constraints)
            throws Exception {
        for (Constructor<?> constructor : PedroPathingRuntime.class.getDeclaredConstructors()) {
            if (constructor.getParameterTypes().length == 5) {
                constructor.setAccessible(true);
                return (PedroPathingRuntime) constructor.newInstance(
                        null,
                        null,
                        follower,
                        adapter,
                        constraints
                );
            }
        }
        fail("PedroPathingRuntime five-role constructor was not found");
        return null;
    }

    private static Object role(Task routine, String fieldName) throws Exception {
        Field field = PhoenixPedroAutoRoutineTask.class.getDeclaredField(fieldName);
        field.setAccessible(true);
        return field.get(routine);
    }

    private static Object preParkRole(Object prePark, String fieldName) throws Exception {
        Field field = PhoenixPedroPreParkTask.class.getDeclaredField(fieldName);
        field.setAccessible(true);
        return field.get(prePark);
    }

    private static Object routeRole(Object route, String fieldName) throws Exception {
        Field field = route.getClass().getDeclaredField(fieldName);
        field.setAccessible(true);
        return field.get(route);
    }

    private static String expectedRoutineName(PhoenixAutoStrategyId strategy) {
        switch (strategy) {
            case SAFE_PRELOAD:
                return "phoenix.safePreload";
            case PRELOAD_AND_PARK:
                return "phoenix.preloadAndPark";
            case PARTNER_AWARE_CYCLE:
                return "phoenix.partnerAware";
            case PEDRO_INTEGRATION_TEST:
            default:
                return "pedro.integrationTest";
        }
    }

    private static final class FactoryTargeting implements PhoenixCapabilities.Targeting {
        @Override
        public PhoenixCapabilities.TargetingStatus status() {
            return null;
        }

        @Override
        public Task aimTask(DriveCommandSink driveSink, DriveGuidanceTask.Config config) {
            return Tasks.noop();
        }
    }

    private static final class FactoryScoring implements PhoenixCapabilities.Scoring {
        @Override
        public void setIntakeEnabled(boolean enabled) {
        }

        @Override
        public void setFlywheelEnabled(boolean enabled) {
        }

        @Override
        public void setShootingEnabled(boolean enabled) {
        }

        @Override
        public void setEjectEnabled(boolean enabled) {
        }

        @Override
        public void requestSingleShot() {
        }

        @Override
        public void requestShots(int shotCount) {
        }

        @Override
        public void cancelTransientActions() {
        }

        @Override
        public void setSelectedVelocityNative(double velocityNative) {
        }

        @Override
        public void adjustSelectedVelocityNative(double deltaNative) {
        }

        @Override
        public void captureSuggestedShotVelocity() {
        }

        @Override
        public boolean hasPendingShots() {
            return false;
        }

        @Override
        public PhoenixCapabilities.ScoringStatus status() {
            return null;
        }
    }

    private static final class FactoryLocalizer implements Localizer {
        private Pose pose;

        FactoryLocalizer(Pose initialPose) {
            pose = initialPose;
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

    private static final class FactoryDrivetrain extends Drivetrain {
        private double xVelocity;
        private double yVelocity;

        FactoryDrivetrain() {
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
            return "FactoryDrivetrain";
        }
    }
}
