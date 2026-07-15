package edu.ftcphoenix.robots.phoenix.autonomous.pedro;

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

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidanceTask;
import edu.ftcphoenix.fw.integrations.pedro.PedroPathingDriveAdapter;
import edu.ftcphoenix.fw.integrations.pedro.PedroPathingRuntime;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.Tasks;
import edu.ftcphoenix.robots.phoenix.PhoenixCapabilities;
import edu.ftcphoenix.robots.phoenix.PhoenixProfile;
import edu.ftcphoenix.robots.phoenix.ScoringPath;
import edu.ftcphoenix.robots.phoenix.ScoringTargeting;
import edu.ftcphoenix.robots.phoenix.autonomous.PhoenixAutoSpec;
import edu.ftcphoenix.robots.phoenix.autonomous.PhoenixAutoStrategyId;

import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the public routine factory keeps every strategy on one fresh private policy graph. */
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
            assertNotSame(
                    strategy + " outboundRoute",
                    role(first, "outboundRoute"),
                    role(second, "outboundRoute")
            );
            assertNotSame(
                    strategy + " scoringAttempt",
                    role(first, "scoringAttempt"),
                    role(second, "scoringAttempt")
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

    private static PhoenixPedroAutoContext contextFor(PhoenixAutoStrategyId strategy)
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
        PhoenixProfile profile = PhoenixProfile.defaults();
        PhoenixCapabilities capabilities = new PhoenixCapabilities(
                new FactoryScoring(),
                new FactoryTargeting()
        );
        PhoenixAutoSpec spec = PhoenixAutoSpec.builder()
                .strategy(strategy)
                .build();
        PhoenixPedroPathFactory pathFactory = new PhoenixPedroPathFactory(runtime, profile.auto);
        PhoenixPedroPathFactory.Paths paths = pathFactory.build(spec, capabilities);
        return new PhoenixPedroAutoContext(
                spec,
                profile,
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
        public ScoringTargeting.Status status(LoopClock clock) {
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
        public ScoringPath.Status status() {
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
