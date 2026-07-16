package edu.ftcphoenix.robots.phoenix.opmode;

import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.List;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.integrations.pedro.PedroFieldTransform;
import edu.ftcphoenix.fw.integrations.pedro.PedroPathingDriveAdapter;
import edu.ftcphoenix.fw.integrations.pedro.PedroPathingRuntime;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;
import edu.ftcphoenix.robots.phoenix.PhoenixProfile;
import edu.ftcphoenix.robots.phoenix.PhoenixReadiness;
import edu.ftcphoenix.robots.phoenix.PhoenixRobot;
import edu.ftcphoenix.robots.phoenix.autonomous.PhoenixAutoSpec;
import edu.ftcphoenix.robots.phoenix.autonomous.PhoenixAutoStrategyId;
import edu.ftcphoenix.robots.phoenix.autonomous.pedro.PhoenixPedroPathFactory;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

/** Verifies the shared Phoenix/Pedro OpMode's fail-closed match-arming boundary. */
public final class PhoenixPedroAutoOpModeBaseTest {

    @Test
    public void blockedMatchAutoNeverConstructsRuntimeAndStartCannotBypassIt() {
        RecordingTelemetry recordingTelemetry = new RecordingTelemetry();
        BlockedMatchAuto mode = new BlockedMatchAuto();
        mode.telemetry = recordingTelemetry.proxy();

        mode.init();

        assertFalse(mode.initialized());
        assertEquals(0, mode.runtimeFactoryCalls);
        PhoenixReadiness.Result readiness = mode.readiness();
        assertNotNull(readiness);
        assertFalse(readiness.isAllowed());
        assertTrue(hasIssue(readiness, "auto.route_not_match_ready"));
        assertTrue(recordingTelemetry.contains("BLOCKED"));
        assertTrue(recordingTelemetry.contains("auto.expectedPhysicalStartPedro"));
        assertTrue(recordingTelemetry.contains("Place the robot"));

        recordingTelemetry.clear();
        mode.start();

        assertFalse(mode.initialized());
        assertEquals(0, mode.runtimeFactoryCalls);
        assertTrue(recordingTelemetry.contains("BLOCKED"));
        assertFalse(recordingTelemetry.contains("Place the robot"));
    }

    @Test
    public void explicitTestPurposeStaysVisibleWhenConstructionFails() {
        RecordingTelemetry recordingTelemetry = new RecordingTelemetry();
        ControlledFailingTestAuto mode = configuredTestAuto(recordingTelemetry);

        mode.init();

        assertEquals(1, mode.runtimeFactoryCalls);
        assertFalse(mode.initialized());
        assertTrue(mode.readiness().isAllowed());
        assertTrue(mode.error().contains("controlled Pedro construction failure 1"));
        assertTrue(recordingTelemetry.contains("auto.purpose TEST"));
        assertTrue(recordingTelemetry.contains("TEST ONLY"));
        assertTrue(recordingTelemetry.contains("TEST ROUTE"));

        recordingTelemetry.clear();
        mode.init_loop();
        assertTrue(recordingTelemetry.contains("auto.purpose TEST"));
        assertTrue(recordingTelemetry.contains("TEST ONLY"));
    }

    @Test
    public void ordinaryConstructionFailureCanRetryWithAFreshReport() {
        RecordingTelemetry recordingTelemetry = new RecordingTelemetry();
        ControlledFailingTestAuto mode = configuredTestAuto(recordingTelemetry);

        mode.init();
        PhoenixReadiness.Result firstReadiness = mode.readiness();
        assertTrue(mode.error().contains("failure 1"));

        assertFalse(mode.retry());

        assertEquals(2, mode.runtimeFactoryCalls);
        assertTrue(mode.error().contains("failure 2"));
        assertNotSame(firstReadiness, mode.readiness());
        assertFalse(mode.cleanupBlocked());
    }

    @Test
    public void cleanupFailurePermanentlyBlocksLaterInitRetries() throws Exception {
        RecordingTelemetry recordingTelemetry = new RecordingTelemetry();
        ControlledFailingTestAuto mode = configuredTestAuto(recordingTelemetry);
        PhoenixRobot existingOwner = new PhoenixRobot(
                mode.hardwareMap,
                mode.telemetry,
                mode.gamepad1,
                mode.gamepad2,
                PhoenixProfile.current()
        );
        setField(
                PhoenixRobot.class,
                existingOwner,
                "autonomousDrive",
                new DriveCommandSink() {
                    @Override
                    public void drive(DriveSignal signal) {
                        // Not used by this cleanup regression.
                    }

                    @Override
                    public void stop() {
                        throw new IllegalStateException("controlled cleanup failure");
                    }
                }
        );
        setField(PhoenixPedroAutoOpModeBase.class, mode, "robot", existingOwner);

        assertFalse(mode.retry());
        assertTrue(mode.cleanupBlocked());
        assertTrue(mode.error().contains("controlled cleanup failure"));
        assertEquals(0, mode.runtimeFactoryCalls);

        recordingTelemetry.clear();
        assertFalse(mode.retry());
        assertTrue(mode.cleanupBlocked());
        assertTrue(mode.error().contains("controlled cleanup failure"));
        assertEquals(0, mode.runtimeFactoryCalls);
        assertTrue(recordingTelemetry.contains("controlled cleanup failure"));
    }

    @Test
    public void allowedIntegrationTestReappliesStartBeforeClockResetAndRootStart()
            throws Exception {
        RecordingTelemetry recordingTelemetry = new RecordingTelemetry();
        AllowedInjectedTestAuto mode = configuredAllowedInjectedAuto(recordingTelemetry);
        List<String> events = new ArrayList<String>();
        PassiveRuntimeFixture runtimeFixture = new PassiveRuntimeFixture(events);
        PhoenixPedroPathFactory.RouteAvailability availability =
                PhoenixPedroPathFactory.routeAvailabilityFor(mode.autoSpec());
        PhoenixReadiness.Result readiness = PhoenixReadiness.pedroAuto(
                mode.autoSpec(),
                PhoenixProfile.current(),
                PhoenixReadiness.AutoPurpose.PEDRO_INTEGRATION_TEST
        );
        assertTrue(readiness.isAllowed());

        PhoenixRobot robot = robotWithAutoLifecycle(mode);
        LoopClock robotClock = (LoopClock) getField(PhoenixRobot.class, robot, "clock");
        robotClock.reset(4.25);
        runtimeFixture.predictorAccess.observedClock = robotClock;

        // Production initialization assigns the selected start once while building the route.
        runtimeFixture.runtime.setStartingPose(availability.expectedPedroStartPose);
        int initSetPoseCount = runtimeFixture.predictorAccess.setPoseCount;
        events.clear();

        RecordingTask root = new RecordingTask(events, null);
        robot.installAutoRoutine(root);
        setField(PhoenixRobot.class, robot, "autonomousDrive", runtimeFixture.adapter);
        injectInitializedState(
                mode,
                robot,
                runtimeFixture,
                availability,
                readiness
        );

        long adapterCycleBeforeInitLoop = (Long) getField(
                PedroPathingDriveAdapter.class,
                runtimeFixture.adapter,
                "lastUpdateCycle"
        );
        mode.init_loop();

        assertEquals(0, root.starts);
        assertEquals(0, root.updates);
        assertEquals(initSetPoseCount, runtimeFixture.predictorAccess.setPoseCount);
        assertEquals(0, runtimeFixture.drivetrain.runDriveCalls);
        assertEquals(
                adapterCycleBeforeInitLoop,
                ((Long) getField(
                        PedroPathingDriveAdapter.class,
                        runtimeFixture.adapter,
                        "lastUpdateCycle"
                )).longValue()
        );
        assertTrue(events.isEmpty());

        mode.runtimeSec = 28.5;
        mode.start();

        assertTrue(mode.initialized());
        assertEquals(initSetPoseCount + 1, runtimeFixture.predictorAccess.setPoseCount);
        assertEquals(4.25, runtimeFixture.predictorAccess.clockAtLastSetPoseSec, 0.0);
        assertEquals(1, root.starts);
        assertEquals(1, root.updates);
        assertEquals(28.5, root.startedAtSec, 0.0);
        assertEquals(28.5, root.updatedAtSec, 0.0);
        assertEquals(0L, root.startedCycle);
        assertEquals(0L, root.updatedCycle);
        assertEquals("startPose", events.get(0));
        assertEquals("root.start", events.get(1));
        assertEquals("root.update", events.get(2));
        assertEquals(0, runtimeFixture.drivetrain.runDriveCalls);
        assertEquals(
                adapterCycleBeforeInitLoop,
                ((Long) getField(
                        PedroPathingDriveAdapter.class,
                        runtimeFixture.adapter,
                        "lastUpdateCycle"
                )).longValue()
        );

        mode.stop();
    }

    @Test
    public void selectorStartWithoutConfirmationCannotBypassMatchBlocker() {
        RecordingTelemetry recordingTelemetry = new RecordingTelemetry();
        PhoenixPedroAutoSelectorOpMode mode = new PhoenixPedroAutoSelectorOpMode();
        mode.telemetry = recordingTelemetry.proxy();
        mode.gamepad1 = new Gamepad();
        mode.gamepad2 = new Gamepad();
        // A blocker must be decided before any hardware map is required.
        mode.hardwareMap = null;

        mode.init();
        assertNull(mode.readinessOrNull());

        recordingTelemetry.clear();
        mode.start();

        assertFalse(mode.isAutoInitialized());
        assertNotNull(mode.activeSpecOrNull());
        assertNotNull(mode.readinessOrNull());
        assertFalse(mode.readinessOrNull().isAllowed());
        assertTrue(hasIssue(mode.readinessOrNull(), "auto.route_not_match_ready"));
        assertNull(mode.initErrorOrNull());
        assertFalse(mode.cleanupFailureBlocksRetry());
        assertTrue(recordingTelemetry.contains("BLOCKED"));
        assertTrue(recordingTelemetry.contains("auto.expectedPhysicalStartPedro"));
    }

    @Test
    public void startFailureRetainsPrimaryAndSuppressesCleanupWhileBlockingRetries()
            throws Exception {
        RecordingTelemetry recordingTelemetry = new RecordingTelemetry();
        AllowedInjectedTestAuto mode = configuredAllowedInjectedAuto(recordingTelemetry);
        List<String> events = new ArrayList<String>();
        PassiveRuntimeFixture runtimeFixture = new PassiveRuntimeFixture(events);
        PhoenixPedroPathFactory.RouteAvailability availability =
                PhoenixPedroPathFactory.routeAvailabilityFor(mode.autoSpec());
        PhoenixReadiness.Result readiness = PhoenixReadiness.pedroAuto(
                mode.autoSpec(),
                PhoenixProfile.current(),
                PhoenixReadiness.AutoPurpose.PEDRO_INTEGRATION_TEST
        );
        assertTrue(readiness.isAllowed());

        runtimeFixture.runtime.setStartingPose(availability.expectedPedroStartPose);
        PhoenixRobot robot = robotWithAutoLifecycle(mode);
        RuntimeException primary = new IllegalStateException("controlled root start failure");
        RuntimeException cleanup = new IllegalStateException("controlled drive cleanup failure");
        RecordingTask root = new RecordingTask(events, primary);
        robot.installAutoRoutine(root);
        setField(
                PhoenixRobot.class,
                robot,
                "autonomousDrive",
                new DriveCommandSink() {
                    @Override
                    public void drive(DriveSignal signal) {
                        // No hardware in this lifecycle regression.
                    }

                    @Override
                    public void stop() {
                        throw cleanup;
                    }
                }
        );
        injectInitializedState(
                mode,
                robot,
                runtimeFixture,
                availability,
                readiness
        );

        mode.runtimeSec = 19.0;
        mode.start();

        RuntimeException retained = (RuntimeException) getField(
                PhoenixPedroAutoOpModeBase.class,
                mode,
                "initFailure"
        );
        assertSame(primary, retained);
        assertEquals(1, retained.getSuppressed().length);
        assertSame(cleanup, retained.getSuppressed()[0]);
        assertFalse(mode.initialized());
        assertTrue(mode.cleanupBlocked());
        assertEquals(1, root.starts);
        assertEquals(1, root.updates);
        assertEquals(1, root.cancels);
        assertTrue(recordingTelemetry.contains("controlled root start failure"));
        assertTrue(recordingTelemetry.contains("Cleanup also failed"));
        assertTrue(recordingTelemetry.contains("controlled drive cleanup failure"));
        assertTrue(recordingTelemetry.contains("Retry disabled"));

        assertFalse(mode.retry());
        assertEquals(0, mode.runtimeFactoryCalls);
        assertSame(
                primary,
                getField(PhoenixPedroAutoOpModeBase.class, mode, "initFailure")
        );
        assertEquals(1, primary.getSuppressed().length);
        assertSame(cleanup, primary.getSuppressed()[0]);

        mode.stop();
    }

    private static ControlledFailingTestAuto configuredTestAuto(
            RecordingTelemetry recordingTelemetry
    ) {
        ControlledFailingTestAuto mode = new ControlledFailingTestAuto();
        mode.telemetry = recordingTelemetry.proxy();
        mode.hardwareMap = new HardwareMap(null, null);
        mode.gamepad1 = new Gamepad();
        mode.gamepad2 = new Gamepad();
        return mode;
    }

    private static AllowedInjectedTestAuto configuredAllowedInjectedAuto(
            RecordingTelemetry recordingTelemetry
    ) {
        AllowedInjectedTestAuto mode = new AllowedInjectedTestAuto();
        mode.telemetry = recordingTelemetry.proxy();
        mode.hardwareMap = new HardwareMap(null, null);
        mode.gamepad1 = new Gamepad();
        mode.gamepad2 = new Gamepad();
        return mode;
    }

    private static PhoenixRobot robotWithAutoLifecycle(AllowedInjectedTestAuto mode)
            throws Exception {
        PhoenixRobot robot = new PhoenixRobot(
                mode.hardwareMap,
                mode.telemetry,
                mode.gamepad1,
                mode.gamepad2,
                PhoenixProfile.current()
        );
        Class<?> lifecycleClass = Class.forName(
                "edu.ftcphoenix.robots.phoenix.PhoenixRobot$AutoRoutineLifecycle"
        );
        Constructor<?> lifecycleConstructor = lifecycleClass.getDeclaredConstructor();
        lifecycleConstructor.setAccessible(true);
        setField(
                PhoenixRobot.class,
                robot,
                "autoRoutineLifecycle",
                lifecycleConstructor.newInstance()
        );
        return robot;
    }

    private static void injectInitializedState(
            AllowedInjectedTestAuto mode,
            PhoenixRobot robot,
            PassiveRuntimeFixture runtimeFixture,
            PhoenixPedroPathFactory.RouteAvailability availability,
            PhoenixReadiness.Result readiness
    ) throws Exception {
        setField(PhoenixPedroAutoOpModeBase.class, mode, "robot", robot);
        setField(PhoenixPedroAutoOpModeBase.class, mode, "pedroRuntime", runtimeFixture.runtime);
        setField(PhoenixPedroAutoOpModeBase.class, mode, "follower", runtimeFixture.follower);
        setField(PhoenixPedroAutoOpModeBase.class, mode, "driveAdapter", runtimeFixture.adapter);
        setField(PhoenixPedroAutoOpModeBase.class, mode, "activeSpec", mode.autoSpec());
        setField(PhoenixPedroAutoOpModeBase.class, mode, "routeAvailability", availability);
        setField(PhoenixPedroAutoOpModeBase.class, mode, "readiness", readiness);
        setField(
                PhoenixPedroAutoOpModeBase.class,
                mode,
                "expectedPedroStartPose",
                availability.expectedPedroStartPose
        );
        setField(PhoenixPedroAutoOpModeBase.class, mode, "pathLabel", "injected-test-route");
    }

    private static void setField(Class<?> owner,
                                 Object target,
                                 String name,
                                 Object value) throws Exception {
        Field field = owner.getDeclaredField(name);
        field.setAccessible(true);
        field.set(target, value);
    }

    private static Object getField(Class<?> owner,
                                   Object target,
                                   String name) throws Exception {
        Field field = owner.getDeclaredField(name);
        field.setAccessible(true);
        return field.get(target);
    }

    private static boolean hasIssue(PhoenixReadiness.Result readiness, String id) {
        for (PhoenixReadiness.Issue issue : readiness.issues()) {
            if (id.equals(issue.id())) {
                return true;
            }
        }
        return false;
    }

    private static final class BlockedMatchAuto extends PhoenixPedroAutoOpModeBase {
        int runtimeFactoryCalls;

        @Override
        protected PhoenixAutoSpec autoSpec() {
            return PhoenixAutoSpec.audienceSafe(PhoenixAutoSpec.Alliance.RED);
        }

        @Override
        protected PedroPathingRuntime createPedroRuntime(HardwareMap hardwareMap,
                                                         PhoenixProfile profile) {
            runtimeFactoryCalls++;
            throw new AssertionError("A blocked match Auto must not construct Pedro hardware");
        }

        boolean initialized() {
            return isAutoInitialized();
        }

        PhoenixReadiness.Result readiness() {
            return readinessOrNull();
        }
    }

    private static final class ControlledFailingTestAuto extends PhoenixPedroAutoOpModeBase {
        int runtimeFactoryCalls;

        @Override
        protected PhoenixAutoSpec autoSpec() {
            return PhoenixAutoSpec.builder()
                    .alliance(PhoenixAutoSpec.Alliance.RED)
                    .startPosition(PhoenixAutoSpec.StartPosition.AUDIENCE)
                    .partnerPlan(PhoenixAutoSpec.PartnerPlan.NONE)
                    .strategy(PhoenixAutoStrategyId.PEDRO_INTEGRATION_TEST)
                    .build();
        }

        @Override
        protected PhoenixReadiness.AutoPurpose autoPurpose() {
            return PhoenixReadiness.AutoPurpose.PEDRO_INTEGRATION_TEST;
        }

        @Override
        protected PedroPathingRuntime createPedroRuntime(HardwareMap hardwareMap,
                                                         PhoenixProfile profile) {
            runtimeFactoryCalls++;
            throw new IllegalStateException(
                    "controlled Pedro construction failure " + runtimeFactoryCalls
            );
        }

        boolean retry() {
            return initializeRobotForSpec(autoSpec());
        }

        boolean initialized() {
            return isAutoInitialized();
        }

        PhoenixReadiness.Result readiness() {
            return readinessOrNull();
        }

        String error() {
            return initErrorOrNull();
        }

        boolean cleanupBlocked() {
            return cleanupFailureBlocksRetry();
        }
    }

    private static final class AllowedInjectedTestAuto extends PhoenixPedroAutoOpModeBase {
        private final PhoenixAutoSpec spec = PhoenixAutoSpec.builder()
                .alliance(PhoenixAutoSpec.Alliance.RED)
                .startPosition(PhoenixAutoSpec.StartPosition.AUDIENCE)
                .partnerPlan(PhoenixAutoSpec.PartnerPlan.NONE)
                .strategy(PhoenixAutoStrategyId.PEDRO_INTEGRATION_TEST)
                .build();
        int runtimeFactoryCalls;
        double runtimeSec;

        @Override
        protected PhoenixAutoSpec autoSpec() {
            return spec;
        }

        @Override
        protected PhoenixReadiness.AutoPurpose autoPurpose() {
            return PhoenixReadiness.AutoPurpose.PEDRO_INTEGRATION_TEST;
        }

        @Override
        protected PedroPathingRuntime createPedroRuntime(HardwareMap hardwareMap,
                                                         PhoenixProfile profile) {
            runtimeFactoryCalls++;
            throw new AssertionError("Injected initialized state must not rebuild Pedro runtime");
        }

        @Override
        public double getRuntime() {
            return runtimeSec;
        }

        boolean retry() {
            return initializeRobotForSpec(spec);
        }

        boolean initialized() {
            return isAutoInitialized();
        }

        boolean cleanupBlocked() {
            return cleanupFailureBlocksRetry();
        }
    }

    private static final class PassiveRuntimeFixture {
        final ReflectivePredictorAccess predictorAccess;
        final FakeDrivetrain drivetrain = new FakeDrivetrain();
        final Follower follower;
        final PedroPathingDriveAdapter adapter;
        final PedroPathingRuntime runtime;

        PassiveRuntimeFixture(List<String> events) throws Exception {
            Class<?> accessType = Class.forName(
                    "edu.ftcphoenix.fw.integrations.pedro."
                            + "PedroPathingPassiveLocalizer$PredictorAccess"
            );
            predictorAccess = new ReflectivePredictorAccess(events);
            Object accessProxy = Proxy.newProxyInstance(
                    accessType.getClassLoader(),
                    new Class<?>[]{accessType},
                    predictorAccess
            );

            Class<?> localizerClass = Class.forName(
                    "edu.ftcphoenix.fw.integrations.pedro.PedroPathingPassiveLocalizer"
            );
            Constructor<?> localizerConstructor = localizerClass.getDeclaredConstructor(
                    accessType,
                    PedroFieldTransform.class
            );
            localizerConstructor.setAccessible(true);
            Object localizer = localizerConstructor.newInstance(
                    accessProxy,
                    PedroFieldTransform.decodeInvertedFtc()
            );

            PathConstraints constraints = PathConstraints.defaultConstraints.copy();
            follower = new Follower(
                    new FollowerConstants(),
                    (Localizer) localizer,
                    drivetrain,
                    constraints.copy()
            );
            Method completeConstruction = localizerClass.getDeclaredMethod(
                    "completeFollowerConstruction"
            );
            completeConstruction.setAccessible(true);
            completeConstruction.invoke(localizer);

            adapter = new PedroPathingDriveAdapter(follower);
            runtime = runtimeFor(localizer, follower, adapter, constraints);
        }

        private static PedroPathingRuntime runtimeFor(Object localizer,
                                                      Follower follower,
                                                      PedroPathingDriveAdapter adapter,
                                                      PathConstraints constraints)
                throws Exception {
            for (Constructor<?> constructor : PedroPathingRuntime.class.getDeclaredConstructors()) {
                if (constructor.getParameterTypes().length == 5) {
                    constructor.setAccessible(true);
                    return (PedroPathingRuntime) constructor.newInstance(
                            null,
                            localizer,
                            follower,
                            adapter,
                            constraints
                    );
                }
            }
            throw new AssertionError("PedroPathingRuntime five-role constructor was not found");
        }
    }

    private static final class ReflectivePredictorAccess implements InvocationHandler {
        private final List<String> events;
        private final Constructor<?> sampleConstructor;
        private Object sample;
        private LoopClock observedClock;
        private int setPoseCount;
        private double clockAtLastSetPoseSec = Double.NaN;

        ReflectivePredictorAccess(List<String> events) throws Exception {
            this.events = events;
            Class<?> sampleClass = Class.forName(
                    "edu.ftcphoenix.fw.integrations.pedro.PedroPathingPassiveLocalizer$Sample"
            );
            sampleConstructor = sampleClass.getDeclaredConstructor(
                    Pose2d.class,
                    boolean.class,
                    boolean.class,
                    long.class,
                    double.class,
                    double.class,
                    double.class,
                    double.class,
                    double.class
            );
            sampleConstructor.setAccessible(true);
            sample = newSample(Pose2d.zero(), false);
        }

        @Override
        public Object invoke(Object proxy, Method method, Object[] args) throws Exception {
            if ("currentSnapshot".equals(method.getName())) {
                return sample;
            }
            if ("setPose".equals(method.getName())) {
                setPoseCount++;
                clockAtLastSetPoseSec = observedClock == null
                        ? Double.NaN
                        : observedClock.nowSec();
                events.add("startPose");
                sample = newSample((Pose2d) args[0], true);
                return null;
            }
            if ("toString".equals(method.getName())) {
                return "ReflectivePredictorAccess";
            }
            if ("hashCode".equals(method.getName())) {
                return System.identityHashCode(proxy);
            }
            if ("equals".equals(method.getName())) {
                return proxy == args[0];
            }
            throw new AssertionError("Unexpected predictor access method: " + method);
        }

        private Object newSample(Pose2d pose, boolean hasPose) throws Exception {
            return sampleConstructor.newInstance(
                    pose,
                    hasPose,
                    false,
                    Long.MIN_VALUE,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0
            );
        }
    }

    private static final class RecordingTask implements Task {
        private final List<String> events;
        private final RuntimeException updateFailure;
        private int starts;
        private int updates;
        private int cancels;
        private boolean started;
        private boolean complete;
        private double startedAtSec = Double.NaN;
        private double updatedAtSec = Double.NaN;
        private long startedCycle = Long.MIN_VALUE;
        private long updatedCycle = Long.MIN_VALUE;

        RecordingTask(List<String> events, RuntimeException updateFailure) {
            this.events = events;
            this.updateFailure = updateFailure;
        }

        @Override
        public void start(LoopClock clock) {
            starts++;
            started = true;
            startedAtSec = clock.nowSec();
            startedCycle = clock.cycle();
            events.add("root.start");
        }

        @Override
        public void update(LoopClock clock) {
            updates++;
            updatedAtSec = clock.nowSec();
            updatedCycle = clock.cycle();
            events.add("root.update");
            if (updateFailure != null) {
                throw updateFailure;
            }
        }

        @Override
        public void cancel() {
            if (!started || complete) {
                return;
            }
            cancels++;
            complete = true;
        }

        @Override
        public boolean isComplete() {
            return complete;
        }

        @Override
        public TaskOutcome getOutcome() {
            return complete ? TaskOutcome.CANCELLED : TaskOutcome.NOT_DONE;
        }
    }

    private static final class FakeDrivetrain extends Drivetrain {
        int runDriveCalls;

        @Override
        public double[] calculateDrive(Vector correctivePower,
                                       Vector headingPower,
                                       Vector pathingPower,
                                       double robotHeading) {
            return new double[]{0.0, 0.0, 0.0, 0.0};
        }

        @Override
        public void updateConstants() {
            // No mutable constants in the fake boundary.
        }

        @Override
        public void breakFollowing() {
            // No hardware in the fake boundary.
        }

        @Override
        public void runDrive(double[] drivePowers) {
            runDriveCalls++;
        }

        @Override
        public void startTeleopDrive() {
            // No hardware in the fake boundary.
        }

        @Override
        public void startTeleopDrive(boolean brakeMode) {
            // No hardware in the fake boundary.
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
            // No hardware in the fake boundary.
        }

        @Override
        public void setYVelocity(double yMovement) {
            // No hardware in the fake boundary.
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

    private static final class RecordingTelemetry implements InvocationHandler {
        private final List<String> entries = new ArrayList<String>();
        private final Telemetry proxy = (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                this
        );

        Telemetry proxy() {
            return proxy;
        }

        boolean contains(String text) {
            for (String entry : entries) {
                if (entry.contains(text)) {
                    return true;
                }
            }
            return false;
        }

        void clear() {
            entries.clear();
        }

        @Override
        public Object invoke(Object proxy, Method method, Object[] args) {
            if (("addData".equals(method.getName()) || "addLine".equals(method.getName()))
                    && args != null) {
                StringBuilder entry = new StringBuilder();
                for (Object arg : args) {
                    if (arg instanceof Object[]) {
                        for (Object nested : (Object[]) arg) {
                            entry.append(' ').append(nested);
                        }
                    } else {
                        entry.append(' ').append(arg);
                    }
                }
                entries.add(entry.toString());
            }

            Class<?> returnType = method.getReturnType();
            if (returnType == boolean.class) {
                return true;
            }
            if (returnType == int.class) {
                return 0;
            }
            return null;
        }
    }
}
