package edu.ftcphoenix.robots.phoenix;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;

import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.ftc.FtcDrives;
import edu.ftcphoenix.fw.ftc.FtcRobotOpMode;
import edu.ftcphoenix.fw.ftc.RobotProgram;
import edu.ftcphoenix.fw.ftc.localization.FtcOdometryAprilTagLocalizationLane;
import edu.ftcphoenix.fw.ftc.vision.AprilTagVisionLane;
import edu.ftcphoenix.fw.ftc.vision.VisionReadiness;
import edu.ftcphoenix.fw.localization.MotionDelta;
import edu.ftcphoenix.fw.localization.MotionPredictor;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.robots.phoenix.scoring.PhoenixScoring;
import edu.ftcphoenix.robots.phoenix.scoring.PhoenixTargeting;
import edu.ftcphoenix.fw.task.TaskOutcome;
import edu.ftcphoenix.robots.phoenix.opmode.PhoenixAutoOpMode;
import edu.ftcphoenix.robots.phoenix.opmode.PhoenixBlueAudienceSafeAuto;
import edu.ftcphoenix.robots.phoenix.opmode.PhoenixPedroAutoSelectorOpMode;
import edu.ftcphoenix.robots.phoenix.opmode.PhoenixPedroAutoTestOpMode;
import edu.ftcphoenix.robots.phoenix.opmode.PhoenixRedAudienceSafeAuto;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

/** Verifies Phoenix Auto's managed phase order, cleanup ownership, and thin entries. */
public final class PhoenixManagedAutoLifecycleTest {

    @Test
    public void initIsInertAndStartAndStopUseTheOneManagedAutoOrder() {
        List<String> events = new ArrayList<>();
        PhoenixProfile profile = PhoenixProfile.current();
        TestHardwareMap hardwareMap = TestHardwareMap.forScoring(profile.scoring, events);
        RecordingTelemetry telemetry = new RecordingTelemetry();
        RecordingVisionLane vision = new RecordingVisionLane(events);
        RecordingMotionPredictor predictor = new RecordingMotionPredictor(events);
        RecordingPedroDrive drive = new RecordingPedroDrive(events);
        RecordingEligibleTags eligibleTags = new RecordingEligibleTags(
                profile.targeting.redAllianceScoringTagId,
                events
        );
        RecordingBooleanSource autoAimEnabled = new RecordingBooleanSource(false);
        RecordingBooleanSource aimOverride = new RecordingBooleanSource(true);
        RecordingAutoAssembly autoAssembly = new RecordingAutoAssembly(
                vision,
                predictor
        );
        RecordingPrestart prestart = new RecordingPrestart(events);
        RecordingTask root = new RecordingTask(events);

        TestHost host = new TestHost(
                profile,
                autoAssembly,
                drive,
                predictor,
                eligibleTags,
                autoAimEnabled,
                aimOverride,
                prestart,
                root,
                events
        );
        host.hardwareMap = hardwareMap;
        host.telemetry = telemetry.proxy();
        host.gamepad1 = new Gamepad();
        host.gamepad2 = new Gamepad();

        host.runtimeSec = 3.0;
        host.init();

        assertEquals(1, prestart.updateCalls);
        assertEquals(0, prestart.freezeCalls);
        assertEquals(0, host.poseApplications);
        assertEquals(0, vision.readinessCalls);
        assertEquals(0, predictor.updateCalls);
        assertEquals(0, eligibleTags.sampleCalls);
        assertEquals(0, autoAimEnabled.sampleCalls);
        assertEquals(0, aimOverride.sampleCalls);
        assertEquals(0, drive.updateCalls);
        assertEquals(0, root.startCalls);
        assertEquals(0, root.updateCalls);
        assertEquals(0, hardwareMap.commandWrites);
        int commitsAfterInit = telemetry.updateCalls;

        events.clear();
        host.runtimeSec = 20.0;
        host.start();

        assertEquals(
                Arrays.asList(
                        "prestart.freeze",
                        "pedro.pose",
                        "vision.readiness",
                        "localization.update",
                        "targeting.update",
                        "pedro.heartbeat",
                        "root.start",
                        "root.update"
                ),
                events.subList(0, 8)
        );
        assertEquals(13, events.size());
        assertAllHavePrefix(events.subList(8, events.size()), "scoring.");
        assertEquals(1, prestart.freezeCalls);
        assertEquals(1, host.poseApplications);
        assertEquals(1, vision.readinessCalls);
        assertEquals(1, predictor.updateCalls);
        assertEquals(1, eligibleTags.sampleCalls);
        assertEquals(1, autoAimEnabled.sampleCalls);
        assertEquals(1, aimOverride.sampleCalls);
        PhoenixCapabilities.TargetingStatus targetingStatus =
                host.capabilities.targeting().status();
        assertEquals(false, targetingStatus.autoAimEnabled);
        assertEquals(true, targetingStatus.aimOverride);
        assertEquals(1, drive.updateCalls);
        assertEquals(1, root.startCalls);
        assertEquals(1, root.updateCalls);
        assertEquals(5, hardwareMap.commandWrites);
        assertEquals(commitsAfterInit, telemetry.updateCalls);

        events.clear();
        host.stop();

        assertEquals("root.cancel", events.get(0));
        int pedroZeroIndex = events.indexOf("pedro.zero");
        assertTrue(pedroZeroIndex > 1);
        assertAllHavePrefix(events.subList(1, pedroZeroIndex), "scoring.");
        assertAllEqual(
                events.subList(pedroZeroIndex + 1, events.size() - 1),
                "targeting.reset"
        );
        assertEquals("vision.close", events.get(events.size() - 1));
        assertEquals(1, root.cancelCalls);
        assertEquals(1, drive.stopCalls);
        assertEquals(1, vision.targetingResetCalls);
        assertEquals(1, vision.closeCalls);

        int commandWritesAfterStop = hardwareMap.commandWrites;
        int eventCountAfterStop = events.size();
        host.stop();

        assertEquals(commandWritesAfterStop, hardwareMap.commandWrites);
        assertEquals(eventCountAfterStop, events.size());
        assertEquals(1, drive.stopCalls);
        assertEquals(1, vision.targetingResetCalls);
        assertEquals(1, vision.closeCalls);
    }

    @Test
    public void everyPhoenixGroupAutoEntryUsesTheManagedBaseWithoutRawFtcCallbacks()
            throws Exception {
        List<Class<?>> entries = Arrays.asList(
                PhoenixBlueAudienceSafeAuto.class,
                PhoenixRedAudienceSafeAuto.class,
                PhoenixPedroAutoSelectorOpMode.class,
                PhoenixPedroAutoTestOpMode.class
        );

        assertSame(FtcRobotOpMode.class, PhoenixAutoOpMode.class.getSuperclass());
        assertTrue(Modifier.isAbstract(PhoenixAutoOpMode.class.getModifiers()));
        assertNoRawCallbacks(PhoenixAutoOpMode.class);

        for (Class<?> entry : entries) {
            Autonomous annotation = entry.getAnnotation(Autonomous.class);
            assertNotNull(annotation);
            assertEquals("Phoenix", annotation.group());
            assertSame(PhoenixAutoOpMode.class, entry.getSuperclass());
            assertNoRawCallbacks(entry);
            for (String callback : Arrays.asList("init", "init_loop", "start", "loop", "stop")) {
                Method inherited = entry.getMethod(callback);
                assertSame(FtcRobotOpMode.class, inherited.getDeclaringClass());
                assertTrue(Modifier.isFinal(inherited.getModifiers()));
            }
        }
    }

    private static void assertNoRawCallbacks(Class<?> type) {
        for (Method method : type.getDeclaredMethods()) {
            assertTrue(
                    type.getSimpleName() + " declares raw FTC callback " + method.getName(),
                    !Arrays.asList("init", "init_loop", "start", "loop", "stop")
                            .contains(method.getName())
            );
        }
    }

    private static void assertAllHavePrefix(List<String> events, String prefix) {
        for (String event : events) {
            assertTrue("Expected prefix " + prefix + " in " + event, event.startsWith(prefix));
        }
    }

    private static void assertAllEqual(List<String> events, String expected) {
        assertTrue("Expected at least one " + expected + " event", !events.isEmpty());
        for (String event : events) {
            assertEquals(expected, event);
        }
    }

    private static final class TestHost extends FtcRobotOpMode {
        private final PhoenixProfile profile;
        private final PhoenixRobot.AutoHardwareAssembly autoAssembly;
        private final RecordingPedroDrive drive;
        private final MotionPredictor predictor;
        private final Source<Set<Integer>> eligibleTags;
        private final BooleanSource autoAimEnabled;
        private final BooleanSource aimOverride;
        private final RecordingPrestart prestart;
        private final RecordingTask root;
        private final List<String> events;
        private double runtimeSec;
        private int poseApplications;
        private PhoenixRobot robot;
        private PhoenixCapabilities capabilities;

        private TestHost(
                PhoenixProfile profile,
                PhoenixRobot.AutoHardwareAssembly autoAssembly,
                RecordingPedroDrive drive,
                MotionPredictor predictor,
                Source<Set<Integer>> eligibleTags,
                BooleanSource autoAimEnabled,
                BooleanSource aimOverride,
                RecordingPrestart prestart,
                RecordingTask root,
                List<String> events
        ) {
            this.profile = profile;
            this.autoAssembly = autoAssembly;
            this.drive = drive;
            this.predictor = predictor;
            this.eligibleTags = eligibleTags;
            this.autoAimEnabled = autoAimEnabled;
            this.aimOverride = aimOverride;
            this.prestart = prestart;
            this.root = root;
            this.events = events;
        }

        @Override
        protected void configure(RobotProgram program) {
            program.prestart(prestart);
            robot = new PhoenixRobot(
                    hardwareMap,
                    UNUSED_TELEOP_ASSEMBLY,
                    autoAssembly
            );
            capabilities = robot.declareAuto(
                    program,
                    profile,
                    drive,
                    predictor,
                    eligibleTags,
                    autoAimEnabled,
                    aimOverride,
                    () -> {
                        poseApplications++;
                        events.add("pedro.pose");
                    }
            );
            program.rootTask(root);
        }

        @Override
        public double getRuntime() {
            return runtimeSec;
        }
    }

    private static final PhoenixRobot.TeleOpHardwareAssembly UNUSED_TELEOP_ASSEMBLY =
            new PhoenixRobot.TeleOpHardwareAssembly() {
                @Override
                public AprilTagVisionLane createVision(
                        HardwareMap hardwareMap,
                        PhoenixVisionFactory.Config visionConfig
                ) {
                    throw new AssertionError("TeleOp hardware must not be created for Auto");
                }

                @Override
                public FtcOdometryAprilTagLocalizationLane createLocalization(
                        HardwareMap hardwareMap,
                        AprilTagVisionLane vision,
                        TagLayout fixedAprilTagLayout,
                        FtcOdometryAprilTagLocalizationLane.Config localizationConfig
                ) {
                    throw new AssertionError("TeleOp localization must not be created for Auto");
                }

                @Override
                public PhoenixScoring createScoring(
                        HardwareMap hardwareMap,
                        PhoenixScoring.Config scoringConfig,
                        PhoenixTargeting targeting
                ) {
                    throw new AssertionError("TeleOp scoring must not be created for Auto");
                }

                @Override
                public DriveCommandSink createDrive(
                        HardwareMap hardwareMap,
                        FtcDrives.MecanumConfig driveConfig
                ) {
                    throw new AssertionError("TeleOp drive must not be created for Auto");
                }
            };

    private static final class RecordingAutoAssembly implements PhoenixRobot.AutoHardwareAssembly {
        private final RecordingVisionLane vision;
        private final RecordingMotionPredictor expectedPredictor;

        private RecordingAutoAssembly(
                RecordingVisionLane vision,
                RecordingMotionPredictor expectedPredictor
        ) {
            this.vision = vision;
            this.expectedPredictor = expectedPredictor;
        }

        @Override
        public AprilTagVisionLane createVision(
                HardwareMap hardwareMap,
                PhoenixVisionFactory.Config visionConfig
        ) {
            return vision;
        }

        @Override
        public FtcOdometryAprilTagLocalizationLane createLocalization(
                MotionPredictor motionPredictor,
                AprilTagVisionLane createdVision,
                TagLayout fixedAprilTagLayout,
                FtcOdometryAprilTagLocalizationLane.EstimatorConfig estimationConfig
        ) {
            assertSame(expectedPredictor, motionPredictor);
            assertSame(vision, createdVision);
            return FtcOdometryAprilTagLocalizationLane.withPredictor(
                    motionPredictor,
                    createdVision,
                    fixedAprilTagLayout,
                    estimationConfig
            );
        }

        @Override
        public PhoenixScoring createScoring(
                HardwareMap hardwareMap,
                PhoenixScoring.Config scoringConfig,
                PhoenixTargeting targeting
        ) {
            return new PhoenixScoring(hardwareMap, scoringConfig, targeting);
        }
    }

    private static final class RecordingPrestart implements RobotProgram.Prestart {
        private final List<String> events;
        private int updateCalls;
        private int freezeCalls;

        private RecordingPrestart(List<String> events) {
            this.events = events;
        }

        @Override
        public void update(LoopClock clock) {
            updateCalls++;
            events.add("prestart.update");
        }

        @Override
        public RobotProgram.StartDisposition freezeForStart() {
            freezeCalls++;
            events.add("prestart.freeze");
            return RobotProgram.StartDisposition.READY;
        }
    }

    private static final class RecordingPedroDrive implements DriveCommandSink {
        private final List<String> events;
        private int updateCalls;
        private int stopCalls;

        private RecordingPedroDrive(List<String> events) {
            this.events = events;
        }

        @Override
        public void update(LoopClock clock) {
            updateCalls++;
            events.add("pedro.heartbeat");
        }

        @Override
        public void drive(DriveSignal signal) {
            throw new AssertionError("Managed Auto service must not command Pedro as direct drive");
        }

        @Override
        public void stop() {
            stopCalls++;
            events.add("pedro.zero");
        }
    }

    private static final class RecordingEligibleTags implements Source<Set<Integer>> {
        private final Set<Integer> tagIds;
        private final List<String> events;
        private int sampleCalls;

        private RecordingEligibleTags(int tagId, List<String> events) {
            tagIds = Collections.singleton(tagId);
            this.events = events;
        }

        @Override
        public Set<Integer> get(LoopClock clock) {
            sampleCalls++;
            events.add("targeting.update");
            return tagIds;
        }
    }

    private static final class RecordingBooleanSource implements BooleanSource {
        private final boolean value;
        private int sampleCalls;

        private RecordingBooleanSource(boolean value) {
            this.value = value;
        }

        @Override
        public boolean getAsBoolean(LoopClock clock) {
            sampleCalls++;
            return value;
        }
    }

    private static final class RecordingMotionPredictor implements MotionPredictor {
        private final List<String> events;
        private PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
        private MotionDelta delta = MotionDelta.none(LoopTimestamp.unavailable());
        private int updateCalls;

        private RecordingMotionPredictor(List<String> events) {
            this.events = events;
        }

        @Override
        public void update(LoopClock clock) {
            updateCalls++;
            events.add("localization.update");
            LoopTimestamp timestamp = clock.nowTimestamp();
            estimate = new PoseEstimate(Pose3d.zero(), false, 0.0, timestamp);
            delta = MotionDelta.none(timestamp);
        }

        @Override
        public PoseEstimate getEstimate() {
            return estimate;
        }

        @Override
        public MotionDelta getLatestMotionDelta() {
            return delta;
        }
    }

    private static final class RecordingVisionLane implements AprilTagVisionLane {
        private final List<String> events;
        private final AprilTagSensor sensor = new AprilTagSensor() {
            @Override
            public AprilTagDetections get(LoopClock clock) {
                return AprilTagDetections.none();
            }

            @Override
            public void reset() {
                targetingResetCalls++;
                events.add("targeting.reset");
            }
        };
        private int readinessCalls;
        private int targetingResetCalls;
        private int closeCalls;

        private RecordingVisionLane(List<String> events) {
            this.events = events;
        }

        @Override
        public AprilTagSensor tagSensor() {
            return sensor;
        }

        @Override
        public CameraMountConfig cameraMountConfig() {
            return CameraMountConfig.identity();
        }

        @Override
        public VisionReadiness readiness(LoopClock clock) {
            readinessCalls++;
            events.add("vision.readiness");
            return VisionReadiness.ready();
        }

        @Override
        public void close() {
            closeCalls++;
            events.add("vision.close");
        }
    }

    private static final class RecordingTask implements Task {
        private final List<String> events;
        private boolean started;
        private boolean cancelled;
        private int startCalls;
        private int updateCalls;
        private int cancelCalls;

        private RecordingTask(List<String> events) {
            this.events = events;
        }

        @Override
        public void start(LoopClock clock) {
            started = true;
            startCalls++;
            events.add("root.start");
        }

        @Override
        public void update(LoopClock clock) {
            updateCalls++;
            events.add("root.update");
        }

        @Override
        public void cancel() {
            if (!started || cancelled) {
                return;
            }
            cancelled = true;
            cancelCalls++;
            events.add("root.cancel");
        }

        @Override
        public boolean isComplete() {
            return cancelled;
        }

        @Override
        public TaskOutcome getOutcome() {
            return cancelled ? TaskOutcome.CANCELLED : TaskOutcome.NOT_DONE;
        }
    }

    private static final class TestHardwareMap extends HardwareMap {
        private final Map<String, HardwareDevice> devices = new HashMap<>();
        private final List<String> events;
        private int commandWrites;

        private TestHardwareMap(List<String> events) {
            super(null, null);
            this.events = events;
        }

        private static TestHardwareMap forScoring(
                PhoenixScoring.Config config,
                List<String> events
        ) {
            TestHardwareMap map = new TestHardwareMap(events);
            map.devices.put(
                    config.nameMotorIntake,
                    map.deviceProxy(DcMotorEx.class, "intake")
            );
            map.devices.put(
                    config.nameMotorShooterWheel,
                    map.deviceProxy(DcMotorEx.class, "shooter")
            );
            map.devices.put(
                    config.nameCrServoIntakeTransfer,
                    map.deviceProxy(CRServo.class, "intakeTransfer")
            );
            map.devices.put(
                    config.nameCrServoShooterTransferLeft,
                    map.deviceProxy(CRServo.class, "shooterTransferLeft")
            );
            map.devices.put(
                    config.nameCrServoShooterTransferRight,
                    map.deviceProxy(CRServo.class, "shooterTransferRight")
            );
            return map;
        }

        @Override
        public <T> T get(Class<? extends T> type, String name) {
            HardwareDevice device = devices.get(name);
            if (device == null || !type.isInstance(device)) {
                throw new IllegalArgumentException(
                        "No test " + type.getSimpleName() + " named " + name
                );
            }
            return type.cast(device);
        }

        private <T extends HardwareDevice> T deviceProxy(Class<T> type, String label) {
            InvocationHandler handler = new InvocationHandler() {
                private DcMotor.RunMode motorMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
                private DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
                private double power;

                @Override
                public Object invoke(Object proxy, Method method, Object[] args) {
                    String name = method.getName();
                    if (method.getDeclaringClass() == Object.class) {
                        if ("equals".equals(name)) {
                            return proxy == args[0];
                        }
                        if ("hashCode".equals(name)) {
                            return System.identityHashCode(proxy);
                        }
                        if ("toString".equals(name)) {
                            return "TestDevice(" + label + ")";
                        }
                    }
                    if ("getManufacturer".equals(name)) {
                        return HardwareDevice.Manufacturer.Other;
                    }
                    if ("getDeviceName".equals(name)) {
                        return label;
                    }
                    if ("getConnectionInfo".equals(name)) {
                        return "test";
                    }
                    if ("getVersion".equals(name)) {
                        return 1;
                    }
                    if ("setMode".equals(name)) {
                        motorMode = (DcMotor.RunMode) args[0];
                        return null;
                    }
                    if ("getMode".equals(name)) {
                        return motorMode;
                    }
                    if ("setDirection".equals(name)) {
                        direction = (DcMotorSimple.Direction) args[0];
                        return null;
                    }
                    if ("getDirection".equals(name)) {
                        return direction;
                    }
                    if ("setPower".equals(name)) {
                        power = (Double) args[0];
                        recordCommand(label, "power");
                        return null;
                    }
                    if ("getPower".equals(name)) {
                        return power;
                    }
                    if ("setVelocity".equals(name)) {
                        recordCommand(label, "velocity");
                        return null;
                    }
                    if ("getVelocity".equals(name)) {
                        return 0.0;
                    }
                    return defaultValue(method.getReturnType());
                }
            };
            return type.cast(Proxy.newProxyInstance(
                    type.getClassLoader(),
                    new Class<?>[]{type},
                    handler
            ));
        }

        private void recordCommand(String label, String command) {
            commandWrites++;
            events.add("scoring." + label + "." + command);
        }
    }

    private static final class RecordingTelemetry implements InvocationHandler {
        private final Telemetry proxy = (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                this
        );
        private int updateCalls;

        private Telemetry proxy() {
            return proxy;
        }

        @Override
        public Object invoke(Object proxy, Method method, Object[] args) {
            if ("update".equals(method.getName())) {
                updateCalls++;
            }
            return defaultValue(method.getReturnType());
        }
    }

    private static Object defaultValue(Class<?> type) {
        if (!type.isPrimitive()) {
            return null;
        }
        if (type == boolean.class) {
            return true;
        }
        if (type == byte.class) {
            return (byte) 0;
        }
        if (type == short.class) {
            return (short) 0;
        }
        if (type == int.class) {
            return 0;
        }
        if (type == long.class) {
            return 0L;
        }
        if (type == float.class) {
            return 0.0f;
        }
        if (type == double.class) {
            return 0.0;
        }
        if (type == char.class) {
            return '\0';
        }
        return null;
    }
}
