package edu.ftcphoenix.robots.phoenix;

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
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.drive.DriveSignal;
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
import edu.ftcphoenix.robots.phoenix.scoring.PhoenixScoring;
import edu.ftcphoenix.robots.phoenix.scoring.PhoenixTargeting;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/** Verifies Phoenix's complete declaration-only TeleOp at the managed FTC boundaries. */
public final class PhoenixManagedTeleOpLifecycleTest {

    @Test
    public void exactStartRealizesScoringBeforeZeroDriveAndCleanupIsSingleOwner() {
        List<String> events = new ArrayList<String>();
        PhoenixProfile profile = PhoenixProfile.current();
        TestHardwareMap hardwareMap = TestHardwareMap.forScoring(profile.scoring, events);
        RecordingTelemetry telemetry = new RecordingTelemetry();
        RecordingVisionLane vision = new RecordingVisionLane(events);
        RecordingMotionPredictor predictor = new RecordingMotionPredictor(events);
        RecordingDriveSink drive = new RecordingDriveSink(events);
        RecordingAssembly assembly = new RecordingAssembly(vision, predictor, drive);
        Gamepad driver = new Gamepad();
        Gamepad operator = new Gamepad();

        TestHost host = new TestHost(profile, assembly);
        host.hardwareMap = hardwareMap;
        host.telemetry = telemetry.proxy();
        host.gamepad1 = driver;
        host.gamepad2 = operator;

        host.runtimeSec = 3.0;
        host.init();

        assertEquals(0, vision.readinessCalls);
        assertEquals(0, predictor.updateCalls);
        assertEquals(0, hardwareMap.commandWrites);
        assertEquals(0, drive.updateCalls);
        assertEquals(0, drive.driveCalls);
        int commitsAfterInit = telemetry.updateCalls;

        // GamepadDevice calibrates the construction-time position as neutral. Moving after INIT
        // models a driver holding forward at START without redefining that position as center.
        driver.left_stick_y = -1.0f;

        events.clear();
        host.runtimeSec = 20.0;
        host.start();

        assertEquals(1, vision.readinessCalls);
        assertEquals(1, predictor.updateCalls);
        assertEquals(5, hardwareMap.commandWrites);
        assertEquals(1, drive.updateCalls);
        assertEquals(1, drive.driveCalls);
        assertEquals(0.0, drive.lastSignal.axial, 0.0);
        assertEquals(0.0, drive.lastSignal.lateral, 0.0);
        assertEquals(0.0, drive.lastSignal.omega, 0.0);
        assertEquals(commitsAfterInit, telemetry.updateCalls);

        int readinessIndex = events.indexOf("vision.readiness");
        int localizationIndex = events.indexOf("localization.update");
        int firstScoringIndex = firstIndexWithPrefix(events, "scoring.");
        int lastScoringIndex = lastIndexWithPrefix(events, "scoring.");
        int driveUpdateIndex = events.indexOf("drive.update");
        int driveWriteIndex = events.indexOf("drive.write");
        assertTrue(readinessIndex >= 0);
        assertTrue(readinessIndex < localizationIndex);
        assertTrue(localizationIndex < firstScoringIndex);
        assertTrue(lastScoringIndex < driveUpdateIndex);
        assertTrue(driveUpdateIndex < driveWriteIndex);

        events.clear();
        host.runtimeSec = 20.02;
        host.loop();

        assertEquals(2, vision.readinessCalls);
        assertEquals(2, predictor.updateCalls);
        assertEquals(2, drive.updateCalls);
        assertEquals(2, drive.driveCalls);
        assertEquals(0.0, drive.lastSignal.axial, 0.0);
        assertEquals(commitsAfterInit + 1, telemetry.updateCalls);

        events.clear();
        host.runtimeSec = 20.04;
        host.loop();
        assertEquals(3, predictor.updateCalls);
        assertEquals(3, drive.driveCalls);
        assertTrue(Math.abs(drive.lastSignal.axial) > 0.0);
        assertEquals(commitsAfterInit + 2, telemetry.updateCalls);

        // Startup release is construction-only. A later localization loss disables assists in
        // their own evidence paths but must not confiscate ordinary robot-centric manual drive.
        events.clear();
        host.runtimeSec = 20.06;
        host.loop();
        assertEquals(4, predictor.updateCalls);
        assertEquals(4, drive.driveCalls);
        assertTrue(Math.abs(drive.lastSignal.axial) > 0.0);

        events.clear();
        host.stop();

        int lastScoringStopIndex = lastIndexWithPrefix(events, "scoring.");
        int driveStopIndex = events.indexOf("drive.stop");
        int visionCloseIndex = events.indexOf("vision.close");
        assertTrue(lastScoringStopIndex >= 0);
        assertTrue(lastScoringStopIndex < driveStopIndex);
        assertTrue(driveStopIndex < visionCloseIndex);
        assertEquals(1, drive.stopCalls);
        assertEquals(1, vision.closeCalls);

        int commandWritesAfterStop = hardwareMap.commandWrites;
        int eventCountAfterStop = events.size();
        host.stop();

        assertEquals(commandWritesAfterStop, hardwareMap.commandWrites);
        assertEquals(eventCountAfterStop, events.size());
        assertEquals(1, drive.stopCalls);
        assertEquals(1, vision.closeCalls);
    }

    private static int firstIndexWithPrefix(List<String> events, String prefix) {
        for (int index = 0; index < events.size(); index++) {
            if (events.get(index).startsWith(prefix)) {
                return index;
            }
        }
        return -1;
    }

    private static int lastIndexWithPrefix(List<String> events, String prefix) {
        for (int index = events.size() - 1; index >= 0; index--) {
            if (events.get(index).startsWith(prefix)) {
                return index;
            }
        }
        return -1;
    }

    private static final class TestHost extends FtcRobotOpMode {
        private final PhoenixProfile profile;
        private final PhoenixRobot.TeleOpHardwareAssembly assembly;
        private double runtimeSec;

        private TestHost(
                PhoenixProfile profile,
                PhoenixRobot.TeleOpHardwareAssembly assembly
        ) {
            this.profile = profile;
            this.assembly = assembly;
        }

        @Override
        protected void configure(RobotProgram program) {
            PhoenixRobot robot = new PhoenixRobot(
                    hardwareMap,
                    telemetry,
                    gamepad1,
                    gamepad2,
                    profile,
                    assembly
            );
            robot.declareTeleOp(
                    program,
                    Source.constant(Collections.singleton(
                            profile.autoAim.scoringTagIdFor(PhoenixAlliance.RED)
                    ))
            );
            program.presenter(robot.teleOpPresenter(PhoenixMatchHandoff.RestoreResult.MISSING));
        }

        @Override
        public double getRuntime() {
            return runtimeSec;
        }
    }

    private static final class RecordingAssembly
            implements PhoenixRobot.TeleOpHardwareAssembly {
        private final RecordingVisionLane vision;
        private final RecordingMotionPredictor predictor;
        private final RecordingDriveSink drive;

        private RecordingAssembly(
                RecordingVisionLane vision,
                RecordingMotionPredictor predictor,
                RecordingDriveSink drive
        ) {
            this.vision = vision;
            this.predictor = predictor;
            this.drive = drive;
        }

        @Override
        public AprilTagVisionLane createVision(
                HardwareMap hardwareMap,
                PhoenixProfile profile
        ) {
            return vision;
        }

        @Override
        public FtcOdometryAprilTagLocalizationLane createLocalization(
                HardwareMap hardwareMap,
                AprilTagVisionLane createdVision,
                PhoenixProfile profile
        ) {
            return FtcOdometryAprilTagLocalizationLane.withPredictor(
                    predictor,
                    createdVision,
                    profile.field.fixedAprilTagLayout,
                    profile.localization.estimation
            );
        }

        @Override
        public PhoenixScoring createScoring(
                HardwareMap hardwareMap,
                PhoenixProfile profile,
                PhoenixTargeting targeting
        ) {
            return new PhoenixScoring(hardwareMap, profile.scoring, targeting);
        }

        @Override
        public DriveCommandSink createDrive(
                HardwareMap hardwareMap,
                PhoenixProfile profile
        ) {
            return drive;
        }
    }

    private static final class RecordingDriveSink implements DriveCommandSink {
        private final List<String> events;
        private int updateCalls;
        private int driveCalls;
        private int stopCalls;
        private DriveSignal lastSignal;

        private RecordingDriveSink(List<String> events) {
            this.events = events;
        }

        @Override
        public void update(LoopClock clock) {
            updateCalls++;
            events.add("drive.update");
        }

        @Override
        public void drive(DriveSignal signal) {
            driveCalls++;
            lastSignal = signal;
            events.add("drive.write");
        }

        @Override
        public void stop() {
            stopCalls++;
            events.add("drive.stop");
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
            Pose3d pose = updateCalls == 1
                    ? new Pose3d(Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0)
                    : Pose3d.zero();
            estimate = new PoseEstimate(
                    pose,
                    updateCalls <= 3,
                    updateCalls <= 3 ? 1.0 : 0.0,
                    updateCalls == 2 ? LoopTimestamp.unavailable() : timestamp
            );
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
        };
        private int readinessCalls;
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

    private static final class TestHardwareMap extends HardwareMap {
        private final Map<String, HardwareDevice> devices =
                new HashMap<String, HardwareDevice>();
        private final List<String> events;
        private int commandWrites;

        private TestHardwareMap(List<String> events) {
            super(null, null);
            this.events = events;
        }

        private static TestHardwareMap forScoring(
                PhoenixProfile.ScoringConfig config,
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
