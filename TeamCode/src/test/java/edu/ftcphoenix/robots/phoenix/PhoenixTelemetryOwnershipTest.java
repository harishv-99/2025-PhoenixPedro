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

import java.lang.reflect.Field;
import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.MecanumDrivebase;
import edu.ftcphoenix.fw.ftc.FtcTelemetryDebugSink;
import edu.ftcphoenix.fw.ftc.localization.FtcOdometryAprilTagLocalizationLane;
import edu.ftcphoenix.fw.ftc.vision.AprilTagVisionLane;
import edu.ftcphoenix.fw.ftc.vision.VisionReadiness;
import edu.ftcphoenix.fw.input.Gamepads;
import edu.ftcphoenix.fw.localization.MotionDelta;
import edu.ftcphoenix.fw.localization.MotionPredictor;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the Phoenix present/render/commit ownership boundary with host-side FTC fakes. */
public final class PhoenixTelemetryOwnershipTest {

    @Test
    public void presenterStagesTeleOpAndAutoRowsWithoutClearingOrCommitting() {
        RecordingTelemetry telemetry = new RecordingTelemetry();
        PhoenixTelemetryPresenter presenter = new PhoenixTelemetryPresenter(
                telemetry.proxy(),
                PhoenixProfile.current()
        );
        PoseEstimate noPose = PoseEstimate.noPose(LoopTimestamp.unavailable());

        telemetry.proxy().addData("upstream.sidecar", "retained");
        presenter.emitTeleOp(null, null, null, null, null, noPose, noPose);
        presenter.emitAuto(null, null, null, null, noPose, noPose);

        assertEquals(0, telemetry.updateAttempts());
        assertEquals(0, telemetry.clearCalls());
        assertTrue(telemetry.pendingContains("upstream.sidecar"));
        assertTrue(telemetry.pendingContains("pose.global"));
        assertTrue(telemetry.pendingContains("auto.routine"));
        assertTrue(telemetry.pendingContains("auto.routineOutcome"));
    }

    @Test
    public void autoRootCommitsEachCompleteFrameOnceAndPreservesUpstreamRows()
            throws Exception {
        RecordingTelemetry telemetry = new RecordingTelemetry();
        AutoFixture fixture = AutoFixture.create(telemetry);
        try {
            telemetry.proxy().addData("upstream.sidecar", "first");
            new FtcTelemetryDebugSink(telemetry.proxy())
                    .addData("selected.debug", "visible");
            fixture.robot.updateAny(0.02);
            fixture.robot.updateAuto();

            telemetry.proxy().addData("upstream.sidecar", "second");
            fixture.robot.updateAny(0.04);
            fixture.robot.updateAuto();

            assertEquals(2, telemetry.updateAttempts());
            assertEquals(0, telemetry.clearCalls());
            assertEquals(2, telemetry.committedFrames().size());
            assertTrue(frameContains(telemetry.committedFrames().get(0), "upstream.sidecar first"));
            assertTrue(frameContains(telemetry.committedFrames().get(0), "selected.debug visible"));
            assertTrue(frameContains(telemetry.committedFrames().get(0), "auto.routine"));
            assertFalse(frameContains(telemetry.committedFrames().get(0), "loopProfile"));
            assertTrue(frameContains(telemetry.committedFrames().get(1), "upstream.sidecar second"));
            assertTrue(frameContains(telemetry.committedFrames().get(1), "auto.routineOutcome"));
        } finally {
            fixture.robot.stop();
        }
    }

    @Test
    public void teleOpRootCommitsOneCompleteFrameAndSkipsCommitAfterRenderFailure()
            throws Exception {
        RecordingTelemetry telemetry = new RecordingTelemetry();
        AutoFixture fixture = AutoFixture.create(telemetry);
        configureTeleOp(fixture.robot);
        RuntimeException renderFailure = new IllegalStateException("controlled TeleOp render failure");
        try {
            telemetry.proxy().addData("upstream.sidecar", "teleop");
            fixture.robot.updateAny(0.02);
            fixture.robot.updateTeleOp();

            assertEquals(1, telemetry.updateAttempts());
            assertEquals(1, telemetry.committedFrames().size());
            assertTrue(frameContains(telemetry.committedFrames().get(0), "upstream.sidecar teleop"));
            assertTrue(frameContains(telemetry.committedFrames().get(0), "shoot.mode"));

            telemetry.proxy().addData("upstream.sidecar", "failed-render");
            telemetry.addFailure = renderFailure;
            fixture.robot.updateAny(0.04);
            try {
                fixture.robot.updateTeleOp();
                fail("expected TeleOp presenter failure");
            } catch (RuntimeException failure) {
                assertSame(renderFailure, failure);
            }

            assertEquals(1, telemetry.updateAttempts());
            assertEquals(1, telemetry.committedFrames().size());
            assertTrue(telemetry.pendingContains("upstream.sidecar failed-render"));
        } finally {
            telemetry.addFailure = null;
            fixture.robot.stop();
        }
    }

    @Test
    public void presenterFailureProducesNoPartialCommit() throws Exception {
        RecordingTelemetry telemetry = new RecordingTelemetry();
        AutoFixture fixture = AutoFixture.create(telemetry);
        RuntimeException renderFailure = new IllegalStateException("controlled render failure");
        try {
            telemetry.proxy().addData("upstream.sidecar", "still pending");
            telemetry.addFailure = renderFailure;
            fixture.robot.updateAny(0.02);

            try {
                fixture.robot.updateAuto();
                fail("expected presenter failure");
            } catch (RuntimeException failure) {
                assertSame(renderFailure, failure);
            }

            assertEquals(0, telemetry.updateAttempts());
            assertEquals(0, telemetry.committedFrames().size());
            assertTrue(telemetry.pendingContains("upstream.sidecar"));
        } finally {
            telemetry.addFailure = null;
            fixture.robot.stop();
        }
    }

    @Test
    public void commitFailureIsAttemptedOnceAndPropagates() throws Exception {
        RecordingTelemetry telemetry = new RecordingTelemetry();
        AutoFixture fixture = AutoFixture.create(telemetry);
        RuntimeException commitFailure = new IllegalStateException("controlled commit failure");
        telemetry.updateFailure = commitFailure;
        try {
            fixture.robot.updateAny(0.02);

            try {
                fixture.robot.updateAuto();
                fail("expected telemetry commit failure");
            } catch (RuntimeException failure) {
                assertSame(commitFailure, failure);
            }

            assertEquals(1, telemetry.updateAttempts());
            assertEquals(0, telemetry.committedFrames().size());
        } finally {
            telemetry.updateFailure = null;
            fixture.robot.stop();
        }
    }

    private static boolean frameContains(List<String> frame, String text) {
        for (String entry : frame) {
            if (entry.contains(text)) {
                return true;
            }
        }
        return false;
    }

    private static void setField(Object target, String name, Object value) throws Exception {
        Field field = PhoenixRobot.class.getDeclaredField(name);
        field.setAccessible(true);
        field.set(target, value);
    }

    private static Object getField(Object target, String name) throws Exception {
        Field field = PhoenixRobot.class.getDeclaredField(name);
        field.setAccessible(true);
        return field.get(target);
    }

    private static LoopClock robotClock(PhoenixRobot robot) throws Exception {
        Field field = PhoenixRobot.class.getDeclaredField("clock");
        field.setAccessible(true);
        return (LoopClock) field.get(robot);
    }

    /** Add the ordinary TeleOp-only owners to the inert Auto fixture's shared robot graph. */
    private static void configureTeleOp(PhoenixRobot robot) throws Exception {
        PhoenixProfile profile = PhoenixProfile.current();
        FtcOdometryAprilTagLocalizationLane localization =
                (FtcOdometryAprilTagLocalizationLane) getField(robot, "localization");
        ScoringTargeting targeting = (ScoringTargeting) getField(robot, "scoringTargeting");
        PhoenixTeleOpControls controls = new PhoenixTeleOpControls(
                Gamepads.create(new Gamepad(), new Gamepad()),
                profile.controls
        );
        PhoenixReadiness.Result readiness = PhoenixReadiness.teleOpPoseAssists(profile);
        PhoenixDriveAssistService driveAssists = new PhoenixDriveAssistService(
                profile.driveAssist,
                controls.manualDriveSource(),
                controls.manualTranslateMagnitudeSource(),
                controls.autoAimEnabledSource(),
                readiness.isAllowed(),
                localization.globalEstimator(),
                targeting.aimOverlay()
        );
        MecanumDrivebase drive = new MecanumDrivebase(
                new InertPowerOutput(),
                new InertPowerOutput(),
                new InertPowerOutput(),
                new InertPowerOutput(),
                profile.drive.drivebase
        );

        setField(robot, "drive", drive);
        setField(robot, "teleOpControls", controls);
        setField(robot, "driveAssists", driveAssists);
        setField(robot, "teleOpDriveSource", driveAssists.driveSource());
        setField(robot, "teleOpPoseAssistReadiness", readiness);
    }

    /** A complete but inert Phoenix Auto graph, assembled without acquiring FTC devices. */
    private static final class AutoFixture {
        final PhoenixRobot robot;

        private AutoFixture(PhoenixRobot robot) {
            this.robot = robot;
        }

        static AutoFixture create(RecordingTelemetry telemetry) throws Exception {
            PhoenixProfile profile = PhoenixProfile.current();
            TestHardwareMap hardwareMap = TestHardwareMap.forScoring(profile.scoring);
            PhoenixRobot robot = new PhoenixRobot(
                    hardwareMap,
                    telemetry.proxy(),
                    new Gamepad(),
                    new Gamepad(),
                    profile
            );
            LoopClock clock = robotClock(robot);
            TestVisionLane vision = new TestVisionLane();
            TestMotionPredictor predictor = new TestMotionPredictor();
            FtcOdometryAprilTagLocalizationLane localization =
                    FtcOdometryAprilTagLocalizationLane.withPredictor(
                            predictor,
                            vision,
                            profile.field.fixedAprilTagLayout,
                            profile.localization
                    );
            ScoringTargeting targeting = new ScoringTargeting(
                    profile.autoAim,
                    profile.localization.aprilTags.fieldPoseSolver.copy(),
                    vision.tagSensor(),
                    vision.cameraMountConfig(),
                    localization.globalEstimator(),
                    profile.field.fixedAprilTagLayout,
                    BooleanSource.constant(true),
                    BooleanSource.constant(false),
                    profile.autoAim.shotVelocityTable
            );
            ScoringPath scoring = new ScoringPath(
                    hardwareMap,
                    profile.scoring,
                    targeting,
                    clock
            );

            setField(robot, "vision", vision);
            setField(robot, "localization", localization);
            setField(robot, "scoringTargeting", targeting);
            setField(robot, "scoringPath", scoring);
            setField(robot, "autonomousDrive", new InertDriveSink());
            setField(robot, "autoRoutineLifecycle", new PhoenixRobot.AutoRoutineLifecycle());

            robot.installAutoRoutine(new RunningTask());
            robot.startAny(0.0);
            robot.startAuto();
            return new AutoFixture(robot);
        }
    }

    private static final class InertDriveSink implements DriveCommandSink {
        @Override
        public void drive(DriveSignal signal) {
            // No physical drivetrain in this frame-ownership test.
        }

        @Override
        public void stop() {
            // No physical drivetrain in this frame-ownership test.
        }
    }

    private static final class InertPowerOutput implements PowerOutput {
        private double commandedPower;

        @Override
        public void setPower(double power) {
            commandedPower = power;
        }

        @Override
        public double getCommandedPower() {
            return commandedPower;
        }
    }

    private static final class RunningTask implements Task {
        private boolean started;
        private boolean cancelled;

        @Override
        public void start(LoopClock clock) {
            if (started) {
                throw new IllegalStateException("test task already started");
            }
            started = true;
        }

        @Override
        public void update(LoopClock clock) {
            if (!started) {
                throw new IllegalStateException("test task not started");
            }
        }

        @Override
        public void cancel() {
            if (started) {
                cancelled = true;
            }
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

    private static final class TestMotionPredictor implements MotionPredictor {
        private PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
        private MotionDelta delta = MotionDelta.none(LoopTimestamp.unavailable());

        @Override
        public void update(LoopClock clock) {
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

    private static final class TestVisionLane implements AprilTagVisionLane {
        private final AprilTagSensor sensor = new AprilTagSensor() {
            @Override
            public AprilTagDetections get(LoopClock clock) {
                return AprilTagDetections.none();
            }
        };

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
            return VisionReadiness.ready();
        }

        @Override
        public void close() {
            // No resource in the host-side fake.
        }
    }

    /** In-memory HardwareMap containing only the scoring devices this fixture constructs. */
    private static final class TestHardwareMap extends HardwareMap {
        private final Map<String, HardwareDevice> devices = new HashMap<String, HardwareDevice>();

        private TestHardwareMap() {
            super(null, null);
        }

        static TestHardwareMap forScoring(PhoenixProfile.ScoringPathConfig config) {
            TestHardwareMap map = new TestHardwareMap();
            map.devices.put(config.nameMotorIntake, deviceProxy(DcMotorEx.class, "intake"));
            map.devices.put(config.nameMotorShooterWheel, deviceProxy(DcMotorEx.class, "shooter"));
            map.devices.put(
                    config.nameCrServoIntakeTransfer,
                    deviceProxy(CRServo.class, "intakeTransfer")
            );
            map.devices.put(
                    config.nameCrServoShooterTransferLeft,
                    deviceProxy(CRServo.class, "shooterTransferLeft")
            );
            map.devices.put(
                    config.nameCrServoShooterTransferRight,
                    deviceProxy(CRServo.class, "shooterTransferRight")
            );
            return map;
        }

        @Override
        public <T> T get(Class<? extends T> type, String name) {
            HardwareDevice device = devices.get(name);
            if (device == null || !type.isInstance(device)) {
                throw new IllegalArgumentException("No test " + type.getSimpleName() + " named " + name);
            }
            return type.cast(device);
        }

        private static <T extends HardwareDevice> T deviceProxy(Class<T> type, String label) {
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
                        return null;
                    }
                    if ("getPower".equals(name)) {
                        return power;
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
    }

    private static final class RecordingTelemetry implements InvocationHandler {
        private final List<String> pendingEntries = new ArrayList<String>();
        private final List<List<String>> committedFrames = new ArrayList<List<String>>();
        private final Telemetry proxy = (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                this
        );
        private RuntimeException addFailure;
        private RuntimeException updateFailure;
        private int updateAttempts;
        private int clearCalls;

        Telemetry proxy() {
            return proxy;
        }

        int updateAttempts() {
            return updateAttempts;
        }

        int clearCalls() {
            return clearCalls;
        }

        List<List<String>> committedFrames() {
            return committedFrames;
        }

        boolean pendingContains(String text) {
            return frameContains(pendingEntries, text);
        }

        @Override
        public Object invoke(Object proxy, Method method, Object[] args) {
            String name = method.getName();
            if ("addData".equals(name) || "addLine".equals(name)) {
                if (addFailure != null) {
                    throw addFailure;
                }
                pendingEntries.add(renderArgs(args));
            } else if ("update".equals(name)) {
                updateAttempts++;
                if (updateFailure != null) {
                    throw updateFailure;
                }
                committedFrames.add(new ArrayList<String>(pendingEntries));
                pendingEntries.clear();
            } else if ("clear".equals(name) || "clearAll".equals(name)) {
                clearCalls++;
                pendingEntries.clear();
            }
            return defaultValue(method.getReturnType());
        }

        private static String renderArgs(Object[] args) {
            StringBuilder result = new StringBuilder();
            if (args == null) {
                return result.toString();
            }
            for (Object arg : args) {
                if (arg instanceof Object[]) {
                    for (Object nested : (Object[]) arg) {
                        append(result, nested);
                    }
                } else {
                    append(result, arg);
                }
            }
            return result.toString();
        }

        private static void append(StringBuilder result, Object value) {
            if (result.length() > 0) {
                result.append(' ');
            }
            result.append(value);
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
