package edu.ftcphoenix.robots.examples.starter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;

import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.ftc.FtcRobotOpMode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies starter configuration snapshots, declarations, and the managed TeleOp/Auto hosts. */
public final class StarterProfileAndRobotTest {

    @Test
    public void profileCopyIsDeepAndManagedHostRetainsThatSnapshot() {
        StarterProfile profile = readyProfile();
        StarterProfile copy = profile.copy();

        assertNotSame(profile.drive, copy.drive);
        assertNotSame(profile.drive.wiring, copy.drive.wiring);
        assertNotSame(profile.drive.drivebase, copy.drive.drivebase);
        assertNotSame(profile.intake, copy.intake);

        profile.drive.wiring.frontLeftName = "changedDrive";
        profile.intake.motorName = "changedIntake";
        profile.intake.collectPower = -0.10;

        assertEquals("frontLeft", copy.drive.wiring.frontLeftName);
        assertEquals("intake", copy.intake.motorName);
        assertEquals(0.65, copy.intake.collectPower, 0.0);

        TestHardwareMap hardwareMap = new TestHardwareMap();
        MotorProbe intakeMotor = hardwareMap.addMotor("intake");
        RecordingTelemetry telemetry = new RecordingTelemetry();
        StarterProfile source = readyProfile();
        StarterAuto mode = prepare(
                new StarterAuto(source),
                hardwareMap,
                telemetry,
                new Gamepad());

        source.intake.motorName = "missingAfterConstruction";
        source.intake.collectPower = -0.90;
        mode.init();
        mode.start();

        assertEquals(1, hardwareMap.lookupCalls);
        assertEquals(0.65, intakeMotor.lastPower, 0.0);
        mode.stop();
    }

    @Test
    public void invalidSharedAndTeleOpConfigurationFailBeforeHardwareLookup() {
        StarterProfile invalidAuto = readyProfile();
        invalidAuto.intake.collectPower = 0.0;
        TestHardwareMap autoMap = new TestHardwareMap();
        StarterAuto autoMode = prepare(
                new StarterAuto(invalidAuto),
                autoMap,
                new RecordingTelemetry(),
                new Gamepad());
        assertEquals(0, autoMap.lookupCalls);

        IllegalStateException autoFailure = expectIllegalState(autoMode::init);
        assertTrue(autoFailure.getMessage().contains("intake.collectPower"));
        assertTrue(autoFailure.getMessage().contains("nonzero"));
        assertEquals(0, autoMap.lookupCalls);

        StarterProfile invalidTeleOp = readyProfile();
        invalidTeleOp.drive.wiring.frontLeftName = invalidTeleOp.intake.motorName;
        TestHardwareMap teleOpMap = new TestHardwareMap();
        StarterTeleOp teleOpMode = prepare(
                new StarterTeleOp(invalidTeleOp),
                teleOpMap,
                new RecordingTelemetry(),
                new Gamepad());

        IllegalStateException teleOpFailure = expectIllegalState(
                teleOpMode::init);
        assertTrue(teleOpFailure.getMessage().contains("duplicates another starter motor name"));
        assertEquals(0, teleOpMap.lookupCalls);
    }

    @Test
    public void autoValidatesOnlyItsIntakeConfiguration() {
        StarterProfile profile = readyProfile();
        profile.drive = null;
        TestHardwareMap hardwareMap = new TestHardwareMap();
        hardwareMap.addMotor(profile.intake.motorName);
        StarterAuto mode = prepare(
                new StarterAuto(profile),
                hardwareMap,
                new RecordingTelemetry(),
                new Gamepad());

        mode.init();

        assertEquals(1, hardwareMap.lookupCalls);
        mode.stop();
    }

    @Test
    public void teleOpDeclarationMapsControlsRealizesOutputsAndPresentsStatus() {
        StarterProfile profile = readyProfile();
        List<String> events = new ArrayList<String>();
        TestHardwareMap hardwareMap = fullTeleOpHardware(profile, events);
        RecordingTelemetry telemetry = new RecordingTelemetry(events);
        Gamepad driver = new Gamepad();
        StarterTeleOp mode = prepare(
                new StarterTeleOp(profile),
                hardwareMap,
                telemetry,
                driver);

        mode.init();
        assertEquals(5, hardwareMap.lookupCalls);
        assertEquals(1, telemetry.updateCalls);
        assertEquals(
                Arrays.asList("intake.mode", "intake.appliedTargetPower"),
                telemetry.lastFrameKeys);
        assertEquals(2, telemetry.dataRowsAtLastUpdate);
        assertEquals(0.0,
                (Double) telemetry.dataValues.get("intake.appliedTargetPower"),
                0.0);

        mode.start();
        mode.loop();
        assertEquals(2, telemetry.updateCalls);
        assertEquals(2, telemetry.dataRowsAtLastUpdate);

        events.clear();
        driver.a = true;
        driver.left_stick_y = -1.0f;
        mode.loop();
        assertEquals(3, telemetry.updateCalls);
        assertEquals(2, telemetry.dataRowsAtLastUpdate);
        assertEquals(
                profile.intake.collectPower,
                (Double) telemetry.dataValues.get("intake.appliedTargetPower"),
                0.0);
        assertEquals(
                StarterIntake.Mode.COLLECT,
                telemetry.dataValues.get("intake.mode"));
        assertEquals(
                profile.intake.collectPower,
                hardwareMap.motor(profile.intake.motorName).lastPower,
                0.0);
        assertEventBefore(events, "power:intake:", "power:frontLeft:");
        assertEventBefore(events, "power:frontLeft:", "telemetry.row:intake.mode");
        assertEventBefore(events, "telemetry.row:intake.appliedTargetPower", "telemetry.commit");

        mode.stop();
        assertAllMotorsStopped(hardwareMap, profile);
        int powerWritesAfterFirstStop = hardwareMap.totalPowerWrites();

        mode.stop();
        mode.loop();
        assertEquals(powerWritesAfterFirstStop, hardwareMap.totalPowerWrites());
        assertEquals(3, telemetry.updateCalls);
    }

    @Test
    public void activeLoopFailureFailStopsEveryConstructedOutput() {
        StarterProfile profile = readyProfile();
        TestHardwareMap hardwareMap = fullTeleOpHardware(profile);
        RecordingTelemetry telemetry = new RecordingTelemetry();
        StarterTeleOp mode = prepare(
                new StarterTeleOp(profile),
                hardwareMap,
                telemetry,
                new Gamepad());

        mode.init();
        mode.start();
        telemetry.failNextUpdate = true;

        IllegalStateException failure = expectIllegalState(mode::loop);

        assertTrue(failure.getMessage().contains("telemetry update failed"));
        assertAllMotorsStopped(hardwareMap, profile);
        int writesAfterFailure = hardwareMap.totalPowerWrites();
        mode.loop();
        assertEquals(writesAfterFailure, hardwareMap.totalPowerWrites());
    }

    @Test
    public void autoDeclarationStartsOneFreshRootAndPresentsStatus() {
        StarterProfile profile = readyProfile();
        List<String> events = new ArrayList<String>();
        TestHardwareMap hardwareMap = new TestHardwareMap(events);
        MotorProbe intakeMotor = hardwareMap.addMotor(profile.intake.motorName);
        RecordingTelemetry telemetry = new RecordingTelemetry(events);
        StarterAuto mode = prepare(
                new StarterAuto(profile),
                hardwareMap,
                telemetry,
                new Gamepad());

        mode.init();
        assertEquals(1, telemetry.updateCalls);
        assertEquals(
                Arrays.asList("intake.mode", "intake.appliedTargetPower", "auto.idle"),
                telemetry.lastFrameKeys);
        assertEquals(3, telemetry.dataRowsAtLastUpdate);
        assertFalse((Boolean) telemetry.dataValues.get("auto.idle"));

        mode.start();
        assertEquals(
                profile.intake.collectPower,
                intakeMotor.lastPower,
                0.0);

        events.clear();
        mode.loop();
        assertEquals(2, telemetry.updateCalls);
        assertEquals(3, telemetry.dataRowsAtLastUpdate);
        assertEquals(
                profile.intake.collectPower,
                (Double) telemetry.dataValues.get("intake.appliedTargetPower"),
                0.0);
        assertFalse((Boolean) telemetry.dataValues.get("auto.idle"));
        assertEventBefore(events, "power:intake:", "telemetry.row:intake.mode");
        assertEventBefore(events, "telemetry.row:auto.idle", "telemetry.commit");

        mode.stop();
        assertEquals(0.0, intakeMotor.lastPower, 0.0);
        int writesAfterFirstStop = intakeMotor.powerWrites;

        mode.stop();
        mode.loop();
        assertEquals(writesAfterFirstStop, intakeMotor.powerWrites);
        assertEquals(2, telemetry.updateCalls);
    }

    private static <T extends FtcRobotOpMode> T prepare(
            T mode,
            HardwareMap hardwareMap,
            RecordingTelemetry telemetry,
            Gamepad gamepad1) {
        mode.hardwareMap = hardwareMap;
        mode.telemetry = telemetry.proxy();
        mode.gamepad1 = gamepad1;
        mode.gamepad2 = new Gamepad();
        mode.resetRuntime();
        return mode;
    }

    private static StarterProfile readyProfile() {
        StarterProfile profile = StarterProfile.current();
        profile.hardwareConfigurationReviewed = true;
        profile.intake.motorName = "intake";
        profile.intake.direction = Direction.FORWARD;
        profile.intake.collectPower = 0.65;
        profile.intake.ejectPower = -0.45;

        profile.drive.wiring.frontLeftName = "frontLeft";
        profile.drive.wiring.frontRightName = "frontRight";
        profile.drive.wiring.backLeftName = "backLeft";
        profile.drive.wiring.backRightName = "backRight";
        profile.drive.wiring.frontLeftDirection = Direction.FORWARD;
        profile.drive.wiring.frontRightDirection = Direction.REVERSE;
        profile.drive.wiring.backLeftDirection = Direction.FORWARD;
        profile.drive.wiring.backRightDirection = Direction.REVERSE;
        return profile;
    }

    private static TestHardwareMap fullTeleOpHardware(StarterProfile profile) {
        return fullTeleOpHardware(profile, null);
    }

    private static TestHardwareMap fullTeleOpHardware(StarterProfile profile,
                                                       List<String> events) {
        TestHardwareMap hardwareMap = new TestHardwareMap(events);
        hardwareMap.addMotor(profile.intake.motorName);
        hardwareMap.addMotor(profile.drive.wiring.frontLeftName);
        hardwareMap.addMotor(profile.drive.wiring.frontRightName);
        hardwareMap.addMotor(profile.drive.wiring.backLeftName);
        hardwareMap.addMotor(profile.drive.wiring.backRightName);
        return hardwareMap;
    }

    private static void assertEventBefore(List<String> events,
                                          String earlierPrefix,
                                          String laterPrefix) {
        int earlier = eventIndex(events, earlierPrefix);
        int later = eventIndex(events, laterPrefix);
        assertTrue("Missing event prefix " + earlierPrefix + " in " + events, earlier >= 0);
        assertTrue("Missing event prefix " + laterPrefix + " in " + events, later >= 0);
        assertTrue("Expected " + earlierPrefix + " before " + laterPrefix + " in " + events,
                earlier < later);
    }

    private static int eventIndex(List<String> events, String prefix) {
        for (int index = 0; index < events.size(); index++) {
            if (events.get(index).startsWith(prefix)) {
                return index;
            }
        }
        return -1;
    }

    private static void assertAllMotorsStopped(TestHardwareMap hardwareMap,
                                               StarterProfile profile) {
        assertEquals(0.0, hardwareMap.motor(profile.intake.motorName).lastPower, 0.0);
        assertEquals(0.0, hardwareMap.motor(profile.drive.wiring.frontLeftName).lastPower, 0.0);
        assertEquals(0.0, hardwareMap.motor(profile.drive.wiring.frontRightName).lastPower, 0.0);
        assertEquals(0.0, hardwareMap.motor(profile.drive.wiring.backLeftName).lastPower, 0.0);
        assertEquals(0.0, hardwareMap.motor(profile.drive.wiring.backRightName).lastPower, 0.0);
    }

    private static IllegalStateException expectIllegalState(Runnable action) {
        try {
            action.run();
            fail("Expected IllegalStateException");
            throw new AssertionError("unreachable");
        } catch (IllegalStateException expected) {
            return expected;
        }
    }

    /** In-memory FTC map with observable lookups and motor commands. */
    private static final class TestHardwareMap extends HardwareMap {
        private final Map<String, MotorProbe> motors = new HashMap<String, MotorProbe>();
        private final List<String> events;
        private int lookupCalls;

        private TestHardwareMap() {
            this(null);
        }

        private TestHardwareMap(List<String> events) {
            super(null, null);
            this.events = events;
        }

        private MotorProbe addMotor(String name) {
            MotorProbe probe = new MotorProbe(name, events);
            motors.put(name.trim(), probe);
            return probe;
        }

        private MotorProbe motor(String name) {
            MotorProbe probe = motors.get(name.trim());
            if (probe == null) {
                throw new AssertionError("No test motor named " + name);
            }
            return probe;
        }

        private int totalPowerWrites() {
            int total = 0;
            for (MotorProbe probe : motors.values()) {
                total += probe.powerWrites;
            }
            return total;
        }

        @Override
        public <T> T get(Class<? extends T> type, String name) {
            lookupCalls++;
            MotorProbe probe = motors.get(name == null ? null : name.trim());
            if (probe == null || !type.isInstance(probe.motor)) {
                throw new IllegalArgumentException("No test " + type.getSimpleName()
                        + " named " + name);
            }
            return type.cast(probe.motor);
        }
    }

    /** Dynamic SDK motor with enough behavior for direct-drive and power-Plant integration. */
    private static final class MotorProbe implements InvocationHandler {
        private final String label;
        private final List<String> events;
        private final DcMotorEx motor;
        private DcMotor.RunMode mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        private DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
        private DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT;
        private double lastPower;
        private int powerWrites;

        private MotorProbe(String label, List<String> events) {
            this.label = label;
            this.events = events;
            motor = (DcMotorEx) Proxy.newProxyInstance(
                    DcMotorEx.class.getClassLoader(),
                    new Class<?>[]{DcMotorEx.class},
                    this);
        }

        @Override
        public Object invoke(Object proxy, Method method, Object[] args) {
            String name = method.getName();
            if (method.getDeclaringClass() == Object.class) {
                return objectMethod(proxy, name, args, label);
            }
            if ("setDirection".equals(name)) {
                direction = (DcMotorSimple.Direction) args[0];
                return null;
            }
            if ("getDirection".equals(name)) {
                return direction;
            }
            if ("setMode".equals(name)) {
                mode = (DcMotor.RunMode) args[0];
                return null;
            }
            if ("getMode".equals(name)) {
                return mode;
            }
            if ("setPower".equals(name)) {
                lastPower = (Double) args[0];
                powerWrites++;
                if (events != null) {
                    events.add("power:" + label + ":" + lastPower);
                }
                return null;
            }
            if ("getPower".equals(name)) {
                return lastPower;
            }
            if ("setZeroPowerBehavior".equals(name)) {
                zeroPowerBehavior = (DcMotor.ZeroPowerBehavior) args[0];
                return null;
            }
            if ("getZeroPowerBehavior".equals(name)) {
                return zeroPowerBehavior;
            }
            if ("getCurrentPosition".equals(name) || "getTargetPosition".equals(name)) {
                return 0;
            }
            if ("getVelocity".equals(name)) {
                return 0.0;
            }
            if ("isBusy".equals(name)) {
                return false;
            }
            return defaultValue(method.getReturnType());
        }
    }

    /** Records complete-frame commits without imposing another production seam. */
    private static final class RecordingTelemetry implements InvocationHandler {
        private final Telemetry telemetry = (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                this);
        private final List<String> dataKeys = new ArrayList<String>();
        private final List<String> lastFrameKeys = new ArrayList<String>();
        private final Map<String, Object> dataValues = new HashMap<String, Object>();
        private int updateCalls;
        private int dataRowsAtLastUpdate;
        private int committedDataRows;
        private boolean failNextUpdate;
        private final List<String> events;

        private RecordingTelemetry() {
            this(null);
        }

        private RecordingTelemetry(List<String> events) {
            this.events = events;
        }

        private Telemetry proxy() {
            return telemetry;
        }

        @Override
        public Object invoke(Object proxy, Method method, Object[] args) {
            if (method.getDeclaringClass() == Object.class) {
                return objectMethod(proxy, method.getName(), args, "RecordingTelemetry");
            }
            if ("update".equals(method.getName())) {
                if (events != null) {
                    events.add("telemetry.commit");
                }
                if (failNextUpdate) {
                    failNextUpdate = false;
                    throw new IllegalStateException("telemetry update failed");
                }
                dataRowsAtLastUpdate = dataKeys.size() - committedDataRows;
                lastFrameKeys.clear();
                lastFrameKeys.addAll(dataKeys.subList(committedDataRows, dataKeys.size()));
                committedDataRows = dataKeys.size();
                updateCalls++;
                return true;
            }
            if ("addData".equals(method.getName())) {
                dataKeys.add((String) args[0]);
                dataValues.put((String) args[0], args[1]);
                if (events != null) {
                    events.add("telemetry.row:" + args[0]);
                }
                return null;
            }
            return defaultValue(method.getReturnType());
        }
    }

    private static Object objectMethod(Object proxy,
                                       String methodName,
                                       Object[] args,
                                       String label) {
        if ("equals".equals(methodName)) {
            return proxy == args[0];
        }
        if ("hashCode".equals(methodName)) {
            return System.identityHashCode(proxy);
        }
        if ("toString".equals(methodName)) {
            return label;
        }
        return null;
    }

    private static Object defaultValue(Class<?> type) {
        if (!type.isPrimitive()) {
            return null;
        }
        if (type == boolean.class) {
            return false;
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
