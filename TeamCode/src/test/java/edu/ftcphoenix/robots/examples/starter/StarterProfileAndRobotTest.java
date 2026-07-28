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
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;
import edu.ftcphoenix.fw.task.Tasks;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies starter configuration snapshots and the complete TeleOp/Auto lifecycle roots. */
public final class StarterProfileAndRobotTest {

    @Test
    public void profileCopyIsDeepAndRobotRetainsThatSnapshot() {
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
        StarterRobot robot = new StarterRobot(hardwareMap, telemetry.proxy(), source);

        source.intake.motorName = "missingAfterConstruction";
        source.intake.collectPower = -0.90;
        robot.initAuto();
        Task task = robot.intake().collectForSeconds(10.0);
        robot.installAutoRoutine(task);
        robot.start(0.0);
        robot.update(0.02);

        assertEquals(1, hardwareMap.lookupCalls);
        assertEquals(0.65, intakeMotor.lastPower, 0.0);
        robot.stop();
    }

    @Test
    public void invalidSharedAndTeleOpConfigurationFailBeforeHardwareLookup() {
        RecordingTelemetry telemetry = new RecordingTelemetry();

        StarterProfile invalidAuto = readyProfile();
        invalidAuto.intake.collectPower = 0.0;
        TestHardwareMap autoMap = new TestHardwareMap();
        StarterRobot autoRobot = new StarterRobot(
                autoMap,
                telemetry.proxy(),
                invalidAuto);
        assertEquals(0, autoMap.lookupCalls);

        IllegalStateException autoFailure = expectIllegalState(autoRobot::initAuto);
        assertTrue(autoFailure.getMessage().contains("intake.collectPower"));
        assertTrue(autoFailure.getMessage().contains("nonzero"));
        assertEquals(0, autoMap.lookupCalls);

        StarterProfile invalidTeleOp = readyProfile();
        invalidTeleOp.drive.wiring.frontLeftName = invalidTeleOp.intake.motorName;
        TestHardwareMap teleOpMap = new TestHardwareMap();
        StarterRobot teleOpRobot = new StarterRobot(
                teleOpMap,
                telemetry.proxy(),
                invalidTeleOp);

        IllegalStateException teleOpFailure = expectIllegalState(
                () -> teleOpRobot.initTeleOp(new Gamepad()));
        assertTrue(teleOpFailure.getMessage().contains("duplicates another starter motor name"));
        assertEquals(0, teleOpMap.lookupCalls);
    }

    @Test
    public void autoValidatesOnlyItsIntakeConfiguration() {
        StarterProfile profile = readyProfile();
        profile.drive = null;
        TestHardwareMap hardwareMap = new TestHardwareMap();
        hardwareMap.addMotor(profile.intake.motorName);
        StarterRobot robot = new StarterRobot(
                hardwareMap,
                new RecordingTelemetry().proxy(),
                profile);

        robot.initAuto();

        assertEquals(1, hardwareMap.lookupCalls);
        robot.stop();
    }

    @Test
    public void teleOpLifecycleIsOneShotCommitsOncePerUpdateAndStopsOnce() {
        StarterProfile profile = readyProfile();
        List<String> events = new ArrayList<String>();
        TestHardwareMap hardwareMap = fullTeleOpHardware(profile, events);
        RecordingTelemetry telemetry = new RecordingTelemetry(events);
        StarterRobot robot = new StarterRobot(hardwareMap, telemetry.proxy(), profile);
        Gamepad driver = new Gamepad();

        robot.initTeleOp(driver);
        assertEquals(5, hardwareMap.lookupCalls);
        expectIllegalState(robot::initAuto);

        robot.start(1.0);
        expectIllegalState(() -> robot.start(1.1));

        robot.update(1.02);
        assertEquals(1, telemetry.updateCalls);
        assertEquals(
                Arrays.asList("intake.mode", "intake.appliedTargetPower"),
                telemetry.dataKeys);
        assertEquals(2, telemetry.dataRowsAtLastUpdate);
        assertEquals(0.0,
                (Double) telemetry.dataValues.get("intake.appliedTargetPower"),
                0.0);

        events.clear();
        driver.a = true;
        driver.left_stick_y = -1.0f;
        robot.update(1.04);
        assertEquals(2, telemetry.updateCalls);
        assertEquals(2, telemetry.dataRowsAtLastUpdate);
        assertEquals(
                profile.intake.collectPower,
                (Double) telemetry.dataValues.get("intake.appliedTargetPower"),
                0.0);
        assertEquals(StarterIntake.Mode.COLLECT, robot.intake().status().mode());
        assertEquals(
                profile.intake.collectPower,
                hardwareMap.motor(profile.intake.motorName).lastPower,
                0.0);
        assertEventBefore(events, "power:frontLeft:", "power:intake:");
        assertEventBefore(events, "power:intake:", "telemetry.row:intake.mode");
        assertEventBefore(events, "telemetry.row:intake.appliedTargetPower", "telemetry.commit");

        robot.stop();
        assertAllMotorsStopped(hardwareMap, profile);
        int powerWritesAfterFirstStop = hardwareMap.totalPowerWrites();

        robot.stop();
        robot.update(1.06);
        assertEquals(powerWritesAfterFirstStop, hardwareMap.totalPowerWrites());
        assertEquals(2, telemetry.updateCalls);
    }

    @Test
    public void activeLoopFailureFailStopsEveryConstructedOutput() {
        StarterProfile profile = readyProfile();
        TestHardwareMap hardwareMap = fullTeleOpHardware(profile);
        RecordingTelemetry telemetry = new RecordingTelemetry();
        StarterRobot robot = new StarterRobot(hardwareMap, telemetry.proxy(), profile);

        robot.initTeleOp(new Gamepad());
        robot.start(0.0);
        telemetry.failNextUpdate = true;

        IllegalStateException failure = expectIllegalState(() -> robot.update(0.02));

        assertTrue(failure.getMessage().contains("telemetry update failed"));
        assertAllMotorsStopped(hardwareMap, profile);
        int writesAfterFailure = hardwareMap.totalPowerWrites();
        robot.update(0.04);
        assertEquals(writesAfterFailure, hardwareMap.totalPowerWrites());
    }

    @Test
    public void autoLifecycleRunsOneRootCommitsOnceAndCancelsBeforeStopCommand() {
        StarterProfile profile = readyProfile();
        List<String> events = new ArrayList<String>();
        TestHardwareMap hardwareMap = new TestHardwareMap(events);
        MotorProbe intakeMotor = hardwareMap.addMotor(profile.intake.motorName);
        RecordingTelemetry telemetry = new RecordingTelemetry(events);
        StarterRobot robot = new StarterRobot(hardwareMap, telemetry.proxy(), profile);

        robot.initAuto();
        Task collect = robot.intake().collectForSeconds(10.0);
        robot.installAutoRoutine(new EventLoggingTask(collect, events));
        expectIllegalState(() -> robot.installAutoRoutine(Tasks.noop()));
        robot.start(0.0);

        robot.update(0.02);
        assertEquals(1, telemetry.updateCalls);
        assertEquals(
                Arrays.asList("intake.mode", "intake.appliedTargetPower", "auto.idle"),
                telemetry.dataKeys);
        assertEquals(3, telemetry.dataRowsAtLastUpdate);
        assertEquals(
                profile.intake.collectPower,
                (Double) telemetry.dataValues.get("intake.appliedTargetPower"),
                0.0);
        assertEventBefore(events, "task.start", "power:intake:");
        assertEventBefore(events, "power:intake:", "telemetry.row:intake.mode");
        assertEventBefore(events, "telemetry.row:auto.idle", "telemetry.commit");
        assertEquals(TaskOutcome.NOT_DONE, collect.getOutcome());
        assertEquals(profile.intake.collectPower, intakeMotor.lastPower, 0.0);

        events.clear();
        robot.stop();
        assertEventBefore(events, "task.cancel", "power:intake:");
        assertEquals(TaskOutcome.CANCELLED, collect.getOutcome());
        assertEquals(0.0, intakeMotor.lastPower, 0.0);
        assertEquals(StarterIntake.Mode.STOPPED, robot.intake().status().mode());
        int writesAfterFirstStop = intakeMotor.powerWrites;

        robot.stop();
        robot.update(0.04);
        assertEquals(writesAfterFirstStop, intakeMotor.powerWrites);
        assertEquals(1, telemetry.updateCalls);
    }

    @Test
    public void autoAdvancesOneClockCyclePerRootUpdate() {
        StarterProfile profile = readyProfile();
        TestHardwareMap hardwareMap = new TestHardwareMap();
        hardwareMap.addMotor(profile.intake.motorName);
        StarterRobot robot = new StarterRobot(
                hardwareMap,
                new RecordingTelemetry().proxy(),
                profile);
        CycleRecordingTask routine = new CycleRecordingTask();

        robot.initAuto();
        robot.installAutoRoutine(routine);
        robot.start(1.0);
        robot.update(1.02);
        robot.update(1.04);

        assertEquals(2L, routine.startCycle);
        assertEquals(Arrays.asList(2L, 3L), routine.updateCycles);
        robot.stop();
        assertTrue(routine.cancelled);
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

    /** Active test Task that exposes which root clock cycles reached the Auto runner. */
    private static final class CycleRecordingTask implements Task {
        private final List<Long> updateCycles = new ArrayList<Long>();
        private long startCycle = Long.MIN_VALUE;
        private boolean started;
        private boolean cancelled;

        @Override
        public void start(edu.ftcphoenix.fw.core.time.LoopClock clock) {
            if (started) {
                throw new IllegalStateException("CycleRecordingTask is single-use");
            }
            started = true;
            startCycle = clock.cycle();
        }

        @Override
        public void update(edu.ftcphoenix.fw.core.time.LoopClock clock) {
            if (!started) {
                throw new IllegalStateException("CycleRecordingTask must start before update");
            }
            updateCycles.add(clock.cycle());
        }

        @Override
        public void cancel() {
            if (started && !cancelled) {
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

    /** Delegates one real capability Task while exposing root-level Task ordering to the test. */
    private static final class EventLoggingTask implements Task {
        private final Task delegate;
        private final List<String> events;

        private EventLoggingTask(Task delegate, List<String> events) {
            this.delegate = delegate;
            this.events = events;
        }

        @Override
        public void start(edu.ftcphoenix.fw.core.time.LoopClock clock) {
            events.add("task.start");
            delegate.start(clock);
        }

        @Override
        public void update(edu.ftcphoenix.fw.core.time.LoopClock clock) {
            events.add("task.update");
            delegate.update(clock);
        }

        @Override
        public void cancel() {
            events.add("task.cancel");
            delegate.cancel();
        }

        @Override
        public boolean isComplete() {
            return delegate.isComplete();
        }

        @Override
        public TaskOutcome getOutcome() {
            return delegate.getOutcome();
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
