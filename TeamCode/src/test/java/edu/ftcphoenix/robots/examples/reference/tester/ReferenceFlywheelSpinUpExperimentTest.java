package edu.ftcphoenix.robots.examples.reference.tester;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.tools.tester.TesterContext;
import edu.ftcphoenix.robots.examples.reference.capability.launcher.ReferenceLauncherMechanism;
import edu.ftcphoenix.fw.testing.ftc.FtcTestHardware;
import edu.ftcphoenix.fw.testing.ftc.FtcTestHardware.MotorProbe;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies locked criteria, ACTIVE gating, retained outcomes, and minimal telemetry. */
public final class ReferenceFlywheelSpinUpExperimentTest {

    private static final double EPSILON = 1e-9;

    @Test
    public void currentCriteriaAreLockedAndUseOnlyExplicitExperimentFields() throws Exception {
        ReferenceFlywheelSpinUpCriteria criteria = ReferenceFlywheelSpinUpCriteria.current();

        assertFalse(criteria.reviewedForMotion);
        assertTrue(criteria.targetVelocityTicksPerSec > 0.0);
        assertTrue(criteria.maximumPoweredRunSec > 0.0);
        assertEquals(boolean.class,
                ReferenceFlywheelSpinUpCriteria.class
                        .getDeclaredField("reviewedForMotion").getType());
        assertEquals(double.class,
                ReferenceFlywheelSpinUpCriteria.class
                        .getDeclaredField("targetVelocityTicksPerSec").getType());
        assertEquals(double.class,
                ReferenceFlywheelSpinUpCriteria.class
                        .getDeclaredField("maximumPoweredRunSec").getType());
        assertMissingMethod("locked");
    }

    @Test
    public void invalidLockedCriteriaFailBeforeAnyHardwareLookup() {
        ReferenceLauncherMechanism.Config config = testConfig();
        ReferenceFlywheelSpinUpCriteria criteria = ReferenceFlywheelSpinUpCriteria.current();
        criteria.reviewedForMotion = true;
        criteria.targetVelocityTicksPerSec = config.velocityToleranceTicksPerSec;
        FtcTestHardware hardware = new FtcTestHardware();
        ReferenceFlywheelSpinUpExperiment tester =
                new ReferenceFlywheelSpinUpExperiment(config, criteria);

        IllegalArgumentException failure = assertThrows(
                IllegalArgumentException.class,
                () -> tester.init(context(hardware, telemetry(), new Gamepad(), new LoopClock())));

        assertTrue(failure.getMessage().contains("targetVelocityTicksPerSec"));
        assertTrue(failure.getMessage().contains("velocityToleranceTicksPerSec"));
        assertEquals(0, hardware.lookupCalls());
    }

    @Test
    public void currentLockedCriteriaRenderWithoutHardwareLookup() {
        FtcTestHardware hardware = new FtcTestHardware();
        RecordingTelemetry telemetry = new RecordingTelemetry();
        LoopClock clock = new LoopClock();
        ReferenceFlywheelSpinUpExperiment tester = new ReferenceFlywheelSpinUpExperiment(
                ReferenceLauncherMechanism.Config.defaults(),
                ReferenceFlywheelSpinUpCriteria.current());

        tester.init(context(hardware, telemetry.telemetry, new Gamepad(), clock));
        tester.initLoop(0.0);

        assertEquals(0, hardware.lookupCalls());
        assertEquals(ReferenceFlywheelSpinUpExperiment.TrialState.IDLE,
                telemetry.value("trialState"));
        assertTrue(telemetry.hasLineContaining("LOCKED"));
        assertFalse(telemetry.has("trialNumber"));
        assertFalse(telemetry.has("targetVelocityTicksPerSec"));
        assertMinimalTelemetry(telemetry);
        tester.stop();
    }

    @Test
    public void initAIsInertAndFirstActiveIdleLoopCannotArmLatentMotion() {
        Rig rig = new Rig();
        rig.initCycle(0.02);
        rig.gamepad1.a = true;
        rig.initCycle(0.02);

        assertEquals(ReferenceFlywheelSpinUpExperiment.TrialState.IDLE,
                rig.telemetry.value("trialState"));
        assertFalse(rig.telemetry.has("trialNumber"));
        assertFalse(rig.telemetry.has("targetVelocityTicksPerSec"));
        assertEquals(0.0, rig.left.commandedVelocityTicksPerSec(), EPSILON);
        assertEquals(0.0, rig.right.commandedVelocityTicksPerSec(), EPSILON);

        rig.gamepad1.a = false;
        rig.initCycle(0.02);
        rig.tester.start();
        rig.activeCycle(0.02);

        assertEquals(ReferenceFlywheelSpinUpExperiment.TrialState.IDLE,
                rig.telemetry.value("trialState"));
        assertFalse(rig.telemetry.has("trialNumber"));
        assertFalse(rig.telemetry.has("spinUpSec"));
        assertEquals(0.0, rig.left.commandedVelocityTicksPerSec(), EPSILON);
        assertEquals(0.0, rig.right.commandedVelocityTicksPerSec(), EPSILON);
        assertMinimalTelemetry(rig.telemetry);
        rig.tester.stop();
    }

    @Test
    public void readinessFirstPublishedBeforeDeadlineFreezesThatSameLoop() {
        Rig rig = new Rig();
        rig.startActive();
        rig.gamepad1.a = true;
        rig.activeCycle(0.02);
        rig.left.setMeasuredVelocityTicksPerSec(rig.criteria.targetVelocityTicksPerSec);
        rig.right.setMeasuredVelocityTicksPerSec(rig.criteria.targetVelocityTicksPerSec);

        double observedSpinUpSec = rig.criteria.maximumPoweredRunSec * 0.4;
        rig.activeCycle(observedSpinUpSec);

        assertEquals(ReferenceFlywheelSpinUpExperiment.TrialState.TARGET_REACHED,
                rig.telemetry.value("trialState"));
        assertNumber(observedSpinUpSec, rig.telemetry.value("elapsedSec"));
        assertNumber(observedSpinUpSec, rig.telemetry.value("spinUpSec"));
        assertNumber(rig.criteria.targetVelocityTicksPerSec,
                rig.telemetry.value("leftMeasuredVelocityTicksPerSec"));
        assertNumber(rig.criteria.targetVelocityTicksPerSec,
                rig.telemetry.value("rightMeasuredVelocityTicksPerSec"));

        rig.left.setMeasuredVelocityTicksPerSec(0.0);
        rig.right.setMeasuredVelocityTicksPerSec(0.0);
        rig.gamepad1.a = false;
        rig.activeCycle(0.20);
        assertEquals(0.0, rig.left.commandedVelocityTicksPerSec(), EPSILON);
        assertEquals(0.0, rig.right.commandedVelocityTicksPerSec(), EPSILON);
        assertNumber(observedSpinUpSec, rig.telemetry.value("spinUpSec"));
        assertNumber(rig.criteria.targetVelocityTicksPerSec,
                rig.telemetry.value("leftMeasuredVelocityTicksPerSec"));
        assertNumber(rig.criteria.targetVelocityTicksPerSec,
                rig.telemetry.value("rightMeasuredVelocityTicksPerSec"));
        rig.tester.stop();
    }

    @Test
    public void readinessFirstAvailableAtBoundaryLosesToPreOutputTimeLimit() {
        Rig rig = new Rig();
        rig.startActive();
        rig.gamepad1.a = true;
        rig.activeCycle(0.02);
        rig.left.setMeasuredVelocityTicksPerSec(rig.criteria.targetVelocityTicksPerSec);
        rig.right.setMeasuredVelocityTicksPerSec(rig.criteria.targetVelocityTicksPerSec);

        rig.activeCycle(rig.criteria.maximumPoweredRunSec);

        assertEquals(ReferenceFlywheelSpinUpExperiment.TrialState.TIME_LIMIT_REACHED,
                rig.telemetry.value("trialState"));
        assertEquals(1L, rig.telemetry.value("trialNumber"));
        assertNumber(rig.criteria.targetVelocityTicksPerSec,
                rig.telemetry.value("targetVelocityTicksPerSec"));
        assertNumber(rig.criteria.maximumPoweredRunSec,
                rig.telemetry.value("elapsedSec"));
        assertFalse(rig.telemetry.has("spinUpSec"));
        assertEquals(0.0, rig.left.commandedVelocityTicksPerSec(), EPSILON);
        assertEquals(0.0, rig.right.commandedVelocityTicksPerSec(), EPSILON);

        rig.left.setMeasuredVelocityTicksPerSec(0.0);
        rig.right.setMeasuredVelocityTicksPerSec(0.0);
        rig.gamepad1.a = false;
        rig.activeCycle(0.20);

        assertEquals(ReferenceFlywheelSpinUpExperiment.TrialState.TIME_LIMIT_REACHED,
                rig.telemetry.value("trialState"));
        assertNumber(0.0,
                rig.telemetry.value("leftMeasuredVelocityTicksPerSec"));
        assertNumber(0.0,
                rig.telemetry.value("rightMeasuredVelocityTicksPerSec"));
        assertNumber(rig.criteria.maximumPoweredRunSec,
                rig.telemetry.value("elapsedSec"));
        assertEquals(0.0, rig.left.commandedVelocityTicksPerSec(), EPSILON);
        assertEquals(0.0, rig.right.commandedVelocityTicksPerSec(), EPSILON);
        assertMinimalTelemetry(rig.telemetry);
        rig.tester.stop();
    }

    @Test
    public void cooperativeTimeLimitFreezesObservedOvershootAndZerosThatCycle() {
        Rig rig = new Rig();
        rig.startActive();
        rig.gamepad1.a = true;
        rig.activeCycle(0.02);
        rig.left.setMeasuredVelocityTicksPerSec(125.0);
        rig.right.setMeasuredVelocityTicksPerSec(275.0);

        rig.activeCycle(rig.criteria.maximumPoweredRunSec * 0.5);
        assertEquals(ReferenceFlywheelSpinUpExperiment.TrialState.RUNNING,
                rig.telemetry.value("trialState"));
        double observedTimeoutSec = rig.criteria.maximumPoweredRunSec * 1.25;
        rig.activeCycle(rig.criteria.maximumPoweredRunSec * 0.75);

        assertEquals(ReferenceFlywheelSpinUpExperiment.TrialState.TIME_LIMIT_REACHED,
                rig.telemetry.value("trialState"));
        assertNumber(rig.criteria.targetVelocityTicksPerSec,
                rig.telemetry.value("targetVelocityTicksPerSec"));
        assertNumber(125.0, rig.telemetry.value("leftMeasuredVelocityTicksPerSec"));
        assertNumber(275.0, rig.telemetry.value("rightMeasuredVelocityTicksPerSec"));
        assertNumber(observedTimeoutSec, rig.telemetry.value("elapsedSec"));
        assertFalse(rig.telemetry.has("spinUpSec"));
        assertEquals(0.0, rig.left.commandedVelocityTicksPerSec(), EPSILON);
        assertEquals(0.0, rig.right.commandedVelocityTicksPerSec(), EPSILON);

        rig.left.setMeasuredVelocityTicksPerSec(rig.criteria.targetVelocityTicksPerSec);
        rig.right.setMeasuredVelocityTicksPerSec(rig.criteria.targetVelocityTicksPerSec);
        rig.gamepad1.a = false;
        rig.activeCycle(0.20);

        assertEquals(ReferenceFlywheelSpinUpExperiment.TrialState.TIME_LIMIT_REACHED,
                rig.telemetry.value("trialState"));
        assertNumber(125.0, rig.telemetry.value("leftMeasuredVelocityTicksPerSec"));
        assertNumber(275.0, rig.telemetry.value("rightMeasuredVelocityTicksPerSec"));
        assertNumber(observedTimeoutSec, rig.telemetry.value("elapsedSec"));
        assertFalse(rig.telemetry.has("spinUpSec"));
        assertMinimalTelemetry(rig.telemetry);
        rig.tester.stop();
    }

    @Test
    public void simultaneousAAndBFreezeAuthoredTargetAndAWhileRunningCannotRestart() {
        Rig rig = new Rig();
        rig.startActive();
        rig.gamepad1.a = true;
        rig.gamepad1.b = true;
        rig.activeCycle(0.02);

        assertEquals(ReferenceFlywheelSpinUpExperiment.TrialState.ABORTED,
                rig.telemetry.value("trialState"));
        assertEquals(1L, rig.telemetry.value("trialNumber"));
        assertNumber(rig.criteria.targetVelocityTicksPerSec,
                rig.telemetry.value("targetVelocityTicksPerSec"));
        assertFalse(rig.telemetry.has("spinUpSec"));
        assertEquals(0.0, rig.left.commandedVelocityTicksPerSec(), EPSILON);

        rig.gamepad1.a = false;
        rig.gamepad1.b = false;
        rig.activeCycle(0.02);
        rig.gamepad1.a = true;
        rig.activeCycle(0.02);
        assertEquals(ReferenceFlywheelSpinUpExperiment.TrialState.RUNNING,
                rig.telemetry.value("trialState"));
        assertEquals(2L, rig.telemetry.value("trialNumber"));

        rig.gamepad1.a = false;
        rig.activeCycle(0.20);
        double elapsedBeforeSecondA = number(rig.telemetry.value("elapsedSec"));
        rig.gamepad1.a = true;
        rig.activeCycle(0.20);

        assertEquals(ReferenceFlywheelSpinUpExperiment.TrialState.RUNNING,
                rig.telemetry.value("trialState"));
        assertEquals(2L, rig.telemetry.value("trialNumber"));
        assertTrue(number(rig.telemetry.value("elapsedSec")) > elapsedBeforeSecondA);

        rig.left.setMeasuredVelocityTicksPerSec(333.0);
        rig.right.setMeasuredVelocityTicksPerSec(444.0);
        rig.gamepad1.a = false;
        rig.activeCycle(0.02);
        rig.gamepad1.b = true;
        rig.activeCycle(0.02);
        assertEquals(ReferenceFlywheelSpinUpExperiment.TrialState.ABORTED,
                rig.telemetry.value("trialState"));
        assertEquals(2L, rig.telemetry.value("trialNumber"));
        assertNumber(333.0, rig.telemetry.value("leftMeasuredVelocityTicksPerSec"));
        assertNumber(444.0, rig.telemetry.value("rightMeasuredVelocityTicksPerSec"));
        double abortedElapsedSec = number(rig.telemetry.value("elapsedSec"));
        assertFalse(rig.telemetry.has("spinUpSec"));

        rig.left.setMeasuredVelocityTicksPerSec(rig.criteria.targetVelocityTicksPerSec);
        rig.right.setMeasuredVelocityTicksPerSec(rig.criteria.targetVelocityTicksPerSec);
        rig.gamepad1.b = false;
        rig.activeCycle(0.20);
        assertEquals(ReferenceFlywheelSpinUpExperiment.TrialState.ABORTED,
                rig.telemetry.value("trialState"));
        assertNumber(333.0, rig.telemetry.value("leftMeasuredVelocityTicksPerSec"));
        assertNumber(444.0, rig.telemetry.value("rightMeasuredVelocityTicksPerSec"));
        assertNumber(abortedElapsedSec, rig.telemetry.value("elapsedSec"));
        assertMinimalTelemetry(rig.telemetry);
        rig.tester.stop();
    }

    private static void assertMinimalTelemetry(RecordingTelemetry telemetry) {
        for (String key : telemetry.values.keySet()) {
            String lower = key.toLowerCase(Locale.ROOT);
            assertFalse("telemetry must not report feed facts: " + key, lower.contains("feed"));
            assertFalse("telemetry must not report object facts: " + key, lower.contains("object"));
            assertFalse("telemetry must not report scoring facts: " + key, lower.contains("scor"));
            assertFalse("telemetry must not claim PASS: " + key, lower.contains("pass"));
        }
        for (String line : telemetry.lines) {
            String lower = line.toLowerCase(Locale.ROOT);
            assertFalse("telemetry must not report feed facts: " + line, lower.contains("feed"));
            assertFalse("telemetry must not report object facts: " + line, lower.contains("object"));
            assertFalse("telemetry must not report scoring facts: " + line, lower.contains("scor"));
            assertFalse("telemetry must not claim PASS: " + line, lower.contains("pass"));
        }
    }

    private static void assertMissingMethod(String name) {
        for (Method method : ReferenceFlywheelSpinUpCriteria.class.getDeclaredMethods()) {
            if (method.getName().equals(name)) {
                fail("Expected method " + name + " to be absent");
            }
        }
    }

    private static void assertNumber(double expected, Object actual) {
        assertNotNull(actual);
        assertEquals(expected, number(actual), EPSILON);
    }

    private static double number(Object value) {
        return ((Number) value).doubleValue();
    }

    private static TesterContext context(FtcTestHardware hardware,
                                         Telemetry telemetry,
                                         Gamepad gamepad1,
                                         LoopClock clock) {
        clock.reset(0.0);
        return new TesterContext(hardware, telemetry, gamepad1, new Gamepad(), clock);
    }

    private static Telemetry telemetry() {
        return new RecordingTelemetry().telemetry;
    }

    private static ReferenceLauncherMechanism.Config testConfig() {
        ReferenceLauncherMechanism.Config config =
                ReferenceLauncherMechanism.Config.defaults();
        config.maximumVelocityTicksPerSec = 2000.0;
        config.velocityToleranceTicksPerSec = 50.0;
        config.launchVelocityTicksPerSec = 1000.0;
        config.spinUpTimeoutSec = 1.0;
        return config;
    }

    private static ReferenceFlywheelSpinUpCriteria reviewedCriteria() {
        ReferenceFlywheelSpinUpCriteria criteria = ReferenceFlywheelSpinUpCriteria.current();
        criteria.reviewedForMotion = true;
        criteria.targetVelocityTicksPerSec = 1000.0;
        criteria.maximumPoweredRunSec = 1.0;
        return criteria;
    }

    private static final class Rig {
        private final ReferenceLauncherMechanism.Config config = testConfig();
        private final ReferenceFlywheelSpinUpCriteria criteria = reviewedCriteria();
        private final FtcTestHardware hardware = new FtcTestHardware();
        private final MotorProbe left = hardware.addMotor(config.leftFlywheelName);
        private final MotorProbe right = hardware.addMotor(config.rightFlywheelName);
        private final Gamepad gamepad1 = new Gamepad();
        private final LoopClock clock = new LoopClock();
        private final RecordingTelemetry telemetry = new RecordingTelemetry();
        private final ReferenceFlywheelSpinUpExperiment tester;
        private double nowSec;

        private Rig() {
            hardware.addCrServo(config.transferName);
            hardware.addServo(config.releaseServoName);
            hardware.addDigitalInput(config.objectSensorName);
            clock.reset(0.0);
            tester = new ReferenceFlywheelSpinUpExperiment(config, criteria);
            tester.init(new TesterContext(
                    hardware,
                    telemetry.telemetry,
                    gamepad1,
                    new Gamepad(),
                    clock));
        }

        private void startActive() {
            initCycle(0.02);
            tester.start();
            activeCycle(0.02);
        }

        private void initCycle(double dtSec) {
            advance(dtSec);
            tester.initLoop(clock.dtSec());
        }

        private void activeCycle(double dtSec) {
            advance(dtSec);
            tester.loop(clock.dtSec());
        }

        private void advance(double dtSec) {
            nowSec += dtSec;
            clock.update(nowSec);
        }
    }

    private static final class RecordingTelemetry {
        private final Map<String, Object> values = new LinkedHashMap<String, Object>();
        private final List<String> lines = new ArrayList<String>();
        private final Telemetry telemetry = (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                (proxy, method, args) -> invoke(method, args));

        private Object value(String key) {
            return values.get(key);
        }

        private boolean has(String key) {
            return values.containsKey(key);
        }

        private boolean hasLineContaining(String text) {
            for (String line : lines) {
                if (line.contains(text)) {
                    return true;
                }
            }
            return false;
        }

        private Object invoke(Method method, Object[] args) {
            String name = method.getName();
            if (method.getDeclaringClass() == Object.class) {
                if ("equals".equals(name)) return telemetry == args[0];
                if ("hashCode".equals(name)) return System.identityHashCode(telemetry);
                if ("toString".equals(name)) return "RecordingTelemetry";
            }
            if ("clearAll".equals(name) || "clear".equals(name)) {
                values.clear();
                lines.clear();
                return null;
            }
            if ("addData".equals(name)) {
                values.put((String) args[0], args[1]);
                return null;
            }
            if ("addLine".equals(name)) {
                lines.add(args == null || args.length == 0 ? "" : String.valueOf(args[0]));
                return null;
            }
            return defaultValue(method.getReturnType());
        }
    }

    private static Object defaultValue(Class<?> type) {
        if (!type.isPrimitive()) return null;
        if (type == boolean.class) return false;
        if (type == byte.class) return (byte) 0;
        if (type == short.class) return (short) 0;
        if (type == int.class) return 0;
        if (type == long.class) return 0L;
        if (type == float.class) return 0.0f;
        if (type == double.class) return 0.0;
        if (type == char.class) return '\0';
        return null;
    }
}
