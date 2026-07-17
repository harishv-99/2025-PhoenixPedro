package edu.ftcphoenix.fw.ftc;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.PositionPlant;
import edu.ftcphoenix.fw.core.control.ScalarRegulator;
import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the direct staged feedback API and FTC external-encoder counter continuity. */
public final class FtcExternalEncoderFeedbackTest {

    private static final double EPSILON = 1e-9;
    private static final ScalarRegulator ZERO_REGULATOR =
            (setpoint, measurement, clock) -> 0.0;

    @Test
    public void regulatedMotorStagesExposeDirectParallelFeedbackAnswers() throws Exception {
        Class<?> velocity = FtcActuators.MotorRegulatedVelocityFeedbackStep.class;
        Class<?> velocityNext = FtcActuators.MotorRegulatedVelocityRegulatorStep.class;
        assertReturns(velocity, "internalEncoder", velocityNext);
        assertReturns(velocity, "internalEncoder", velocityNext, String.class);
        assertReturns(velocity, "averageInternalEncoders", velocityNext);
        assertReturns(velocity, "externalEncoder", velocityNext, String.class);
        assertReturns(velocity, "externalEncoder", velocityNext, String.class, Direction.class);
        assertReturns(velocity, "nativeFeedback", velocityNext, ScalarSource.class);

        Class<?> position = FtcActuators.MotorRegulatedPositionFeedbackStep.class;
        Class<?> positionNext = FtcActuators.MotorRegulatedPositionRegulatorStep.class;
        assertReturns(position, "internalEncoder", positionNext);
        assertReturns(position, "internalEncoder", positionNext, String.class);
        assertReturns(position, "averageInternalEncoders", positionNext);
        assertReturns(position, "externalEncoder", positionNext, String.class);
        assertReturns(position, "externalEncoder", positionNext, String.class, Direction.class);
        assertReturns(position, "nativeFeedback", positionNext, ScalarSource.class);

        Class<?> crPosition = FtcActuators.CrServoRegulatedPositionFeedbackStep.class;
        Class<?> crPositionNext = FtcActuators.CrServoRegulatedPositionRegulatorStep.class;
        assertReturns(crPosition, "externalEncoder", crPositionNext, String.class);
        assertReturns(crPosition, "externalEncoder", crPositionNext, String.class, Direction.class);
        assertReturns(crPosition, "nativeFeedback", crPositionNext, ScalarSource.class);
        Set<String> actualCrAnswers = new TreeSet<>();
        for (Method method : crPosition.getDeclaredMethods()) {
            actualCrAnswers.add(methodSignature(method));
        }
        assertEquals(new TreeSet<>(Arrays.asList(
                        "externalEncoder(String)",
                        "externalEncoder(String,Direction)",
                        "nativeFeedback(ScalarSource)")),
                actualCrAnswers);

        for (Class<?> nested : FtcActuators.class.getDeclaredClasses()) {
            assertNotEquals("PositionFeedback", nested.getSimpleName());
            assertNotEquals("VelocityFeedback", nested.getSimpleName());
        }
    }

    @Test
    public void directInternalAnswersKeepActionableGroupErrors() {
        HardwareMap hardwareMap = new TestHardwareMap();

        assertIllegalStateContains(() -> FtcActuators.plant(hardwareMap)
                        .motor("left", Direction.FORWARD)
                        .andMotor("right", Direction.FORWARD)
                        .velocity()
                        .regulated()
                        .internalEncoder(),
                "internalEncoder() is ambiguous",
                "averageInternalEncoders()",
                "externalEncoder(...)");

        assertIllegalStateContains(() -> FtcActuators.plant(hardwareMap)
                        .motor("left", Direction.FORWARD)
                        .andMotor("right", Direction.FORWARD)
                        .position()
                        .regulated()
                        .internalEncoder("missing"),
                "internalEncoder(\"missing\") does not match any selected motor");
    }

    @Test
    public void regulatedVelocityInternalEncoderDispatchesOnlyToSdkVelocity() {
        HardwareMap hardwareMap = new TestHardwareMap();
        MotorProbe flywheel = new MotorProbe(125, 2468.5);
        hardwareMap.put("flywheel", flywheel.motor());

        Plant plant = FtcActuators.plant(hardwareMap)
                .motor("flywheel", Direction.FORWARD)
                .velocity()
                .regulated()
                .internalEncoder()
                .regulator(ZERO_REGULATOR)
                .unbounded()
                .nativeUnits()
                .targetedByDefaultWritable(0.0)
                .build();

        ManualLoopClock clock = new ManualLoopClock();
        plant.update(clock.clock());

        assertEquals(2468.5, plant.getMeasurement(), EPSILON);
        assertEquals(1, flywheel.velocityReadCount);
        assertEquals(0, flywheel.positionReadCount);

        plant.update(clock.clock());
        assertEquals(1, flywheel.velocityReadCount);
        assertEquals(0, flywheel.positionReadCount);
    }

    @Test
    public void regulatedVelocityExternalEncoderDerivesRateOnlyFromPosition() {
        HardwareMap hardwareMap = new TestHardwareMap();
        MotorProbe flywheel = new MotorProbe(700, 7000.0);
        MotorProbe boreEncoder = new MotorProbe(100, 9999.0);
        hardwareMap.put("flywheel", flywheel.motor());
        hardwareMap.put("bore", boreEncoder.motor());

        Plant plant = FtcActuators.plant(hardwareMap)
                .motor("flywheel", Direction.FORWARD)
                .velocity()
                .regulated()
                .externalEncoder("bore")
                .regulator(ZERO_REGULATOR)
                .unbounded()
                .nativeUnits()
                .targetedByDefaultWritable(0.0)
                .build();

        ManualLoopClock clock = new ManualLoopClock();
        plant.update(clock.clock());
        assertEquals(0.0, plant.getMeasurement(), EPSILON);

        boreEncoder.positionTicks = 110;
        clock.nextCycle(0.5);
        plant.update(clock.clock());

        assertEquals(20.0, plant.getMeasurement(), EPSILON);
        assertEquals(2, boreEncoder.positionReadCount);
        assertEquals(0, boreEncoder.velocityReadCount);
        assertEquals(0, flywheel.positionReadCount);
        assertEquals(0, flywheel.velocityReadCount);

        boreEncoder.positionTicks = 500;
        plant.reset();
        plant.update(clock.clock());
        assertEquals(0.0, plant.getMeasurement(), EPSILON);
        assertEquals(3, boreEncoder.positionReadCount);
        assertEquals(0, boreEncoder.velocityReadCount);
    }

    @Test
    public void regulatedMotorPositionExternalEncoderReadsRawPositionOnly() {
        HardwareMap hardwareMap = new TestHardwareMap();
        MotorProbe liftMotor = new MotorProbe(15, 1500.0);
        MotorProbe liftEncoder = new MotorProbe(321, 9876.0);
        hardwareMap.put("lift", liftMotor.motor());
        hardwareMap.put("liftEncoder", liftEncoder.motor());

        PositionPlant plant = FtcActuators.plant(hardwareMap)
                .motor("lift", Direction.FORWARD)
                .position()
                .regulated()
                .externalEncoder("liftEncoder")
                .regulator(ZERO_REGULATOR)
                .linear()
                .unbounded()
                .nativeUnits()
                .alreadyReferenced()
                .targetedByDefaultWritable(0.0)
                .build();

        plant.update(new ManualLoopClock().clock());

        assertEquals(321.0, plant.getMeasurement(), EPSILON);
        assertEquals(1, liftEncoder.positionReadCount);
        assertEquals(0, liftEncoder.velocityReadCount);
        assertEquals(0, liftMotor.positionReadCount);
        assertEquals(0, liftMotor.velocityReadCount);
    }

    @Test
    public void regulatedCrServoPositionExternalEncoderReadsRawPositionOnly() {
        HardwareMap hardwareMap = new TestHardwareMap();
        CrServoProbe turret = new CrServoProbe();
        MotorProbe turretEncoder = new MotorProbe(-42, 4321.0);
        hardwareMap.put("turret", turret.servo());
        hardwareMap.put("turretEncoder", turretEncoder.motor());

        PositionPlant plant = FtcActuators.plant(hardwareMap)
                .crServo("turret", Direction.FORWARD)
                .position()
                .regulated()
                .externalEncoder("turretEncoder")
                .regulator(ZERO_REGULATOR)
                .linear()
                .unbounded()
                .nativeUnits()
                .alreadyReferenced()
                .targetedByDefaultWritable(0.0)
                .build();

        plant.update(new ManualLoopClock().clock());

        assertEquals(-42.0, plant.getMeasurement(), EPSILON);
        assertEquals(1, turretEncoder.positionReadCount);
        assertEquals(0, turretEncoder.velocityReadCount);
        assertEquals(1, turret.powerWriteCount);
    }

    @Test
    public void regulatedVelocityDefensivelyRejectsMissingOrNullFeedbackAnswers() {
        HardwareMap hardwareMap = new HardwareMap(null, null);
        FtcActuators.MotorRegulatedVelocityFeedbackStep feedbackStep =
                FtcActuators.plant(hardwareMap)
                        .motor("flywheel", Direction.FORWARD)
                        .velocity()
                        .regulated();

        FtcActuators.MotorRegulatedVelocityRegulatorStep bypassedFeedback =
                (FtcActuators.MotorRegulatedVelocityRegulatorStep) feedbackStep;
        FtcActuators.PlantBuildStep missingFeedbackBuild = bypassedFeedback
                .regulator(ZERO_REGULATOR)
                .unbounded()
                .nativeUnits()
                .targetedByDefaultWritable(0.0);
        assertIllegalStateContains(missingFeedbackBuild::build,
                "Regulated motor velocity requires a feedback answer",
                "internalEncoder()",
                "externalEncoder(...)",
                "nativeFeedback(...)");

        assertThrowsContains(NullPointerException.class,
                () -> feedbackStep.nativeFeedback(null),
                "source");
        assertThrowsContains(NullPointerException.class,
                () -> feedbackStep.internalEncoder((String) null),
                "motorName");
        assertThrowsContains(IllegalArgumentException.class,
                () -> feedbackStep.externalEncoder(null),
                "name is required");
    }

    @Test
    public void continuousPositionCrossesPositiveSigned32RolloverOncePerCycle() {
        EncoderProbe encoder = new EncoderProbe(Integer.MAX_VALUE - 1);
        ScalarSource position = FtcSensors.continuousMotorPositionTicks(
                encoder.motor(), Direction.FORWARD);
        ManualLoopClock clock = new ManualLoopClock();

        assertEquals(2147483646.0, position.getAsDouble(clock.clock()), EPSILON);
        encoder.rawTicks = Integer.MIN_VALUE + 1;
        assertEquals(2147483646.0, position.getAsDouble(clock.clock()), EPSILON);
        assertEquals(1, encoder.readCount);

        clock.nextCycle(0.02);
        assertEquals(2147483649.0, position.getAsDouble(clock.clock()), EPSILON);
        assertEquals(2, encoder.readCount);

        encoder.rawTicks = Integer.MIN_VALUE + 5;
        clock.nextCycle(0.02);
        assertEquals(2147483653.0, position.getAsDouble(clock.clock()), EPSILON);
    }

    @Test
    public void continuousPositionCrossesNegativeRolloverAndAppliesDirectionAfterUnwrap() {
        EncoderProbe encoder = new EncoderProbe(Integer.MIN_VALUE + 1);
        ScalarSource position = FtcSensors.continuousMotorPositionTicks(
                encoder.motor(), Direction.REVERSE);
        ManualLoopClock clock = new ManualLoopClock();

        assertEquals(2147483647.0, position.getAsDouble(clock.clock()), EPSILON);

        encoder.rawTicks = Integer.MAX_VALUE - 1;
        clock.nextCycle(0.02);
        assertEquals(2147483650.0, position.getAsDouble(clock.clock()), EPSILON);
    }

    @Test
    public void continuousPositionResetDiscardsOnlySoftwareContinuityHistory() {
        EncoderProbe encoder = new EncoderProbe(100);
        ScalarSource position = FtcSensors.continuousMotorPositionTicks(
                encoder.motor(), Direction.FORWARD);
        ManualLoopClock clock = new ManualLoopClock();

        assertEquals(100.0, position.getAsDouble(clock.clock()), EPSILON);
        encoder.rawTicks = 120;
        clock.nextCycle(0.02);
        assertEquals(120.0, position.getAsDouble(clock.clock()), EPSILON);

        encoder.rawTicks = -30;
        position.reset();
        assertEquals(-30.0, position.getAsDouble(clock.clock()), EPSILON);
        assertEquals(3, encoder.readCount);
    }

    @Test
    public void derivedVelocityRemainsFiniteAcrossRolloverAndRebaselinesOnReset() {
        EncoderProbe encoder = new EncoderProbe(Integer.MAX_VALUE - 2);
        ScalarSource velocity = FtcSensors.continuousMotorPositionTicks(
                encoder.motor(), Direction.FORWARD).ratePerSecond();
        ManualLoopClock clock = new ManualLoopClock();

        assertEquals(0.0, velocity.getAsDouble(clock.clock()), EPSILON);

        encoder.rawTicks = Integer.MIN_VALUE + 1;
        clock.nextCycle(0.25);
        assertEquals(16.0, velocity.getAsDouble(clock.clock()), EPSILON);

        encoder.rawTicks = 1000;
        velocity.reset();
        assertEquals(0.0, velocity.getAsDouble(clock.clock()), EPSILON);

        encoder.rawTicks = 1010;
        clock.nextCycle(0.5);
        assertEquals(20.0, velocity.getAsDouble(clock.clock()), EPSILON);
    }

    private static void assertReturns(Class<?> owner,
                                      String methodName,
                                      Class<?> returnType,
                                      Class<?>... parameterTypes) throws Exception {
        Method method = owner.getMethod(methodName, parameterTypes);
        assertEquals(returnType, method.getReturnType());
    }

    private static String methodSignature(Method method) {
        StringBuilder signature = new StringBuilder(method.getName()).append('(');
        Class<?>[] parameterTypes = method.getParameterTypes();
        for (int i = 0; i < parameterTypes.length; i++) {
            if (i > 0) signature.append(',');
            signature.append(parameterTypes[i].getSimpleName());
        }
        return signature.append(')').toString();
    }

    private static void assertIllegalStateContains(Runnable action, String... fragments) {
        try {
            action.run();
            fail("Expected IllegalStateException");
        } catch (IllegalStateException expected) {
            for (String fragment : fragments) {
                assertTrue("Expected message to contain: " + fragment,
                        expected.getMessage().contains(fragment));
            }
        }
    }

    private static void assertThrowsContains(Class<? extends Throwable> expectedType,
                                             Runnable action,
                                             String fragment) {
        try {
            action.run();
        } catch (Throwable actual) {
            assertTrue("Expected " + expectedType.getSimpleName() + " but got " + actual,
                    expectedType.isInstance(actual));
            assertTrue("Expected message to contain: " + fragment,
                    actual.getMessage() != null && actual.getMessage().contains(fragment));
            return;
        }
        fail("Expected " + expectedType.getSimpleName());
    }

    /** In-memory HardwareMap that avoids Android-only SDK discovery during local JVM tests. */
    private static final class TestHardwareMap extends HardwareMap {
        private final Map<String, HardwareDevice> devices = new HashMap<>();

        private TestHardwareMap() {
            super(null, null);
        }

        @Override
        public void put(String name, HardwareDevice device) {
            devices.put(name, device);
        }

        @Override
        public <T> T get(Class<? extends T> type, String name) {
            HardwareDevice device = devices.get(name);
            if (device == null) {
                throw new IllegalArgumentException("No test device named " + name);
            }
            if (!type.isInstance(device)) {
                throw new IllegalArgumentException(name + " is not a " + type.getSimpleName());
            }
            return type.cast(device);
        }
    }

    /** Dynamic SDK motor with independent position/velocity values and read counters. */
    private static final class MotorProbe {
        private int positionTicks;
        private double velocityTicksPerSec;
        private int positionReadCount;
        private int velocityReadCount;
        private int powerWriteCount;
        private double power;
        private DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
        private DcMotor.RunMode runMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        private final DcMotorEx motor;

        private MotorProbe(int positionTicks, double velocityTicksPerSec) {
            this.positionTicks = positionTicks;
            this.velocityTicksPerSec = velocityTicksPerSec;
            motor = (DcMotorEx) Proxy.newProxyInstance(
                    DcMotorEx.class.getClassLoader(),
                    new Class<?>[]{DcMotorEx.class},
                    (proxy, method, args) -> invoke(proxy, method, args));
        }

        private DcMotorEx motor() {
            return motor;
        }

        private Object invoke(Object proxy, Method method, Object[] args) {
            String name = method.getName();
            if (method.getDeclaringClass() == Object.class) {
                return objectMethod(proxy, method, args, "MotorProbe");
            }
            if ("getCurrentPosition".equals(name)) {
                positionReadCount++;
                return positionTicks;
            }
            if ("getVelocity".equals(name)) {
                velocityReadCount++;
                return velocityTicksPerSec;
            }
            if ("setPower".equals(name)) {
                powerWriteCount++;
                power = (double) args[0];
                return null;
            }
            if ("getPower".equals(name)) return power;
            if ("setDirection".equals(name)) {
                direction = (DcMotorSimple.Direction) args[0];
                return null;
            }
            if ("getDirection".equals(name)) return direction;
            if ("setMode".equals(name)) {
                runMode = (DcMotor.RunMode) args[0];
                return null;
            }
            if ("getMode".equals(name)) return runMode;
            if ("getManufacturer".equals(name)) return HardwareDevice.Manufacturer.Other;
            if ("getDeviceName".equals(name)) return "MotorProbe";
            if ("getConnectionInfo".equals(name)) return "test";
            if ("getVersion".equals(name)) return 1;
            return defaultValue(method.getReturnType());
        }
    }

    /** Dynamic SDK CR servo used to verify regulated position dispatch. */
    private static final class CrServoProbe {
        private int powerWriteCount;
        private double power;
        private DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
        private final CRServo servo;

        private CrServoProbe() {
            servo = (CRServo) Proxy.newProxyInstance(
                    CRServo.class.getClassLoader(),
                    new Class<?>[]{CRServo.class},
                    (proxy, method, args) -> invoke(proxy, method, args));
        }

        private CRServo servo() {
            return servo;
        }

        private Object invoke(Object proxy, Method method, Object[] args) {
            String name = method.getName();
            if (method.getDeclaringClass() == Object.class) {
                return objectMethod(proxy, method, args, "CrServoProbe");
            }
            if ("setPower".equals(name)) {
                powerWriteCount++;
                power = (double) args[0];
                return null;
            }
            if ("getPower".equals(name)) return power;
            if ("setDirection".equals(name)) {
                direction = (DcMotorSimple.Direction) args[0];
                return null;
            }
            if ("getDirection".equals(name)) return direction;
            if ("getManufacturer".equals(name)) return HardwareDevice.Manufacturer.Other;
            if ("getDeviceName".equals(name)) return "CrServoProbe";
            if ("getConnectionInfo".equals(name)) return "test";
            if ("getVersion".equals(name)) return 1;
            return defaultValue(method.getReturnType());
        }
    }

    /** Minimal dynamic FTC motor used to count and control position reads. */
    private static final class EncoderProbe {
        private int rawTicks;
        private int readCount;

        private EncoderProbe(int rawTicks) {
            this.rawTicks = rawTicks;
        }

        private DcMotor motor() {
            return (DcMotor) Proxy.newProxyInstance(
                    DcMotor.class.getClassLoader(),
                    new Class<?>[]{DcMotor.class},
                    (proxy, method, args) -> {
                        if ("getCurrentPosition".equals(method.getName())) {
                            readCount++;
                            return rawTicks;
                        }
                        return defaultValue(method.getReturnType());
                    });
        }
    }

    private static Object objectMethod(Object proxy,
                                       Method method,
                                       Object[] args,
                                       String label) {
        if ("equals".equals(method.getName())) return proxy == args[0];
        if ("hashCode".equals(method.getName())) return System.identityHashCode(proxy);
        if ("toString".equals(method.getName())) return label;
        return null;
    }

    private static Object defaultValue(Class<?> returnType) {
        if (!returnType.isPrimitive()) return null;
        if (returnType == boolean.class) return false;
        if (returnType == byte.class) return (byte) 0;
        if (returnType == short.class) return (short) 0;
        if (returnType == int.class) return 0;
        if (returnType == long.class) return 0L;
        if (returnType == float.class) return 0.0f;
        if (returnType == double.class) return 0.0;
        if (returnType == char.class) return '\0';
        return null;
    }
}
