package edu.ftcphoenix.fw.ftc;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.HashMap;
import java.util.Map;

import edu.ftcphoenix.fw.actuation.Plants;
import edu.ftcphoenix.fw.actuation.PositionPlant;
import edu.ftcphoenix.fw.core.control.ScalarRegulator;
import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies finite position-reference answers at the FTC construction boundary. */
public final class FtcPositionReferenceValidationTest {

    private static final double EPSILON = 1.0e-12;

    @Test
    public void deviceManagedAnswersRejectEveryNonFiniteValueBeforeSdkEffects() {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        hardwareMap.addMotor("lift", 25);
        Plants.PositionCoordinateReferenceStep<Plants.SymmetricOutputPowerPolicyStep<PositionPlant>>
                reference =
                tunedDeviceManagedReferenceStep(hardwareMap, "lift", 10.0);

        for (double invalid : nonFiniteValues()) {
            assertExactIllegalArgument(
                    "FtcActuators.plantPositionMapsToNative(...): plantPosition "
                            + "must be finite in plant units, got " + invalid,
                    () -> reference.plantPositionMapsToNative(invalid, Double.NaN));
            hardwareMap.effects.assertNone();

            assertExactIllegalArgument(
                    "FtcActuators.plantPositionMapsToNative(...): nativePosition "
                            + "must be finite in native units, got " + invalid,
                    () -> reference.plantPositionMapsToNative(0.0, invalid));
            hardwareMap.effects.assertNone();

            assertExactIllegalArgument(
                    "FtcActuators.assumeCurrentPositionIs(...): plantPosition "
                            + "must be finite in plant units, got " + invalid,
                    () -> reference.assumeCurrentPositionIs(invalid));
            hardwareMap.effects.assertNone();
        }
    }

    @Test
    public void finiteAnswersPreserveSignedZerosAndFiniteExtremes() throws Exception {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        hardwareMap.addMotor("lift", 0);

        Plants.PositionToleranceStep<Plants.SymmetricOutputPowerPolicyStep<PositionPlant>>
                staticAnswer = boundedDeviceManagedReferenceStep(
                hardwareMap, "lift", 1.0)
                .plantPositionMapsToNative(-0.0, +0.0);
        assertRawDoubleEquals(-0.0, privateDouble(staticAnswer, "plantReference"));
        assertRawDoubleEquals(+0.0, privateDouble(staticAnswer, "nativeReference"));

        Plants.PositionToleranceStep<Plants.SymmetricOutputPowerPolicyStep<PositionPlant>>
                extremePlantAnswer =
                deviceManagedReferenceStep(hardwareMap, "lift", 1.0)
                        .plantPositionMapsToNative(Double.MAX_VALUE, 0.0);
        assertRawDoubleEquals(Double.MAX_VALUE,
                privateDouble(extremePlantAnswer, "plantReference"));
        assertRawDoubleEquals(0.0,
                privateDouble(extremePlantAnswer, "nativeReference"));

        Plants.PositionToleranceStep<Plants.SymmetricOutputPowerPolicyStep<PositionPlant>>
                extremeNativeAnswer =
                deviceManagedReferenceStep(hardwareMap, "lift", 1.0)
                        .plantPositionMapsToNative(0.0, -Double.MAX_VALUE);
        assertRawDoubleEquals(0.0,
                privateDouble(extremeNativeAnswer, "plantReference"));
        assertRawDoubleEquals(-Double.MAX_VALUE,
                privateDouble(extremeNativeAnswer, "nativeReference"));

        Plants.PositionToleranceStep<Plants.SymmetricOutputPowerPolicyStep<PositionPlant>>
                assumedAnswer = boundedDeviceManagedReferenceStep(
                hardwareMap, "lift", 1.0)
                .assumeCurrentPositionIs(-0.0);
        assertRawDoubleEquals(-0.0, privateDouble(assumedAnswer, "assumePlantPosition"));
        hardwareMap.effects.assertNone();
    }

    @Test
    public void invalidRetainedAliasCannotReplaceEarlierAcceptedReferenceMode() {
        TestHardwareMap staticMap = new TestHardwareMap();
        MotorProbe staticMotor = staticMap.addMotor("arm", 130);
        Plants.PositionCoordinateReferenceStep<Plants.SymmetricOutputPowerPolicyStep<PositionPlant>>
                retainedStatic =
                deviceManagedReferenceStep(staticMap, "arm", 10.0);
        Plants.PositionToleranceStep<Plants.SymmetricOutputPowerPolicyStep<PositionPlant>>
                acceptedStatic =
                retainedStatic.plantPositionMapsToNative(2.0, 100.0);

        assertExactIllegalArgument(
                "FtcActuators.assumeCurrentPositionIs(...): plantPosition "
                        + "must be finite in plant units, got NaN",
                () -> retainedStatic.assumeCurrentPositionIs(Double.NaN));
        assertExactIllegalArgument(
                "FtcActuators.plantPositionMapsToNative(...): nativePosition "
                        + "must be finite in native units, got Infinity",
                () -> retainedStatic.plantPositionMapsToNative(
                        3.0, Double.POSITIVE_INFINITY));
        staticMap.effects.assertNone();

        PositionPlant staticPlant = acceptedStatic
                .positionTolerance(0.0)
                .targetFromNewCommand(5.0)
                .build();
        staticPlant.update(new ManualLoopClock().clock());
        assertEquals(5.0, staticPlant.getMeasurement(), EPSILON);
        assertEquals(130, staticMotor.targetPosition);

        TestHardwareMap assumeMap = new TestHardwareMap();
        MotorProbe assumeMotor = assumeMap.addMotor("wrist", 200);
        Plants.PositionCoordinateReferenceStep<Plants.SymmetricOutputPowerPolicyStep<PositionPlant>>
                retainedAssume =
                deviceManagedReferenceStep(assumeMap, "wrist", 10.0);
        Plants.PositionToleranceStep<Plants.SymmetricOutputPowerPolicyStep<PositionPlant>>
                acceptedAssume =
                retainedAssume.assumeCurrentPositionIs(7.0);

        assertExactIllegalArgument(
                "FtcActuators.plantPositionMapsToNative(...): plantPosition "
                        + "must be finite in plant units, got Infinity",
                () -> retainedAssume.plantPositionMapsToNative(
                        Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY));
        assumeMap.effects.assertNone();

        PositionPlant assumedPlant = acceptedAssume
                .positionTolerance(0.0)
                .targetFromNewCommand(8.0)
                .build();
        assumedPlant.update(new ManualLoopClock().clock());
        assertEquals(7.0, assumedPlant.getMeasurement(), EPSILON);
        assertEquals(210, assumeMotor.targetPosition);
    }

    @Test
    public void regulatedMotorRejectsWithoutEffectsAfterFeedbackSelectionAndAllowsRetry() {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        hardwareMap.addMotor("arm", 120);
        ScalarRegulator regulator = recordingZeroRegulator(hardwareMap.effects);
        Plants.PositionCoordinateReferenceStep<Plants.PositionControlStep> reference =
                FtcActuators.plant(hardwareMap)
                .motor("arm", Direction.FORWARD)
                .position()
                .regulated()
                .internalEncoder()
                .nonPeriodic()
                .unbounded()
                .scaleToNative(10.0);
        int[] afterFeedbackSelection = hardwareMap.effects.snapshot();

        assertExactIllegalArgument(
                "FtcActuators.plantPositionMapsToNative(...): nativePosition "
                        + "must be finite in native units, got -Infinity",
                () -> reference.plantPositionMapsToNative(2.0, Double.NEGATIVE_INFINITY));
        assertArrayEquals(afterFeedbackSelection, hardwareMap.effects.snapshot());

        PositionPlant plant = reference
                .plantPositionMapsToNative(2.0, 100.0)
                .positionTolerance(0.0)
                .controlFromCustomRegulator(regulator)
                .targetFromNewCommand(4.0)
                .build();
        plant.update(new ManualLoopClock().clock());

        assertEquals(4.0, plant.getMeasurement(), EPSILON);
        assertEquals(1, hardwareMap.effects.regulatorCalls);
    }

    @Test
    public void regulatedCrServoRejectsWithoutEffectsAfterFeedbackSelectionAndAllowsRetry() {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        hardwareMap.addCrServo("turret");
        hardwareMap.addMotor("turretEncoder", 75);
        ScalarRegulator regulator = recordingZeroRegulator(hardwareMap.effects);
        Plants.PositionCoordinateReferenceStep<Plants.PositionControlStep> reference =
                FtcActuators.plant(hardwareMap)
                .crServo("turret", Direction.FORWARD)
                .position()
                .regulated()
                .externalEncoder("turretEncoder")
                .nonPeriodic()
                .unbounded()
                .scaleToNative(5.0);
        int[] afterFeedbackSelection = hardwareMap.effects.snapshot();

        assertExactIllegalArgument(
                "FtcActuators.assumeCurrentPositionIs(...): plantPosition "
                        + "must be finite in plant units, got -Infinity",
                () -> reference.assumeCurrentPositionIs(Double.NEGATIVE_INFINITY));
        assertArrayEquals(afterFeedbackSelection, hardwareMap.effects.snapshot());

        PositionPlant plant = reference
                .assumeCurrentPositionIs(7.0)
                .positionTolerance(0.0)
                .controlFromCustomRegulator(regulator)
                .targetFromNewCommand(8.0)
                .build();
        plant.update(new ManualLoopClock().clock());

        assertEquals(7.0, plant.getMeasurement(), EPSILON);
        assertEquals(1, hardwareMap.effects.regulatorCalls);
    }

    private static Plants.PositionCoordinateReferenceStep<
            Plants.SymmetricOutputPowerPolicyStep<PositionPlant>> tunedDeviceManagedReferenceStep(
            TestHardwareMap hardwareMap,
            String motorName,
            double nativePerPlantUnit) {
        return FtcActuators.plant(hardwareMap)
                .motor(motorName, Direction.FORWARD)
                .position()
                .deviceManagedWithOverrides()
                .outerPositionP(1.25)
                .innerVelocityPidf(2.0, 0.5, 0.25, 0.0)
                .devicePositionToleranceTicks(3)
                .doneOverrides()
                .nonPeriodic()
                .unbounded()
                .scaleToNative(nativePerPlantUnit);
    }

    private static Plants.PositionCoordinateReferenceStep<
            Plants.SymmetricOutputPowerPolicyStep<PositionPlant>> deviceManagedReferenceStep(
            TestHardwareMap hardwareMap,
            String motorName,
            double nativePerPlantUnit) {
        return FtcActuators.plant(hardwareMap)
                .motor(motorName, Direction.FORWARD)
                .position()
                .deviceManaged()
                .nonPeriodic()
                .unbounded()
                .scaleToNative(nativePerPlantUnit);
    }

    private static Plants.PositionCoordinateReferenceStep<
            Plants.SymmetricOutputPowerPolicyStep<PositionPlant>> boundedDeviceManagedReferenceStep(
            TestHardwareMap hardwareMap,
            String motorName,
            double nativePerPlantUnit) {
        return boundedDeviceManagedReferenceStep(
                hardwareMap, motorName, -1.0, 1.0, nativePerPlantUnit);
    }

    private static Plants.PositionCoordinateReferenceStep<
            Plants.SymmetricOutputPowerPolicyStep<PositionPlant>> boundedDeviceManagedReferenceStep(
            TestHardwareMap hardwareMap,
            String motorName,
            double plantMin,
            double plantMax,
            double nativePerPlantUnit) {
        return FtcActuators.plant(hardwareMap)
                .motor(motorName, Direction.FORWARD)
                .position()
                .deviceManaged()
                .nonPeriodic()
                .bounded(plantMin, plantMax)
                .scaleToNative(nativePerPlantUnit);
    }

    private static ScalarRegulator recordingZeroRegulator(Effects effects) {
        return (setpoint, measurement, clock) -> {
            effects.regulatorCalls++;
            return 0.0;
        };
    }

    private static double[] nonFiniteValues() {
        return new double[]{
                Double.NaN,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY
        };
    }

    private static void assertExactIllegalArgument(String expectedMessage, Runnable action) {
        try {
            action.run();
        } catch (RuntimeException failure) {
            assertTrue("expected IllegalArgumentException, got " + failure,
                    failure instanceof IllegalArgumentException);
            assertEquals(expectedMessage, failure.getMessage());
            return;
        }
        fail("Expected IllegalArgumentException: " + expectedMessage);
    }

    private static double privateDouble(Object owner, String fieldName) throws Exception {
        Object candidate = owner;
        while (candidate != null) {
            Class<?> type = candidate.getClass();
            while (type != null) {
                try {
                    Field field = type.getDeclaredField(fieldName);
                    field.setAccessible(true);
                    return field.getDouble(candidate);
                } catch (NoSuchFieldException ignored) {
                    type = type.getSuperclass();
                }
            }

            Field enclosing = candidate.getClass().getDeclaredField("this$0");
            enclosing.setAccessible(true);
            candidate = enclosing.get(candidate);
        }
        fail("No field named " + fieldName + " on " + owner.getClass());
        return Double.NaN;
    }

    private static void assertRawDoubleEquals(double expected, double actual) {
        assertEquals(Double.doubleToRawLongBits(expected), Double.doubleToRawLongBits(actual));
    }

    /** In-memory HardwareMap whose relevant SDK interactions are observable. */
    private static final class TestHardwareMap extends HardwareMap {
        private final Map<String, HardwareDevice> devices = new HashMap<>();
        private final Effects effects = new Effects();

        private TestHardwareMap() {
            super(null, null);
        }

        private MotorProbe addMotor(String name, int positionTicks) {
            MotorProbe probe = new MotorProbe(effects, positionTicks);
            devices.put(name, probe.motor);
            return probe;
        }

        private void addCrServo(String name) {
            devices.put(name, new CrServoProbe(effects).servo);
        }

        @Override
        public <T> T get(Class<? extends T> type, String name) {
            effects.lookupCalls++;
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

    /** Aggregate construction, sensing, control, and output effects. */
    private static final class Effects {
        private int lookupCalls;
        private int directionWrites;
        private int modeReads;
        private int modeWrites;
        private int powerWrites;
        private int targetPositionWrites;
        private int positionReads;
        private int positionPidfWrites;
        private int velocityPidfWrites;
        private int targetToleranceWrites;
        private int regulatorCalls;

        private int[] snapshot() {
            return new int[]{
                    lookupCalls,
                    directionWrites,
                    modeReads,
                    modeWrites,
                    powerWrites,
                    targetPositionWrites,
                    positionReads,
                    positionPidfWrites,
                    velocityPidfWrites,
                    targetToleranceWrites,
                    regulatorCalls
            };
        }

        private void assertNone() {
            assertArrayEquals(new int[snapshot().length], snapshot());
        }
    }

    /** Dynamic SDK motor with observable configuration, sensing, and output calls. */
    private static final class MotorProbe {
        private final Effects effects;
        private final DcMotorEx motor;
        private final int positionTicks;
        private DcMotor.RunMode mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        private int targetPosition;

        private MotorProbe(Effects effects, int positionTicks) {
            this.effects = effects;
            this.positionTicks = positionTicks;
            motor = (DcMotorEx) Proxy.newProxyInstance(
                    DcMotorEx.class.getClassLoader(),
                    new Class<?>[]{DcMotorEx.class},
                    this::invoke);
        }

        private Object invoke(Object proxy, Method method, Object[] args) {
            String name = method.getName();
            if (method.getDeclaringClass() == Object.class) {
                return objectMethod(proxy, name, args, "MotorProbe");
            }
            if ("setDirection".equals(name)) {
                effects.directionWrites++;
                return null;
            }
            if ("getDirection".equals(name)) return DcMotorSimple.Direction.FORWARD;
            if ("getMode".equals(name)) {
                effects.modeReads++;
                return mode;
            }
            if ("setMode".equals(name)) {
                effects.modeWrites++;
                mode = (DcMotor.RunMode) args[0];
                return null;
            }
            if ("setPower".equals(name)) {
                effects.powerWrites++;
                return null;
            }
            if ("getPower".equals(name)) return 0.0;
            if ("setTargetPosition".equals(name)) {
                effects.targetPositionWrites++;
                targetPosition = (int) args[0];
                return null;
            }
            if ("getTargetPosition".equals(name)) return targetPosition;
            if ("getCurrentPosition".equals(name)) {
                effects.positionReads++;
                return positionTicks;
            }
            if ("setPositionPIDFCoefficients".equals(name)) {
                effects.positionPidfWrites++;
                return null;
            }
            if ("setVelocityPIDFCoefficients".equals(name)) {
                effects.velocityPidfWrites++;
                return null;
            }
            if ("setTargetPositionTolerance".equals(name)) {
                effects.targetToleranceWrites++;
                return null;
            }
            if ("getZeroPowerBehavior".equals(name)) return DcMotor.ZeroPowerBehavior.FLOAT;
            if ("isBusy".equals(name)) return false;
            return hardwareDeviceOrDefault(method);
        }
    }

    /** Dynamic CR servo with observable direction and power writes. */
    private static final class CrServoProbe {
        private final Effects effects;
        private final CRServo servo;

        private CrServoProbe(Effects effects) {
            this.effects = effects;
            servo = (CRServo) Proxy.newProxyInstance(
                    CRServo.class.getClassLoader(),
                    new Class<?>[]{CRServo.class},
                    this::invoke);
        }

        private Object invoke(Object proxy, Method method, Object[] args) {
            String name = method.getName();
            if (method.getDeclaringClass() == Object.class) {
                return objectMethod(proxy, name, args, "CrServoProbe");
            }
            if ("setDirection".equals(name)) {
                effects.directionWrites++;
                return null;
            }
            if ("getDirection".equals(name)) return DcMotorSimple.Direction.FORWARD;
            if ("setPower".equals(name)) {
                effects.powerWrites++;
                return null;
            }
            if ("getPower".equals(name)) return 0.0;
            return hardwareDeviceOrDefault(method);
        }
    }

    private static Object objectMethod(Object proxy,
                                       String methodName,
                                       Object[] args,
                                       String label) {
        if ("equals".equals(methodName)) return proxy == args[0];
        if ("hashCode".equals(methodName)) return System.identityHashCode(proxy);
        if ("toString".equals(methodName)) return label;
        return null;
    }

    private static Object hardwareDeviceOrDefault(Method method) {
        String name = method.getName();
        if ("getManufacturer".equals(name)) return HardwareDevice.Manufacturer.Other;
        if ("getDeviceName".equals(name)) return "CAL-02 probe";
        if ("getConnectionInfo".equals(name)) return "test";
        if ("getVersion".equals(name)) return 1;
        return defaultValue(method.getReturnType());
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
