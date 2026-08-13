package edu.ftcphoenix.fw.ftc;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.junit.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Proxy;

import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.hal.PowerLimitedPositionOutput;
import edu.ftcphoenix.fw.core.hal.PositionOutput;
import edu.ftcphoenix.fw.core.hal.VelocityOutput;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies MAP-01's final numeric guards at the direct FTC output seams. */
public final class FtcHardwareCommandDomainValidationTest {

    private static final double EPSILON = 1.0e-12;

    @Test
    public void motorPositionFactoriesReturnPairedCapabilityWithoutFactoryPowerOverloads()
            throws Exception {
        Method named = FtcHardware.class.getDeclaredMethod(
                "motorPosition", HardwareMap.class, String.class, Direction.class);
        Method direct = FtcHardware.class.getDeclaredMethod(
                "motorPosition", DcMotorEx.class, Direction.class);

        assertEquals(PowerLimitedPositionOutput.class, named.getReturnType());
        assertEquals(PowerLimitedPositionOutput.class, direct.getReturnType());
        assertNoDeclaredMethod(FtcHardware.class,
                "motorPosition", HardwareMap.class, String.class, Direction.class, double.class);
        assertNoDeclaredMethod(FtcHardware.class,
                "motorPosition", DcMotorEx.class, Direction.class, double.class);

        Method namedServo = FtcHardware.class.getDeclaredMethod(
                "servoPosition", HardwareMap.class, String.class, Direction.class);
        Method directServo = FtcHardware.class.getDeclaredMethod(
                "servoPosition", Servo.class, Direction.class);
        assertEquals(PositionOutput.class, namedServo.getReturnType());
        assertEquals(PositionOutput.class, directServo.getReturnType());
    }

    @Test
    public void servoStopWaitsForFirstSuccessfulCommandThenReassertsIt() {
        ServoProbe probe = new ServoProbe();
        PositionOutput output = FtcHardware.servoPosition(probe.servo, Direction.FORWARD);

        assertFalse(output instanceof PowerLimitedPositionOutput);
        assertTrue(Double.isNaN(output.getCommandedPosition()));
        output.stop();
        assertEquals(0, probe.positionWrites);

        probe.positionFailure = new IllegalStateException("simulated servo write failure");
        expect(IllegalStateException.class, () -> output.setPosition(0.35));
        assertTrue(Double.isNaN(output.getCommandedPosition()));
        assertEquals(0, probe.positionWrites);
        output.stop();
        assertEquals(0, probe.positionWrites);

        probe.positionFailure = null;
        output.setPosition(0.35);
        assertEquals(0.35, output.getCommandedPosition(), EPSILON);
        assertEquals(1, probe.positionWrites);

        output.stop();
        assertEquals(2, probe.positionWrites);
        assertEquals(0.35, probe.position, EPSILON);
    }

    @Test
    public void servoRejectsNonFiniteBeforeCacheOrSdkAndRetainsFiniteClampDefense() {
        ServoProbe probe = new ServoProbe();
        PositionOutput output = FtcHardware.servoPosition(probe.servo, Direction.FORWARD);

        output.setPosition(0.25);
        assertEquals(0.25, output.getCommandedPosition(), EPSILON);
        assertEquals(0.25, probe.position, EPSILON);
        int acceptedWrites = probe.positionWrites;

        for (double invalid : nonFiniteValues()) {
            expect(IllegalArgumentException.class, () -> output.setPosition(invalid));
            assertEquals(acceptedWrites, probe.positionWrites);
            assertEquals(0.25, output.getCommandedPosition(), EPSILON);
            assertEquals(0.25, probe.position, EPSILON);
        }

        output.setPosition(-4.0);
        assertEquals(0.0, output.getCommandedPosition(), EPSILON);
        assertEquals(0.0, probe.position, EPSILON);
        output.setPosition(4.0);
        assertEquals(1.0, output.getCommandedPosition(), EPSILON);
        assertEquals(1.0, probe.position, EPSILON);
    }

    @Test
    public void motorVelocityRejectsNonFiniteBeforeCacheModeOrSdkAndKeepsNaturalStop() {
        MotorProbe probe = new MotorProbe();
        VelocityOutput output = FtcHardware.motorVelocity(probe.motor, Direction.FORWARD);

        output.setVelocity(1234.5);
        assertEquals(1234.5, output.getCommandedVelocity(), EPSILON);
        assertEquals(1, probe.modeWrites);
        assertEquals(1, probe.velocityWrites);
        int acceptedPowerWrites = probe.powerWrites;

        for (double invalid : nonFiniteValues()) {
            expect(IllegalArgumentException.class, () -> output.setVelocity(invalid));
            assertEquals(1234.5, output.getCommandedVelocity(), EPSILON);
            assertEquals(1, probe.modeWrites);
            assertEquals(1, probe.velocityWrites);
            assertEquals(acceptedPowerWrites, probe.powerWrites);
        }

        output.stop();
        assertEquals(2, probe.velocityWrites);
        assertEquals(0.0, probe.velocity, EPSILON);
        assertEquals(acceptedPowerWrites + 1, probe.powerWrites);
        assertEquals(0.0, probe.power, EPSILON);
        assertEquals(1234.5, output.getCommandedVelocity(), EPSILON);
    }

    @Test
    public void motorPositionPairsRoundedTargetAndPowerMagnitudeInOneCall() {
        MotorProbe probe = new MotorProbe();
        PowerLimitedPositionOutput output =
                FtcHardware.motorPosition(probe.motor, Direction.REVERSE);

        assertTrue(Double.isNaN(output.getCommandedPosition()));
        assertTrue(Double.isNaN(output.getCommandedMaximumOutputPowerMagnitude()));

        output.setPosition(10.6, 0.5);
        assertEquals(11.0, output.getCommandedPosition(), EPSILON);
        assertEquals(0.5, output.getCommandedMaximumOutputPowerMagnitude(), EPSILON);
        assertEquals(11, probe.targetPosition);
        assertEquals(0.5, probe.power, EPSILON);
        assertEquals(1, probe.targetWrites);
        assertEquals(1, probe.modeWrites);
        assertEquals(1, probe.powerWrites);

        output.setPosition(12.0);
        assertEquals(12.0, output.getCommandedPosition(), EPSILON);
        assertEquals(1.0, output.getCommandedMaximumOutputPowerMagnitude(), EPSILON);
        assertEquals(1.0, probe.power, EPSILON);

        output.setPosition(13.0, -0.0);
        assertEquals(13.0, output.getCommandedPosition(), EPSILON);
        assertRawDoubleEquals(-0.0, output.getCommandedMaximumOutputPowerMagnitude());
        assertRawDoubleEquals(-0.0, probe.power);
    }

    @Test
    public void motorPositionValidatesCompletePairBeforeEveryEffectAndPreservesCache() {
        MotorProbe probe = new MotorProbe();
        PowerLimitedPositionOutput output =
                FtcHardware.motorPosition(probe.motor, Direction.REVERSE);

        output.setPosition(10.6, 0.5);
        assertEquals(11.0, output.getCommandedPosition(), EPSILON);
        assertEquals(0.5, output.getCommandedMaximumOutputPowerMagnitude(), EPSILON);

        int targetWrites = probe.targetWrites;
        int modeWrites = probe.modeWrites;
        int powerWrites = probe.powerWrites;
        double[] invalid = {
                Double.NaN,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY,
                (double) Integer.MAX_VALUE + 0.5,
                Math.nextDown((double) Integer.MIN_VALUE - 0.5),
                Double.MAX_VALUE,
                -Double.MAX_VALUE
        };
        for (double value : invalid) {
            expect(IllegalArgumentException.class, () -> output.setPosition(value, 0.25));
            assertEquals(11.0, output.getCommandedPosition(), EPSILON);
            assertEquals(0.5, output.getCommandedMaximumOutputPowerMagnitude(), EPSILON);
            assertEquals(targetWrites, probe.targetWrites);
            assertEquals(modeWrites, probe.modeWrites);
            assertEquals(powerWrites, probe.powerWrites);
        }

        double[] invalidMagnitudes = {
                Double.NaN,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY,
                Math.nextDown(0.0),
                Math.nextUp(1.0)
        };
        for (double value : invalidMagnitudes) {
            expect(IllegalArgumentException.class, () -> output.setPosition(20.0, value));
            assertEquals(11.0, output.getCommandedPosition(), EPSILON);
            assertEquals(0.5, output.getCommandedMaximumOutputPowerMagnitude(), EPSILON);
            assertEquals(targetWrites, probe.targetWrites);
            assertEquals(modeWrites, probe.modeWrites);
            assertEquals(powerWrites, probe.powerWrites);
        }

        output.setPosition(Integer.MAX_VALUE, 0.75);
        assertEquals((double) Integer.MAX_VALUE, output.getCommandedPosition(), 0.0);
        assertEquals(0.75, output.getCommandedMaximumOutputPowerMagnitude(), 0.0);
        assertEquals(Integer.MAX_VALUE, probe.targetPosition);

        // Math.round(-2147483648.5) is exactly Integer.MIN_VALUE and remains in-domain.
        output.setPosition((double) Integer.MIN_VALUE - 0.5, 0.25);
        assertEquals((double) Integer.MIN_VALUE, output.getCommandedPosition(), 0.0);
        assertEquals(0.25, output.getCommandedMaximumOutputPowerMagnitude(), 0.0);
        assertEquals(Integer.MIN_VALUE, probe.targetPosition);

        output.stop();
        assertEquals(0.0, probe.power, EPSILON);
        assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, probe.mode);
        assertEquals((double) Integer.MIN_VALUE, output.getCommandedPosition(), 0.0);
        assertEquals(0.25, output.getCommandedMaximumOutputPowerMagnitude(), 0.0);
    }

    @Test
    public void motorPositionInvalidatesPairCacheWhenPairedSubmissionFails() {
        MotorProbe probe = new MotorProbe();
        PowerLimitedPositionOutput output =
                FtcHardware.motorPosition(probe.motor, Direction.FORWARD);
        output.setPosition(10.0, 0.5);

        probe.powerFailure = new IllegalStateException("simulated power write failure");
        expect(IllegalStateException.class, () -> output.setPosition(20.0, 0.25));

        assertTrue(Double.isNaN(output.getCommandedPosition()));
        assertTrue(Double.isNaN(output.getCommandedMaximumOutputPowerMagnitude()));
        assertEquals(20, probe.targetPosition);
        assertEquals(DcMotor.RunMode.RUN_TO_POSITION, probe.mode);

        probe.powerFailure = null;
        output.stop();
        assertEquals(0.0, probe.power, EPSILON);
        assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, probe.mode);
    }

    private static double[] nonFiniteValues() {
        return new double[]{Double.NaN, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY};
    }

    private static void expect(Class<? extends RuntimeException> expected, Runnable action) {
        try {
            action.run();
        } catch (RuntimeException failure) {
            assertTrue("Expected " + expected.getSimpleName() + ", got " + failure,
                    expected.isInstance(failure));
            return;
        }
        fail("Expected " + expected.getSimpleName());
    }

    private static void assertRawDoubleEquals(double expected, double actual) {
        assertEquals(Double.doubleToRawLongBits(expected), Double.doubleToRawLongBits(actual));
    }

    private static void assertNoDeclaredMethod(Class<?> owner,
                                               String name,
                                               Class<?>... parameterTypes) {
        try {
            owner.getDeclaredMethod(name, parameterTypes);
            fail(owner.getSimpleName() + " must not declare legacy " + name + " overload");
        } catch (NoSuchMethodException expected) {
            // Expected.
        }
    }

    private static final class MotorProbe {
        private final DcMotorEx motor;
        private int targetPosition;
        private int targetWrites;
        private int modeWrites;
        private int velocityWrites;
        private int powerWrites;
        private double velocity;
        private double power;
        private RuntimeException powerFailure;
        private DcMotor.RunMode mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        private DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;

        private MotorProbe() {
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
                direction = (DcMotorSimple.Direction) args[0];
                return null;
            }
            if ("getDirection".equals(name)) return direction;
            if ("setTargetPosition".equals(name)) {
                targetPosition = (int) args[0];
                targetWrites++;
                return null;
            }
            if ("getTargetPosition".equals(name)) return targetPosition;
            if ("setMode".equals(name)) {
                mode = (DcMotor.RunMode) args[0];
                modeWrites++;
                return null;
            }
            if ("getMode".equals(name)) return mode;
            if ("setVelocity".equals(name)) {
                velocity = (double) args[0];
                velocityWrites++;
                return null;
            }
            if ("getVelocity".equals(name)) return velocity;
            if ("setPower".equals(name)) {
                if (powerFailure != null) throw powerFailure;
                power = (double) args[0];
                powerWrites++;
                return null;
            }
            if ("getPower".equals(name)) return power;
            if ("getCurrentPosition".equals(name)) return 0;
            if ("getZeroPowerBehavior".equals(name)) return DcMotor.ZeroPowerBehavior.FLOAT;
            if ("isBusy".equals(name)) return false;
            return hardwareDeviceOrDefault(method, "MotorProbe");
        }
    }

    private static final class ServoProbe {
        private final Servo servo;
        private int positionWrites;
        private double position;
        private RuntimeException positionFailure;
        private Servo.Direction direction = Servo.Direction.FORWARD;

        private ServoProbe() {
            servo = (Servo) Proxy.newProxyInstance(
                    Servo.class.getClassLoader(),
                    new Class<?>[]{Servo.class},
                    this::invoke);
        }

        private Object invoke(Object proxy, Method method, Object[] args) {
            String name = method.getName();
            if (method.getDeclaringClass() == Object.class) {
                return objectMethod(proxy, name, args, "ServoProbe");
            }
            if ("setDirection".equals(name)) {
                direction = (Servo.Direction) args[0];
                return null;
            }
            if ("getDirection".equals(name)) return direction;
            if ("setPosition".equals(name)) {
                if (positionFailure != null) throw positionFailure;
                position = (double) args[0];
                positionWrites++;
                return null;
            }
            if ("getPosition".equals(name)) return position;
            return hardwareDeviceOrDefault(method, "ServoProbe");
        }
    }

    private static Object objectMethod(Object proxy, String name, Object[] args, String label) {
        if ("equals".equals(name)) return proxy == args[0];
        if ("hashCode".equals(name)) return System.identityHashCode(proxy);
        if ("toString".equals(name)) return label;
        return null;
    }

    private static Object hardwareDeviceOrDefault(Method method, String label) {
        String name = method.getName();
        if ("getManufacturer".equals(name)) return HardwareDevice.Manufacturer.Other;
        if ("getDeviceName".equals(name)) return label;
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
