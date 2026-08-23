package edu.ftcphoenix.fw.ftc;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.junit.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.hal.PowerOutput;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the FTC leaf adapters' truthful power-command cache and fail-stop behavior. */
public final class FtcPowerOutputFailStopTest {

    private static final double EPSILON = 1.0e-12;

    @Test
    public void leafCachesRemainUnknownUntilSdkPowerCallsReturn() {
        MotorProbe motorProbe = new MotorProbe(DcMotor.RunMode.RUN_USING_ENCODER);
        PowerOutput motor = FtcHardware.motorPower(motorProbe.motor, Direction.FORWARD);
        motorProbe.powerObserver = () -> assertTrue(Double.isNaN(motor.getCommandedPower()));

        motor.setPower(0.35);
        assertEquals(0.35, motor.getCommandedPower(), EPSILON);
        motor.stop();
        assertEquals(0.0, motor.getCommandedPower(), EPSILON);

        CrServoProbe servoProbe = new CrServoProbe();
        PowerOutput servo = FtcHardware.crServoPower(servoProbe.servo, Direction.FORWARD);
        servoProbe.powerObserver = () -> assertTrue(Double.isNaN(servo.getCommandedPower()));

        servo.setPower(-0.45);
        assertEquals(-0.45, servo.getCommandedPower(), EPSILON);
        servo.stop();
        assertEquals(0.0, servo.getCommandedPower(), EPSILON);
    }

    @Test
    public void motorCommitsOnlySuccessfulFiniteCommandsAndSupportsExplicitRecovery() {
        MotorProbe probe = new MotorProbe(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PowerOutput output = FtcHardware.motorPower(probe.motor, Direction.FORWARD);

        assertTrue(Double.isNaN(output.getCommandedPower()));
        assertEquals(0, probe.powerCalls);

        output.setPower(2.5);
        assertEquals(1.0, probe.power, EPSILON);
        assertEquals(1.0, output.getCommandedPower(), EPSILON);

        RuntimeException writeFailure = new IllegalStateException("requested write failed");
        probe.failPowerCall(probe.powerCalls + 1, writeFailure);
        assertSame(writeFailure, expect(RuntimeException.class, () -> output.setPower(0.4)));
        assertEquals(0.4, probe.lastPowerRequest(), EPSILON);
        assertTrue(Double.isNaN(output.getCommandedPower()));

        output.setPower(-3.0);
        assertEquals(-1.0, probe.power, EPSILON);
        assertEquals(-1.0, output.getCommandedPower(), EPSILON);

        RuntimeException postEffectFailure =
                new IllegalStateException("requested write failed after probe mutation");
        probe.failPowerCallAfterEffect(probe.powerCalls + 1, postEffectFailure);
        assertSame(postEffectFailure, expect(
                RuntimeException.class,
                () -> output.setPower(0.6)));
        assertEquals(0.6, probe.power, EPSILON);
        assertTrue(Double.isNaN(output.getCommandedPower()));

        RuntimeException stopFailure = new IllegalStateException("stop write failed");
        probe.failPowerCall(probe.powerCalls + 1, stopFailure);
        assertSame(stopFailure, expect(RuntimeException.class, output::stop));
        assertEquals(0.0, probe.lastPowerRequest(), EPSILON);
        assertTrue(Double.isNaN(output.getCommandedPower()));

        output.stop();
        assertEquals(0.0, probe.power, EPSILON);
        assertEquals(0.0, output.getCommandedPower(), EPSILON);

        int callsBeforeRepeatedStop = probe.powerCalls;
        output.stop();
        assertEquals(callsBeforeRepeatedStop + 1, probe.powerCalls);
        assertEquals(0.0, probe.lastPowerRequest(), EPSILON);
        assertEquals(0.0, output.getCommandedPower(), EPSILON);
    }

    @Test
    public void motorRejectsNonFinitePowerWithNaturalZeroAndNoModeAcquisition() {
        MotorProbe probe = new MotorProbe(DcMotor.RunMode.RUN_TO_POSITION);
        PowerOutput output = FtcHardware.motorPower(probe.motor, Direction.FORWARD);

        for (double invalid : nonFiniteValues()) {
            output.stop();
            int modeReadsBefore = probe.getModeCalls;
            int powerCallsBefore = probe.powerCalls;
            IllegalArgumentException observed =
                    expect(IllegalArgumentException.class, () -> output.setPower(invalid));

            assertTrue(observed.getMessage().contains("must be finite"));
            assertTrue(observed.getMessage().contains("finite normalized value"));
            assertEquals(modeReadsBefore, probe.getModeCalls);
            assertEquals(powerCallsBefore + 1, probe.powerCalls);
            assertEquals(DcMotor.RunMode.RUN_TO_POSITION, probe.mode);
            assertEquals(0.0, probe.lastPowerRequest(), EPSILON);
            assertTrue(Double.isNaN(output.getCommandedPower()));
        }

        RuntimeException cleanupFailure = new IllegalStateException("invalid cleanup failed");
        output.stop();
        int powerCallsBefore = probe.powerCalls;
        probe.failPowerCall(probe.powerCalls + 1, cleanupFailure);
        IllegalArgumentException observed =
                expect(IllegalArgumentException.class, () -> output.setPower(Double.NaN));

        assertEquals(1, observed.getSuppressed().length);
        assertSame(cleanupFailure, observed.getSuppressed()[0]);
        assertEquals(powerCallsBefore + 1, probe.powerCalls);
        assertEquals(0, probe.getModeCalls);
        assertTrue(Double.isNaN(output.getCommandedPower()));

        output.stop();
        assertEquals(0, probe.getModeCalls);
        assertEquals(0.0, output.getCommandedPower(), EPSILON);
    }

    @Test
    public void motorModeDiagnosticRetainsNestedCleanupAndUnknownCacheUntilRecovery() {
        MotorProbe probe = new MotorProbe(DcMotor.RunMode.RUN_USING_ENCODER);
        RuntimeException modeFailure = new IllegalStateException("mode read failed");
        RuntimeException cleanupFailure = new IllegalStateException("mode cleanup failed");
        probe.failModeRead(1, modeFailure);
        probe.failPowerCall(1, cleanupFailure);
        PowerOutput output = FtcHardware.motorPower(probe.motor, Direction.FORWARD);

        IllegalStateException observed =
                expect(IllegalStateException.class, () -> output.setPower(0.5));

        assertTrue(observed.getMessage().contains("Cannot acquire raw motor-power mode"));
        assertTrue(observed.getMessage().contains("requiredMode=RUN_WITHOUT_ENCODER"));
        assertSame(modeFailure, observed.getCause());
        assertEquals(1, modeFailure.getSuppressed().length);
        assertSame(cleanupFailure, modeFailure.getSuppressed()[0]);
        assertTrue(Double.isNaN(output.getCommandedPower()));

        output.stop();
        assertEquals(0.0, output.getCommandedPower(), EPSILON);
        assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, probe.mode);

        output.setPower(0.5);
        assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, probe.mode);
        assertEquals(0.5, probe.power, EPSILON);
        assertEquals(0.5, output.getCommandedPower(), EPSILON);
    }

    @Test
    public void crServoCommitsOnlySuccessfulFiniteCommandsAndSupportsExplicitRecovery() {
        CrServoProbe probe = new CrServoProbe();
        PowerOutput output = FtcHardware.crServoPower(probe.servo, Direction.REVERSE);

        assertTrue(Double.isNaN(output.getCommandedPower()));
        assertEquals(0, probe.powerCalls);

        output.setPower(4.0);
        assertEquals(1.0, probe.power, EPSILON);
        assertEquals(1.0, output.getCommandedPower(), EPSILON);

        RuntimeException writeFailure = new IllegalStateException("servo write failed");
        probe.failPowerCall(probe.powerCalls + 1, writeFailure);
        assertSame(writeFailure, expect(RuntimeException.class, () -> output.setPower(0.35)));
        assertTrue(Double.isNaN(output.getCommandedPower()));

        output.setPower(-4.0);
        assertEquals(-1.0, probe.power, EPSILON);
        assertEquals(-1.0, output.getCommandedPower(), EPSILON);

        RuntimeException postEffectFailure =
                new IllegalStateException("servo write failed after probe mutation");
        probe.failPowerCallAfterEffect(probe.powerCalls + 1, postEffectFailure);
        assertSame(postEffectFailure, expect(
                RuntimeException.class,
                () -> output.setPower(0.45)));
        assertEquals(0.45, probe.power, EPSILON);
        assertTrue(Double.isNaN(output.getCommandedPower()));

        RuntimeException stopFailure = new IllegalStateException("servo stop failed");
        probe.failPowerCall(probe.powerCalls + 1, stopFailure);
        assertSame(stopFailure, expect(RuntimeException.class, output::stop));
        assertTrue(Double.isNaN(output.getCommandedPower()));

        output.stop();
        assertEquals(0.0, probe.power, EPSILON);
        assertEquals(0.0, output.getCommandedPower(), EPSILON);
        int callsBeforeRepeatedStop = probe.powerCalls;
        output.stop();
        assertEquals(callsBeforeRepeatedStop + 1, probe.powerCalls);
        assertEquals(0.0, probe.lastPowerRequest(), EPSILON);
        assertEquals(0.0, output.getCommandedPower(), EPSILON);
    }

    @Test
    public void crServoRejectsEveryNonFinitePowerAndSuppressesZeroFailure() {
        CrServoProbe probe = new CrServoProbe();
        PowerOutput output = FtcHardware.crServoPower(probe.servo, Direction.FORWARD);

        for (double invalid : nonFiniteValues()) {
            output.stop();
            int powerCallsBefore = probe.powerCalls;
            IllegalArgumentException observed =
                    expect(IllegalArgumentException.class, () -> output.setPower(invalid));

            assertTrue(observed.getMessage().contains("must be finite"));
            assertTrue(observed.getMessage().contains("finite normalized value"));
            assertEquals(powerCallsBefore + 1, probe.powerCalls);
            assertEquals(0.0, probe.lastPowerRequest(), EPSILON);
            assertTrue(Double.isNaN(output.getCommandedPower()));
        }

        RuntimeException cleanupFailure = new IllegalStateException("servo cleanup failed");
        output.stop();
        int powerCallsBefore = probe.powerCalls;
        probe.failPowerCall(probe.powerCalls + 1, cleanupFailure);
        IllegalArgumentException observed = expect(
                IllegalArgumentException.class,
                () -> output.setPower(Double.POSITIVE_INFINITY));

        assertEquals(1, observed.getSuppressed().length);
        assertSame(cleanupFailure, observed.getSuppressed()[0]);
        assertEquals(powerCallsBefore + 1, probe.powerCalls);
        assertTrue(Double.isNaN(output.getCommandedPower()));

        output.setPower(0.25);
        assertEquals(0.25, output.getCommandedPower(), EPSILON);
    }

    private static double[] nonFiniteValues() {
        return new double[]{Double.NaN, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY};
    }

    private static <T extends RuntimeException> T expect(Class<T> type, Runnable action) {
        try {
            action.run();
        } catch (RuntimeException failure) {
            assertTrue("Expected " + type.getSimpleName() + ", got " + failure,
                    type.isInstance(failure));
            return type.cast(failure);
        }
        fail("Expected " + type.getSimpleName());
        return null;
    }

    private static final class MotorProbe {
        private final DcMotorEx motor;
        private final List<Double> powerRequests = new ArrayList<>();
        private final Map<Integer, RuntimeException> powerFailures = new HashMap<>();
        private final Map<Integer, RuntimeException> postEffectPowerFailures = new HashMap<>();
        private final Map<Integer, RuntimeException> modeReadFailures = new HashMap<>();
        private int powerCalls;
        private int getModeCalls;
        private double power;
        private DcMotor.RunMode mode;
        private DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
        private Runnable powerObserver;

        private MotorProbe(DcMotor.RunMode mode) {
            this.mode = mode;
            motor = (DcMotorEx) Proxy.newProxyInstance(
                    DcMotorEx.class.getClassLoader(),
                    new Class<?>[]{DcMotorEx.class},
                    this::invoke);
        }

        private void failPowerCall(int call, RuntimeException failure) {
            powerFailures.put(call, failure);
        }

        private void failPowerCallAfterEffect(int call, RuntimeException failure) {
            postEffectPowerFailures.put(call, failure);
        }

        private void failModeRead(int call, RuntimeException failure) {
            modeReadFailures.put(call, failure);
        }

        private double lastPowerRequest() {
            return powerRequests.get(powerRequests.size() - 1);
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
            if ("setPower".equals(name)) {
                double requested = (double) args[0];
                powerCalls++;
                powerRequests.add(requested);
                RuntimeException failure = powerFailures.get(powerCalls);
                if (failure != null) throw failure;
                power = requested;
                if (powerObserver != null) powerObserver.run();
                RuntimeException postEffectFailure = postEffectPowerFailures.get(powerCalls);
                if (postEffectFailure != null) throw postEffectFailure;
                return null;
            }
            if ("getPower".equals(name)) return power;
            if ("getMode".equals(name)) {
                getModeCalls++;
                RuntimeException failure = modeReadFailures.get(getModeCalls);
                if (failure != null) throw failure;
                return mode;
            }
            if ("setMode".equals(name)) {
                mode = (DcMotor.RunMode) args[0];
                return null;
            }
            if ("getCurrentPosition".equals(name)) return 0;
            if ("getTargetPosition".equals(name)) return 0;
            if ("getVelocity".equals(name)) return 0.0;
            if ("getZeroPowerBehavior".equals(name)) return DcMotor.ZeroPowerBehavior.FLOAT;
            if ("isBusy".equals(name)) return false;
            return hardwareDeviceOrDefault(method, "MotorProbe");
        }
    }

    private static final class CrServoProbe {
        private final CRServo servo;
        private final List<Double> powerRequests = new ArrayList<>();
        private final Map<Integer, RuntimeException> powerFailures = new HashMap<>();
        private final Map<Integer, RuntimeException> postEffectPowerFailures = new HashMap<>();
        private int powerCalls;
        private double power;
        private DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
        private Runnable powerObserver;

        private CrServoProbe() {
            servo = (CRServo) Proxy.newProxyInstance(
                    CRServo.class.getClassLoader(),
                    new Class<?>[]{CRServo.class},
                    this::invoke);
        }

        private void failPowerCall(int call, RuntimeException failure) {
            powerFailures.put(call, failure);
        }

        private void failPowerCallAfterEffect(int call, RuntimeException failure) {
            postEffectPowerFailures.put(call, failure);
        }

        private double lastPowerRequest() {
            return powerRequests.get(powerRequests.size() - 1);
        }

        private Object invoke(Object proxy, Method method, Object[] args) {
            String name = method.getName();
            if (method.getDeclaringClass() == Object.class) {
                return objectMethod(proxy, name, args, "CrServoProbe");
            }
            if ("setDirection".equals(name)) {
                direction = (DcMotorSimple.Direction) args[0];
                return null;
            }
            if ("getDirection".equals(name)) return direction;
            if ("setPower".equals(name)) {
                double requested = (double) args[0];
                powerCalls++;
                powerRequests.add(requested);
                RuntimeException failure = powerFailures.get(powerCalls);
                if (failure != null) throw failure;
                power = requested;
                if (powerObserver != null) powerObserver.run();
                RuntimeException postEffectFailure = postEffectPowerFailures.get(powerCalls);
                if (postEffectFailure != null) throw postEffectFailure;
                return null;
            }
            if ("getPower".equals(name)) return power;
            return hardwareDeviceOrDefault(method, "CrServoProbe");
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
