package edu.ftcsushi.robots.phoenix.tester;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.lang.reflect.Proxy;
import java.util.HashMap;
import java.util.Map;

import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.ftc.FtcDrives;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.tools.tester.TesterContext;
import edu.ftcsushi.robots.phoenix.PhoenixProfile;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

/** Focused lifecycle and command-isolation checks for Phoenix's drivetrain verifier. */
public final class ConfiguredDrivetrainVerificationTesterTest {

    private static final double EPSILON = 1e-9;

    @Test
    public void constructionSurfaceRequiresOneExplicitDriveConfig() {
        Constructor<?>[] constructors =
                ConfiguredDrivetrainVerificationTester.class.getDeclaredConstructors();
        assertEquals(1, constructors.length);
        assertEquals(1, constructors[0].getParameterTypes().length);
        assertEquals(
                FtcDrives.MecanumConfig.class,
                constructors[0].getParameterTypes()[0]
        );
        assertTrue(Modifier.isFinal(
                ConfiguredDrivetrainVerificationTester.class.getModifiers()
        ));
        assertFalse(Modifier.isPublic(
                ConfiguredDrivetrainVerificationTester.class.getModifiers()
        ));
        assertFalse(Modifier.isPublic(constructors[0].getModifiers()));
    }

    @Test
    public void startNeutralExactOneConflictReleaseAndStopStaySafe() {
        TestHardwareMap hardware = new TestHardwareMap();
        MotorProbe frontLeft = hardware.addMotor("frontLeftMotor");
        MotorProbe frontRight = hardware.addMotor("frontRightMotor");
        MotorProbe backLeft = hardware.addMotor("backLeftMotor");
        MotorProbe backRight = hardware.addMotor("backRightMotor");
        Rig rig = new Rig(hardware);
        PhoenixProfile profile = PhoenixProfile.current();
        ConfiguredDrivetrainVerificationTester tester =
                new ConfiguredDrivetrainVerificationTester(profile.drive);

        rig.gamepad1.a = true;
        tester.init(rig.context());
        rig.initCycle(tester);
        rig.initCycle(tester);

        assertEquals("INIT must not resolve drivetrain hardware", 0, hardware.lookups);
        assertNoWrites(frontLeft, frontRight, backLeft, backRight);

        tester.start();
        assertEquals(8, hardware.lookups);
        assertDirection(profile.drive.wiring.frontLeftDirection, frontLeft);
        assertDirection(profile.drive.wiring.frontRightDirection, frontRight);
        assertDirection(profile.drive.wiring.backLeftDirection, backLeft);
        assertDirection(profile.drive.wiring.backRightDirection, backRight);
        assertAllPower(0.0, frontLeft, frontRight, backLeft, backRight);

        rig.activeCycle(tester); // Held suite-selection A must not run back-left after START.
        assertAllPower(0.0, frontLeft, frontRight, backLeft, backRight);

        rig.clearFaceButtons();
        rig.activeCycle(tester); // One neutral cycle arms all four controls.
        assertAllPower(0.0, frontLeft, frontRight, backLeft, backRight);

        rig.gamepad1.x = true;
        rig.activeCycle(tester);
        assertPowers(0.20, 0.0, 0.0, 0.0, frontLeft, frontRight, backLeft, backRight);

        rig.clearFaceButtons();
        rig.activeCycle(tester); // Release must command every motor zero.
        assertAllPower(0.0, frontLeft, frontRight, backLeft, backRight);

        rig.gamepad1.y = true;
        rig.activeCycle(tester);
        assertPowers(0.0, 0.20, 0.0, 0.0, frontLeft, frontRight, backLeft, backRight);

        rig.clearFaceButtons();
        rig.gamepad1.a = true;
        rig.activeCycle(tester);
        assertPowers(0.0, 0.0, 0.20, 0.0, frontLeft, frontRight, backLeft, backRight);

        rig.clearFaceButtons();
        rig.gamepad1.b = true;
        rig.activeCycle(tester);
        assertPowers(0.0, 0.0, 0.0, 0.20, frontLeft, frontRight, backLeft, backRight);

        rig.gamepad1.x = true;
        rig.activeCycle(tester); // Two held buttons must force every command to zero.
        assertAllPower(0.0, frontLeft, frontRight, backLeft, backRight);

        rig.clearFaceButtons();
        rig.gamepad1.a = true;
        rig.activeCycle(tester);
        assertPowers(0.0, 0.0, 0.20, 0.0, frontLeft, frontRight, backLeft, backRight);

        tester.stop();
        assertAllPower(0.0, frontLeft, frontRight, backLeft, backRight);
    }

    @Test
    public void duplicateTrimmedNamesFailBeforeAnyHardwareLookupOrWrite() {
        TestHardwareMap hardware = new TestHardwareMap();
        MotorProbe frontLeft = hardware.addMotor("frontLeftMotor");
        MotorProbe frontRight = hardware.addMotor("frontRightMotor");
        MotorProbe backLeft = hardware.addMotor("backLeftMotor");
        MotorProbe backRight = hardware.addMotor("backRightMotor");
        FtcDrives.MecanumConfig config = FtcDrives.MecanumConfig.defaults();
        config.wiring.frontRightName = "  frontLeftMotor  ";
        Rig rig = new Rig(hardware);
        ConfiguredDrivetrainVerificationTester tester =
                new ConfiguredDrivetrainVerificationTester(config);
        tester.init(rig.context());

        RuntimeException failure = assertThrows(RuntimeException.class, tester::start);

        assertTrue(failure.getMessage().contains("unique"));
        assertEquals(0, hardware.lookups);
        assertNoWrites(frontLeft, frontRight, backLeft, backRight);
    }

    @Test
    public void missingLateMotorFailsAfterPreflightLookupsButBeforeAnyWrite() {
        TestHardwareMap hardware = new TestHardwareMap();
        MotorProbe frontLeft = hardware.addMotor("frontLeftMotor");
        MotorProbe frontRight = hardware.addMotor("frontRightMotor");
        MotorProbe backLeft = hardware.addMotor("backLeftMotor");
        FtcDrives.MecanumConfig config = FtcDrives.MecanumConfig.defaults();
        config.wiring.backRightName = "missingBackRight";
        Rig rig = new Rig(hardware);
        ConfiguredDrivetrainVerificationTester tester =
                new ConfiguredDrivetrainVerificationTester(config);
        tester.init(rig.context());

        RuntimeException failure = assertThrows(RuntimeException.class, tester::start);

        assertTrue(failure.getMessage().contains("back-right"));
        assertEquals(4, hardware.lookups);
        assertNoWrites(frontLeft, frontRight, backLeft);
    }

    @Test
    public void distinctNamesResolvingToOneMotorFailBeforeAnyWrite() {
        TestHardwareMap hardware = new TestHardwareMap();
        MotorProbe frontLeft = hardware.addMotor("frontLeftMotor");
        MotorProbe frontRight = hardware.addMotor("frontRightMotor");
        MotorProbe backLeft = hardware.addMotor("backLeftMotor");
        hardware.addAlias("backRightMotor", frontLeft);
        Rig rig = new Rig(hardware);
        ConfiguredDrivetrainVerificationTester tester =
                new ConfiguredDrivetrainVerificationTester(FtcDrives.MecanumConfig.defaults());
        tester.init(rig.context());

        RuntimeException failure = assertThrows(RuntimeException.class, tester::start);

        assertTrue(failure.getMessage().contains("same motor device"));
        assertEquals(4, hardware.lookups);
        assertNoWrites(frontLeft, frontRight, backLeft);
    }

    private static void assertNoWrites(MotorProbe... motors) {
        for (MotorProbe motor : motors) {
            assertEquals(0, motor.directionWrites);
            assertEquals(0, motor.powerWrites);
            assertEquals(0, motor.modeWrites);
        }
    }

    private static void assertDirection(Direction configured, MotorProbe motor) {
        assertEquals(1, motor.directionWrites);
        assertEquals(
                configured == Direction.REVERSE
                        ? DcMotorSimple.Direction.REVERSE
                        : DcMotorSimple.Direction.FORWARD,
                motor.direction);
    }

    private static void assertAllPower(double expected, MotorProbe... motors) {
        for (MotorProbe motor : motors) {
            assertEquals(expected, motor.power, EPSILON);
        }
    }

    private static void assertPowers(double frontLeftPower,
                                     double frontRightPower,
                                     double backLeftPower,
                                     double backRightPower,
                                     MotorProbe frontLeft,
                                     MotorProbe frontRight,
                                     MotorProbe backLeft,
                                     MotorProbe backRight) {
        assertEquals(frontLeftPower, frontLeft.power, EPSILON);
        assertEquals(frontRightPower, frontRight.power, EPSILON);
        assertEquals(backLeftPower, backLeft.power, EPSILON);
        assertEquals(backRightPower, backRight.power, EPSILON);
    }

    private static final class Rig {
        final Gamepad gamepad1 = new Gamepad();
        final LoopClock clock = new LoopClock();
        private final TesterContext context;
        private double nowSec;

        Rig(HardwareMap hardware) {
            clock.reset(0.0);
            context = new TesterContext(
                    hardware,
                    telemetry(),
                    gamepad1,
                    new Gamepad(),
                    clock);
        }

        TesterContext context() {
            return context;
        }

        void initCycle(ConfiguredDrivetrainVerificationTester tester) {
            advance();
            tester.initLoop(clock.dtSec());
        }

        void activeCycle(ConfiguredDrivetrainVerificationTester tester) {
            advance();
            tester.loop(clock.dtSec());
        }

        void clearFaceButtons() {
            gamepad1.a = false;
            gamepad1.b = false;
            gamepad1.x = false;
            gamepad1.y = false;
        }

        private void advance() {
            nowSec += 0.02;
            clock.update(nowSec);
        }
    }

    private static final class TestHardwareMap extends HardwareMap {
        private final Map<String, HardwareDevice> devices = new HashMap<>();
        int lookups;

        TestHardwareMap() {
            super(null, null);
        }

        MotorProbe addMotor(String name) {
            MotorProbe probe = new MotorProbe();
            devices.put(name, probe.motor);
            return probe;
        }

        void addAlias(String name, MotorProbe probe) {
            devices.put(name, probe.motor);
        }

        @Override
        public <T> T get(Class<? extends T> type, String name) {
            lookups++;
            HardwareDevice device = devices.get(name == null ? null : name.trim());
            if (device == null || !type.isInstance(device)) {
                throw new IllegalArgumentException("No " + type.getSimpleName() + " named " + name);
            }
            return type.cast(device);
        }
    }

    private static final class MotorProbe {
        final DcMotorEx motor;
        double power;
        int powerWrites;
        int directionWrites;
        int modeWrites;
        DcMotor.RunMode mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;

        MotorProbe() {
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
            if ("setPower".equals(name)) {
                power = (Double) args[0];
                powerWrites++;
                return null;
            }
            if ("getPower".equals(name)) return power;
            if ("setMode".equals(name)) {
                mode = (DcMotor.RunMode) args[0];
                modeWrites++;
                return null;
            }
            if ("getMode".equals(name)) return mode;
            if ("setDirection".equals(name)) {
                direction = (DcMotorSimple.Direction) args[0];
                directionWrites++;
                return null;
            }
            if ("getDirection".equals(name)) return direction;
            if ("getZeroPowerBehavior".equals(name)) return DcMotor.ZeroPowerBehavior.FLOAT;
            if ("isBusy".equals(name)) return false;
            return hardwareDeviceOrDefault(method);
        }
    }

    private static Telemetry telemetry() {
        return (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                (proxy, method, args) -> {
                    if (method.getDeclaringClass() == Object.class) {
                        return objectMethod(proxy, method.getName(), args, "TelemetryProbe");
                    }
                    return defaultValue(method.getReturnType());
                });
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
        if ("getManufacturer".equals(method.getName())) return HardwareDevice.Manufacturer.Other;
        if ("getDeviceName".equals(method.getName())) return "Phoenix drivetrain verifier probe";
        if ("getConnectionInfo".equals(method.getName())) return "test";
        if ("getVersion".equals(method.getName())) return 1;
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
