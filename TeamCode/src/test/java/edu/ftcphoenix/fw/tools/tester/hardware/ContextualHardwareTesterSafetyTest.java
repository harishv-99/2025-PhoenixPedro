package edu.ftcphoenix.fw.tools.tester.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.HashMap;
import java.util.Map;
import java.util.SortedSet;
import java.util.TreeSet;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.tools.tester.TeleOpTester;
import edu.ftcphoenix.fw.tools.tester.TesterContext;
import edu.ftcphoenix.fw.tools.tester.TesterSuite;
import edu.ftcphoenix.robots.phoenix.tester.DrivetrainMotorDirectionTester;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/** Real tester lifecycle regressions for picker-to-control rearming and retained commands. */
public final class ContextualHardwareTesterSafetyTest {

    private static final double EPSILON = 1e-9;

    @Test
    public void crServoBackAndReselectionRequireFreshEnableFromZero() {
        TestHardwareMap hardware = new TestHardwareMap();
        CrServoProbe first = hardware.addCrServo("crA");
        CrServoProbe second = hardware.addCrServo("crB");
        Rig rig = new Rig(hardware);
        CrServoPowerTester tester = new CrServoPowerTester();
        tester.init(rig.context());

        rig.cycle(tester);
        rig.gamepad1.a = true;
        rig.cycle(tester); // Picker chooses crA; the same A must not enable output.
        assertEquals(0.0, first.power, EPSILON);

        rig.gamepad1.a = false;
        rig.cycle(tester);
        rig.gamepad1.a = true;
        rig.cycle(tester); // Fresh A enables the selected servo at its zero target.
        rig.gamepad1.a = false;
        rig.cycle(tester);
        rig.gamepad1.dpad_up = true;
        rig.cycle(tester);
        assertEquals(0.05, first.power, EPSILON);

        rig.gamepad1.dpad_up = false;
        rig.cycle(tester);
        assertTrue(tester.onBackPressed());
        assertEquals(0.0, first.power, EPSILON);

        rig.cycle(tester);
        rig.gamepad1.dpad_down = true;
        rig.cycle(tester);
        rig.gamepad1.dpad_down = false;
        rig.cycle(tester);
        rig.gamepad1.a = true;
        rig.cycle(tester); // Picker chooses crB; retained crA power must not transfer.
        assertTrue(second.powerWrites > 0);
        assertEquals(0.0, second.power, EPSILON);

        rig.gamepad1.a = false;
        rig.cycle(tester);
        assertEquals(0.0, second.power, EPSILON);
        rig.gamepad1.a = true;
        rig.cycle(tester); // Fresh enable still starts from the reset zero target.
        assertEquals(0.0, second.power, EPSILON);
        rig.gamepad1.a = false;
        rig.cycle(tester);
        rig.gamepad1.dpad_up = true;
        rig.cycle(tester);
        assertEquals(0.05, second.power, EPSILON);
    }

    @Test
    public void servoReselectionUsesEachDevicesSnapshotAndDoesNotWriteWhileDisabled() {
        TestHardwareMap hardware = new TestHardwareMap();
        ServoProbe first = hardware.addServo("servoA", 0.20);
        ServoProbe second = hardware.addServo("servoB", 0.80);
        Rig rig = new Rig(hardware);
        ServoPositionTester tester = new ServoPositionTester();
        tester.init(rig.context());

        rig.cycle(tester);
        rig.gamepad1.a = true;
        rig.cycle(tester); // Picker chooses servoA and snapshots 0.20 without writing.
        assertEquals(0, first.positionWrites);

        rig.gamepad1.a = false;
        rig.cycle(tester);
        assertEquals(0, first.positionWrites);
        rig.gamepad1.a = true;
        rig.cycle(tester); // Fresh A enables and applies servoA's own snapshot.
        assertEquals(0.20, first.position, EPSILON);
        rig.gamepad1.a = false;
        rig.cycle(tester);
        rig.gamepad1.dpad_up = true;
        rig.cycle(tester);
        assertEquals(0.21, first.position, EPSILON);

        rig.gamepad1.dpad_up = false;
        rig.cycle(tester);
        rig.gamepad1.x = true;
        rig.cycle(tester); // Device-local inversion must not carry into the next selection.
        assertEquals(0.79, first.position, EPSILON);
        rig.gamepad1.x = false;
        rig.cycle(tester);
        int writesBeforeBack = first.positionWrites;
        assertTrue(tester.onBackPressed());
        assertEquals(writesBeforeBack + 1, first.positionWrites);
        assertEquals(0.79, first.position, EPSILON);

        rig.cycle(tester);
        rig.gamepad1.dpad_down = true;
        rig.cycle(tester);
        rig.gamepad1.dpad_down = false;
        rig.cycle(tester);
        rig.gamepad1.a = true;
        rig.cycle(tester); // Picker chooses servoB; servoA's 0.21 must not be written.
        assertEquals(0, second.positionWrites);

        rig.gamepad1.a = false;
        rig.cycle(tester);
        assertEquals(0, second.positionWrites);
        rig.gamepad1.a = true;
        rig.cycle(tester); // Fresh enable applies servoB's own 0.80 snapshot.
        assertEquals(1, second.positionWrites);
        assertEquals(0.80, second.position, EPSILON);
    }

    @Test
    public void heldSuiteSelectionDoesNotRunActualDrivetrainMotor() {
        TestHardwareMap hardware = new TestHardwareMap();
        hardware.addMotor("frontLeftMotor");
        hardware.addMotor("frontRightMotor");
        MotorProbe backLeft = hardware.addMotor("backLeftMotor");
        hardware.addMotor("backRightMotor");
        Rig rig = new Rig(hardware);
        TesterSuite suite = new TesterSuite()
                .add("Direction", DrivetrainMotorDirectionTester::new);
        suite.init(rig.context());

        rig.cycle(suite);
        rig.gamepad1.a = true;
        rig.cycle(suite); // The suite-selection A is held as the real child initializes.
        assertEquals(0.0, backLeft.power, EPSILON);

        rig.gamepad1.a = false;
        rig.cycle(suite);
        assertEquals(0.0, backLeft.power, EPSILON);

        rig.gamepad1.a = true;
        rig.cycle(suite); // Only a fresh A runs the back-left motor.
        assertEquals(0.5, backLeft.power, EPSILON);
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

        void cycle(TeleOpTester tester) {
            nowSec += 0.02;
            clock.update(nowSec);
            tester.initLoop(clock.dtSec());
        }
    }

    private static final class TestHardwareMap extends HardwareMap {
        private final Map<String, HardwareDevice> devices = new HashMap<>();

        TestHardwareMap() {
            super(null, null);
        }

        CrServoProbe addCrServo(String name) {
            CrServoProbe probe = new CrServoProbe();
            devices.put(name, probe.servo);
            return probe;
        }

        ServoProbe addServo(String name, double position) {
            ServoProbe probe = new ServoProbe(position);
            devices.put(name, probe.servo);
            return probe;
        }

        MotorProbe addMotor(String name) {
            MotorProbe probe = new MotorProbe();
            devices.put(name, probe.motor);
            return probe;
        }

        @Override
        public <T> T get(Class<? extends T> type, String name) {
            HardwareDevice device = devices.get(name == null ? null : name.trim());
            if (device == null || !type.isInstance(device)) {
                throw new IllegalArgumentException("No " + type.getSimpleName() + " named " + name);
            }
            return type.cast(device);
        }

        @Override
        public SortedSet<String> getAllNames(Class<? extends HardwareDevice> type) {
            SortedSet<String> names = new TreeSet<>();
            for (Map.Entry<String, HardwareDevice> entry : devices.entrySet()) {
                if (type.isInstance(entry.getValue())) {
                    names.add(entry.getKey());
                }
            }
            return names;
        }
    }

    private static final class CrServoProbe {
        final CRServo servo;
        double power;
        int powerWrites;
        private DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;

        CrServoProbe() {
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
            if ("setPower".equals(name)) {
                power = (Double) args[0];
                powerWrites++;
                return null;
            }
            if ("getPower".equals(name)) return power;
            if ("setDirection".equals(name)) {
                direction = (DcMotorSimple.Direction) args[0];
                return null;
            }
            if ("getDirection".equals(name)) return direction;
            return hardwareDeviceOrDefault(method);
        }
    }

    private static final class ServoProbe {
        final Servo servo;
        double position;
        int positionWrites;
        private Servo.Direction direction = Servo.Direction.FORWARD;

        ServoProbe(double position) {
            this.position = position;
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
            if ("setPosition".equals(name)) {
                position = (Double) args[0];
                positionWrites++;
                return null;
            }
            if ("getPosition".equals(name)) return position;
            if ("setDirection".equals(name)) {
                direction = (Servo.Direction) args[0];
                return null;
            }
            if ("getDirection".equals(name)) return direction;
            return hardwareDeviceOrDefault(method);
        }
    }

    private static final class MotorProbe {
        final DcMotorEx motor;
        double power;
        private DcMotor.RunMode mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        private DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;

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
                return null;
            }
            if ("getPower".equals(name)) return power;
            if ("setMode".equals(name)) {
                mode = (DcMotor.RunMode) args[0];
                return null;
            }
            if ("getMode".equals(name)) return mode;
            if ("setDirection".equals(name)) {
                direction = (DcMotorSimple.Direction) args[0];
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
        if ("getDeviceName".equals(method.getName())) return "INPUT-01 probe";
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
