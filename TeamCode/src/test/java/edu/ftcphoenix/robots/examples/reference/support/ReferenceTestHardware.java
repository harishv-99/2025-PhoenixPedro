package edu.ftcphoenix.robots.examples.reference.support;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.HashMap;
import java.util.Map;

/** Small in-memory FTC device registry for the managed Reference example tests. */
public final class ReferenceTestHardware {
    private ReferenceTestHardware() {
        // Test fixture.
    }

    /** Hardware map with observable lookup count and configurable device probes. */
    public static final class HardwareMapProbe extends HardwareMap {
        private final Map<String, Object> devices = new HashMap<String, Object>();
        private final Map<String, MotorProbe> motors = new HashMap<String, MotorProbe>();
        private final Map<String, DigitalProbe> digitalChannels =
                new HashMap<String, DigitalProbe>();
        private final Map<String, CrServoProbe> crServos =
                new HashMap<String, CrServoProbe>();
        private final Map<String, ServoProbe> servos = new HashMap<String, ServoProbe>();
        private int lookupCalls;

        public HardwareMapProbe() {
            super(null, null);
        }

        public MotorProbe addMotor(String name) {
            MotorProbe probe = new MotorProbe(name);
            add(name, probe.motor);
            motors.put(key(name), probe);
            return probe;
        }

        public DigitalProbe addDigital(String name) {
            DigitalProbe probe = new DigitalProbe(name);
            add(name, probe.channel);
            digitalChannels.put(key(name), probe);
            return probe;
        }

        public CrServoProbe addCrServo(String name) {
            CrServoProbe probe = new CrServoProbe(name);
            add(name, probe.servo);
            crServos.put(key(name), probe);
            return probe;
        }

        public ServoProbe addServo(String name) {
            ServoProbe probe = new ServoProbe(name);
            add(name, probe.servo);
            servos.put(key(name), probe);
            return probe;
        }

        public MotorProbe motor(String name) {
            return motors.get(key(name));
        }

        public DigitalProbe digital(String name) {
            return digitalChannels.get(key(name));
        }

        public CrServoProbe crServo(String name) {
            return crServos.get(key(name));
        }

        public ServoProbe servo(String name) {
            return servos.get(key(name));
        }

        public int lookupCalls() {
            return lookupCalls;
        }

        @Override
        public <T> T get(Class<? extends T> type, String name) {
            lookupCalls++;
            Object device = devices.get(key(name));
            if (device == null || !type.isInstance(device)) {
                throw new IllegalArgumentException(
                        "No test " + type.getSimpleName() + " named " + name);
            }
            return type.cast(device);
        }

        private void add(String name, Object device) {
            devices.put(key(name), device);
        }
    }

    /** Motor behavior needed by the Reference position and velocity Plants. */
    public static final class MotorProbe implements InvocationHandler {
        private final DcMotorEx motor;
        private DcMotor.RunMode mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        private DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
        private DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT;
        private int currentPosition;
        private int targetPosition;
        private double power;
        private int powerWrites;
        private double measuredVelocity;
        private double commandedVelocity;
        private int velocityReadCalls;

        private MotorProbe(String name) {
            motor = (DcMotorEx) Proxy.newProxyInstance(
                    DcMotorEx.class.getClassLoader(),
                    new Class<?>[]{DcMotorEx.class},
                    this);
        }

        public void setCurrentPosition(int currentPosition) {
            this.currentPosition = currentPosition;
        }

        public void setVelocity(double velocity) {
            this.measuredVelocity = velocity;
        }

        public int targetPosition() {
            return targetPosition;
        }

        public double power() {
            return power;
        }

        public int powerWrites() {
            return powerWrites;
        }

        public double commandedVelocity() {
            return commandedVelocity;
        }

        public int velocityReadCalls() {
            return velocityReadCalls;
        }

        @Override
        public Object invoke(Object proxy, Method method, Object[] args) {
            String name = method.getName();
            if (method.getDeclaringClass() == Object.class) {
                return objectMethod(proxy, name, args, "MotorProbe");
            }
            if ("setDirection".equals(name)) {
                direction = (DcMotorSimple.Direction) args[0];
                return null;
            }
            if ("getDirection".equals(name)) return direction;
            if ("setMode".equals(name)) {
                mode = (DcMotor.RunMode) args[0];
                if (mode == DcMotor.RunMode.STOP_AND_RESET_ENCODER) currentPosition = 0;
                return null;
            }
            if ("getMode".equals(name)) return mode;
            if ("setTargetPosition".equals(name)) {
                targetPosition = (Integer) args[0];
                return null;
            }
            if ("getTargetPosition".equals(name)) return targetPosition;
            if ("setPower".equals(name)) {
                power = (Double) args[0];
                powerWrites++;
                return null;
            }
            if ("getPower".equals(name)) return power;
            if ("setVelocity".equals(name)) {
                commandedVelocity = (Double) args[0];
                return null;
            }
            if ("getVelocity".equals(name)) {
                velocityReadCalls++;
                return measuredVelocity;
            }
            if ("getCurrentPosition".equals(name)) return currentPosition;
            if ("isBusy".equals(name)) return currentPosition != targetPosition;
            if ("setZeroPowerBehavior".equals(name)) {
                zeroPowerBehavior = (DcMotor.ZeroPowerBehavior) args[0];
                return null;
            }
            if ("getZeroPowerBehavior".equals(name)) return zeroPowerBehavior;
            return defaultValue(method.getReturnType());
        }
    }

    /** Active-high/active-low digital input probe; HIGH is the default unpressed level. */
    public static final class DigitalProbe implements InvocationHandler {
        private final DigitalChannel channel;
        private DigitalChannel.Mode mode = DigitalChannel.Mode.INPUT;
        private boolean high = true;

        private DigitalProbe(String name) {
            channel = (DigitalChannel) Proxy.newProxyInstance(
                    DigitalChannel.class.getClassLoader(),
                    new Class<?>[]{DigitalChannel.class},
                    this);
        }

        public void setHigh(boolean high) {
            this.high = high;
        }

        @Override
        public Object invoke(Object proxy, Method method, Object[] args) {
            String name = method.getName();
            if (method.getDeclaringClass() == Object.class) {
                return objectMethod(proxy, name, args, "DigitalProbe");
            }
            if ("setMode".equals(name)) {
                mode = (DigitalChannel.Mode) args[0];
                return null;
            }
            if ("getMode".equals(name)) return mode;
            if ("getState".equals(name)) return high;
            return defaultValue(method.getReturnType());
        }
    }

    /** CR-servo probe retaining the last commanded power. */
    public static final class CrServoProbe implements InvocationHandler {
        private final CRServo servo;
        private DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
        private double power;

        private CrServoProbe(String name) {
            servo = (CRServo) Proxy.newProxyInstance(
                    CRServo.class.getClassLoader(), new Class<?>[]{CRServo.class}, this);
        }

        public double power() {
            return power;
        }

        @Override
        public Object invoke(Object proxy, Method method, Object[] args) {
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
                power = (Double) args[0];
                return null;
            }
            if ("getPower".equals(name)) return power;
            return defaultValue(method.getReturnType());
        }
    }

    /** Standard-servo probe retaining the last commanded logical position. */
    public static final class ServoProbe implements InvocationHandler {
        private final Servo servo;
        private Servo.Direction direction = Servo.Direction.FORWARD;
        private double position;

        private ServoProbe(String name) {
            servo = (Servo) Proxy.newProxyInstance(
                    Servo.class.getClassLoader(), new Class<?>[]{Servo.class}, this);
        }

        public double position() {
            return position;
        }

        @Override
        public Object invoke(Object proxy, Method method, Object[] args) {
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
                position = (Double) args[0];
                return null;
            }
            if ("getPosition".equals(name)) return position;
            return defaultValue(method.getReturnType());
        }
    }

    private static String key(String name) {
        return name == null ? null : name.trim();
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
