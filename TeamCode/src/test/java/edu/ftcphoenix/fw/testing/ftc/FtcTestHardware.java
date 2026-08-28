package edu.ftcphoenix.fw.testing.ftc;

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
import java.util.List;
import java.util.Map;

/**
 * Test-only FTC device registry for exercising real mechanism construction without a Control Hub.
 *
 * <p>Feedback is injected through each probe while actuator commands are recorded separately.
 * Motor feedback has a deterministic software baseline of zero ticks and zero ticks/second;
 * digital inputs begin HIGH. A scenario should set each input explicitly before that evidence
 * matters. Commands never manufacture encoder, velocity, or digital-input evidence.</p>
 */
public final class FtcTestHardware extends HardwareMap {
    private final Map<String, Object> devices = new HashMap<String, Object>();
    private final Map<String, MotorProbe> motors = new HashMap<String, MotorProbe>();
    private final Map<String, DigitalProbe> digitalInputs =
            new HashMap<String, DigitalProbe>();
    private final Map<String, CrServoProbe> crServos =
            new HashMap<String, CrServoProbe>();
    private final Map<String, ServoProbe> servos = new HashMap<String, ServoProbe>();
    private final List<String> events;
    private int lookupCalls;
    private String lastLookupName;

    /** Creates an empty registry without an event log. */
    public FtcTestHardware() {
        this(null);
    }

    /**
     * Creates an empty registry that appends motor-power writes to {@code events} when non-null.
     *
     * @param events optional shared ordering log
     */
    public FtcTestHardware(List<String> events) {
        super(null, null);
        this.events = events;
    }

    /** Registers one {@link DcMotorEx} under the FTC configuration name. */
    public MotorProbe addMotor(String name) {
        String key = availableKey(name);
        MotorProbe probe = new MotorProbe(key, events);
        devices.put(key, probe.motor);
        motors.put(key, probe);
        return probe;
    }

    /** Registers one electrical digital input, initially HIGH. */
    public DigitalProbe addDigitalInput(String name) {
        String key = availableKey(name);
        DigitalProbe probe = new DigitalProbe(key);
        devices.put(key, probe.channel);
        digitalInputs.put(key, probe);
        return probe;
    }

    /** Registers a second FTC name for an existing digital probe to test identity validation. */
    public void addDigitalInputAlias(String name, DigitalProbe probe) {
        String key = availableKey(name);
        DigitalProbe required = java.util.Objects.requireNonNull(probe, "probe is required");
        devices.put(key, required.channel);
        digitalInputs.put(key, required);
    }

    /** Registers one continuous-rotation servo. */
    public CrServoProbe addCrServo(String name) {
        String key = availableKey(name);
        CrServoProbe probe = new CrServoProbe(key);
        devices.put(key, probe.servo);
        crServos.put(key, probe);
        return probe;
    }

    /** Registers one standard positional servo. */
    public ServoProbe addServo(String name) {
        String key = availableKey(name);
        ServoProbe probe = new ServoProbe(key);
        devices.put(key, probe.servo);
        servos.put(key, probe);
        return probe;
    }

    /** Returns the registered motor probe or fails with its effective FTC name. */
    public MotorProbe motor(String name) {
        return requiredProbe(motors, name, "motor");
    }

    /** Returns the registered digital-input probe or fails with its effective FTC name. */
    public DigitalProbe digitalInput(String name) {
        return requiredProbe(digitalInputs, name, "digital input");
    }

    /** Returns the registered CR-servo probe or fails with its effective FTC name. */
    public CrServoProbe crServo(String name) {
        return requiredProbe(crServos, name, "CR servo");
    }

    /** Returns the registered standard-servo probe or fails with its effective FTC name. */
    public ServoProbe servo(String name) {
        return requiredProbe(servos, name, "servo");
    }

    /** Returns how many SDK lookups have crossed this registry. */
    public int lookupCalls() {
        return lookupCalls;
    }

    /** Returns the exact name supplied to the most recent SDK lookup, or null before any lookup. */
    public String lastLookupName() {
        return lastLookupName;
    }

    /** Returns the number of motor power writes recorded across every registered motor. */
    public int totalMotorPowerWrites() {
        int total = 0;
        for (MotorProbe probe : motors.values()) {
            total += probe.powerWrites();
        }
        return total;
    }

    /** Resolves a registered fake through the same API production FTC adapters use. */
    @Override
    public <T> T get(Class<? extends T> type, String name) {
        if (type == null) {
            throw new IllegalArgumentException("FTC test lookup type must not be null");
        }
        lookupCalls++;
        lastLookupName = name;
        String key = effectiveName(name);
        Object device = devices.get(key);
        if (device == null) {
            throw new IllegalArgumentException(
                    "No test " + type.getSimpleName() + " named '" + key + "'");
        }
        if (!type.isInstance(device)) {
            throw new IllegalArgumentException(
                    "Test device '" + key + "' is not a " + type.getSimpleName());
        }
        return type.cast(device);
    }

    private String availableKey(String name) {
        String key = effectiveName(name);
        if (devices.containsKey(key)) {
            throw new IllegalArgumentException(
                    "A test device named '" + key + "' is already registered after trimming");
        }
        return key;
    }

    private static String effectiveName(String name) {
        if (name == null || name.trim().isEmpty()) {
            throw new IllegalArgumentException("FTC test device name must not be blank");
        }
        return name.trim();
    }

    private static <T> T requiredProbe(Map<String, T> probes, String name, String kind) {
        String key = effectiveName(name);
        T probe = probes.get(key);
        if (probe == null) {
            throw new IllegalArgumentException(
                    "No test " + kind + " probe named '" + key + "'");
        }
        return probe;
    }

    /** Records motor commands and accepts independent encoder and velocity observations. */
    public static final class MotorProbe implements InvocationHandler {
        private final String label;
        private final List<String> events;
        private final DcMotorEx motor;
        private DcMotor.RunMode mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        private DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
        private DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT;
        private int currentPositionTicks;
        private int targetPositionTicks;
        private int targetPositionWrites;
        private double power;
        private int powerWrites;
        private double measuredVelocityTicksPerSec;
        private double commandedVelocityTicksPerSec;
        private int velocityWrites;
        private int velocityReadCalls;

        private MotorProbe(String label, List<String> events) {
            this.label = label;
            this.events = events;
            motor = (DcMotorEx) Proxy.newProxyInstance(
                    DcMotorEx.class.getClassLoader(),
                    new Class<?>[]{DcMotorEx.class},
                    this);
        }

        /** Injects the encoder position returned by later SDK reads, in ticks. */
        public void setCurrentPositionTicks(int currentPositionTicks) {
            this.currentPositionTicks = currentPositionTicks;
        }

        /** Injects the velocity returned by later SDK reads, in ticks per second. */
        public void setMeasuredVelocityTicksPerSec(double measuredVelocityTicksPerSec) {
            this.measuredVelocityTicksPerSec = measuredVelocityTicksPerSec;
        }

        /** Returns the independently injected encoder position, in ticks. */
        public int currentPositionTicks() {
            return currentPositionTicks;
        }

        /** Returns the most recent target-position command, in ticks. */
        public int targetPositionTicks() {
            return targetPositionTicks;
        }

        /** Returns the number of target-position commands submitted. */
        public int targetPositionWrites() {
            return targetPositionWrites;
        }

        /** Returns the most recent normalized power command. */
        public double power() {
            return power;
        }

        /** Returns the number of normalized power commands submitted. */
        public int powerWrites() {
            return powerWrites;
        }

        /** Returns the independently injected measured velocity, in ticks per second. */
        public double measuredVelocityTicksPerSec() {
            return measuredVelocityTicksPerSec;
        }

        /** Returns the most recent velocity command, in ticks per second. */
        public double commandedVelocityTicksPerSec() {
            return commandedVelocityTicksPerSec;
        }

        /** Returns the number of velocity commands submitted. */
        public int velocityWrites() {
            return velocityWrites;
        }

        /** Returns the number of SDK velocity feedback reads. */
        public int velocityReadCalls() {
            return velocityReadCalls;
        }

        /** Returns the current FTC motor run mode. */
        public DcMotor.RunMode mode() {
            return mode;
        }

        /** Returns the most recently configured FTC direction. */
        public DcMotorSimple.Direction direction() {
            return direction;
        }

        /** Returns the most recently configured zero-power behavior. */
        public DcMotor.ZeroPowerBehavior zeroPowerBehavior() {
            return zeroPowerBehavior;
        }

        @Override
        public Object invoke(Object proxy, Method method, Object[] args) {
            String name = method.getName();
            if (method.getDeclaringClass() == Object.class) {
                return objectMethod(proxy, name, args, "MotorProbe(" + label + ")");
            }
            if (matches(method, "setDirection", DcMotorSimple.Direction.class)) {
                direction = (DcMotorSimple.Direction) args[0];
                return null;
            }
            if (matches(method, "getDirection")) return direction;
            if (matches(method, "setMode", DcMotor.RunMode.class)) {
                mode = (DcMotor.RunMode) args[0];
                return null;
            }
            if (matches(method, "getMode")) return mode;
            if (matches(method, "setTargetPosition", int.class)) {
                targetPositionTicks = (Integer) args[0];
                targetPositionWrites++;
                return null;
            }
            if (matches(method, "getTargetPosition")) return targetPositionTicks;
            if (matches(method, "setPower", double.class)) {
                power = (Double) args[0];
                powerWrites++;
                if (events != null) {
                    events.add("power:" + label + ":" + power);
                }
                return null;
            }
            if (matches(method, "getPower")) return power;
            if (matches(method, "setVelocity", double.class)) {
                commandedVelocityTicksPerSec = (Double) args[0];
                velocityWrites++;
                return null;
            }
            if (matches(method, "getVelocity")) {
                velocityReadCalls++;
                return measuredVelocityTicksPerSec;
            }
            if (matches(method, "getCurrentPosition")) return currentPositionTicks;
            if (matches(method, "setZeroPowerBehavior", DcMotor.ZeroPowerBehavior.class)) {
                zeroPowerBehavior = (DcMotor.ZeroPowerBehavior) args[0];
                return null;
            }
            if (matches(method, "getZeroPowerBehavior")) return zeroPowerBehavior;
            throw unsupported("motor", label, method);
        }
    }

    /** Accepts an explicitly injected electrical HIGH/LOW level. */
    public static final class DigitalProbe implements InvocationHandler {
        private final String label;
        private final DigitalChannel channel;
        private DigitalChannel.Mode mode = DigitalChannel.Mode.INPUT;
        private boolean high = true;
        private int stateReadCalls;
        private int modeWriteCalls;
        private RuntimeException readFailure;
        private Runnable beforeNextRead;

        private DigitalProbe(String label) {
            this.label = label;
            channel = (DigitalChannel) Proxy.newProxyInstance(
                    DigitalChannel.class.getClassLoader(),
                    new Class<?>[]{DigitalChannel.class},
                    this);
        }

        /** Injects the electrical level returned by later SDK reads. */
        public void setHigh(boolean high) {
            this.high = high;
        }

        /** Returns the independently injected electrical HIGH/LOW level. */
        public boolean high() {
            return high;
        }

        /** Returns the current FTC digital-channel mode. */
        public DigitalChannel.Mode mode() {
            return mode;
        }

        /** Returns the number of SDK state reads. */
        public int stateReadCalls() {
            return stateReadCalls;
        }

        /** Returns the number of SDK input/output mode writes. */
        public int modeWriteCalls() {
            return modeWriteCalls;
        }

        /** Makes later state reads throw this exact failure; null restores successful reads. */
        public void setReadFailure(RuntimeException readFailure) {
            this.readFailure = readFailure;
        }

        /** Runs one callback immediately before the next state read. */
        public void beforeNextRead(Runnable callback) {
            beforeNextRead = callback;
        }

        @Override
        public Object invoke(Object proxy, Method method, Object[] args) {
            String name = method.getName();
            if (method.getDeclaringClass() == Object.class) {
                return objectMethod(proxy, name, args, "DigitalProbe(" + label + ")");
            }
            if (matches(method, "setMode", DigitalChannel.Mode.class)) {
                mode = (DigitalChannel.Mode) args[0];
                modeWriteCalls++;
                return null;
            }
            if (matches(method, "getMode")) return mode;
            if (matches(method, "getState")) {
                stateReadCalls++;
                Runnable callback = beforeNextRead;
                beforeNextRead = null;
                if (callback != null) callback.run();
                if (readFailure != null) throw readFailure;
                return high;
            }
            throw unsupported("digital input", label, method);
        }
    }

    /** Records CR-servo power and direction commands. */
    public static final class CrServoProbe implements InvocationHandler {
        private final String label;
        private final CRServo servo;
        private DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
        private double power;
        private int powerWrites;

        private CrServoProbe(String label) {
            this.label = label;
            servo = (CRServo) Proxy.newProxyInstance(
                    CRServo.class.getClassLoader(), new Class<?>[]{CRServo.class}, this);
        }

        /** Returns the most recent normalized power command. */
        public double power() {
            return power;
        }

        /** Returns the number of normalized power commands submitted. */
        public int powerWrites() {
            return powerWrites;
        }

        /** Returns the most recently configured FTC direction. */
        public DcMotorSimple.Direction direction() {
            return direction;
        }

        @Override
        public Object invoke(Object proxy, Method method, Object[] args) {
            String name = method.getName();
            if (method.getDeclaringClass() == Object.class) {
                return objectMethod(proxy, name, args, "CrServoProbe(" + label + ")");
            }
            if (matches(method, "setDirection", DcMotorSimple.Direction.class)) {
                direction = (DcMotorSimple.Direction) args[0];
                return null;
            }
            if (matches(method, "getDirection")) return direction;
            if (matches(method, "setPower", double.class)) {
                power = (Double) args[0];
                powerWrites++;
                return null;
            }
            if (matches(method, "getPower")) return power;
            throw unsupported("CR servo", label, method);
        }
    }

    /** Records standard-servo position and direction commands. */
    public static final class ServoProbe implements InvocationHandler {
        private final String label;
        private final Servo servo;
        private Servo.Direction direction = Servo.Direction.FORWARD;
        private double position;
        private int positionWrites;

        private ServoProbe(String label) {
            this.label = label;
            servo = (Servo) Proxy.newProxyInstance(
                    Servo.class.getClassLoader(), new Class<?>[]{Servo.class}, this);
        }

        /** Returns the most recent logical position command. */
        public double position() {
            return position;
        }

        /** Returns the number of logical position commands submitted. */
        public int positionWrites() {
            return positionWrites;
        }

        /** Returns the most recently configured FTC direction. */
        public Servo.Direction direction() {
            return direction;
        }

        @Override
        public Object invoke(Object proxy, Method method, Object[] args) {
            String name = method.getName();
            if (method.getDeclaringClass() == Object.class) {
                return objectMethod(proxy, name, args, "ServoProbe(" + label + ")");
            }
            if (matches(method, "setDirection", Servo.Direction.class)) {
                direction = (Servo.Direction) args[0];
                return null;
            }
            if (matches(method, "getDirection")) return direction;
            if (matches(method, "setPosition", double.class)) {
                position = (Double) args[0];
                positionWrites++;
                return null;
            }
            if (matches(method, "getPosition")) return position;
            throw unsupported("servo", label, method);
        }
    }

    private static boolean matches(Method method, String name, Class<?>... parameterTypes) {
        if (!name.equals(method.getName())) return false;
        Class<?>[] actual = method.getParameterTypes();
        if (actual.length != parameterTypes.length) return false;
        for (int index = 0; index < actual.length; index++) {
            if (actual[index] != parameterTypes[index]) return false;
        }
        return true;
    }

    private static UnsupportedOperationException unsupported(
            String kind,
            String label,
            Method method) {
        StringBuilder signature = new StringBuilder(method.getName()).append('(');
        Class<?>[] parameters = method.getParameterTypes();
        for (int index = 0; index < parameters.length; index++) {
            if (index > 0) signature.append(", ");
            signature.append(parameters[index].getSimpleName());
        }
        signature.append(')');
        return new UnsupportedOperationException(
                "FtcTestHardware " + kind + " '" + label + "' does not model " + signature
                        + "; add explicit input or output evidence before using this SDK call");
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

}
