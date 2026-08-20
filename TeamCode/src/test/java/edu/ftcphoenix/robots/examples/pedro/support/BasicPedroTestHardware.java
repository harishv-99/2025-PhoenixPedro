package edu.ftcphoenix.robots.examples.pedro.support;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.HashMap;
import java.util.Map;

/** Package-private FTC probes shared by the focused Basic Pedro tests. */
public final class BasicPedroTestHardware {

    private BasicPedroTestHardware() {
        // Test fixture.
    }

    /** In-memory FTC map with observable motor lookup and command effects. */
    public static final class HardwareMapProbe extends HardwareMap {
        private final Map<String, MotorProbe> motors = new HashMap<String, MotorProbe>();
        private int lookupCalls;
        private String lastLookupName;

        public HardwareMapProbe() {
            super(null, null);
        }

        public MotorProbe addMotor(String name) {
            MotorProbe probe = new MotorProbe(name);
            motors.put(name.trim(), probe);
            return probe;
        }

        public int lookupCalls() {
            return lookupCalls;
        }

        public String lastLookupName() {
            return lastLookupName;
        }

        public int totalPowerWrites() {
            int total = 0;
            for (MotorProbe probe : motors.values()) {
                total += probe.powerWrites;
            }
            return total;
        }

        @Override
        public <T> T get(Class<? extends T> type, String name) {
            lookupCalls++;
            lastLookupName = name;
            MotorProbe probe = motors.get(name == null ? null : name.trim());
            if (probe == null || !type.isInstance(probe.motor)) {
                throw new IllegalArgumentException(
                        "No test " + type.getSimpleName() + " named " + name);
            }
            return type.cast(probe.motor);
        }
    }

    /** Dynamic SDK motor with the raw-power behavior used by the example mechanism. */
    public static final class MotorProbe implements InvocationHandler {
        private final String label;
        private final DcMotorEx motor;
        private DcMotor.RunMode mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        private DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
        private double lastPower;
        private int powerWrites;

        private MotorProbe(String label) {
            this.label = label;
            motor = (DcMotorEx) Proxy.newProxyInstance(
                    DcMotorEx.class.getClassLoader(),
                    new Class<?>[]{DcMotorEx.class},
                    this);
        }

        public double lastPower() {
            return lastPower;
        }

        public int powerWrites() {
            return powerWrites;
        }

        public DcMotorSimple.Direction direction() {
            return direction;
        }

        @Override
        public Object invoke(Object proxy, Method method, Object[] args) {
            String name = method.getName();
            if (method.getDeclaringClass() == Object.class) {
                return objectMethod(proxy, name, args);
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
                return null;
            }
            if ("getPower".equals(name)) {
                return lastPower;
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

        private Object objectMethod(Object proxy, String methodName, Object[] args) {
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
