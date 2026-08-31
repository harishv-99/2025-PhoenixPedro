package edu.ftcsushi.fw.ftc;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.junit.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.lang.reflect.Proxy;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Deque;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.source.ScalarSource;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the explicit-amps FTC motor-current boundary and its per-source memoization. */
public final class FtcSensorsMotorCurrentTest {

    @Test
    public void publicApiContainsExactlyTheDirectAndNamedScalarSourceOverloads() throws Exception {
        Set<String> actual = new TreeSet<>();
        for (Method method : FtcSensors.class.getDeclaredMethods()) {
            if (!"motorCurrentAmps".equals(method.getName())
                    || !Modifier.isPublic(method.getModifiers())) {
                continue;
            }

            assertTrue(Modifier.isStatic(method.getModifiers()));
            assertEquals(ScalarSource.class, method.getReturnType());
            actual.add(methodSignature(method));
        }

        assertEquals(new TreeSet<>(Arrays.asList(
                "motorCurrentAmps(DcMotorEx)",
                "motorCurrentAmps(HardwareMap,String)")), actual);

        assertEquals(ScalarSource.class,
                FtcSensors.class.getMethod("motorCurrentAmps", DcMotorEx.class).getReturnType());
        assertEquals(ScalarSource.class,
                FtcSensors.class.getMethod(
                        "motorCurrentAmps", HardwareMap.class, String.class).getReturnType());
    }

    @Test
    public void constructionValidatesInOrderAndPerformsOneExactEagerLookupWithoutReading() {
        IllegalArgumentException directNull = expectIllegalArgument(
                () -> FtcSensors.motorCurrentAmps((DcMotorEx) null));
        assertEquals("motor is required", directNull.getMessage());

        IllegalArgumentException mapFirst = expectIllegalArgument(
                () -> FtcSensors.motorCurrentAmps((HardwareMap) null, null));
        assertEquals("HardwareMap is required", mapFirst.getMessage());

        TestHardwareMap nullNameMap = new TestHardwareMap();
        IllegalArgumentException nullName = expectIllegalArgument(
                () -> FtcSensors.motorCurrentAmps(nullNameMap, null));
        assertEquals("name is required", nullName.getMessage());
        assertEquals(0, nullNameMap.lookupCount);

        MotorProbe motor = new MotorProbe(3.25);
        ScalarSource direct = FtcSensors.motorCurrentAmps(motor.motor());
        assertNotNull(direct);
        assertEquals(0, motor.currentReadCount);
        assertTrue(motor.otherMotorCalls.isEmpty());

        TestHardwareMap map = new TestHardwareMap();
        map.put(" intake ", motor.motor());
        ScalarSource named = FtcSensors.motorCurrentAmps(map, " intake ");

        assertNotNull(named);
        assertEquals(1, map.lookupCount);
        assertEquals(DcMotorEx.class, map.lookupTypes.get(0));
        assertEquals(" intake ", map.lookupNames.get(0));
        assertEquals(0, motor.currentReadCount);
        assertTrue(motor.otherMotorCalls.isEmpty());

        ManualLoopClock clock = new ManualLoopClock();
        assertEquals(3.25, named.getAsDouble(clock.clock()), 0.0);
        assertEquals(1, motor.currentReadCount);
        assertEquals(1, map.lookupCount);

        named.reset();
        assertEquals(1, map.lookupCount);
        motor.currentAmps = 4.25;
        assertEquals(4.25, named.getAsDouble(clock.clock()), 0.0);
        assertEquals(1, map.lookupCount);

        clock.nextCycle(0.02);
        motor.currentAmps = 5.25;
        assertEquals(5.25, named.getAsDouble(clock.clock()), 0.0);
        assertEquals(1, map.lookupCount);
        assertEquals(3, motor.currentReadCount);
        assertAllAmps(motor.requestedUnits);
        assertTrue(motor.otherMotorCalls.isEmpty());
    }

    @Test
    public void namedLookupLeavesMissingAndWrongTypeFailuresToHardwareMap() {
        TestHardwareMap missingMap = new TestHardwareMap();
        IllegalArgumentException missing = expectIllegalArgument(
                () -> FtcSensors.motorCurrentAmps(missingMap, "missing"));
        assertSame(missingMap.lastLookupFailure, missing);
        assertEquals("No test device named missing", missing.getMessage());
        assertEquals(1, missingMap.lookupCount);
        assertEquals(DcMotorEx.class, missingMap.lookupTypes.get(0));
        assertEquals("missing", missingMap.lookupNames.get(0));

        TestHardwareMap wrongTypeMap = new TestHardwareMap();
        wrongTypeMap.put("plain", hardwareDevice("PlainHardwareDevice"));
        IllegalArgumentException wrongType = expectIllegalArgument(
                () -> FtcSensors.motorCurrentAmps(wrongTypeMap, "plain"));
        assertSame(wrongTypeMap.lastLookupFailure, wrongType);
        assertEquals("plain is not a DcMotorEx", wrongType.getMessage());
        assertEquals(1, wrongTypeMap.lookupCount);
        assertEquals(DcMotorEx.class, wrongTypeMap.lookupTypes.get(0));
        assertEquals("plain", wrongTypeMap.lookupNames.get(0));
    }

    @Test
    public void derivedConsumersShareOneAmpsReadAndNextCycleReadsAgain() {
        MotorProbe motor = new MotorProbe(4.25);
        ScalarSource current = FtcSensors.motorCurrentAmps(motor.motor());
        BooleanSource aboveFourAmps = current.above(4.0);
        ScalarSource doubled = current.scaled(2.0);
        ManualLoopClock clock = new ManualLoopClock();

        assertTrue(aboveFourAmps.getAsBoolean(clock.clock()));
        motor.currentAmps = 9.0;
        assertEquals(8.5, doubled.getAsDouble(clock.clock()), 0.0);
        assertEquals(1, motor.currentReadCount);

        clock.nextCycle(0.02);
        assertEquals(18.0, doubled.getAsDouble(clock.clock()), 0.0);
        assertTrue(aboveFourAmps.getAsBoolean(clock.clock()));
        assertEquals(2, motor.currentReadCount);
        assertAllAmps(motor.requestedUnits);
    }

    @Test
    public void independentlyConstructedSourcesKeepIndependentMemosAndResetIsolation() {
        MotorProbe motor = new MotorProbe(1.5);
        ScalarSource first = FtcSensors.motorCurrentAmps(motor.motor());
        ScalarSource second = FtcSensors.motorCurrentAmps(motor.motor());
        ManualLoopClock clock = new ManualLoopClock();

        assertEquals(1.5, first.getAsDouble(clock.clock()), 0.0);
        motor.currentAmps = 2.5;
        assertEquals(2.5, second.getAsDouble(clock.clock()), 0.0);
        assertEquals(2, motor.currentReadCount);

        motor.currentAmps = 3.5;
        first.reset();
        assertEquals(2, motor.currentReadCount);
        assertEquals(2.5, second.getAsDouble(clock.clock()), 0.0);
        assertEquals(2, motor.currentReadCount);
        assertEquals(3.5, first.getAsDouble(clock.clock()), 0.0);
        assertEquals(3, motor.currentReadCount);
        assertEquals(2.5, second.getAsDouble(clock.clock()), 0.0);
        assertAllAmps(motor.requestedUnits);
    }

    @Test
    public void resetAllowsAFreshSameCycleReadWithoutProbingDuringReset() {
        MotorProbe motor = new MotorProbe(5.0);
        ScalarSource current = FtcSensors.motorCurrentAmps(motor.motor());
        ManualLoopClock clock = new ManualLoopClock();

        assertEquals(5.0, current.getAsDouble(clock.clock()), 0.0);
        motor.currentAmps = 6.0;
        current.reset();

        assertEquals(1, motor.currentReadCount);
        assertTrue(motor.otherMotorCalls.isEmpty());
        assertEquals(6.0, current.getAsDouble(clock.clock()), 0.0);
        assertEquals(6.0, current.getAsDouble(clock.clock()), 0.0);
        assertEquals(2, motor.currentReadCount);
        assertAllAmps(motor.requestedUnits);
        assertTrue(motor.otherMotorCalls.isEmpty());
    }

    @Test
    public void escapingFailurePublishesNeitherSeededValueNorZeroAndCanRetrySameCycle() {
        RuntimeException failure = new RuntimeException("current read failed");
        MotorProbe motor = new MotorProbe(99.0);
        motor.enqueueValue(7.25);
        motor.enqueueFailure(failure);
        motor.enqueueValue(8.75);
        ScalarSource current = FtcSensors.motorCurrentAmps(motor.motor());
        ManualLoopClock clock = new ManualLoopClock();

        assertEquals(7.25, current.getAsDouble(clock.clock()), 0.0);
        clock.nextCycle(0.02);
        assertSame(failure, expectRuntime(() -> current.getAsDouble(clock.clock())));

        assertEquals(8.75, current.getAsDouble(clock.clock()), 0.0);
        assertEquals(8.75, current.getAsDouble(clock.clock()), 0.0);
        assertEquals(3, motor.currentReadCount);
        assertAllAmps(motor.requestedUnits);
    }

    @Test
    public void returnedZeroNegativeAndNonFiniteValuesAreForwardedAndMemoized() {
        double[] values = {
                0.0,
                -3.25,
                Double.NaN,
                Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY
        };
        MotorProbe motor = new MotorProbe(values[0]);
        ScalarSource current = FtcSensors.motorCurrentAmps(motor.motor());
        ManualLoopClock clock = new ManualLoopClock();

        for (int i = 0; i < values.length; i++) {
            if (i > 0) {
                clock.nextCycle(0.02);
            }
            motor.currentAmps = values[i];

            assertDoubleEquals(values[i], current.getAsDouble(clock.clock()));
            motor.currentAmps = 123.0;
            assertDoubleEquals(values[i], current.getAsDouble(clock.clock()));
            assertEquals(i + 1, motor.currentReadCount);
        }

        assertAllAmps(motor.requestedUnits);
    }

    @Test
    public void samplingAndResetNeverCallOtherMotorOrCurrentAlertOperations() {
        MotorProbe motor = new MotorProbe(2.0);
        ScalarSource current = FtcSensors.motorCurrentAmps(motor.motor());
        ManualLoopClock clock = new ManualLoopClock();

        assertEquals(2.0, current.getAsDouble(clock.clock()), 0.0);
        current.reset();
        assertEquals(2.0, current.getAsDouble(clock.clock()), 0.0);

        assertEquals(2, motor.currentReadCount);
        assertTrue("Unexpected motor calls: " + motor.otherMotorCalls,
                motor.otherMotorCalls.isEmpty());
    }

    @Test
    public void structuralDiagnosticsDoNotReadOrOperateTheMotor() {
        MotorProbe motor = new MotorProbe(4.0);
        ScalarSource current = FtcSensors.motorCurrentAmps(motor.motor());
        CapturingDebugSink debug = new CapturingDebugSink();

        current.debugDump(debug, "current");

        assertEquals(2, debug.data.size());
        assertEquals("MemoizedScalar", debug.data.get("current.class"));
        assertEquals("RawScalar", debug.data.get("current.src.class"));
        assertEquals(0, motor.currentReadCount);
        assertTrue(motor.otherMotorCalls.isEmpty());
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

    private static IllegalArgumentException expectIllegalArgument(Runnable action) {
        try {
            action.run();
            fail("Expected IllegalArgumentException");
            return null;
        } catch (IllegalArgumentException expected) {
            return expected;
        }
    }

    private static RuntimeException expectRuntime(Runnable action) {
        try {
            action.run();
            fail("Expected RuntimeException");
            return null;
        } catch (RuntimeException expected) {
            return expected;
        }
    }

    private static void assertDoubleEquals(double expected, double actual) {
        if (Double.isNaN(expected)) {
            assertTrue(Double.isNaN(actual));
            return;
        }
        assertEquals(expected, actual, 0.0);
    }

    private static void assertAllAmps(List<CurrentUnit> units) {
        assertTrue("Expected at least one current read", !units.isEmpty());
        for (CurrentUnit unit : units) {
            assertSame(CurrentUnit.AMPS, unit);
        }
    }

    private static HardwareDevice hardwareDevice(String label) {
        return (HardwareDevice) Proxy.newProxyInstance(
                HardwareDevice.class.getClassLoader(),
                new Class<?>[]{HardwareDevice.class},
                (proxy, method, args) -> {
                    if (method.getDeclaringClass() == Object.class) {
                        return objectMethod(proxy, method, args, label);
                    }
                    return defaultValue(method.getReturnType());
                });
    }

    /** In-memory map with observable lookup type, name, order, and count. */
    private static final class TestHardwareMap extends HardwareMap {
        private final Map<String, HardwareDevice> devices = new HashMap<>();
        private final List<Class<?>> lookupTypes = new ArrayList<>();
        private final List<String> lookupNames = new ArrayList<>();
        private IllegalArgumentException lastLookupFailure;
        private int lookupCount;

        private TestHardwareMap() {
            super(null, null);
        }

        @Override
        public void put(String name, HardwareDevice device) {
            devices.put(name, device);
        }

        @Override
        public <T> T get(Class<? extends T> type, String name) {
            lookupCount++;
            lookupTypes.add(type);
            lookupNames.add(name);

            HardwareDevice device = devices.get(name);
            if (device == null) {
                lastLookupFailure = new IllegalArgumentException("No test device named " + name);
                throw lastLookupFailure;
            }
            if (!type.isInstance(device)) {
                lastLookupFailure = new IllegalArgumentException(
                        name + " is not a " + type.getSimpleName());
                throw lastLookupFailure;
            }
            return type.cast(device);
        }
    }

    /** Dynamic FTC motor with programmable current outcomes and complete non-current call capture. */
    private static final class MotorProbe {
        private final Deque<Object> currentOutcomes = new ArrayDeque<>();
        private final List<CurrentUnit> requestedUnits = new ArrayList<>();
        private final List<String> otherMotorCalls = new ArrayList<>();
        private final DcMotorEx motor;
        private double currentAmps;
        private int currentReadCount;

        private MotorProbe(double currentAmps) {
            this.currentAmps = currentAmps;
            motor = (DcMotorEx) Proxy.newProxyInstance(
                    DcMotorEx.class.getClassLoader(),
                    new Class<?>[]{DcMotorEx.class},
                    (proxy, method, args) -> invoke(proxy, method, args));
        }

        private DcMotorEx motor() {
            return motor;
        }

        private void enqueueValue(double value) {
            currentOutcomes.addLast(Double.valueOf(value));
        }

        private void enqueueFailure(RuntimeException failure) {
            currentOutcomes.addLast(failure);
        }

        private Object invoke(Object proxy, Method method, Object[] args) {
            if (method.getDeclaringClass() == Object.class) {
                return objectMethod(proxy, method, args, "MotorCurrentProbe");
            }
            if ("getCurrent".equals(method.getName())) {
                currentReadCount++;
                requestedUnits.add((CurrentUnit) args[0]);
                if (!currentOutcomes.isEmpty()) {
                    Object outcome = currentOutcomes.removeFirst();
                    if (outcome instanceof RuntimeException) {
                        throw (RuntimeException) outcome;
                    }
                    return ((Double) outcome).doubleValue();
                }
                return currentAmps;
            }

            otherMotorCalls.add(method.getName());
            return defaultValue(method.getReturnType());
        }
    }

    private static final class CapturingDebugSink implements DebugSink {
        private final Map<String, Object> data = new LinkedHashMap<>();

        @Override
        public DebugSink addData(String key, Object value) {
            data.put(key, value);
            return this;
        }

        @Override
        public DebugSink addLine(String text) {
            return this;
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
        if (!returnType.isPrimitive() || returnType == void.class) return null;
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
