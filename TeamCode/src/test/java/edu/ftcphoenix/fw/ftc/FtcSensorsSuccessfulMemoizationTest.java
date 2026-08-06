package edu.ftcphoenix.fw.ftc;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.junit.Test;

import java.lang.reflect.Proxy;

import edu.ftcphoenix.fw.core.color.Rgba;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.fail;

/** Proves FTC sensor adapters inherit successful-only memoization from source decorators. */
public final class FtcSensorsSuccessfulMemoizationTest {

    private static final double EPSILON = 1.0e-12;

    @Test
    public void scalarSensorFailureCanRetryInTheSameCycle() {
        ManualLoopClock time = new ManualLoopClock();
        RuntimeException failure = new RuntimeException("distance read failed");
        int[] calls = {0};
        DistanceSensor sensor = (DistanceSensor) Proxy.newProxyInstance(
                DistanceSensor.class.getClassLoader(),
                new Class<?>[]{DistanceSensor.class},
                (proxy, method, args) -> {
                    if ("getDistance".equals(method.getName())) {
                        calls[0]++;
                        if (calls[0] == 1) {
                            throw failure;
                        }
                        return 42.5;
                    }
                    return defaultValue(method.getReturnType());
                }
        );
        ScalarSource source = FtcSensors.distanceCm(sensor);

        assertSame(failure, expectRuntime(() -> source.getAsDouble(time.clock())));
        assertEquals(42.5, source.getAsDouble(time.clock()), EPSILON);
        assertEquals(42.5, source.getAsDouble(time.clock()), EPSILON);
        assertEquals(2, calls[0]);
    }

    @Test
    public void objectSensorFailureDoesNotPublishNullOrAStaleColor() {
        ManualLoopClock time = new ManualLoopClock();
        RuntimeException failure = new RuntimeException("blue channel failed");
        int[] blueCalls = {0};
        ColorSensor sensor = (ColorSensor) Proxy.newProxyInstance(
                ColorSensor.class.getClassLoader(),
                new Class<?>[]{ColorSensor.class},
                (proxy, method, args) -> {
                    switch (method.getName()) {
                        case "red":
                            return 10;
                        case "green":
                            return 20;
                        case "blue":
                            blueCalls[0]++;
                            if (blueCalls[0] == 1) {
                                throw failure;
                            }
                            return 30;
                        case "alpha":
                            return 40;
                        default:
                            return defaultValue(method.getReturnType());
                    }
                }
        );
        Source<Rgba> source = FtcSensors.rgba(sensor);

        assertSame(failure, expectRuntime(() -> source.get(time.clock())));
        Rgba color = source.get(time.clock());
        assertEquals(10, color.r);
        assertEquals(20, color.g);
        assertEquals(30, color.b);
        assertEquals(40, color.a);
        assertSame(color, source.get(time.clock()));
        assertEquals(2, blueCalls[0]);
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

    private static Object defaultValue(Class<?> type) {
        if (!type.isPrimitive() || type == void.class) {
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
