package edu.ftcphoenix.fw.actuation;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.junit.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.HashMap;
import java.util.Map;

import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.ftc.FtcActuators;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies that the FTC Plant gateway rejects malformed ranges before touching hardware. */
public final class FtcActuatorsRangeValidationTest {

    private static final double EPSILON = 1.0e-12;

    @Test
    public void invalidFtcBoundsDoNotTouchHardwareOrPoisonRetainedRangeStage() {
        RecordingHardwareMap hardwareMap = new RecordingHardwareMap();
        ServoProbe servo = hardwareMap.addServo("claw");
        FtcActuators.ServoPositionBoundsStep retained = FtcActuators.plant(hardwareMap)
                .servo("claw", Direction.FORWARD)
                .position()
                .nonPeriodic();

        assertIllegalArgumentContains(
                () -> retained.bounded(Double.NaN, 1.0),
                "ScalarRange.bounded", "finite", "minValue", "NaN");
        hardwareMap.assertUntouched(servo);

        assertIllegalArgumentContains(
                () -> retained.bounded(0.0, Double.POSITIVE_INFINITY),
                "ScalarRange.bounded", "finite", "maxValue", "Infinity");
        hardwareMap.assertUntouched(servo);

        assertIllegalArgumentContains(
                () -> retained.bounded(1.0, 0.0),
                "ScalarRange.bounded", "minValue", "1.0", "maxValue", "0.0");
        hardwareMap.assertUntouched(servo);

        PositionPlant plant = retained
                .bounded(0.0, 1.0)
                .nativeUnits()
                .targetFromNewCommand(0.25)
                .build();

        assertEquals(1, hardwareMap.lookupCalls);
        assertEquals(1, servo.directionWrites);
        assertEquals(0, servo.positionWrites);

        plant.update(new ManualLoopClock().clock());
        assertEquals(1, servo.positionWrites);
        assertEquals(0.25, servo.commandedPosition, EPSILON);
    }

    private static void assertIllegalArgumentContains(Runnable action, String... fragments) {
        try {
            action.run();
            fail("Expected IllegalArgumentException");
        } catch (IllegalArgumentException expected) {
            String message = expected.getMessage();
            assertTrue("Expected exception message", message != null);
            for (String fragment : fragments) {
                assertTrue(message, message.contains(fragment));
            }
        }
    }

    /** In-memory HardwareMap that records whether staged construction resolved the servo. */
    private static final class RecordingHardwareMap extends HardwareMap {
        private final Map<String, HardwareDevice> devices = new HashMap<>();
        private int lookupCalls;

        private RecordingHardwareMap() {
            super(null, null);
        }

        private ServoProbe addServo(String name) {
            ServoProbe probe = new ServoProbe();
            devices.put(name, probe.servo);
            return probe;
        }

        @Override
        public <T> T get(Class<? extends T> type, String name) {
            lookupCalls++;
            HardwareDevice device = devices.get(name);
            if (device == null || !type.isInstance(device)) {
                throw new IllegalArgumentException("No test " + type.getSimpleName()
                        + " named " + name);
            }
            return type.cast(device);
        }

        private void assertUntouched(ServoProbe servo) {
            assertEquals(0, lookupCalls);
            assertEquals(0, servo.directionWrites);
            assertEquals(0, servo.positionWrites);
            assertTrue(Double.isNaN(servo.commandedPosition));
        }
    }

    /** Dynamic FTC servo with observable configuration and output writes. */
    private static final class ServoProbe {
        private double commandedPosition = Double.NaN;
        private Servo.Direction direction = Servo.Direction.FORWARD;
        private int directionWrites;
        private int positionWrites;
        private final Servo servo;

        private ServoProbe() {
            servo = (Servo) Proxy.newProxyInstance(
                    Servo.class.getClassLoader(),
                    new Class<?>[]{Servo.class},
                    this::invoke);
        }

        private Object invoke(Object proxy, Method method, Object[] args) {
            String name = method.getName();
            if (method.getDeclaringClass() == Object.class) {
                if ("toString".equals(name)) return "ServoProbe";
                if ("hashCode".equals(name)) return System.identityHashCode(proxy);
                if ("equals".equals(name)) return proxy == args[0];
                return null;
            }
            if ("setPosition".equals(name)) {
                commandedPosition = (double) args[0];
                positionWrites++;
                return null;
            }
            if ("getPosition".equals(name)) return commandedPosition;
            if ("setDirection".equals(name)) {
                direction = (Servo.Direction) args[0];
                directionWrites++;
                return null;
            }
            if ("getDirection".equals(name)) return direction;
            if ("getManufacturer".equals(name)) return HardwareDevice.Manufacturer.Other;
            if ("getDeviceName".equals(name)) return "ServoProbe";
            if ("getConnectionInfo".equals(name)) return "test";
            if ("getVersion".equals(name)) return 1;
            return defaultValue(method.getReturnType());
        }
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
