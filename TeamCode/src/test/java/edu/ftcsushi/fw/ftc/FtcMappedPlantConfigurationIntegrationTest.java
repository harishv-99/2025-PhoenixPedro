package edu.ftcsushi.fw.ftc;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.junit.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.HashMap;
import java.util.Map;

import edu.ftcsushi.fw.actuation.Plant;
import edu.ftcsushi.fw.actuation.PlantTargetContext;
import edu.ftcsushi.fw.actuation.PlantTargetResolution;
import edu.ftcsushi.fw.actuation.PlantTargetResolver;
import edu.ftcsushi.fw.actuation.PlantTargets;
import edu.ftcsushi.fw.actuation.PositionPlant;
import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.core.source.ScalarTarget;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/** Verifies that FtcActuators carries mapped configuration into observable Plant behavior. */
public final class FtcMappedPlantConfigurationIntegrationTest {

    private static final double EPSILON = 1.0e-12;

    @Test
    public void nonUnitVelocityMappingConvertsDeviceCommandAndMeasurement() {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        MotorProbe motor = hardwareMap.addMotor("flywheel");
        motor.velocityTicksPerSec = 50.0;

        Plant plant = FtcActuators.plant(hardwareMap)
                .motor("flywheel", Direction.FORWARD)
                .velocity()
                .deviceManaged()
                .bounded(0.0, 100.0)
                .scaleToNative(4.0)
                .velocityTolerance(0.0)
                .targetFromNewCommand(12.5)
                .build();

        plant.update(new ManualLoopClock().clock());

        assertEquals(12.5, plant.getMeasurement(), EPSILON);
        assertEquals(50.0, motor.commandedVelocityTicksPerSec, EPSILON);
        assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, motor.mode);
        assertTrue(plant.atTarget());
    }

    @Test
    public void staticPositionReferencePreservesNonUnitMapping() {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        MotorProbe motor = hardwareMap.addMotor("arm");
        motor.positionTicks = 112;

        PositionPlant plant = FtcActuators.plant(hardwareMap)
                .motor("arm", Direction.FORWARD)
                .position()
                .deviceManaged()
                .nonPeriodic()
                .bounded(-10.0, 10.0)
                .scaleToNative(4.0)
                .plantPositionMapsToNative(2.0, 100.0)
                .positionTolerance(0.0)
                .targetFromNewCommand(5.0)
                .build();

        plant.update(new ManualLoopClock().clock());

        assertTrue(plant.isReferenced());
        assertEquals("referenced", plant.referenceStatus());
        assertEquals(5.0, plant.getMeasurement(), EPSILON);
        assertEquals(112, motor.targetPositionTicks);
        assertTrue(plant.atTarget());
    }

    @Test
    public void assumeCurrentReferenceUsesMappedNativeSample() {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        MotorProbe motor = hardwareMap.addMotor("wrist");
        motor.positionTicks = 200;

        PositionPlant plant = FtcActuators.plant(hardwareMap)
                .motor("wrist", Direction.FORWARD)
                .position()
                .deviceManaged()
                .nonPeriodic()
                .unbounded()
                .scaleToNative(5.0)
                .assumeCurrentPositionIs(7.0)
                .positionTolerance(0.0)
                .targetFromNewCommand(8.0)
                .build();

        plant.update(new ManualLoopClock().clock());

        assertTrue(plant.isReferenced());
        assertEquals(7.0, plant.getMeasurement(), EPSILON);
        assertEquals(205, motor.targetPositionTicks);
    }

    @Test
    public void needsReferenceBlocksMappedOutputUntilReferenceIsEstablished() {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        MotorProbe motor = hardwareMap.addMotor("lift");
        motor.positionTicks = 300;
        ManualLoopClock clock = new ManualLoopClock();

        PositionPlant plant = FtcActuators.plant(hardwareMap)
                .motor("lift", Direction.FORWARD)
                .position()
                .deviceManaged()
                .nonPeriodic()
                .unbounded()
                .scaleToNative(10.0)
                .needsReference("lift not homed")
                .positionTolerance(0.0)
                .targetFromNewCommand(4.0)
                .build();

        assertFalse(plant.isReferenced());
        assertFalse(plant.targetRange().valid);
        assertEquals("lift not homed", plant.referenceStatus());

        plant.update(clock.clock());

        assertEquals(0, motor.targetPositionWriteCount);
        assertFalse(plant.isReferenced());

        plant.establishReferenceAt(3.0);
        plant.update(clock.nextCycle(0.02));

        assertTrue(plant.isReferenced());
        assertEquals("referenced", plant.referenceStatus());
        assertEquals(3.0, plant.getMeasurement(), EPSILON);
        assertEquals(310, motor.targetPositionTicks);
        assertEquals(1, motor.targetPositionWriteCount);
    }

    @Test
    public void standardServoEndpointMappingUsesDeclaredPlantRange() {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        ServoProbe servo = hardwareMap.addServo("claw");
        ScalarTarget target = ScalarTarget.create(-1.0);
        ManualLoopClock clock = new ManualLoopClock();

        PositionPlant plant = FtcActuators.plant(hardwareMap)
                .servo("claw", Direction.FORWARD)
                .position()
                .nonPeriodic()
                .bounded(-1.0, 1.0)
                .rangeMapsToNative(0.2, 0.8)
                .targetFromResolver(PlantTargets.exact(target))
                .build();

        plant.update(clock.clock());
        assertEquals(PositionPlant.Periodicity.NON_PERIODIC, plant.periodicity());
        assertEquals(-1.0, plant.getAppliedTarget(), EPSILON);
        assertEquals(0.2, servo.commandedPosition, EPSILON);

        target.set(1.0);
        plant.update(clock.nextCycle(0.02));
        assertEquals(1.0, plant.getAppliedTarget(), EPSILON);
        assertEquals(0.8, servo.commandedPosition, EPSILON);
        assertFalse(plant.hasFeedback());
    }

    @Test
    public void standardServoPeriodicCoordinateUsesTheSameBoundedMappingGrammar() {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        ServoProbe servo = hardwareMap.addServo("plate");

        PositionPlant plant = FtcActuators.plant(hardwareMap)
                .servo("plate", Direction.FORWARD)
                .position()
                .periodic(360.0)
                .bounded(0.0, 360.0)
                .rangeMapsToNative(0.0, 1.0)
                .targetFromNewCommand(180.0)
                .build();

        plant.update(new ManualLoopClock().clock());

        assertEquals(PositionPlant.Periodicity.PERIODIC, plant.periodicity());
        assertEquals(360.0, plant.period(), EPSILON);
        assertEquals(180.0, plant.getAppliedTarget(), EPSILON);
        assertEquals(0.5, servo.commandedPosition, EPSILON);
        assertFalse(plant.hasFeedback());
    }

    @Test
    public void servoPlantsStoppedBeforeFirstUpdateDoNotResolveOrWrite() {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        ServoProbe singleServo = hardwareMap.addServo("single");
        ServoProbe leftServo = hardwareMap.addServo("left");
        ServoProbe rightServo = hardwareMap.addServo("right");
        CountingResolver singleResolver = new CountingResolver();
        CountingResolver groupResolver = new CountingResolver();

        PositionPlant singlePlant = FtcActuators.plant(hardwareMap)
                .servo("single", Direction.FORWARD)
                .position()
                .nonPeriodic()
                .bounded(0.0, 1.0)
                .nativeUnits()
                .targetFromResolver(singleResolver)
                .build();
        PositionPlant groupPlant = FtcActuators.plant(hardwareMap)
                .servo("left", Direction.FORWARD)
                .andServo("right", Direction.REVERSE)
                .position()
                .nonPeriodic()
                .bounded(0.0, 1.0)
                .nativeUnits()
                .targetFromResolver(groupResolver)
                .build();

        singlePlant.stop();
        groupPlant.stop();

        assertEquals(0, singleServo.positionWriteCount);
        assertEquals(0, leftServo.positionWriteCount);
        assertEquals(0, rightServo.positionWriteCount);

        LoopClock clock = new ManualLoopClock().clock();
        singlePlant.update(clock);
        groupPlant.update(clock);

        assertEquals(0, singleResolver.resolutionCount);
        assertEquals(0, groupResolver.resolutionCount);
        assertEquals(0, singleServo.positionWriteCount);
        assertEquals(0, leftServo.positionWriteCount);
        assertEquals(0, rightServo.positionWriteCount);
        assertTrue(Double.isNaN(singleServo.commandedPosition));
        assertTrue(Double.isNaN(leftServo.commandedPosition));
        assertTrue(Double.isNaN(rightServo.commandedPosition));
    }

    /** In-memory HardwareMap that avoids Android-only SDK discovery in local JVM tests. */
    private static final class TestHardwareMap extends HardwareMap {
        private final Map<String, HardwareDevice> devices = new HashMap<>();

        private TestHardwareMap() {
            super(null, null);
        }

        private MotorProbe addMotor(String name) {
            MotorProbe probe = new MotorProbe();
            devices.put(name, probe.motor);
            return probe;
        }

        private ServoProbe addServo(String name) {
            ServoProbe probe = new ServoProbe();
            devices.put(name, probe.servo);
            return probe;
        }

        @Override
        public <T> T get(Class<? extends T> type, String name) {
            HardwareDevice device = devices.get(name);
            if (device == null) {
                throw new IllegalArgumentException("No test device named " + name);
            }
            if (!type.isInstance(device)) {
                throw new IllegalArgumentException(name + " is not a " + type.getSimpleName());
            }
            return type.cast(device);
        }
    }

    /** Dynamic SDK motor with independently observable position and velocity state. */
    private static final class MotorProbe {
        private int positionTicks;
        private int targetPositionTicks;
        private int targetPositionWriteCount;
        private double velocityTicksPerSec;
        private double commandedVelocityTicksPerSec;
        private double power;
        private DcMotor.RunMode mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        private DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
        private final DcMotorEx motor;

        private MotorProbe() {
            motor = (DcMotorEx) Proxy.newProxyInstance(
                    DcMotorEx.class.getClassLoader(),
                    new Class<?>[]{DcMotorEx.class},
                    this::invoke);
        }

        private Object invoke(Object proxy, Method method, Object[] args) {
            String name = method.getName();
            if (method.getDeclaringClass() == Object.class) {
                return objectMethod(proxy, method, args, "MotorProbe");
            }
            if ("getCurrentPosition".equals(name)) return positionTicks;
            if ("getVelocity".equals(name)) return velocityTicksPerSec;
            if ("setVelocity".equals(name)) {
                commandedVelocityTicksPerSec = (double) args[0];
                return null;
            }
            if ("setTargetPosition".equals(name)) {
                targetPositionTicks = (int) args[0];
                targetPositionWriteCount++;
                return null;
            }
            if ("getTargetPosition".equals(name)) return targetPositionTicks;
            if ("setPower".equals(name)) {
                power = (double) args[0];
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
            if ("getManufacturer".equals(name)) return HardwareDevice.Manufacturer.Other;
            if ("getDeviceName".equals(name)) return "MotorProbe";
            if ("getConnectionInfo".equals(name)) return "test";
            if ("getVersion".equals(name)) return 1;
            return defaultValue(method.getReturnType());
        }
    }

    /** Dynamic standard servo with an observable raw position command. */
    private static final class ServoProbe {
        private double commandedPosition = Double.NaN;
        private int positionWriteCount;
        private Servo.Direction direction = Servo.Direction.FORWARD;
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
                return objectMethod(proxy, method, args, "ServoProbe");
            }
            if ("setPosition".equals(name)) {
                commandedPosition = (double) args[0];
                positionWriteCount++;
                return null;
            }
            if ("getPosition".equals(name)) return commandedPosition;
            if ("setDirection".equals(name)) {
                direction = (Servo.Direction) args[0];
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

    private static final class CountingResolver implements PlantTargetResolver {
        private int resolutionCount;

        @Override
        public PlantTargetResolution resolve(PlantTargetContext context, LoopClock clock) {
            resolutionCount++;
            return PlantTargetResolution.exact(0.5, "counted test target");
        }
    }

    private static Object objectMethod(Object proxy,
                                       Method method,
                                       Object[] args,
                                       String label) {
        if ("toString".equals(method.getName())) return label;
        if ("hashCode".equals(method.getName())) return System.identityHashCode(proxy);
        if ("equals".equals(method.getName())) return proxy == args[0];
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
