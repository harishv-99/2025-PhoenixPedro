package edu.ftcphoenix.fw.ftc;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.Plants;
import edu.ftcphoenix.fw.actuation.PositionPlant;
import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.hal.PositionOutput;
import edu.ftcphoenix.fw.core.hal.VelocityOutput;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Focused MAP-01 coverage for FTC child mappings and final native command domains. */
public final class FtcActuatorMappingDomainValidationTest {

    private static final double EPSILON = 1.0e-9;

    @Test
    public void crServoGroupSurfaceHasScaleButNoBias() {
        boolean hasScale = false;
        for (Method method : FtcActuators.CrServoGroupAddedStep.class.getDeclaredMethods()) {
            if ("scale".equals(method.getName())) hasScale = true;
            assertFalse("CR-servo child bias must not remain public",
                    "bias".equals(method.getName()));
        }
        assertTrue(hasScale);
    }

    @Test
    public void retainedChildSettersRejectNonFiniteWithoutReplacingAcceptedValues() {
        TestHardwareMap motorMap = new TestHardwareMap();
        MotorProbe motorA = motorMap.addMotor("motorA");
        MotorProbe motorB = motorMap.addMotor("motorB");
        FtcActuators.MotorGroupAddedStep motors = FtcActuators.plant(motorMap)
                .motor("motorA", Direction.FORWARD)
                .andMotor("motorB", Direction.FORWARD)
                .scale(0.5)
                .bias(0.0);
        for (double invalid : nonFiniteValues()) {
            expect(IllegalArgumentException.class, () -> motors.scale(invalid));
            expect(IllegalArgumentException.class, () -> motors.bias(invalid));
        }
        motorMap.assertNoLookup();
        Plant motorPlant = motors.power().targetFromNewCommand(1.0).build();
        motorPlant.update(new ManualLoopClock().clock());
        assertEquals(1.0, motorA.lastPower, EPSILON);
        assertEquals(0.5, motorB.lastPower, EPSILON);

        TestHardwareMap servoMap = new TestHardwareMap();
        ServoProbe servoA = servoMap.addServo("servoA");
        ServoProbe servoB = servoMap.addServo("servoB");
        FtcActuators.ServoGroupAddedStep servos = FtcActuators.plant(servoMap)
                .servo("servoA", Direction.FORWARD)
                .andServo("servoB", Direction.FORWARD)
                .scale(0.5)
                .bias(0.25);
        for (double invalid : nonFiniteValues()) {
            expect(IllegalArgumentException.class, () -> servos.scale(invalid));
            expect(IllegalArgumentException.class, () -> servos.bias(invalid));
        }
        servoMap.assertNoLookup();
        PositionPlant servoPlant = servos.position()
                .nonPeriodic()
                .bounded(0.0, 1.0)
                .nativeUnits()
                .targetFromNewCommand(1.0)
                .build();
        servoPlant.update(new ManualLoopClock().clock());
        assertEquals(1.0, servoA.lastPosition, EPSILON);
        assertEquals(0.75, servoB.lastPosition, EPSILON);

        TestHardwareMap crMap = new TestHardwareMap();
        CrServoProbe crA = crMap.addCrServo("crA");
        CrServoProbe crB = crMap.addCrServo("crB");
        FtcActuators.CrServoGroupAddedStep crServos = FtcActuators.plant(crMap)
                .crServo("crA", Direction.FORWARD)
                .andCrServo("crB", Direction.FORWARD)
                .scale(0.4);
        for (double invalid : nonFiniteValues()) {
            expect(IllegalArgumentException.class, () -> crServos.scale(invalid));
        }
        crMap.assertNoLookup();
        Plant crPlant = crServos.power().targetFromNewCommand(-1.0).build();
        crPlant.update(new ManualLoopClock().clock());
        assertEquals(-1.0, crA.lastPower, EPSILON);
        assertEquals(-0.4, crB.lastPower, EPSILON);
    }

    @Test
    public void directPowerPreflightsNeutralAndFullRangeAtPowerAnswerAndAllowsRetry() {
        TestHardwareMap motorMap = new TestHardwareMap();
        motorMap.addMotor("left");
        motorMap.addMotor("right");
        FtcActuators.MotorGroupAddedStep motors = FtcActuators.plant(motorMap)
                .motor("left", Direction.FORWARD)
                .andMotor("right", Direction.FORWARD)
                .bias(0.01);

        expect(IllegalStateException.class, motors::power);
        motorMap.assertNoLookup();
        motors.bias(-0.0).scale(Math.nextUp(1.0));
        expect(IllegalStateException.class, motors::power);
        motorMap.assertNoLookup();

        Plant motorPlant = motors.scale(1.0)
                .power()
                .targetFromNewCommand(0.0)
                .build();
        motorPlant.update(new ManualLoopClock().clock());
        assertEquals(0.0, motorMap.motor("left").lastPower, EPSILON);
        assertEquals(0.0, motorMap.motor("right").lastPower, EPSILON);

        TestHardwareMap crMap = new TestHardwareMap();
        crMap.addCrServo("left");
        crMap.addCrServo("right");
        FtcActuators.CrServoGroupAddedStep crServos = FtcActuators.plant(crMap)
                .crServo("left", Direction.FORWARD)
                .andCrServo("right", Direction.FORWARD)
                .scale(Math.nextUp(1.0));
        expect(IllegalStateException.class, crServos::power);
        crMap.assertNoLookup();
        crServos.scale(-1.0).power().targetFromNewCommand(1.0).build();
        assertEquals(2, crMap.lookupCount);
    }

    @Test
    public void servoMappingPreflightsComposedEndpointsBeforeLookupAndSupportsRetryAndMirrors() {
        TestHardwareMap singleMap = new TestHardwareMap();
        ServoProbe single = singleMap.addServo("servo");
        FtcActuators.ServoBoundedPositionMappingStep singleMapping =
                FtcActuators.plant(singleMap)
                        .servo("servo", Direction.FORWARD)
                        .position()
                        .nonPeriodic()
                        .bounded(-1.0, 1.0);

        expect(IllegalStateException.class, singleMapping::nativeUnits);
        singleMap.assertNoLookup();
        expect(IllegalStateException.class,
                () -> singleMapping.rangeMapsToNative(0.0, Math.nextUp(1.0)));
        singleMap.assertNoLookup();
        PositionPlant reversed = singleMapping.rangeMapsToNative(1.0, 0.0)
                .targetFromNewCommand(-1.0)
                .build();
        ManualLoopClock reversedClock = new ManualLoopClock();
        reversed.update(reversedClock.clock());
        assertEquals(1.0, single.lastPosition, EPSILON);
        reversed.commandTarget().set(1.0);
        reversed.update(reversedClock.nextCycle(0.02));
        assertEquals(0.0, single.lastPosition, EPSILON);

        TestHardwareMap groupMap = new TestHardwareMap();
        ServoProbe groupFirst = groupMap.addServo("first");
        ServoProbe groupMirror = groupMap.addServo("mirror");
        FtcActuators.ServoGroupAddedStep group = FtcActuators.plant(groupMap)
                .servo("first", Direction.FORWARD)
                .andServo("mirror", Direction.FORWARD)
                .scale(-1.0)
                .bias(Math.nextUp(1.0));
        FtcActuators.ServoBoundedPositionMappingStep groupMapping = group.position()
                .nonPeriodic()
                .bounded(0.0, 1.0);
        expect(IllegalStateException.class, groupMapping::nativeUnits);
        groupMap.assertNoLookup();
        group.bias(1.0);
        PositionPlant mirrored = groupMapping.nativeUnits()
                .targetFromNewCommand(0.25)
                .build();
        mirrored.update(new ManualLoopClock().clock());
        assertEquals(0.25, groupFirst.lastPosition, EPSILON);
        assertEquals(0.75, groupMirror.lastPosition, EPSILON);
    }

    @Test
    public void deviceVelocityRejectsInvalidChildSemanticsBeforeConsumingBranchOrMap() {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        hardwareMap.addMotor("left");
        hardwareMap.addMotor("right");
        FtcActuators.MotorGroupAddedStep group = FtcActuators.plant(hardwareMap)
                .motor("left", Direction.FORWARD)
                .andMotor("right", Direction.REVERSE)
                .scale(0.0);
        FtcActuators.MotorVelocityControlStep velocity = group.velocity();

        expect(IllegalStateException.class, velocity::deviceManagedWithDefaults);
        hardwareMap.assertNoLookup();
        group.scale(1.0).bias(1.0);
        expect(IllegalStateException.class, velocity::deviceManagedWithDefaults);
        hardwareMap.assertNoLookup();

        group.bias(-0.0);
        Plants.VelocityMappingStep mapping = velocity.deviceManagedWithDefaults()
                .bounded(Double.MAX_VALUE / 2.0, Double.MAX_VALUE);
        expect(IllegalArgumentException.class, () -> mapping.scaleToNative(2.0));
        hardwareMap.assertNoLookup();
        Plant plant = mapping.scaleToNative(0.5)
                .velocityTolerance(0.0)
                .targetFromNewCommand(Double.MAX_VALUE / 2.0)
                .build();
        // Two feedback sources plus two command outputs resolve at build.
        assertEquals(4, hardwareMap.lookupCount);
        assertTrue(plant.hasFeedback());
    }

    @Test
    public void regulatedVelocityPreflightsBoundedSharedMapBeforeLookupAndAllowsRetry() {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        hardwareMap.addMotor("flywheel");
        Plants.VelocityMappingStep mapping = FtcActuators.plant(hardwareMap)
                .motor("flywheel", Direction.FORWARD)
                .velocity()
                .regulated()
                .nativeFeedback(clock -> 0.0)
                .regulator((setpoint, measurement, clock) -> 0.0)
                .bounded(1.0, 2.0);

        expect(IllegalArgumentException.class,
                () -> mapping.scaleToNative(Double.MAX_VALUE));
        hardwareMap.assertNoLookup();

        Plant recovered = mapping.scaleToNative(1.0)
                .velocityTolerance(0.0)
                .targetFromNewCommand(1.0)
                .build();
        assertEquals(1, hardwareMap.lookupCount);
        recovered.update(new ManualLoopClock().clock());
        assertEquals(0.0, hardwareMap.motor("flywheel").lastPower, 0.0);
    }

    @Test
    public void finiteSubEpsilonSharedScalesRemainValidWhenEndpointImagesAreFinite() {
        TestHardwareMap velocityMap = new TestHardwareMap();
        MotorProbe motor = velocityMap.addMotor("motor");
        Plant velocity = FtcActuators.plant(velocityMap)
                .motor("motor", Direction.FORWARD)
                .velocity()
                .deviceManagedWithDefaults()
                .bounded(0.0, 1.0)
                .scaleToNative(Double.MIN_VALUE)
                .velocityTolerance(0.0)
                .targetFromNewCommand(1.0)
                .build();
        velocity.update(new ManualLoopClock().clock());
        assertEquals(Double.MIN_VALUE, motor.lastVelocity, 0.0);

        TestHardwareMap servoMap = new TestHardwareMap();
        ServoProbe servo = servoMap.addServo("servo");
        PositionPlant position = FtcActuators.plant(servoMap)
                .servo("servo", Direction.FORWARD)
                .position()
                .nonPeriodic()
                .bounded(0.0, 1.0)
                .rangeMapsToNative(0.0, Double.MIN_VALUE)
                .targetFromNewCommand(1.0)
                .build();
        position.update(new ManualLoopClock().clock());
        assertEquals(Double.MIN_VALUE, servo.lastPosition, 0.0);
    }

    @Test
    public void opposedFlywheelScalePreservesZeroReverseAndInverseGroupCoordinate() {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        MotorProbe left = hardwareMap.addMotor("left");
        MotorProbe right = hardwareMap.addMotor("right");
        left.velocityMeasurement = 2500.0;
        right.velocityMeasurement = 2400.0;
        Plant flywheels = FtcActuators.plant(hardwareMap)
                .motor("left", Direction.FORWARD)
                .andMotor("right", Direction.REVERSE)
                .scale(0.96)
                .velocity()
                .deviceManagedWithDefaults()
                .bounded(-2600.0, 2600.0)
                .nativeUnits()
                .velocityTolerance(0.0)
                .targetFromNewCommand(2500.0)
                .build();
        ManualLoopClock clock = new ManualLoopClock();

        flywheels.update(clock.clock());
        assertEquals(2500.0, left.lastVelocity, EPSILON);
        assertEquals(2400.0, right.lastVelocity, EPSILON);
        assertEquals(2500.0, flywheels.getMeasurement(), EPSILON);
        assertTrue(flywheels.atTarget());

        left.velocityMeasurement = 0.0;
        right.velocityMeasurement = 0.0;
        flywheels.commandTarget().set(0.0);
        flywheels.update(clock.nextCycle(0.02));
        assertEquals(0.0, left.lastVelocity, EPSILON);
        assertEquals(0.0, right.lastVelocity, EPSILON);
        assertEquals(0.0, flywheels.getMeasurement(), EPSILON);

        left.velocityMeasurement = -2500.0;
        right.velocityMeasurement = -2400.0;
        flywheels.commandTarget().set(-2500.0);
        flywheels.update(clock.nextCycle(0.02));
        assertEquals(-2500.0, left.lastVelocity, EPSILON);
        assertEquals(-2400.0, right.lastVelocity, EPSILON);
        assertEquals(-2500.0, flywheels.getMeasurement(), EPSILON);
    }

    @Test
    public void regulatedGroupsRequireExactIdentityAtTheBranchAnswerAndAllowRetry() {
        TestHardwareMap motorMap = new TestHardwareMap();
        motorMap.addMotor("left");
        motorMap.addMotor("right");
        FtcActuators.MotorGroupAddedStep motors = FtcActuators.plant(motorMap)
                .motor("left", Direction.FORWARD)
                .andMotor("right", Direction.FORWARD)
                .scale(Math.nextUp(1.0));
        FtcActuators.MotorVelocityControlStep velocity = motors.velocity();
        expect(IllegalStateException.class, velocity::regulated);
        motorMap.assertNoLookup();
        motors.scale(1.0);
        assertTrue(velocity.regulated() != null);
        motorMap.assertNoLookup();

        TestHardwareMap crMap = new TestHardwareMap();
        crMap.addCrServo("left");
        crMap.addCrServo("right");
        FtcActuators.CrServoGroupAddedStep crServos = FtcActuators.plant(crMap)
                .crServo("left", Direction.FORWARD)
                .andCrServo("right", Direction.FORWARD)
                .scale(Math.nextDown(1.0));
        FtcActuators.CrServoPositionControlStep position = crServos.position();
        expect(IllegalStateException.class, position::regulated);
        crMap.assertNoLookup();
        crServos.scale(1.0);
        assertTrue(position.regulated() != null);
        crMap.assertNoLookup();
    }

    @Test
    public void unboundedVelocityPrecomputesEveryChildBeforeAnyFanoutAndPreservesCaches()
            throws Exception {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        MotorProbe first = hardwareMap.addMotor("first");
        MotorProbe second = hardwareMap.addMotor("second");
        Plant plant = FtcActuators.plant(hardwareMap)
                .motor("first", Direction.FORWARD)
                .andMotor("second", Direction.FORWARD)
                .scale(2.0)
                .velocity()
                .deviceManagedWithDefaults()
                .unbounded()
                .nativeUnits()
                .velocityTolerance(0.0)
                .targetFromNewCommand(1.0)
                .build();
        ManualLoopClock time = new ManualLoopClock();
        plant.update(time.clock());
        VelocityOutput groupOutput = privateField(
                plant, "velocityOut", VelocityOutput.class);
        assertEquals(1.0, groupOutput.getCommandedVelocity(), 0.0);
        assertEquals(1.0, first.lastVelocity, 0.0);
        assertEquals(2.0, second.lastVelocity, 0.0);
        int firstVelocityWrites = first.velocityWrites.size();
        int secondVelocityWrites = second.velocityWrites.size();
        int firstModeWrites = first.modeWrites;
        int secondModeWrites = second.modeWrites;

        plant.commandTarget().set(Double.MAX_VALUE);
        expect(IllegalStateException.class,
                () -> plant.update(time.nextCycle(0.02)));
        assertEquals(1.0, groupOutput.getCommandedVelocity(), 0.0);
        assertEquals(firstVelocityWrites, first.velocityWrites.size());
        assertEquals(secondVelocityWrites, second.velocityWrites.size());
        assertEquals(firstModeWrites, first.modeWrites);
        assertEquals(secondModeWrites, second.modeWrites);
        assertEquals(1.0, first.lastVelocity, 0.0);
        assertEquals(2.0, second.lastVelocity, 0.0);
    }

    @Test
    public void inverseMappedMeasurementsFailClosedAndFiniteLargeAverageDoesNotOverflow() {
        TestHardwareMap largeMap = new TestHardwareMap();
        MotorProbe largeA = largeMap.addMotor("a");
        MotorProbe largeB = largeMap.addMotor("b");
        largeA.velocityMeasurement = Double.MAX_VALUE;
        largeB.velocityMeasurement = Double.MAX_VALUE;
        Plant largePlant = velocityGroup(largeMap, "a", "b", 1.0);
        largePlant.update(new ManualLoopClock().clock());
        assertEquals(Double.MAX_VALUE, largePlant.getMeasurement(), 0.0);

        TestHardwareMap invalidMap = new TestHardwareMap();
        MotorProbe invalidA = invalidMap.addMotor("a");
        MotorProbe invalidB = invalidMap.addMotor("b");
        invalidA.velocityMeasurement = 0.0;
        invalidB.velocityMeasurement = Double.NaN;
        Plant invalidPlant = velocityGroup(invalidMap, "a", "b", 1.0);
        invalidPlant.update(new ManualLoopClock().clock());
        assertTrue(Double.isNaN(invalidPlant.getMeasurement()));
        assertFalse(invalidPlant.atTarget());

        TestHardwareMap inverseOverflowMap = new TestHardwareMap();
        MotorProbe inverseA = inverseOverflowMap.addMotor("a");
        MotorProbe inverseB = inverseOverflowMap.addMotor("b");
        inverseA.velocityMeasurement = 0.0;
        inverseB.velocityMeasurement = Double.MAX_VALUE;
        Plant inverseOverflow = velocityGroup(
                inverseOverflowMap, "a", "b", Double.MIN_VALUE);
        inverseOverflow.update(new ManualLoopClock().clock());
        assertTrue(Double.isNaN(inverseOverflow.getMeasurement()));
        assertFalse(inverseOverflow.atTarget());
    }

    @Test
    public void devicePositionStaticPreflightIsRetryableAndRuntimeChecksAllChildrenBeforeWrite()
            throws Exception {
        TestHardwareMap boundedMap = new TestHardwareMap();
        boundedMap.addMotor("left");
        boundedMap.addMotor("right");
        FtcActuators.MotorGroupAddedStep boundedGroup = FtcActuators.plant(boundedMap)
                .motor("left", Direction.FORWARD)
                .andMotor("right", Direction.FORWARD)
                .bias((double) Integer.MAX_VALUE + 1.0);
        Plants.PositionReferenceStep reference = boundedGroup.position()
                .deviceManagedWithDefaults()
                .nonPeriodic()
                .bounded(0.0, 1.0)
                .nativeUnits();

        expect(IllegalStateException.class, reference::alreadyReferenced);
        boundedMap.assertNoLookup();
        boundedGroup.bias(0.0);
        PositionPlant recovered = reference.alreadyReferenced()
                .positionTolerance(0.0)
                .targetFromNewCommand(1.0)
                .build();
        recovered.update(new ManualLoopClock().clock());
        assertEquals(1, boundedMap.motor("left").lastTargetPosition);
        assertEquals(1, boundedMap.motor("right").lastTargetPosition);

        TestHardwareMap runtimeMap = new TestHardwareMap();
        MotorProbe runtimeFirst = runtimeMap.addMotor("first");
        MotorProbe runtimeSecond = runtimeMap.addMotor("second");
        PositionPlant runtimePlant = FtcActuators.plant(runtimeMap)
                .motor("first", Direction.FORWARD)
                .andMotor("second", Direction.FORWARD)
                .scale(2.0)
                .position()
                .deviceManagedWithDefaults()
                .nonPeriodic()
                .unbounded()
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(0.0)
                .targetFromNewCommand(1.0)
                .build();
        ManualLoopClock runtimeClock = new ManualLoopClock();
        runtimePlant.update(runtimeClock.clock());
        PositionOutput groupOutput = privateField(
                runtimePlant, "positionOut", PositionOutput.class);
        assertEquals(1.0, groupOutput.getCommandedPosition(), 0.0);
        assertEquals(1, runtimeFirst.lastTargetPosition);
        assertEquals(2, runtimeSecond.lastTargetPosition);
        int firstTargetWrites = runtimeFirst.targetPositionWrites;
        int secondTargetWrites = runtimeSecond.targetPositionWrites;
        int firstModeWrites = runtimeFirst.modeWrites;
        int secondModeWrites = runtimeSecond.modeWrites;
        int firstPowerWrites = runtimeFirst.powerWrites;
        int secondPowerWrites = runtimeSecond.powerWrites;

        runtimePlant.commandTarget().set(Integer.MAX_VALUE);
        expect(IllegalStateException.class,
                () -> runtimePlant.update(runtimeClock.nextCycle(0.02)));
        assertEquals(1.0, groupOutput.getCommandedPosition(), 0.0);
        assertEquals(firstTargetWrites, runtimeFirst.targetPositionWrites);
        assertEquals(secondTargetWrites, runtimeSecond.targetPositionWrites);
        assertEquals(firstModeWrites, runtimeFirst.modeWrites);
        assertEquals(secondModeWrites, runtimeSecond.modeWrites);
        assertEquals(firstPowerWrites, runtimeFirst.powerWrites);
        assertEquals(secondPowerWrites, runtimeSecond.powerWrites);
        assertEquals(1, runtimeFirst.lastTargetPosition);
        assertEquals(2, runtimeSecond.lastTargetPosition);
    }

    private static Plant velocityGroup(TestHardwareMap hardwareMap,
                                       String first,
                                       String second,
                                       double secondScale) {
        return FtcActuators.plant(hardwareMap)
                .motor(first, Direction.FORWARD)
                .andMotor(second, Direction.FORWARD)
                .scale(secondScale)
                .velocity()
                .deviceManagedWithDefaults()
                .bounded(0.0, 0.0)
                .nativeUnits()
                .velocityTolerance(0.0)
                .targetFromNewCommand(0.0)
                .build();
    }

    private static double[] nonFiniteValues() {
        return new double[]{Double.NaN, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY};
    }

    private static <T> T privateField(Object owner, String fieldName, Class<T> fieldType)
            throws Exception {
        Class<?> type = owner.getClass();
        while (type != null) {
            try {
                Field field = type.getDeclaredField(fieldName);
                field.setAccessible(true);
                return fieldType.cast(field.get(owner));
            } catch (NoSuchFieldException ignored) {
                type = type.getSuperclass();
            }
        }
        fail("No field named " + fieldName + " on " + owner.getClass());
        return null;
    }

    private static void expect(Class<? extends RuntimeException> expected, Runnable action) {
        try {
            action.run();
        } catch (RuntimeException failure) {
            assertTrue("Expected " + expected.getSimpleName() + ", got " + failure,
                    expected.isInstance(failure));
            return;
        }
        fail("Expected " + expected.getSimpleName());
    }

    private static final class TestHardwareMap extends HardwareMap {
        private final Map<String, HardwareDevice> devices = new HashMap<>();
        private final Map<String, MotorProbe> motors = new HashMap<>();
        private int lookupCount;

        private TestHardwareMap() {
            super(null, null);
        }

        private MotorProbe addMotor(String name) {
            MotorProbe probe = new MotorProbe();
            devices.put(name, probe.motor);
            motors.put(name, probe);
            return probe;
        }

        private ServoProbe addServo(String name) {
            ServoProbe probe = new ServoProbe();
            devices.put(name, probe.servo);
            return probe;
        }

        private CrServoProbe addCrServo(String name) {
            CrServoProbe probe = new CrServoProbe();
            devices.put(name, probe.servo);
            return probe;
        }

        private MotorProbe motor(String name) {
            return motors.get(name);
        }

        private void assertNoLookup() {
            assertEquals(0, lookupCount);
        }

        @Override
        public <T> T get(Class<? extends T> type, String name) {
            lookupCount++;
            HardwareDevice device = devices.get(name);
            if (device == null || !type.isInstance(device)) {
                throw new IllegalArgumentException("No test " + type.getSimpleName() + " named " + name);
            }
            return type.cast(device);
        }
    }

    private static final class MotorProbe {
        private final List<Double> velocityWrites = new ArrayList<>();
        private final DcMotorEx motor;
        private double lastPower;
        private double lastVelocity;
        private double velocityMeasurement;
        private int currentPosition;
        private int lastTargetPosition;
        private int targetPositionWrites;
        private int modeWrites;
        private int powerWrites;
        private DcMotor.RunMode mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        private DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;

        private MotorProbe() {
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
            if ("setDirection".equals(name)) {
                direction = (DcMotorSimple.Direction) args[0];
                return null;
            }
            if ("getDirection".equals(name)) return direction;
            if ("setPower".equals(name)) {
                lastPower = (double) args[0];
                powerWrites++;
                return null;
            }
            if ("getPower".equals(name)) return lastPower;
            if ("setVelocity".equals(name)) {
                lastVelocity = (double) args[0];
                velocityWrites.add(lastVelocity);
                return null;
            }
            if ("getVelocity".equals(name)) return velocityMeasurement;
            if ("setTargetPosition".equals(name)) {
                lastTargetPosition = (int) args[0];
                targetPositionWrites++;
                return null;
            }
            if ("getTargetPosition".equals(name)) return lastTargetPosition;
            if ("getCurrentPosition".equals(name)) return currentPosition;
            if ("setMode".equals(name)) {
                mode = (DcMotor.RunMode) args[0];
                modeWrites++;
                return null;
            }
            if ("getMode".equals(name)) return mode;
            if ("getZeroPowerBehavior".equals(name)) return DcMotor.ZeroPowerBehavior.FLOAT;
            if ("isBusy".equals(name)) return false;
            return hardwareDeviceOrDefault(method, "MotorProbe");
        }
    }

    private static final class ServoProbe {
        private final Servo servo;
        private double lastPosition;
        private Servo.Direction direction = Servo.Direction.FORWARD;

        private ServoProbe() {
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
                lastPosition = (double) args[0];
                return null;
            }
            if ("getPosition".equals(name)) return lastPosition;
            if ("setDirection".equals(name)) {
                direction = (Servo.Direction) args[0];
                return null;
            }
            if ("getDirection".equals(name)) return direction;
            return hardwareDeviceOrDefault(method, "ServoProbe");
        }
    }

    private static final class CrServoProbe {
        private final CRServo servo;
        private double lastPower;
        private DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;

        private CrServoProbe() {
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
                lastPower = (double) args[0];
                return null;
            }
            if ("getPower".equals(name)) return lastPower;
            if ("setDirection".equals(name)) {
                direction = (DcMotorSimple.Direction) args[0];
                return null;
            }
            if ("getDirection".equals(name)) return direction;
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
