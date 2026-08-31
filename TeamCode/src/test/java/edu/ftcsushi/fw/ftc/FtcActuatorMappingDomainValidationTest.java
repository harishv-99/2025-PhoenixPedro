package edu.ftcsushi.fw.ftc;

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

import edu.ftcsushi.fw.actuation.Plant;
import edu.ftcsushi.fw.actuation.PlantTargetStatus;
import edu.ftcsushi.fw.actuation.Plants;
import edu.ftcsushi.fw.actuation.PositionPlant;
import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.core.hal.PowerOutput;
import edu.ftcsushi.fw.core.hal.PositionOutput;
import edu.ftcsushi.fw.core.hal.VelocityOutput;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Focused coverage for FTC child mappings and final native command domains. */
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
    public void groupedStopAttemptsEveryChildAndPreservesFailureOrder() {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        MotorProbe first = hardwareMap.addMotor("first");
        MotorProbe second = hardwareMap.addMotor("second");
        Plant plant = FtcActuators.plant(hardwareMap)
                .motor("first", Direction.FORWARD)
                .andMotor("second", Direction.FORWARD)
                .power()
                .targetFromNewCommand(0.5)
                .build();
        plant.update(new ManualLoopClock().clock());

        RuntimeException firstFailure = new IllegalStateException("first stop failed");
        RuntimeException secondFailure = new IllegalStateException("second stop failed");
        first.stopPowerFailure = firstFailure;
        second.stopPowerFailure = secondFailure;
        int firstAttempts = first.powerAttempts;
        int secondAttempts = second.powerAttempts;

        RuntimeException thrown = null;
        try {
            plant.stop();
        } catch (RuntimeException failure) {
            thrown = failure;
        }

        assertTrue("The first child failure must remain primary", thrown == firstFailure);
        assertEquals(firstAttempts + 1, first.powerAttempts);
        assertEquals(secondAttempts + 1, second.powerAttempts);
        assertEquals(1, thrown.getSuppressed().length);
        assertTrue("The later child failure must be suppressed in order",
                thrown.getSuppressed()[0] == secondFailure);
        assertEquals("A failed physical stop must retain the last applied target fact",
                0.5, plant.getAppliedTarget(), 0.0);
        assertEquals(PlantTargetStatus.Kind.ACCEPTED, plant.getTargetStatus().kind());
        assertTrue(plant.getTargetResolution().hasTarget());

        plant.stop();
        assertEquals("A terminal Plant must not retry a failed physical stop",
                firstAttempts + 1, first.powerAttempts);
        assertEquals("A terminal Plant must not retry a failed physical stop",
                secondAttempts + 1, second.powerAttempts);
    }

    @Test
    public void groupedVelocityStopAttemptsEveryChildInOrderAndTerminalPlantDoesNotRetry() {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        MotorProbe first = hardwareMap.addMotor("first");
        MotorProbe second = hardwareMap.addMotor("second");
        Plant plant = FtcActuators.plant(hardwareMap)
                .motor("first", Direction.FORWARD)
                .andMotor("second", Direction.FORWARD)
                .velocity()
                .deviceManaged()
                .bounded(-10.0, 10.0)
                .nativeUnits()
                .velocityTolerance(0.0)
                .targetFromNewCommand(2.0)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        plant.update(clock.clock());

        RuntimeException firstFailure = new IllegalStateException("first velocity stop failed");
        RuntimeException secondFailure = new IllegalStateException("second velocity stop failed");
        first.stopPowerFailure = firstFailure;
        second.stopPowerFailure = secondFailure;
        int firstPowerAttempts = first.powerAttempts;
        int secondPowerAttempts = second.powerAttempts;
        int firstVelocityWrites = first.velocityWrites.size();
        int secondVelocityWrites = second.velocityWrites.size();

        RuntimeException thrown = null;
        try {
            plant.stop();
        } catch (RuntimeException failure) {
            thrown = failure;
        }

        assertTrue("The first velocity child failure must remain primary", thrown == firstFailure);
        assertEquals(firstPowerAttempts + 1, first.powerAttempts);
        assertEquals(secondPowerAttempts + 1, second.powerAttempts);
        assertEquals(firstVelocityWrites + 1, first.velocityWrites.size());
        assertEquals(secondVelocityWrites + 1, second.velocityWrites.size());
        assertEquals(1, thrown.getSuppressed().length);
        assertTrue("The later velocity child failure must be suppressed in order",
                thrown.getSuppressed()[0] == secondFailure);
        assertEquals(2.0, plant.getAppliedTarget(), 0.0);
        assertEquals(PlantTargetStatus.Kind.ACCEPTED, plant.getTargetStatus().kind());

        plant.stop();
        plant.commandTarget().set(4.0);
        plant.update(clock.nextCycle(0.02));
        assertEquals(firstPowerAttempts + 1, first.powerAttempts);
        assertEquals(secondPowerAttempts + 1, second.powerAttempts);
        assertEquals(firstVelocityWrites + 1, first.velocityWrites.size());
        assertEquals(secondVelocityWrites + 1, second.velocityWrites.size());
    }

    @Test
    public void groupedPositionStopAttemptsEveryChildInOrderAndTerminalPlantDoesNotRetry() {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        ServoProbe first = hardwareMap.addServo("first");
        ServoProbe second = hardwareMap.addServo("second");
        PositionPlant plant = FtcActuators.plant(hardwareMap)
                .servo("first", Direction.FORWARD)
                .andServo("second", Direction.FORWARD)
                .position()
                .nonPeriodic()
                .bounded(0.0, 1.0)
                .nativeUnits()
                .targetFromNewCommand(0.6)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        plant.update(clock.clock());

        RuntimeException firstFailure = new IllegalStateException("first position stop failed");
        RuntimeException secondFailure = new IllegalStateException("second position stop failed");
        first.stopPositionFailure = firstFailure;
        second.stopPositionFailure = secondFailure;
        int firstAttempts = first.positionAttempts;
        int secondAttempts = second.positionAttempts;

        RuntimeException thrown = null;
        try {
            plant.stop();
        } catch (RuntimeException failure) {
            thrown = failure;
        }

        assertTrue("The first position child failure must remain primary", thrown == firstFailure);
        assertEquals(firstAttempts + 1, first.positionAttempts);
        assertEquals(secondAttempts + 1, second.positionAttempts);
        assertEquals(1, thrown.getSuppressed().length);
        assertTrue("The later position child failure must be suppressed in order",
                thrown.getSuppressed()[0] == secondFailure);
        assertEquals(0.6, plant.getAppliedTarget(), 0.0);
        assertEquals(PlantTargetStatus.Kind.ACCEPTED, plant.getTargetStatus().kind());

        plant.stop();
        plant.commandTarget().set(0.2);
        plant.update(clock.nextCycle(0.02));
        assertEquals(firstAttempts + 1, first.positionAttempts);
        assertEquals(secondAttempts + 1, second.positionAttempts);
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

        expect(IllegalStateException.class, velocity::deviceManaged);
        hardwareMap.assertNoLookup();
        group.scale(1.0).bias(1.0);
        expect(IllegalStateException.class, velocity::deviceManaged);
        hardwareMap.assertNoLookup();

        group.bias(-0.0);
        Plants.VelocityMappingStep<Plants.TargetStep<Plant>> mapping = velocity.deviceManaged()
                .bounded(Double.MAX_VALUE / 2.0, Double.MAX_VALUE);
        expect(IllegalArgumentException.class, () -> mapping.scaleToNative(2.0));
        hardwareMap.assertNoLookup();
        Plant plant = mapping.scaleToNative(0.5)
                .velocityTolerance(0.0)
                .targetFromNewCommand(Double.MAX_VALUE / 2.0)
                .build();
        // Each child resolves once; the Plant and tuning evidence share those exact devices.
        assertEquals(2, hardwareMap.lookupCount);
        assertTrue(plant.hasFeedback());
    }

    @Test
    public void regulatedVelocityPreflightsBoundedSharedMapBeforeLookupAndAllowsRetry() {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        hardwareMap.addMotor("flywheel");
        Plants.VelocityMappingStep<Plants.VelocityControlStep> mapping =
                FtcActuators.plant(hardwareMap)
                .motor("flywheel", Direction.FORWARD)
                .velocity()
                .regulated()
                .nativeFeedback(clock -> 0.0)
                .bounded(1.0, 2.0);

        expect(IllegalArgumentException.class,
                () -> mapping.scaleToNative(Double.MAX_VALUE));
        hardwareMap.assertNoLookup();

        Plant recovered = mapping.scaleToNative(1.0)
                .velocityTolerance(0.0)
                .controlFromCustomRegulator((setpoint, measurement, clock) -> 0.0)
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
                .deviceManaged()
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
                .deviceManaged()
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
    public void unboundedVelocityPreflightIsAtomicThenPlantFailStopsAndCanRetry()
            throws Exception {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        MotorProbe first = hardwareMap.addMotor("first");
        MotorProbe second = hardwareMap.addMotor("second");
        Plant plant = FtcActuators.plant(hardwareMap)
                .motor("first", Direction.FORWARD)
                .andMotor("second", Direction.FORWARD)
                .scale(2.0)
                .velocity()
                .deviceManaged()
                .unbounded()
                .nativeUnits()
                .velocityTolerance(0.0)
                .targetFromNewCommand(1.0)
                .build();
        ManualLoopClock time = new ManualLoopClock();
        plant.update(time.clock());
        Object binding = privateField(plant, "binding", Object.class);
        VelocityOutput groupOutput = privateField(
                binding, "output", VelocityOutput.class);
        assertEquals(1.0, groupOutput.getCommandedVelocity(), 0.0);
        assertEquals(1.0, first.lastVelocity, 0.0);
        assertEquals(2.0, second.lastVelocity, 0.0);
        int firstVelocityWrites = first.velocityWrites.size();
        int secondVelocityWrites = second.velocityWrites.size();
        int firstModeWrites = first.modeWrites;
        int secondModeWrites = second.modeWrites;
        int firstPowerWrites = first.powerWrites;
        int secondPowerWrites = second.powerWrites;

        plant.commandTarget().set(Double.MAX_VALUE);
        expect(IllegalStateException.class,
                () -> plant.update(time.nextCycle(0.02)));

        // The invalid grouped command itself is atomic: the sole later child write is the
        // Plant-level compensating natural stop, never a partially mapped target.
        assertEquals(0.0, groupOutput.getCommandedVelocity(), 0.0);
        assertEquals(firstVelocityWrites + 1, first.velocityWrites.size());
        assertEquals(secondVelocityWrites + 1, second.velocityWrites.size());
        assertEquals(0.0, first.velocityWrites.get(firstVelocityWrites), 0.0);
        assertEquals(0.0, second.velocityWrites.get(secondVelocityWrites), 0.0);
        assertEquals(firstModeWrites, first.modeWrites);
        assertEquals(secondModeWrites, second.modeWrites);
        assertEquals(firstPowerWrites + 1, first.powerWrites);
        assertEquals(secondPowerWrites + 1, second.powerWrites);
        assertEquals(0.0, first.lastVelocity, 0.0);
        assertEquals(0.0, second.lastVelocity, 0.0);
        assertEquals(0.0, first.lastPower, 0.0);
        assertEquals(0.0, second.lastPower, 0.0);
        assertEquals(0.0, plant.getAppliedTarget(), 0.0);
        assertEquals(PlantTargetStatus.Kind.STOPPED, plant.getTargetStatus().kind());

        plant.commandTarget().set(0.5);
        plant.update(time.nextCycle(0.02));

        assertEquals(0.5, groupOutput.getCommandedVelocity(), 0.0);
        assertEquals(firstVelocityWrites + 2, first.velocityWrites.size());
        assertEquals(secondVelocityWrites + 2, second.velocityWrites.size());
        assertEquals(0.5, first.lastVelocity, 0.0);
        assertEquals(1.0, second.lastVelocity, 0.0);
        assertEquals(firstModeWrites + 1, first.modeWrites);
        assertEquals(secondModeWrites + 1, second.modeWrites);
        assertEquals(firstPowerWrites + 1, first.powerWrites);
        assertEquals(secondPowerWrites + 1, second.powerWrites);
        assertEquals(0.5, plant.getAppliedTarget(), 0.0);
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
    public void devicePositionPreflightIsAtomicThenPlantFailStopsAndCanRetry()
            throws Exception {
        TestHardwareMap boundedMap = new TestHardwareMap();
        boundedMap.addMotor("left");
        boundedMap.addMotor("right");
        FtcActuators.MotorGroupAddedStep boundedGroup = FtcActuators.plant(boundedMap)
                .motor("left", Direction.FORWARD)
                .andMotor("right", Direction.FORWARD)
                .bias((double) Integer.MAX_VALUE + 1.0);
        Plants.PositionCoordinateReferenceStep<Plants.SymmetricOutputPowerPolicyStep<PositionPlant>>
                reference = boundedGroup.position()
                .deviceManaged()
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
                .deviceManaged()
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

        // Runtime mapping still preflights every child before any target fanout. The subsequent
        // Plant-level fail-stop retains position commands while removing RUN_TO_POSITION power.
        // The position and calibration-power outputs are distinct owned seams even though this FTC
        // adapter happens to back both with one motor, so both zero-power stops must be attempted.
        assertEquals(1.0, groupOutput.getCommandedPosition(), 0.0);
        assertEquals(firstTargetWrites, runtimeFirst.targetPositionWrites);
        assertEquals(secondTargetWrites, runtimeSecond.targetPositionWrites);
        assertEquals(firstModeWrites + 1, runtimeFirst.modeWrites);
        assertEquals(secondModeWrites + 1, runtimeSecond.modeWrites);
        assertEquals(firstPowerWrites + 2, runtimeFirst.powerWrites);
        assertEquals(secondPowerWrites + 2, runtimeSecond.powerWrites);
        assertEquals(1, runtimeFirst.lastTargetPosition);
        assertEquals(2, runtimeSecond.lastTargetPosition);
        assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, runtimeFirst.mode);
        assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, runtimeSecond.mode);
        assertEquals(0.0, runtimeFirst.lastPower, 0.0);
        assertEquals(0.0, runtimeSecond.lastPower, 0.0);
        // The generic Plant cannot infer that an arbitrary throwing PositionOutput was atomic, so
        // its conservative effect fact remains the attempted candidate even though this grouped
        // adapter's own cache and child-write evidence prove that no partial fanout occurred.
        assertEquals((double) Integer.MAX_VALUE, runtimePlant.getAppliedTarget(), 0.0);
        assertEquals(PlantTargetStatus.Kind.STOPPED, runtimePlant.getTargetStatus().kind());

        runtimePlant.commandTarget().set(2.0);
        runtimePlant.update(runtimeClock.nextCycle(0.02));

        assertEquals(2.0, groupOutput.getCommandedPosition(), 0.0);
        assertEquals(firstTargetWrites + 1, runtimeFirst.targetPositionWrites);
        assertEquals(secondTargetWrites + 1, runtimeSecond.targetPositionWrites);
        assertEquals(firstModeWrites + 2, runtimeFirst.modeWrites);
        assertEquals(secondModeWrites + 2, runtimeSecond.modeWrites);
        assertEquals(firstPowerWrites + 3, runtimeFirst.powerWrites);
        assertEquals(secondPowerWrites + 3, runtimeSecond.powerWrites);
        assertEquals(2, runtimeFirst.lastTargetPosition);
        assertEquals(4, runtimeSecond.lastTargetPosition);
        assertEquals(DcMotor.RunMode.RUN_TO_POSITION, runtimeFirst.mode);
        assertEquals(DcMotor.RunMode.RUN_TO_POSITION, runtimeSecond.mode);
        assertEquals(2.0, runtimePlant.getAppliedTarget(), 0.0);
    }

    @Test
    public void devicePositionCalibrationSearchBypassesChildPositionMapping() throws Exception {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        MotorProbe first = hardwareMap.addMotor("first");
        MotorProbe second = hardwareMap.addMotor("second");
        PositionPlant plant = FtcActuators.plant(hardwareMap)
                .motor("first", Direction.FORWARD)
                .andMotor("second", Direction.REVERSE)
                .scale(2.0)
                .bias(0.75)
                .position()
                .deviceManaged()
                .nonPeriodic()
                .bounded(0.0, 10.0)
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(0.0)
                .targetFromNewCommand(3.0)
                .build();
        ManualLoopClock clock = new ManualLoopClock();

        plant.update(clock.clock());
        assertEquals(3, first.lastTargetPosition);
        assertEquals(7, second.lastTargetPosition);
        assertEquals(DcMotor.RunMode.RUN_TO_POSITION, first.mode);
        assertEquals(DcMotor.RunMode.RUN_TO_POSITION, second.mode);
        assertEquals(DcMotorSimple.Direction.FORWARD, first.direction);
        assertEquals(DcMotorSimple.Direction.REVERSE, second.direction);

        PowerOutput groupedSearch = privateField(plant, "searchPowerOut", PowerOutput.class);
        for (double searchPower : new double[]{-1.0, -0.4, -0.0, 0.0, 0.4, 1.0}) {
            plant.beginCalibrationSearch(searchPower);
            assertRawDoubleEquals(0.0, first.lastPower);
            assertRawDoubleEquals(0.0, second.lastPower);
            assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, first.mode);
            assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, second.mode);

            plant.update(clock.nextCycle(0.02));

            assertRawDoubleEquals(searchPower, groupedSearch.getCommandedPower());
            assertRawDoubleEquals(searchPower, first.lastPower);
            assertRawDoubleEquals(searchPower, second.lastPower);
            assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, first.mode);
            assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, second.mode);

            plant.endCalibrationSearch();
            assertRawDoubleEquals(0.0, groupedSearch.getCommandedPower());
            assertRawDoubleEquals(0.0, first.lastPower);
            assertRawDoubleEquals(0.0, second.lastPower);
            assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, first.mode);
            assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, second.mode);

            plant.update(clock.nextCycle(0.02));
            assertEquals(3, first.lastTargetPosition);
            assertEquals(7, second.lastTargetPosition);
            assertEquals(DcMotor.RunMode.RUN_TO_POSITION, first.mode);
            assertEquals(DcMotor.RunMode.RUN_TO_POSITION, second.mode);
        }

        int firstPowerWritesBeforeInvalid = first.powerWrites;
        int secondPowerWritesBeforeInvalid = second.powerWrites;
        int firstModeWrites = first.modeWrites;
        int secondModeWrites = second.modeWrites;
        DcMotor.RunMode firstMode = first.mode;
        DcMotor.RunMode secondMode = second.mode;

        // Non-finite raw input owns a group-local all-child cleanup attempt. Finite domain
        // rejection remains an all-before-effects validation path.
        expect(IllegalArgumentException.class, () -> groupedSearch.setPower(Double.NaN));
        assertTrue(Double.isNaN(groupedSearch.getCommandedPower()));
        assertEquals(firstPowerWritesBeforeInvalid + 1, first.powerWrites);
        assertEquals(secondPowerWritesBeforeInvalid + 1, second.powerWrites);
        assertEquals(firstModeWrites, first.modeWrites);
        assertEquals(secondModeWrites, second.modeWrites);
        assertEquals(firstMode, first.mode);
        assertEquals(secondMode, second.mode);
        assertRawDoubleEquals(0.0, first.lastPower);
        assertRawDoubleEquals(0.0, second.lastPower);

        int firstPowerWritesAfterCleanup = first.powerWrites;
        int secondPowerWritesAfterCleanup = second.powerWrites;
        for (double invalidPower : new double[]{-1.01, 1.01}) {
            expect(IllegalStateException.class, () -> groupedSearch.setPower(invalidPower));
            assertTrue(Double.isNaN(groupedSearch.getCommandedPower()));
            assertEquals(firstPowerWritesAfterCleanup, first.powerWrites);
            assertEquals(secondPowerWritesAfterCleanup, second.powerWrites);
            assertEquals(firstModeWrites, first.modeWrites);
            assertEquals(secondModeWrites, second.modeWrites);
            assertEquals(firstMode, first.mode);
            assertEquals(secondMode, second.mode);
            assertRawDoubleEquals(0.0, first.lastPower);
            assertRawDoubleEquals(0.0, second.lastPower);
        }
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
                .deviceManaged()
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

    private static void assertRawDoubleEquals(double expected, double actual) {
        assertEquals(Double.doubleToRawLongBits(expected), Double.doubleToRawLongBits(actual));
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
        private int powerAttempts;
        private RuntimeException stopPowerFailure;
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
                double power = (double) args[0];
                powerAttempts++;
                if (power == 0.0 && stopPowerFailure != null) {
                    throw stopPowerFailure;
                }
                lastPower = power;
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
        private int positionAttempts;
        private RuntimeException stopPositionFailure;
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
                positionAttempts++;
                if (stopPositionFailure != null) throw stopPositionFailure;
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
