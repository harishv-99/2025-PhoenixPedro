package edu.ftcsushi.fw.ftc;

import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorPIDControlLoopCoefficientsCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorTargetPositionCommand;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.junit.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.HashMap;
import java.util.Map;

import edu.ftcsushi.fw.actuation.Plant;
import edu.ftcsushi.fw.actuation.Plants;
import edu.ftcsushi.fw.actuation.PositionPlant;
import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.core.hal.PowerLimitedPositionOutput;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Focused FTC-02 coverage for fixed device-managed controller configuration. */
public final class FtcDeviceManagedControllerConfigurationValidationTest {

    private static final double COEFFICIENT_SCALE = 65536.0;
    private static final double MAX_COEFFICIENT = Integer.MAX_VALUE / COEFFICIENT_SCALE;
    private static final double MIN_COEFFICIENT = -MAX_COEFFICIENT;

    @Test
    public void pinnedSdkPublicConversionAndToleranceDomainsRemainTheValidatedAssumptions() {
        assertEquals(Integer.MAX_VALUE,
                LynxSetMotorPIDControlLoopCoefficientsCommand
                        .internalCoefficientFromExternal(MAX_COEFFICIENT));
        assertEquals(-Integer.MAX_VALUE,
                LynxSetMotorPIDControlLoopCoefficientsCommand
                        .internalCoefficientFromExternal(MIN_COEFFICIENT));

        // The immediately adjacent out-of-domain doubles silently saturate at the public SDK seam.
        assertEquals(Integer.MAX_VALUE,
                LynxSetMotorPIDControlLoopCoefficientsCommand
                        .internalCoefficientFromExternal(Math.nextUp(MAX_COEFFICIENT)));
        assertEquals(-Integer.MAX_VALUE,
                LynxSetMotorPIDControlLoopCoefficientsCommand
                        .internalCoefficientFromExternal(Math.nextDown(MIN_COEFFICIENT)));
        assertEquals(0,
                LynxSetMotorPIDControlLoopCoefficientsCommand
                        .internalCoefficientFromExternal(Double.NaN));

        assertEquals(0, LynxSetMotorTargetPositionCommand.apiToleranceFirst);
        assertEquals(65535, LynxSetMotorTargetPositionCommand.apiToleranceLast);
    }

    @Test
    public void velocityOverrideIsOneAnswerAndPositionRetainsItsMultiSettingExit()
            throws Exception {
        Method velocityPidf = FtcActuators.MotorDeviceManagedVelocityStep.class.getMethod(
                "velocityPidf", double.class, double.class, double.class, double.class);
        assertEquals(Plants.VelocityBoundsStep.class, velocityPidf.getReturnType());
        for (Method method : FtcActuators.MotorDeviceManagedVelocityStep.class
                .getDeclaredMethods()) {
            assertFalse("velocity overrides must not retain an administrative exit",
                    "doneOverrides".equals(method.getName()));
        }

        Method positionExit = FtcActuators.MotorDeviceManagedPositionStep.class.getMethod(
                "doneOverrides");
        assertEquals(Plants.PositionPeriodicityStep.class, positionExit.getReturnType());
    }

    @Test
    public void velocityPidfRejectsEveryInvalidSlotBeforeEffectsWithExactDiagnostics() {
        String[] slots = {"p", "i", "d", "f"};
        for (int slot = 0; slot < slots.length; slot++) {
            for (double invalid : invalidCoefficients()) {
                TestHardwareMap hardwareMap = mapWithMotor("flywheel");
                FtcActuators.MotorDeviceManagedVelocityStep tuning =
                        velocityTuning(hardwareMap, "flywheel");
                double[] tuple = {1.0, 2.0, 3.0, 4.0};
                tuple[slot] = invalid;

                assertExactIllegalArgument(
                        coefficientDiagnostic("FtcActuators.velocityPidf(...)", slots[slot], invalid),
                        () -> tuning.velocityPidf(tuple[0], tuple[1], tuple[2], tuple[3]));
                hardwareMap.assertNoEffects();
            }
        }
    }

    @Test
    public void positionCoefficientsRejectEveryInvalidSlotBeforeEffectsWithExactDiagnostics() {
        for (double invalid : invalidCoefficients()) {
            TestHardwareMap outerMap = mapWithMotor("arm");
            FtcActuators.MotorDeviceManagedPositionStep outer =
                    positionTuning(outerMap, "arm");
            assertExactIllegalArgument(
                    coefficientDiagnostic(
                            "FtcActuators.outerPositionP(...)", "outerPositionP", invalid),
                    () -> outer.outerPositionP(invalid));
            outerMap.assertNoEffects();
        }

        String[] slots = {"p", "i", "d", "f"};
        for (int slot = 0; slot < slots.length; slot++) {
            for (double invalid : invalidCoefficients()) {
                TestHardwareMap innerMap = mapWithMotor("arm");
                FtcActuators.MotorDeviceManagedPositionStep inner =
                        positionTuning(innerMap, "arm");
                double[] tuple = {1.0, 2.0, 3.0, 4.0};
                tuple[slot] = invalid;

                assertExactIllegalArgument(
                        coefficientDiagnostic(
                                "FtcActuators.innerVelocityPidf(...)", slots[slot], invalid),
                        () -> inner.innerVelocityPidf(
                                tuple[0], tuple[1], tuple[2], tuple[3]));
                innerMap.assertNoEffects();
            }
        }
    }

    @Test
    public void rejectedTupleDoesNotCommitAndValidRetryForwardsExactValuesAndSignedZeros() {
        TestHardwareMap velocityMap = mapWithMotor("flywheel");
        FtcActuators.MotorDeviceManagedVelocityStep velocity =
                velocityTuning(velocityMap, "flywheel");
        assertThrows(IllegalArgumentException.class,
                () -> velocity.velocityPidf(12.5, -3.25, Double.NaN, 9.0));
        velocityMap.assertNoEffects();

        double[] velocityTuple = {MIN_COEFFICIENT, -0.0, +0.0, MAX_COEFFICIENT};
        finishVelocity(velocity.velocityPidf(
                velocityTuple[0], velocityTuple[1], velocityTuple[2], velocityTuple[3]));
        MotorProbe velocityMotor = velocityMap.motor("flywheel");
        assertEquals(1, velocityMotor.velocityPidfWrites);
        assertRawArrayEquals(velocityTuple, velocityMotor.lastVelocityPidf);

        TestHardwareMap positionMap = mapWithMotor("arm");
        FtcActuators.MotorDeviceManagedPositionStep position =
                positionTuning(positionMap, "arm");
        assertThrows(IllegalArgumentException.class,
                () -> position.innerVelocityPidf(7.5, 8.5, 9.5, Double.POSITIVE_INFINITY));
        positionMap.assertNoEffects();

        double[] innerTuple = {MAX_COEFFICIENT, +0.0, -0.0, MIN_COEFFICIENT};
        Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep<
                Plants.SymmetricOutputPowerPolicyStep<PositionPlant>>> periodicity = position
                .outerPositionP(-0.0)
                .innerVelocityPidf(
                        innerTuple[0], innerTuple[1], innerTuple[2], innerTuple[3])
                .doneOverrides();
        finishPosition(periodicity, 0.0);
        MotorProbe positionMotor = positionMap.motor("arm");
        assertEquals(1, positionMotor.positionPWrites);
        assertRawDoubleEquals(-0.0, positionMotor.lastPositionP);
        assertEquals(1, positionMotor.velocityPidfWrites);
        assertRawArrayEquals(innerTuple, positionMotor.lastVelocityPidf);
    }

    @Test
    public void outerPositionPAcceptsAndForwardsExactSymmetricConversionEndpoints() {
        for (double endpoint : new double[]{MIN_COEFFICIENT, MAX_COEFFICIENT}) {
            TestHardwareMap hardwareMap = mapWithMotor("arm");
            finishPosition(positionTuning(hardwareMap, "arm")
                    .outerPositionP(endpoint)
                    .doneOverrides(), 0.0);
            MotorProbe motor = hardwareMap.motor("arm");
            assertEquals(1, motor.positionPWrites);
            assertRawDoubleEquals(endpoint, motor.lastPositionP);
        }
    }

    @Test
    public void stagedOutputPowerRejectsOutsideNormalizedDomainAndAllowsExactRetry() {
        TestHardwareMap hardwareMap = mapWithMotor("arm");
        Plants.SymmetricOutputPowerPolicyStep<PositionPlant> outputPolicy =
                positionOutputPolicy(FtcActuators.plant(hardwareMap)
                        .motor("arm", Direction.FORWARD)
                        .position()
                        .deviceManaged());
        for (double invalid : invalidOutputPowerMagnitudes()) {
            assertExactIllegalArgument(
                    maximumOutputPowerMagnitudeDiagnostic(
                            "FtcActuators.outputPowerLimitedTo(...)", invalid),
                    () -> outputPolicy.outputPowerLimitedTo(invalid));
            hardwareMap.assertNoEffects();
        }

        PositionPlant plant = outputPolicy.outputPowerLimitedTo(-0.0)
                .targetFromNewCommand(12.0)
                .build();
        plant.update(new ManualLoopClock().clock());
        assertRawDoubleEquals(-0.0, hardwareMap.motor("arm").lastPower);

        for (double valid : new double[]{+0.0, 0.25, 1.0}) {
            TestHardwareMap acceptedMap = mapWithMotor("arm");
            PositionPlant accepted = finishPositionWithOutputPower(
                    FtcActuators.plant(acceptedMap)
                            .motor("arm", Direction.FORWARD)
                            .position()
                            .deviceManaged(), 1.0, valid);
            accepted.update(new ManualLoopClock().clock());
            assertRawDoubleEquals(valid, acceptedMap.motor("arm").lastPower);
        }
    }

    @Test
    public void motorPositionPairedCommandsValidatePowerBeforeCommandEffectsAndAllowRetry() {
        TestHardwareMap namedMap = mapWithMotor("arm");
        PowerLimitedPositionOutput named = FtcHardware.motorPosition(
                namedMap, "arm", Direction.FORWARD);
        assertEquals(1, namedMap.lookupCalls);
        assertEquals(1, namedMap.motor("arm").directionWrites);
        for (double invalid : invalidOutputPowerMagnitudes()) {
            assertExactIllegalArgument(
                    maximumOutputPowerMagnitudeDiagnostic(
                            "FtcHardware.motorPosition(...).setPosition(...)", invalid),
                    () -> named.setPosition(8.0, invalid));
            assertTrue(Double.isNaN(named.getCommandedPosition()));
            assertTrue(Double.isNaN(
                    named.getCommandedMaximumOutputPowerMagnitude()));
            assertEquals(0, namedMap.motor("arm").targetWrites);
            assertEquals(0, namedMap.motor("arm").modeWrites);
            assertEquals(0, namedMap.motor("arm").powerWrites);
        }

        named.setPosition(8.0, -0.0);
        assertEquals(8.0, named.getCommandedPosition(), 0.0);
        assertRawDoubleEquals(
                -0.0, named.getCommandedMaximumOutputPowerMagnitude());
        assertRawDoubleEquals(-0.0, namedMap.motor("arm").lastPower);

        MotorProbe directProbe = new MotorProbe();
        PowerLimitedPositionOutput direct = FtcHardware.motorPosition(
                directProbe.motor, Direction.REVERSE);
        assertEquals(1, directProbe.directionWrites);
        for (double invalid : invalidOutputPowerMagnitudes()) {
            assertExactIllegalArgument(
                    maximumOutputPowerMagnitudeDiagnostic(
                            "FtcHardware.motorPosition(...).setPosition(...)", invalid),
                    () -> direct.setPosition(9.0, invalid));
            assertTrue(Double.isNaN(direct.getCommandedPosition()));
            assertTrue(Double.isNaN(
                    direct.getCommandedMaximumOutputPowerMagnitude()));
            assertEquals(0, directProbe.targetWrites);
            assertEquals(0, directProbe.modeWrites);
            assertEquals(0, directProbe.powerWrites);
        }

        direct.setPosition(9.0, 0.5);
        assertEquals(9.0, direct.getCommandedPosition(), 0.0);
        assertRawDoubleEquals(
                0.5, direct.getCommandedMaximumOutputPowerMagnitude());
        assertRawDoubleEquals(0.5, directProbe.lastPower);
        assertEquals(9, directProbe.lastTargetPosition);
    }

    @Test
    public void deviceToleranceEnforcesUnsigned16BitDomainAndPreservesRetryAndEndpoints() {
        TestHardwareMap retryMap = mapWithMotor("arm");
        FtcActuators.MotorDeviceManagedPositionStep retry =
                positionTuning(retryMap, "arm");
        assertExactIllegalArgument(toleranceDiagnostic(-1),
                () -> retry.devicePositionToleranceTicks(-1));
        assertExactIllegalArgument(toleranceDiagnostic(65536),
                () -> retry.devicePositionToleranceTicks(65536));
        retryMap.assertNoEffects();
        finishPosition(retry.devicePositionToleranceTicks(0).doneOverrides(), 0.0);
        assertEquals(1, retryMap.motor("arm").toleranceWrites);
        assertEquals(0, retryMap.motor("arm").lastToleranceTicks);

        TestHardwareMap upperMap = mapWithMotor("arm");
        finishPosition(positionTuning(upperMap, "arm")
                .devicePositionToleranceTicks(65535)
                .doneOverrides(), 0.0);
        assertEquals(1, upperMap.motor("arm").toleranceWrites);
        assertEquals(65535, upperMap.motor("arm").lastToleranceTicks);
    }

    @Test
    public void defaultsPerformNoOptionalWritesAndPositionUsesFullPower() {
        TestHardwareMap velocityMap = mapWithMotor("flywheel");
        Plant velocity = finishVelocity(FtcActuators.plant(velocityMap)
                .motor("flywheel", Direction.FORWARD)
                .velocity()
                .deviceManaged());
        velocity.update(new ManualLoopClock().clock());
        velocityMap.motor("flywheel").assertNoOptionalControllerWrites();

        TestHardwareMap positionMap = mapWithMotor("arm");
        PositionPlant position = finishPosition(FtcActuators.plant(positionMap)
                .motor("arm", Direction.FORWARD)
                .position()
                .deviceManaged(), 3.0);
        position.update(new ManualLoopClock().clock());
        MotorProbe motor = positionMap.motor("arm");
        motor.assertNoOptionalControllerWrites();
        assertRawDoubleEquals(1.0, motor.lastPower);
    }

    @Test
    public void positionTuningRequiresNonemptyClosedSectionAndClosedAliasesCannotMutate() {
        TestHardwareMap hardwareMap = mapWithMotor("arm");
        FtcActuators.MotorDeviceManagedPositionStep retained =
                positionTuning(hardwareMap, "arm");

        Throwable empty = assertThrows(IllegalStateException.class, retained::doneOverrides);
        assertContainsIgnoreCase(empty, "at least", "override");
        hardwareMap.assertNoEffects();

        Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep<
                Plants.SymmetricOutputPowerPolicyStep<PositionPlant>>> closed = retained
                .outerPositionP(2.0)
                .doneOverrides();
        Throwable repeatedClose =
                assertThrows(IllegalStateException.class, retained::doneOverrides);
        assertContainsIgnoreCase(repeatedClose, "closed");
        Throwable mutationAfterClose = assertThrows(
                IllegalStateException.class, () -> retained.devicePositionToleranceTicks(-1));
        assertContainsIgnoreCase(mutationAfterClose, "closed");
        hardwareMap.assertNoEffects();

        PositionPlant plant = finishPositionWithOutputPower(closed, 4.0, 0.4);
        plant.update(new ManualLoopClock().clock());
        assertRawDoubleEquals(0.4, hardwareMap.motor("arm").lastPower);
    }

    @Test
    public void distinctPositionSettingsAreOrderIndependentAndDuplicatesPreserveFirstAnswers() {
        TestHardwareMap hardwareMap = mapWithMotor("arm");
        FtcActuators.MotorDeviceManagedPositionStep retained =
                positionTuning(hardwareMap, "arm");
        double[] acceptedInner = {-4.0, 3.0, -2.0, 1.0};

        retained.devicePositionToleranceTicks(27)
                .innerVelocityPidf(
                        acceptedInner[0], acceptedInner[1], acceptedInner[2], acceptedInner[3])
                .outerPositionP(-5.0);

        assertDuplicate(() -> retained.devicePositionToleranceTicks(65536));
        assertDuplicate(() -> retained.innerVelocityPidf(
                Double.NaN, Double.NaN, Double.NaN, Double.NaN));
        assertDuplicate(() -> retained.outerPositionP(Double.NaN));
        hardwareMap.assertNoEffects();

        PositionPlant plant = finishPositionWithOutputPower(
                retained.doneOverrides(), 2.0, 0.6);
        plant.update(new ManualLoopClock().clock());
        MotorProbe motor = hardwareMap.motor("arm");
        assertEquals(27, motor.lastToleranceTicks);
        assertRawArrayEquals(acceptedInner, motor.lastVelocityPidf);
        assertRawDoubleEquals(0.6, motor.lastPower);
        assertRawDoubleEquals(-5.0, motor.lastPositionP);
    }

    @Test
    public void acceptedVelocityAnswerClosesRetainedTuningAliasWithoutReplacingTuple() {
        TestHardwareMap hardwareMap = mapWithMotor("flywheel");
        FtcActuators.MotorDeviceManagedVelocityStep retained =
                velocityTuning(hardwareMap, "flywheel");
        double[] accepted = {1.25, -2.5, 3.75, -0.0};
        Plants.VelocityBoundsStep<Plants.TargetStep<Plant>> bounds = retained.velocityPidf(
                accepted[0], accepted[1], accepted[2], accepted[3]);

        Throwable duplicate = assertThrows(IllegalStateException.class,
                () -> retained.velocityPidf(Double.NaN, 0.0, 0.0, 0.0));
        assertContainsIgnoreCase(duplicate, "already", "answered");
        hardwareMap.assertNoEffects();

        finishVelocity(bounds);
        assertRawArrayEquals(accepted, hardwareMap.motor("flywheel").lastVelocityPidf);
    }

    @Test
    public void defaultStagesRejectTuningCastsAndRemainUsable() {
        TestHardwareMap velocityMap = mapWithMotor("flywheel");
        Plants.VelocityBoundsStep<Plants.TargetStep<Plant>> velocityDefaults =
                FtcActuators.plant(velocityMap)
                .motor("flywheel", Direction.FORWARD)
                .velocity()
                .deviceManaged();
        assertThrows(ClassCastException.class, () -> {
            FtcActuators.MotorDeviceManagedVelocityStep ignored =
                    (FtcActuators.MotorDeviceManagedVelocityStep) velocityDefaults;
        });
        velocityMap.assertNoEffects();
        finishVelocity(velocityDefaults);
        velocityMap.motor("flywheel").assertNoOptionalControllerWrites();

        TestHardwareMap positionMap = mapWithMotor("arm");
        Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep<
                Plants.SymmetricOutputPowerPolicyStep<PositionPlant>>> positionDefaults =
                FtcActuators.plant(positionMap)
                        .motor("arm", Direction.FORWARD)
                        .position()
                        .deviceManaged();
        assertThrows(ClassCastException.class, () -> {
            FtcActuators.MotorDeviceManagedPositionStep ignored =
                    (FtcActuators.MotorDeviceManagedPositionStep) positionDefaults;
        });
        positionMap.assertNoEffects();
        finishPosition(positionDefaults, 0.0);
        positionMap.motor("arm").assertNoOptionalControllerWrites();
    }

    @Test
    @SuppressWarnings("unchecked")
    public void castBypassesStillFailFinalPreflightBeforeHardwareResolution() {
        TestHardwareMap velocityMap = mapWithMotor("left");
        velocityMap.addMotor("right");
        FtcActuators.MotorDeviceManagedVelocityStep velocityTuning = FtcActuators
                .plant(velocityMap)
                .motor("left", Direction.FORWARD)
                .andMotor("right", Direction.REVERSE)
                .velocity()
                .deviceManagedWithOverrides();
        Plants.TargetStep<Plant> velocityBypass =
                (Plants.TargetStep<Plant>) velocityTuning;
        Throwable velocityFailure = assertThrows(
                IllegalStateException.class,
                () -> velocityBypass.targetFromNewCommand(0.0));
        assertContainsIgnoreCase(velocityFailure, "velocityPidf");
        velocityMap.assertNoEffects();

        TestHardwareMap positionMap = mapWithMotor("left");
        positionMap.addMotor("right");
        FtcActuators.MotorDeviceManagedPositionStep positionTuning = FtcActuators
                .plant(positionMap)
                .motor("left", Direction.FORWARD)
                .andMotor("right", Direction.REVERSE)
                .position()
                .deviceManagedWithOverrides();
        Plants.TargetStep<PositionPlant> positionBypass =
                (Plants.TargetStep<PositionPlant>) positionTuning;
        Throwable positionFailure = assertThrows(
                IllegalStateException.class,
                () -> positionBypass.targetFromNewCommand(0.0));
        assertContainsIgnoreCase(positionFailure, "doneOverrides");
        positionMap.assertNoEffects();
    }

    @Test
    public void validConfiguredGroupForwardsOneCompleteConfigurationToEveryChild() {
        TestHardwareMap hardwareMap = mapWithMotor("left");
        hardwareMap.addMotor("right");
        double[] inner = {-1.0, 2.0, -3.0, 4.0};
        FtcActuators.MotorDeviceManagedPositionStep tuning = FtcActuators.plant(hardwareMap)
                .motor("left", Direction.FORWARD)
                .andMotor("right", Direction.REVERSE)
                .position()
                .deviceManagedWithOverrides();
        assertThrows(IllegalArgumentException.class,
                () -> tuning.innerVelocityPidf(1.0, 2.0, 3.0, Double.NaN));
        hardwareMap.assertNoEffects();

        PositionPlant plant = finishPositionWithOutputPower(tuning
                .devicePositionToleranceTicks(31)
                .innerVelocityPidf(inner[0], inner[1], inner[2], inner[3])
                .outerPositionP(5.0)
                .doneOverrides(), 16.0, 0.75);
        plant.update(new ManualLoopClock().clock());

        for (String name : new String[]{"left", "right"}) {
            MotorProbe motor = hardwareMap.motor(name);
            assertEquals(1, motor.positionPWrites);
            assertRawDoubleEquals(5.0, motor.lastPositionP);
            assertEquals(1, motor.velocityPidfWrites);
            assertRawArrayEquals(inner, motor.lastVelocityPidf);
            assertEquals(1, motor.toleranceWrites);
            assertEquals(31, motor.lastToleranceTicks);
            assertRawDoubleEquals(0.75, motor.lastPower);
        }
    }

    @Test
    public void structurallyValidSdkFailureKeepsMotorNamedWrapperAndOriginalCause() {
        TestHardwareMap hardwareMap = mapWithMotor("flywheel");
        MotorProbe motor = hardwareMap.motor("flywheel");
        RuntimeException sdkFailure = new UnsupportedOperationException("test SDK rejection");
        motor.velocityPidfFailure = sdkFailure;

        Throwable failure = assertThrows(IllegalStateException.class,
                () -> finishVelocity(velocityTuning(hardwareMap, "flywheel")
                        .velocityPidf(1.0, 2.0, 3.0, 4.0)));

        assertContainsIgnoreCase(failure, "device-managed velocity", "flywheel", "SDK");
        assertSame(sdkFailure, failure.getCause());
        assertEquals(1, motor.velocityPidfAttempts);
        assertEquals(0, motor.velocityPidfWrites);
    }

    private static FtcActuators.MotorDeviceManagedVelocityStep velocityTuning(
            TestHardwareMap hardwareMap,
            String motorName) {
        return FtcActuators.plant(hardwareMap)
                .motor(motorName, Direction.FORWARD)
                .velocity()
                .deviceManagedWithOverrides();
    }

    private static FtcActuators.MotorDeviceManagedPositionStep positionTuning(
            TestHardwareMap hardwareMap,
            String motorName) {
        return FtcActuators.plant(hardwareMap)
                .motor(motorName, Direction.FORWARD)
                .position()
                .deviceManagedWithOverrides();
    }

    private static Plant finishVelocity(
            Plants.VelocityBoundsStep<Plants.TargetStep<Plant>> bounds) {
        return bounds
                .unbounded()
                .nativeUnits()
                .velocityTolerance(0.0)
                .targetFromNewCommand(0.0)
                .build();
    }

    private static PositionPlant finishPosition(
            Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep<
                    Plants.SymmetricOutputPowerPolicyStep<PositionPlant>>> periodicity,
            double target) {
        return periodicity
                .nonPeriodic()
                .unbounded()
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(0.0)
                .targetFromNewCommand(target)
                .build();
    }

    private static PositionPlant finishPositionWithOutputPower(
            Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep<
                    Plants.SymmetricOutputPowerPolicyStep<PositionPlant>>> periodicity,
            double target,
            double maximumPower) {
        return positionOutputPolicy(periodicity)
                .outputPowerLimitedTo(maximumPower)
                .targetFromNewCommand(target)
                .build();
    }

    private static Plants.SymmetricOutputPowerPolicyStep<PositionPlant> positionOutputPolicy(
            Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep<
                    Plants.SymmetricOutputPowerPolicyStep<PositionPlant>>> periodicity) {
        return periodicity
                .nonPeriodic()
                .unbounded()
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(0.0);
    }

    private static TestHardwareMap mapWithMotor(String name) {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        hardwareMap.addMotor(name);
        return hardwareMap;
    }

    private static double[] invalidCoefficients() {
        return new double[]{
                Double.NaN,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY,
                Math.nextDown(MIN_COEFFICIENT),
                Math.nextUp(MAX_COEFFICIENT)
        };
    }

    private static double[] invalidOutputPowerMagnitudes() {
        return new double[]{
                Double.NaN,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY,
                Math.nextDown(0.0),
                Math.nextUp(1.0)
        };
    }

    private static String coefficientDiagnostic(String operation, String argument, double value) {
        return operation + ": " + argument
                + " must be finite in FTC controller coefficient units within the inclusive ["
                + MIN_COEFFICIENT + ", " + MAX_COEFFICIENT
                + "] REV conversion domain, got " + value;
    }

    private static String maximumOutputPowerMagnitudeDiagnostic(
            String operation,
            double value) {
        return operation
                + ": maximum output-power magnitude must be finite normalized "
                + "RUN_TO_POSITION power within "
                + "the inclusive [0.0, 1.0] domain, got " + value;
    }

    private static String toleranceDiagnostic(int ticks) {
        return "FtcActuators.devicePositionToleranceTicks(...): ticks must be native FTC "
                + "controller tolerance "
                + "ticks within the inclusive [0, 65535] domain, got " + ticks;
    }

    private static void assertDuplicate(Runnable action) {
        Throwable failure = assertThrows(IllegalStateException.class, action);
        assertContainsIgnoreCase(failure, "already", "answered");
    }

    private static void assertExactIllegalArgument(String expectedMessage, Runnable action) {
        Throwable failure = assertThrows(IllegalArgumentException.class, action);
        assertEquals(expectedMessage, failure.getMessage());
    }

    private static Throwable assertThrows(
            Class<? extends RuntimeException> expected,
            Runnable action) {
        try {
            action.run();
        } catch (RuntimeException failure) {
            assertTrue("Expected " + expected.getSimpleName() + ", got " + failure,
                    expected.isInstance(failure));
            return failure;
        }
        fail("Expected " + expected.getSimpleName());
        return null;
    }

    private static void assertContainsIgnoreCase(Throwable failure, String... fragments) {
        assertNotNull(failure);
        String message = failure.getMessage();
        assertNotNull(message);
        String lowered = message.toLowerCase();
        for (String fragment : fragments) {
            assertTrue("Expected '" + message + "' to contain '" + fragment + "'",
                    lowered.contains(fragment.toLowerCase()));
        }
    }

    private static void assertRawArrayEquals(double[] expected, double[] actual) {
        assertNotNull(actual);
        assertEquals(expected.length, actual.length);
        for (int i = 0; i < expected.length; i++) {
            assertRawDoubleEquals(expected[i], actual[i]);
        }
    }

    private static void assertRawDoubleEquals(double expected, double actual) {
        assertEquals(Double.doubleToRawLongBits(expected), Double.doubleToRawLongBits(actual));
    }

    /** In-memory HardwareMap with observable configuration, lookup, direction, and command effects. */
    private static final class TestHardwareMap extends HardwareMap {
        private final Map<String, MotorProbe> motors = new HashMap<>();
        private int lookupCalls;

        private TestHardwareMap() {
            super(null, null);
        }

        private MotorProbe addMotor(String name) {
            MotorProbe probe = new MotorProbe();
            motors.put(name, probe);
            return probe;
        }

        private MotorProbe motor(String name) {
            MotorProbe probe = motors.get(name);
            assertNotNull("No test motor named " + name, probe);
            return probe;
        }

        @Override
        public <T> T get(Class<? extends T> type, String name) {
            lookupCalls++;
            MotorProbe probe = motors.get(name);
            if (probe == null) {
                throw new IllegalArgumentException("No test motor named " + name);
            }
            if (!type.isInstance(probe.motor)) {
                throw new IllegalArgumentException("Wrong test device type for " + name);
            }
            return type.cast(probe.motor);
        }

        private void assertNoEffects() {
            assertEquals(0, lookupCalls);
            for (MotorProbe motor : motors.values()) {
                motor.assertNoEffects();
            }
        }
    }

    /** Dynamic DcMotorEx probe that retains raw doubles at each controller seam. */
    private static final class MotorProbe {
        private final DcMotorEx motor;
        private int directionWrites;
        private int velocityPidfAttempts;
        private int velocityPidfWrites;
        private int positionPWrites;
        private int toleranceWrites;
        private int targetWrites;
        private int modeWrites;
        private int powerWrites;
        private double[] lastVelocityPidf;
        private double lastPositionP;
        private int lastToleranceTicks;
        private int lastTargetPosition;
        private double lastPower;
        private double lastVelocity;
        private int currentPosition;
        private DcMotor.RunMode mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        private DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
        private RuntimeException velocityPidfFailure;
        private PIDFCoefficients positionPidf = new PIDFCoefficients(
                0.0, 0.0, 0.0, 0.0, MotorControlAlgorithm.PIDF);
        private PIDFCoefficients velocityPidf = new PIDFCoefficients(
                0.0, 0.0, 0.0, 0.0, MotorControlAlgorithm.PIDF);

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
                directionWrites++;
                return null;
            }
            if ("getDirection".equals(name)) return direction;
            if ("setVelocityPIDFCoefficients".equals(name)) {
                velocityPidfAttempts++;
                if (velocityPidfFailure != null) throw velocityPidfFailure;
                lastVelocityPidf = new double[]{
                        (double) args[0], (double) args[1],
                        (double) args[2], (double) args[3]
                };
                velocityPidf = new PIDFCoefficients(
                        lastVelocityPidf[0], lastVelocityPidf[1],
                        lastVelocityPidf[2], lastVelocityPidf[3],
                        MotorControlAlgorithm.PIDF);
                velocityPidfWrites++;
                return null;
            }
            if ("setPositionPIDFCoefficients".equals(name)) {
                lastPositionP = (double) args[0];
                positionPidf = new PIDFCoefficients(
                        lastPositionP, 0.0, 0.0, 0.0, MotorControlAlgorithm.PIDF);
                positionPWrites++;
                return null;
            }
            if ("getPIDFCoefficients".equals(name)) {
                return new PIDFCoefficients(
                        args[0] == DcMotor.RunMode.RUN_TO_POSITION
                                ? positionPidf : velocityPidf);
            }
            if ("setPIDFCoefficients".equals(name)) {
                PIDFCoefficients value = new PIDFCoefficients((PIDFCoefficients) args[1]);
                if (args[0] == DcMotor.RunMode.RUN_TO_POSITION) positionPidf = value;
                else velocityPidf = value;
                return null;
            }
            if ("setTargetPositionTolerance".equals(name)) {
                lastToleranceTicks = (int) args[0];
                toleranceWrites++;
                return null;
            }
            if ("getTargetPositionTolerance".equals(name)) return lastToleranceTicks;
            if ("setTargetPosition".equals(name)) {
                lastTargetPosition = (int) args[0];
                targetWrites++;
                return null;
            }
            if ("getTargetPosition".equals(name)) return lastTargetPosition;
            if ("setMode".equals(name)) {
                mode = (DcMotor.RunMode) args[0];
                modeWrites++;
                return null;
            }
            if ("getMode".equals(name)) return mode;
            if ("setPower".equals(name)) {
                lastPower = (double) args[0];
                powerWrites++;
                return null;
            }
            if ("getPower".equals(name)) return lastPower;
            if ("setVelocity".equals(name)) {
                lastVelocity = (double) args[0];
                return null;
            }
            if ("getVelocity".equals(name)) return lastVelocity;
            if ("getCurrentPosition".equals(name)) return currentPosition;
            if ("getZeroPowerBehavior".equals(name)) return DcMotor.ZeroPowerBehavior.FLOAT;
            if ("isBusy".equals(name)) return false;
            return hardwareDeviceOrDefault(method, "MotorProbe");
        }

        private void assertNoEffects() {
            assertEquals(0, directionWrites);
            assertEquals(0, velocityPidfAttempts);
            assertEquals(0, positionPWrites);
            assertEquals(0, toleranceWrites);
            assertEquals(0, targetWrites);
            assertEquals(0, modeWrites);
            assertEquals(0, powerWrites);
        }

        private void assertNoOptionalControllerWrites() {
            assertEquals(0, velocityPidfAttempts);
            assertEquals(0, positionPWrites);
            assertEquals(0, toleranceWrites);
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
