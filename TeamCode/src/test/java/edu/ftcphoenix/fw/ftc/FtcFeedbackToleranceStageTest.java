package edu.ftcphoenix.fw.ftc;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.Test;

import java.lang.reflect.Method;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.PlantTargetResolver;
import edu.ftcphoenix.fw.actuation.PlantTargets;
import edu.ftcphoenix.fw.actuation.Plants;
import edu.ftcphoenix.fw.actuation.PositionPlant;
import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.control.ScalarRegulator;
import edu.ftcphoenix.fw.core.source.BooleanSource;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies that every FTC feedback Plant explicitly chooses tolerance in its public units. */
public final class FtcFeedbackToleranceStageTest {

    @Test
    public void feedbackMappingStagesRequireParallelToleranceAnswers() throws Exception {
        assertReturns(Plants.VelocityMappingStep.class, "nativeUnits",
                Plants.VelocityToleranceStep.class);
        assertReturns(Plants.VelocityMappingStep.class, "scaleToNative",
                Plants.VelocityToleranceStep.class, double.class);
        assertReturns(Plants.VelocityToleranceStep.class, "velocityTolerance",
                Object.class, double.class);
        assertEquals(1, Plants.VelocityToleranceStep.class.getDeclaredMethods().length);

        assertReturns(Plants.FeedbackBoundedPositionMappingStep.class, "rangeMapsToNative",
                Plants.PositionToleranceStep.class, double.class, double.class);
        assertReturns(Plants.PositionCoordinateReferenceStep.class, "alreadyReferenced",
                Plants.PositionToleranceStep.class);
        assertReturns(Plants.PositionCoordinateReferenceStep.class, "plantPositionMapsToNative",
                Plants.PositionToleranceStep.class, double.class, double.class);
        assertReturns(Plants.PositionCoordinateReferenceStep.class, "assumeCurrentPositionIs",
                Plants.PositionToleranceStep.class, double.class);
        assertReturns(Plants.PositionCoordinateReferenceStep.class, "needsReference",
                Plants.PositionToleranceStep.class, String.class);
        assertReturns(Plants.PositionToleranceStep.class, "positionTolerance",
                Object.class, double.class);
        assertEquals(1, Plants.PositionToleranceStep.class.getDeclaredMethods().length);

        assertReturns(FtcActuators.ServoBoundedPositionMappingStep.class, "nativeUnits",
                Plants.TargetStep.class);
        assertReturns(FtcActuators.ServoBoundedPositionMappingStep.class, "rangeMapsToNative",
                Plants.TargetStep.class, double.class, double.class);

        assertReturns(Plants.TargetStep.class, "targetGuards", Plants.TargetGuardStep.class);
        assertReturns(Plants.TargetStep.class, "targetFromNewCommand",
                Plants.BuildStep.class, double.class);
        assertReturns(Plants.TargetStep.class, "targetFromResolver",
                Plants.BuildStep.class, PlantTargetResolver.class);
        assertEquals(3, Plants.TargetStep.class.getDeclaredMethods().length);

        for (Class<?> nested : FtcActuators.class.getDeclaredClasses()) {
            assertNotEquals("VelocityBuildStep", nested.getSimpleName());
            assertNotEquals("PositionBuildStep", nested.getSimpleName());
            assertNotEquals("ServoPositionBuildStep", nested.getSimpleName());
            assertNotEquals("PlantTargetStep", nested.getSimpleName());
            assertNotEquals("PositionTargetStep", nested.getSimpleName());
            assertNotEquals("PlantTargetGuardStep", nested.getSimpleName());
            assertNotEquals("PositionTargetGuardStep", nested.getSimpleName());
        }
    }

    @Test
    public void toleranceAnswerIsValidatedAndCannotBeReplacedThroughRetainedStage() {
        HardwareMap hardwareMap = new HardwareMap(null, null);
        Plants.VelocityToleranceStep<Plants.TargetStep<Plant>> velocity =
                FtcActuators.plant(hardwareMap)
                .motor("flywheel", Direction.FORWARD)
                .velocity()
                .deviceManaged()
                .unbounded()
                .nativeUnits();

        assertIllegalArgument(() -> velocity.velocityTolerance(Double.NaN),
                "velocityTolerance", "finite", ">= 0");
        Plants.TargetStep<Plant> velocityTarget = velocity.velocityTolerance(0.0);
        assertTrue(velocityTarget != null);
        assertIllegalState(() -> velocity.velocityTolerance(1.0),
                "velocityTolerance(...) has already been answered");

        Plants.PositionToleranceStep<Plants.SymmetricOutputPowerPolicyStep<PositionPlant>> position =
                FtcActuators.plant(hardwareMap)
                .motor("lift", Direction.FORWARD)
                .position()
                .deviceManaged()
                .nonPeriodic()
                .unbounded()
                .nativeUnits()
                .alreadyReferenced();

        assertIllegalArgument(() -> position.positionTolerance(Double.POSITIVE_INFINITY),
                "positionTolerance", "finite", ">= 0");
        Plants.SymmetricOutputPowerPolicyStep<PositionPlant> positionTarget =
                position.positionTolerance(0.0);
        assertTrue(positionTarget != null);
        assertIllegalState(() -> position.positionTolerance(1.0),
                "positionTolerance(...) has already been answered");
    }

    @Test
    public void retainedFtcPidAliasCannotReorderLaterControlStages() {
        HardwareMap hardwareMap = new HardwareMap(null, null);
        Plants.PositionDirectPidStep pid = FtcActuators.plant(hardwareMap)
                .motor("lift", Direction.FORWARD)
                .position()
                .regulated()
                .nativeFeedback(clock -> 0.0)
                .nonPeriodic()
                .unbounded()
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(0.0)
                .setpointFromAppliedTarget()
                .feedbackFromPid(0.1);

        Plants.OutputPowerPolicyStep<PositionPlant> output =
                pid.feedforwardFromLift(0.2);
        assertIllegalState(() -> pid.feedbackIntegralLimitedTo(-0.1, 0.1),
                "cannot change PID/feedforward");
        assertIllegalState(() -> pid.feedforwardFromArm(0.2, 0.0, 1.0),
                "cannot change PID/feedforward");

        output.outputPowerLimitedTo(0.5);
        assertIllegalState(() -> pid.feedbackOutputLimitedTo(-0.2, 0.2),
                "cannot change PID/feedforward");
    }

    @Test
    public void ftcProfiledSetpointsRejectTargetRateGuardsBeforeRecipeRetention() {
        HardwareMap hardwareMap = new HardwareMap(null, null);
        BooleanSource allowed = clock -> true;
        Plants.PositionProfiledPidStep position = FtcActuators.plant(hardwareMap)
                .motor("lift", Direction.FORWARD)
                .position()
                .regulated()
                .nativeFeedback(clock -> 0.0)
                .nonPeriodic()
                .bounded(-2.0, 2.0)
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(0.01)
                .setpointFromTrapezoidalProfile(1.0, 2.0)
                .feedbackFromPid(0.1);
        Plants.TargetGuardStep<PositionPlant> positionGuards = position.targetGuards();
        assertIllegalState(() -> positionGuards.maxTargetRate(1.0),
                "setpointFromTrapezoidalProfile", "one motion-shaping owner");
        assertTrue(positionGuards.holdLastTargetUnless("ready", allowed)
                .doneTargetGuards()
                .targetFromNewCommand(0.0) != null);

        Plants.VelocityProfiledPidStep velocity = FtcActuators.plant(hardwareMap)
                .motor("flywheel", Direction.FORWARD)
                .velocity()
                .regulated()
                .nativeFeedback(clock -> 0.0)
                .bounded(-2.0, 2.0)
                .nativeUnits()
                .velocityTolerance(0.01)
                .setpointFromAccelerationLimitedProfile(2.0)
                .feedbackFromPid(0.1);
        Plants.TargetGuardStep<Plant> velocityGuards = velocity.targetGuards();
        assertIllegalState(() -> velocityGuards.maxTargetRates(1.0, 1.0),
                "setpointFromAccelerationLimitedProfile", "one motion-shaping owner");
        assertTrue(velocityGuards.holdLastTargetUnless("ready", allowed)
                .doneTargetGuards()
                .targetFromNewCommand(0.0) != null);
    }

    @Test
    public void nullCustomRegulatorsDoNotPoisonFtcControlSelection() {
        HardwareMap hardwareMap = new HardwareMap(null, null);
        ScalarRegulator valid = (setpoint, measurement, clock) -> 0.0;

        Plants.PositionControlStep position = FtcActuators.plant(hardwareMap)
                .motor("lift", Direction.FORWARD)
                .position()
                .regulated()
                .nativeFeedback(clock -> 0.0)
                .nonPeriodic()
                .bounded(-1.0, 1.0)
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(0.01);
        assertNullPointer(() -> position.controlFromCustomRegulator(null), "regulator");
        assertTrue(position.controlFromCustomRegulator(valid)
                .targetFromNewCommand(0.0) != null);

        Plants.VelocityControlStep velocity = FtcActuators.plant(hardwareMap)
                .motor("flywheel", Direction.FORWARD)
                .velocity()
                .regulated()
                .nativeFeedback(clock -> 0.0)
                .bounded(-1.0, 1.0)
                .nativeUnits()
                .velocityTolerance(0.01);
        assertNullPointer(() -> velocity.controlFromCustomRegulator(null), "regulator");
        assertTrue(velocity.controlFromCustomRegulator(valid)
                .targetFromNewCommand(0.0) != null);
    }

    @Test
    public void runtimeStageBypassFailsAtTargetAnswerWithoutPoisoningRecipe() {
        HardwareMap hardwareMap = new HardwareMap(null, null);
        Plants.VelocityToleranceStep<Plants.TargetStep<Plant>> velocity =
                FtcActuators.plant(hardwareMap)
                .motor("flywheel", Direction.FORWARD)
                .velocity()
                .deviceManaged()
                .unbounded()
                .nativeUnits();
        assertClassCast(() -> {
            @SuppressWarnings("unchecked")
            Plants.TargetStep<Plant> ignored =
                    (Plants.TargetStep<Plant>) (Object) velocity;
        });
        assertTrue(velocity.velocityTolerance(0.0).targetFromNewCommand(0.0) != null);

        Plants.PositionToleranceStep<Plants.SymmetricOutputPowerPolicyStep<PositionPlant>> position =
                FtcActuators.plant(hardwareMap)
                .motor("lift", Direction.FORWARD)
                .position()
                .deviceManaged()
                .nonPeriodic()
                .unbounded()
                .nativeUnits()
                .alreadyReferenced();
        assertClassCast(() -> {
            @SuppressWarnings("unchecked")
            Plants.TargetStep<PositionPlant> ignored =
                    (Plants.TargetStep<PositionPlant>) (Object) position;
        });
        assertTrue(position.positionTolerance(0.0)
                .targetFromResolver(PlantTargets.exact(0.0)) != null);
    }

    private static void assertReturns(Class<?> owner, String methodName, Class<?> returnType,
                                      Class<?>... parameterTypes) throws Exception {
        Method method = owner.getMethod(methodName, parameterTypes);
        assertEquals(returnType, method.getReturnType());
    }

    private static void assertIllegalArgument(Runnable action, String... fragments) {
        try {
            action.run();
            fail("Expected IllegalArgumentException");
        } catch (IllegalArgumentException expected) {
            assertContains(expected.getMessage(), fragments);
        }
    }

    private static void assertIllegalState(Runnable action, String... fragments) {
        try {
            action.run();
            fail("Expected IllegalStateException");
        } catch (IllegalStateException expected) {
            assertContains(expected.getMessage(), fragments);
        }
    }

    private static void assertClassCast(Runnable action) {
        try {
            action.run();
            fail("Expected ClassCastException");
        } catch (ClassCastException expected) {
            // Expected: staged wrappers do not implement later grammar stages.
        }
    }

    private static void assertNullPointer(Runnable action, String... fragments) {
        try {
            action.run();
            fail("Expected NullPointerException");
        } catch (NullPointerException expected) {
            assertContains(expected.getMessage(), fragments);
        }
    }

    private static void assertContains(String message, String... fragments) {
        assertTrue("Expected an actionable exception message", message != null);
        for (String fragment : fragments) {
            assertTrue("Expected '" + message + "' to contain '" + fragment + "'",
                    message.contains(fragment));
        }
    }
}
