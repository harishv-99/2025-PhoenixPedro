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
                Plants.TargetStep.class, double.class);
        assertEquals(1, Plants.VelocityToleranceStep.class.getDeclaredMethods().length);

        assertReturns(Plants.FeedbackBoundedPositionMappingStep.class, "rangeMapsToNative",
                Plants.PositionToleranceStep.class, double.class, double.class);
        assertReturns(Plants.PositionReferenceStep.class, "alreadyReferenced",
                Plants.PositionToleranceStep.class);
        assertReturns(Plants.PositionReferenceStep.class, "plantPositionMapsToNative",
                Plants.PositionToleranceStep.class, double.class, double.class);
        assertReturns(Plants.PositionReferenceStep.class, "assumeCurrentPositionIs",
                Plants.PositionToleranceStep.class, double.class);
        assertReturns(Plants.PositionReferenceStep.class, "needsReference",
                Plants.PositionToleranceStep.class, String.class);
        assertReturns(Plants.PositionToleranceStep.class, "positionTolerance",
                Plants.TargetStep.class, double.class);
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
        Plants.VelocityToleranceStep velocity = FtcActuators.plant(hardwareMap)
                .motor("flywheel", Direction.FORWARD)
                .velocity()
                .deviceManagedWithDefaults()
                .unbounded()
                .nativeUnits();

        assertIllegalArgument(() -> velocity.velocityTolerance(Double.NaN),
                "velocityTolerance", "finite", ">= 0");
        Plants.TargetStep<Plant> velocityTarget = velocity.velocityTolerance(0.0);
        assertTrue(velocityTarget != null);
        assertIllegalState(() -> velocity.velocityTolerance(1.0),
                "velocityTolerance(...) has already been answered");

        Plants.PositionToleranceStep position = FtcActuators.plant(hardwareMap)
                .motor("lift", Direction.FORWARD)
                .position()
                .deviceManagedWithDefaults()
                .nonPeriodic()
                .unbounded()
                .nativeUnits()
                .alreadyReferenced();

        assertIllegalArgument(() -> position.positionTolerance(Double.POSITIVE_INFINITY),
                "positionTolerance", "finite", ">= 0");
        Plants.TargetStep<PositionPlant> positionTarget = position.positionTolerance(0.0);
        assertTrue(positionTarget != null);
        assertIllegalState(() -> position.positionTolerance(1.0),
                "positionTolerance(...) has already been answered");
    }

    @Test
    public void runtimeStageBypassFailsAtTargetAnswerWithoutPoisoningRecipe() {
        HardwareMap hardwareMap = new HardwareMap(null, null);
        Plants.VelocityToleranceStep velocity = FtcActuators.plant(hardwareMap)
                .motor("flywheel", Direction.FORWARD)
                .velocity()
                .deviceManagedWithDefaults()
                .unbounded()
                .nativeUnits();
        @SuppressWarnings("unchecked")
        Plants.TargetStep<Plant> velocityBypass =
                (Plants.TargetStep<Plant>) (Object) velocity;

        assertIllegalState(() -> velocityBypass.targetFromNewCommand(0.0),
                "requires velocityTolerance(...)", "plant velocity units");
        assertTrue(velocity.velocityTolerance(0.0).targetFromNewCommand(0.0) != null);

        Plants.PositionToleranceStep position = FtcActuators.plant(hardwareMap)
                .motor("lift", Direction.FORWARD)
                .position()
                .deviceManagedWithDefaults()
                .nonPeriodic()
                .unbounded()
                .nativeUnits()
                .alreadyReferenced();
        @SuppressWarnings("unchecked")
        Plants.TargetStep<PositionPlant> positionBypass =
                (Plants.TargetStep<PositionPlant>) (Object) position;

        assertIllegalState(
                () -> positionBypass.targetFromResolver(PlantTargets.exact(0.0)),
                "requires positionTolerance(...)", "plant position units");
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

    private static void assertContains(String message, String... fragments) {
        assertTrue("Expected an actionable exception message", message != null);
        for (String fragment : fragments) {
            assertTrue("Expected '" + message + "' to contain '" + fragment + "'",
                    message.contains(fragment));
        }
    }
}
