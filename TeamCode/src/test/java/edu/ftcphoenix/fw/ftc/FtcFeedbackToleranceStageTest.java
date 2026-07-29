package edu.ftcphoenix.fw.ftc;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.Test;

import java.lang.reflect.Method;

import edu.ftcphoenix.fw.actuation.PlantTargetResolver;
import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.source.ScalarTarget;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies that every FTC feedback Plant explicitly chooses tolerance in its public units. */
public final class FtcFeedbackToleranceStageTest {

    @Test
    public void feedbackMappingStagesRequireParallelToleranceAnswers() throws Exception {
        assertReturns(FtcActuators.VelocityMappingStep.class, "nativeUnits",
                FtcActuators.VelocityToleranceStep.class);
        assertReturns(FtcActuators.VelocityMappingStep.class, "scaleToNative",
                FtcActuators.VelocityToleranceStep.class, double.class);
        assertReturns(FtcActuators.VelocityToleranceStep.class, "velocityTolerance",
                FtcActuators.PlantTargetStep.class, double.class);
        assertEquals(1, FtcActuators.VelocityToleranceStep.class.getDeclaredMethods().length);

        assertReturns(FtcActuators.BoundedPositionMappingStep.class, "rangeMapsToNative",
                FtcActuators.PositionToleranceStep.class, double.class, double.class);
        assertReturns(FtcActuators.PositionReferenceStep.class, "alreadyReferenced",
                FtcActuators.PositionToleranceStep.class);
        assertReturns(FtcActuators.PositionReferenceStep.class, "plantPositionMapsToNative",
                FtcActuators.PositionToleranceStep.class, double.class, double.class);
        assertReturns(FtcActuators.PositionReferenceStep.class, "assumeCurrentPositionIs",
                FtcActuators.PositionToleranceStep.class, double.class);
        assertReturns(FtcActuators.PositionReferenceStep.class, "needsReference",
                FtcActuators.PositionToleranceStep.class, String.class);
        assertReturns(FtcActuators.PositionToleranceStep.class, "positionTolerance",
                FtcActuators.PositionTargetStep.class, double.class);
        assertEquals(1, FtcActuators.PositionToleranceStep.class.getDeclaredMethods().length);

        assertReturns(FtcActuators.ServoBoundedPositionMappingStep.class, "nativeUnits",
                FtcActuators.PositionTargetStep.class);
        assertReturns(FtcActuators.ServoBoundedPositionMappingStep.class, "rangeMapsToNative",
                FtcActuators.PositionTargetStep.class, double.class, double.class);

        assertReturns(FtcActuators.PlantTargetStep.class, "targetGuards",
                FtcActuators.PlantTargetGuardStep.class);
        assertReturns(FtcActuators.PlantTargetStep.class, "targetedBy",
                FtcActuators.PlantBuildStep.class, ScalarTarget.class);
        assertReturns(FtcActuators.PlantTargetStep.class, "targetedBy",
                FtcActuators.PlantBuildStep.class, PlantTargetResolver.class);
        assertEquals(3, FtcActuators.PlantTargetStep.class.getDeclaredMethods().length);

        assertReturns(FtcActuators.PositionTargetStep.class, "targetGuards",
                FtcActuators.PositionTargetGuardStep.class);
        assertReturns(FtcActuators.PositionTargetStep.class, "targetedBy",
                FtcActuators.PositionPlantBuildStep.class, ScalarTarget.class);
        assertReturns(FtcActuators.PositionTargetStep.class, "targetedBy",
                FtcActuators.PositionPlantBuildStep.class, PlantTargetResolver.class);
        assertEquals(3, FtcActuators.PositionTargetStep.class.getDeclaredMethods().length);

        for (Class<?> nested : FtcActuators.class.getDeclaredClasses()) {
            assertNotEquals("VelocityBuildStep", nested.getSimpleName());
            assertNotEquals("PositionBuildStep", nested.getSimpleName());
            assertNotEquals("ServoPositionBuildStep", nested.getSimpleName());
        }
    }

    @Test
    public void toleranceAnswerIsValidatedAndCannotBeReplacedThroughRetainedStage() {
        HardwareMap hardwareMap = new HardwareMap(null, null);
        FtcActuators.VelocityToleranceStep velocity = FtcActuators.plant(hardwareMap)
                .motor("flywheel", Direction.FORWARD)
                .velocity()
                .deviceManagedWithDefaults()
                .unbounded()
                .nativeUnits();

        assertIllegalArgument(() -> velocity.velocityTolerance(Double.NaN),
                "velocityTolerance", "finite", ">= 0");
        FtcActuators.PlantTargetStep velocityTarget = velocity.velocityTolerance(0.0);
        assertTrue(velocityTarget != null);
        assertIllegalState(() -> velocity.velocityTolerance(1.0),
                "velocityTolerance(...) has already been answered");

        FtcActuators.PositionToleranceStep position = FtcActuators.plant(hardwareMap)
                .motor("lift", Direction.FORWARD)
                .position()
                .deviceManagedWithDefaults()
                .linear()
                .unbounded()
                .nativeUnits()
                .alreadyReferenced();

        assertIllegalArgument(() -> position.positionTolerance(Double.POSITIVE_INFINITY),
                "positionTolerance", "finite", ">= 0");
        FtcActuators.PositionTargetStep positionTarget = position.positionTolerance(0.0);
        assertTrue(positionTarget != null);
        assertIllegalState(() -> position.positionTolerance(1.0),
                "positionTolerance(...) has already been answered");
    }

    @Test
    public void runtimeStageBypassFailsBeforeHardwareResolutionOrConfiguration() {
        HardwareMap hardwareMap = new HardwareMap(null, null);
        ScalarTarget velocityTarget = ScalarTarget.create(0.0);
        FtcActuators.VelocityToleranceStep velocity = FtcActuators.plant(hardwareMap)
                .motor("flywheel", Direction.FORWARD)
                .velocity()
                .deviceManagedWithDefaults()
                .unbounded()
                .nativeUnits();
        FtcActuators.PlantBuildStep velocityBuild =
                ((FtcActuators.PlantTargetStep) velocity).targetedBy(velocityTarget);

        assertIllegalState(velocityBuild::build,
                "requires velocityTolerance(...)", "plant velocity units");

        ScalarTarget positionTarget = ScalarTarget.create(0.0);
        FtcActuators.PositionToleranceStep position = FtcActuators.plant(hardwareMap)
                .motor("lift", Direction.FORWARD)
                .position()
                .deviceManagedWithDefaults()
                .linear()
                .unbounded()
                .nativeUnits()
                .alreadyReferenced();
        FtcActuators.PositionPlantBuildStep positionBuild =
                ((FtcActuators.PositionTargetStep) position).targetedBy(positionTarget);

        assertIllegalState(positionBuild::build,
                "requires positionTolerance(...)", "plant position units");
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
