package edu.ftcphoenix.fw.actuation;

import org.junit.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;

import edu.ftcphoenix.fw.core.control.ScalarRegulator;
import edu.ftcphoenix.fw.core.hal.PositionOutput;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.hal.VelocityOutput;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.source.ScalarTarget;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies explicit plant-unit tolerance choices at the hardware-neutral mapped boundary. */
public final class MappedPlantToleranceTest {

    private static final double EPSILON = 1.0e-12;

    @Test
    public void publicFactoriesAndStageTransitionsExposeOnlyTheOrderedApi() throws Exception {
        assertSame(MappedPositionPlant.CommandedConfigurationStep.class,
                MappedPositionPlant.class.getDeclaredMethod(
                        "commanded", PositionOutput.class).getReturnType());
        assertSame(MappedPositionPlant.FeedbackConfigurationStep.class,
                MappedPositionPlant.class.getDeclaredMethod(
                        "positionOutput", PositionOutput.class, ScalarSource.class).getReturnType());
        assertSame(MappedPositionPlant.FeedbackConfigurationStep.class,
                MappedPositionPlant.class.getDeclaredMethod(
                        "regulated", PowerOutput.class, ScalarSource.class,
                        ScalarRegulator.class).getReturnType());
        assertSame(MappedVelocityPlant.FeedbackConfigurationStep.class,
                MappedVelocityPlant.class.getDeclaredMethod(
                        "velocityOutput", VelocityOutput.class, ScalarSource.class).getReturnType());
        assertSame(MappedVelocityPlant.FeedbackConfigurationStep.class,
                MappedVelocityPlant.class.getDeclaredMethod(
                        "regulated", PowerOutput.class, ScalarSource.class,
                        ScalarRegulator.class).getReturnType());

        assertTrue(MappedPlantTargetStep.class.isAssignableFrom(
                MappedPositionPlant.CommandedConfigurationStep.class));
        assertSame(MappedPlantTargetStep.class,
                MappedPositionPlant.FeedbackConfigurationStep.class.getDeclaredMethod(
                        "positionTolerance", double.class).getReturnType());
        assertSame(MappedPlantTargetStep.class,
                MappedVelocityPlant.FeedbackConfigurationStep.class.getDeclaredMethod(
                        "velocityTolerance", double.class).getReturnType());
        assertSame(MappedPlantTargetStep.class,
                MappedPlantTargetStep.class.getDeclaredMethod(
                        "targetGuards", PlantTargetGuards.class).getReturnType());
        assertSame(MappedPlantBuildStep.class,
                MappedPlantTargetStep.class.getDeclaredMethod(
                        "targetedBy", ScalarTarget.class).getReturnType());
        assertSame(MappedPlantBuildStep.class,
                MappedPlantTargetStep.class.getDeclaredMethod(
                        "targetedBy", PlantTargetSource.class).getReturnType());
        assertEquals(3, MappedPlantTargetStep.class.getDeclaredMethods().length);
        assertSame(Plant.class,
                MappedPlantBuildStep.class.getDeclaredMethod("build").getReturnType());

        assertNoPublicMethodNamed(
                MappedPositionPlant.CommandedConfigurationStep.class, "positionTolerance");
        assertNoPublicMethodNamed(
                MappedPositionPlant.CommandedConfigurationStep.class, "build");
        assertNoPublicMethodNamed(
                MappedPositionPlant.FeedbackConfigurationStep.class, "targetedBy");
        assertNoPublicMethodNamed(
                MappedPositionPlant.FeedbackConfigurationStep.class, "build");
        assertNoPublicMethodNamed(
                MappedVelocityPlant.FeedbackConfigurationStep.class, "targetedBy");
        assertNoPublicMethodNamed(
                MappedVelocityPlant.FeedbackConfigurationStep.class, "build");
        assertNoPublicMethodNamed(MappedPlantTargetStep.class, "build");

        assertPrivateConcreteBuilder(MappedPositionPlant.class);
        assertPrivateConcreteBuilder(MappedVelocityPlant.class);
    }

    @Test
    public void velocityOutputRequiresExplicitPlantUnitTolerance() {
        MappedVelocityPlant.FeedbackConfigurationStep configuration =
                MappedVelocityPlant.velocityOutput(
                        new RecordingVelocityOutput(), clock -> 0.0);
        assertIllegalStateContains(() -> bypassTarget(configuration)
                        .targetedBy(ScalarTarget.create(0.0))
                        .build(),
                "velocityTolerance(...)", "plant velocity units");
    }

    @Test
    public void regulatedVelocityRequiresExplicitPlantUnitTolerance() {
        MappedVelocityPlant.FeedbackConfigurationStep configuration =
                MappedVelocityPlant.regulated(
                        new RecordingPowerOutput(), clock -> 0.0,
                        (setpoint, measurement, clock) -> 0.0);
        assertIllegalStateContains(() -> bypassTarget(configuration)
                        .targetedBy(ScalarTarget.create(0.0))
                        .build(),
                "velocityTolerance(...)", "plant velocity units");
    }

    @Test
    public void positionOutputRequiresExplicitPlantUnitTolerance() {
        MappedPositionPlant.FeedbackConfigurationStep configuration =
                MappedPositionPlant.positionOutput(
                        new RecordingPositionOutput(), clock -> 0.0);
        assertIllegalStateContains(() -> bypassTarget(configuration)
                        .targetedBy(ScalarTarget.create(0.0))
                        .build(),
                "positionTolerance(...)", "plant position units");
    }

    @Test
    public void regulatedPositionRequiresExplicitPlantUnitTolerance() {
        MappedPositionPlant.FeedbackConfigurationStep configuration =
                MappedPositionPlant.regulated(
                        new RecordingPowerOutput(), clock -> 0.0,
                        (setpoint, measurement, clock) -> 0.0);
        assertIllegalStateContains(() -> bypassTarget(configuration)
                        .targetedBy(ScalarTarget.create(0.0))
                        .build(),
                "positionTolerance(...)", "plant position units");
    }

    @Test
    public void commandedPositionBuildsWithoutAndRejectsTolerance() {
        RecordingPositionOutput output = new RecordingPositionOutput();
        MappedPositionPlant plant = MappedPositionPlant.commanded(output)
                .targetedBy(ScalarTarget.create(0.4))
                .build();

        plant.update(new ManualLoopClock().clock());

        assertFalse(plant.hasFeedback());
        assertEquals(0.4, output.commandedPosition, EPSILON);
        MappedPositionPlant.CommandedConfigurationStep commanded =
                MappedPositionPlant.commanded(new RecordingPositionOutput());
        MappedPositionPlant.FeedbackConfigurationStep invalidFeedbackCast =
                (MappedPositionPlant.FeedbackConfigurationStep) (Object) commanded;
        assertIllegalStateContains(() -> invalidFeedbackCast.positionTolerance(0.1),
                "has no feedback", "does not use positionTolerance(...)");
    }

    @Test
    public void retainedConfigurationRejectsRepeatedToleranceAnswers() {
        MappedPositionPlant.FeedbackConfigurationStep position =
                MappedPositionPlant.positionOutput(new RecordingPositionOutput(), clock -> 0.0);
        position.positionTolerance(0.1);
        assertIllegalStateContains(() -> position.positionTolerance(0.2),
                "positionTolerance(...)", "already been answered");

        MappedVelocityPlant.FeedbackConfigurationStep velocity =
                MappedVelocityPlant.velocityOutput(new RecordingVelocityOutput(), clock -> 0.0);
        velocity.velocityTolerance(10.0);
        assertIllegalStateContains(() -> velocity.velocityTolerance(20.0),
                "velocityTolerance(...)", "already been answered");
    }

    @Test
    public void retainedTargetStageCannotBuildBeforeTargetSelection() {
        MappedPlantTargetStep<MappedPositionPlant> positionTarget =
                MappedPositionPlant.positionOutput(new RecordingPositionOutput(), clock -> 0.0)
                        .positionTolerance(0.1);
        assertIllegalStateContains(() -> MappedPlantToleranceTest
                        .<MappedPositionPlant>bypassBuild(positionTarget).build(),
                "targetedBy(...)");

        MappedPlantTargetStep<MappedVelocityPlant> velocityTarget =
                MappedVelocityPlant.velocityOutput(new RecordingVelocityOutput(), clock -> 0.0)
                        .velocityTolerance(10.0);
        assertIllegalStateContains(() -> MappedPlantToleranceTest
                        .<MappedVelocityPlant>bypassBuild(velocityTarget).build(),
                "targetedBy(...)");
    }

    @Test
    public void feedbackToleranceValidationRejectsInvalidValuesAndAcceptsZero() {
        double[] invalid = {-0.1, Double.NaN, Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY};
        for (double value : invalid) {
            assertIllegalArgumentContains(() -> MappedPositionPlant.positionOutput(
                            new RecordingPositionOutput(), clock -> 0.0)
                            .positionTolerance(value),
                    "positionTolerance", "finite and >= 0");
            assertIllegalArgumentContains(() -> MappedVelocityPlant.velocityOutput(
                            new RecordingVelocityOutput(), clock -> 0.0)
                            .velocityTolerance(value),
                    "velocityTolerance", "finite and >= 0");
        }

        MappedPositionPlant position = MappedPositionPlant.positionOutput(
                        new RecordingPositionOutput(), clock -> 0.0)
                .positionTolerance(0.0)
                .targetedBy(ScalarTarget.create(0.0))
                .build();
        MappedVelocityPlant velocity = MappedVelocityPlant.velocityOutput(
                        new RecordingVelocityOutput(), clock -> 0.0)
                .velocityTolerance(0.0)
                .targetedBy(ScalarTarget.create(0.0))
                .build();

        assertTrue(position.hasFeedback());
        assertTrue(velocity.hasFeedback());
    }

    @Test
    public void mappedPositionUsesToleranceInPlantUnitsAtExactBoundary() {
        double[] nativeMeasurement = {21.0};
        MappedPositionPlant plant = MappedPositionPlant.positionOutput(
                        new RecordingPositionOutput(), clock -> nativeMeasurement[0])
                .nativePerPlantUnit(4.0)
                .positionTolerance(0.25)
                .targetedBy(ScalarTarget.create(5.0))
                .build();
        ManualLoopClock clock = new ManualLoopClock();

        plant.update(clock.clock());

        assertEquals(5.25, plant.getMeasurement(), EPSILON);
        assertTrue(plant.atTarget());
        assertTrue(plant.atTarget(5.0));

        nativeMeasurement[0] = 21.0004;
        plant.update(clock.nextCycle(0.02));

        assertFalse(plant.atTarget());
        assertFalse(plant.atTarget(5.0));
    }

    @Test
    public void mappedVelocityUsesToleranceInPlantUnitsAtExactBoundary() {
        double[] nativeMeasurement = {2100.0};
        MappedVelocityPlant plant = MappedVelocityPlant.velocityOutput(
                        new RecordingVelocityOutput(), clock -> nativeMeasurement[0])
                .nativePerPlantUnit(2.0)
                .velocityTolerance(50.0)
                .targetedBy(ScalarTarget.create(1000.0))
                .build();
        ManualLoopClock clock = new ManualLoopClock();

        plant.update(clock.clock());

        assertEquals(1050.0, plant.getMeasurement(), EPSILON);
        assertTrue(plant.atTarget());
        assertTrue(plant.atTarget(1000.0));

        nativeMeasurement[0] = 2100.2;
        plant.update(clock.nextCycle(0.02));

        assertFalse(plant.atTarget());
        assertFalse(plant.atTarget(1000.0));
    }

    private static void assertIllegalStateContains(Runnable action, String... fragments) {
        try {
            action.run();
            fail("Expected IllegalStateException");
        } catch (IllegalStateException expected) {
            for (String fragment : fragments)
                assertTrue(expected.getMessage().contains(fragment));
        }
    }

    private static void assertIllegalArgumentContains(Runnable action, String... fragments) {
        try {
            action.run();
            fail("Expected IllegalArgumentException");
        } catch (IllegalArgumentException expected) {
            for (String fragment : fragments)
                assertTrue(expected.getMessage().contains(fragment));
        }
    }

    private static void assertPrivateConcreteBuilder(Class<?> plantType)
            throws ClassNotFoundException {
        Class<?> builder = Class.forName(plantType.getName() + "$Builder");
        int modifiers = builder.getModifiers();
        assertTrue(Modifier.isPrivate(modifiers));
        assertTrue(Modifier.isStatic(modifiers));
        assertTrue(Modifier.isFinal(modifiers));
        assertFalse(Modifier.isPublic(modifiers));
    }

    private static void assertNoPublicMethodNamed(Class<?> type, String methodName) {
        for (Method method : type.getMethods()) {
            if (method.getName().equals(methodName)) {
                fail(type.getSimpleName() + " must not expose " + methodName + "(...)");
            }
        }
    }

    @SuppressWarnings("unchecked")
    private static <P extends Plant> MappedPlantTargetStep<P> bypassTarget(Object configuration) {
        return (MappedPlantTargetStep<P>) configuration;
    }

    @SuppressWarnings("unchecked")
    private static <P extends Plant> MappedPlantBuildStep<P> bypassBuild(Object targetStep) {
        return (MappedPlantBuildStep<P>) targetStep;
    }

    private static final class RecordingPositionOutput implements PositionOutput {
        private double commandedPosition = Double.NaN;

        @Override
        public void setPosition(double position) {
            commandedPosition = position;
        }

        @Override
        public double getCommandedPosition() {
            return commandedPosition;
        }
    }

    private static final class RecordingVelocityOutput implements VelocityOutput {
        private double commandedVelocity = Double.NaN;

        @Override
        public void setVelocity(double velocity) {
            commandedVelocity = velocity;
        }

        @Override
        public double getCommandedVelocity() {
            return commandedVelocity;
        }
    }

    private static final class RecordingPowerOutput implements PowerOutput {
        private double commandedPower = Double.NaN;

        @Override
        public void setPower(double power) {
            commandedPower = power;
        }

        @Override
        public double getCommandedPower() {
            return commandedPower;
        }
    }
}
