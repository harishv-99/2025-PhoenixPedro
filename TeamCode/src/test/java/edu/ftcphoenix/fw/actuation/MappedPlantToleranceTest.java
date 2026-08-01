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

/** Verifies the sole hardware-neutral Plant grammar and its staged lifecycle. */
public final class MappedPlantToleranceTest {

    private static final double EPSILON = 1.0e-12;

    @Test
    public void fromOutputsIsTheOnlyPublicNeutralStartAndExposesSixBranches() throws Exception {
        Method fromOutputs = Plants.class.getDeclaredMethod("fromOutputs");
        assertTrue(Modifier.isPublic(fromOutputs.getModifiers()));
        assertTrue(Modifier.isStatic(fromOutputs.getModifiers()));
        assertSame(Plants.FromOutputsStep.class, fromOutputs.getReturnType());

        int publicStarts = 0;
        for (Method method : Plants.class.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers())) {
                publicStarts++;
                assertEquals("fromOutputs", method.getName());
            }
        }
        assertEquals(1, publicStarts);
        assertEquals(6, Plants.FromOutputsStep.class.getDeclaredMethods().length);
        assertSame(Plants.TargetStep.class, Plants.FromOutputsStep.class
                .getDeclaredMethod("power", PowerOutput.class).getReturnType());
        assertSame(Plants.PositionPeriodicityStep.class, Plants.FromOutputsStep.class
                .getDeclaredMethod("commandedPosition", PositionOutput.class).getReturnType());
        assertSame(Plants.DeviceManagedPositionStep.class, Plants.FromOutputsStep.class
                .getDeclaredMethod("deviceManagedPosition", PositionOutput.class, ScalarSource.class)
                .getReturnType());
        assertSame(Plants.VelocityBoundsStep.class, Plants.FromOutputsStep.class
                .getDeclaredMethod("deviceManagedVelocity", VelocityOutput.class, ScalarSource.class)
                .getReturnType());
        assertSame(Plants.PositionPeriodicityStep.class, Plants.FromOutputsStep.class
                .getDeclaredMethod("regulatedPosition", PowerOutput.class, ScalarSource.class,
                        ScalarRegulator.class).getReturnType());
        assertSame(Plants.VelocityBoundsStep.class, Plants.FromOutputsStep.class
                .getDeclaredMethod("regulatedVelocity", PowerOutput.class, ScalarSource.class,
                        ScalarRegulator.class).getReturnType());
    }

    @Test
    public void sharedTailHasOneCommandAnswerOneResolverAnswerAndInlineGuards() throws Exception {
        assertEquals(3, Plants.TargetStep.class.getDeclaredMethods().length);
        assertSame(Plants.TargetGuardStep.class, Plants.TargetStep.class
                .getDeclaredMethod("targetGuards").getReturnType());
        assertSame(Plants.BuildStep.class, Plants.TargetStep.class
                .getDeclaredMethod("targetFromNewCommand", double.class).getReturnType());
        assertSame(Plants.BuildStep.class, Plants.TargetStep.class
                .getDeclaredMethod("targetFromResolver", PlantTargetResolver.class).getReturnType());
        assertNoDeclaredMethod(Plants.TargetStep.class, "targetFromResolver", ScalarTarget.class);
        assertSame(Plant.class, Plants.BuildStep.class.getDeclaredMethod("build").getReturnType());

        assertEquals(7, Plants.TargetGuardStep.class.getDeclaredMethods().length);
        assertSame(Plants.TargetGuardStep.class, Plants.TargetGuardStep.class
                .getDeclaredMethod("holdLastTargetUnless", String.class,
                        edu.ftcphoenix.fw.core.source.BooleanSource.class).getReturnType());
        assertSame(Plants.TargetGuardStep.class, Plants.TargetGuardStep.class
                .getDeclaredMethod("holdLastTargetUnless", String.class, PlantTargetGate.class)
                .getReturnType());
        assertSame(Plants.TargetStep.class, Plants.TargetGuardStep.class
                .getDeclaredMethod("doneTargetGuards").getReturnType());

        assertFalse(Modifier.isPublic(MappedPositionPlant.class.getModifiers()));
        assertFalse(Modifier.isPublic(MappedVelocityPlant.class.getModifiers()));
        assertFalse(Modifier.isPublic(MappedPlantTargetStep.class.getModifiers()));
        assertFalse(Modifier.isPublic(MappedPlantBuildStep.class.getModifiers()));
        assertFalse(Modifier.isPublic(PlantTargetGuards.class.getModifiers()));
        assertFalse(Modifier.isPublic(MappedPositionPlant.class
                .getDeclaredMethod("commanded", PositionOutput.class).getModifiers()));
        assertFalse(Modifier.isPublic(MappedVelocityPlant.class
                .getDeclaredMethod("velocityOutput", VelocityOutput.class, ScalarSource.class)
                .getModifiers()));
    }

    @Test
    public void allSixNeutralBranchesBuildThroughTheSharedTail() {
        RecordingPowerOutput powerOutput = new RecordingPowerOutput();
        Plant power = Plants.fromOutputs()
                .power(powerOutput)
                .targetFromNewCommand(0.0)
                .build();

        RecordingPositionOutput commandedOutput = new RecordingPositionOutput();
        PositionPlant commanded = Plants.fromOutputs()
                .commandedPosition(commandedOutput)
                .periodic(2.0 * Math.PI)
                .bounded(-Math.PI, Math.PI)
                .rangeMapsToNative(1.0, 0.0)
                .targetFromNewCommand(0.0)
                .build();

        RecordingPositionOutput devicePositionOutput = new RecordingPositionOutput();
        RecordingPowerOutput searchOutput = new RecordingPowerOutput();
        PositionPlant devicePosition = Plants.fromOutputs()
                .deviceManagedPosition(devicePositionOutput, clock -> 0.0)
                .searchPowerOutput(searchOutput)
                .nonPeriodic()
                .unbounded()
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(0.0)
                .targetFromNewCommand(0.0)
                .build();

        RecordingVelocityOutput velocityOutput = new RecordingVelocityOutput();
        Plant deviceVelocity = Plants.fromOutputs()
                .deviceManagedVelocity(velocityOutput, clock -> 0.0)
                .bounded(-100.0, 100.0)
                .scaleToNative(-2.0)
                .velocityTolerance(0.0)
                .targetFromNewCommand(0.0)
                .build();

        RecordingPowerOutput regulatedPositionOutput = new RecordingPowerOutput();
        PositionPlant regulatedPosition = Plants.fromOutputs()
                .regulatedPosition(regulatedPositionOutput, clock -> 0.0,
                        (setpoint, measurement, clock) -> 0.0)
                .periodic(360.0)
                .bounded(0.0, 720.0)
                .nativeUnits()
                .needsReference("index required")
                .positionTolerance(1.0)
                .targetFromNewCommand(0.0)
                .build();

        RecordingPowerOutput regulatedVelocityOutput = new RecordingPowerOutput();
        Plant regulatedVelocity = Plants.fromOutputs()
                .regulatedVelocity(regulatedVelocityOutput, clock -> 0.0,
                        (setpoint, measurement, clock) -> 0.0)
                .unbounded()
                .nativeUnits()
                .velocityTolerance(0.0)
                .targetFromNewCommand(0.0)
                .build();

        assertFalse(power.hasFeedback());
        assertFalse(commanded.hasFeedback());
        assertTrue(devicePosition.hasFeedback());
        assertTrue(devicePosition.supportsCalibrationSearch());
        assertTrue(deviceVelocity.hasFeedback());
        assertTrue(regulatedPosition.hasFeedback());
        assertTrue(regulatedPosition.supportsCalibrationSearch());
        assertTrue(regulatedVelocity.hasFeedback());
        assertEquals(PositionPlant.Periodicity.PERIODIC, commanded.periodicity());
        assertEquals(PositionPlant.Periodicity.PERIODIC, regulatedPosition.periodicity());
    }

    @Test
    public void targetFromNewCommandCreatesOneStableCommandTarget() {
        RecordingPowerOutput output = new RecordingPowerOutput();
        Plant plant = Plants.fromOutputs()
                .power(output)
                .targetFromNewCommand(0.25)
                .build();

        ScalarTarget command = plant.commandTarget();
        assertSame(command, plant.commandTarget());
        assertEquals(0.25, command.get(), EPSILON);

        ManualLoopClock time = new ManualLoopClock();
        plant.update(time.clock());
        assertEquals(0.25, output.commandedPower, EPSILON);

        command.set(-0.5);
        plant.update(time.nextCycle(0.02));
        assertEquals(-0.5, output.commandedPower, EPSILON);
    }

    @Test
    public void namedCommandUsesExplicitExactResolverAndRetainsIdentity() {
        ScalarTarget command = ScalarTarget.create(0.4);
        Plant plant = Plants.fromOutputs()
                .power(new RecordingPowerOutput())
                .targetFromResolver(PlantTargets.exact(command))
                .build();

        assertTrue(plant.hasCommandTarget());
        assertSame(command, plant.commandTarget());
    }

    @Test
    public void invalidCommandAnswerDoesNotPoisonRecipe() {
        Plants.TargetStep<Plant> power = Plants.fromOutputs()
                .power(new RecordingPowerOutput());

        assertIllegalArgumentContains(() -> power.targetFromNewCommand(Double.NaN), "finite");
        assertIllegalArgumentContains(() -> power.targetFromNewCommand(1.01), "outside", "[-1.0, 1.0]");

        Plant built = power.targetFromNewCommand(0.5).build();
        assertEquals(0.5, built.commandTarget().get(), EPSILON);
    }

    @Test
    public void incompleteRetainedStageCanRecoverButTargetFreezeAndBuildAreSingleUse() {
        Plants.FromOutputsStep root = Plants.fromOutputs();
        Plants.VelocityBoundsStep velocityBounds = root.deviceManagedVelocity(
                new RecordingVelocityOutput(), clock -> 0.0);

        @SuppressWarnings("unchecked")
        Plants.TargetStep<Plant> bypass = (Plants.TargetStep<Plant>) (Object) velocityBounds;
        assertIllegalStateContains(() -> bypass.targetFromNewCommand(0.0), "requires bounded", "unbounded");

        Plants.TargetStep<Plant> target = velocityBounds
                .unbounded()
                .nativeUnits()
                .velocityTolerance(0.0);
        Plants.BuildStep<Plant> build = target.targetFromNewCommand(0.0);

        assertIllegalStateContains(target::targetGuards, "after its target has been selected");
        assertIllegalStateContains(
                () -> target.targetFromNewCommand(0.5),
                "targetFromNewCommand(...)", "after its target has been selected");
        assertIllegalStateContains(
                () -> target.targetFromResolver(PlantTargets.exact(0.0)),
                "after its target has been selected");
        assertIllegalStateContains(
                () -> root.power(new RecordingPowerOutput()),
                "already selected a control path");

        Plant plant = build.build();
        assertTrue(plant.hasCommandTarget());
        assertIllegalStateContains(build::build, "already attempted build");
    }

    @Test
    public void reversedEndpointAndNegativeScaleMappingsRemainSupported() {
        RecordingPositionOutput positionOutput = new RecordingPositionOutput();
        PositionPlant position = Plants.fromOutputs()
                .commandedPosition(positionOutput)
                .nonPeriodic()
                .bounded(-1.0, 1.0)
                .rangeMapsToNative(1.0, 0.0)
                .targetFromNewCommand(0.5)
                .build();
        position.update(new ManualLoopClock().clock());
        assertEquals(0.25, positionOutput.commandedPosition, EPSILON);

        RecordingVelocityOutput velocityOutput = new RecordingVelocityOutput();
        Plant velocity = Plants.fromOutputs()
                .deviceManagedVelocity(velocityOutput, clock -> -8.0)
                .unbounded()
                .scaleToNative(-2.0)
                .velocityTolerance(0.0)
                .targetFromNewCommand(4.0)
                .build();
        velocity.update(new ManualLoopClock().clock());
        assertEquals(-8.0, velocityOutput.commandedVelocity, EPSILON);
        assertEquals(4.0, velocity.getMeasurement(), EPSILON);
        assertTrue(velocity.atTarget());
    }

    private static void assertNoDeclaredMethod(Class<?> type, String name, Class<?>... parameterTypes) {
        try {
            type.getDeclaredMethod(name, parameterTypes);
            fail(type.getSimpleName() + " must not declare " + name + " with the removed signature");
        } catch (NoSuchMethodException expected) {
            // Expected removed surface.
        }
    }

    private static void assertIllegalStateContains(Runnable action, String... fragments) {
        try {
            action.run();
            fail("Expected IllegalStateException");
        } catch (IllegalStateException expected) {
            for (String fragment : fragments) {
                assertTrue(expected.getMessage(), expected.getMessage().contains(fragment));
            }
        }
    }

    private static void assertIllegalArgumentContains(Runnable action, String... fragments) {
        try {
            action.run();
            fail("Expected IllegalArgumentException");
        } catch (IllegalArgumentException expected) {
            for (String fragment : fragments) {
                assertTrue(expected.getMessage(), expected.getMessage().contains(fragment));
            }
        }
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
