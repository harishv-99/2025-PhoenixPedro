package edu.ftcphoenix.fw.actuation;

import org.junit.Test;

import edu.ftcphoenix.fw.core.hal.PositionOutput;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.hal.VelocityOutput;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.source.ScalarTarget;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies that Plant command capability is derived only from a target graph's stable base. */
public final class PlantCommandTargetTest {

    private static final double EPSILON = 1e-12;

    @Test
    public void exactRecognizesRuntimeScalarTargetBeforeMemoizingWithoutSamplingIt() {
        CountingTarget command = new CountingTarget(0.25);
        ScalarSource upcast = command;

        PlantTargetSource exact = PlantTargets.exact(upcast);

        assertSame(command, PlantTargets.commandTargetOf(exact));
        assertEquals(0, command.reads);
    }

    @Test
    public void overlayPropagatesOnlyItsStableBaseCommandIncludingNestedBases() {
        ScalarTarget command = ScalarTarget.held(0.0);
        ScalarTarget innerLayer = ScalarTarget.held(0.4);
        ScalarTarget outerLayer = ScalarTarget.held(0.8);

        PlantTargetSource inner = PlantTargets.overlay(command)
                .add("inner", BooleanSource.constant(true), innerLayer)
                .build();
        PlantTargetSource outer = PlantTargets.overlay(inner)
                .add("outer", BooleanSource.constant(true), outerLayer)
                .build();

        assertSame(command, PlantTargets.commandTargetOf(inner));
        assertSame(command, PlantTargets.commandTargetOf(outer));
    }

    @Test
    public void mutableLayersNeverCreateOrRedirectCommandOwnership() {
        ScalarTarget firstLayer = ScalarTarget.held(0.3);
        ScalarTarget secondLayer = ScalarTarget.held(0.7);
        PlantTargetSource readOnlyBase = PlantTargets.exact(0.0);

        PlantTargetSource overlay = PlantTargets.overlay(readOnlyBase)
                .add("first", BooleanSource.constant(true), firstLayer)
                .add("second", BooleanSource.constant(false), secondLayer)
                .build();

        assertNull(PlantTargets.commandTargetOf(overlay));
    }

    @Test
    public void constantsHoldsPlannersWrappedAndOpaqueSourcesRemainReadOnly() {
        ScalarTarget command = ScalarTarget.held(1.0);
        ScalarSource wrappedCommand = command.scaled(1.0);
        PlantTargetSource exactCommand = PlantTargets.exact(command);
        PlantTargetSource opaqueWrapper = (context, clock) -> exactCommand.resolve(context, clock);
        PlantTargetSource planner = PlantTargets.plan()
                .request(Source.constant(PlantTargetRequest.exact("preset", 2.0)))
                .nearestToMeasurement()
                .rejectUnreachable()
                .whenUnavailable().holdLastTarget(0.0);

        assertNull(PlantTargets.commandTargetOf(PlantTargets.exact(1.0)));
        assertNull(PlantTargets.commandTargetOf(PlantTargets.exact(wrappedCommand)));
        assertNull(PlantTargets.commandTargetOf(PlantTargets.holdLastTarget(0.0)));
        assertNull(PlantTargets.commandTargetOf(
                PlantTargets.holdMeasuredTargetOnEntry(0.0)));
        assertNull(PlantTargets.commandTargetOf(planner));
        assertNull(PlantTargets.commandTargetOf(opaqueWrapper));
    }

    @Test
    public void plantTaskWritesGraphBaseWhileActiveOverlayStillOwnsRequestedTarget() {
        ScalarTarget command = ScalarTarget.held(0.0);
        final boolean[] overrideEnabled = {false};
        PlantTargetSource finalTarget = PlantTargets.overlay(command)
                .add("override", clock -> overrideEnabled[0], 0.8)
                .build();
        RecordingPowerOutput output = new RecordingPowerOutput();
        Plant plant = Plants.power(output, finalTarget);
        ManualLoopClock time = new ManualLoopClock();

        assertTrue(plant.hasCommandTarget());
        assertSame(command, plant.commandTarget());

        Task firstWrite = PlantTasks.setTarget(plant, 0.5);
        firstWrite.start(time.clock());
        plant.update(time.clock());

        assertEquals(0.5, command.get(), EPSILON);
        assertEquals(0.5, plant.getRequestedTarget(), EPSILON);
        assertEquals(0.5, output.commanded, EPSILON);

        overrideEnabled[0] = true;
        Task maskedWrite = PlantTasks.setTarget(plant, 0.2);
        maskedWrite.start(time.nextCycle(0.02));
        plant.update(time.clock());

        assertEquals(0.2, command.get(), EPSILON);
        assertEquals(0.8, plant.getRequestedTarget(), EPSILON);
        assertEquals(0.8, output.commanded, EPSILON);
    }

    @Test
    public void mappedPositionDerivesOverlayBaseCommandAndRetargetingClearsCapability() {
        ScalarTarget command = ScalarTarget.held(0.0);
        PlantTargetSource overlay = PlantTargets.overlay(command)
                .add("override", BooleanSource.constant(false), 0.75)
                .build();
        MappedPositionPlant commandBacked = MappedPositionPlant.commanded(
                        new RecordingPositionOutput())
                .targetedBy(overlay)
                .build();
        ScalarTarget abandonedCommand = ScalarTarget.held(0.5);
        MappedPlantTargetStep<MappedPositionPlant> retainedTarget =
                MappedPositionPlant.commanded(
                new RecordingPositionOutput());

        MappedPlantBuildStep<MappedPositionPlant> retainedBuild =
                retainedTarget.targetedBy(abandonedCommand);
        retainedTarget.targetedBy(PlantTargets.exact(0.25));
        MappedPositionPlant retargetedReadOnly = retainedBuild.build();

        assertTrue(commandBacked.hasCommandTarget());
        assertSame(command, commandBacked.commandTarget());
        assertFalse(retargetedReadOnly.hasCommandTarget());
    }

    @Test
    public void mappedVelocityRecognizesUpcastCommandAndRetargetingClearsCapability() {
        ScalarTarget command = ScalarTarget.held(0.0);
        ScalarSource upcast = command;
        MappedVelocityPlant commandBacked = MappedVelocityPlant.velocityOutput(
                        new RecordingVelocityOutput(), clock -> 0.0)
                .velocityTolerance(0.0)
                .targetedBy(upcast)
                .build();
        ScalarTarget abandonedCommand = ScalarTarget.held(0.5);
        MappedPlantTargetStep<MappedVelocityPlant> retainedTarget =
                MappedVelocityPlant.velocityOutput(
                        new RecordingVelocityOutput(), clock -> 0.0)
                        .velocityTolerance(0.0);

        MappedPlantBuildStep<MappedVelocityPlant> retainedBuild =
                retainedTarget.targetedBy(abandonedCommand);
        retainedTarget.targetedBy(PlantTargets.holdLastTarget(0.0));
        MappedVelocityPlant retargetedReadOnly = retainedBuild.build();

        assertTrue(commandBacked.hasCommandTarget());
        assertSame(command, commandBacked.commandTarget());
        assertFalse(retargetedReadOnly.hasCommandTarget());
    }

    @Test
    public void plantTasksRejectReadOnlyGraphWithCommandSpecificGuidance() {
        Plant plant = Plants.power(new RecordingPowerOutput(), PlantTargets.exact(0.0));

        assertFalse(plant.hasCommandTarget());
        try {
            plant.commandTarget();
            fail("Expected read-only Plant command access to fail");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("no command target"));
        }

        try {
            PlantTasks.setTarget(plant, 0.5);
            fail("Expected PlantTasks to reject a read-only Plant");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("requires a plant with a command target"));
            assertTrue(expected.getMessage().contains("stable base is a ScalarTarget"));
        }
    }

    @Test
    public void requestedTargetErrorNamesMatchPlantSourceView() {
        Plant plant = new Plant() {
            @Override
            public void update(LoopClock clock) {
            }

            @Override
            public double getRequestedTarget() {
                return 7.0;
            }

            @Override
            public double getAppliedTarget() {
                return 6.0;
            }

            @Override
            public PlantTargetStatus getTargetStatus() {
                return PlantTargetStatus.ACCEPTED;
            }

            @Override
            public double getMeasurement() {
                return 2.0;
            }

            @Override
            public void stop() {
            }
        };
        ManualLoopClock time = new ManualLoopClock();

        assertEquals(5.0, plant.getRequestedTargetError(), EPSILON);
        assertEquals(5.0,
                PlantSources.requestedTargetError(plant).getAsDouble(time.clock()), EPSILON);
    }

    /** ScalarTarget probe proving graph inspection never samples command state. */
    private static final class CountingTarget implements ScalarTarget {
        private double value;
        private int reads;

        CountingTarget(double initialValue) {
            value = initialValue;
        }

        @Override
        public void set(double value) {
            this.value = value;
        }

        @Override
        public double get() {
            reads++;
            return value;
        }
    }

    private static final class RecordingPowerOutput implements PowerOutput {
        private double commanded = Double.NaN;

        @Override
        public void setPower(double power) {
            commanded = power;
        }

        @Override
        public double getCommandedPower() {
            return commanded;
        }
    }

    private static final class RecordingPositionOutput implements PositionOutput {
        private double commanded;

        @Override
        public void setPosition(double position) {
            commanded = position;
        }

        @Override
        public double getCommandedPosition() {
            return commanded;
        }
    }

    private static final class RecordingVelocityOutput implements VelocityOutput {
        private double commanded;

        @Override
        public void setVelocity(double velocity) {
            commanded = velocity;
        }

        @Override
        public double getCommandedVelocity() {
            return commanded;
        }
    }
}
