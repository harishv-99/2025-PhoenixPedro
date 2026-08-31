package edu.ftcsushi.fw.actuation;

import org.junit.Test;

import edu.ftcsushi.fw.core.hal.PositionOutput;
import edu.ftcsushi.fw.core.hal.PowerOutput;
import edu.ftcsushi.fw.core.hal.VelocityOutput;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.source.ScalarSource;
import edu.ftcsushi.fw.core.source.ScalarTarget;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.testing.ManualLoopClock;

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

        PlantTargetResolver exact = PlantTargets.exact(upcast);

        assertSame(command, PlantTargets.commandTargetOf(exact));
        assertEquals(0, command.reads);
    }

    @Test
    public void ordinaryPlantCommandTargetIsStableAndRepeatedAccessDoesNotSampleIt() {
        CountingTarget command = new CountingTarget(0.25);
        Plant plant = Plants.fromOutputs()
                .power(new RecordingPowerOutput())
                .targetFromResolver(PlantTargets.exact(command))
                .build();

        assertTrue(plant.hasCommandTarget());
        assertSame(command, plant.commandTarget());
        assertSame(command, plant.commandTarget());
        assertEquals(0, command.reads);

        plant.commandTarget().set(0.75);

        assertSame(command, plant.commandTarget());
        assertEquals(0, command.reads);

        ManualLoopClock time = new ManualLoopClock();
        plant.update(time.clock());
        int readsAfterUpdate = command.reads;
        assertSame(command, plant.commandTarget());
        assertEquals(readsAfterUpdate, command.reads);

        plant.stop();
        assertSame(command, plant.commandTarget());
        assertEquals(readsAfterUpdate, command.reads);
    }

    @Test
    public void overlayPropagatesOnlyItsStableBaseCommandIncludingNestedBases() {
        ScalarTarget command = ScalarTarget.create(0.0);
        ScalarTarget innerLayer = ScalarTarget.create(0.4);
        ScalarTarget outerLayer = ScalarTarget.create(0.8);

        PlantTargetResolver inner = PlantTargets.overlay(command)
                .add("inner", BooleanSource.constant(true), innerLayer)
                .build();
        PlantTargetResolver outer = PlantTargets.overlay(inner)
                .add("outer", BooleanSource.constant(true), outerLayer)
                .build();

        assertSame(command, PlantTargets.commandTargetOf(inner));
        assertSame(command, PlantTargets.commandTargetOf(outer));
    }

    @Test
    public void mutableLayersNeverCreateOrRedirectCommandOwnership() {
        ScalarTarget firstLayer = ScalarTarget.create(0.3);
        ScalarTarget secondLayer = ScalarTarget.create(0.7);
        PlantTargetResolver readOnlyBase = PlantTargets.exact(0.0);

        PlantTargetResolver overlay = PlantTargets.overlay(readOnlyBase)
                .add("first", BooleanSource.constant(true), firstLayer)
                .add("second", BooleanSource.constant(false), secondLayer)
                .build();

        assertNull(PlantTargets.commandTargetOf(overlay));
    }

    @Test
    public void constantsHoldsPlannersWrappedAndOpaqueSourcesRemainReadOnly() {
        ScalarTarget command = ScalarTarget.create(1.0);
        ScalarSource wrappedCommand = command.scaled(1.0);
        PlantTargetResolver exactCommand = PlantTargets.exact(command);
        PlantTargetResolver opaqueWrapper = (context, clock) -> exactCommand.resolve(context, clock);
        PlantTargetResolver planner = PlantTargets.plan(PlantTargetRequest.exact("preset", 2.0))
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
    public void scalarTaskWritesGraphBaseWhileActiveOverlayStillOwnsRequestedTarget() {
        ScalarTarget command = ScalarTarget.create(0.0);
        final boolean[] overrideEnabled = {false};
        PlantTargetResolver finalTarget = PlantTargets.overlay(command)
                .add("override", clock -> overrideEnabled[0], 0.8)
                .build();
        RecordingPowerOutput output = new RecordingPowerOutput();
        Plant plant = Plants.fromOutputs()
                .power(output)
                .targetFromResolver(finalTarget)
                .build();
        ManualLoopClock time = new ManualLoopClock();

        assertTrue(plant.hasCommandTarget());
        assertSame(command, plant.commandTarget());

        Task firstWrite = ScalarTasks.set(command, 0.5).build();
        firstWrite.start(time.clock());
        plant.update(time.clock());

        assertEquals(0.5, command.get(), EPSILON);
        assertEquals(0.5, plant.getRequestedTarget(), EPSILON);
        assertEquals(0.5, output.commanded, EPSILON);

        overrideEnabled[0] = true;
        Task maskedWrite = ScalarTasks.set(command, 0.2).build();
        maskedWrite.start(time.nextCycle(0.02));
        plant.update(time.clock());

        assertEquals(0.2, command.get(), EPSILON);
        assertEquals(0.8, plant.getRequestedTarget(), EPSILON);
        assertEquals(0.8, output.commanded, EPSILON);
    }

    @Test
    public void positionBranchDerivesOverlayBaseAndRejectsRetargetingWithoutChangingIt() {
        ScalarTarget command = ScalarTarget.create(0.0);
        PlantTargetResolver overlay = PlantTargets.overlay(command)
                .add("override", BooleanSource.constant(false), 0.75)
                .build();
        PositionPlant commandBacked = Plants.fromOutputs()
                .commandedPosition(new RecordingPositionOutput())
                .nonPeriodic()
                .unbounded()
                .nativeUnits()
                .targetFromResolver(overlay)
                .build();
        ScalarTarget retainedCommand = ScalarTarget.create(0.5);
        Plants.TargetStep<PositionPlant> retainedTarget = Plants.fromOutputs()
                .commandedPosition(new RecordingPositionOutput())
                .nonPeriodic()
                .unbounded()
                .nativeUnits();

        Plants.BuildStep<PositionPlant> retainedBuild =
                retainedTarget.targetFromResolver(PlantTargets.exact(retainedCommand));
        IllegalStateException repeated = expectRepeatedTargetAnswer(
                () -> retainedTarget.targetFromResolver(PlantTargets.exact(0.25)));
        PositionPlant preserved = retainedBuild.build();

        assertTrue(commandBacked.hasCommandTarget());
        assertSame(command, commandBacked.commandTarget());
        assertTrue(repeated.getMessage().contains("after its target has been selected"));
        assertTrue(preserved.hasCommandTarget());
        assertSame(retainedCommand, preserved.commandTarget());
    }

    @Test
    public void velocityBranchRecognizesExplicitlyLiftedUpcastAndRejectsRetargeting() {
        ScalarTarget command = ScalarTarget.create(0.0);
        ScalarSource upcast = command;
        Plant commandBacked = Plants.fromOutputs()
                .deviceManagedVelocity(new RecordingVelocityOutput(), clock -> 0.0)
                .unbounded()
                .nativeUnits()
                .velocityTolerance(0.0)
                .targetFromResolver(PlantTargets.exact(upcast))
                .build();
        ScalarTarget retainedCommand = ScalarTarget.create(0.5);
        Plants.TargetStep<Plant> retainedTarget = Plants.fromOutputs()
                .deviceManagedVelocity(new RecordingVelocityOutput(), clock -> 0.0)
                .unbounded()
                .nativeUnits()
                .velocityTolerance(0.0);

        Plants.BuildStep<Plant> retainedBuild =
                retainedTarget.targetFromResolver(PlantTargets.exact(retainedCommand));
        IllegalStateException repeated = expectRepeatedTargetAnswer(
                () -> retainedTarget.targetFromResolver(PlantTargets.holdLastTarget(0.0)));
        Plant preserved = retainedBuild.build();

        assertTrue(commandBacked.hasCommandTarget());
        assertSame(command, commandBacked.commandTarget());
        assertTrue(repeated.getMessage().contains("after its target has been selected"));
        assertTrue(preserved.hasCommandTarget());
        assertSame(retainedCommand, preserved.commandTarget());
    }

    @Test
    public void readOnlyGraphExposesNoCommandTarget() {
        Plant plant = Plants.fromOutputs()
                .power(new RecordingPowerOutput())
                .targetFromResolver(PlantTargets.exact(0.0))
                .build();

        assertFalse(plant.hasCommandTarget());
        try {
            plant.commandTarget();
            fail("Expected read-only Plant command access to fail");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("no command target"));
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

    private static IllegalStateException expectRepeatedTargetAnswer(Runnable action) {
        try {
            action.run();
            fail("Expected a repeated targetFromResolver(...) answer to fail");
            return null;
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("targetFromResolver(...)"));
            assertTrue(expected.getMessage().contains("after its target has been selected"));
            return expected;
        }
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
