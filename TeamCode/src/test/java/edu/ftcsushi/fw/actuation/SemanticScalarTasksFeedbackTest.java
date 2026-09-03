package edu.ftcsushi.fw.actuation;

import org.junit.Test;

import edu.ftcsushi.fw.core.hal.PositionOutput;
import edu.ftcsushi.fw.core.hal.PowerOutput;
import edu.ftcsushi.fw.core.hal.VelocityOutput;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies feedback completion against exact semantic request and Plant provenance. */
public final class SemanticScalarTasksFeedbackTest {

    private static final double EPSILON = 1e-9;

    private enum Mode {
        IDLE,
        ACTIVE,
        OVERRIDE
    }

    @Test
    public void exactFeedbackMovePublishesOnceAndSucceedsAtItsOwnRequest() {
        SemanticScalarCommand<Mode> command = command();
        double[] measurement = {0.75};
        PositionPlant plant = nonPeriodicPlant(
                PlantTargets.exact(command), measurement, -1.0, 1.0);
        SemanticScalarCommand<Mode> different = command();
        SemanticScalarCommand.Request<Mode> initial = command.request();
        Task move = SemanticScalarTasks.set(command, Mode.ACTIVE)
                .untilReachedBy(plant)
                .leaveRequestOnCancel()
                .build();
        ManualLoopClock time = new ManualLoopClock();

        assertTrue(plant.carriesSemanticCommand(command));
        assertFalse(plant.carriesSemanticCommand(different));
        assertSame(initial, command.request());

        move.start(time.clock());
        SemanticScalarCommand.Request<Mode> started = command.request();
        plant.update(time.clock());
        move.update(time.clock());

        assertTrue(move.isComplete());
        assertEquals(TaskOutcome.SUCCESS, move.getOutcome());
        assertSame("Feedback updates must not republish the request", started,
                command.request());
        assertEquals(Mode.ACTIVE, started.semantic());
        assertEquals(0.75, plant.getRequestedTarget(), EPSILON);

        move.cancel();
        assertSame(started, command.request());
    }

    @Test
    public void sameValuedSupersessionCannotCompleteOrBeReclaimedByFeedbackMove() {
        SemanticScalarCommand<Mode> command = command();
        double[] measurement = {0.75};
        PositionPlant plant = nonPeriodicPlant(
                PlantTargets.exact(command), measurement, -1.0, 1.0);
        Task move = SemanticScalarTasks.set(command, Mode.ACTIVE)
                .untilReachedBy(plant)
                .leaveRequestOnCancel()
                .timeout(0.20)
                .build();
        ManualLoopClock time = new ManualLoopClock();

        move.start(time.clock());
        SemanticScalarCommand.Request<Mode> started = command.request();
        plant.update(time.clock());
        assertTrue(plant.getTargetResolution()
                .satisfiesSemanticCommand(command, started));

        SemanticScalarCommand.Request<Mode> superseding = command.set(Mode.ACTIVE);
        move.update(time.clock());
        assertFalse(move.isComplete());
        assertSame(superseding, command.request());

        plant.update(time.nextCycle(0.02));
        assertTrue(plant.getTargetResolution()
                .satisfiesSemanticCommand(command, superseding));
        move.update(time.clock());

        assertFalse("Arrival for the newer equal-valued request is not this Task's arrival",
                move.isComplete());
        assertSame("Feedback Tasks yield permanently to superseding requests",
                superseding, command.request());

        plant.update(time.nextCycle(0.19));
        move.update(time.clock());
        assertEquals(TaskOutcome.TIMEOUT, move.getOutcome());
        assertSame(superseding, command.request());
    }

    @Test
    public void stableSuccessWinsAnExactTieWithTimeoutAndLeavesTheRequest() {
        SemanticScalarCommand<Mode> command = command();
        double[] measurement = {0.75};
        PositionPlant plant = nonPeriodicPlant(
                PlantTargets.exact(command), measurement, -1.0, 1.0);
        Task move = SemanticScalarTasks.set(command, Mode.ACTIVE)
                .untilReachedBy(plant)
                .leaveRequestOnCancel()
                .stableFor(0.10)
                .timeout(0.10)
                .build();
        ManualLoopClock time = new ManualLoopClock();

        move.start(time.clock());
        SemanticScalarCommand.Request<Mode> started = command.request();
        plant.update(time.clock());
        move.update(time.clock());

        plant.update(time.nextCycle(0.11));
        move.update(time.clock());

        assertTrue(move.isComplete());
        assertEquals(TaskOutcome.SUCCESS, move.getOutcome());
        assertSame(started, command.request());
    }

    @Test
    public void losingSemanticSelectionResetsTheStableWindow() {
        SemanticScalarCommand<Mode> command = command();
        boolean[] override = {false};
        double[] measurement = {0.75};
        PlantTargetResolver resolver = PlantTargets.overlay(command)
                .add("sameValueOverride", clock -> override[0], 0.75)
                .build();
        PositionPlant plant = nonPeriodicPlant(resolver, measurement, -1.0, 1.0);
        Task move = SemanticScalarTasks.set(command, Mode.ACTIVE)
                .untilReachedBy(plant)
                .leaveRequestOnCancel()
                .stableFor(0.10)
                .build();
        ManualLoopClock time = new ManualLoopClock();

        assertTrue(plant.carriesSemanticCommand(command));
        move.start(time.clock());
        plant.update(time.clock());
        move.update(time.clock());

        plant.update(time.nextCycle(0.06));
        move.update(time.clock());
        override[0] = true;
        plant.update(time.nextCycle(0.02));
        assertFalse(plant.getTargetResolution()
                .satisfiesSemanticCommand(command, command.request()));
        move.update(time.clock());

        override[0] = false;
        plant.update(time.nextCycle(0.06));
        move.update(time.clock());
        plant.update(time.nextCycle(0.06));
        move.update(time.clock());
        assertFalse("Stable time before the override must not count", move.isComplete());

        plant.update(time.nextCycle(0.05));
        move.update(time.clock());
        assertEquals(TaskOutcome.SUCCESS, move.getOutcome());
    }

    @Test
    public void timeoutStartsWhenTheFeedbackTaskStartsAndLeavesTheLatestRequest() {
        SemanticScalarCommand<Mode> command = command();
        double[] measurement = {0.0};
        PositionPlant plant = nonPeriodicPlant(
                PlantTargets.exact(command), measurement, -1.0, 1.0);
        Task move = SemanticScalarTasks.set(command, Mode.ACTIVE)
                .untilReachedBy(plant)
                .leaveRequestOnCancel()
                .timeout(0.10)
                .build();
        ManualLoopClock time = new ManualLoopClock();

        time.nextCycle(1.0);
        move.start(time.clock());
        SemanticScalarCommand.Request<Mode> started = command.request();
        plant.update(time.clock());
        move.update(time.clock());
        assertFalse(move.isComplete());

        plant.update(time.nextCycle(0.09));
        move.update(time.clock());
        assertFalse(move.isComplete());

        plant.update(time.nextCycle(0.02));
        move.update(time.clock());
        assertEquals(TaskOutcome.TIMEOUT, move.getOutcome());
        assertSame(started, command.request());
    }

    @Test
    public void cancelToPublishesOnceOnlyForActiveCancellation() {
        SemanticScalarCommand<Mode> command = command();
        double[] measurement = {0.0};
        PositionPlant plant = nonPeriodicPlant(
                PlantTargets.exact(command), measurement, -1.0, 1.0);
        Task move = SemanticScalarTasks.set(command, Mode.ACTIVE)
                .untilReachedBy(plant)
                .cancelTo(Mode.IDLE)
                .build();
        ManualLoopClock time = new ManualLoopClock();
        SemanticScalarCommand.Request<Mode> initial = command.request();

        move.cancel();
        assertFalse(move.isComplete());
        assertSame(initial, command.request());
        assertLifecycleFailure(() -> move.update(time.clock()), "before start");

        move.start(time.clock());
        SemanticScalarCommand.Request<Mode> active = command.request();
        move.cancel();
        SemanticScalarCommand.Request<Mode> cancelled = command.request();

        assertEquals(TaskOutcome.CANCELLED, move.getOutcome());
        assertEquals(Mode.IDLE, cancelled.semantic());
        assertNotSame(active, cancelled);

        move.cancel();
        move.update(time.clock());
        assertSame(cancelled, command.request());
        assertLifecycleFailure(() -> move.start(time.clock()), "single-use");
        assertSame(cancelled, command.request());
    }

    @Test
    public void leaveRequestOnCancelPreservesASupersedingRequest() {
        SemanticScalarCommand<Mode> command = command();
        double[] measurement = {0.0};
        PositionPlant plant = nonPeriodicPlant(
                PlantTargets.exact(command), measurement, -1.0, 1.0);
        Task move = SemanticScalarTasks.set(command, Mode.ACTIVE)
                .untilReachedBy(plant)
                .leaveRequestOnCancel()
                .build();
        ManualLoopClock time = new ManualLoopClock();

        move.start(time.clock());
        SemanticScalarCommand.Request<Mode> override = command.set(Mode.OVERRIDE);
        move.cancel();
        move.cancel();

        assertEquals(TaskOutcome.CANCELLED, move.getOutcome());
        assertSame(override, command.request());
    }

    @Test
    public void equivalentPositionMoveCompletesAtTheSelectedPhysicalRepresentative() {
        SemanticScalarCommand<Mode> command = SemanticScalarCommand.create(
                Mode.IDLE, mode -> mode == Mode.ACTIVE ? 20.0 : 0.0);
        double[] measurement = {350.0};
        PlantTargetResolver resolver = PlantTargets.equivalentPositionsOf(command)
                .nearestToMeasurement()
                .whenUnavailable().reportUnavailable();
        PositionPlant plant = periodicPlant(resolver, measurement, 360.0, 0.0, 720.0);
        Task move = SemanticScalarTasks.set(command, Mode.ACTIVE)
                .untilReachedBy(plant)
                .leaveRequestOnCancel()
                .build();
        ManualLoopClock time = new ManualLoopClock();

        assertTrue(plant.carriesSemanticCommand(command));
        move.start(time.clock());
        plant.update(time.clock());
        move.update(time.clock());

        assertEquals(380.0, plant.getRequestedTarget(), EPSILON);
        assertFalse(move.isComplete());

        measurement[0] = 380.0;
        plant.update(time.nextCycle(0.02));
        move.update(time.clock());

        assertEquals(TaskOutcome.SUCCESS, move.getOutcome());
    }

    @Test
    public void actualOpenLoopAndOwnerlessLayerGraphsAreRejected() {
        SemanticScalarCommand<Mode> command = command();
        Plant openLoop = Plants.fromOutputs()
                .power(new RecordingPowerOutput())
                .targetFromResolver(PlantTargets.exact(command))
                .build();
        Plant ownerlessLayer = Plants.fromOutputs()
                .deviceManagedPosition(new RecordingPositionOutput(), clock -> 0.75)
                .nonPeriodic()
                .bounded(-1.0, 1.0)
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(0.01)
                .targetFromResolver(PlantTargets.overlay(0.0)
                        .add("semanticLayer", clock -> true, PlantTargets.exact(command))
                        .build())
                .build();
        SemanticScalarTasks.SetReadyStep<Mode> set =
                SemanticScalarTasks.set(command, Mode.ACTIVE);

        assertTrue(openLoop.carriesSemanticCommand(command));
        assertFalse(openLoop.hasFeedback());
        assertFailure(() -> set.untilReachedBy(openLoop),
                IllegalStateException.class, "feedback");

        assertFalse(ownerlessLayer.carriesSemanticCommand(command));
        assertFailure(() -> set.untilReachedBy(ownerlessLayer),
                IllegalArgumentException.class, "carries this exact");
    }

    @Test
    public void mappedPositionAndVelocityPlantsPreserveTheSemanticCommandRelationship() {
        SemanticScalarCommand<Mode> command = command();
        SemanticScalarCommand<Mode> different = command();
        PositionPlant mappedPosition = Plants.fromOutputs()
                .deviceManagedPosition(new RecordingPositionOutput(), clock -> 0.0)
                .nonPeriodic()
                .bounded(-1.0, 1.0)
                .scaleToNative(2.0)
                .alreadyReferenced()
                .positionTolerance(0.01)
                .targetFromResolver(PlantTargets.exact(command))
                .build();
        Plant mappedVelocity = Plants.fromOutputs()
                .deviceManagedVelocity(new RecordingVelocityOutput(), clock -> 0.0)
                .bounded(-1.0, 1.0)
                .scaleToNative(2.0)
                .velocityTolerance(0.01)
                .targetFromResolver(PlantTargets.exact(command))
                .build();

        assertTrue(mappedPosition.carriesSemanticCommand(command));
        assertFalse(mappedPosition.carriesSemanticCommand(different));
        assertTrue(mappedVelocity.carriesSemanticCommand(command));
        assertFalse(mappedVelocity.carriesSemanticCommand(different));
    }

    private static SemanticScalarCommand<Mode> command() {
        return SemanticScalarCommand.create(Mode.IDLE, mode -> {
            switch (mode) {
                case ACTIVE:
                    return 0.75;
                case OVERRIDE:
                    return -0.5;
                default:
                    return 0.0;
            }
        });
    }

    private static PositionPlant nonPeriodicPlant(PlantTargetResolver resolver,
                                                   double[] measurement,
                                                   double min,
                                                   double max) {
        return Plants.fromOutputs()
                .deviceManagedPosition(
                        new RecordingPositionOutput(), clock -> measurement[0])
                .nonPeriodic()
                .bounded(min, max)
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(0.01)
                .targetFromResolver(resolver)
                .build();
    }

    private static PositionPlant periodicPlant(PlantTargetResolver resolver,
                                                double[] measurement,
                                                double period,
                                                double min,
                                                double max) {
        return Plants.fromOutputs()
                .deviceManagedPosition(
                        new RecordingPositionOutput(), clock -> measurement[0])
                .periodic(period)
                .bounded(min, max)
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(0.01)
                .targetFromResolver(resolver)
                .build();
    }

    private static void assertLifecycleFailure(Runnable action, String messagePart) {
        try {
            action.run();
            fail("expected lifecycle failure");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains(messagePart));
        }
    }

    private static void assertFailure(Runnable action,
                                      Class<? extends RuntimeException> expectedType,
                                      String messagePart) {
        try {
            action.run();
            fail("expected " + expectedType.getSimpleName());
        } catch (RuntimeException expected) {
            assertTrue("wrong exception: " + expected, expectedType.isInstance(expected));
            assertTrue(expected.getMessage().contains(messagePart));
        }
    }

    private static final class RecordingPositionOutput implements PositionOutput {
        private double command = Double.NaN;

        @Override
        public void setPosition(double position) {
            command = position;
        }

        @Override
        public double getCommandedPosition() {
            return command;
        }
    }

    private static final class RecordingPowerOutput implements PowerOutput {
        private double command = Double.NaN;

        @Override
        public void setPower(double power) {
            command = power;
        }

        @Override
        public double getCommandedPower() {
            return command;
        }
    }

    private static final class RecordingVelocityOutput implements VelocityOutput {
        private double command = Double.NaN;

        @Override
        public void setVelocity(double velocity) {
            command = velocity;
        }

        @Override
        public double getCommandedVelocity() {
            return command;
        }
    }
}
