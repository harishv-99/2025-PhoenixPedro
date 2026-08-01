package edu.ftcphoenix.fw.actuation;

import org.junit.Test;

import java.util.function.UnaryOperator;

import edu.ftcphoenix.fw.core.hal.PositionOutput;
import edu.ftcphoenix.fw.core.source.ScalarTarget;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/** Verifies ScalarTasks feedback completion against physically selected equivalent positions. */
public final class PlantEquivalentPositionTaskTest {

    private static final UnaryOperator<Plants.TargetStep<PositionPlant>> NO_GUARDS = step -> step;

    private static final double EPSILON = 1e-9;

    @Test
    public void logicalMoveCompletesAtTheSelectedPhysicalEquivalent() {
        ScalarTarget command = ScalarTarget.create(0.0);
        final double[] measurement = {350.0};
        PositionPlant plant = periodicPlant(command, measurement, NO_GUARDS);
        ManualLoopClock time = new ManualLoopClock();
        Task move = moveTo20(plant);

        move.start(time.clock());
        plant.update(time.clock());
        move.update(time.clock());

        assertEquals(20.0, command.get(), EPSILON);
        assertEquals(380.0, plant.getRequestedTarget(), EPSILON);
        assertFalse(move.isComplete());

        measurement[0] = 380.0;
        plant.update(time.nextCycle(0.02));
        move.update(time.clock());

        assertTrue(move.isComplete());
        assertEquals(TaskOutcome.SUCCESS, move.getOutcome());
    }

    @Test
    public void sameValuedOverlayCannotMasqueradeAsTheCommandPath() {
        ScalarTarget command = ScalarTarget.create(0.0);
        final boolean[] override = {true};
        final double[] measurement = {380.0};
        PlantTargetResolver logical = PlantTargets.overlay(command)
                .add("sameValueOverride", clock -> override[0], 20.0)
                .build();
        PlantTargetResolver target = PlantTargets.equivalentPositionsOf(logical)
                .nearestToMeasurement()
                .whenUnavailable().reportUnavailable();
        PositionPlant plant = feedbackPlant(target, measurement,
                PositionPlant.Periodicity.PERIODIC, 360.0,
                ScalarRange.bounded(0.0, 720.0), NO_GUARDS);
        ManualLoopClock time = new ManualLoopClock();
        Task move = moveTo20(plant);

        move.start(time.clock());
        plant.update(time.clock());
        assertTrue("The mechanism is physically at the same numeric override",
                plant.atTarget(380.0));
        move.update(time.clock());
        assertFalse(move.isComplete());

        override[0] = false;
        plant.update(time.nextCycle(0.02));
        move.update(time.clock());
        assertTrue(move.isComplete());
    }

    @Test
    public void fallbackEqualToTheLogicalMoveValueDoesNotComplete() {
        ScalarTarget command = ScalarTarget.create(0.0);
        final double[] measurement = {20.0};
        PlantTargetResolver target = PlantTargets.equivalentPositionsOf(command)
                .nearestToMeasurement()
                .whenUnavailable().fallbackTo(20.0);
        PositionPlant plant = feedbackPlant(target, measurement,
                PositionPlant.Periodicity.NON_PERIODIC, Double.NaN,
                ScalarRange.bounded(0.0, 100.0), NO_GUARDS);
        ManualLoopClock time = new ManualLoopClock();
        Task move = moveTo20(plant);

        move.start(time.clock());
        plant.update(time.clock());
        assertEquals(PlantTargetResolution.Kind.FALLBACK, plant.getTargetResolution().kind());
        assertTrue(plant.atTarget(20.0));
        move.update(time.clock());

        assertFalse(move.isComplete());
    }

    @Test
    public void guardFallbackBlocksCompletionUntilPhysicalCommandIsAccepted() {
        ScalarTarget command = ScalarTarget.create(0.0);
        final boolean[] clear = {false};
        final double[] measurement = {300.0};
        PositionPlant plant = periodicPlant(command, measurement,
                step -> step.targetGuards()
                        .fallbackTargetUnless("turretClear", clock -> clear[0], 300.0)
                        .doneTargetGuards());
        ManualLoopClock time = new ManualLoopClock();
        Task move = moveTo20(plant);

        move.start(time.clock());
        plant.update(time.clock());
        assertEquals(380.0, plant.getRequestedTarget(), EPSILON);
        assertEquals(300.0, plant.getAppliedTarget(), EPSILON);
        assertEquals(PlantTargetStatus.Kind.FALLBACK_ACTIVE,
                plant.getTargetStatus().kind());
        move.update(time.clock());
        assertFalse(move.isComplete());

        clear[0] = true;
        measurement[0] = 380.0;
        plant.update(time.nextCycle(0.02));
        move.update(time.clock());
        assertTrue(move.isComplete());
    }

    @Test
    public void plannerClampAtTheRequestedNumberCannotMasqueradeAsTheCommandPath() {
        ScalarTarget command = ScalarTarget.create(0.0);
        final boolean[] plannerActive = {true};
        final double[] measurement = {100.0};
        PlantTargetResolver clampedPlan = PlantTargets.plan(
                        clock -> PlantTargetRequest.exact("outsideRange", 150.0))
                .nearestToMeasurement()
                .clampUnreachableToRange()
                .whenUnavailable().reportUnavailable();
        PlantTargetResolver target = PlantTargets.overlay(command)
                .add("planner", clock -> plannerActive[0], clampedPlan)
                .build();
        PositionPlant plant = feedbackPlant(target, measurement,
                PositionPlant.Periodicity.NON_PERIODIC, Double.NaN,
                ScalarRange.bounded(0.0, 100.0), NO_GUARDS);
        ManualLoopClock time = new ManualLoopClock();
        Task move = ScalarTasks.set(command, 100.0)
                .untilReachedBy(plant)
                .leaveTargetOnCancel()
                .build();

        move.start(time.clock());
        plant.update(time.clock());

        assertTrue(plant.getTargetResolution().clampedByPlanner());
        assertEquals(100.0, plant.getTargetResolution().target(), EPSILON);
        assertTrue("The mechanism is physically at the planner's clamped value",
                plant.atTarget(100.0));
        move.update(time.clock());
        assertFalse(move.isComplete());

        plannerActive[0] = false;
        plant.update(time.nextCycle(0.02));
        move.update(time.clock());

        assertTrue(move.isComplete());
        assertEquals(TaskOutcome.SUCCESS, move.getOutcome());
    }

    @Test
    public void rateLimitedAppliedTargetBlocksCompletionUntilTheCommandIsPhysicallyAccepted() {
        ScalarTarget command = ScalarTarget.create(0.0);
        final double[] measurement = {0.0};
        PositionPlant plant = feedbackPlant(PlantTargets.exact(command), measurement,
                PositionPlant.Periodicity.NON_PERIODIC, Double.NaN,
                ScalarRange.bounded(0.0, 100.0),
                step -> step.targetGuards()
                        .maxTargetRate(10.0)
                        .doneTargetGuards());
        ManualLoopClock time = new ManualLoopClock();
        Task move = ScalarTasks.set(command, 20.0)
                .untilReachedBy(plant)
                .leaveTargetOnCancel()
                .build();

        // Establish the limiter's physical starting point before the Task requests a move.
        plant.update(time.clock());
        move.start(time.clock());

        measurement[0] = 5.0;
        plant.update(time.nextCycle(0.5));
        assertEquals(20.0, plant.getRequestedTarget(), EPSILON);
        assertEquals(5.0, plant.getAppliedTarget(), EPSILON);
        assertEquals(PlantTargetStatus.Kind.RATE_LIMITED, plant.getTargetStatus().kind());
        move.update(time.clock());
        assertFalse(move.isComplete());

        measurement[0] = 20.0;
        plant.update(time.nextCycle(1.5));
        move.update(time.clock());

        assertEquals(PlantTargetStatus.Kind.ACCEPTED, plant.getTargetStatus().kind());
        assertTrue(move.isComplete());
        assertEquals(TaskOutcome.SUCCESS, move.getOutcome());
    }

    @Test
    public void changingTheLiveCommandInvalidatesAPreviouslyResolvedPlan() {
        ScalarTarget command = ScalarTarget.create(0.0);
        final double[] measurement = {380.0};
        PositionPlant plant = periodicPlant(command, measurement, NO_GUARDS);
        ManualLoopClock time = new ManualLoopClock();
        Task move = moveTo20(plant);

        move.start(time.clock());
        plant.update(time.clock());
        assertTrue(plant.atTarget(380.0));

        command.set(20.1);
        move.update(time.clock());

        assertFalse(move.isComplete());
    }

    @Test
    public void losingTheCommandPathResetsStableCompletionTime() {
        ScalarTarget command = ScalarTarget.create(0.0);
        final boolean[] override = {false};
        final double[] measurement = {380.0};
        PlantTargetResolver logical = PlantTargets.overlay(command)
                .add("sameValueOverride", clock -> override[0], 20.0)
                .build();
        PlantTargetResolver target = PlantTargets.equivalentPositionsOf(logical)
                .nearestToMeasurement()
                .whenUnavailable().reportUnavailable();
        PositionPlant plant = feedbackPlant(target, measurement,
                PositionPlant.Periodicity.PERIODIC, 360.0,
                ScalarRange.bounded(0.0, 720.0), NO_GUARDS);
        ManualLoopClock time = new ManualLoopClock();
        Task move = ScalarTasks.set(command, 20.0)
                .untilReachedBy(plant)
                .leaveTargetOnCancel()
                .stableFor(0.10)
                .build();

        move.start(time.clock());
        plant.update(time.clock());
        move.update(time.clock());

        override[0] = true;
        plant.update(time.nextCycle(0.06));
        move.update(time.clock());
        assertFalse(move.isComplete());

        override[0] = false;
        plant.update(time.nextCycle(0.06));
        move.update(time.clock());
        plant.update(time.nextCycle(0.06));
        move.update(time.clock());
        assertFalse("stable time before the override must not count", move.isComplete());

        plant.update(time.nextCycle(0.05));
        move.update(time.clock());
        assertTrue(move.isComplete());
    }

    @Test
    public void losingAValidFrameworkPlanResetsStableCompletionTime() {
        ScalarTarget command = ScalarTarget.create(0.0);
        final boolean[] unavailable = {false};
        final double[] measurement = {20.0};
        PlantTargetResolver target = PlantTargets.overlay(command)
                .add("requiredUnavailable", clock -> unavailable[0],
                        PlantTargets.exact(clock -> Double.NaN))
                .build();
        PositionPlant plant = feedbackPlant(target, measurement,
                PositionPlant.Periodicity.NON_PERIODIC, Double.NaN,
                ScalarRange.bounded(0.0, 100.0), NO_GUARDS);
        ManualLoopClock time = new ManualLoopClock();
        Task move = ScalarTasks.set(command, 20.0)
                .untilReachedBy(plant)
                .leaveTargetOnCancel()
                .stableFor(0.10)
                .build();

        move.start(time.clock());
        plant.update(time.clock());
        assertTrue(plant.getTargetResolution().hasTarget());
        move.update(time.clock());

        unavailable[0] = true;
        plant.update(time.nextCycle(0.06));
        assertFalse(plant.getTargetResolution().hasTarget());
        assertEquals(PlantTargetStatus.Kind.TARGET_UNAVAILABLE,
                plant.getTargetStatus().kind());
        move.update(time.clock());
        assertFalse(move.isComplete());

        unavailable[0] = false;
        plant.update(time.nextCycle(0.06));
        move.update(time.clock());
        plant.update(time.nextCycle(0.06));
        move.update(time.clock());
        assertFalse("stable time before plan loss must not count", move.isComplete());

        plant.update(time.nextCycle(0.05));
        move.update(time.clock());
        assertTrue(move.isComplete());
        assertEquals(TaskOutcome.SUCCESS, move.getOutcome());
    }

    @Test
    public void periodicityDoesNotAutomaticallyWrapAnExactCommand() {
        ScalarTarget command = ScalarTarget.create(0.0);
        final double[] measurement = {380.0};
        PositionPlant plant = feedbackPlant(PlantTargets.exact(command), measurement,
                PositionPlant.Periodicity.PERIODIC, 360.0,
                ScalarRange.bounded(0.0, 720.0), NO_GUARDS);
        ManualLoopClock time = new ManualLoopClock();
        Task move = moveTo20(plant);

        move.start(time.clock());
        plant.update(time.clock());
        move.update(time.clock());

        assertEquals(20.0, plant.getRequestedTarget(), EPSILON);
        assertFalse(move.isComplete());

        measurement[0] = 20.0;
        plant.update(time.nextCycle(0.02));
        move.update(time.clock());
        assertTrue(move.isComplete());
    }

    private static PositionPlant periodicPlant(
            ScalarTarget command,
            double[] measurement,
            UnaryOperator<Plants.TargetStep<PositionPlant>> guardConfiguration) {
        PlantTargetResolver target = PlantTargets.equivalentPositionsOf(command)
                .nearestToMeasurement()
                .whenUnavailable().reportUnavailable();
        return feedbackPlant(target, measurement, PositionPlant.Periodicity.PERIODIC, 360.0,
                ScalarRange.bounded(0.0, 720.0), guardConfiguration);
    }

    private static PositionPlant feedbackPlant(
            PlantTargetResolver target,
            double[] measurement,
            PositionPlant.Periodicity periodicity,
            double period,
            ScalarRange range,
            UnaryOperator<Plants.TargetStep<PositionPlant>> guardConfiguration) {
        Plants.TargetStep<PositionPlant> targetStep;
        if (periodicity == PositionPlant.Periodicity.PERIODIC) {
            targetStep = Plants.fromOutputs()
                    .deviceManagedPosition(new RecordingPositionOutput(), clock -> measurement[0])
                    .periodic(period)
                    .bounded(range.minValue, range.maxValue)
                    .nativeUnits()
                    .alreadyReferenced()
                    .positionTolerance(0.5);
        } else {
            targetStep = Plants.fromOutputs()
                    .deviceManagedPosition(new RecordingPositionOutput(), clock -> measurement[0])
                    .nonPeriodic()
                    .bounded(range.minValue, range.maxValue)
                    .nativeUnits()
                    .alreadyReferenced()
                    .positionTolerance(0.5);
        }
        return guardConfiguration.apply(targetStep)
                .targetFromResolver(target)
                .build();
    }

    private static Task moveTo20(Plant plant) {
        return ScalarTasks.set(plant.commandTarget(), 20.0)
                .untilReachedBy(plant)
                .leaveTargetOnCancel()
                .build();
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
}
