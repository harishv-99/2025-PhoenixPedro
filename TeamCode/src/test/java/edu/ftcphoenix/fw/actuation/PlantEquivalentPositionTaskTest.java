package edu.ftcphoenix.fw.actuation;

import org.junit.Test;

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

    private static final double EPSILON = 1e-9;

    @Test
    public void logicalMoveCompletesAtTheSelectedPhysicalEquivalent() {
        ScalarTarget command = ScalarTarget.create(0.0);
        final double[] measurement = {350.0};
        MappedPositionPlant plant = periodicPlant(command, measurement, PlantTargetGuards.none());
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
        PlantTargetSource logical = PlantTargets.overlay(command)
                .add("sameValueOverride", clock -> override[0], 20.0)
                .build();
        PlantTargetSource target = PlantTargets.equivalentPositionsOf(logical)
                .nearestToMeasurement()
                .whenUnavailable().reportUnavailable();
        MappedPositionPlant plant = feedbackPlant(target, measurement,
                PositionPlant.Topology.PERIODIC, 360.0,
                ScalarRange.bounded(0.0, 720.0), PlantTargetGuards.none());
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
        PlantTargetSource target = PlantTargets.equivalentPositionsOf(command)
                .nearestToMeasurement()
                .whenUnavailable().fallbackTo(20.0);
        MappedPositionPlant plant = feedbackPlant(target, measurement,
                PositionPlant.Topology.LINEAR, Double.NaN,
                ScalarRange.bounded(0.0, 100.0), PlantTargetGuards.none());
        ManualLoopClock time = new ManualLoopClock();
        Task move = moveTo20(plant);

        move.start(time.clock());
        plant.update(time.clock());
        assertEquals(PlantTargetPlan.Kind.FALLBACK, plant.getTargetPlan().kind());
        assertTrue(plant.atTarget(20.0));
        move.update(time.clock());

        assertFalse(move.isComplete());
    }

    @Test
    public void guardFallbackBlocksCompletionUntilPhysicalCommandIsAccepted() {
        ScalarTarget command = ScalarTarget.create(0.0);
        final boolean[] clear = {false};
        final double[] measurement = {300.0};
        PlantTargetGuards guards = PlantTargetGuards.builder()
                .fallbackTargetUnless("turretClear", clock -> clear[0], 300.0)
                .build();
        MappedPositionPlant plant = periodicPlant(command, measurement, guards);
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
        PlantTargetSource clampedPlan = PlantTargets.plan()
                .request(clock -> PlantTargetRequest.exact("outsideRange", 150.0))
                .nearestToMeasurement()
                .clampUnreachableToRange()
                .whenUnavailable().reportUnavailable();
        PlantTargetSource target = PlantTargets.overlay(command)
                .add("planner", clock -> plannerActive[0], clampedPlan)
                .build();
        MappedPositionPlant plant = feedbackPlant(target, measurement,
                PositionPlant.Topology.LINEAR, Double.NaN,
                ScalarRange.bounded(0.0, 100.0), PlantTargetGuards.none());
        ManualLoopClock time = new ManualLoopClock();
        Task move = ScalarTasks.set(command, 100.0)
                .untilReachedBy(plant)
                .leaveTargetOnCancel()
                .build();

        move.start(time.clock());
        plant.update(time.clock());

        assertTrue(plant.getTargetPlan().clampedByPlanner());
        assertEquals(100.0, plant.getTargetPlan().target(), EPSILON);
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
        PlantTargetGuards guards = PlantTargetGuards.builder()
                .maxTargetRate(10.0)
                .build();
        MappedPositionPlant plant = feedbackPlant(PlantTargets.exact(command), measurement,
                PositionPlant.Topology.LINEAR, Double.NaN,
                ScalarRange.bounded(0.0, 100.0), guards);
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
        MappedPositionPlant plant = periodicPlant(command, measurement, PlantTargetGuards.none());
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
        PlantTargetSource logical = PlantTargets.overlay(command)
                .add("sameValueOverride", clock -> override[0], 20.0)
                .build();
        PlantTargetSource target = PlantTargets.equivalentPositionsOf(logical)
                .nearestToMeasurement()
                .whenUnavailable().reportUnavailable();
        MappedPositionPlant plant = feedbackPlant(target, measurement,
                PositionPlant.Topology.PERIODIC, 360.0,
                ScalarRange.bounded(0.0, 720.0), PlantTargetGuards.none());
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
        PlantTargetSource target = PlantTargets.overlay(command)
                .add("requiredUnavailable", clock -> unavailable[0],
                        PlantTargets.exact(clock -> Double.NaN))
                .build();
        MappedPositionPlant plant = feedbackPlant(target, measurement,
                PositionPlant.Topology.LINEAR, Double.NaN,
                ScalarRange.bounded(0.0, 100.0), PlantTargetGuards.none());
        ManualLoopClock time = new ManualLoopClock();
        Task move = ScalarTasks.set(command, 20.0)
                .untilReachedBy(plant)
                .leaveTargetOnCancel()
                .stableFor(0.10)
                .build();

        move.start(time.clock());
        plant.update(time.clock());
        assertTrue(plant.getTargetPlan().hasTarget());
        move.update(time.clock());

        unavailable[0] = true;
        plant.update(time.nextCycle(0.06));
        assertFalse(plant.getTargetPlan().hasTarget());
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
    public void periodicTopologyDoesNotAutomaticallyWrapAnExactCommand() {
        ScalarTarget command = ScalarTarget.create(0.0);
        final double[] measurement = {380.0};
        MappedPositionPlant plant = feedbackPlant(PlantTargets.exact(command), measurement,
                PositionPlant.Topology.PERIODIC, 360.0,
                ScalarRange.bounded(0.0, 720.0), PlantTargetGuards.none());
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

    private static MappedPositionPlant periodicPlant(ScalarTarget command,
                                                     double[] measurement,
                                                     PlantTargetGuards guards) {
        PlantTargetSource target = PlantTargets.equivalentPositionsOf(command)
                .nearestToMeasurement()
                .whenUnavailable().reportUnavailable();
        return feedbackPlant(target, measurement, PositionPlant.Topology.PERIODIC, 360.0,
                ScalarRange.bounded(0.0, 720.0), guards);
    }

    private static MappedPositionPlant feedbackPlant(PlantTargetSource target,
                                                     double[] measurement,
                                                     PositionPlant.Topology topology,
                                                     double period,
                                                     ScalarRange range,
                                                     PlantTargetGuards guards) {
        return MappedPositionPlant.positionOutput(
                        new RecordingPositionOutput(), clock -> measurement[0])
                .topology(topology, period)
                .range(range)
                .positionTolerance(0.5)
                .targetGuards(guards)
                .targetedBy(target)
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
