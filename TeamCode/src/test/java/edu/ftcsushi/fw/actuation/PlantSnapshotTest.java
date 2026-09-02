package edu.ftcsushi.fw.actuation;

import org.junit.Test;

import java.lang.reflect.Method;

import edu.ftcsushi.fw.core.hal.PositionOutput;
import edu.ftcsushi.fw.core.hal.PowerOutput;
import edu.ftcsushi.fw.core.hal.VelocityOutput;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.source.ScalarTarget;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Focused contract tests for immutable scalar Plant captures. */
public final class PlantSnapshotTest {

    private static final double EPSILON = 1e-12;

    @Test
    public void publicApiUsesOneCovariantSnapshotNoun() throws Exception {
        Method plantSnapshot = Plant.class.getDeclaredMethod("snapshot");
        Method positionSnapshot = PositionPlant.class.getDeclaredMethod("snapshot");

        assertSame(PlantSnapshot.class, plantSnapshot.getReturnType());
        assertSame(PositionPlantSnapshot.class, positionSnapshot.getReturnType());
        assertTrue(PlantSnapshot.class.isAssignableFrom(PositionPlantSnapshot.class));
    }

    @Test
    public void defaultCaptureReadsEachCachedFactOnceAndFreezesDerivedErrors() {
        CountingPlant plant = new CountingPlant();
        plant.requested = 7.0;
        plant.applied = 6.0;
        plant.measurement = 4.0;
        plant.resolution = PlantTargetResolution.exact(7.0, "counting target");
        plant.status = PlantTargetStatus.rateLimited("counting rate limit");
        plant.feedback = true;
        plant.atCurrent = false;

        PlantSnapshot captured = plant.snapshot();

        assertFalse(captured.hasCommandTarget());
        assertTrue(Double.isNaN(captured.commandTarget()));
        assertEquals(7.0, captured.requestedTarget(), 0.0);
        assertEquals(6.0, captured.appliedTarget(), 0.0);
        assertSame(plant.resolution, captured.targetResolution());
        assertSame(plant.status, captured.targetStatus());
        assertTrue(captured.hasFeedback());
        assertTrue(captured.hasMeasurement());
        assertEquals(4.0, captured.measurement(), 0.0);
        assertEquals(3.0, captured.requestedTargetError(), 0.0);
        assertEquals(2.0, captured.appliedTargetError(), 0.0);
        assertFalse(captured.atTarget());
        assertFalse(captured.atCommandTarget());
        plant.assertBaseQueries(1);
        assertEquals("snapshot derives requested error from its one measurement", 0,
                plant.requestedErrorQueries);
        assertEquals("snapshot derives applied error from its one measurement", 0,
                plant.appliedErrorQueries);
        assertEquals(0, plant.literalArrivalQueries);

        plant.requested = 100.0;
        plant.applied = 99.0;
        plant.measurement = 98.0;
        plant.atCurrent = true;

        assertEquals(7.0, captured.requestedTarget(), 0.0);
        assertEquals(6.0, captured.appliedTarget(), 0.0);
        assertEquals(4.0, captured.measurement(), 0.0);
        assertFalse(captured.atTarget());
    }

    @Test
    public void customPlantWithoutFrameworkProvenanceUsesLiteralCommandEvidence() {
        CountingPlant plant = new CountingPlant();
        plant.command = new CountingTarget(5.0);
        plant.requested = 5.0;
        plant.applied = 5.0;
        plant.measurement = 5.0;
        plant.resolution = PlantTargetResolution.exact(5.0, "custom exact result");
        plant.status = PlantTargetStatus.ACCEPTED;
        plant.feedback = true;
        plant.atCurrent = true;

        PlantSnapshot reached = plant.snapshot();

        assertTrue(reached.hasCommandTarget());
        assertEquals(5.0, reached.commandTarget(), 0.0);
        assertTrue(reached.atTarget());
        assertTrue(reached.atCommandTarget());
        assertEquals(1, plant.command.getQueries);
        assertEquals(1, plant.literalArrivalQueries);

        plant.command.set(6.0);
        PlantSnapshot superseded = plant.snapshot();

        assertEquals(6.0, superseded.commandTarget(), 0.0);
        assertTrue("current-target evidence remains a distinct cached Plant fact",
                superseded.atTarget());
        assertFalse(superseded.atCommandTarget());
        assertEquals(2, plant.literalArrivalQueries);
    }

    @Test
    public void literalCommandArrivalDoesNotRequireCurrentTargetArrivalOverride() {
        CountingPlant plant = new CountingPlant();
        plant.command = new CountingTarget(5.0);
        plant.requested = 5.0;
        plant.applied = 5.0;
        plant.measurement = 5.0;
        plant.resolution = PlantTargetResolution.exact(5.0, "custom literal result");
        plant.status = PlantTargetStatus.ACCEPTED;
        plant.feedback = true;
        plant.atCurrent = false;
        plant.literalArrival = true;

        PlantSnapshot snapshot = plant.snapshot();

        assertFalse(snapshot.atTarget());
        assertTrue("literal atTarget(value) is the custom command-completion contract",
                snapshot.atCommandTarget());
    }

    @Test
    public void frameworkCommandProvenanceInvalidatesBeforeTheNextHeartbeat() {
        RecordingVelocityOutput output = new RecordingVelocityOutput();
        double[] measurement = {4.0};
        Plant plant = Plants.fromOutputs()
                .deviceManagedVelocity(output, clock -> measurement[0])
                .bounded(0.0, 10.0)
                .nativeUnits()
                .velocityTolerance(0.0)
                .targetFromNewCommand(4.0)
                .build();
        ManualLoopClock time = new ManualLoopClock();

        PlantSnapshot beforeHeartbeat = plant.snapshot();
        assertTrue(beforeHeartbeat.hasCommandTarget());
        assertEquals(4.0, beforeHeartbeat.commandTarget(), 0.0);
        assertFalse(beforeHeartbeat.hasMeasurement());
        assertFalse(beforeHeartbeat.atCommandTarget());

        plant.update(time.clock());
        PlantSnapshot reached = plant.snapshot();
        assertEquals(4.0, reached.requestedTarget(), 0.0);
        assertEquals(4.0, reached.appliedTarget(), 0.0);
        assertEquals(4.0, reached.measurement(), 0.0);
        assertTrue(reached.atTarget());
        assertTrue(reached.atCommandTarget());

        plant.commandTarget().set(5.0);
        PlantSnapshot pending = plant.snapshot();
        assertEquals(5.0, pending.commandTarget(), 0.0);
        assertEquals(4.0, pending.requestedTarget(), 0.0);
        assertTrue("the last-heartbeat arrival fact remains separately truthful",
                pending.atTarget());
        assertFalse("new live command has no matching resolution provenance yet",
                pending.atCommandTarget());

        measurement[0] = 5.0;
        plant.update(time.nextCycle(0.02));
        PlantSnapshot reachedNext = plant.snapshot();
        assertTrue(reachedNext.atCommandTarget());
        assertEquals("older immutable captures do not drift", 4.0, reached.commandTarget(), 0.0);

        plant.stop();
        PlantSnapshot stopped = plant.snapshot();
        assertEquals(5.0, stopped.commandTarget(), 0.0);
        assertEquals(0.0, stopped.appliedTarget(), 0.0);
        assertEquals(PlantTargetStatus.Kind.STOPPED, stopped.targetStatus().kind());
        assertFalse(stopped.atTarget());
        assertFalse(stopped.atCommandTarget());
    }

    @Test
    public void overlayCannotClaimCommandArrivalFromACoincidentallyEqualTarget() {
        ScalarTarget command = ScalarTarget.create(4.0);
        PlantTargetResolver resolver = PlantTargets.overlay(PlantTargets.exact(command))
                .add("equal override", BooleanSource.constant(true), 4.0)
                .build();
        Plant plant = Plants.fromOutputs()
                .deviceManagedVelocity(new RecordingVelocityOutput(), clock -> 4.0)
                .bounded(0.0, 10.0)
                .nativeUnits()
                .velocityTolerance(0.0)
                .targetFromResolver(resolver)
                .build();

        plant.update(new ManualLoopClock().clock());
        PlantSnapshot snapshot = plant.snapshot();

        assertTrue(snapshot.atTarget());
        assertEquals(snapshot.commandTarget(), snapshot.requestedTarget(), 0.0);
        assertFalse("selected overlay provenance must not be inferred from equal doubles",
                snapshot.atCommandTarget());
    }

    @Test
    public void positionCaptureAddsReferenceRangeAndSearchCapability() {
        RecordingPositionOutput output = new RecordingPositionOutput();
        RecordingPowerOutput search = new RecordingPowerOutput();
        double[] nativeMeasurement = {10.0};
        PositionPlant plant = Plants.fromOutputs()
                .deviceManagedPosition(output, clock -> nativeMeasurement[0])
                .searchPowerOutput(search)
                .nonPeriodic()
                .bounded(0.0, 20.0)
                .nativeUnits()
                .needsReference("lift reference required")
                .positionTolerance(0.1)
                .targetFromNewCommand(2.0)
                .build();
        ManualLoopClock time = new ManualLoopClock();

        PositionPlantSnapshot initial = plant.snapshot();
        assertEquals(PositionPlant.Periodicity.NON_PERIODIC, initial.periodicity());
        assertTrue(Double.isNaN(initial.period()));
        assertFalse(initial.targetRange().valid);
        assertFalse(initial.isReferenced());
        assertEquals("lift reference required", initial.referenceStatus());
        assertTrue(initial.supportsCalibrationSearch());
        assertTrue(initial.hasFeedback());
        assertFalse(initial.hasMeasurement());

        plant.update(time.clock());
        plant.establishReferenceAt(0.0);
        PositionPlantSnapshot referenced = plant.snapshot();
        assertTrue(referenced.isReferenced());
        assertTrue(referenced.targetRange().valid);
        assertEquals(0.0, referenced.targetRange().minValue, 0.0);
        assertEquals(20.0, referenced.targetRange().maxValue, 0.0);
        assertEquals(0.0, referenced.measurement(), 0.0);

        plant.beginCalibrationSearch(-0.2);
        PositionPlantSnapshot searching = plant.snapshot();
        assertEquals(PlantTargetStatus.Kind.HOLDING_LAST, searching.targetStatus().kind());
        assertFalse(searching.atTarget());
        assertFalse(searching.atCommandTarget());
        plant.endCalibrationSearch();

        plant.update(time.nextCycle(0.02));
        PositionPlantSnapshot resumed = plant.snapshot();
        assertEquals(2.0, resumed.requestedTarget(), 0.0);
        assertEquals(2.0, resumed.appliedTarget(), 0.0);
        assertTrue("old position capture stays unreferenced", !initial.isReferenced());
    }

    @Test
    public void failedTerminalStopRetainsTargetFactsButCannotRetainCommandArrival() {
        RecordingVelocityOutput output = new RecordingVelocityOutput();
        output.stopFailure = new IllegalStateException("velocity stop failed");
        Plant plant = Plants.fromOutputs()
                .deviceManagedVelocity(output, clock -> 4.0)
                .bounded(0.0, 10.0)
                .nativeUnits()
                .velocityTolerance(0.0)
                .targetFromNewCommand(4.0)
                .build();
        plant.update(new ManualLoopClock().clock());
        assertTrue(plant.snapshot().atCommandTarget());

        try {
            plant.stop();
            fail("Expected terminal stop failure");
        } catch (RuntimeException actual) {
            assertSame(output.stopFailure, actual);
        }

        PlantSnapshot failedStop = plant.snapshot();
        assertEquals(4.0, failedStop.appliedTarget(), 0.0);
        assertEquals(PlantTargetStatus.Kind.ACCEPTED, failedStop.targetStatus().kind());
        assertFalse(failedStop.atTarget());
        assertFalse(failedStop.atCommandTarget());
    }

    private static final class CountingPlant implements Plant {
        double requested;
        double applied;
        double measurement = Double.NaN;
        PlantTargetResolution resolution = PlantTargetResolution.unavailable("not sampled");
        PlantTargetStatus status = PlantTargetStatus.STOPPED;
        boolean feedback;
        boolean atCurrent;
        boolean literalArrival;
        CountingTarget command;

        int commandCapabilityQueries;
        int commandObjectQueries;
        int requestedQueries;
        int appliedQueries;
        int resolutionQueries;
        int statusQueries;
        int feedbackQueries;
        int measurementQueries;
        int currentArrivalQueries;
        int literalArrivalQueries;
        int requestedErrorQueries;
        int appliedErrorQueries;

        @Override public void update(LoopClock clock) { }
        @Override public double getRequestedTarget() { requestedQueries++; return requested; }
        @Override public double getAppliedTarget() { appliedQueries++; return applied; }
        @Override public PlantTargetResolution getTargetResolution() {
            resolutionQueries++;
            return resolution;
        }
        @Override public PlantTargetStatus getTargetStatus() { statusQueries++; return status; }
        @Override public boolean hasFeedback() { feedbackQueries++; return feedback; }
        @Override public double getMeasurement() { measurementQueries++; return measurement; }
        @Override public double getRequestedTargetError() {
            requestedErrorQueries++;
            return Plant.super.getRequestedTargetError();
        }
        @Override public double getAppliedTargetError() {
            appliedErrorQueries++;
            return Plant.super.getAppliedTargetError();
        }
        @Override public boolean atTarget() { currentArrivalQueries++; return atCurrent; }
        @Override public boolean atTarget(double target) {
            literalArrivalQueries++;
            return (atCurrent || literalArrival) && Double.compare(target, requested) == 0;
        }
        @Override public boolean hasCommandTarget() {
            commandCapabilityQueries++;
            return command != null;
        }
        @Override public ScalarTarget commandTarget() {
            commandObjectQueries++;
            return command;
        }
        @Override public void stop() { atCurrent = false; }

        void assertBaseQueries(int expected) {
            assertEquals(expected, commandCapabilityQueries);
            assertEquals(command == null ? 0 : expected, commandObjectQueries);
            assertEquals(expected, requestedQueries);
            assertEquals(expected, appliedQueries);
            assertEquals(expected, resolutionQueries);
            assertEquals(expected, statusQueries);
            assertEquals(expected, feedbackQueries);
            assertEquals(expected, measurementQueries);
            assertEquals(expected, currentArrivalQueries);
        }
    }

    private static final class CountingTarget implements ScalarTarget {
        double value;
        int getQueries;

        CountingTarget(double value) {
            this.value = value;
        }

        @Override public void set(double value) { this.value = value; }
        @Override public double get() { getQueries++; return value; }
    }

    private static final class RecordingVelocityOutput implements VelocityOutput {
        double command = Double.NaN;
        RuntimeException stopFailure;

        @Override public void setVelocity(double velocity) { command = velocity; }
        @Override public double getCommandedVelocity() { return command; }
        @Override public void stop() {
            if (stopFailure != null) throw stopFailure;
            command = 0.0;
        }
    }

    private static final class RecordingPositionOutput implements PositionOutput {
        double command;

        @Override public void setPosition(double position) { command = position; }
        @Override public double getCommandedPosition() { return command; }
    }

    private static final class RecordingPowerOutput implements PowerOutput {
        double command = Double.NaN;

        @Override public void setPower(double power) { command = power; }
        @Override public double getCommandedPower() { return command; }
    }
}
