package edu.ftcsushi.fw.actuation;

import org.junit.Test;

import edu.ftcsushi.fw.core.control.ScalarRegulator;
import edu.ftcsushi.fw.core.hal.PowerOutput;
import edu.ftcsushi.fw.core.hal.PositionOutput;
import edu.ftcsushi.fw.core.hal.VelocityOutput;
import edu.ftcsushi.fw.core.source.ScalarSource;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies finite affine-map construction, runtime conversion, and failure-state truth. */
public final class MappedPlantAffineValidationTest {

    @Test
    public void boundedVelocityMapRejectsBeforeMutationAndAllowsRetry() {
        RecordingVelocityOutput output = new RecordingVelocityOutput();
        Plants.VelocityMappingStep<Plants.TargetStep<Plant>> mapping = Plants.fromOutputs()
                .deviceManagedVelocity(output, clock -> 0.0)
                .bounded(0.0, Double.MAX_VALUE);

        RuntimeException failure = expectRuntime(() -> mapping.scaleToNative(2.0));

        assertBoundedMappingFailure(failure, "velocity", Double.MAX_VALUE);
        assertEquals(0, output.setCalls);
        assertEquals(0, output.stopCalls);

        Plant plant = mapping.scaleToNative(1.0)
                .velocityTolerance(0.0)
                .targetFromNewCommand(0.0)
                .build();
        plant.update(new ManualLoopClock().clock());

        assertEquals(1, output.setCalls);
        assertEquals(0.0, output.commanded, 0.0);
    }

    @Test
    public void boundedPositionReferenceRejectsBeforeMutationAndAllowsRetry() {
        RecordingPositionOutput output = new RecordingPositionOutput();
        Plants.CommandedPositionReferenceStep reference = Plants.fromOutputs()
                .commandedPosition(output)
                .nonPeriodic()
                .bounded(0.0, Double.MAX_VALUE)
                .scaleToNative(1.0);

        RuntimeException failure = expectRuntime(() ->
                reference.plantPositionMapsToNative(0.0, Double.MAX_VALUE));

        assertBoundedMappingFailure(failure, "position", Double.MAX_VALUE);
        assertEquals(0, output.setCalls);
        assertEquals(0, output.stopCalls);

        PositionPlant plant = reference.plantPositionMapsToNative(0.0, 0.0)
                .targetFromNewCommand(0.0)
                .build();
        plant.update(new ManualLoopClock().clock());

        assertEquals(1, output.setCalls);
        assertEquals(0.0, output.commanded, 0.0);
    }

    @Test
    public void mappedRuntimeBuildRechecksFullyBoundedEndpointImages() {
        RecordingVelocityOutput velocityOutput = new RecordingVelocityOutput();
        RuntimeException velocityFailure = expectRuntime(() ->
                MappedVelocityPlant.velocityOutput(velocityOutput, clock -> 0.0)
                        .range(ScalarRange.bounded(0.0, Double.MAX_VALUE))
                        .nativePerPlantUnit(2.0)
                        .velocityTolerance(0.0)
                        .targetFromResolver(PlantTargets.exact(0.0))
                        .build());
        assertBoundedMappingFailure(velocityFailure, "velocity", Double.MAX_VALUE);

        RecordingPositionOutput positionOutput = new RecordingPositionOutput();
        RuntimeException positionFailure = expectRuntime(() ->
                MappedPositionPlant.commanded(positionOutput)
                        .range(ScalarRange.bounded(0.0, Double.MAX_VALUE))
                        .nativePerPlantUnit(1.0)
                        .plantPositionMapsToNative(0.0, Double.MAX_VALUE)
                        .targetFromResolver(PlantTargets.exact(0.0))
                        .build());
        assertBoundedMappingFailure(positionFailure, "position", Double.MAX_VALUE);

        assertEquals(0, velocityOutput.setCalls);
        assertEquals(0, velocityOutput.stopCalls);
        assertEquals(0, positionOutput.setCalls);
        assertEquals(0, positionOutput.stopCalls);
    }

    @Test
    public void unboundedVelocityOverflowStopsBeforeAppliedCommitOrOutput() {
        RecordingVelocityOutput output = new RecordingVelocityOutput();
        Plant plant = Plants.fromOutputs()
                .deviceManagedVelocity(output, clock -> 2.0)
                .unbounded()
                .scaleToNative(2.0)
                .velocityTolerance(0.0)
                .targetFromNewCommand(1.0)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        plant.update(clock.clock());
        plant.commandTarget().set(Double.MAX_VALUE);

        RuntimeException failure = expectRuntime(() -> plant.update(clock.nextCycle(0.02)));

        assertRuntimeMappingFailure(failure, "VelocityPlant.update", Double.MAX_VALUE);
        assertEquals(Double.MAX_VALUE, plant.getRequestedTarget(), 0.0);
        assertEquals(0.0, plant.getAppliedTarget(), 0.0);
        assertEquals(PlantTargetStatus.Kind.STOPPED, plant.getTargetStatus().kind());
        assertFalse(plant.getTargetResolution().hasTarget());
        assertFalse(plant.atTarget());
        assertEquals(1, output.setCalls);
        assertEquals(1, output.stopCalls);
        assertEquals(0.0, output.commanded, 0.0);

        plant.commandTarget().set(3.0);
        plant.update(clock.nextCycle(0.02));

        assertEquals(3.0, plant.getAppliedTarget(), 0.0);
        assertEquals(PlantTargetStatus.Kind.ACCEPTED, plant.getTargetStatus().kind());
        assertEquals(2, output.setCalls);
        assertEquals(1, output.stopCalls);
        assertEquals(6.0, output.commanded, 0.0);
    }

    @Test
    public void unboundedPositionOverflowStopsBeforeAppliedCommitOrOutput() {
        RecordingPositionOutput output = new RecordingPositionOutput();
        PositionPlant plant = Plants.fromOutputs()
                .commandedPosition(output)
                .nonPeriodic()
                .unbounded()
                .scaleToNative(2.0)
                .plantPositionMapsToNative(0.0, 0.0)
                .targetFromNewCommand(1.0)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        plant.update(clock.clock());
        plant.commandTarget().set(Double.MAX_VALUE);

        RuntimeException failure = expectRuntime(() -> plant.update(clock.nextCycle(0.02)));

        assertRuntimeMappingFailure(failure, "PositionPlant.update", Double.MAX_VALUE);
        assertEquals(Double.MAX_VALUE, plant.getRequestedTarget(), 0.0);
        assertEquals(1.0, plant.getAppliedTarget(), 0.0);
        assertEquals(PlantTargetStatus.Kind.STOPPED, plant.getTargetStatus().kind());
        assertFalse(plant.getTargetResolution().hasTarget());
        assertFalse(plant.atTarget());
        assertEquals(1, output.setCalls);
        assertEquals(1, output.stopCalls);
        assertEquals(2.0, output.commanded, 0.0);

        plant.commandTarget().set(3.0);
        plant.update(clock.nextCycle(0.02));

        assertEquals(3.0, plant.getAppliedTarget(), 0.0);
        assertEquals(PlantTargetStatus.Kind.ACCEPTED, plant.getTargetStatus().kind());
        assertEquals(2, output.setCalls);
        assertEquals(1, output.stopCalls);
        assertEquals(6.0, output.commanded, 0.0);
    }

    @Test
    public void velocityStopFailureIsSuppressedAndPriorTargetFactsAreRestored() {
        RecordingVelocityOutput output = new RecordingVelocityOutput();
        Plant plant = Plants.fromOutputs()
                .deviceManagedVelocity(output, clock -> 2.0)
                .unbounded()
                .scaleToNative(2.0)
                .velocityTolerance(0.0)
                .targetFromNewCommand(1.0)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        plant.update(clock.clock());
        double priorApplied = plant.getAppliedTarget();
        PlantTargetStatus priorStatus = plant.getTargetStatus();
        PlantTargetResolution priorResolution = plant.getTargetResolution();
        output.failStop = true;
        plant.commandTarget().set(Double.MAX_VALUE);

        RuntimeException failure = expectRuntime(() -> plant.update(clock.nextCycle(0.02)));

        assertRuntimeMappingFailure(failure, "VelocityPlant.update", Double.MAX_VALUE);
        assertEquals(1, failure.getSuppressed().length);
        assertTrue(failure.getSuppressed()[0].getMessage().contains("velocity stop failed"));
        assertEquals(Double.MAX_VALUE, plant.getRequestedTarget(), 0.0);
        assertEquals(priorApplied, plant.getAppliedTarget(), 0.0);
        assertSame(priorStatus, plant.getTargetStatus());
        assertSame(priorResolution, plant.getTargetResolution());
        assertEquals(1, output.setCalls);
        assertEquals(1, output.stopCalls);
    }

    @Test
    public void positionStopFailureIsSuppressedAndPriorTargetFactsAreRestored() {
        RecordingPositionOutput output = new RecordingPositionOutput();
        PositionPlant plant = Plants.fromOutputs()
                .commandedPosition(output)
                .nonPeriodic()
                .unbounded()
                .scaleToNative(2.0)
                .plantPositionMapsToNative(0.0, 0.0)
                .targetFromNewCommand(1.0)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        plant.update(clock.clock());
        double priorApplied = plant.getAppliedTarget();
        PlantTargetStatus priorStatus = plant.getTargetStatus();
        PlantTargetResolution priorResolution = plant.getTargetResolution();
        output.failStop = true;
        plant.commandTarget().set(Double.MAX_VALUE);

        RuntimeException failure = expectRuntime(() -> plant.update(clock.nextCycle(0.02)));

        assertRuntimeMappingFailure(failure, "PositionPlant.update", Double.MAX_VALUE);
        assertEquals(1, failure.getSuppressed().length);
        assertTrue(failure.getSuppressed()[0].getMessage().contains("position stop failed"));
        assertEquals(Double.MAX_VALUE, plant.getRequestedTarget(), 0.0);
        assertEquals(priorApplied, plant.getAppliedTarget(), 0.0);
        assertSame(priorStatus, plant.getTargetStatus());
        assertSame(priorResolution, plant.getTargetResolution());
        assertEquals(1, output.setCalls);
        assertEquals(1, output.stopCalls);
    }

    @Test
    public void throwingRegulatorResetRestoresPriorFactsAfterPhysicalPositionStop() {
        RecordingPowerOutput output = new RecordingPowerOutput();
        MutableScalarSource source = new MutableScalarSource(0.0);
        ThrowingResetRegulator regulator = new ThrowingResetRegulator();
        PositionPlant plant = Plants.fromOutputs()
                .regulatedPosition(output, source)
                .nonPeriodic()
                .bounded(0.0, Double.MAX_VALUE)
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(0.0)
                .controlFromCustomRegulator(regulator)
                .targetFromNewCommand(0.0)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        plant.update(clock.clock());
        double priorApplied = plant.getAppliedTarget();
        PlantTargetStatus priorStatus = plant.getTargetStatus();
        PlantTargetResolution priorResolution = plant.getTargetResolution();
        source.value = Double.MAX_VALUE;
        regulator.failReset = true;

        RuntimeException failure = expectRuntime(() ->
                plant.establishReferenceAt(0.0, clock.nextCycle(0.02)));

        assertRuntimeMappingFailure(
                failure, "PositionPlant.establishReferenceAt", Double.MAX_VALUE);
        assertEquals(1, failure.getSuppressed().length);
        assertTrue(failure.getSuppressed()[0].getMessage().contains("regulator reset failed"));
        assertEquals(0.0, plant.getRequestedTarget(), 0.0);
        assertEquals(priorApplied, plant.getAppliedTarget(), 0.0);
        assertSame(priorStatus, plant.getTargetStatus());
        assertSame(priorResolution, plant.getTargetResolution());
        assertTrue(plant.isReferenced());
        assertEquals(0.0, plant.getMeasurement(), 0.0);
        assertEquals(1, output.setCalls);
        assertEquals(1, output.stopCalls);
    }

    @Test
    public void inverseVelocityOverflowPublishesUnavailableMeasurement() {
        RecordingVelocityOutput output = new RecordingVelocityOutput();
        Plant plant = Plants.fromOutputs()
                .deviceManagedVelocity(output, clock -> Double.MAX_VALUE)
                .unbounded()
                .scaleToNative(1.0e-12)
                .velocityTolerance(0.0)
                .targetFromNewCommand(0.0)
                .build();

        plant.update(new ManualLoopClock().clock());

        assertTrue(Double.isNaN(plant.getMeasurement()));
        assertFalse(plant.atTarget());
        assertFalse(plant.atTarget(0.0));
        assertEquals(0.0, output.commanded, 0.0);
    }

    @Test
    public void inversePositionOverflowPublishesUnavailableMeasurement() {
        RecordingPositionOutput output = new RecordingPositionOutput();
        PositionPlant plant = Plants.fromOutputs()
                .deviceManagedPosition(output, clock -> Double.MAX_VALUE)
                .nonPeriodic()
                .unbounded()
                .nativeUnits()
                .plantPositionMapsToNative(0.0, -Double.MAX_VALUE)
                .positionTolerance(0.0)
                .targetFromNewCommand(0.0)
                .build();

        plant.update(new ManualLoopClock().clock());

        assertTrue(Double.isNaN(plant.getMeasurement()));
        assertFalse(plant.atTarget());
        assertFalse(plant.atTarget(0.0));
        assertEquals(-Double.MAX_VALUE, output.commanded, 0.0);
    }

    @Test
    public void dynamicBoundedReferenceOverflowIsAtomicAndCanRetry() {
        RecordingPositionOutput output = new RecordingPositionOutput();
        MutableScalarSource source = new MutableScalarSource(Double.MAX_VALUE);
        PositionPlant plant = Plants.fromOutputs()
                .deviceManagedPosition(output, source)
                .nonPeriodic()
                .bounded(0.0, Double.MAX_VALUE)
                .nativeUnits()
                .assumeCurrentPositionIs(0.0)
                .positionTolerance(0.0)
                .targetFromNewCommand(0.0)
                .build();
        ManualLoopClock clock = new ManualLoopClock();

        RuntimeException failure = expectRuntime(() -> plant.update(clock.clock()));

        assertRuntimeMappingFailure(
                failure, "PositionPlant.establishReferenceAt", Double.MAX_VALUE);
        assertFalse(plant.isReferenced());
        assertTrue(plant.referenceStatus().contains("pending"));
        assertTrue(Double.isNaN(plant.getMeasurement()));
        assertEquals(0.0, plant.getAppliedTarget(), 0.0);
        assertEquals(PlantTargetStatus.Kind.STOPPED, plant.getTargetStatus().kind());
        assertEquals(0, output.setCalls);
        assertEquals(1, output.stopCalls);

        source.value = 0.0;
        plant.update(clock.nextCycle(0.02));

        assertTrue(plant.isReferenced());
        assertEquals(0.0, plant.getMeasurement(), 0.0);
        assertEquals(0.0, output.commanded, 0.0);
        assertEquals(1, output.setCalls);
        assertEquals(1, output.stopCalls);
    }

    private static void assertBoundedMappingFailure(
            RuntimeException failure,
            String coordinate,
            double endpoint) {
        assertTrue("expected IllegalArgumentException, got " + failure,
                failure instanceof IllegalArgumentException);
        String message = String.valueOf(failure.getMessage());
        assertTrue(message.contains("bounded Plant " + coordinate + " endpoint"));
        assertTrue(message.contains(Double.toString(endpoint)));
        assertTrue(message.contains("non-finite"));
    }

    private static void assertRuntimeMappingFailure(
            RuntimeException failure,
            String operation,
            double value) {
        assertTrue("expected IllegalStateException, got " + failure,
                failure instanceof IllegalStateException);
        String message = String.valueOf(failure.getMessage());
        assertTrue(message.contains(operation));
        assertTrue(message.contains(Double.toString(value)));
        assertTrue(message.contains("finite"));
    }

    private static RuntimeException expectRuntime(Runnable operation) {
        try {
            operation.run();
            fail("Expected RuntimeException");
            return null;
        } catch (RuntimeException expected) {
            return expected;
        }
    }

    private static final class RecordingVelocityOutput implements VelocityOutput {
        private double commanded;
        private int setCalls;
        private int stopCalls;
        private boolean failStop;

        @Override
        public void setVelocity(double velocity) {
            setCalls++;
            commanded = velocity;
        }

        @Override
        public double getCommandedVelocity() {
            return commanded;
        }

        @Override
        public void stop() {
            stopCalls++;
            if (failStop) throw new IllegalStateException("velocity stop failed");
            commanded = 0.0;
        }
    }

    private static final class RecordingPositionOutput implements PositionOutput {
        private double commanded;
        private int setCalls;
        private int stopCalls;
        private boolean failStop;

        @Override
        public void setPosition(double position) {
            setCalls++;
            commanded = position;
        }

        @Override
        public double getCommandedPosition() {
            return commanded;
        }

        @Override
        public void stop() {
            stopCalls++;
            if (failStop) throw new IllegalStateException("position stop failed");
        }
    }

    private static final class RecordingPowerOutput implements PowerOutput {
        private double commanded;
        private int setCalls;
        private int stopCalls;

        @Override
        public void setPower(double power) {
            setCalls++;
            commanded = power;
        }

        @Override
        public double getCommandedPower() {
            return commanded;
        }

        @Override
        public void stop() {
            stopCalls++;
            commanded = 0.0;
        }
    }

    private static final class MutableScalarSource implements ScalarSource {
        private double value;

        private MutableScalarSource(double value) {
            this.value = value;
        }

        @Override
        public double getAsDouble(LoopClock clock) {
            return value;
        }
    }

    private static final class ThrowingResetRegulator implements ScalarRegulator {
        private boolean failReset;

        @Override
        public double update(double setpoint, double measurement, LoopClock clock) {
            return 0.0;
        }

        @Override
        public void reset() {
            if (failReset) throw new IllegalStateException("regulator reset failed");
        }
    }
}
