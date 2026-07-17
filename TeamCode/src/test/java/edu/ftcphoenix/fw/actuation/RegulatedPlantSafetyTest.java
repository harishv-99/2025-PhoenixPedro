package edu.ftcphoenix.fw.actuation;

import org.junit.Test;

import java.util.LinkedHashMap;
import java.util.Map;

import edu.ftcphoenix.fw.core.control.PidfRegulator;
import edu.ftcphoenix.fw.core.control.ScalarRegulator;
import edu.ftcphoenix.fw.core.control.ScalarRegulators;
import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.source.ScalarTarget;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies SAFE-03 behavior at every regulated Plant integration path. */
public final class RegulatedPlantSafetyTest {

    private static final double EPSILON = 1e-12;

    @Test
    public void lowerLevelRegulatedPlantsSubmitNormalizedPowerAndPreserveRawAliases() {
        ScalarTarget positionTarget = ScalarTarget.held(10.0);
        RecordingPowerOutput positionOut = new RecordingPowerOutput();
        Plant position = Plants.positionFromPower(
                positionOut,
                PlantTargets.exact(positionTarget),
                positionTarget,
                PlantTargetGuards.none(),
                clock -> 10.0,
                new FixedRegulator(1.25),
                0.0);

        position.update(new ManualLoopClock().clock());

        assertEquals(1.0, positionOut.commanded, 0.0);
        assertEquals(10.0, position.getAppliedTarget(), 0.0);
        assertTrue(position.atTarget());
        assertTrue(position.atTarget(10.0));
        CapturingDebugSink positionDebug = debug(position, "position");
        assertEquals(1.25, number(positionDebug, "position.output"), 0.0);
        assertEquals(1.25, number(positionDebug, "position.regulatorOutput"), 0.0);
        assertEquals(1.0, number(positionDebug, "position.normalizedPowerCommand"), 0.0);
        assertEquals("SATURATED_AND_SUBMITTED",
                positionDebug.data.get("position.regulatedPowerStatus"));

        ScalarTarget velocityTarget = ScalarTarget.held(20.0);
        RecordingPowerOutput velocityOut = new RecordingPowerOutput();
        Plant velocity = Plants.velocityFromPower(
                velocityOut,
                PlantTargets.exact(velocityTarget),
                velocityTarget,
                PlantTargetGuards.none(),
                clock -> 20.0,
                new FixedRegulator(-1.25),
                0.0);

        velocity.update(new ManualLoopClock().clock());

        assertEquals(-1.0, velocityOut.commanded, 0.0);
        assertTrue(velocity.atTarget());
        CapturingDebugSink velocityDebug = debug(velocity, "velocity");
        assertEquals(-1.25, number(velocityDebug, "velocity.output"), 0.0);
        assertEquals(-1.25, number(velocityDebug, "velocity.regulatorOutput"), 0.0);
        assertEquals(-1.0, number(velocityDebug, "velocity.normalizedPowerCommand"), 0.0);
    }

    @Test
    public void mappedRegulatedPlantsSubmitNormalizedPowerAndPreserveRawAliases() {
        ScalarTarget positionTarget = ScalarTarget.held(10.0);
        RecordingPowerOutput positionOut = new RecordingPowerOutput();
        MappedPositionPlant position = MappedPositionPlant.regulated(
                positionOut, clock -> 10.0, new FixedRegulator(1.25))
                .positionTolerance(0.0)
                .targetedBy(positionTarget)
                .build();

        position.update(new ManualLoopClock().clock());

        assertEquals(1.0, positionOut.commanded, 0.0);
        assertTrue(position.atTarget());
        CapturingDebugSink positionDebug = debug(position, "position");
        assertEquals(1.25, number(positionDebug, "position.lastRegulatorOutput"), 0.0);
        assertEquals(1.25, number(positionDebug, "position.regulatorOutput"), 0.0);
        assertEquals(1.0, number(positionDebug, "position.normalizedPowerCommand"), 0.0);

        ScalarTarget velocityTarget = ScalarTarget.held(20.0);
        RecordingPowerOutput velocityOut = new RecordingPowerOutput();
        MappedVelocityPlant velocity = MappedVelocityPlant.regulated(
                velocityOut, clock -> 20.0, new FixedRegulator(-1.25))
                .velocityTolerance(0.0)
                .targetedBy(velocityTarget)
                .build();

        velocity.update(new ManualLoopClock().clock());

        assertEquals(-1.0, velocityOut.commanded, 0.0);
        assertTrue(velocity.atTarget());
        CapturingDebugSink velocityDebug = debug(velocity, "velocity");
        assertEquals(-1.25, number(velocityDebug, "velocity.regulatorOutput"), 0.0);
        assertEquals(-1.0, number(velocityDebug, "velocity.normalizedPowerCommand"), 0.0);
    }

    @Test
    public void everyRegulatedPlantPathRejectsNonFiniteOutputAndFailStops() {
        ScalarTarget lowerPositionTarget = ScalarTarget.held(10.0);
        ScalarTarget lowerVelocityTarget = ScalarTarget.held(10.0);
        ScalarTarget mappedPositionTarget = ScalarTarget.held(10.0);
        ScalarTarget mappedVelocityTarget = ScalarTarget.held(10.0);
        RecordingPowerOutput[] outputs = {
                new RecordingPowerOutput(),
                new RecordingPowerOutput(),
                new RecordingPowerOutput(),
                new RecordingPowerOutput()
        };
        Plant[] plants = {
                Plants.positionFromPower(outputs[0], PlantTargets.exact(lowerPositionTarget),
                        lowerPositionTarget, PlantTargetGuards.none(), clock -> 10.0,
                        new FixedRegulator(Double.NaN), 0.0),
                Plants.velocityFromPower(outputs[1], PlantTargets.exact(lowerVelocityTarget),
                        lowerVelocityTarget, PlantTargetGuards.none(), clock -> 10.0,
                        new FixedRegulator(Double.NaN), 0.0),
                MappedPositionPlant.regulated(outputs[2], clock -> 10.0,
                                new FixedRegulator(Double.NaN))
                        .positionTolerance(0.0)
                        .targetedBy(mappedPositionTarget)
                        .build(),
                MappedVelocityPlant.regulated(outputs[3], clock -> 10.0,
                                new FixedRegulator(Double.NaN))
                        .velocityTolerance(0.0)
                        .targetedBy(mappedVelocityTarget)
                        .build()
        };

        for (int i = 0; i < plants.length; i++) {
            try {
                plants[i].update(new ManualLoopClock().clock());
                fail("Expected path " + i + " to reject a non-finite regulator result");
            } catch (IllegalStateException expected) {
                assertTrue(expected.getMessage().contains("non-finite power"));
            }
            assertEquals(0, outputs[i].setCalls);
            assertEquals(1, outputs[i].stopCalls);
            assertEquals(0.0, outputs[i].commanded, 0.0);
            assertFalse(plants[i].atTarget());
            assertFalse(plants[i].atTarget(10.0));
            assertEquals(PlantTargetStatus.Kind.STOPPED, plants[i].getTargetStatus().kind());
        }
    }

    @Test
    public void successfulFailStopClearsCompletionAndAppliesVelocityStopSemantics() {
        RuntimeException regulatorFailure = new IllegalArgumentException("regulator failed");
        SequencedRegulator regulator = new SequencedRegulator(0.25, regulatorFailure);
        RecordingPowerOutput output = new RecordingPowerOutput();
        ScalarTarget target = ScalarTarget.held(20.0);
        MappedVelocityPlant plant = MappedVelocityPlant.regulated(
                output, clock -> 20.0, regulator)
                .velocityTolerance(0.0)
                .targetedBy(target)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        plant.update(clock.clock());
        assertTrue(plant.atTarget());
        assertTrue(plant.atTarget(20.0));

        try {
            plant.update(clock.nextCycle(0.02));
            fail("Expected regulator failure");
        } catch (RuntimeException actual) {
            assertSame(regulatorFailure, actual);
        }

        assertEquals(1, output.stopCalls);
        assertEquals(1, regulator.resetCalls);
        assertEquals(0.0, output.commanded, 0.0);
        assertEquals(0.0, plant.getAppliedTarget(), 0.0);
        assertEquals(PlantTargetStatus.Kind.STOPPED, plant.getTargetStatus().kind());
        assertFalse(plant.getTargetPlan().hasTarget());
        assertFalse(plant.atTarget());
        assertFalse(plant.atTarget(20.0));
    }

    @Test
    public void failedFailStopPreservesPriorPublicTargetFactsButInvalidatesCompletion() {
        RuntimeException regulatorFailure = new IllegalArgumentException("regulator failed");
        RuntimeException stopFailure = new IllegalStateException("stop failed");
        SequencedRegulator regulator = new SequencedRegulator(0.25, regulatorFailure);
        RecordingPowerOutput output = new RecordingPowerOutput();
        output.stopFailure = stopFailure;
        ScalarTarget target = ScalarTarget.held(20.0);
        MappedVelocityPlant plant = MappedVelocityPlant.regulated(
                output, clock -> 20.0, regulator)
                .velocityTolerance(0.0)
                .targetedBy(target)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        plant.update(clock.clock());
        assertTrue(plant.atTarget());

        try {
            plant.update(clock.nextCycle(0.02));
            fail("Expected regulator failure");
        } catch (RuntimeException actual) {
            assertSame(regulatorFailure, actual);
            assertEquals(1, actual.getSuppressed().length);
            assertSame(stopFailure, actual.getSuppressed()[0]);
        }

        assertEquals(20.0, plant.getAppliedTarget(), 0.0);
        assertEquals(PlantTargetStatus.Kind.ACCEPTED, plant.getTargetStatus().kind());
        assertTrue(plant.getTargetPlan().hasTarget());
        assertEquals(20.0, plant.getTargetPlan().target(), 0.0);
        assertFalse(plant.atTarget());
        assertFalse(plant.atTarget(20.0));
        CapturingDebugSink debug = debug(plant, "velocity");
        assertTrue(Double.isNaN(number(debug, "velocity.normalizedPowerCommand")));
        assertEquals("REGULATOR_FAILED_STOP_FAILED_RESET_SUCCEEDED",
                debug.data.get("velocity.regulatedPowerStatus"));
    }

    @Test
    public void successfulOutputStopAppliesPlantStopEvenWhenRegulatorResetFails() {
        RuntimeException resetFailure = new IllegalStateException("reset failed");
        FixedRegulator regulator = new FixedRegulator(0.25);
        RecordingPowerOutput output = new RecordingPowerOutput();
        MappedVelocityPlant plant = MappedVelocityPlant.regulated(
                output, clock -> 20.0, regulator)
                .velocityTolerance(0.0)
                .targetedBy(ScalarTarget.held(20.0))
                .build();
        plant.update(new ManualLoopClock().clock());
        assertTrue(plant.atTarget());
        regulator.resetFailure = resetFailure;

        try {
            plant.stop();
            fail("Expected regulator reset failure");
        } catch (RuntimeException actual) {
            assertSame(resetFailure, actual);
        }

        assertEquals(1, output.stopCalls);
        assertEquals(0.0, output.commanded, 0.0);
        assertEquals(0.0, plant.getAppliedTarget(), 0.0);
        assertEquals(PlantTargetStatus.Kind.STOPPED, plant.getTargetStatus().kind());
        assertFalse(plant.getTargetPlan().hasTarget());
        assertFalse(plant.atTarget());
        assertFalse(plant.atTarget(20.0));
        CapturingDebugSink debug = debug(plant, "velocity");
        assertEquals(0.0, number(debug, "velocity.normalizedPowerCommand"), 0.0);
        assertEquals("STOP_SUBMITTED_RESET_FAILED",
                debug.data.get("velocity.regulatedPowerStatus"));
    }

    @Test
    public void outerPolicyLimiterFailureFailStopsThePriorPlantCommand() {
        final int[] updates = {0};
        ScalarRegulator constrained = ScalarRegulators.outputLimited(
                (setpoint, measurement, clock) -> updates[0]++ == 0 ? 0.4 : Double.NaN,
                0.0,
                0.65);
        RecordingPowerOutput output = new RecordingPowerOutput();
        MappedVelocityPlant plant = MappedVelocityPlant.regulated(
                output, clock -> 20.0, constrained)
                .velocityTolerance(0.0)
                .targetedBy(ScalarTarget.held(20.0))
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        plant.update(clock.clock());
        assertEquals(0.4, output.commanded, 0.0);

        try {
            plant.update(clock.nextCycle(0.02));
            fail("Expected outputLimited to reject its non-finite inner result");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("non-finite"));
        }

        assertEquals(1, output.stopCalls);
        assertEquals(0.0, output.commanded, 0.0);
        assertEquals(PlantTargetStatus.Kind.STOPPED, plant.getTargetStatus().kind());
        assertFalse(plant.atTarget());
        assertFalse(plant.atTarget(20.0));
    }

    @Test
    public void pidfLimitsPreserveNonFiniteMeasurementForPlantFailStop() {
        PidfRegulator pidf = ScalarRegulators.pidf(1.0, 0.0, 0.0, 0.1)
                .setIntegralLimits(-0.5, 0.5)
                .setPidOutputLimits(-1.0, 1.0);
        ScalarRegulator constrained = ScalarRegulators.outputLimited(pidf, 0.0, 0.65);
        RecordingPowerOutput output = new RecordingPowerOutput();
        MappedVelocityPlant plant = MappedVelocityPlant.regulated(
                output, clock -> Double.NEGATIVE_INFINITY, constrained)
                .velocityTolerance(0.0)
                .targetedBy(ScalarTarget.held(100.0))
                .build();

        try {
            plant.update(new ManualLoopClock().clock());
            fail("Expected non-finite PIDF measurement math to fail closed");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("non-finite"));
        }

        assertEquals(0, output.setCalls);
        assertEquals(1, output.stopCalls);
        assertEquals(0.0, output.commanded, 0.0);
        assertEquals(PlantTargetStatus.Kind.STOPPED, plant.getTargetStatus().kind());
        assertFalse(plant.atTarget());
        assertFalse(plant.atTarget(100.0));

        CapturingDebugSink pidfDebug = new CapturingDebugSink();
        pidf.debugDump(pidfDebug, "pidf");
        assertTrue(Double.isNaN(number(pidfDebug, "pidf.lastSetpoint")));
        assertTrue(Double.isNaN(number(pidfDebug, "pidf.lastOutput")));
    }

    @Test
    public void mappedPositionStopDeduplicatesSharedOutputAndStillAttemptsDistinctOutput() {
        RecordingPowerOutput shared = new RecordingPowerOutput();
        MappedPositionPlant sharedPlant = MappedPositionPlant.regulated(
                shared, clock -> 0.0, new FixedRegulator(0.0))
                .searchPowerOutput(shared)
                .targetedBy(ScalarTarget.held(0.0))
                .build();

        sharedPlant.stop();

        assertEquals(1, shared.stopCalls);

        RuntimeException regulatedStopFailure = new IllegalStateException("regulated stop failed");
        RecordingPowerOutput regulated = new RecordingPowerOutput();
        regulated.stopFailure = regulatedStopFailure;
        RecordingPowerOutput search = new RecordingPowerOutput();
        MappedPositionPlant distinctPlant = MappedPositionPlant.regulated(
                regulated, clock -> 5.0, new FixedRegulator(0.25))
                .positionTolerance(0.0)
                .searchPowerOutput(search)
                .targetedBy(ScalarTarget.held(5.0))
                .build();
        distinctPlant.update(new ManualLoopClock().clock());

        try {
            distinctPlant.stop();
            fail("Expected regulated output stop failure");
        } catch (RuntimeException actual) {
            assertSame(regulatedStopFailure, actual);
        }

        assertEquals(1, regulated.stopCalls);
        assertEquals(1, search.stopCalls);
        assertEquals(5.0, distinctPlant.getAppliedTarget(), 0.0);
        assertEquals(PlantTargetStatus.Kind.ACCEPTED, distinctPlant.getTargetStatus().kind());
        assertTrue(distinctPlant.getTargetPlan().hasTarget());
        assertFalse(distinctPlant.atTarget());
        assertFalse(distinctPlant.atTarget(5.0));

        RecordingPowerOutput successfulRegulated = new RecordingPowerOutput();
        RecordingPowerOutput failingSearch = new RecordingPowerOutput();
        RuntimeException searchStopFailure = new IllegalStateException("search stop failed");
        failingSearch.stopFailure = searchStopFailure;
        MappedPositionPlant searchFailurePlant = MappedPositionPlant.regulated(
                successfulRegulated, clock -> 6.0, new FixedRegulator(0.3))
                .positionTolerance(0.0)
                .searchPowerOutput(failingSearch)
                .targetedBy(ScalarTarget.held(6.0))
                .build();
        searchFailurePlant.update(new ManualLoopClock().clock());

        try {
            searchFailurePlant.stop();
            fail("Expected search output stop failure");
        } catch (RuntimeException actual) {
            assertSame(searchStopFailure, actual);
            assertEquals(0, actual.getSuppressed().length);
        }

        assertEquals(1, successfulRegulated.stopCalls);
        assertEquals(1, failingSearch.stopCalls);
        assertEquals(6.0, searchFailurePlant.getAppliedTarget(), 0.0);
        assertEquals(PlantTargetStatus.Kind.ACCEPTED,
                searchFailurePlant.getTargetStatus().kind());
        assertTrue(searchFailurePlant.getTargetPlan().hasTarget());
        assertFalse(searchFailurePlant.atTarget());
        assertFalse(searchFailurePlant.atTarget(6.0));
        CapturingDebugSink searchFailureDebug = debug(searchFailurePlant, "position");
        assertTrue(Double.isNaN(number(
                searchFailureDebug, "position.normalizedPowerCommand")));
        assertEquals("STOP_FAILED_RESET_SUCCEEDED",
                searchFailureDebug.data.get("position.regulatedPowerStatus"));
    }

    private static CapturingDebugSink debug(Plant plant, String prefix) {
        CapturingDebugSink debug = new CapturingDebugSink();
        plant.debugDump(debug, prefix);
        return debug;
    }

    private static double number(CapturingDebugSink debug, String key) {
        Object value = debug.data.get(key);
        assertTrue(key + " must contain a number", value instanceof Number);
        return ((Number) value).doubleValue();
    }

    private static class RecordingPowerOutput implements PowerOutput {
        private double commanded = Double.NaN;
        private int setCalls;
        private int stopCalls;
        private RuntimeException stopFailure;

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
            if (stopFailure != null) throw stopFailure;
            commanded = 0.0;
        }
    }

    private static class FixedRegulator implements ScalarRegulator {
        private final double output;
        int resetCalls;
        private RuntimeException resetFailure;

        FixedRegulator(double output) {
            this.output = output;
        }

        @Override
        public double update(double setpoint, double measurement, LoopClock clock) {
            return output;
        }

        @Override
        public void reset() {
            resetCalls++;
            if (resetFailure != null) throw resetFailure;
        }
    }

    private static final class SequencedRegulator extends FixedRegulator {
        private final RuntimeException failure;
        private int updateCalls;

        SequencedRegulator(double firstOutput, RuntimeException failure) {
            super(firstOutput);
            this.failure = failure;
        }

        @Override
        public double update(double setpoint, double measurement, LoopClock clock) {
            if (updateCalls++ > 0) throw failure;
            return super.update(setpoint, measurement, clock);
        }
    }

    private static final class CapturingDebugSink implements DebugSink {
        private final Map<String, Object> data = new LinkedHashMap<>();

        @Override
        public DebugSink addData(String key, Object value) {
            data.put(key, value);
            return this;
        }

        @Override
        public DebugSink addLine(String text) {
            return this;
        }
    }
}
