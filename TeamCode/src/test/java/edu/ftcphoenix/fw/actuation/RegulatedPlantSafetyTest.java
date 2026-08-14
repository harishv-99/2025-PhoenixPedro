package edu.ftcphoenix.fw.actuation;

import org.junit.Test;

import java.util.LinkedHashMap;
import java.util.Map;

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
    public void regulatedBranchesSubmitNormalizedPowerAndPreserveRawDiagnostics() {
        ScalarTarget positionTarget = ScalarTarget.create(10.0);
        RecordingPowerOutput positionOut = new RecordingPowerOutput();
        PositionPlant position = Plants.fromOutputs()
                .regulatedPosition(positionOut, clock -> 10.0)
                .nonPeriodic()
                .unbounded()
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(0.0)
                .controlFromCustomRegulator(new FixedRegulator(1.25))
                .targetFromResolver(PlantTargets.exact(positionTarget))
                .build();

        position.update(new ManualLoopClock().clock());

        assertEquals(1.0, positionOut.commanded, 0.0);
        assertEquals(10.0, position.getAppliedTarget(), 0.0);
        assertTrue(position.atTarget());
        assertTrue(position.atTarget(10.0));
        CapturingDebugSink positionDebug = debug(position, "position");
        assertEquals(1.25, number(positionDebug, "position.lastRegulatorOutput"), 0.0);
        assertEquals(1.25, number(positionDebug, "position.regulatorOutput"), 0.0);
        assertEquals(1.0, number(positionDebug, "position.normalizedPowerCommand"), 0.0);
        assertEquals("SATURATED_AND_SUBMITTED",
                positionDebug.data.get("position.regulatedPowerStatus"));

        ScalarTarget velocityTarget = ScalarTarget.create(20.0);
        RecordingPowerOutput velocityOut = new RecordingPowerOutput();
        Plant velocity = Plants.fromOutputs()
                .regulatedVelocity(velocityOut, clock -> 20.0)
                .unbounded()
                .nativeUnits()
                .velocityTolerance(0.0)
                .controlFromCustomRegulator(new FixedRegulator(-1.25))
                .targetFromResolver(PlantTargets.exact(velocityTarget))
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
        RecordingPowerOutput[] outputs = {
                new RecordingPowerOutput(),
                new RecordingPowerOutput()
        };
        Plant[] plants = {
                Plants.fromOutputs()
                        .regulatedPosition(outputs[0], clock -> 10.0)
                        .nonPeriodic()
                        .unbounded()
                        .nativeUnits()
                        .alreadyReferenced()
                        .positionTolerance(0.0)
                        .controlFromCustomRegulator(new FixedRegulator(Double.NaN))
                        .targetFromNewCommand(10.0)
                        .build(),
                Plants.fromOutputs()
                        .regulatedVelocity(outputs[1], clock -> 10.0)
                        .unbounded()
                        .nativeUnits()
                        .velocityTolerance(0.0)
                        .controlFromCustomRegulator(new FixedRegulator(Double.NaN))
                        .targetFromNewCommand(10.0)
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
        ScalarTarget target = ScalarTarget.create(20.0);
        Plant plant = Plants.fromOutputs()
                .regulatedVelocity(output, clock -> 20.0)
                .unbounded()
                .nativeUnits()
                .velocityTolerance(0.0)
                .controlFromCustomRegulator(regulator)
                .targetFromResolver(PlantTargets.exact(target))
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
        assertFalse(plant.getTargetResolution().hasTarget());
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
        ScalarTarget target = ScalarTarget.create(20.0);
        Plant plant = Plants.fromOutputs()
                .regulatedVelocity(output, clock -> 20.0)
                .unbounded()
                .nativeUnits()
                .velocityTolerance(0.0)
                .controlFromCustomRegulator(regulator)
                .targetFromResolver(PlantTargets.exact(target))
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
        assertTrue(plant.getTargetResolution().hasTarget());
        assertEquals(20.0, plant.getTargetResolution().target(), 0.0);
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
        Plant plant = Plants.fromOutputs()
                .regulatedVelocity(output, clock -> 20.0)
                .unbounded()
                .nativeUnits()
                .velocityTolerance(0.0)
                .controlFromCustomRegulator(regulator)
                .targetFromNewCommand(20.0)
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
        assertFalse(plant.getTargetResolution().hasTarget());
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
        Plant plant = Plants.fromOutputs()
                .regulatedVelocity(output, clock -> 20.0)
                .unbounded()
                .nativeUnits()
                .velocityTolerance(0.0)
                .controlFromCustomRegulator(constrained)
                .targetFromNewCommand(20.0)
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
    public void standardControlLimitsPreserveNonFiniteMeasurementForPlantFailStop() {
        RecordingPowerOutput output = new RecordingPowerOutput();
        Plant plant = Plants.fromOutputs()
                .regulatedVelocity(output, clock -> Double.NEGATIVE_INFINITY)
                .unbounded()
                .nativeUnits()
                .velocityTolerance(0.0)
                .setpointFromAppliedTarget()
                .feedbackFromPid(1.0, 0.0, 0.0)
                .feedbackIntegralLimitedTo(-0.5, 0.5)
                .feedbackOutputLimitedTo(-1.0, 1.0)
                .feedforwardFromMotion(0.0, 0.1)
                .outputPowerLimitedTo(0.0, 0.65)
                .targetFromNewCommand(100.0)
                .build();

        try {
            plant.update(new ManualLoopClock().clock());
            fail("Expected non-finite standard-control measurement math to fail closed");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("must be finite"));
        }

        assertEquals(0, output.setCalls);
        assertEquals(1, output.stopCalls);
        assertEquals(0.0, output.commanded, 0.0);
        assertEquals(PlantTargetStatus.Kind.STOPPED, plant.getTargetStatus().kind());
        assertFalse(plant.atTarget());
        assertFalse(plant.atTarget(100.0));

    }

    @Test
    public void regulatedPositionStopUsesItsOwnedPowerOutputOnce() {
        RecordingPowerOutput shared = new RecordingPowerOutput();
        PositionPlant sharedPlant = Plants.fromOutputs()
                .regulatedPosition(shared, clock -> 0.0)
                .nonPeriodic()
                .unbounded()
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(0.0)
                .controlFromCustomRegulator(new FixedRegulator(0.0))
                .targetFromNewCommand(0.0)
                .build();

        sharedPlant.stop();

        assertEquals(1, shared.stopCalls);
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
