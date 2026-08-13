package edu.ftcphoenix.fw.actuation;

import org.junit.Test;

import edu.ftcphoenix.fw.core.control.ScalarRegulator;
import edu.ftcphoenix.fw.core.hal.PositionOutput;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the Plant-owned one-heartbeat gate across every concrete Plant realization. */
public final class PlantUpdateCycleIntegrationTest {

    @Test
    public void sameCycleSuccessSkipsEveryRealizationBeforeResamplingOrRewriting() {
        CountingResolver powerResolver = new CountingResolver(0.4);
        RecordingPowerOutput powerOutput = new RecordingPowerOutput();
        Plant power = Plants.fromOutputs()
                .power(powerOutput)
                .targetFromResolver(powerResolver)
                .build();

        CountingResolver positionResolver = new CountingResolver(4.0);
        RecordingPositionOutput positionOutput = new RecordingPositionOutput();
        PositionPlant position = Plants.fromOutputs()
                .commandedPosition(positionOutput)
                .nonPeriodic()
                .bounded(-10.0, 10.0)
                .nativeUnits()
                .targetFromResolver(positionResolver)
                .build();

        CountingResolver velocityResolver = new CountingResolver(2.0);
        CountingScalarSource velocityFeedback = new CountingScalarSource(1.0);
        CountingRegulator velocityRegulator = new CountingRegulator(0.3);
        RecordingPowerOutput velocityOutput = new RecordingPowerOutput();
        Plant velocity = Plants.fromOutputs()
                .regulatedVelocity(velocityOutput, velocityFeedback)
                .bounded(-10.0, 10.0)
                .nativeUnits()
                .velocityTolerance(0.0)
                .controlFromCustomRegulator(velocityRegulator)
                .targetFromResolver(velocityResolver)
                .build();

        LoopClock sameCycle = new ManualLoopClock().clock();
        power.update(sameCycle);
        power.update(sameCycle);
        position.update(sameCycle);
        position.update(sameCycle);
        velocity.update(sameCycle);
        velocity.update(sameCycle);

        assertEquals(1, powerResolver.resolutions);
        assertEquals(1, powerOutput.writes);
        assertEquals(1, positionResolver.resolutions);
        assertEquals(1, positionOutput.writes);
        assertEquals(1, velocityFeedback.samples);
        assertEquals(1, velocityResolver.resolutions);
        assertEquals(1, velocityRegulator.updates);
        assertEquals(1, velocityOutput.writes);
    }

    @Test
    public void sameCycleFailureIsReplayedWithoutRepeatingCleanupInEveryRealization() {
        RuntimeException powerFailure = new IllegalStateException("power write failed");
        FailingPowerOutput powerOutput = new FailingPowerOutput(powerFailure);
        CountingResolver powerResolver = new CountingResolver(0.4);
        Plant power = Plants.fromOutputs()
                .power(powerOutput)
                .targetFromResolver(powerResolver)
                .build();

        RuntimeException positionFailure = new IllegalStateException("position write failed");
        FailingPositionOutput positionOutput = new FailingPositionOutput(positionFailure);
        CountingResolver positionResolver = new CountingResolver(4.0);
        PositionPlant position = Plants.fromOutputs()
                .commandedPosition(positionOutput)
                .nonPeriodic()
                .bounded(-10.0, 10.0)
                .nativeUnits()
                .targetFromResolver(positionResolver)
                .build();

        RuntimeException regulatorFailure = new IllegalStateException("regulator failed");
        CountingScalarSource velocityFeedback = new CountingScalarSource(1.0);
        FailingRegulator velocityRegulator = new FailingRegulator(regulatorFailure);
        RecordingPowerOutput velocityOutput = new RecordingPowerOutput();
        CountingResolver velocityResolver = new CountingResolver(2.0);
        Plant velocity = Plants.fromOutputs()
                .regulatedVelocity(velocityOutput, velocityFeedback)
                .bounded(-10.0, 10.0)
                .nativeUnits()
                .velocityTolerance(0.0)
                .controlFromCustomRegulator(velocityRegulator)
                .targetFromResolver(velocityResolver)
                .build();

        LoopClock sameCycle = new ManualLoopClock().clock();
        assertSame(powerFailure, expectRuntime(() -> power.update(sameCycle)));
        assertSame(powerFailure, expectRuntime(() -> power.update(sameCycle)));
        assertSame(positionFailure, expectRuntime(() -> position.update(sameCycle)));
        assertSame(positionFailure, expectRuntime(() -> position.update(sameCycle)));
        assertSame(regulatorFailure, expectRuntime(() -> velocity.update(sameCycle)));
        assertSame(regulatorFailure, expectRuntime(() -> velocity.update(sameCycle)));

        assertEquals(1, powerResolver.resolutions);
        assertEquals(1, powerOutput.writes);
        assertEquals(1, powerOutput.stops);
        assertEquals(1, positionResolver.resolutions);
        assertEquals(1, positionOutput.writes);
        assertEquals(1, positionOutput.stops);
        assertEquals(1, velocityFeedback.samples);
        assertEquals(1, velocityResolver.resolutions);
        assertEquals(1, velocityRegulator.updates);
        assertEquals(1, velocityRegulator.resets);
        assertEquals(1, velocityOutput.stops);
        assertEquals(0, velocityOutput.writes);
    }

    @Test
    public void reentrantHeartbeatFailsFastAndTheOuterAttemptRetainsThatFailure() {
        ManualLoopClock time = new ManualLoopClock();
        CountingResolver resolver = new CountingResolver(0.4);
        ReentrantPowerOutput output = new ReentrantPowerOutput(time.clock());
        Plant plant = Plants.fromOutputs()
                .power(output)
                .targetFromResolver(resolver)
                .build();
        output.plant = plant;

        RuntimeException first = expectRuntime(() -> plant.update(time.clock()));
        RuntimeException repeated = expectRuntime(() -> plant.update(time.clock()));

        assertTrue(first.getMessage(), first.getMessage().contains("reentrant"));
        assertSame(first, repeated);
        assertEquals(1, resolver.resolutions);
        assertEquals(1, output.writes);
        assertEquals(1, output.stops);
    }

    @Test
    public void plantLifecycleBindsOneClockIdentity() {
        CountingResolver resolver = new CountingResolver(0.4);
        RecordingPowerOutput output = new RecordingPowerOutput();
        Plant plant = Plants.fromOutputs()
                .power(output)
                .targetFromResolver(resolver)
                .build();
        ManualLoopClock first = new ManualLoopClock();
        ManualLoopClock second = new ManualLoopClock();
        plant.update(first.clock());

        RuntimeException failure = expectRuntime(() -> plant.update(second.clock()));

        assertTrue(failure.getMessage(), failure.getMessage().contains("different clock identity"));
        assertEquals(1, resolver.resolutions);
        assertEquals(1, output.writes);
    }

    private static RuntimeException expectRuntime(Runnable action) {
        try {
            action.run();
            fail("Expected RuntimeException");
            return null;
        } catch (RuntimeException expected) {
            return expected;
        }
    }

    private static final class CountingResolver implements PlantTargetResolver {
        private final double target;
        private int resolutions;

        private CountingResolver(double target) {
            this.target = target;
        }

        @Override
        public PlantTargetResolution resolve(PlantTargetContext context, LoopClock clock) {
            resolutions++;
            return PlantTargetResolution.exact(target, "test target");
        }
    }

    private static final class CountingScalarSource implements ScalarSource {
        private final double value;
        private int samples;

        private CountingScalarSource(double value) {
            this.value = value;
        }

        @Override
        public double getAsDouble(LoopClock clock) {
            samples++;
            return value;
        }
    }

    private static final class CountingRegulator implements ScalarRegulator {
        private final double output;
        private int updates;

        private CountingRegulator(double output) {
            this.output = output;
        }

        @Override
        public double update(double setpoint, double measurement, LoopClock clock) {
            updates++;
            return output;
        }
    }

    private static final class FailingRegulator implements ScalarRegulator {
        private final RuntimeException failure;
        private int updates;
        private int resets;

        private FailingRegulator(RuntimeException failure) {
            this.failure = failure;
        }

        @Override
        public double update(double setpoint, double measurement, LoopClock clock) {
            updates++;
            throw failure;
        }

        @Override
        public void reset() {
            resets++;
        }
    }

    private static class RecordingPowerOutput implements PowerOutput {
        private double commandedPower = Double.NaN;
        int writes;
        int stops;

        @Override
        public void setPower(double power) {
            writes++;
            commandedPower = power;
        }

        @Override
        public double getCommandedPower() {
            return commandedPower;
        }

        @Override
        public void stop() {
            stops++;
            commandedPower = 0.0;
        }
    }

    private static final class FailingPowerOutput extends RecordingPowerOutput {
        private final RuntimeException failure;

        private FailingPowerOutput(RuntimeException failure) {
            this.failure = failure;
        }

        @Override
        public void setPower(double power) {
            super.setPower(power);
            throw failure;
        }
    }

    private static class RecordingPositionOutput implements PositionOutput {
        private double commandedPosition = Double.NaN;
        int writes;
        int stops;

        @Override
        public void setPosition(double position) {
            writes++;
            commandedPosition = position;
        }

        @Override
        public double getCommandedPosition() {
            return commandedPosition;
        }

        @Override
        public void stop() {
            stops++;
        }
    }

    private static final class FailingPositionOutput extends RecordingPositionOutput {
        private final RuntimeException failure;

        private FailingPositionOutput(RuntimeException failure) {
            this.failure = failure;
        }

        @Override
        public void setPosition(double position) {
            super.setPosition(position);
            throw failure;
        }
    }

    private static final class ReentrantPowerOutput implements PowerOutput {
        private final LoopClock clock;
        private Plant plant;
        private int writes;
        private int stops;

        private ReentrantPowerOutput(LoopClock clock) {
            this.clock = clock;
        }

        @Override
        public void setPower(double power) {
            writes++;
            plant.update(clock);
        }

        @Override
        public double getCommandedPower() {
            return Double.NaN;
        }

        @Override
        public void stop() {
            stops++;
        }
    }
}
