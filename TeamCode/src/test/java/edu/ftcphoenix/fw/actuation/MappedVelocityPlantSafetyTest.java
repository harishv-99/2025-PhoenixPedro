package edu.ftcphoenix.fw.actuation;

import org.junit.Test;

import java.util.LinkedHashMap;
import java.util.Map;

import edu.ftcphoenix.fw.core.control.ScalarRegulator;
import edu.ftcphoenix.fw.core.control.ScalarRegulators;
import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.hal.VelocityOutput;
import edu.ftcphoenix.fw.core.source.ScalarTarget;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies final target safety for mapped velocity Plants. */
public final class MappedVelocityPlantSafetyTest {

    private static final double EPSILON = 1e-12;

    @Test
    public void targetGuardRejectsFallbackOutsideDeclaredRange() {
        try {
            Plants.fromOutputs()
                    .deviceManagedVelocity(new RecordingVelocityOutput(), clock -> 0.0)
                    .bounded(10.0, 20.0)
                    .nativeUnits()
                    .velocityTolerance(0.0)
                    .targetGuards()
                    .fallbackTargetUnless("mechanismClear", clock -> false, 5.0)
                    .doneTargetGuards()
                    .targetFromNewCommand(15.0)
                    .build();
            fail("Expected an out-of-range fallback to be rejected");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("mechanismClear"));
            assertTrue(expected.getMessage().contains("5.0"));
            assertTrue(expected.getMessage().contains("10.0"));
            assertTrue(expected.getMessage().contains("20.0"));
            assertTrue(expected.getMessage().contains("declared Plant range"));
        }
    }

    @Test
    public void boundedRejectsNonFiniteEndpoints() {
        try {
            Plants.fromOutputs()
                    .deviceManagedVelocity(new RecordingVelocityOutput(), clock -> 0.0)
                    .bounded(Double.POSITIVE_INFINITY, 20.0);
            fail("Expected non-finite velocity bounds to be rejected");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("finite"));
        }
    }

    @Test
    public void boundedRejectsInvertedRange() {
        try {
            Plants.fromOutputs()
                    .deviceManagedVelocity(new RecordingVelocityOutput(), clock -> 0.0)
                    .bounded(20.0, 10.0);
            fail("Expected inverted velocity bounds to be rejected");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("min"));
            assertTrue(expected.getMessage().contains("max"));
        }
    }

    @Test
    public void inRangeFallbackRetainsFallbackStatus() {
        RecordingVelocityOutput output = new RecordingVelocityOutput();
        ScalarTarget target = ScalarTarget.create(18.0);
        Plant plant = Plants.fromOutputs()
                .deviceManagedVelocity(output, clock -> 0.0)
                .bounded(10.0, 20.0)
                .nativeUnits()
                .velocityTolerance(0.0)
                .targetGuards()
                .fallbackTargetUnless("mechanismClear", clock -> false, 12.0)
                .doneTargetGuards()
                .targetFromResolver(PlantTargets.exact(target))
                .build();

        plant.update(new ManualLoopClock().clock());

        assertEquals(18.0, plant.getRequestedTarget(), EPSILON);
        assertEquals(12.0, plant.getAppliedTarget(), EPSILON);
        assertEquals(12.0, output.getCommandedVelocity(), EPSILON);
        assertEquals(PlantTargetStatus.Kind.FALLBACK_ACTIVE, plant.getTargetStatus().kind());
        assertTrue(plant.getTargetStatus().message().contains("mechanismClear"));
    }

    @Test
    public void tinyBoundaryClampIsNeverReportedAccepted() {
        RecordingVelocityOutput output = new RecordingVelocityOutput();
        double request = 20.0 + 1.0e-10;
        Plant plant = Plants.fromOutputs()
                .deviceManagedVelocity(output, clock -> 0.0)
                .bounded(10.0, 20.0)
                .nativeUnits()
                .velocityTolerance(0.0)
                .targetFromResolver(PlantTargets.exact(request))
                .build();

        plant.update(new ManualLoopClock().clock());

        assertEquals(request, plant.getRequestedTarget(), 0.0);
        assertEquals(20.0, plant.getAppliedTarget(), EPSILON);
        assertEquals(20.0, output.getCommandedVelocity(), EPSILON);
        assertEquals(PlantTargetStatus.Kind.CLAMPED_TO_RANGE, plant.getTargetStatus().kind());
    }

    @Test
    public void signedZeroInsideRangeRemainsAccepted() {
        Plant plant = Plants.fromOutputs()
                .deviceManagedVelocity(new RecordingVelocityOutput(), clock -> 0.0)
                .bounded(0.0, 20.0)
                .nativeUnits()
                .velocityTolerance(0.0)
                .targetFromNewCommand(-0.0)
                .build();

        plant.update(new ManualLoopClock().clock());

        assertEquals(PlantTargetStatus.Kind.ACCEPTED, plant.getTargetStatus().kind());
    }

    @Test
    public void initialHoldIsClampedAndLimiterContinuesFromActualCommand() {
        RecordingVelocityOutput output = new RecordingVelocityOutput();
        ScalarTarget target = ScalarTarget.create(20.0);
        final boolean[] mechanismClear = {false};
        Plant plant = Plants.fromOutputs()
                .deviceManagedVelocity(output, clock -> 0.0)
                .bounded(10.0, 20.0)
                .nativeUnits()
                .velocityTolerance(0.0)
                .targetGuards()
                .maxTargetRate(1.0)
                .holdLastTargetUnless("mechanismClear", clock -> mechanismClear[0])
                .doneTargetGuards()
                .targetFromResolver(PlantTargets.exact(target))
                .build();
        ManualLoopClock clock = new ManualLoopClock();

        plant.update(clock.clock());

        assertEquals(10.0, plant.getAppliedTarget(), EPSILON);
        assertEquals(10.0, output.getCommandedVelocity(), EPSILON);
        assertEquals(PlantTargetStatus.Kind.CLAMPED_TO_RANGE, plant.getTargetStatus().kind());
        assertTrue(plant.getTargetStatus().message().contains("HOLDING_LAST"));

        mechanismClear[0] = true;
        plant.update(clock.nextCycle(1.0));

        assertEquals(11.0, plant.getAppliedTarget(), EPSILON);
        assertEquals(11.0, output.getCommandedVelocity(), EPSILON);
        assertEquals(PlantTargetStatus.Kind.RATE_LIMITED, plant.getTargetStatus().kind());
    }

    @Test
    public void nonFiniteLimiterResultRetainsPriorCommandAndRecovers() {
        RecordingVelocityOutput output = new RecordingVelocityOutput();
        ScalarTarget target = ScalarTarget.create(Double.MAX_VALUE);
        Plant plant = Plants.fromOutputs()
                .deviceManagedVelocity(output, clock -> 0.0)
                .unbounded()
                .nativeUnits()
                .velocityTolerance(0.0)
                .targetGuards()
                .maxTargetRate(Double.MAX_VALUE)
                .doneTargetGuards()
                .targetFromResolver(PlantTargets.exact(target))
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        plant.update(clock.clock());
        double prior = plant.getAppliedTarget();

        target.set(-Double.MAX_VALUE);
        plant.update(clock.nextCycle(2.0));

        assertTrue(Double.isFinite(plant.getAppliedTarget()));
        assertEquals(prior, plant.getAppliedTarget(), 0.0);
        assertEquals(prior, output.getCommandedVelocity(), 0.0);
        assertEquals(PlantTargetStatus.Kind.TARGET_UNAVAILABLE, plant.getTargetStatus().kind());
        assertTrue(plant.getTargetStatus().message().contains("non-finite"));

        target.set(0.0);
        plant.update(clock.nextCycle(1.0));

        assertEquals(0.0, plant.getAppliedTarget(), EPSILON);
        assertEquals(0.0, output.getCommandedVelocity(), EPSILON);
        assertEquals(PlantTargetStatus.Kind.ACCEPTED, plant.getTargetStatus().kind());
    }

    @Test
    public void nonFiniteClockCannotBypassRateLimitDuringRecovery() {
        RecordingVelocityOutput output = new RecordingVelocityOutput();
        ScalarTarget target = ScalarTarget.create(10.0);
        Plant plant = Plants.fromOutputs()
                .deviceManagedVelocity(output, clock -> 0.0)
                .unbounded()
                .nativeUnits()
                .velocityTolerance(0.0)
                .targetGuards()
                .maxTargetRate(1.0)
                .doneTargetGuards()
                .targetFromResolver(PlantTargets.exact(target))
                .build();
        LoopClock clock = new LoopClock();
        clock.reset(0.0);
        plant.update(clock);

        target.set(20.0);
        clock.update(Double.NaN);
        plant.update(clock);
        assertEquals(10.0, plant.getAppliedTarget(), EPSILON);
        assertEquals(10.0, output.getCommandedVelocity(), EPSILON);
        assertEquals(PlantTargetStatus.Kind.RATE_LIMITED, plant.getTargetStatus().kind());

        clock.update(1.0);
        plant.update(clock);
        assertEquals(10.0, plant.getAppliedTarget(), EPSILON);
        assertEquals(10.0, output.getCommandedVelocity(), EPSILON);
        assertEquals(PlantTargetStatus.Kind.RATE_LIMITED, plant.getTargetStatus().kind());

        clock.update(2.0);
        plant.update(clock);
        assertEquals(11.0, plant.getAppliedTarget(), EPSILON);
        assertEquals(11.0, output.getCommandedVelocity(), EPSILON);
        assertEquals(PlantTargetStatus.Kind.RATE_LIMITED, plant.getTargetStatus().kind());
    }

    @Test
    public void explicitRegulatorLimitBoundsPowerWithoutChangingPlantTargetUnits() {
        RecordingPowerOutput output = new RecordingPowerOutput();
        ScalarTarget targetRpm = ScalarTarget.create(4200.0);
        ScalarRegulator constrained = ScalarRegulators.outputLimited(
                (setpoint, measurement, clock) -> 0.85,
                0.0,
                0.65);
        Plant plant = Plants.fromOutputs()
                .regulatedVelocity(output, clock -> 2000.0, constrained)
                .bounded(0.0, 6000.0)
                .scaleToNative(2.0)
                .velocityTolerance(0.0)
                .targetFromResolver(PlantTargets.exact(targetRpm))
                .build();

        plant.update(new ManualLoopClock().clock());

        assertEquals(4200.0, plant.getRequestedTarget(), 0.0);
        assertEquals(4200.0, plant.getAppliedTarget(), 0.0);
        assertEquals(1000.0, plant.getMeasurement(), 0.0);
        assertEquals(0.65, output.getCommandedPower(), 0.0);

        CapturingDebugSink debug = new CapturingDebugSink();
        plant.debugDump(debug, "shooter.flywheel");
        assertEquals(0.65, number(debug, "shooter.flywheel.regulatorOutput"), 0.0);
        assertEquals(0.85,
                number(debug, "shooter.flywheel.regulator.lastUnconstrainedOutput"), 0.0);
        assertEquals(0.65,
                number(debug, "shooter.flywheel.regulator.lastOutput"), 0.0);
        assertEquals(Boolean.TRUE,
                debug.data.get("shooter.flywheel.regulator.lastOutputLimited"));
    }

    private static double number(CapturingDebugSink debug, String key) {
        Object value = debug.data.get(key);
        assertTrue(key + " must contain a number", value instanceof Number);
        return ((Number) value).doubleValue();
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

    private static final class RecordingVelocityOutput implements VelocityOutput {
        private double commanded = Double.NaN;

        @Override
        public void setVelocity(double velocity) {
            commanded = velocity;
        }

        @Override
        public double getCommandedVelocity() {
            return commanded;
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
