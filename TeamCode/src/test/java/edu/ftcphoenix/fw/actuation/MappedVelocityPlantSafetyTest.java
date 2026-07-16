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
    public void buildRejectsFallbackOutsideDeclaredRange() {
        PlantTargetGuards guards = PlantTargetGuards.builder()
                .fallbackTargetUnless("mechanismClear", clock -> false, 5.0)
                .build();

        try {
            MappedVelocityPlant.velocityOutput(new RecordingVelocityOutput(), clock -> 0.0)
                    .range(ScalarRange.bounded(10.0, 20.0))
                    .targetGuards(guards)
                    .targetedBy(ScalarTarget.held(15.0))
                    .build();
            fail("Expected an out-of-range fallback to be rejected");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("mechanismClear"));
            assertTrue(expected.getMessage().contains("5.0"));
            assertTrue(expected.getMessage().contains("10.0"));
            assertTrue(expected.getMessage().contains("20.0"));
            assertTrue(expected.getMessage().contains("Choose a fallback inside"));
        }
    }

    @Test
    public void buildRejectsRangeWithoutFiniteCommand() {
        try {
            MappedVelocityPlant.velocityOutput(new RecordingVelocityOutput(), clock -> 0.0)
                    .range(ScalarRange.minOnly(Double.POSITIVE_INFINITY))
                    .targetedBy(ScalarTarget.held(15.0))
                    .build();
            fail("Expected a range with no finite command to be rejected");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("finite plant-unit command"));
        }
    }

    @Test
    public void invalidVelocityRangeHasApplicableBuildGuidance() {
        try {
            MappedVelocityPlant.velocityOutput(new RecordingVelocityOutput(), clock -> 0.0)
                    .range(ScalarRange.invalid("velocity range not configured"))
                    .targetedBy(ScalarTarget.held(15.0))
                    .build();
            fail("Expected an invalid configured range to be rejected");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("valid configured target range"));
            assertTrue(expected.getMessage().contains("velocity range not configured"));
            assertFalse(expected.getMessage().contains("needsReference"));
        }
    }

    @Test
    public void inRangeFallbackRetainsFallbackStatus() {
        RecordingVelocityOutput output = new RecordingVelocityOutput();
        ScalarTarget target = ScalarTarget.held(18.0);
        PlantTargetGuards guards = PlantTargetGuards.builder()
                .fallbackTargetUnless("mechanismClear", clock -> false, 12.0)
                .build();
        MappedVelocityPlant plant = MappedVelocityPlant.velocityOutput(output, clock -> 0.0)
                .range(ScalarRange.bounded(10.0, 20.0))
                .targetGuards(guards)
                .targetedBy(target)
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
        MappedVelocityPlant plant = MappedVelocityPlant.velocityOutput(output, clock -> 0.0)
                .range(ScalarRange.bounded(10.0, 20.0))
                .targetedBy(ScalarTarget.held(request))
                .build();

        plant.update(new ManualLoopClock().clock());

        assertEquals(request, plant.getRequestedTarget(), 0.0);
        assertEquals(20.0, plant.getAppliedTarget(), EPSILON);
        assertEquals(20.0, output.getCommandedVelocity(), EPSILON);
        assertEquals(PlantTargetStatus.Kind.CLAMPED_TO_RANGE, plant.getTargetStatus().kind());
    }

    @Test
    public void signedZeroInsideRangeRemainsAccepted() {
        MappedVelocityPlant plant = MappedVelocityPlant.velocityOutput(
                new RecordingVelocityOutput(), clock -> 0.0)
                .range(ScalarRange.bounded(0.0, 20.0))
                .targetedBy(ScalarTarget.held(-0.0))
                .build();

        plant.update(new ManualLoopClock().clock());

        assertEquals(PlantTargetStatus.Kind.ACCEPTED, plant.getTargetStatus().kind());
    }

    @Test
    public void initialHoldIsClampedAndLimiterContinuesFromActualCommand() {
        RecordingVelocityOutput output = new RecordingVelocityOutput();
        ScalarTarget target = ScalarTarget.held(20.0);
        final boolean[] mechanismClear = {false};
        PlantTargetGuards guards = PlantTargetGuards.builder()
                .maxTargetRate(1.0)
                .holdLastTargetUnless("mechanismClear", clock -> mechanismClear[0])
                .build();
        MappedVelocityPlant plant = MappedVelocityPlant.velocityOutput(output, clock -> 0.0)
                .range(ScalarRange.bounded(10.0, 20.0))
                .targetGuards(guards)
                .targetedBy(target)
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
        ScalarTarget target = ScalarTarget.held(Double.MAX_VALUE);
        PlantTargetGuards guards = PlantTargetGuards.builder()
                .maxTargetRate(Double.MAX_VALUE)
                .build();
        MappedVelocityPlant plant = MappedVelocityPlant.velocityOutput(output, clock -> 0.0)
                .targetGuards(guards)
                .targetedBy(target)
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
        ScalarTarget target = ScalarTarget.held(10.0);
        PlantTargetGuards guards = PlantTargetGuards.builder()
                .maxTargetRate(1.0)
                .build();
        MappedVelocityPlant plant = MappedVelocityPlant.velocityOutput(output, clock -> 0.0)
                .targetGuards(guards)
                .targetedBy(target)
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
        ScalarTarget targetRpm = ScalarTarget.held(4200.0);
        ScalarRegulator constrained = ScalarRegulators.outputLimited(
                (setpoint, measurement, clock) -> 0.85,
                0.0,
                0.65);
        MappedVelocityPlant plant = MappedVelocityPlant.regulated(
                output,
                clock -> 2000.0,
                constrained)
                .range(ScalarRange.bounded(0.0, 6000.0))
                .nativePerPlantUnit(2.0)
                .targetedBy(targetRpm)
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
