package edu.ftcphoenix.fw.actuation;

import org.junit.Test;

import edu.ftcphoenix.fw.core.hal.PositionOutput;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.source.ScalarTarget;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies final target safety for lower-level Plant factories. */
public final class PlantsTargetSafetyTest {

    private static final double EPSILON = 1e-12;

    @Test
    public void powerClampsFiniteRequestsBeforeTheOutputAndReportsIt() {
        ClampingPowerOutput output = new ClampingPowerOutput();
        ScalarTarget target = ScalarTarget.held(2.0);
        Plant plant = Plants.power(output, target);
        ManualLoopClock clock = new ManualLoopClock();

        plant.update(clock.clock());

        assertEquals(2.0, plant.getRequestedTarget(), EPSILON);
        assertEquals(1.0, plant.getAppliedTarget(), EPSILON);
        assertEquals(1.0, output.received, EPSILON);
        assertEquals(1.0, output.getCommandedPower(), EPSILON);
        assertEquals(PlantTargetStatus.Kind.CLAMPED_TO_RANGE, plant.getTargetStatus().kind());

        target.set(-2.0);
        plant.update(clock.nextCycle(0.02));

        assertEquals(-2.0, plant.getRequestedTarget(), EPSILON);
        assertEquals(-1.0, plant.getAppliedTarget(), EPSILON);
        assertEquals(-1.0, output.received, EPSILON);
        assertEquals(-1.0, output.getCommandedPower(), EPSILON);
        assertEquals(PlantTargetStatus.Kind.CLAMPED_TO_RANGE, plant.getTargetStatus().kind());
    }

    @Test
    public void normalizedPowerBoundariesAndInteriorRemainAccepted() {
        ClampingPowerOutput output = new ClampingPowerOutput();
        ScalarTarget target = ScalarTarget.held(-1.0);
        Plant plant = Plants.power(output, target);
        ManualLoopClock clock = new ManualLoopClock();
        double[] requests = {-1.0, 0.0, 0.35, 1.0};

        for (int i = 0; i < requests.length; i++) {
            target.set(requests[i]);
            if (i > 0) clock.nextCycle(0.02);
            plant.update(clock.clock());

            assertEquals(requests[i], plant.getRequestedTarget(), EPSILON);
            assertEquals(requests[i], plant.getAppliedTarget(), EPSILON);
            assertEquals(requests[i], output.received, EPSILON);
            assertEquals(PlantTargetStatus.Kind.ACCEPTED, plant.getTargetStatus().kind());
        }
    }

    @Test
    public void powerTargetSourceSeesNormalizedRange() {
        final ScalarRange[] seenRange = {null};
        PlantTargetSource source = (context, clock) -> {
            seenRange[0] = context.targetRange();
            return PlantTargetPlan.exact(0.0, "capture power range");
        };
        Plant plant = Plants.power(new ClampingPowerOutput(), source);

        plant.update(new ManualLoopClock().clock());

        assertTrue(seenRange[0].valid);
        assertEquals(-1.0, seenRange[0].minValue, EPSILON);
        assertEquals(1.0, seenRange[0].maxValue, EPSILON);
    }

    @Test
    public void unavailableNonFinitePowerSourceStartsAtNeutral() {
        ClampingPowerOutput output = new ClampingPowerOutput();
        Plant plant = Plants.power(output, ScalarTarget.held(Double.NaN));

        plant.update(new ManualLoopClock().clock());

        assertTrue(Double.isFinite(plant.getAppliedTarget()));
        assertEquals(0.0, plant.getAppliedTarget(), EPSILON);
        assertEquals(0.0, output.received, EPSILON);
        assertEquals(0.0, output.getCommandedPower(), EPSILON);
        assertEquals(PlantTargetStatus.Kind.TARGET_UNAVAILABLE, plant.getTargetStatus().kind());
        assertFalse(plant.getTargetPlan().hasTarget());
    }

    @Test
    public void unavailableNonFinitePowerSourceRetainsPriorSafeCommand() {
        ClampingPowerOutput output = new ClampingPowerOutput();
        ScalarTarget target = ScalarTarget.held(0.4);
        Plant plant = Plants.power(output, target);
        ManualLoopClock clock = new ManualLoopClock();
        plant.update(clock.clock());

        double[] unavailable = {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY};
        for (double value : unavailable) {
            target.set(value);
            plant.update(clock.nextCycle(0.02));

            assertTrue(Double.isFinite(plant.getAppliedTarget()));
            assertEquals(0.4, plant.getAppliedTarget(), EPSILON);
            assertEquals(0.4, output.received, EPSILON);
            assertEquals(PlantTargetStatus.Kind.TARGET_UNAVAILABLE, plant.getTargetStatus().kind());
            assertFalse(plant.getTargetPlan().hasTarget());
            assertTrue(plant.getTargetPlan().reason().contains("non-finite"));
        }
    }

    @Test
    public void powerGuardReceivesNormalizedCandidate() {
        ClampingPowerOutput output = new ClampingPowerOutput();
        ScalarTarget target = ScalarTarget.held(2.0);
        final double[] guardedCandidate = {Double.NaN};
        PlantTargetGuards guards = PlantTargetGuards.builder()
                .holdLastTargetUnless("capture", (candidate, clock) -> {
                    guardedCandidate[0] = candidate;
                    return true;
                })
                .build();
        Plant plant = Plants.power(output, PlantTargets.exact(target), target, guards);

        plant.update(new ManualLoopClock().clock());

        assertEquals(1.0, guardedCandidate[0], EPSILON);
        assertEquals(1.0, plant.getAppliedTarget(), EPSILON);
        assertEquals(PlantTargetStatus.Kind.CLAMPED_TO_RANGE, plant.getTargetStatus().kind());
    }

    @Test
    public void inRangePowerFallbackRetainsFallbackStatus() {
        ClampingPowerOutput output = new ClampingPowerOutput();
        ScalarTarget target = ScalarTarget.held(0.8);
        PlantTargetGuards guards = PlantTargetGuards.builder()
                .fallbackTargetUnless("mechanismClear", clock -> false, 0.25)
                .build();
        Plant plant = Plants.power(output, PlantTargets.exact(target), target, guards);

        plant.update(new ManualLoopClock().clock());

        assertEquals(0.8, plant.getRequestedTarget(), EPSILON);
        assertEquals(0.25, plant.getAppliedTarget(), EPSILON);
        assertEquals(0.25, output.received, EPSILON);
        assertEquals(PlantTargetStatus.Kind.FALLBACK_ACTIVE, plant.getTargetStatus().kind());
    }

    @Test
    public void powerRejectsStaticFallbackOutsideNormalizedRange() {
        PlantTargetGuards guards = PlantTargetGuards.builder()
                .fallbackTargetUnless("mechanismClear", clock -> false, 2.0)
                .build();

        try {
            Plants.power(new ClampingPowerOutput(), PlantTargets.exact(0.0), null, guards);
            fail("Expected an out-of-range power fallback to be rejected");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("PowerPlant"));
            assertTrue(expected.getMessage().contains("mechanismClear"));
            assertTrue(expected.getMessage().contains("2.0"));
            assertTrue(expected.getMessage().contains("-1.0"));
            assertTrue(expected.getMessage().contains("1.0"));
        }
    }

    @Test
    public void powerRateLimiterMovesTowardNormalizedBoundary() {
        ClampingPowerOutput output = new ClampingPowerOutput();
        ScalarTarget target = ScalarTarget.held(0.0);
        PlantTargetGuards guards = PlantTargetGuards.builder()
                .maxTargetRate(0.25)
                .build();
        Plant plant = Plants.power(output, PlantTargets.exact(target), target, guards);
        ManualLoopClock clock = new ManualLoopClock();
        plant.update(clock.clock());

        target.set(2.0);
        plant.update(clock.nextCycle(1.0));

        assertEquals(2.0, plant.getRequestedTarget(), EPSILON);
        assertEquals(0.25, plant.getAppliedTarget(), EPSILON);
        assertEquals(0.25, output.received, EPSILON);
        assertEquals(PlantTargetStatus.Kind.RATE_LIMITED, plant.getTargetStatus().kind());
    }

    @Test
    public void nonFiniteGuardResultNeverReachesLowLevelUnboundedOutput() {
        RecordingPositionOutput output = new RecordingPositionOutput();
        ScalarTarget target = ScalarTarget.held(Double.MAX_VALUE);
        PlantTargetGuards guards = PlantTargetGuards.builder()
                .maxTargetRate(Double.MAX_VALUE)
                .build();
        Plant plant = Plants.position(output, PlantTargets.exact(target), target, guards);
        ManualLoopClock clock = new ManualLoopClock();
        plant.update(clock.clock());
        double prior = plant.getAppliedTarget();

        target.set(-Double.MAX_VALUE);
        plant.update(clock.nextCycle(2.0));

        assertTrue(Double.isFinite(plant.getAppliedTarget()));
        assertEquals(prior, plant.getAppliedTarget(), 0.0);
        assertEquals(prior, output.getCommandedPosition(), 0.0);
        assertEquals(PlantTargetStatus.Kind.TARGET_UNAVAILABLE, plant.getTargetStatus().kind());

        target.set(0.0);
        plant.update(clock.nextCycle(1.0));

        assertEquals(0.0, plant.getAppliedTarget(), EPSILON);
        assertEquals(0.0, output.getCommandedPosition(), EPSILON);
    }

    private static final class ClampingPowerOutput implements PowerOutput {
        private double received = Double.NaN;
        private double commanded = Double.NaN;

        @Override
        public void setPower(double power) {
            received = power;
            commanded = Math.max(-1.0, Math.min(1.0, power));
        }

        @Override
        public double getCommandedPower() {
            return commanded;
        }
    }

    private static final class RecordingPositionOutput implements PositionOutput {
        private double commanded = Double.NaN;

        @Override
        public void setPosition(double position) {
            commanded = position;
        }

        @Override
        public double getCommandedPosition() {
            return commanded;
        }
    }
}
