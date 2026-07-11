package edu.ftcphoenix.fw.actuation;

import org.junit.Test;

import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.source.ScalarTarget;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/** Verifies the shared final-safety path used by lower-level Plants factories. */
public final class PlantsTargetSafetyTest {

    private static final double EPSILON = 1e-12;

    @Test
    public void finitePowerOutsideNormalizedRangeIsNotChangedBySafe01() {
        RecordingPowerOutput output = new RecordingPowerOutput();
        ScalarTarget target = ScalarTarget.held(2.0);
        Plant plant = Plants.power(output, target);

        plant.update(new ManualLoopClock().clock());

        assertEquals(2.0, plant.getAppliedTarget(), EPSILON);
        assertEquals(2.0, output.getCommandedPower(), EPSILON);
        assertEquals(PlantTargetStatus.Kind.ACCEPTED, plant.getTargetStatus().kind());
    }

    @Test
    public void nonFiniteGuardResultNeverReachesLowLevelOutput() {
        RecordingPowerOutput output = new RecordingPowerOutput();
        ScalarTarget target = ScalarTarget.held(Double.MAX_VALUE);
        PlantTargetGuards guards = PlantTargetGuards.builder()
                .maxTargetRate(Double.MAX_VALUE)
                .build();
        Plant plant = Plants.power(output, PlantTargets.exact(target), target, guards);
        ManualLoopClock clock = new ManualLoopClock();
        plant.update(clock.clock());
        double prior = plant.getAppliedTarget();

        target.set(-Double.MAX_VALUE);
        plant.update(clock.nextCycle(2.0));

        assertTrue(Double.isFinite(plant.getAppliedTarget()));
        assertEquals(prior, plant.getAppliedTarget(), 0.0);
        assertEquals(prior, output.getCommandedPower(), 0.0);
        assertEquals(PlantTargetStatus.Kind.TARGET_UNAVAILABLE, plant.getTargetStatus().kind());

        target.set(0.0);
        plant.update(clock.nextCycle(1.0));

        assertEquals(0.0, plant.getAppliedTarget(), EPSILON);
        assertEquals(0.0, output.getCommandedPower(), EPSILON);
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
}
