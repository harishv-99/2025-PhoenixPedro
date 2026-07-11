package edu.ftcphoenix.fw.actuation;

import org.junit.Test;

import edu.ftcphoenix.fw.core.hal.PositionOutput;
import edu.ftcphoenix.fw.core.source.ScalarTarget;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies final target safety for mapped position Plants. */
public final class MappedPositionPlantSafetyTest {

    private static final double EPSILON = 1e-12;

    @Test
    public void buildRejectsFallbackOutsideDeclaredRange() {
        PlantTargetGuards guards = PlantTargetGuards.builder()
                .fallbackTargetUnless("mechanismClear", clock -> false, 5.0)
                .build();

        try {
            MappedPositionPlant.commanded(new RecordingPositionOutput())
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
    public void inRangeFallbackRetainsFallbackStatus() {
        RecordingPositionOutput output = new RecordingPositionOutput();
        ScalarTarget target = ScalarTarget.held(18.0);
        PlantTargetGuards guards = PlantTargetGuards.builder()
                .fallbackTargetUnless("mechanismClear", clock -> false, 12.0)
                .build();
        MappedPositionPlant plant = MappedPositionPlant.commanded(output)
                .range(ScalarRange.bounded(10.0, 20.0))
                .targetGuards(guards)
                .targetedBy(target)
                .build();

        plant.update(new ManualLoopClock().clock());

        assertEquals(18.0, plant.getRequestedTarget(), EPSILON);
        assertEquals(12.0, plant.getAppliedTarget(), EPSILON);
        assertEquals(12.0, output.getCommandedPosition(), EPSILON);
        assertEquals(PlantTargetStatus.Kind.FALLBACK_ACTIVE, plant.getTargetStatus().kind());
        assertTrue(plant.getTargetStatus().message().contains("mechanismClear"));
    }

    @Test
    public void tinyBoundaryClampIsNeverReportedAccepted() {
        RecordingPositionOutput output = new RecordingPositionOutput();
        double request = 20.0 + 1.0e-10;
        MappedPositionPlant plant = MappedPositionPlant.commanded(output)
                .range(ScalarRange.bounded(10.0, 20.0))
                .targetedBy(ScalarTarget.held(request))
                .build();

        plant.update(new ManualLoopClock().clock());

        assertEquals(request, plant.getRequestedTarget(), 0.0);
        assertEquals(20.0, plant.getAppliedTarget(), EPSILON);
        assertEquals(20.0, output.getCommandedPosition(), EPSILON);
        assertEquals(PlantTargetStatus.Kind.CLAMPED_TO_RANGE, plant.getTargetStatus().kind());
    }

    @Test
    public void signedZeroInsideRangeRemainsAccepted() {
        MappedPositionPlant plant = MappedPositionPlant.commanded(new RecordingPositionOutput())
                .range(ScalarRange.bounded(0.0, 20.0))
                .targetedBy(ScalarTarget.held(-0.0))
                .build();

        plant.update(new ManualLoopClock().clock());

        assertEquals(PlantTargetStatus.Kind.ACCEPTED, plant.getTargetStatus().kind());
    }

    @Test
    public void initialHoldIsClampedAndLimiterContinuesFromActualCommand() {
        RecordingPositionOutput output = new RecordingPositionOutput();
        ScalarTarget target = ScalarTarget.held(20.0);
        final boolean[] mechanismClear = {false};
        PlantTargetGuards guards = PlantTargetGuards.builder()
                .maxTargetRate(1.0)
                .holdLastTargetUnless("mechanismClear", clock -> mechanismClear[0])
                .build();
        MappedPositionPlant plant = MappedPositionPlant.commanded(output)
                .range(ScalarRange.bounded(10.0, 20.0))
                .targetGuards(guards)
                .targetedBy(target)
                .build();
        ManualLoopClock clock = new ManualLoopClock();

        plant.update(clock.clock());

        assertEquals(10.0, plant.getAppliedTarget(), EPSILON);
        assertEquals(10.0, output.getCommandedPosition(), EPSILON);
        assertEquals(PlantTargetStatus.Kind.CLAMPED_TO_RANGE, plant.getTargetStatus().kind());
        assertTrue(plant.getTargetStatus().message().contains("HOLDING_LAST"));

        mechanismClear[0] = true;
        plant.update(clock.nextCycle(1.0));

        assertEquals(11.0, plant.getAppliedTarget(), EPSILON);
        assertEquals(11.0, output.getCommandedPosition(), EPSILON);
        assertEquals(PlantTargetStatus.Kind.RATE_LIMITED, plant.getTargetStatus().kind());
    }

    @Test
    public void unavailableNonFiniteSourceRetainsFiniteCommandAndExplainsIt() {
        RecordingPositionOutput output = new RecordingPositionOutput();
        ScalarTarget target = ScalarTarget.held(12.0);
        MappedPositionPlant plant = MappedPositionPlant.commanded(output)
                .range(ScalarRange.bounded(10.0, 20.0))
                .targetedBy(target)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        plant.update(clock.clock());

        target.set(Double.NaN);
        plant.update(clock.nextCycle(0.02));

        assertTrue(Double.isFinite(plant.getAppliedTarget()));
        assertEquals(12.0, plant.getAppliedTarget(), EPSILON);
        assertEquals(12.0, output.getCommandedPosition(), EPSILON);
        assertEquals(PlantTargetStatus.Kind.TARGET_UNAVAILABLE, plant.getTargetStatus().kind());
        assertTrue(plant.getTargetStatus().message().contains("non-finite"));
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
