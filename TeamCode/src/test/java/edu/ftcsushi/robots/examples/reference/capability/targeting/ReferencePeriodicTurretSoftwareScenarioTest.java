package edu.ftcsushi.robots.examples.reference.capability.targeting;

import org.junit.Test;

import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.fw.testing.ftc.FtcTestHardware;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/**
 * Causal software scenario for a numeric periodic position.
 *
 * <p>Question: does the real mechanism select and reach the nearest legal full-turn equivalent?
 * Keep real: command, resolver, Plant, Task, status, and heartbeat. Replace: the FTC motor only.
 * Observe: requested, selected, applied, measured, and arrival facts. This cannot prove encoder
 * zero, scale, cable clearance, direction, tuning, or physical safety.</p>
 */
public final class ReferencePeriodicTurretSoftwareScenarioTest {
    private static final double EPSILON = 1e-9;

    @Test
    public void heartbeatSelectsTheNearestLegalRepresentativeAndLaterEvidenceArrives() {
        // ARRANGE: the turret begins near +3π inside a cable-limited three-turn range.
        Fixture scenario = new Fixture(2.9 * Math.PI);
        double logicalRequestRad = 0.1 * Math.PI;
        double expectedPhysicalRad = 2.1 * Math.PI;

        // REQUEST / BEFORE HEARTBEAT: intent changes without bypassing the output owner.
        scenario.turret.setAngleRad(logicalRequestRad);
        assertEquals(0, scenario.motor.targetPositionTicks());

        // HEARTBEAT: equivalentPositionsOf chooses the nearest in-range full-turn representative.
        scenario.turret.update(scenario.time.clock());
        ReferencePeriodicTurretMechanism.Status moving = scenario.turret.status();
        assertEquals(logicalRequestRad, moving.requestedAngleRad(), EPSILON);
        assertEquals(expectedPhysicalRad, moving.selectedAngleRad(), EPSILON);
        assertEquals(expectedPhysicalRad, moving.appliedAngleRad(), EPSILON);
        assertEquals(scenario.toTicks(expectedPhysicalRad), scenario.motor.targetPositionTicks());
        assertTrue(moving.requestWasSelected());
        assertFalse(moving.arrived());

        // INJECT EVIDENCE / HEARTBEAT: only encoder evidence at that representative proves arrival.
        scenario.motor.setCurrentPositionTicks(scenario.toTicks(expectedPhysicalRad));
        scenario.turret.update(scenario.time.nextCycle(0.02));

        // ASSERT / NEXT GATE: software arrival is proven; physical bounds still need isolated test.
        assertTrue(scenario.turret.status().arrived());
    }

    @Test
    public void taskReportsTimeoutAndCancellationWithoutInventingAnotherHoldPoint() {
        // ARRANGE / REQUEST: feedback stays away from the newly selected representative.
        Fixture scenario = new Fixture(0.0);
        double requestRad = 0.75 * Math.PI;
        Task timed = scenario.turret.setAngleTask(requestRad, 1.0);
        timed.start(scenario.time.clock());
        scenario.turret.update(scenario.time.clock());

        // ASSERT: timeout reports missing arrival and leaves the persistent position request.
        timed.update(scenario.time.nextCycle(1.0));
        assertEquals(TaskOutcome.TIMEOUT, timed.getOutcome());
        assertEquals(requestRad, scenario.turret.status().requestedAngleRad(), EPSILON);

        // REQUEST / ASSERT: active cancellation has the same deliberate hold-request policy.
        double cancelledRequestRad = -0.25 * Math.PI;
        Task cancelled = scenario.turret.setAngleTask(cancelledRequestRad, 1.0);
        cancelled.start(scenario.time.nextCycle(0.02));
        cancelled.cancel();
        scenario.turret.update(scenario.time.clock());
        assertEquals(TaskOutcome.CANCELLED, cancelled.getOutcome());
        assertEquals(cancelledRequestRad,
                scenario.turret.status().requestedAngleRad(), EPSILON);
    }

    private static final class Fixture {
        private final ManualLoopClock time = new ManualLoopClock();
        private final ReferencePeriodicTurretMechanism.Config config = config();
        private final FtcTestHardware hardware = new FtcTestHardware();
        private final FtcTestHardware.MotorProbe motor = hardware.addMotor(config.motorName);
        private final ReferencePeriodicTurretMechanism turret;

        private Fixture(double initialMeasuredAngleRad) {
            motor.setCurrentPositionTicks(toTicks(initialMeasuredAngleRad));
            turret = new ReferencePeriodicTurretMechanism(hardware, config);
        }

        private int toTicks(double angleRad) {
            return (int) Math.round(angleRad * config.ticksPerRad);
        }

        private static ReferencePeriodicTurretMechanism.Config config() {
            ReferencePeriodicTurretMechanism.Config c =
                    ReferencePeriodicTurretMechanism.Config.defaults();
            c.motorName = "turret";
            c.ticksPerRad = 1000.0;
            c.positionToleranceRad = 0.01;
            return c;
        }
    }
}
