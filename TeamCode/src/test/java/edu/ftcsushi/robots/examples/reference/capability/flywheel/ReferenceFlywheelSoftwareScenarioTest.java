package edu.ftcsushi.robots.examples.reference.capability.flywheel;

import org.junit.Test;

import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.fw.testing.ftc.FtcTestHardware;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

/**
 * Causal software scenario for paired flywheel status and Tasks.
 *
 * <p>Question: does the real mechanism owner command one grouped Plant while requiring both
 * independent wheel samples for readiness? Keep real: mechanism, Plant, Task, and heartbeat.
 * Replace: FTC motors only. Observe: commands, immutable status, and Task outcome. This cannot
 * prove safe direction, tuning, physical balance under load, release, or scoring.</p>
 */
public final class ReferenceFlywheelSoftwareScenarioTest {
    private static final double EPSILON = 1e-9;

    @Test
    public void bothIndependentMeasurementsCauseReadinessAndTaskSuccess() {
        // ARRANGE: the opposite wheel errors cancel in the grouped mean but exceed tolerance.
        Fixture scenario = new Fixture();
        scenario.left.setMeasuredVelocityTicksPerSec(800.0);
        scenario.right.setMeasuredVelocityTicksPerSec(1200.0);

        // REQUEST: direct control changes intent; hardware changes only at the output heartbeat.
        scenario.flywheels.setVelocityTicksPerSec(1000.0);
        assertEquals(0.0, scenario.left.commandedVelocityTicksPerSec(), EPSILON);

        // HEARTBEAT: the production owner writes both motors and publishes one complete status.
        scenario.flywheels.update(scenario.time.clock());
        ReferenceFlywheels.Status unbalanced = scenario.flywheels.status();
        assertEquals(1000.0, scenario.left.commandedVelocityTicksPerSec(), EPSILON);
        assertTrue("the grouped mean is at target", unbalanced.plantSnapshot().atCommandTarget());
        assertFalse("independent member evidence prevents a false ready claim", unbalanced.ready());

        // REQUEST: every factory call creates a fresh single-use request-and-wait Task.
        Task waitForBoth = scenario.flywheels.setVelocityTask(1000.0, 1.0);
        assertNotSame(waitForBoth, scenario.flywheels.setVelocityTask(1000.0, 1.0));
        waitForBoth.start(scenario.time.nextCycle(0.02));
        scenario.flywheels.update(scenario.time.clock());
        waitForBoth.update(scenario.time.nextCycle(0.02));
        assertFalse(waitForBoth.isComplete());

        // INJECT EVIDENCE: only later independent samples place both wheels in tolerance.
        scenario.left.setMeasuredVelocityTicksPerSec(1000.0);
        scenario.right.setMeasuredVelocityTicksPerSec(1000.0);
        scenario.flywheels.update(scenario.time.clock());

        // ASSERT: the next Task phase consumes that cached publication and succeeds.
        waitForBoth.update(scenario.time.nextCycle(0.02));
        assertTrue(waitForBoth.isComplete());
        assertEquals(TaskOutcome.SUCCESS, waitForBoth.getOutcome());
        assertTrue(scenario.flywheels.status().ready());

        // NEXT GATE: isolate and restrain the physical mechanism before testing direction/load.
    }

    @Test
    public void timeoutLeavesTheRequestWhileActiveCancellationRequestsIdle() {
        // ARRANGE / REQUEST: no injected velocity can satisfy this one-second move.
        Fixture scenario = new Fixture();
        Task timed = scenario.flywheels.setVelocityTask(1000.0, 1.0);
        timed.start(scenario.time.clock());
        scenario.flywheels.update(scenario.time.clock());

        // HEARTBEAT / ASSERT: timeout is an outcome, not an invented recovery command.
        timed.update(scenario.time.nextCycle(1.0));
        assertEquals(TaskOutcome.TIMEOUT, timed.getOutcome());
        assertEquals(1000.0,
                scenario.flywheels.status().requestedVelocityTicksPerSec(), EPSILON);

        // REQUEST / ASSERT: active cancellation has the separately documented idle policy.
        Task cancelled = scenario.flywheels.setVelocityTask(750.0, 1.0);
        cancelled.start(scenario.time.nextCycle(0.02));
        cancelled.cancel();
        assertEquals(TaskOutcome.CANCELLED, cancelled.getOutcome());
        scenario.flywheels.update(scenario.time.clock());
        assertEquals(0.0, scenario.left.commandedVelocityTicksPerSec(), EPSILON);
        assertEquals(0.0, scenario.right.commandedVelocityTicksPerSec(), EPSILON);

        assertThrows(IllegalStateException.class, () -> cancelled.start(scenario.time.clock()));
    }

    private static final class Fixture {
        private final ManualLoopClock time = new ManualLoopClock();
        private final ReferenceFlywheelMechanism.Config config = config();
        private final FtcTestHardware hardware = new FtcTestHardware();
        private final FtcTestHardware.MotorProbe left = hardware.addMotor(config.leftMotorName);
        private final FtcTestHardware.MotorProbe right = hardware.addMotor(config.rightMotorName);
        private final ReferenceFlywheelMechanism flywheels =
                new ReferenceFlywheelMechanism(hardware, config);

        private static ReferenceFlywheelMechanism.Config config() {
            ReferenceFlywheelMechanism.Config c =
                    ReferenceFlywheelMechanism.Config.defaults();
            c.leftMotorName = "left";
            c.rightMotorName = "right";
            c.maximumVelocityTicksPerSec = 2000.0;
            c.velocityToleranceTicksPerSec = 50.0;
            return c;
        }
    }
}
