package edu.ftcsushi.robots.examples.basicflywheel;

import org.junit.Test;

import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.fw.testing.ftc.FtcTestHardware;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/**
 * Beginner software proof for one numeric velocity capability and its feedback-aware Task.
 *
 * <p>Question: do one request, the normal output heartbeat, and later encoder evidence stay
 * distinct? Keep real: the capability, mechanism, Plant, Task, and heartbeat. Replace: only the
 * FTC motor. Observe: recorded commands, immutable status, and Task outcomes. This cannot prove
 * physical direction, safe speed, controller tuning, loaded response, or stopping distance.</p>
 */
public final class BasicFlywheelSoftwareScenarioTest {
    private static final double EPSILON = 1e-9;

    @Test
    public void requestAppliesOnHeartbeatAndFreshFeedbackCompletesTask() {
        // ARRANGE: the software motor begins with no command and an authored zero measurement.
        Scenario scenario = new Scenario();
        scenario.motor.setMeasuredVelocityTicksPerSec(0.0);

        // REQUEST: intent changes now; cached applied/measurement facts and hardware do not.
        scenario.flywheel.setVelocityTicksPerSec(200.0);
        BasicFlywheel.Status requested = scenario.flywheel.status();
        assertEquals(200.0, requested.requestedVelocityTicksPerSec(), EPSILON);
        assertEquals(0.0, requested.appliedVelocityTicksPerSec(), EPSILON);
        assertTrue(Double.isNaN(requested.measuredVelocityTicksPerSec()));
        assertEquals(0, scenario.motor.velocityWrites());

        // HEARTBEAT: the production owner applies the request and samples independent feedback.
        scenario.flywheel.update(scenario.time.clock());
        BasicFlywheel.Status applied = scenario.flywheel.status();
        assertEquals(200.0, scenario.motor.commandedVelocityTicksPerSec(), EPSILON);
        assertEquals(200.0, applied.appliedVelocityTicksPerSec(), EPSILON);
        assertEquals(0.0, applied.measuredVelocityTicksPerSec(), EPSILON);
        assertFalse(applied.atRequestedVelocity());

        // OLD SUCCESS: make the prior request's cached evidence genuinely successful.
        scenario.motor.setMeasuredVelocityTicksPerSec(200.0);
        scenario.flywheel.update(scenario.time.nextCycle(0.02));
        assertTrue(scenario.flywheel.status().atRequestedVelocity());

        // REQUEST: the Task writes a different numeric intent but cannot reuse that old success.
        Task reachVelocity = scenario.flywheel.setVelocityTask(300.0);
        reachVelocity.start(scenario.time.nextCycle(0.02));
        reachVelocity.update(scenario.time.clock());
        assertFalse(reachVelocity.isComplete());

        // INJECT EVIDENCE + HEARTBEAT: a later owner update observes the requested velocity.
        scenario.motor.setMeasuredVelocityTicksPerSec(300.0);
        scenario.flywheel.update(scenario.time.clock());

        // ASSERT: the following Task phase consumes that cached feedback and succeeds.
        reachVelocity.update(scenario.time.nextCycle(0.02));
        BasicFlywheel.Status reached = scenario.flywheel.status();
        assertEquals(TaskOutcome.SUCCESS, reachVelocity.getOutcome());
        assertEquals(300.0, reached.measuredVelocityTicksPerSec(), EPSILON);
        assertTrue(reached.atRequestedVelocity());

        // NEXT GATE: isolate the real mechanism before checking direction, tuning, or load.
    }

    @Test
    public void timeoutLeavesRequestButCancellationRequestsZeroOnNextHeartbeat() {
        // ARRANGE / REQUEST: no authored feedback can satisfy this first velocity Task.
        Scenario scenario = new Scenario();
        scenario.motor.setMeasuredVelocityTicksPerSec(0.0);
        Task timedOut = scenario.flywheel.setVelocityTask(300.0);
        timedOut.start(scenario.time.clock());
        timedOut.update(scenario.time.clock());
        scenario.flywheel.update(scenario.time.clock());

        // ASSERT: timeout reports the missing arrival evidence and leaves persistent intent alone.
        timedOut.update(scenario.time.nextCycle(scenario.config.spinUpTimeoutSec));
        assertEquals(TaskOutcome.TIMEOUT, timedOut.getOutcome());
        assertEquals(300.0,
                scenario.flywheel.status().requestedVelocityTicksPerSec(), EPSILON);

        // REQUEST: active cancellation selects the explicit zero policy immediately.
        Task cancelled = scenario.flywheel.setVelocityTask(200.0);
        cancelled.start(scenario.time.nextCycle(0.02));
        cancelled.update(scenario.time.clock());
        cancelled.cancel();
        BasicFlywheel.Status cancellationRequested = scenario.flywheel.status();
        assertEquals(TaskOutcome.CANCELLED, cancelled.getOutcome());
        assertEquals(0.0, cancellationRequested.requestedVelocityTicksPerSec(), EPSILON);
        assertEquals(300.0, cancellationRequested.appliedVelocityTicksPerSec(), EPSILON);
        assertEquals(300.0, scenario.motor.commandedVelocityTicksPerSec(), EPSILON);

        // HEARTBEAT + ASSERT: only the normal output phase realizes the zero request.
        scenario.flywheel.update(scenario.time.clock());
        BasicFlywheel.Status stopped = scenario.flywheel.status();
        assertEquals(0.0, stopped.appliedVelocityTicksPerSec(), EPSILON);
        assertEquals(0.0, scenario.motor.commandedVelocityTicksPerSec(), EPSILON);

        // NEXT GATE: robot testing must measure actual coast-down and emergency-stop behavior.
    }

    private static final class Scenario {
        private final BasicFlywheelMechanism.Config config =
                BasicFlywheelMechanism.Config.defaults();
        private final FtcTestHardware hardware = new FtcTestHardware();
        private final FtcTestHardware.MotorProbe motor = hardware.addMotor(config.motorName);
        private final BasicFlywheelMechanism flywheel =
                new BasicFlywheelMechanism(hardware, config);
        private final ManualLoopClock time = new ManualLoopClock();
    }
}
