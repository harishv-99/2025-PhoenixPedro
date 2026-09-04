package edu.ftcsushi.robots.examples.starter.opmode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.junit.Test;

import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.fw.testing.ftc.FtcTestHardware;
import edu.ftcsushi.robots.examples.starter.capability.intake.StarterIntake;
import edu.ftcsushi.robots.examples.starter.robot.StarterProfile;
import edu.ftcsushi.robots.examples.starter.robot.StarterRobot;
import edu.ftcsushi.robots.examples.starter.support.StarterTestHardware;

import static edu.ftcsushi.robots.examples.starter.support.StarterTestHardware.prepare;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Managed software proof for the exact one-timed-Task policy selected by {@link StarterAuto}. */
public final class StarterTimedAutoSoftwareScenarioTest {

    @Test
    public void timedRootBeginsAtStartAndRequestsStoppedAtItsOwnBoundary() {
        // ARRANGE: keep production declaration/routine/lifecycle; replace devices, time, and gate.
        StarterProfile profile = enabledProfile();
        FtcTestHardware hardware = new FtcTestHardware();
        FtcTestHardware.MotorProbe motor = hardware.addMotor(profile.intake.motorName);
        ManagedAuto mode = prepare(
                new ManagedAuto(profile),
                hardware,
                new StarterTestHardware.TelemetryProbe(),
                new Gamepad());
        mode.advanceTo(5.0);
        mode.init();

        // BEFORE START: configuration built the fresh Task but neither Task nor output has run.
        assertEquals(0, motor.powerWrites());
        assertEquals(StarterIntake.Mode.STOPPED, mode.intake.status().mode());

        // START: the host resets its clock, starts the root, then realizes COLLECT once.
        mode.advanceTo(10.0);
        mode.start();
        assertEquals(profile.intake.collectPower, motor.power(), 0.0);
        assertEquals(StarterIntake.Mode.COLLECT, mode.intake.status().mode());

        // HEARTBEAT: INIT time was not charged; the request remains active before 0.75 seconds.
        mode.advanceTo(10.74);
        mode.loop();
        assertFalse(mode.root.isComplete());
        assertEquals(profile.intake.collectPower, motor.power(), 0.0);

        // ASSERT: at the Task's own boundary, STOPPED is applied in the same managed cycle.
        mode.advanceTo(10.75);
        mode.loop();
        assertTrue(mode.root.isComplete());
        assertEquals(TaskOutcome.SUCCESS, mode.root.getOutcome());
        assertEquals(StarterIntake.Mode.STOPPED, mode.intake.status().mode());
        assertEquals(0.0, motor.power(), 0.0);
        mode.stop();
        // NEXT GATE: confirm direction, duration, and physical stop with the intake isolated.
    }

    @Test
    public void stopCancelsTheActiveRootAndTheFactoryBuildsFreshSingleUseWork() {
        StarterProfile profile = enabledProfile();
        FtcTestHardware firstHardware = new FtcTestHardware();
        FtcTestHardware.MotorProbe firstMotor =
                firstHardware.addMotor(profile.intake.motorName);
        ManagedAuto first = prepare(
                new ManagedAuto(profile),
                firstHardware,
                new StarterTestHardware.TelemetryProbe(),
                new Gamepad());
        first.init();
        first.start();

        // STOP: managed cancellation publishes the safe request before the Plant is terminally zeroed.
        first.stop();
        assertEquals(TaskOutcome.CANCELLED, first.root.getOutcome());
        assertEquals(StarterIntake.Mode.STOPPED, first.intake.status().mode());
        assertEquals(0.0, firstMotor.power(), 0.0);

        // FRESHNESS: another configuration invokes the same production factory for a new Task.
        StarterProfile secondProfile = enabledProfile();
        FtcTestHardware secondHardware = new FtcTestHardware();
        secondHardware.addMotor(secondProfile.intake.motorName);
        ManagedAuto second = prepare(
                new ManagedAuto(secondProfile),
                secondHardware,
                new StarterTestHardware.TelemetryProbe(),
                new Gamepad());
        second.init();
        assertNotSame(first.root, second.root);

        try {
            first.root.start(new ManualLoopClock().clock());
            fail("Expected a started Task to reject reuse");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("single-use"));
        }
        second.start();
        assertEquals(secondProfile.intake.collectPower,
                secondHardware.motor(secondProfile.intake.motorName).power(), 0.0);
        second.stop();
    }

    private static StarterProfile enabledProfile() {
        StarterProfile profile = StarterProfile.current();
        profile.allowIntakeMotion = true;
        return profile;
    }

    /** Test-only managed host that changes only the fail-closed profile permission. */
    private static final class ManagedAuto extends FtcRobotOpMode {
        private final StarterProfile profile;
        private double runtimeSec;
        private StarterIntake intake;
        private Task root;

        private ManagedAuto(StarterProfile profile) {
            this.profile = profile;
        }

        @Override
        protected void configure(RobotProgram program) {
            intake = new StarterRobot(hardwareMap).declareAuto(program, profile);
            root = StarterAuto.oneTimedCollect(intake);
            program.rootTask(root);
        }

        @Override
        public double getRuntime() {
            return runtimeSec;
        }

        private void advanceTo(double nowSec) {
            runtimeSec = nowSec;
        }
    }
}
