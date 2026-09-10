package edu.ftcsushi.robots.examples.reference.capability.launcher;

import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.fw.testing.ftc.FtcTestHardware;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;

/** Maintainer-only fixture: real owners and loop order, independently authored FTC readings. */
final class ReferenceLauncherTestRig {
    static final double STEP = 0.03125;
    static final double EPS = 1e-9;

    final ReferenceLauncherMechanism.Config config;
    final FtcTestHardware hardware = new FtcTestHardware();
    final FtcTestHardware.MotorProbe left;
    final FtcTestHardware.MotorProbe right;
    final FtcTestHardware.CrServoProbe transfer;
    final FtcTestHardware.ServoProbe release;
    final FtcTestHardware.DigitalProbe first;
    final FtcTestHardware.DigitalProbe second;
    final FtcTestHardware.DigitalProbe third;
    final ManualLoopClock time = new ManualLoopClock();
    final ReferenceLauncherMechanism mechanism;

    ReferenceLauncherTestRig() { this(config()); }

    ReferenceLauncherTestRig(ReferenceLauncherMechanism.Config config) {
        this.config = config;
        left = hardware.addMotor(config.flywheels.leftMotorName);
        right = hardware.addMotor(config.flywheels.rightMotorName);
        transfer = hardware.addCrServo(config.transferName);
        release = hardware.addServo(config.releaseServoName);
        first = hardware.addDigitalInput(config.inventory.firstPositionSensorName);
        second = hardware.addDigitalInput(config.inventory.secondPositionSensorName);
        third = hardware.addDigitalInput(config.inventory.thirdPositionSensorName);
        staged(true);
        measured(1000.0, 1000.0);
        mechanism = new ReferenceLauncherMechanism(hardware, config);
    }

    static ReferenceLauncherMechanism.Config config() {
        ReferenceLauncherMechanism.Config c = ReferenceLauncherMechanism.Config.defaults();
        c.flywheels.leftMotorName = "left";
        c.flywheels.rightMotorName = "right";
        c.flywheels.maximumVelocityTicksPerSec = 2000.0;
        c.flywheels.velocityToleranceTicksPerSec = 50.0;
        c.feedVelocityTicksPerSec = 1000.0;
        c.spinUpTimeoutSec = 2.0;
        c.readySettlingSec = 0.125;
        c.evidenceMaxAgeSec = 0.25;
        c.releaseDurationSec = 0.125;
        c.transferDurationSec = 0.25;
        c.departureTimeoutSec = 0.75;
        c.inventory.occupiedDebounceSec = 0.0;
        c.inventory.vacatedDebounceSec = 0.0;
        return c;
    }

    void measured(double leftTicksPerSec, double rightTicksPerSec) {
        left.setMeasuredVelocityTicksPerSec(leftTicksPerSec);
        right.setMeasuredVelocityTicksPerSec(rightTicksPerSec);
    }

    void staged(boolean occupied) { first.setHigh(!occupied); }

    Task start() {
        Task task = mechanism.feedOne();
        task.start(time.clock());
        task.update(time.clock());
        mechanism.update(time.clock());
        return task;
    }

    void cycle(Task task) { cycle(task, STEP); }

    void cycle(Task task, double elapsedSec) {
        time.nextCycle(elapsedSec);
        task.update(time.clock());
        mechanism.update(time.clock());
    }

    void cycles(Task task, int count) {
        for (int i = 0; i < count; i++) cycle(task);
    }

    void outputCycle() { mechanism.update(time.nextCycle(STEP)); }

    void reach(Task task, ReferenceLauncher.Phase phase) {
        for (int i = 0; i < 80 && !task.isComplete()
                && mechanism.status().phase() != phase; i++) cycle(task);
        assertFalse("unexpected early ending: " + mechanism.status().reason(), task.isComplete());
        assertEquals(phase, mechanism.status().phase());
    }

    void finish(Task task) {
        for (int i = 0; i < 100 && !task.isComplete(); i++) cycle(task);
        org.junit.Assert.assertTrue("bounded attempt did not end", task.isComplete());
    }

    void assertFeedIdle() {
        assertEquals(0.0, transfer.power(), EPS);
        assertEquals(config.releaseRetractedNativePosition, release.position(), EPS);
        assertFalse(mechanism.status().transferActive());
    }

    void assertIdle() {
        assertFeedIdle();
        assertEquals(0.0, left.commandedVelocityTicksPerSec(), EPS);
        assertEquals(0.0, right.commandedVelocityTicksPerSec(), EPS);
        assertEquals(0.0, mechanism.status().flywheels().requestedVelocityTicksPerSec(), EPS);
    }
}
