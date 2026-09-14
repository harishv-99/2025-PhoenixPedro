package edu.ftcsushi.robots.examples.cameraonlypickup;

import java.util.Collections;

import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.drive.DriveSignal;
import edu.ftcsushi.fw.input.binding.Bindings;
import edu.ftcsushi.fw.sensing.observation.TargetObservation2d;
import edu.ftcsushi.fw.sensing.observation.TargetObservations2d;
import edu.ftcsushi.fw.task.TaskBindings;
import edu.ftcsushi.fw.task.TaskRunner;
import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.fw.testing.ftc.FtcTestHardware;

/** Scripted outside world; real profile, capability, controls, runner, mechanism and Plant. */
final class CameraOnlyPickupTestRig {
    final ManualLoopClock time = new ManualLoopClock();
    final CameraOnlyPickupProfile profile = CameraOnlyPickupProfile.example();
    final FtcTestHardware hardware = new FtcTestHardware();
    final FtcTestHardware.MotorProbe motor = hardware.addMotor("intakeMotor");
    final FtcTestHardware.DigitalProbe sensor = hardware.addDigitalInput("intakeOccupied");
    final CameraOnlyPickupIntake intake = new CameraOnlyPickupIntake(hardware, profile.intake);
    final Bindings bindings = new Bindings();
    final TaskRunner runner = new TaskRunner();
    final DriveSignal manual = new DriveSignal(0.12, -0.08, 0.05);
    TargetObservations2d frame = TargetObservations2d.unavailable("no scripted frame yet");
    boolean held;
    boolean override;
    final CameraOnlyPickup pickup = new CameraOnlyPickup(profile, Source.of(clock -> frame),
            intake.occupancy(), intake::setCollecting, clock -> manual);
    final CameraOnlyPickupControls controls = new CameraOnlyPickupControls(
            clock -> held, clock -> override);

    CameraOnlyPickupTestRig() {
        sensor.setHigh(true);
        publish(14);
        controls.bind(bindings, TaskBindings.of(bindings, runner), pickup);
        bindings.update(clock());
    }

    LoopClock clock() { return time.clock(); }

    /** Authors one actual frame, with both the frame and candidate sharing its capture timestamp. */
    void publish(double forwardInches) {
        LoopTimestamp timestamp = clock().nowTimestamp();
        frame = TargetObservations2d.fromFrame(timestamp, Collections.singletonList(
                TargetObservation2d.ofRobotRelativePosition(forwardInches, 0, Double.NaN, timestamp)));
    }

    /** Advances time and independently injects camera geometry, then retains normal managed order. */
    void cycle(double seconds, double targetForward, boolean runTasks) {
        time.nextCycle(seconds);
        publish(targetForward);
        runPhases(runTasks);
    }

    /** A paused runner exists only to test queued controls; production never chooses its own runner. */
    void runPhases(boolean runTasks) {
        pickup.update(clock());
        bindings.update(clock());
        if (runTasks) runner.update(clock());
        intake.update(clock());
        pickup.driveSource().get(clock());
    }
}
