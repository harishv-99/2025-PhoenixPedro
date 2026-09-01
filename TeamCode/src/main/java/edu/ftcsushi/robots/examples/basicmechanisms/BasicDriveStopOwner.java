package edu.ftcsushi.robots.examples.basicmechanisms;

import java.util.Objects;

import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.drive.DriveCommandSink;
import edu.ftcsushi.fw.ftc.RobotProgram;

/**
 * Stable lifecycle owner that makes one terminal software-zero attempt for an Auto drive sink.
 *
 * <p>Its output heartbeat intentionally performs no sink update or drive command; the active
 * {@code DriveTasks} leaf remains the only command owner. Register this owner immediately after
 * acquiring the sink so STOP-before-START and an Auto that ends before its drive phase still call
 * the sink's immediate {@link DriveCommandSink#stop()} boundary.</p>
 */
final class BasicDriveStopOwner implements RobotProgram.Output {

    private final DriveCommandSink driveSink;
    private boolean stopped;

    BasicDriveStopOwner(DriveCommandSink driveSink) {
        this.driveSink = Objects.requireNonNull(driveSink, "driveSink");
    }

    /** No-op by design: this lifecycle owner never competes with the exclusive drive Task. */
    @Override
    public void update(LoopClock clock) {
        // DriveTasks owns every active update and command.
    }

    /** Requests terminal software zero at most once across failure cleanup and managed STOP. */
    @Override
    public void stop() {
        if (stopped) {
            return;
        }
        stopped = true;
        driveSink.stop();
    }
}
