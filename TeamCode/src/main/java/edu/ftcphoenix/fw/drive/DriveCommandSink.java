package edu.ftcphoenix.fw.drive;

import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Minimal sink for normalized {@link DriveSignal} commands.
 *
 * <p>This is the smallest Phoenix-owned seam that autonomous helpers and route adapters should
 * depend on when they only need to command drive signals. The default drivetrain implementation is
 * {@link MecanumDrivebase}, but teams integrating Road Runner, Pedro Pathing, or custom followers
 * can expose their own sink without pulling the rest of their route stack into the framework.</p>
 *
 * <p>Typical usage:</p>
 * <pre>{@code
 * DriveCommandSink sink = drivebase; // MecanumDrivebase implements it
 * Task nudge = DriveTasks.driveForSeconds(sink, new DriveSignal(0.2, 0.0, 0.0), 0.15);
 *
 * // Or from an external route follower adapter:
 * public final class PedroSink implements DriveCommandSink {
 *     private final Follower follower;
 *
 *     public void update(LoopClock clock) {
 *         follower.update();
 *     }
 *
 *     public void drive(DriveSignal signal) {
 *         follower.setTeleOpMovementVectors(signal.axial, signal.lateral, signal.omega);
 *     }
 *
 *     public void stop() {
 *         follower.setTeleOpMovementVectors(0.0, 0.0, 0.0);
 *     }
 * }
 * }</pre>
 */
public interface DriveCommandSink {

    /**
     * Optional lifecycle hook for per-loop timing or housekeeping.
     *
     * <p>The default implementation is a no-op so simple sinks do not need to care about it.
     * Guidance tasks call this once per loop before emitting a new drive command.</p>
     */
    default void update(LoopClock clock) {
        // default no-op
    }

    /**
     * Apply a normalized drive command for the current loop.
     *
     * @param signal normalized robot-centric command to apply
     */
    void drive(DriveSignal signal);

    /**
     * Stop drive output immediately.
     *
     * <p>Implementations should make a best effort to leave the drivetrain in a predictable,
     * motionless state.</p>
     */
    void stop();
}
