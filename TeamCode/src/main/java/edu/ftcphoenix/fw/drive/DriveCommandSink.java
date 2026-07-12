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
 * <p>Simple sinks apply commands immediately. Stateful external adapters may instead stage a
 * command for their next owned heartbeat. Such an adapter must document that timing, have one
 * stable composition-root owner call {@link #update(LoopClock)} every loop, and make that hook
 * cycle-idempotent when guidance or drive Tasks can also call it.</p>
 *
 * <p>Typical direct-sink usage:</p>
 * <pre>{@code
 * DriveCommandSink sink = drivebase; // MecanumDrivebase implements it
 * Task nudge = DriveTasks.driveForSeconds(sink, new DriveSignal(0.2, 0.0, 0.0), 0.15);
 *
 * // A cycle-owned external adapter is also one sink:
 * DriveCommandSink autoDrive = new PedroPathingDriveAdapter(follower);
 * autoDrive.update(clock); // composition-root heartbeat, every Auto loop
 * }</pre>
 */
public interface DriveCommandSink {

    /**
     * Optional lifecycle hook for per-loop timing, realization, or housekeeping.
     *
     * <p>The default implementation is a no-op so simple sinks do not need to care about it.
     * Guidance and timed-drive Tasks call this before emitting a new drive command. An external
     * sink that requires updates even while another Task is active must also have one stable
     * composition-root owner call this every loop and must deduplicate repeated calls by
     * {@link LoopClock#cycle()}.</p>
     *
     * @param clock shared loop clock for the current cycle
     */
    default void update(LoopClock clock) {
        // default no-op
    }

    /**
     * Submit a normalized robot-centric drive command.
     *
     * <p>Direct sinks normally apply it immediately. A documented heartbeat-driven adapter may
     * retain it for the next owned {@link #update(LoopClock)} call.</p>
     *
     * @param signal normalized robot-centric command to apply
     */
    void drive(DriveSignal signal);

    /**
     * Stop drive output immediately.
     *
     * <p>Implementations should make a best effort to leave the drivetrain in a predictable,
     * motionless state. Merely staging a zero command for a future heartbeat does not satisfy this
     * contract.</p>
     */
    void stop();
}
