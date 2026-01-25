package edu.ftcphoenix.fw.drive;

import edu.ftcphoenix.fw.task.InstantTask;
import edu.ftcphoenix.fw.task.RunForSecondsTask;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.util.LoopClock;

/**
 * Small helper class for creating common drive-related {@link Task} patterns.
 *
 * <p>The goal is to give robot code very simple, readable building blocks like:</p>
 *
 * <pre>{@code
 * // Example: simple "L-shape" macro in TeleOp
 * Task macro = SequenceTask.of(
 *     DriveTasks.driveForSeconds(drivebase,
 *         new DriveSignal(+0.7, 0.0, 0.0), 0.8),   // forward
 *     DriveTasks.driveForSeconds(drivebase,
 *         new DriveSignal(0.0, 0.0, +0.7), 0.6)    // rotate CCW
 * );
 *
 * macroRunner.enqueue(macro);
 * }</pre>
 */
public final class DriveTasks {

    private DriveTasks() {
        // utility class; do not instantiate
    }

    // ------------------------------------------------------------------------
    // Timed drive
    // ------------------------------------------------------------------------

    /**
     * Create a {@link Task} that holds a given {@link DriveSignal} for a fixed
     * amount of time, then stops the drivebase.
     *
     * <p>Semantics:</p>
     * <ul>
     *   <li>On start:
     *     <ul>
     *       <li>Calls {@link MecanumDrivebase#drive(DriveSignal)} with the
     *           provided signal.</li>
     *     </ul>
     *   </li>
     *   <li>While running:
     *     <ul>
     *       <li>No further changes are made to the drivebase; your main robot
     *           loop is responsible for calling {@link MecanumDrivebase#update(LoopClock)}
     *           if needed.</li>
     *     </ul>
     *   </li>
     *   <li>After {@code durationSec} seconds:
     *     <ul>
     *       <li>Calls {@link MecanumDrivebase#stop()}.</li>
     *       <li>{@link Task#isComplete()} becomes {@code true}.</li>
     *     </ul>
     *   </li>
     * </ul>
     *
     * <p>This is a good fit for simple autonomous segments like “drive forward
     * for N seconds” when you don't yet have a trajectory or encoder-based
     * controller wired up.</p>
     *
     * @param drivebase   the drivebase to command
     * @param signal      the drive signal to hold (e.g., x, y, rot)
     * @param durationSec how long to hold the signal, in seconds (must be {@code >= 0})
     * @return a {@link Task} that runs the drive signal for a fixed time
     */
    public static Task driveForSeconds(final MecanumDrivebase drivebase,
                                       final DriveSignal signal,
                                       final double durationSec) {
        return new RunForSecondsTask(
                durationSec,
                // onStart: set the drive signal
                () -> drivebase.drive(signal),
                // onUpdate: no-op; drivebase.update(dt) is handled elsewhere
                null,
                // onFinish: stop the drive
                drivebase::stop
        );
    }

    // ------------------------------------------------------------------------
    // Instant drive helpers
    // ------------------------------------------------------------------------

    /**
     * Create a {@link Task} that sets a drive signal once and then finishes
     * immediately. This is a thin wrapper around {@link InstantTask}.
     *
     * <p>Useful when you want a one-shot drive step inside a larger sequence,
     * for example “snap to a heading” or “apply a braking mode,” while keeping
     * the code in terms of {@link Task}.</p>
     *
     * @param drivebase the drivebase to command
     * @param signal    the drive signal to apply
     * @return a {@link Task} that sets the drive signal once and then completes
     */
    public static Task driveInstant(final MecanumDrivebase drivebase,
                                    final DriveSignal signal) {
        return new InstantTask(() -> drivebase.drive(signal));
    }

    /**
     * Create a {@link Task} that stops the drivebase once and then finishes.
     *
     * <p>This keeps the “drive tasks” vocabulary consistent when used with
     * {@link edu.ftcphoenix.fw.task.SequenceTask} or a {@link edu.ftcphoenix.fw.task.TaskRunner}
     * in robot code and examples.</p>
     *
     * <pre>{@code
     * TaskRunner runner = new TaskRunner();
     * runner.enqueue(DriveTasks.stop(drivebase));
     * }</pre>
     *
     * @param drivebase the drivebase to stop
     * @return a {@link Task} that stops the drivebase once and then finishes
     */
    public static Task stop(final MecanumDrivebase drivebase) {
        return new InstantTask(drivebase::stop);
    }
}
