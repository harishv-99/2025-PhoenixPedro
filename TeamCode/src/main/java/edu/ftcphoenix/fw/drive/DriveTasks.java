package edu.ftcphoenix.fw.drive;

import edu.ftcphoenix.fw.task.InstantTask;
import edu.ftcphoenix.fw.task.RunForSecondsTask;
import edu.ftcphoenix.fw.task.Task;

/**
 * Small helper class for creating common drive-related {@link Task} patterns.
 *
 * <p>This class intentionally contains only <b>generic</b> drive helpers (timed drive,
 * stop, etc.). Higher-level behaviors like go-to-pose live in their own modules
 * (see {@code edu.ftcphoenix.fw.drive.guidance.GoToPoseTasks}).</p>
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
     * @param drivebase   the drivebase to command
     * @param signal      the drive signal to hold (robot-centric, normalized)
     * @param durationSec how long to hold the signal, in seconds (must be {@code >= 0})
     * @return a {@link Task} that runs the drive signal for a fixed time
     */
    public static Task driveForSeconds(final MecanumDrivebase drivebase,
                                       final DriveSignal signal,
                                       final double durationSec) {
        return new RunForSecondsTask(
                durationSec,
                // onStart: apply drive signal
                () -> drivebase.drive(signal),
                // onUpdate: no-op
                null,
                // onFinish: stop
                drivebase::stop
        );
    }

    // ------------------------------------------------------------------------
    // Instant drive helpers
    // ------------------------------------------------------------------------

    /**
     * Create a {@link Task} that applies a drive signal once and then finishes immediately.
     */
    public static Task driveInstant(final MecanumDrivebase drivebase,
                                    final DriveSignal signal) {
        return new InstantTask(() -> drivebase.drive(signal));
    }

    /**
     * Create a {@link Task} that stops the drivebase once and then finishes.
     */
    public static Task stop(final MecanumDrivebase drivebase) {
        return new InstantTask(drivebase::stop);
    }
}
