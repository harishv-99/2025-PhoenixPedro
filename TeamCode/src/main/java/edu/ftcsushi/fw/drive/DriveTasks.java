package edu.ftcsushi.fw.drive;

import java.util.Objects;

import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.task.AbstractTask;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;

/**
 * Creates the generic direct-sink drive {@link Task} used for short autonomous moves and testers.
 *
 * <p>The timed helper is explicitly <em>exclusive</em>: while it is active, no other behavior may
 * command the same {@link DriveCommandSink}. It is intended for Auto-style runners and focused
 * testers, not for the ordinary TeleOp path where a {@link DriveSource} and overlays propose one
 * command to a composition-root final writer. Higher-level movement normally belongs in route or
 * guidance Tasks.</p>
 *
 * <p>If an adapter's supported lifecycle requires updates beyond active Tasks, its composition root
 * continues calling {@link DriveCommandSink#update(LoopClock)} with the shared clock; the adapter
 * must deduplicate that call from the Task's same-cycle call. Pedro is one adapter that requires
 * this stable Auto-loop heartbeat.</p>
 *
 * <p>Ordinary managed Auto usage:</p>
 * <pre>{@code
 * Task nudgeForward = DriveTasks.driveExclusivelyForSeconds(
 *         autoDrive,
 *         new DriveSignal(0.20, 0.0, 0.0),
 *         0.15);
 *
 * program.rootTask(nudgeForward);
 * }</pre>
 *
 * <p>A focused tester or custom host may instead use an explicitly owned private runner.</p>
 */
public final class DriveTasks {

    private DriveTasks() {
        // utility class; do not instantiate
    }

    /**
     * Create an exclusive {@link Task} that refreshes a {@link DriveSignal} for a fixed amount of
     * time, then stops the drive sink.
     *
     * <p>The duration begins at this Task's own {@link LoopClock#nowSec()} start boundary. For every
     * active, unexpired cycle, including the start cycle, the Task calls
     * {@link DriveCommandSink#update(LoopClock)} once and, if it remains active after that callback,
     * calls {@link DriveCommandSink#drive(DriveSignal)} once. Repeated Task calls in the same
     * {@link LoopClock#cycle()} are ignored. A positive-duration command is therefore submitted to
     * the sink in its start cycle even when the preceding loop interval was longer than the
     * requested duration. A zero duration publishes no motion and stops immediately.</p>
     *
     * <p>The caller must ensure this Task is the only behavior-command writer for {@code sink}
     * while active. If an adapter requires updates beyond active Tasks, its composition root must
     * keep calling {@code update(clock)} with the shared clock and the adapter must make same-cycle
     * calls idempotent. Do not use this helper in an ordinary TeleOp loop that later writes a
     * {@link DriveSource} command to the same sink.</p>
     *
     * <p>The returned Task is single-use. Active cancellation stops the sink once; cancellation
     * before start and terminal or repeated cancellation are no-ops.</p>
     *
     * @param sink        the exclusively owned drive-command sink
     * @param signal      the drive signal to hold (robot-centric, normalized)
     * @param durationSec how long to hold the signal, in seconds; must be finite and
     *                    {@code >= 0}
     * @return a single-use exclusive timed-drive Task
     * @throws NullPointerException     if {@code sink} or {@code signal} is {@code null}
     * @throws IllegalArgumentException if {@code durationSec} is negative or non-finite
     */
    public static Task driveExclusivelyForSeconds(final DriveCommandSink sink,
                                                  final DriveSignal signal,
                                                  final double durationSec) {
        Objects.requireNonNull(sink, "sink must not be null");
        Objects.requireNonNull(signal, "signal must not be null");
        if (!Double.isFinite(durationSec) || durationSec < 0.0) {
            throw new IllegalArgumentException(
                    "durationSec must be finite and >= 0, got " + durationSec);
        }
        return new ExclusiveTimedDriveTask(sink, signal, durationSec);
    }

    /** Private timing policy; the shared lifecycle owns terminal cleanup and failure retention. */
    private static final class ExclusiveTimedDriveTask extends AbstractTask {
        private final DriveCommandSink sink;
        private final DriveSignal signal;
        private final double durationSec;
        private boolean commandCycleRecorded;
        private long lastCommandCycle;
        private double startSec;

        private ExclusiveTimedDriveTask(DriveCommandSink sink,
                                        DriveSignal signal,
                                        double durationSec) {
            super("DriveTasks.driveExclusivelyForSeconds(" + durationSec + ")");
            this.sink = sink;
            this.signal = signal;
            this.durationSec = durationSec;
        }

        @Override
        protected void onStart(LoopClock clock) {
            startSec = clock.nowSec();
            if (durationSec == 0.0) {
                complete(TaskOutcome.SUCCESS);
                return;
            }
            publishActiveCycle(clock);
        }

        @Override
        protected void onUpdate(LoopClock clock) {
            publishActiveCycle(clock);
        }

        @Override
        protected void onCancel() {
            // No separate resource: onFinish owns the one terminal sink-stop attempt.
        }

        @Override
        protected void onFinish() {
            sink.stop();
        }

        /** Refresh once in this cycle, preserving the command already published during start. */
        private void publishActiveCycle(LoopClock clock) {
            long cycle = clock.cycle();
            if (commandCycleRecorded && cycle == lastCommandCycle) {
                return;
            }
            commandCycleRecorded = true;
            lastCommandCycle = cycle;

            double elapsedSec = Math.max(0.0, clock.nowSec() - startSec);
            if (elapsedSec >= durationSec) {
                complete(TaskOutcome.SUCCESS);
                return;
            }

            sink.update(clock);
            // Cancellation from the update callback must not be followed by another drive write.
            if (!isActive()) {
                return;
            }
            sink.drive(signal);
        }
    }
}
