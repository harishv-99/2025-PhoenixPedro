package edu.ftcphoenix.fw.drive;

import java.util.Objects;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;

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
 * <p>Typical usage:</p>
 * <pre>{@code
 * Task nudgeForward = DriveTasks.driveExclusivelyForSeconds(
 *         autoDrive,
 *         new DriveSignal(0.20, 0.0, 0.0),
 *         0.15);
 *
 * runner.enqueue(nudgeForward);
 * }</pre>
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

    /** Private lifecycle state machine that enforces the exclusive timed-drive contract. */
    private static final class ExclusiveTimedDriveTask implements Task {
        private final DriveCommandSink sink;
        private final DriveSignal signal;
        private final double durationSec;

        private boolean startAttempted;
        private boolean started;
        private boolean complete;
        private boolean stopAttempted;
        private boolean commandCycleRecorded;
        private long lastCommandCycle;
        private double startSec;
        private TaskOutcome outcome = TaskOutcome.NOT_DONE;

        private ExclusiveTimedDriveTask(DriveCommandSink sink,
                                        DriveSignal signal,
                                        double durationSec) {
            this.sink = sink;
            this.signal = signal;
            this.durationSec = durationSec;
        }

        @Override
        public void start(LoopClock clock) {
            markStartAttempt();
            requireClock(clock);

            started = true;
            complete = false;
            outcome = TaskOutcome.NOT_DONE;
            startSec = clock.nowSec();

            if (durationSec == 0.0) {
                finish(TaskOutcome.SUCCESS);
                return;
            }

            publishActiveCycle(clock);
        }

        @Override
        public void update(LoopClock clock) {
            if (!started) {
                throw new IllegalStateException(getDebugName() + " cannot be updated before "
                        + "start(clock). Start it first, normally by enqueueing it in a "
                        + "TaskRunner.");
            }
            if (complete) {
                return;
            }
            requireClock(clock);
            publishActiveCycle(clock);
        }

        @Override
        public void cancel() {
            if (!started || complete) {
                return;
            }
            finish(TaskOutcome.CANCELLED);
        }

        @Override
        public boolean isComplete() {
            return complete;
        }

        @Override
        public TaskOutcome getOutcome() {
            return complete ? outcome : TaskOutcome.NOT_DONE;
        }

        @Override
        public String getDebugName() {
            return "DriveTasks.driveExclusivelyForSeconds(" + durationSec + ")";
        }

        /** Refresh the command at most once in this cycle, or finish before an expired write. */
        private void publishActiveCycle(LoopClock clock) {
            long cycle = clock.cycle();
            if (commandCycleRecorded && cycle == lastCommandCycle) {
                return;
            }

            // Record before callbacks so reentrant Task updates remain same-cycle no-ops.
            commandCycleRecorded = true;
            lastCommandCycle = cycle;

            double elapsedSec = Math.max(0.0, clock.nowSec() - startSec);
            if (elapsedSec >= durationSec) {
                finish(TaskOutcome.SUCCESS);
                return;
            }

            sink.update(clock);
            // The update hook may reentrantly cancel this Task and stop the sink.
            if (complete) {
                return;
            }
            sink.drive(signal);
        }

        /** Become terminal before attempting the exactly-once physical stop. */
        private void finish(TaskOutcome terminalOutcome) {
            if (complete) {
                return;
            }
            outcome = terminalOutcome;
            complete = true;
            stopOnce();
        }

        /** Attempt the sink stop once even when the stop callback throws or re-enters the Task. */
        private void stopOnce() {
            if (stopAttempted) {
                return;
            }
            stopAttempted = true;
            sink.stop();
        }

        /** Record the single permitted start attempt before any sink callback can run. */
        private void markStartAttempt() {
            if (startAttempted) {
                throw new IllegalStateException(getDebugName() + " is single-use and "
                        + "start(clock) was called more than once. Create a fresh Task by "
                        + "calling DriveTasks.driveExclusivelyForSeconds(...) again or rebuild "
                        + "the macro; use a Supplier<Task> for repeated scheduling.");
            }
            startAttempted = true;
        }

        /** Reject direct active lifecycle calls that omit Phoenix's shared loop clock. */
        private void requireClock(LoopClock clock) {
            if (clock == null) {
                throw new IllegalArgumentException(getDebugName()
                        + " requires a non-null LoopClock.");
            }
        }
    }
}
