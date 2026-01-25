package edu.ftcphoenix.fw.task;

import java.util.Objects;
import java.util.function.BooleanSupplier;

import edu.ftcphoenix.fw.util.LoopClock;

/**
 * Convenience factory and composition helpers for {@link Task} instances.
 *
 * <p>This is a thin utility layer on top of the core task implementations
 * such as {@link InstantTask}, {@link WaitUntilTask}, {@link RunForSecondsTask},
 * {@link SequenceTask}, and {@link ParallelAllTask}. The goal is to make
 * robot code more expressive and readable, without hiding the underlying
 * primitives.</p>
 *
 * <p>Typical usage:</p>
 *
 * <pre>{@code
 * Task auto = Tasks.sequence(
 *     // 1) Wait for shooter to be ready.
 *     Tasks.waitUntil(() -> shooterReady()),
 *
 *     // 2) Drive forward for 0.8 seconds (using DriveTasks).
 *     DriveTasks.driveForSeconds(drivebase, forwardSignal, 0.8),
 *
 *     // 3) Run intake for 0.5 seconds.
 *     Tasks.waitForSeconds(0.5)
 * );
 *
 * TaskRunner runner = new TaskRunner();
 * runner.enqueue(auto);
 * }</pre>
 */
public final class Tasks {

    private Tasks() {
        // utility class; do not instantiate
    }

    /**
     * Internal phase enum used by {@link #branchOnOutcome(Task, Task, Task)}’s
     * anonymous task. Declared at class scope to be Java 8 compatible.
     */
    private enum BranchPhase {
        MOVE,
        BRANCH,
        DONE
    }

    // ---------------------------------------------------------------------
    // Simple core tasks
    // ---------------------------------------------------------------------

    /**
     * A {@link Task} that does nothing and completes immediately.
     *
     * <p>Useful when a method must return a {@link Task} but in some cases
     * you do not want to perform any action:</p>
     *
     * <pre>{@code
     * return shouldShoot ? createShootMacro() : Tasks.noop();
     * }</pre>
     *
     * <p>Outcome semantics: this task reports
     * {@link TaskOutcome#SUCCESS} as soon as it is created, since there
     * is no failure mode.</p>
     */
    public static Task noop() {
        return new Task() {
            @Override
            public void start(LoopClock clock) {
                // no-op
            }

            @Override
            public void update(LoopClock clock) {
                // no-op
            }

            @Override
            public boolean isComplete() {
                return true;
            }

            @Override
            public TaskOutcome getOutcome() {
                // A no-op task is always considered a successful no-op.
                return TaskOutcome.SUCCESS;
            }
        };
    }

    /**
     * Create a {@link Task} that waits for a fixed amount of time.
     *
     * @param seconds duration in seconds; must be {@code >= 0}
     */
    public static Task waitForSeconds(double seconds) {
        return new RunForSecondsTask(seconds, null, null, null);
    }

    /**
     * Create a {@link Task} that waits until a condition becomes {@code true}.
     *
     * <p>This wraps {@link WaitUntilTask} with no timeout. If the condition
     * can get stuck, prefer {@link #waitUntil(BooleanSupplier, double)} or
     * construct a {@link WaitUntilTask} directly with a timeout.</p>
     *
     * @param condition condition to wait for
     */
    public static Task waitUntil(BooleanSupplier condition) {
        return new WaitUntilTask(condition);
    }

    /**
     * Create a {@link Task} that waits until a condition becomes {@code true},
     * but gives up if it takes longer than the given timeout.
     *
     * <p>This is a convenience overload for {@link WaitUntilTask} that exposes
     * the timeout in the {@code Tasks} facade. The returned task will report
     * {@link TaskOutcome#TIMEOUT} if the timeout elapses before the condition
     * becomes {@code true}.</p>
     *
     * @param condition  condition to wait for; must not be {@code null}
     * @param timeoutSec timeout in seconds; must be {@code >= 0.0}
     */
    public static Task waitUntil(BooleanSupplier condition, double timeoutSec) {
        return new WaitUntilTask(condition, timeoutSec);
    }

    /**
     * Create a {@link Task} that runs {@code action.run()} once when the task
     * starts and then completes immediately.
     *
     * @param action action to run once; must not be {@code null}
     */
    public static Task runOnce(Runnable action) {
        return new InstantTask(Objects.requireNonNull(action, "action is required"));
    }

    // ---------------------------------------------------------------------
    // Composition helpers
    // ---------------------------------------------------------------------

    /**
     * Create a {@link Task} that runs the given tasks one after another.
     *
     * <p>This is a thin wrapper around {@link SequenceTask#of(Task...)}.</p>
     *
     * @param tasks tasks to run in order; must not be {@code null} or contain nulls
     */
    public static Task sequence(Task... tasks) {
        return SequenceTask.of(tasks);
    }

    /**
     * Create a {@link Task} that runs the given tasks in parallel and completes
     * only when <b>all</b> have completed.
     *
     * <p>This is a thin wrapper around {@link ParallelAllTask#of(Task...)}.</p>
     *
     * @param tasks tasks to run in parallel; must not be {@code null} or contain nulls
     */
    public static Task parallelAll(Task... tasks) {
        return ParallelAllTask.of(tasks);
    }

    // ---------------------------------------------------------------------
    // Outcome-aware helpers
    // ---------------------------------------------------------------------

    /**
     * Create a {@link Task} that:
     * <ol>
     *   <li>Runs the given {@link Task} until it completes, then</li>
     *   <li>Runs either {@code onSuccess} or {@code onTimeout} depending on
     *       the {@link TaskOutcome} reported by {@link Task#getOutcome()} on
     *       the {@code move} task.</li>
     * </ol>
     *
     * <p>Outcome semantics for the returned task:</p>
     * <ul>
     *   <li>While it is still running (move or branch phase), the outcome
     *       mirrors the currently active child task's outcome.</li>
     *   <li>Once the chosen branch has completed and the wrapper is done,
     *       {@link Task#getOutcome()} returns <b>the chosen branch's outcome</b>.
     *       The initial {@code move} outcome is used only to decide which
     *       branch to execute; it does not directly drive the wrapper's
     *       final outcome.</li>
     * </ul>
     *
     * <p>This makes {@code branchOnOutcome} behave like a structured
     * "try/handle-timeout" block: a timeout in {@code move} is <em>handled</em>
     * by running {@code onTimeout}, and from the outside, what matters is
     * whether that timeout-handling branch ultimately succeeded or not.</p>
     *
     * @param move      the task to run first
     * @param onSuccess task to run if the move succeeds or completes normally
     * @param onTimeout task to run if the move ends with {@link TaskOutcome#TIMEOUT}
     */
    public static Task branchOnOutcome(final Task move,
                                       final Task onSuccess,
                                       final Task onTimeout) {

        Objects.requireNonNull(move, "move is required");
        Objects.requireNonNull(onSuccess, "onSuccess is required");
        Objects.requireNonNull(onTimeout, "onTimeout is required");

        return new Task() {
            private BranchPhase phase = BranchPhase.MOVE;
            private Task current = move;

            private TaskOutcome branchOutcome = TaskOutcome.UNKNOWN;

            @Override
            public void start(LoopClock clock) {
                current.start(clock);
            }

            @Override
            public void update(LoopClock clock) {
                if (phase == BranchPhase.DONE) {
                    return;
                }

                current.update(clock);

                if (!current.isComplete()) {
                    return;
                }

                switch (phase) {
                    case MOVE:
                        // Decide which branch to run based on the move's outcome.
                        TaskOutcome moveOutcome = move.getOutcome();
                        if (moveOutcome == TaskOutcome.TIMEOUT) {
                            current = onTimeout;
                        } else {
                            current = onSuccess;
                        }
                        current.start(clock);
                        phase = BranchPhase.BRANCH;
                        break;

                    case BRANCH:
                        // Finished running the chosen branch.
                        branchOutcome = current.getOutcome();
                        phase = BranchPhase.DONE;
                        break;

                    default:
                        break;
                }
            }

            @Override
            public boolean isComplete() {
                return phase == BranchPhase.DONE;
            }

            @Override
            public TaskOutcome getOutcome() {
                switch (phase) {
                    case MOVE:
                    case BRANCH:
                        // While executing, mirror the active child.
                        return current.getOutcome();

                    case DONE:
                    default:
                        // Once done, report the branch's outcome. Any timeout
                        // in the move phase is considered "handled" by the
                        // chosen branch.
                        return branchOutcome;
                }
            }

            @Override
            public String getDebugName() {
                return "BranchOnOutcome(" + move.getDebugName() + ")";
            }
        };
    }
}
