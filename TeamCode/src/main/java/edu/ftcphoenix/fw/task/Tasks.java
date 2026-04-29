package edu.ftcphoenix.fw.task;

import java.util.Objects;
import java.util.function.BooleanSupplier;

import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.time.LoopClock;

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
            /** {@inheritDoc} */
            @Override
            public void start(LoopClock clock) {
                // no-op
            }

            /** {@inheritDoc} */
            @Override
            public void update(LoopClock clock) {
                // no-op
            }

            /** {@inheritDoc} */
            @Override
            public boolean isComplete() {
                return true;
            }

            /** {@inheritDoc} */
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
     * can get stuck, prefer {@link #waitUntil(BooleanSource, double)} or
     * construct a {@link WaitUntilTask} directly with a timeout.</p>
     *
     * @param condition condition to wait for
     */
    public static Task waitUntil(BooleanSource condition) {
        return new WaitUntilTask(condition);
    }

    /**
     * Convenience overload: wrap a raw BooleanSupplier into a BooleanSource.
     *
     * <p>Prefer using a BooleanSource directly for sensor gates and debounced/hysteresis signals.
     */
    public static Task waitUntil(BooleanSupplier condition) {
        return waitUntil(BooleanSource.of(condition));
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
    public static Task waitUntil(BooleanSource condition, double timeoutSec) {
        return new WaitUntilTask(condition, timeoutSec);
    }

    /**
     * Convenience overload: wrap a raw BooleanSupplier into a BooleanSource.
     */
    public static Task waitUntil(BooleanSupplier condition, double timeoutSec) {
        return waitUntil(BooleanSource.of(condition), timeoutSec);
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
    // Output tasks & queues
    // ---------------------------------------------------------------------

    /**
     * Create a new {@link OutputTaskRunner} with the given idle output.
     *
     * <p>Output queues are intended to be applied to a Plant (or other output sink) from your
     * subsystem loop. The queue itself is a {@link edu.ftcphoenix.fw.core.source.ScalarSource}.</p>
     *
     * @param idleOutput value returned when no output task is active
     */
    public static OutputTaskRunner outputQueue(double idleOutput) {
        return new OutputTaskRunner(idleOutput);
    }

    /**
     * Convenience: create an {@link OutputTaskRunner} with idle output = 0.
     */
    public static OutputTaskRunner outputQueue() {
        return new OutputTaskRunner(0.0);
    }

    /**
     * Start a guided builder for a reusable output pulse recipe.
     *
     * <p>The builder asks the common robot questions in order: when may the pulse start, what
     * output should it produce, how should it end, and whether it needs a cooldown. It returns an
     * {@link OutputTaskFactory} because queued pulses must be fresh task instances.</p>
     *
     * <pre>{@code
     * OutputTaskFactory feedOne = Tasks.outputPulse("feedOne")
     *         .startWhen(canShootNow)
     *         .runOutput(0.90)
     *         .forSeconds(0.12)
     *         .cooldownSec(0.05)
     *         .build();
     *
     * feederQueue.whileHigh(clock, shootHeld, 1, feedOne);
     * }</pre>
     *
     * @param name debug label used by created tasks
     * @return first pulse-builder step
     */
    public static OutputPulseStartStep outputPulse(String name) {
        return new OutputPulseBuilder(name);
    }

    /**
     * First output-pulse builder step: choose the start gate.
     */
    public interface OutputPulseStartStep {
        /**
         * Wait in the task's idle phase until {@code startWhen} is high.
         */
        OutputPulseOutputStep startWhen(BooleanSource startWhen);

        /**
         * Start the pulse as soon as the task begins running.
         */
        OutputPulseOutputStep startImmediately();
    }

    /**
     * Second output-pulse builder step: choose the output while the pulse is running.
     */
    public interface OutputPulseOutputStep {
        /**
         * Use a constant scalar output while the pulse is running.
         */
        OutputPulseEndStep runOutput(double output);

        /**
         * Use a source-shaped scalar output while the pulse is running.
         */
        OutputPulseEndStep runOutput(ScalarSource output);
    }

    /**
     * Third output-pulse builder step: choose how the pulse ends.
     */
    public interface OutputPulseEndStep {
        /**
         * Run for exactly {@code durationSec} seconds after the start gate opens.
         */
        OutputPulseReadyStep forSeconds(double durationSec);

        /**
         * Run until {@code doneWhen} is high, with a required max-run safety cap.
         */
        OutputPulseUntilStep until(BooleanSource doneWhen);
    }

    /**
     * Sensor-ended pulse step before the required max-run cap has been supplied.
     */
    public interface OutputPulseUntilStep {
        /**
         * Require the pulse to run for at least {@code minRunSec}, then require {@link #maxRunSec(double)}.
         */
        OutputPulseUntilMaxStep minRunSec(double minRunSec);

        /**
         * Set the max-run safety cap with an implicit minimum run time of zero.
         */
        OutputPulseReadyStep maxRunSec(double maxRunSec);

        /**
         * Set both minimum and maximum run times at once.
         */
        OutputPulseReadyStep runWindow(double minRunSec, double maxRunSec);
    }

    /**
     * Sensor-ended pulse step after the minimum run time has been supplied.
     */
    public interface OutputPulseUntilMaxStep {
        /**
         * Set the max-run safety cap and continue to optional idle/cooldown choices.
         */
        OutputPulseReadyStep maxRunSec(double maxRunSec);
    }

    /**
     * Final output-pulse builder step: optional idle/cooldown choices, then build.
     */
    public interface OutputPulseReadyStep {
        /**
         * Output returned while waiting, cooling down, or idle after completion. Defaults to 0.0.
         */
        OutputPulseReadyStep idleOutput(double idleOutput);

        /**
         * Add a cooldown phase after the pulse finishes. Defaults to 0.0 seconds.
         */
        OutputPulseReadyStep cooldownSec(double cooldownSec);

        /**
         * Build a reusable factory that creates a fresh output task each time.
         */
        OutputTaskFactory build();

        /**
         * Build one fresh task immediately for one-shot enqueueing.
         */
        OutputTask buildTask();
    }

    private static final class OutputPulseBuilder implements OutputPulseStartStep,
            OutputPulseOutputStep, OutputPulseEndStep, OutputPulseUntilStep,
            OutputPulseUntilMaxStep, OutputPulseReadyStep {
        private final String name;
        private BooleanSource startWhen;
        private ScalarSource runOutput;
        private BooleanSource doneWhen;
        private double idleOutput = 0.0;
        private double minRunSec = 0.0;
        private double maxRunSec = Double.NaN;
        private double cooldownSec = 0.0;

        OutputPulseBuilder(String name) {
            this.name = (name == null || name.trim().isEmpty()) ? "outputPulse" : name.trim();
        }

        @Override
        public OutputPulseOutputStep startWhen(BooleanSource startWhen) {
            this.startWhen = Objects.requireNonNull(startWhen, "startWhen");
            return this;
        }

        @Override
        public OutputPulseOutputStep startImmediately() {
            return startWhen(BooleanSource.constant(true));
        }

        @Override
        public OutputPulseEndStep runOutput(double output) {
            return runOutput(ScalarSource.constant(output));
        }

        @Override
        public OutputPulseEndStep runOutput(ScalarSource output) {
            this.runOutput = Objects.requireNonNull(output, "output");
            return this;
        }

        @Override
        public OutputPulseReadyStep forSeconds(double durationSec) {
            requireNonNegativeFinite(durationSec, "durationSec");
            this.doneWhen = BooleanSource.constant(true);
            this.minRunSec = durationSec;
            this.maxRunSec = durationSec;
            return this;
        }

        @Override
        public OutputPulseUntilStep until(BooleanSource doneWhen) {
            this.doneWhen = Objects.requireNonNull(doneWhen, "doneWhen");
            this.minRunSec = 0.0;
            this.maxRunSec = Double.NaN;
            return this;
        }

        @Override
        public OutputPulseUntilMaxStep minRunSec(double minRunSec) {
            requireNonNegativeFinite(minRunSec, "minRunSec");
            this.minRunSec = minRunSec;
            return this;
        }

        @Override
        public OutputPulseReadyStep maxRunSec(double maxRunSec) {
            requireNonNegativeFinite(maxRunSec, "maxRunSec");
            if (maxRunSec < minRunSec) {
                throw new IllegalArgumentException("maxRunSec must be >= minRunSec, got max=" + maxRunSec + " min=" + minRunSec);
            }
            this.maxRunSec = maxRunSec;
            return this;
        }

        @Override
        public OutputPulseReadyStep runWindow(double minRunSec, double maxRunSec) {
            this.minRunSec(minRunSec);
            return this.maxRunSec(maxRunSec);
        }

        @Override
        public OutputPulseReadyStep idleOutput(double idleOutput) {
            if (!Double.isFinite(idleOutput)) {
                throw new IllegalArgumentException("idleOutput must be finite, got " + idleOutput);
            }
            this.idleOutput = idleOutput;
            return this;
        }

        @Override
        public OutputPulseReadyStep cooldownSec(double cooldownSec) {
            requireNonNegativeFinite(cooldownSec, "cooldownSec");
            this.cooldownSec = cooldownSec;
            return this;
        }

        @Override
        public OutputTaskFactory build() {
            final String taskName = name;
            final BooleanSource start = Objects.requireNonNull(startWhen, "startWhen");
            final BooleanSource done = Objects.requireNonNull(doneWhen, "doneWhen");
            final ScalarSource output = Objects.requireNonNull(runOutput, "runOutput");
            final double idle = idleOutput;
            final double min = minRunSec;
            final double max = maxRunSec;
            final double cooldown = cooldownSec;
            if (!Double.isFinite(max)) {
                throw new IllegalStateException("Output pulse requires forSeconds(...) or until(...).maxRunSec(...)");
            }
            return () -> new GatedOutputUntilTask(taskName, start, done, output, idle, min, max, cooldown);
        }

        @Override
        public OutputTask buildTask() {
            return build().create();
        }

        private static void requireNonNegativeFinite(double value, String name) {
            if (value < 0.0 || !Double.isFinite(value)) {
                throw new IllegalArgumentException(name + " must be finite and >= 0, got " + value);
            }
        }
    }

    /**
     * Output a constant value for a fixed duration.
     */
    public static OutputTask outputForSeconds(String name, double output, double durationSec) {
        return new OutputForSecondsTask(name, output, durationSec);
    }

    /**
     * Wait for {@code startWhen}, then output {@code runOutput} until {@code doneWhen} is satisfied
     * (and {@code minRunSec} has elapsed), or until {@code maxRunSec} elapses.
     */
    public static OutputTask gatedOutputUntil(String name,
                                              BooleanSource startWhen,
                                              BooleanSource doneWhen,
                                              ScalarSource runOutput,
                                              double idleOutput,
                                              double minRunSec,
                                              double maxRunSec,
                                              double cooldownSec) {

        Objects.requireNonNull(startWhen, "startWhen");
        Objects.requireNonNull(doneWhen, "doneWhen");
        Objects.requireNonNull(runOutput, "runOutput");

        return new GatedOutputUntilTask(name, startWhen, doneWhen, runOutput, idleOutput, minRunSec, maxRunSec, cooldownSec);
    }

    /**
     * Convenience overload for constant run output and common defaults.
     */
    public static OutputTask gatedOutputUntil(String name,
                                              BooleanSource startWhen,
                                              BooleanSource doneWhen,
                                              double runOutput,
                                              double minRunSec,
                                              double maxRunSec) {

        return gatedOutputUntil(name,
                startWhen,
                doneWhen,
                ScalarSource.constant(runOutput),
                0.0,
                minRunSec,
                maxRunSec,
                0.0);
    }

    /**
     * Convenience overload: output while a condition is true, up to a timeout.
     */
    public static OutputTask outputUntil(String name,
                                         BooleanSource doneWhen,
                                         double runOutput,
                                         double maxRunSec) {

        return gatedOutputUntil(name,
                BooleanSource.constant(true),
                doneWhen,
                ScalarSource.constant(runOutput),
                0.0,
                0.0,
                maxRunSec,
                0.0);
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

            /** {@inheritDoc} */
            @Override
            public void start(LoopClock clock) {
                phase = BranchPhase.MOVE;
                current = move;
                branchOutcome = TaskOutcome.UNKNOWN;
                current.start(clock);
            }

            /** {@inheritDoc} */
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
                            current.start(clock);
                            phase = BranchPhase.BRANCH;
                        } else if (moveOutcome == TaskOutcome.CANCELLED) {
                            branchOutcome = TaskOutcome.CANCELLED;
                            phase = BranchPhase.DONE;
                        } else {
                            current = onSuccess;
                            current.start(clock);
                            phase = BranchPhase.BRANCH;
                        }
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

            /**
             * {@inheritDoc}
             */
            @Override
            public void cancel() {
                if (phase == BranchPhase.DONE) {
                    return;
                }
                if (current != null && !current.isComplete()) {
                    current.cancel();
                }
                branchOutcome = TaskOutcome.CANCELLED;
                phase = BranchPhase.DONE;
            }

            /** {@inheritDoc} */
            @Override
            public boolean isComplete() {
                return phase == BranchPhase.DONE;
            }

            /** {@inheritDoc} */
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

            /** {@inheritDoc} */
            @Override
            public String getDebugName() {
                return "BranchOnOutcome(" + move.getDebugName() + ")";
            }
        };
    }
}
