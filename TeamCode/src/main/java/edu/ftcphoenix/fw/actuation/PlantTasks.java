package edu.ftcphoenix.fw.actuation;

import java.util.Objects;

import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;
import edu.ftcphoenix.fw.util.LoopClock;

/**
 * Helper methods and builders for creating common {@link Task} patterns
 * that command a {@link Plant}.
 *
 * <p>The goal is to make robot code read like:</p>
 *
 * <pre>{@code
 * // Intake at full power for 0.7 seconds, then stop.
 * Task intakePulse = PlantTasks.holdForThen(intakePlant, +1.0, 0.7, 0.0);
 *
 * // Arm: move to an angle (feedback plant) and wait until atSetpoint() or timeout.
 * Task moveArm = PlantTasks.moveTo(
 *     armPlant,
 *     Math.toRadians(45.0),
 *     1.5
 * );
 * }</pre>
 *
 * <p>Helpers that rely on {@link Plant#atSetpoint()} (for example,
 * {@link #moveTo(Plant, double)}, {@link #moveTo(Plant, double, double)}, and
 * {@link #moveToThen(Plant, double, double, double)}) require a
 * <b>feedback-capable</b> plant where {@link Plant#hasFeedback()} returns
 * {@code true}. In practice this usually means plants created from DC motors
 * using the {@code Actuators.plant(...).motor(...).position(...)} or
 * {@code Actuators.plant(...).motor(...).velocity(...)} paths.</p>
 *
 * <p>Time-based helpers such as {@link #holdFor(Plant, double, double)} and
 * {@link #holdForThen(Plant, double, double, double)} only care about time and
 * work with both feedback and open-loop plants (for example, servo position or
 * power-only plants where {@code hasFeedback() == false}).</p>
 *
 * <p>All helpers here are <b>non-blocking</b> and are intended to be used with
 * {@link edu.ftcphoenix.fw.task.TaskRunner} and the rest of the {@code fw.task}
 * package. These tasks set targets on plants and rely on some other mechanism
 * to call {@link Plant#update(double)} each loop.</p>
 */
public final class PlantTasks {

    private PlantTasks() {
        // Utility class; do not instantiate.
    }

    // ------------------------------------------------------------------------
    // Feedback-based move-to-setpoint helpers (student-friendly)
    // ------------------------------------------------------------------------

    /**
     * Create a {@link Task} that:
     * <ol>
     *   <li>Sets the plant's target once at start.</li>
     *   <li>Finishes when {@link Plant#atSetpoint()} becomes {@code true}.</li>
     *   <li>Leaves the plant holding that target.</li>
     * </ol>
     *
     * <p><b>Only use this with feedback-capable plants</b> where
     * {@link Plant#hasFeedback()} returns {@code true} and {@link Plant#atSetpoint()}
     * has a meaningful implementation (for example, velocity plants or motor
     * position plants). At runtime this method will throw an
     * {@link IllegalStateException} if {@code plant.hasFeedback() == false}.</p>
     *
     * @param plant  plant to command (must be feedback-capable)
     * @param target target setpoint (e.g., angle, position, velocity)
     * @return a {@link Task} that moves to the setpoint and then holds it
     */
    public static Task moveTo(final Plant plant,
                              final double target) {
        Objects.requireNonNull(plant, "plant is required");
        ensureFeedbackPlantForMove(plant, "moveTo(plant, target)");
        return configureTask(plant, target)
                .waitForSetpoint()
                .thenHold()
                .build();
    }

    /**
     * Create a {@link Task} that:
     * <ol>
     *   <li>Sets the plant's target once at start.</li>
     *   <li>Finishes when {@link Plant#atSetpoint()} becomes {@code true}, or
     *       when {@code timeoutSec} seconds elapse (whichever comes first).</li>
     *   <li>Leaves the plant holding that target.</li>
     * </ol>
     *
     * <p>If the timeout elapses before {@code atSetpoint()} is true, the
     * task's {@link Task#getOutcome()} will report {@link TaskOutcome#TIMEOUT}.
     * Otherwise it reports {@link TaskOutcome#SUCCESS}.</p>
     *
     * <p><b>Only use this with feedback-capable plants</b> where
     * {@link Plant#hasFeedback()} returns {@code true} and {@link Plant#atSetpoint()}
     * has a meaningful implementation (for example, velocity plants or motor
     * position plants). At runtime this method will throw an
     * {@link IllegalStateException} if {@code plant.hasFeedback() == false}.</p>
     *
     * @param plant      plant to command (must be feedback-capable)
     * @param target     target setpoint
     * @param timeoutSec timeout in seconds; must be {@code > 0}
     * @return a {@link Task} that moves to the setpoint with a timeout
     */
    public static Task moveTo(final Plant plant,
                              final double target,
                              final double timeoutSec) {
        Objects.requireNonNull(plant, "plant is required");
        ensureFeedbackPlantForMove(plant, "moveTo(plant, target, timeoutSec)");
        if (timeoutSec <= 0.0) {
            throw new IllegalArgumentException(
                    "timeoutSec must be > 0, got " + timeoutSec);
        }
        return configureTask(plant, target)
                .waitForSetpointOrTimeout(timeoutSec)
                .thenHold()
                .build();
    }

    /**
     * Create a {@link Task} that:
     * <ol>
     *   <li>Sets the plant's target once at start.</li>
     *   <li>Finishes when {@link Plant#atSetpoint()} becomes {@code true}, or
     *       when {@code timeoutSec} seconds elapse (whichever comes first).</li>
     *   <li>Then sets the plant's target once to {@code finalTarget} as a
     *       follow-up.</li>
     * </ol>
     *
     * <p>This is useful for patterns like “move arm to scoring position, then
     * go back to travel/stowed position”. Note that the final set to
     * {@code finalTarget} is a <b>single</b> {@link Plant#setTarget(double)}
     * call; this task does not wait for a second move to complete.</p>
     *
     * <p><b>Only use this with feedback-capable plants</b> where
     * {@link Plant#hasFeedback()} returns {@code true} and {@link Plant#atSetpoint()}
     * has a meaningful implementation. At runtime this method will throw an
     * {@link IllegalStateException} if {@code plant.hasFeedback() == false}.</p>
     *
     * @param plant       plant to command (must be feedback-capable)
     * @param target      intermediate target setpoint
     * @param timeoutSec  timeout in seconds; must be {@code > 0}
     * @param finalTarget target to apply once the move is complete or times out
     * @return a {@link Task} that moves to the setpoint then sets finalTarget
     */
    public static Task moveToThen(final Plant plant,
                                  final double target,
                                  final double timeoutSec,
                                  final double finalTarget) {
        Objects.requireNonNull(plant, "plant is required");
        ensureFeedbackPlantForMove(plant, "moveToThen(plant, target, timeoutSec, finalTarget)");
        if (timeoutSec <= 0.0) {
            throw new IllegalArgumentException(
                    "timeoutSec must be > 0, got " + timeoutSec);
        }

        return configureTask(plant, target)
                .waitForSetpointOrTimeout(timeoutSec)
                .then(finalTarget)
                .build();
    }

    /**
     * Create a {@link Task} that:
     * <ol>
     *   <li>Sets the plant's target once at start.</li>
     *   <li>Finishes when {@link Plant#atSetpoint()} becomes {@code true}.</li>
     *   <li>Then sets the plant's target once to {@code finalTarget} as a
     *       follow-up.</li>
     * </ol>
     *
     * <p>This is the no-timeout variant of
     * {@link #moveToThen(Plant, double, double, double)}. The task will wait
     * indefinitely for {@link Plant#atSetpoint()} to become {@code true}. If
     * you want to guard against a stuck mechanism, use the overload that accepts
     * {@code timeoutSec} instead.</p>
     *
     * @param plant       plant to command (must be feedback-capable)
     * @param target      target setpoint
     * @param finalTarget target value to apply once the initial setpoint is
     *                    reached
     * @return a {@link Task} that moves to the setpoint and then sets
     *         {@code finalTarget}
     */
    public static Task moveToThen(final Plant plant,
                                  final double target,
                                  final double finalTarget) {
        Objects.requireNonNull(plant, "plant is required");
        ensureFeedbackPlantForMove(plant, "moveToThen(plant, target, finalTarget)");
        return configureTask(plant, target)
                .waitForSetpoint()
                .then(finalTarget)
                .build();
    }

    private static void ensureFeedbackPlantForMove(Plant plant, String methodName) {
        if (!plant.hasFeedback()) {
            throw new IllegalStateException(
                    "PlantTasks." + methodName + " requires a feedback-capable plant "
                            + "(plant.hasFeedback() == true). For open-loop plants "
                            + "(e.g., servos or power-only outputs), use holdFor(...), "
                            + "holdForThen(...), or setInstant(...) instead."
            );
        }
    }

    // ------------------------------------------------------------------------
    // Timed hold patterns (simple helpers backed by the builder)
    // ------------------------------------------------------------------------

    /**
     * Create a {@link Task} that:
     * <ol>
     *   <li>Sets the plant target once at the start.</li>
     *   <li>Relies on your main loop / mechanism to call
     *       {@link Plant#update(double)} each iteration.</li>
     *   <li>Holds that target for a fixed duration (purely by time).</li>
     *   <li>After the duration elapses, sets a follow-up target.</li>
     * </ol>
     *
     * <p>This is implemented using the builder:</p>
     *
     * <pre>{@code
     * PlantTasks.configureTask(plant, target)
     *     .waitFor(durationSec)
     *     .then(finalTarget)
     *     .build();
     * }</pre>
     *
     * @param plant       the plant to command
     * @param target      target value to hold during the timed interval
     * @param durationSec duration in seconds; must be {@code >= 0}
     * @param finalTarget target value to apply once the time elapses
     * @return a {@link Task} that performs the timed hold then sets finalTarget
     */
    public static Task holdForThen(final Plant plant,
                                   final double target,
                                   final double durationSec,
                                   final double finalTarget) {
        Objects.requireNonNull(plant, "plant is required");

        return configureTask(plant, target)
                .waitFor(durationSec)
                .then(finalTarget)
                .build();
    }

    /**
     * Create a {@link Task} that:
     * <ol>
     *   <li>Sets the plant target once at the start.</li>
     *   <li>Relies on your main loop / mechanism to call
     *       {@link Plant#update(double)} each iteration.</li>
     *   <li>Holds that target for a fixed duration (purely by time).</li>
     *   <li>After the duration elapses, leaves the plant holding that target.</li>
     * </ol>
     *
     * <p>Internally this is just {@link #holdForThen(Plant, double, double, double)}
     * with {@code finalTarget == target}.</p>
     *
     * @param plant       the plant to command
     * @param target      target value to hold during the timed interval
     * @param durationSec duration in seconds; must be {@code >= 0}
     * @return a {@link Task} that performs the timed hold and leaves the target
     * at the commanded value once the time has elapsed
     */
    public static Task holdFor(final Plant plant,
                               final double target,
                               final double durationSec) {
        return holdForThen(plant, target, durationSec, target);
    }

    // ------------------------------------------------------------------------
    // Instant target set (simple helper backed by the builder)
    // ------------------------------------------------------------------------

    /**
     * Create a {@link Task} that:
     * <ol>
     *   <li>Sets the plant's target once at start.</li>
     *   <li>Relies on your main loop / mechanism to call
     *       {@link Plant#update(double)} each iteration.</li>
     *   <li>Completes immediately (in a single iteration), leaving the plant
     *       holding that target.</li>
     * </ol>
     *
     * <p>Internally this is implemented via the builder as:</p>
     *
     * <pre>{@code
     * PlantTasks.configureTask(plant, target)
     *     .instant()
     *     .build();
     * }</pre>
     *
     * @param plant  plant to command
     * @param target target value
     * @return a {@link Task} that sets the target once and then completes
     */
    public static Task setInstant(final Plant plant,
                                  final double target) {
        Objects.requireNonNull(plant, "plant is required");
        return configureTask(plant, target)
                .instant()
                .build();
    }

    // ------------------------------------------------------------------------
    // Builder API (advanced / customizable)
    // ------------------------------------------------------------------------

    /**
     * Begin configuring a {@link Task} for a plant that will start by calling
     * {@link Plant#setTarget(double)} with {@code initialTarget}, and then
     * use a configurable completion and post behavior.
     *
     * <p>The builder lets you choose:</p>
     * <ul>
     *   <li>How the task decides it is complete:
     *     <ul>
     *       <li>{@link TargetTaskStart#waitForSetpoint()}</li>
     *       <li>{@link TargetTaskStart#waitForSetpointOrTimeout(double)}</li>
     *       <li>{@link TargetTaskStart#waitFor(double)}</li>
     *       <li>{@link TargetTaskStart#instant()}</li>
     *     </ul>
     *   </li>
     *   <li>What happens to the plant's target once the task is complete
     *       (for the non-instant completion modes):
     *     <ul>
     *       <li>{@link TargetTaskPost#thenHold()}</li>
     *       <li>{@link TargetTaskPost#then(double)}</li>
     *     </ul>
     *   </li>
     * </ul>
     *
     * <p>For common cases, prefer the simpler helpers:
     * {@link #setInstant(Plant, double)},
     * {@link #holdFor(Plant, double, double)},
     * {@link #holdForThen(Plant, double, double, double)},
     * {@link #moveTo(Plant, double)},
     * {@link #moveTo(Plant, double, double)},
     * and {@link #moveToThen(Plant, double, double, double)}.</p>
     *
     * @param plant         plant to command
     * @param initialTarget target value to apply when the task starts
     * @return the first stage of the builder
     */
    public static TargetTaskStart configureTask(final Plant plant,
                                                final double initialTarget) {
        Objects.requireNonNull(plant, "plant is required");
        return new TargetTaskBuilder(plant, initialTarget);
    }

    /**
     * First builder stage: choose how the task decides it is complete.
     */
    public interface TargetTaskStart {
        /**
         * Complete when {@link Plant#atSetpoint()} first becomes true.
         *
         * <p>This requires a feedback-capable plant where
         * {@link Plant#hasFeedback()} returns {@code true}. Implementations
         * will throw an {@link IllegalStateException} if the plant does not
         * advertise feedback.</p>
         */
        TargetTaskPost waitForSetpoint();

        /**
         * Complete when {@link Plant#atSetpoint()} becomes true, or when the
         * given timeout elapses, whichever happens first.
         *
         * <p>This requires a feedback-capable plant where
         * {@link Plant#hasFeedback()} returns {@code true}. Implementations
         * will throw an {@link IllegalStateException} if the plant does not
         * advertise feedback.</p>
         *
         * @param timeoutSec timeout in seconds; must be {@code > 0}
         */
        TargetTaskPost waitForSetpointOrTimeout(double timeoutSec);

        /**
         * Complete after a fixed amount of time has elapsed.
         *
         * @param seconds duration in seconds; must be {@code >= 0}
         */
        TargetTaskPost waitFor(double seconds);

        /**
         * Complete immediately (in a single iteration).
         */
        TargetTaskBuild instant();
    }

    /**
     * Second builder stage: choose what happens to the plant's target once the
     * task is complete.
     */
    public interface TargetTaskPost {
        /**
         * Leave the plant holding the last target value that was sent to it.
         *
         * <p>For example, if the task was created with
         * {@link TargetTaskStart#waitForSetpoint()} and the plant is a velocity
         * plant, the plant will keep holding the velocity that was set when the
         * task started.</p>
         */
        TargetTaskBuild thenHold();

        /**
         * Set the plant's target once to a new value after the task is complete.
         *
         * <p>For example, you can wait for a shooter flywheel to spin up to a
         * velocity, then once that is done set the velocity to zero.</p>
         *
         * @param finalTarget target value to apply once the task is complete
         */
        TargetTaskBuild then(double finalTarget);
    }

    /**
     * Final builder stage.
     */
    public interface TargetTaskBuild {
        /**
         * Build a {@link Task} implementing the configured behavior.
         */
        Task build();
    }

    private enum CompletionMode {
        INSTANT,
        WAIT_SETPOINT,
        WAIT_TIME,
        WAIT_SETPOINT_OR_TIMEOUT
    }

    private enum PostBehavior {
        HOLD,
        FINAL_TARGET
    }

    private static final class TargetTaskBuilder
            implements TargetTaskStart, TargetTaskPost, TargetTaskBuild {

        private final Plant plant;
        private final double initialTarget;

        private CompletionMode completionMode = CompletionMode.INSTANT;
        private double waitSeconds = 0.0;
        private double timeoutSec = 0.0;

        private PostBehavior postBehavior = PostBehavior.HOLD;
        private double finalTarget = 0.0;

        TargetTaskBuilder(final Plant plant, final double initialTarget) {
            this.plant = plant;
            this.initialTarget = initialTarget;
        }

        @Override
        public TargetTaskPost waitForSetpoint() {
            if (!plant.hasFeedback()) {
                throw new IllegalStateException(
                        "TargetTaskStart.waitForSetpoint(...) requires a feedback-capable plant "
                                + "(plant.hasFeedback() == true).");
            }
            this.completionMode = CompletionMode.WAIT_SETPOINT;
            this.waitSeconds = 0.0;
            this.timeoutSec = 0.0;
            return this;
        }

        @Override
        public TargetTaskPost waitForSetpointOrTimeout(final double timeoutSec) {
            if (timeoutSec <= 0.0) {
                throw new IllegalArgumentException(
                        "timeoutSec must be > 0, got " + timeoutSec);
            }
            if (!plant.hasFeedback()) {
                throw new IllegalStateException(
                        "TargetTaskStart.waitForSetpointOrTimeout(...) requires a feedback-capable plant "
                                + "(plant.hasFeedback() == true).");
            }
            this.completionMode = CompletionMode.WAIT_SETPOINT_OR_TIMEOUT;
            this.waitSeconds = 0.0;
            this.timeoutSec = timeoutSec;
            return this;
        }

        @Override
        public TargetTaskPost waitFor(final double seconds) {
            if (seconds < 0.0) {
                throw new IllegalArgumentException(
                        "seconds must be >= 0, got " + seconds);
            }
            this.completionMode = CompletionMode.WAIT_TIME;
            this.waitSeconds = seconds;
            this.timeoutSec = 0.0;
            return this;
        }

        @Override
        public TargetTaskBuild instant() {
            this.completionMode = CompletionMode.INSTANT;
            this.waitSeconds = 0.0;
            this.timeoutSec = 0.0;
            return this;
        }

        @Override
        public TargetTaskBuild thenHold() {
            this.postBehavior = PostBehavior.HOLD;
            return this;
        }

        @Override
        public TargetTaskBuild then(final double finalTarget) {
            this.postBehavior = PostBehavior.FINAL_TARGET;
            this.finalTarget = finalTarget;
            return this;
        }

        @Override
        public Task build() {
            return new TargetTask(
                    plant,
                    initialTarget,
                    completionMode,
                    waitSeconds,
                    timeoutSec,
                    postBehavior,
                    finalTarget
            );
        }
    }

    /**
     * Unified task that supports both time-based and setpoint-based completion
     * modes and reports its outcome via {@link Task#getOutcome()}.
     */
    private static final class TargetTask implements Task {

        private final Plant plant;
        private final double initialTarget;
        private final CompletionMode completionMode;

        private final double waitSeconds;
        private final double timeoutSec;

        private final PostBehavior postBehavior;
        private final double finalTarget;

        private boolean started = false;
        private boolean finished = false;
        private TaskOutcome outcome = TaskOutcome.UNKNOWN;

        private double elapsedSec = 0.0;
        private double remainingSec = 0.0;

        TargetTask(final Plant plant,
                   final double initialTarget,
                   final CompletionMode completionMode,
                   final double waitSeconds,
                   final double timeoutSec,
                   final PostBehavior postBehavior,
                   final double finalTarget) {
            this.plant = plant;
            this.initialTarget = initialTarget;
            this.completionMode = completionMode;
            this.waitSeconds = waitSeconds;
            this.timeoutSec = timeoutSec;
            this.postBehavior = postBehavior;
            this.finalTarget = finalTarget;
        }

        @Override
        public void start(final LoopClock clock) {
            if (started) {
                return;
            }
            started = true;
            finished = false;
            elapsedSec = 0.0;
            remainingSec = waitSeconds;

            outcome = TaskOutcome.UNKNOWN;

            plant.setTarget(initialTarget);

            // Edge case: INSTANT completion finishes immediately.
            if (completionMode == CompletionMode.INSTANT) {
                finished = true;
                outcome = TaskOutcome.SUCCESS;
                applyPostBehavior();
            } else if (completionMode == CompletionMode.WAIT_TIME && waitSeconds == 0.0) {
                // Zero-duration time wait: also finish immediately.
                finished = true;
                outcome = TaskOutcome.SUCCESS;
                applyPostBehavior();
            }
        }

        @Override
        public void update(final LoopClock clock) {
            if (!started || finished) {
                return;
            }

            double dt = clock.dtSec();
            if (dt < 0.0) {
                dt = 0.0;
            }
            elapsedSec += dt;

            switch (completionMode) {
                case INSTANT:
                    finished = true;
                    outcome = TaskOutcome.SUCCESS;
                    break;

                case WAIT_TIME:
                    remainingSec -= dt;
                    if (remainingSec <= 0.0) {
                        finished = true;
                        outcome = TaskOutcome.SUCCESS;
                    }
                    break;

                case WAIT_SETPOINT:
                    if (plant.atSetpoint()) {
                        finished = true;
                        outcome = TaskOutcome.SUCCESS;
                    }
                    break;

                case WAIT_SETPOINT_OR_TIMEOUT:
                    if (plant.atSetpoint()) {
                        finished = true;
                        outcome = TaskOutcome.SUCCESS;
                    } else if (elapsedSec >= timeoutSec) {
                        finished = true;
                        outcome = TaskOutcome.TIMEOUT;
                    }
                    break;
            }

            if (finished) {
                applyPostBehavior();
            }
        }

        @Override
        public boolean isComplete() {
            return finished;
        }

        @Override
        public TaskOutcome getOutcome() {
            return outcome;
        }

        private boolean postApplied = false;

        private void applyPostBehavior() {
            if (postApplied) {
                return;
            }
            postApplied = true;

            if (postBehavior == PostBehavior.FINAL_TARGET) {
                plant.setTarget(finalTarget);
            }
            // HOLD: do nothing; keep last target.
        }

        @Override
        public String getDebugName() {
            return "PlantTask(target=" + initialTarget + ")";
        }
    }
}
