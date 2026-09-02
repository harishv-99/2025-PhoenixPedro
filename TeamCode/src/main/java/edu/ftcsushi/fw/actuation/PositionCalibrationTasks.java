package edu.ftcsushi.fw.actuation;

import java.util.Objects;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;

/**
 * Task helpers for establishing {@link PositionPlant} references by searching for a physical cue.
 *
 * <p>These tasks cover homing and indexing patterns without teaching the framework about a specific
 * robot mechanism. The cue can be any {@link BooleanSource}: a limit switch, color mark detector,
 * beam break, stall detector, vision predicate, or manual operator button.</p>
 *
 * <p>Search power is a required finite normalized answer in the inclusive
 * {@code [-1.0, +1.0]} range. That structural check does not choose a mechanically safe magnitude
 * or direction for a particular mechanism; the robot owner must validate those physical facts.</p>
 *
 * <p>The reference is a finite plant-unit coordinate anchor and need not lie inside the Plant's
 * legal target range. That structural check does not prove that the value matches a safe physical
 * pose.</p>
 *
 * <h2>Typical bottom-reference search</h2>
 *
 * <pre>{@code
 * Task findBottomReference = PositionCalibrationTasks.search(lift)
 *     .withPower(-0.20)
 *     .until(bottomSwitch)
 *     .establishReferenceAt(0.0)
 *     .failAfterSec(3.0)
 *     .build();
 * }</pre>
 *
 * <h2>Typical tray indexing</h2>
 *
 * <pre>{@code
 * Task indexTray = PositionCalibrationTasks.search(tray)
 *     .withPower(0.12)
 *     .until(paintedMarkSeen)
 *     .establishReferenceAt(0.0)
 *     .failAfterSec(5.0)
 *     .build();
 * }</pre>
 *
 * <p>The task owns the temporary search lifecycle, cue, reference, timeout, output-stop request,
 * and release. It never reads or writes the Plant's persistent command or final target resolver,
 * and it never calls {@link Plant#update(LoopClock)}. Success, timeout, and active cancellation all
 * preserve the persistent request. The mechanism or subsystem remains the sole Plant heartbeat
 * owner and must update the Plant once in the normal downstream Plant phase after its
 * {@code TaskRunner} advances this task. Any success-only semantic request, such as selecting a
 * named stowed height, belongs in a following Task that calls the mechanism's normal setter.</p>
 *
 * <p>For periodic plants, {@code establishReferenceAt(x)} establishes reference {@code x} modulo
 * the plant period and preserves the nearest equivalent unwrapped position when the plant is already
 * referenced, using the current loop's native sample.</p>
 */
public final class PositionCalibrationTasks {
    private PositionCalibrationTasks() {
    }

    /**
     * Starts building a calibration-search task for a searchable position plant.
     */
    public static SearchPowerStep search(PositionPlant plant) {
        return new Builder(plant);
    }

    /**
     * First search-task question: what open-loop power should be used while searching?
     */
    public interface SearchPowerStep {
        /**
         * Stages this normalized power for the Plant owner's downstream updates while the task
         * waits for the reference condition.
         *
         * <p>The value must be finite and inside the inclusive {@code [-1.0, +1.0]} range. Invalid
         * values are rejected immediately rather than clamped, before this builder stores the
         * answer or any task, Plant, or output lifecycle begins.</p>
         *
         * @param power finite normalized open-loop search power
         * @throws IllegalArgumentException if {@code power} is non-finite or outside
         *                                  {@code [-1.0, +1.0]}
         */
        SearchUntilStep withPower(double power);
    }

    /**
     * Second search-task question: what condition means the reference has been found?
     */
    public interface SearchUntilStep {
        /**
         * Finishes the search when {@code condition} becomes true.
         */
        SearchReferenceStep until(BooleanSource condition);
    }

    /**
     * Third search-task question: what plant-unit reference does the found condition represent?
     */
    public interface SearchReferenceStep {
        /**
         * Establishes that the current mechanism position corresponds to {@code plantPosition}.
         *
         * <p>The value must be finite in plant units and is rejected immediately before the
         * builder stores it or any Task, Plant, cue, feedback, or output lifecycle begins. It is a
         * coordinate anchor rather than a target request and therefore need not lie inside the
         * Plant's legal target range.</p>
         *
         * @param plantPosition finite reference coordinate in plant units
         * @throws IllegalArgumentException if {@code plantPosition} is non-finite
         */
        SearchTimeoutStep establishReferenceAt(double plantPosition);
    }

    /**
     * Final search-task question: when should an unfinished search fail?
     *
     * <p>Calibration searches often drive gently into a limit, mark, or other reference cue. The
     * timeout behavior is required so callers deliberately choose between a bounded search and an
     * intentionally unbounded one.</p>
     */
    public interface SearchTimeoutStep {
        /**
         * Fail with {@link TaskOutcome#TIMEOUT} if the condition is not found within this many seconds.
         */
        SearchBuildStep failAfterSec(double timeoutSec);

        /**
         * Allow the search to run until the reference is found or the task is cancelled.
         *
         * <p>Use this only when another scheduler, driver action, or safety interlock is guaranteed
         * to stop the task if the reference cue cannot be found.</p>
         */
        SearchBuildStep neverTimeout();
    }

    /**
     * Build step available only after the timeout policy has been answered explicitly.
     */
    public interface SearchBuildStep {
        /**
         * Build a new single-use non-blocking calibration task.
         *
         * <p>Build a fresh search task if calibration must be scheduled again. Advance it in the
         * Task phase, then let the owning mechanism update the Plant exactly once downstream.</p>
         */
        Task build();
    }

    private static final class Builder implements SearchPowerStep, SearchUntilStep, SearchReferenceStep,
            SearchTimeoutStep, SearchBuildStep {
        private final PositionPlant plant;
        private double power;
        private BooleanSource condition;
        private double reference;
        private double timeoutSec = Double.POSITIVE_INFINITY;

        private Builder(PositionPlant plant) {
            this.plant = Objects.requireNonNull(plant, "plant");
            if (!plant.supportsCalibrationSearch()) {
                throw new IllegalStateException("PositionCalibrationTasks.search(...) requires a PositionPlant "
                        + "that supports calibration search drive. Standard-servo position plants usually use "
                        + "rangeMapsToNative(...) or an already established reference instead.");
            }
        }

        @Override
        public SearchUntilStep withPower(double power) {
            this.power = CalibrationSearchPowerValidation.requireValid(
                    power, "PositionCalibrationTasks.withPower(...)");
            return this;
        }

        @Override
        public SearchReferenceStep until(BooleanSource condition) {
            this.condition = Objects.requireNonNull(condition, "condition");
            return this;
        }

        @Override
        public SearchTimeoutStep establishReferenceAt(double plantPosition) {
            this.reference = PositionCalibrationValueValidation.requireFinitePlantValue(
                    plantPosition,
                    "PositionCalibrationTasks.establishReferenceAt(...)",
                    "plantPosition");
            return this;
        }

        @Override
        public SearchBuildStep failAfterSec(double timeoutSec) {
            if (!(timeoutSec > 0.0) || !Double.isFinite(timeoutSec))
                throw new IllegalArgumentException("timeoutSec must be finite and > 0");
            this.timeoutSec = timeoutSec;
            return this;
        }

        @Override
        public SearchBuildStep neverTimeout() {
            this.timeoutSec = Double.POSITIVE_INFINITY;
            return this;
        }

        @Override
        public Task build() {
            return new SearchTask(plant, power, condition, reference, timeoutSec);
        }
    }

    private static final class SearchTask implements Task {
        private final PositionPlant plant;
        private final double power;
        private final BooleanSource condition;
        private final double reference;
        private final double timeoutSec;
        private boolean startAttempted;
        private boolean started;
        private boolean searchAcquired;
        private boolean complete;
        private double startSec;
        private TaskOutcome outcome = TaskOutcome.NOT_DONE;

        private SearchTask(PositionPlant plant,
                           double power,
                           BooleanSource condition,
                           double reference,
                           double timeoutSec) {
            this.plant = plant;
            this.power = power;
            this.condition = condition;
            this.reference = reference;
            this.timeoutSec = timeoutSec;
        }

        @Override
        public void start(LoopClock clock) {
            if (startAttempted) {
                throw new IllegalStateException("PositionCalibrationTasks.search(...) is single-use "
                        + "and cannot be started more than once. Create a fresh Task by rebuilding "
                        + "the calibration search; use a Supplier<Task> for repeated scheduling.");
            }
            startAttempted = true;
            started = true;
            searchAcquired = false;
            complete = false;
            startSec = clock != null ? clock.nowSec() : 0.0;
            outcome = TaskOutcome.NOT_DONE;
            condition.reset();
            if (complete) return;
            plant.beginCalibrationSearch(power);
            // A normally returning begin transfers this search to the Task. A throwing begin must
            // leave no newly acquired search, so ownership is recorded only after return.
            searchAcquired = true;
            // Cancellation may have re-entered while the Plant was acquiring the search. In that
            // case the terminal Task must release the newly returned acquisition exactly once.
            if (complete) releaseSearch();
        }

        @Override
        public void update(LoopClock clock) {
            if (!started) {
                throw new IllegalStateException("PositionCalibrationTasks.search(...) cannot be "
                        + "updated before start(clock). Start it first, normally by enqueueing it "
                        + "in a TaskRunner.");
            }
            if (complete) return;
            boolean referenceFound = condition.getAsBoolean(clock);
            if (complete) return;
            if (referenceFound) {
                plant.establishReferenceAt(reference, clock);
                if (complete) return;
                outcome = TaskOutcome.SUCCESS;
                complete = true;
                releaseSearch();
                return;
            }
            if (Double.isFinite(timeoutSec) && clock != null && clock.nowSec() - startSec >= timeoutSec) {
                outcome = TaskOutcome.TIMEOUT;
                complete = true;
                releaseSearch();
            }
        }

        @Override
        public void cancel() {
            if (!started || complete) return;

            // Mark terminal before external cleanup so a throwing Plant cleanup is not repeated.
            outcome = TaskOutcome.CANCELLED;
            complete = true;
            releaseSearch();
        }

        /** Release an acquired search once, clearing Task ownership before external cleanup. */
        private void releaseSearch() {
            if (!searchAcquired) return;
            searchAcquired = false;
            plant.endCalibrationSearch();
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
            return "PositionCalibrationSearch";
        }

        @Override
        public void debugDump(DebugSink dbg, String prefix) {
            if (dbg == null) return;
            String p = (prefix == null || prefix.isEmpty()) ? "positionCalibrationSearch" : prefix;
            dbg.addData(p + ".power", power)
                    .addData(p + ".reference", reference)
                    .addData(p + ".timeoutSec", timeoutSec)
                    .addData(p + ".started", started)
                    .addData(p + ".searchAcquired", searchAcquired)
                    .addData(p + ".complete", complete)
                    .addData(p + ".outcome", getOutcome())
                    .addData(p + ".plantReferenced", plant.isReferenced())
                    .addData(p + ".plantReferenceStatus", plant.referenceStatus());
        }
    }
}
