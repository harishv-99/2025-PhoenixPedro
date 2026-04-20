package edu.ftcphoenix.fw.actuation;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;

/**
 * Task helpers for establishing {@link PositionPlant} references by searching for a physical cue.
 *
 * <p>These tasks cover homing and indexing patterns without teaching the framework about a specific
 * robot mechanism. The cue can be any {@link BooleanSource}: a limit switch, color mark detector,
 * beam break, stall detector, vision predicate, or manual operator button.</p>
 *
 * <h2>Typical lift homing</h2>
 *
 * <pre>{@code
 * Task homeLift = PositionCalibrationTasks.search(lift)
 *     .withPower(-0.20)
 *     .until(bottomSwitch)
 *     .establishReferenceAt(0.0)
 *     .thenHold(0.0)
 *     .failAfter(3.0)
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
 *     .thenStop()
 *     .failAfter(5.0)
 *     .build();
 * }</pre>
 *
 * <p>For periodic plants, {@code establishReferenceAt(x)} establishes reference {@code x} modulo
 * the plant period and preserves the nearest equivalent unwrapped position when the plant is already
 * referenced.</p>
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
         * Applies this normalized power while the task waits for the reference condition.
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
         */
        SearchAfterStep establishReferenceAt(double plantPosition);
    }

    /**
     * Fourth search-task question: what should happen after the reference is established?
     */
    public interface SearchAfterStep {
        /**
         * Stop the search output after a successful reference.
         */
        SearchBuildStep thenStop();

        /**
         * Hold a plant-unit target after a successful reference.
         */
        SearchBuildStep thenHold(double plantTarget);
    }

    /**
     * Optional timeout/build step.
     */
    public interface SearchBuildStep {
        /**
         * Fail with {@link TaskOutcome#TIMEOUT} if the condition is not found within this duration.
         */
        SearchBuildStep failAfter(double timeoutSec);

        /**
         * Build the non-blocking calibration task.
         */
        Task build();
    }

    private static final class Builder implements SearchPowerStep, SearchUntilStep, SearchReferenceStep,
            SearchAfterStep, SearchBuildStep {
        private final PositionPlant plant;
        private double power;
        private BooleanSource condition;
        private double reference;
        private boolean holdAfter;
        private double holdTarget;
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
            this.power = power;
            return this;
        }

        @Override
        public SearchReferenceStep until(BooleanSource condition) {
            this.condition = Objects.requireNonNull(condition, "condition");
            return this;
        }

        @Override
        public SearchAfterStep establishReferenceAt(double plantPosition) {
            this.reference = plantPosition;
            return this;
        }

        @Override
        public SearchBuildStep thenStop() {
            this.holdAfter = false;
            return this;
        }

        @Override
        public SearchBuildStep thenHold(double plantTarget) {
            this.holdAfter = true;
            this.holdTarget = plantTarget;
            return this;
        }

        @Override
        public SearchBuildStep failAfter(double timeoutSec) {
            if (!(timeoutSec > 0.0) || !Double.isFinite(timeoutSec))
                throw new IllegalArgumentException("timeoutSec must be finite and > 0");
            this.timeoutSec = timeoutSec;
            return this;
        }

        @Override
        public Task build() {
            return new SearchTask(plant, power, condition, reference, holdAfter, holdTarget, timeoutSec);
        }
    }

    private static final class SearchTask implements Task {
        private final PositionPlant plant;
        private final double power;
        private final BooleanSource condition;
        private final double reference;
        private final boolean holdAfter;
        private final double holdTarget;
        private final double timeoutSec;
        private boolean started;
        private boolean complete;
        private double startSec;
        private TaskOutcome outcome = TaskOutcome.NOT_DONE;

        private SearchTask(PositionPlant plant,
                           double power,
                           BooleanSource condition,
                           double reference,
                           boolean holdAfter,
                           double holdTarget,
                           double timeoutSec) {
            this.plant = plant;
            this.power = power;
            this.condition = condition;
            this.reference = reference;
            this.holdAfter = holdAfter;
            this.holdTarget = holdTarget;
            this.timeoutSec = timeoutSec;
        }

        @Override
        public void start(LoopClock clock) {
            started = true;
            complete = false;
            startSec = clock != null ? clock.nowSec() : 0.0;
            outcome = TaskOutcome.NOT_DONE;
            condition.reset();
            plant.beginCalibrationSearch(power);
        }

        @Override
        public void update(LoopClock clock) {
            if (!started || complete) return;
            plant.update(clock);
            if (condition.getAsBoolean(clock)) {
                plant.establishReferenceAt(reference, clock);
                plant.endCalibrationSearch(true);
                if (holdAfter) plant.setTarget(holdTarget);
                outcome = TaskOutcome.SUCCESS;
                complete = true;
                return;
            }
            if (Double.isFinite(timeoutSec) && clock != null && clock.nowSec() - startSec >= timeoutSec) {
                plant.endCalibrationSearch(true);
                outcome = TaskOutcome.TIMEOUT;
                complete = true;
            }
        }

        @Override
        public void cancel() {
            if (!complete) {
                plant.endCalibrationSearch(true);
                outcome = TaskOutcome.CANCELLED;
                complete = true;
            }
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
                    .addData(p + ".holdAfter", holdAfter)
                    .addData(p + ".holdTarget", holdTarget)
                    .addData(p + ".timeoutSec", timeoutSec)
                    .addData(p + ".started", started)
                    .addData(p + ".complete", complete)
                    .addData(p + ".outcome", getOutcome())
                    .addData(p + ".plantReferenced", plant.isReferenced())
                    .addData(p + ".plantReferenceStatus", plant.referenceStatus());
        }
    }
}
