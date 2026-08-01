package edu.ftcphoenix.fw.actuation;

import java.util.Objects;

/**
 * Per-loop facts a {@link PlantTargetResolver} may use to choose a requested plant target.
 *
 * <p>The plant creates this context during {@link Plant#update(edu.ftcphoenix.fw.core.time.LoopClock)}
 * after refreshing any feedback that target resolution may need. Context-aware resolvers use these
 * facts to select equivalent positions, hold the current measured position, or choose alternatives
 * inside the plant's legal range. A simple exact resolver can ignore the context.</p>
 *
 * <p>All scalar values are in the plant's public units: the same units used by
 * {@link Plant#getRequestedTarget()}, {@link Plant#getAppliedTarget()}, and
 * {@link PositionPlant#targetRange()}.</p>
 */
public final class PlantTargetContext {

    private final boolean feedbackAvailable;
    private final double measurement;
    private final ScalarRange targetRange;
    private final PositionPlant.Periodicity periodicity;
    private final double period;
    private final double previousRequestedTarget;
    private final double previousAppliedTarget;

    private PlantTargetContext(boolean feedbackAvailable,
                               double measurement,
                               ScalarRange targetRange,
                               PositionPlant.Periodicity periodicity,
                               double period,
                               double previousRequestedTarget,
                               double previousAppliedTarget) {
        this.feedbackAvailable = feedbackAvailable;
        this.measurement = measurement;
        this.targetRange = Objects.requireNonNull(targetRange, "targetRange");
        this.periodicity = Objects.requireNonNull(periodicity, "periodicity");
        this.period = period;
        this.previousRequestedTarget = previousRequestedTarget;
        this.previousAppliedTarget = previousAppliedTarget;
    }

    /**
     * Build a context for non-position plants or simple scalar outputs.
     */
    public static PlantTargetContext simple(boolean feedbackAvailable,
                                            double measurement,
                                            ScalarRange targetRange,
                                            double previousRequestedTarget,
                                            double previousAppliedTarget) {
        return new PlantTargetContext(feedbackAvailable,
                measurement,
                targetRange != null ? targetRange : ScalarRange.unbounded(),
                PositionPlant.Periodicity.NON_PERIODIC,
                Double.NaN,
                previousRequestedTarget,
                previousAppliedTarget);
    }

    /**
     * Build a context for a position plant.
     */
    public static PlantTargetContext position(boolean feedbackAvailable,
                                              double measurement,
                                              ScalarRange targetRange,
                                              PositionPlant.Periodicity periodicity,
                                              double period,
                                              double previousRequestedTarget,
                                              double previousAppliedTarget) {
        return new PlantTargetContext(feedbackAvailable,
                measurement,
                targetRange != null ? targetRange : ScalarRange.unbounded(),
                periodicity != null ? periodicity : PositionPlant.Periodicity.NON_PERIODIC,
                period,
                previousRequestedTarget,
                previousAppliedTarget);
    }

    /**
     * True when the measurement is meaningful for planning or feedback decisions.
     */
    public boolean feedbackAvailable() {
        return feedbackAvailable && Double.isFinite(measurement);
    }

    /**
     * Latest plant measurement in public plant units, or {@link Double#NaN} when unavailable.
     */
    public double measurement() {
        return measurement;
    }

    /**
     * Current legal requested-target range in plant units.
     */
    public ScalarRange targetRange() {
        return targetRange;
    }

    /**
     * Position periodicity. Non-position plants report
     * {@link PositionPlant.Periodicity#NON_PERIODIC}.
     */
    public PositionPlant.Periodicity periodicity() {
        return periodicity;
    }

    /**
     * Period for periodic position plants, or {@link Double#NaN} for non-periodic/non-position plants.
     */
    public double period() {
        return period;
    }

    /**
     * Requested target from the previous update, or {@link Double#NaN} before the first update.
     */
    public double previousRequestedTarget() {
        return previousRequestedTarget;
    }

    /**
     * Applied target from the previous update.
     */
    public double previousAppliedTarget() {
        return previousAppliedTarget;
    }

    @Override
    public String toString() {
        return "PlantTargetContext{"
                + "feedbackAvailable=" + feedbackAvailable()
                + ", measurement=" + measurement
                + ", targetRange=" + targetRange
                + ", periodicity=" + periodicity
                + ", period=" + period
                + ", previousRequestedTarget=" + previousRequestedTarget
                + ", previousAppliedTarget=" + previousAppliedTarget
                + '}';
    }
}
