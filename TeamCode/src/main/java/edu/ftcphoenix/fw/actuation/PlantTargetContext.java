package edu.ftcphoenix.fw.actuation;

import java.util.Objects;

/**
 * Per-loop facts a {@link PlantTargetSource} may use to choose a requested plant target.
 *
 * <p>The plant creates this context during {@link Plant#update(edu.ftcphoenix.fw.core.time.LoopClock)}
 * after refreshing any feedback that target planning may need. Smart target sources use the context
 * to resolve equivalent positions, hold the current measured position, or choose candidates inside
 * the plant's legal range. Plain scalar targets can ignore the context.</p>
 *
 * <p>All scalar values are in the plant's public units: the same units used by
 * {@link Plant#getRequestedTarget()}, {@link Plant#getAppliedTarget()}, and
 * {@link PositionPlant#targetRange()}.</p>
 */
public final class PlantTargetContext {

    private final boolean feedbackAvailable;
    private final double measurement;
    private final ScalarRange targetRange;
    private final PositionPlant.Topology topology;
    private final double period;
    private final double previousRequestedTarget;
    private final double previousAppliedTarget;

    private PlantTargetContext(boolean feedbackAvailable,
                               double measurement,
                               ScalarRange targetRange,
                               PositionPlant.Topology topology,
                               double period,
                               double previousRequestedTarget,
                               double previousAppliedTarget) {
        this.feedbackAvailable = feedbackAvailable;
        this.measurement = measurement;
        this.targetRange = Objects.requireNonNull(targetRange, "targetRange");
        this.topology = Objects.requireNonNull(topology, "topology");
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
                PositionPlant.Topology.LINEAR,
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
                                              PositionPlant.Topology topology,
                                              double period,
                                              double previousRequestedTarget,
                                              double previousAppliedTarget) {
        return new PlantTargetContext(feedbackAvailable,
                measurement,
                targetRange != null ? targetRange : ScalarRange.unbounded(),
                topology != null ? topology : PositionPlant.Topology.LINEAR,
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
     * Position topology. Non-position plants report {@link PositionPlant.Topology#LINEAR}.
     */
    public PositionPlant.Topology topology() {
        return topology;
    }

    /**
     * Period for periodic position plants, or {@link Double#NaN} for linear/non-position plants.
     */
    public double period() {
        return period;
    }

    /**
     * True when this context represents a periodic position coordinate.
     */
    public boolean periodic() {
        return topology == PositionPlant.Topology.PERIODIC && period > 0.0 && Double.isFinite(period);
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
                + ", topology=" + topology
                + ", period=" + period
                + ", previousRequestedTarget=" + previousRequestedTarget
                + ", previousAppliedTarget=" + previousAppliedTarget
                + '}';
    }
}
