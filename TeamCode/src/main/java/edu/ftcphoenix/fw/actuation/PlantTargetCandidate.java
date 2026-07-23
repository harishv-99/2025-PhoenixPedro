package edu.ftcphoenix.fw.actuation;

/**
 * One acceptable scalar target family in a plant's public units.
 *
 * <p>A candidate may be exact or periodic-equivalent, and absolute or relative to the current
 * measurement. A tray with several valid slots can issue a request with several candidates; a turret
 * facing a point usually issues one periodic-equivalent candidate.</p>
 *
 * <p>Ordinary factories describe timeless robot intent. {@code observed...} factories describe a
 * value derived from an observation and carry that observation's stable timestamp in the consuming
 * {@link edu.ftcphoenix.fw.core.time.LoopClock} timebase. Observation age is derived by the planner
 * when the request is resolved; it is not stored as a second, potentially contradictory freshness fact.
 * Quality and timestamp metadata are checked during resolution so a malformed live observation can
 * follow the plan's unavailable policy instead of throwing while the source constructs it.</p>
 */
public final class PlantTargetCandidate {

    public final String id;
    public final double value;
    public final boolean periodic;
    public final double period;
    public final boolean usesPlantPeriod;
    public final boolean relative;
    public final double quality;
    public final double timestampSec;
    private final boolean observed;

    private PlantTargetCandidate(String id,
                                 double value,
                                 boolean periodic,
                                 double period,
                                 boolean usesPlantPeriod,
                                 boolean relative,
                                 double quality,
                                 double timestampSec,
                                 boolean observed) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException("Plant target candidate value must be finite");
        }
        if (periodic && !usesPlantPeriod && (!(period > 0.0) || !Double.isFinite(period))) {
            throw new IllegalArgumentException("Periodic plant target candidates require finite period > 0");
        }
        this.id = (id == null || id.trim().isEmpty()) ? "candidate" : id.trim();
        this.value = value;
        this.periodic = periodic;
        this.period = period;
        this.usesPlantPeriod = usesPlantPeriod;
        this.relative = relative;
        this.quality = quality;
        this.timestampSec = timestampSec;
        this.observed = observed;
    }

    /**
     * Creates an absolute exact candidate.
     */
    public static PlantTargetCandidate exact(String id, double value) {
        return new PlantTargetCandidate(id, value, false, Double.NaN, false, false,
                1.0, Double.NaN, false);
    }

    /**
     * Creates an absolute exact candidate derived from an observation.
     *
     * @param quality observation quality for planner acceptance
     * @param timestampSec stable observation timestamp in the consuming loop clock's timebase
     */
    public static PlantTargetCandidate observedExact(String id,
                                                     double value,
                                                     double quality,
                                                     double timestampSec) {
        return new PlantTargetCandidate(id, value, false, Double.NaN, false, false,
                quality, timestampSec, true);
    }

    /**
     * Creates an absolute periodic-equivalent candidate that uses the downstream position plant's
     * declared period.
     */
    public static PlantTargetCandidate equivalentPosition(String id, double value) {
        return new PlantTargetCandidate(id, value, true, Double.NaN, true, false,
                1.0, Double.NaN, false);
    }

    /**
     * Creates a plant-period equivalent-position candidate derived from an observation.
     *
     * @param quality observation quality for planner acceptance
     * @param timestampSec stable observation timestamp in the consuming loop clock's timebase
     */
    public static PlantTargetCandidate observedEquivalentPosition(String id,
                                                                  double value,
                                                                  double quality,
                                                                  double timestampSec) {
        return new PlantTargetCandidate(id, value, true, Double.NaN, true, false,
                quality, timestampSec, true);
    }

    /**
     * Creates an absolute periodic candidate with an explicit period.
     */
    public static PlantTargetCandidate periodic(String id, double value, double period) {
        return new PlantTargetCandidate(id, value, true, period, false, false,
                1.0, Double.NaN, false);
    }

    /**
     * Creates an absolute periodic candidate derived from an observation.
     *
     * @param quality observation quality for planner acceptance
     * @param timestampSec stable observation timestamp in the consuming loop clock's timebase
     */
    public static PlantTargetCandidate observedPeriodic(String id,
                                                        double value,
                                                        double period,
                                                        double quality,
                                                        double timestampSec) {
        return new PlantTargetCandidate(id, value, true, period, false, false,
                quality, timestampSec, true);
    }

    /**
     * Creates a relative exact candidate: target = current measurement + delta.
     */
    public static PlantTargetCandidate relative(String id, double delta) {
        return new PlantTargetCandidate(id, delta, false, Double.NaN, false, true,
                1.0, Double.NaN, false);
    }

    /**
     * Creates a relative exact candidate derived from an observation.
     *
     * @param quality observation quality for planner acceptance
     * @param timestampSec stable observation timestamp in the consuming loop clock's timebase
     */
    public static PlantTargetCandidate observedRelative(String id,
                                                        double delta,
                                                        double quality,
                                                        double timestampSec) {
        return new PlantTargetCandidate(id, delta, false, Double.NaN, false, true,
                quality, timestampSec, true);
    }

    /**
     * Creates a relative equivalent-position candidate using the downstream plant's period.
     */
    public static PlantTargetCandidate relativeEquivalentPosition(String id, double delta) {
        return new PlantTargetCandidate(id, delta, true, Double.NaN, true, true,
                1.0, Double.NaN, false);
    }

    /**
     * Creates a relative equivalent-position candidate derived from an observation and using the
     * downstream plant's period.
     *
     * @param quality observation quality for planner acceptance
     * @param timestampSec stable observation timestamp in the consuming loop clock's timebase
     */
    public static PlantTargetCandidate observedRelativeEquivalentPosition(String id,
                                                                          double delta,
                                                                          double quality,
                                                                          double timestampSec) {
        return new PlantTargetCandidate(id, delta, true, Double.NaN, true, true,
                quality, timestampSec, true);
    }

    /**
     * Creates a relative periodic candidate: target = current measurement + delta + k*period.
     */
    public static PlantTargetCandidate relativePeriodic(String id, double delta, double period) {
        return new PlantTargetCandidate(id, delta, true, period, false, true,
                1.0, Double.NaN, false);
    }

    /**
     * Creates a relative periodic candidate derived from an observation:
     * target = current measurement + delta + k*period.
     *
     * @param quality observation quality for planner acceptance
     * @param timestampSec stable observation timestamp in the consuming loop clock's timebase
     */
    public static PlantTargetCandidate observedRelativePeriodic(String id,
                                                                double delta,
                                                                double period,
                                                                double quality,
                                                                double timestampSec) {
        return new PlantTargetCandidate(id, delta, true, period, false, true,
                quality, timestampSec, true);
    }

    boolean isObserved() {
        return observed;
    }

    @Override
    public String toString() {
        return "PlantTargetCandidate{id='" + id + "', value=" + value
                + ", periodic=" + periodic + ", period=" + period
                + ", usesPlantPeriod=" + usesPlantPeriod
                + ", relative=" + relative + ", quality=" + quality
                + ", observed=" + observed + ", timestampSec=" + timestampSec + '}';
    }
}
