package edu.ftcphoenix.fw.actuation;

/**
 * One acceptable scalar target family in plant units.
 *
 * <p>A candidate may be exact or periodic, and absolute or relative to the current measurement. A
 * tray with several possible slots can issue a request with several candidates; a turret facing a
 * point usually issues one periodic-equivalent candidate.</p>
 *
 * <p>When a planner is built with {@link ScalarSetpointPlanner.Builder#forPositionPlant(PositionPlant)},
 * candidates created by {@link #equivalentPosition(String, double)} use the plant's declared period.
 * This keeps periodicity on the position plant/domain instead of repeating the period in every
 * request.</p>
 */
public final class ScalarSetpointCandidate {

    public final String id;
    public final double value;
    public final boolean periodic;
    public final double period;
    public final boolean usesPlantPeriod;
    public final boolean relative;
    public final double quality;
    public final double ageSec;
    public final double timestampSec;

    private ScalarSetpointCandidate(String id,
                                    double value,
                                    boolean periodic,
                                    double period,
                                    boolean usesPlantPeriod,
                                    boolean relative,
                                    double quality,
                                    double ageSec,
                                    double timestampSec) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException("Scalar setpoint candidate value must be finite");
        }
        if (periodic && !usesPlantPeriod && (!(period > 0.0) || !Double.isFinite(period))) {
            throw new IllegalArgumentException("Periodic scalar setpoint candidates require finite period > 0");
        }
        this.id = (id == null || id.isEmpty()) ? "candidate" : id;
        this.value = value;
        this.periodic = periodic;
        this.period = period;
        this.usesPlantPeriod = usesPlantPeriod;
        this.relative = relative;
        this.quality = quality;
        this.ageSec = ageSec;
        this.timestampSec = timestampSec;
    }

    /**
     * Creates an absolute exact candidate.
     */
    public static ScalarSetpointCandidate exact(String id, double value) {
        return exact(id, value, 1.0, 0.0, Double.NaN);
    }

    /** Creates an absolute exact candidate with quality/timing metadata. */
    public static ScalarSetpointCandidate exact(String id, double value, double quality, double ageSec, double timestampSec) {
        return new ScalarSetpointCandidate(id, value, false, Double.NaN, false, false, quality, ageSec, timestampSec);
    }

    /**
     * Creates an absolute periodic-equivalent candidate that uses the downstream position plant's
     * declared period.
     *
     * <p>Use this when the request is interpreted by a planner built with
     * {@link ScalarSetpointPlanner.Builder#forPositionPlant(PositionPlant)}.</p>
     */
    public static ScalarSetpointCandidate equivalentPosition(String id, double value) {
        return equivalentPosition(id, value, 1.0, 0.0, Double.NaN);
    }

    /**
     * Creates a plant-period equivalent-position candidate with quality/timing metadata.
     */
    public static ScalarSetpointCandidate equivalentPosition(String id,
                                                             double value,
                                                             double quality,
                                                             double ageSec,
                                                             double timestampSec) {
        return new ScalarSetpointCandidate(id, value, true, Double.NaN, true, false, quality, ageSec, timestampSec);
    }

    /**
     * Creates an absolute periodic candidate with an explicit period.
     *
     * <p>This is still useful for standalone scalar planning that is not bound to a
     * {@link PositionPlant}. Prefer {@link #equivalentPosition(String, double)} when the plant owns
     * the periodicity.</p>
     */
    public static ScalarSetpointCandidate periodic(String id, double value, double period) {
        return periodic(id, value, period, 1.0, 0.0, Double.NaN);
    }

    /** Creates an absolute periodic candidate with quality/timing metadata. */
    public static ScalarSetpointCandidate periodic(String id, double value, double period, double quality, double ageSec, double timestampSec) {
        return new ScalarSetpointCandidate(id, value, true, period, false, false, quality, ageSec, timestampSec);
    }

    /** Creates a relative exact candidate: target = current measurement + delta. */
    public static ScalarSetpointCandidate relative(String id, double delta) {
        return new ScalarSetpointCandidate(id, delta, false, Double.NaN, false, true, 1.0, 0.0, Double.NaN);
    }

    /**
     * Creates a relative equivalent-position candidate using the downstream plant's period.
     */
    public static ScalarSetpointCandidate relativeEquivalentPosition(String id,
                                                                     double delta,
                                                                     double quality,
                                                                     double ageSec,
                                                                     double timestampSec) {
        return new ScalarSetpointCandidate(id, delta, true, Double.NaN, true, true, quality, ageSec, timestampSec);
    }

    /** Creates a relative periodic candidate: target = current measurement + delta + k*period. */
    public static ScalarSetpointCandidate relativePeriodic(String id, double delta, double period, double quality, double ageSec, double timestampSec) {
        return new ScalarSetpointCandidate(id, delta, true, period, false, true, quality, ageSec, timestampSec);
    }

    @Override
    public String toString() {
        return "ScalarSetpointCandidate{id='" + id + "', value=" + value
                + ", periodic=" + periodic + ", period=" + period
                + ", usesPlantPeriod=" + usesPlantPeriod
                + ", relative=" + relative + ", quality=" + quality
                + ", ageSec=" + ageSec + ", timestampSec=" + timestampSec + '}';
    }
}
