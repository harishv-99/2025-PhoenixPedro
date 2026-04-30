package edu.ftcphoenix.fw.actuation;

/**
 * One acceptable scalar target family in a plant's public units.
 *
 * <p>A candidate may be exact or periodic-equivalent, and absolute or relative to the current
 * measurement. A tray with several valid slots can issue a request with several candidates; a turret
 * facing a point usually issues one periodic-equivalent candidate.</p>
 */
public final class PlantTargetCandidate {

    public final String id;
    public final double value;
    public final boolean periodic;
    public final double period;
    public final boolean usesPlantPeriod;
    public final boolean relative;
    public final double quality;
    public final double ageSec;
    public final double timestampSec;

    private PlantTargetCandidate(String id,
                                 double value,
                                 boolean periodic,
                                 double period,
                                 boolean usesPlantPeriod,
                                 boolean relative,
                                 double quality,
                                 double ageSec,
                                 double timestampSec) {
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
        this.ageSec = ageSec;
        this.timestampSec = timestampSec;
    }

    /**
     * Creates an absolute exact candidate.
     */
    public static PlantTargetCandidate exact(String id, double value) {
        return exact(id, value, 1.0, 0.0, Double.NaN);
    }

    /**
     * Creates an absolute exact candidate with quality/timing metadata.
     */
    public static PlantTargetCandidate exact(String id, double value, double quality, double ageSec, double timestampSec) {
        return new PlantTargetCandidate(id, value, false, Double.NaN, false, false, quality, ageSec, timestampSec);
    }

    /**
     * Creates an absolute periodic-equivalent candidate that uses the downstream position plant's
     * declared period.
     */
    public static PlantTargetCandidate equivalentPosition(String id, double value) {
        return equivalentPosition(id, value, 1.0, 0.0, Double.NaN);
    }

    /**
     * Creates a plant-period equivalent-position candidate with quality/timing metadata.
     */
    public static PlantTargetCandidate equivalentPosition(String id,
                                                          double value,
                                                          double quality,
                                                          double ageSec,
                                                          double timestampSec) {
        return new PlantTargetCandidate(id, value, true, Double.NaN, true, false, quality, ageSec, timestampSec);
    }

    /**
     * Creates an absolute periodic candidate with an explicit period.
     */
    public static PlantTargetCandidate periodic(String id, double value, double period) {
        return periodic(id, value, period, 1.0, 0.0, Double.NaN);
    }

    /**
     * Creates an absolute periodic candidate with quality/timing metadata.
     */
    public static PlantTargetCandidate periodic(String id, double value, double period, double quality, double ageSec, double timestampSec) {
        return new PlantTargetCandidate(id, value, true, period, false, false, quality, ageSec, timestampSec);
    }

    /**
     * Creates a relative exact candidate: target = current measurement + delta.
     */
    public static PlantTargetCandidate relative(String id, double delta) {
        return new PlantTargetCandidate(id, delta, false, Double.NaN, false, true, 1.0, 0.0, Double.NaN);
    }

    /**
     * Creates a relative equivalent-position candidate using the downstream plant's period.
     */
    public static PlantTargetCandidate relativeEquivalentPosition(String id, double delta) {
        return relativeEquivalentPosition(id, delta, 1.0, 0.0, Double.NaN);
    }

    /**
     * Creates a relative equivalent-position candidate using the downstream plant's period.
     */
    public static PlantTargetCandidate relativeEquivalentPosition(String id,
                                                                  double delta,
                                                                  double quality,
                                                                  double ageSec,
                                                                  double timestampSec) {
        return new PlantTargetCandidate(id, delta, true, Double.NaN, true, true, quality, ageSec, timestampSec);
    }

    /**
     * Creates a relative periodic candidate: target = current measurement + delta + k*period.
     */
    public static PlantTargetCandidate relativePeriodic(String id, double delta, double period, double quality, double ageSec, double timestampSec) {
        return new PlantTargetCandidate(id, delta, true, period, false, true, quality, ageSec, timestampSec);
    }

    /**
     * Creates a relative periodic candidate: target = current measurement + delta + k*period.
     */
    public static PlantTargetCandidate relativePeriodic(String id, double delta, double period) {
        return relativePeriodic(id, delta, period, 1.0, 0.0, Double.NaN);
    }

    @Override
    public String toString() {
        return "PlantTargetCandidate{id='" + id + "', value=" + value
                + ", periodic=" + periodic + ", period=" + period
                + ", usesPlantPeriod=" + usesPlantPeriod
                + ", relative=" + relative + ", quality=" + quality
                + ", ageSec=" + ageSec + ", timestampSec=" + timestampSec + '}';
    }
}
