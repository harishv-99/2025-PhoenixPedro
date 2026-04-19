package edu.ftcphoenix.fw.spatial;

/**
 * Shared freshness/quality gate for selecting spatial lane solutions.
 *
 * <p>The gate is deliberately small and stateless. Higher-level planners can use it consistently
 * for drive guidance, scalar setpoint requests, and custom mechanism logic.</p>
 */
public final class SpatialSolutionGate {

    public final double maxAgeSec;
    public final double minQuality;

    private SpatialSolutionGate(double maxAgeSec, double minQuality) {
        this.maxAgeSec = maxAgeSec;
        this.minQuality = minQuality;
    }

    /**
     * Returns a permissive default gate.
     */
    public static SpatialSolutionGate defaults() {
        return builder().build();
    }

    /**
     * Starts building a solution gate.
     */
    public static Builder builder() {
        return new Builder();
    }

    /**
     * Returns true when a facing solution passes this gate.
     */
    public boolean accepts(FacingSolution solution) {
        return solution != null
                && solution.quality >= minQuality
                && (!Double.isFinite(maxAgeSec) || solution.ageSec <= maxAgeSec);
    }

    /**
     * Returns true when a translation solution passes this gate.
     */
    public boolean accepts(TranslationSolution solution) {
        return solution != null
                && solution.quality >= minQuality
                && (!Double.isFinite(maxAgeSec) || solution.ageSec <= maxAgeSec);
    }

    /**
     * Builder for {@link SpatialSolutionGate}.
     */
    public static final class Builder {
        private double maxAgeSec = Double.POSITIVE_INFINITY;
        private double minQuality = 0.0;

        /**
         * Reject solutions older than this age; use infinity to disable.
         */
        public Builder maxAgeSec(double maxAgeSec) {
            this.maxAgeSec = maxAgeSec;
            return this;
        }

        /**
         * Reject solutions below this lane-specific quality score.
         */
        public Builder minQuality(double minQuality) {
            this.minQuality = minQuality;
            return this;
        }

        /**
         * Builds the immutable gate.
         */
        public SpatialSolutionGate build() {
            return new SpatialSolutionGate(maxAgeSec, minQuality);
        }
    }
}
