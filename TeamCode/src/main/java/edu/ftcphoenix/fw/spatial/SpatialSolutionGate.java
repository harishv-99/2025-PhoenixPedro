package edu.ftcphoenix.fw.spatial;

import java.util.Objects;

import edu.ftcphoenix.fw.core.time.LoopTimestamp;

/**
 * Shared freshness/quality gate for selecting spatial lane solutions.
 *
 * <p>The gate is deliberately small and stateless. Higher-level planners can use it consistently
 * for drive guidance, Plant target requests, and custom mechanism logic.</p>
 */
public final class SpatialSolutionGate {

    private static final double SAMPLE_TIME_FUTURE_TOLERANCE_SEC = 1.0e-6;

    public final double maxAgeSec;
    public final double minQuality;

    private SpatialSolutionGate(double maxAgeSec, double minQuality) {
        if (Double.isNaN(maxAgeSec) || maxAgeSec < 0.0) {
            throw new IllegalArgumentException(
                    "maxAgeSec must be >= 0 or positive infinity, got " + maxAgeSec);
        }
        if (!Double.isFinite(minQuality)) {
            throw new IllegalArgumentException("minQuality must be finite, got " + minQuality);
        }
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
     * Returns true when a facing solution passes this gate at the supplied coherent query-sample
     * time. The two timestamps retain their clock and epoch, so reset-invalidated or mismatched
     * timing cannot be accepted as current data.
     */
    public boolean accepts(FacingSolution solution, LoopTimestamp sampleTimestamp) {
        return solution != null
                && solution.quality >= minQuality
                && acceptsTimestamp(solution.timestamp, sampleTimestamp);
    }

    /**
     * Returns true when a translation solution passes this gate at the supplied coherent
     * query-sample time.
     */
    public boolean accepts(TranslationSolution solution, LoopTimestamp sampleTimestamp) {
        return solution != null
                && solution.quality >= minQuality
                && acceptsTimestamp(solution.timestamp, sampleTimestamp);
    }

    private boolean acceptsTimestamp(LoopTimestamp timestamp, LoopTimestamp sampleTimestamp) {
        Objects.requireNonNull(timestamp, "timestamp");
        Objects.requireNonNull(sampleTimestamp, "sampleTimestamp");
        double ageSec = sampleTimestamp.secondsSince(timestamp);
        return Double.isFinite(ageSec)
                && ageSec >= -SAMPLE_TIME_FUTURE_TOLERANCE_SEC
                && (!Double.isFinite(maxAgeSec) || Math.max(0.0, ageSec) <= maxAgeSec);
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
