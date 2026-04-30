package edu.ftcphoenix.fw.actuation;

import java.util.Objects;

/**
 * Result of resolving a {@link PlantTargetSource} for one plant update.
 *
 * <p>A plan describes target <em>selection</em>, not physical arrival. It tells the plant which
 * requested target should be considered for this loop and why that target was chosen. Completion
 * still belongs to {@link Plant#atTarget()} and {@link Plant#atTarget(double)}, because only the
 * plant knows its feedback tolerance, hardware guards, and applied target.</p>
 */
public final class PlantTargetPlan {

    /**
     * High-level reason a requested target was produced.
     */
    public enum Kind {
        /**
         * A plain exact scalar value was used.
         */
        EXACT,
        /**
         * A candidate/equivalent target was selected from a request.
         */
        PLANNED_CANDIDATE,
        /**
         * The target came from a fallback value.
         */
        FALLBACK,
        /**
         * The target held a previously produced target.
         */
        HOLD_LAST_TARGET,
        /**
         * The target latched a measured position.
         */
        HOLD_MEASURED_TARGET,
        /**
         * No requested target was available. Final sources bound to plants should avoid this.
         */
        UNAVAILABLE
    }

    private final boolean hasTarget;
    private final double target;
    private final Kind kind;
    private final boolean satisfiesRequest;
    private final boolean usedFallback;
    private final boolean clampedByPlanner;
    private final String selectedCandidateId;
    private final double selectedQuality;
    private final double selectedAgeSec;
    private final double selectedTimestampSec;
    private final String reason;

    private PlantTargetPlan(boolean hasTarget,
                            double target,
                            Kind kind,
                            boolean satisfiesRequest,
                            boolean usedFallback,
                            boolean clampedByPlanner,
                            String selectedCandidateId,
                            double selectedQuality,
                            double selectedAgeSec,
                            double selectedTimestampSec,
                            String reason) {
        this.hasTarget = hasTarget;
        this.target = target;
        this.kind = Objects.requireNonNull(kind, "kind");
        this.satisfiesRequest = satisfiesRequest;
        this.usedFallback = usedFallback;
        this.clampedByPlanner = clampedByPlanner;
        this.selectedCandidateId = selectedCandidateId != null ? selectedCandidateId : "";
        this.selectedQuality = selectedQuality;
        this.selectedAgeSec = selectedAgeSec;
        this.selectedTimestampSec = selectedTimestampSec;
        this.reason = reason != null ? reason : "";
    }

    /**
     * Create a simple exact target plan.
     */
    public static PlantTargetPlan exact(double target, String reason) {
        requireFinite(target, "target");
        return new PlantTargetPlan(true, target, Kind.EXACT, true, false, false,
                "exact", 1.0, 0.0, Double.NaN, clean(reason, "exact target"));
    }

    /**
     * Create a planned candidate target.
     */
    public static PlantTargetPlan planned(double target,
                                          PlantTargetCandidate candidate,
                                          boolean clampedByPlanner,
                                          String reason) {
        requireFinite(target, "target");
        Objects.requireNonNull(candidate, "candidate");
        return new PlantTargetPlan(true, target, Kind.PLANNED_CANDIDATE,
                !clampedByPlanner,
                false,
                clampedByPlanner,
                candidate.id,
                candidate.quality,
                candidate.ageSec,
                candidate.timestampSec,
                clean(reason, "planned target"));
    }

    /**
     * Create a fallback target plan.
     */
    public static PlantTargetPlan fallback(double target, String reason) {
        requireFinite(target, "target");
        return new PlantTargetPlan(true, target, Kind.FALLBACK, false, true, false,
                "fallback", 1.0, 0.0, Double.NaN, clean(reason, "fallback target"));
    }

    /**
     * Create a hold-last target plan.
     */
    public static PlantTargetPlan holdLast(double target, String reason) {
        requireFinite(target, "target");
        return new PlantTargetPlan(true, target, Kind.HOLD_LAST_TARGET, false, true, false,
                "hold-last", 1.0, 0.0, Double.NaN, clean(reason, "holding last target"));
    }

    /**
     * Create a hold-measured target plan.
     */
    public static PlantTargetPlan holdMeasured(double target, String reason) {
        requireFinite(target, "target");
        return new PlantTargetPlan(true, target, Kind.HOLD_MEASURED_TARGET, false, true, false,
                "hold-measured", 1.0, 0.0, Double.NaN, clean(reason, "holding measured target"));
    }

    /**
     * Create a plan with no commandable target.
     */
    public static PlantTargetPlan unavailable(String reason) {
        return new PlantTargetPlan(false, Double.NaN, Kind.UNAVAILABLE, false, false, false,
                "", Double.NaN, Double.NaN, Double.NaN, clean(reason, "target unavailable"));
    }

    /**
     * True when this plan contains a finite requested target value.
     */
    public boolean hasTarget() {
        return hasTarget;
    }

    /**
     * Requested target value in plant units.
     *
     * @throws IllegalStateException if {@link #hasTarget()} is false
     */
    public double target() {
        if (!hasTarget) throw new IllegalStateException("PlantTargetPlan has no target: " + reason);
        return target;
    }

    /**
     * Why this plan was produced.
     */
    public Kind kind() {
        return kind;
    }

    /**
     * True when the target is a real member of the original request, not fallback or clamp output.
     */
    public boolean satisfiesRequest() {
        return satisfiesRequest;
    }

    /**
     * True when fallback/hold policy produced the target instead of the active request itself.
     */
    public boolean usedFallback() {
        return usedFallback;
    }

    /**
     * True when the planner clamped an unreachable requested value into the legal range.
     */
    public boolean clampedByPlanner() {
        return clampedByPlanner;
    }

    /**
     * Identifier of the chosen candidate, useful in telemetry.
     */
    public String selectedCandidateId() {
        return selectedCandidateId;
    }

    /**
     * Quality metadata copied from the chosen candidate.
     */
    public double selectedQuality() {
        return selectedQuality;
    }

    /**
     * Age metadata copied from the chosen candidate, in seconds.
     */
    public double selectedAgeSec() {
        return selectedAgeSec;
    }

    /**
     * Timestamp metadata copied from the chosen candidate, in seconds, if supplied.
     */
    public double selectedTimestampSec() {
        return selectedTimestampSec;
    }

    /**
     * Short human-readable explanation for debug telemetry.
     */
    public String reason() {
        return reason;
    }

    private static void requireFinite(double value, String name) {
        if (!Double.isFinite(value))
            throw new IllegalArgumentException(name + " must be finite, got " + value);
    }

    private static String clean(String text, String fallback) {
        return text == null || text.trim().isEmpty() ? fallback : text.trim();
    }

    @Override
    public String toString() {
        return hasTarget
                ? "PlantTargetPlan{" + kind + ", target=" + target + ", candidate='" + selectedCandidateId + "', reason='" + reason + "'}"
                : "PlantTargetPlan{UNAVAILABLE, reason='" + reason + "'}";
    }
}
