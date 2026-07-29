package edu.ftcphoenix.fw.actuation;

import java.util.Objects;

import edu.ftcphoenix.fw.core.source.ScalarTarget;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;

/**
 * Result of resolving a {@link PlantTargetSource} for one plant update.
 *
 * <p>A plan describes target <em>selection</em>, not physical arrival. It tells the plant which
 * requested target should be considered for this loop and why that target was chosen. Completion
 * still belongs to {@link Plant#atTarget()} and {@link Plant#atTarget(double)}, because only the
 * plant knows its feedback tolerance, hardware guards, and applied target. Framework-created plans
 * also carry private winning-command evidence so {@link ScalarTasks} can distinguish a logical
 * command from a same-valued overlay or fallback without adding another public status concept.</p>
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
         * A candidate was selected from an advanced request.
         */
        PLANNED_CANDIDATE,
        /**
         * A legal physical representative of an equivalent-position family was selected.
         */
        EQUIVALENT_POSITION,
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
    private final LoopTimestamp selectedTimestamp;
    private final String reason;
    private final ScalarTarget resolutionCommandTarget;
    private final boolean commandPathSelected;
    private final double logicalCommandValue;

    private PlantTargetPlan(boolean hasTarget,
                            double target,
                            Kind kind,
                            boolean satisfiesRequest,
                            boolean usedFallback,
                            boolean clampedByPlanner,
                            String selectedCandidateId,
                            double selectedQuality,
                            double selectedAgeSec,
                            LoopTimestamp selectedTimestamp,
                            String reason,
                            ScalarTarget resolutionCommandTarget,
                            boolean commandPathSelected,
                            double logicalCommandValue) {
        this.hasTarget = hasTarget;
        this.target = target;
        this.kind = Objects.requireNonNull(kind, "kind");
        this.satisfiesRequest = satisfiesRequest;
        this.usedFallback = usedFallback;
        this.clampedByPlanner = clampedByPlanner;
        this.selectedCandidateId = selectedCandidateId != null ? selectedCandidateId : "";
        this.selectedQuality = selectedQuality;
        this.selectedAgeSec = selectedAgeSec;
        this.selectedTimestamp = Objects.requireNonNull(selectedTimestamp, "selectedTimestamp");
        this.reason = reason != null ? reason : "";
        this.resolutionCommandTarget = resolutionCommandTarget;
        this.commandPathSelected = commandPathSelected;
        this.logicalCommandValue = logicalCommandValue;
    }

    /**
     * Create a simple exact target plan.
     */
    public static PlantTargetPlan exact(double target, String reason) {
        requireFinite(target, "target");
        return new PlantTargetPlan(true, target, Kind.EXACT, true, false, false,
                "exact", 1.0, Double.NaN, LoopTimestamp.unavailable(),
                clean(reason, "exact target"), null, false, Double.NaN);
    }

    /** Create a planned candidate after the framework planner has validated its metadata. */
    static PlantTargetPlan planned(double target,
                                   PlantTargetCandidate candidate,
                                   double selectedAgeSec,
                                   boolean clampedByPlanner,
                                   String reason) {
        requireFinite(target, "target");
        Objects.requireNonNull(candidate, "candidate");
        if (candidate.isObserved()
                && (!Double.isFinite(selectedAgeSec) || selectedAgeSec < 0.0)) {
            throw new IllegalArgumentException(
                    "selectedAgeSec for an observed candidate must be finite and >= 0, got "
                            + selectedAgeSec);
        }
        return new PlantTargetPlan(true, target, Kind.PLANNED_CANDIDATE,
                !clampedByPlanner,
                false,
                clampedByPlanner,
                candidate.id,
                candidate.quality,
                candidate.isObserved() ? selectedAgeSec : Double.NaN,
                candidate.isObserved() ? candidate.timestamp : LoopTimestamp.unavailable(),
                clean(reason, "planned target"), null, false, Double.NaN);
    }

    /**
     * Create a fallback target plan.
     */
    public static PlantTargetPlan fallback(double target, String reason) {
        requireFinite(target, "target");
        return new PlantTargetPlan(true, target, Kind.FALLBACK, false, true, false,
                "fallback", 1.0, Double.NaN, LoopTimestamp.unavailable(),
                clean(reason, "fallback target"), null, false, Double.NaN);
    }

    /**
     * Create a hold-last target plan.
     */
    public static PlantTargetPlan holdLast(double target, String reason) {
        requireFinite(target, "target");
        return new PlantTargetPlan(true, target, Kind.HOLD_LAST_TARGET, false, true, false,
                "hold-last", 1.0, Double.NaN, LoopTimestamp.unavailable(),
                clean(reason, "holding last target"), null, false, Double.NaN);
    }

    /**
     * Create a hold-measured target plan.
     */
    public static PlantTargetPlan holdMeasured(double target, String reason) {
        requireFinite(target, "target");
        return new PlantTargetPlan(true, target, Kind.HOLD_MEASURED_TARGET, false, true, false,
                "hold-measured", 1.0, Double.NaN, LoopTimestamp.unavailable(),
                clean(reason, "holding measured target"), null, false, Double.NaN);
    }

    /**
     * Create a plan with no commandable target.
     */
    public static PlantTargetPlan unavailable(String reason) {
        return new PlantTargetPlan(false, Double.NaN, Kind.UNAVAILABLE, false, false, false,
                "", Double.NaN, Double.NaN, LoopTimestamp.unavailable(),
                clean(reason, "target unavailable"), null, false, Double.NaN);
    }

    /** Attach internal evidence that this plan selected one graph-owned command value. */
    PlantTargetPlan withSelectedCommand(ScalarTarget commandTarget, double logicalValue) {
        Objects.requireNonNull(commandTarget, "commandTarget");
        requireFinite(logicalValue, "logicalValue");
        if (!hasTarget) {
            throw new IllegalStateException("Cannot select a command path without a target");
        }
        return copy(target, kind, reason, commandTarget, true, logicalValue);
    }

    /** Attach internal evidence that this graph's command path did not produce the final target. */
    PlantTargetPlan withoutSelectedCommand(ScalarTarget commandTarget) {
        return copy(target, kind, reason, Objects.requireNonNull(commandTarget, "commandTarget"),
                false, Double.NaN);
    }

    /**
     * Return this plan with a selected physical equivalent while retaining selection metadata.
     */
    PlantTargetPlan withEquivalentTarget(double equivalentTarget, String equivalentReason) {
        requireFinite(equivalentTarget, "equivalentTarget");
        if (!hasTarget) {
            throw new IllegalStateException("Cannot select an equivalent position without a target");
        }
        Kind resolvedKind = satisfiesRequest && !usedFallback && !clampedByPlanner
                ? Kind.EQUIVALENT_POSITION
                : kind;
        return copy(equivalentTarget, resolvedKind,
                clean(equivalentReason, "selected equivalent position"),
                resolutionCommandTarget, commandPathSelected, logicalCommandValue);
    }

    /** True when this plan reports the winning-path relation for {@code commandTarget}. */
    boolean reportsCommandResolutionFor(ScalarTarget commandTarget) {
        return resolutionCommandTarget == commandTarget;
    }

    /** True when this plan represents the supplied logical value from the supplied command path. */
    boolean satisfiesCommand(ScalarTarget commandTarget, double logicalValue) {
        return reportsCommandResolutionFor(commandTarget)
                && commandPathSelected
                && logicalCommandValue == logicalValue
                && hasTarget
                && satisfiesRequest
                && !usedFallback
                && !clampedByPlanner;
    }

    private PlantTargetPlan copy(double copiedTarget,
                                 Kind copiedKind,
                                 String copiedReason,
                                 ScalarTarget copiedCommandTarget,
                                 boolean copiedCommandPathSelected,
                                 double copiedLogicalCommandValue) {
        return new PlantTargetPlan(hasTarget, copiedTarget, copiedKind, satisfiesRequest,
                usedFallback, clampedByPlanner, selectedCandidateId, selectedQuality,
                selectedAgeSec, selectedTimestamp, copiedReason, copiedCommandTarget,
                copiedCommandPathSelected, copiedLogicalCommandValue);
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
     * Age of the chosen observation when the plan was resolved, in seconds.
     *
     * <p>Returns {@link Double#NaN} unless this plan selected an observed candidate.</p>
     */
    public double selectedAgeSec() {
        return selectedAgeSec;
    }

    /**
     * Epoch-safe timestamp of the chosen observation.
     *
     * <p>Returns {@link LoopTimestamp#unavailable()} unless this plan selected an observed
     * candidate.</p>
     */
    public LoopTimestamp selectedTimestamp() {
        return selectedTimestamp;
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
