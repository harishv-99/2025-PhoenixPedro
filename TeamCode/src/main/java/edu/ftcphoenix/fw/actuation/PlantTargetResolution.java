package edu.ftcphoenix.fw.actuation;

import java.util.Objects;

import edu.ftcphoenix.fw.core.source.ScalarTarget;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;

/**
 * Immutable result of resolving a {@link PlantTargetResolver} for one Plant update.
 *
 * <p>A resolution describes target <em>selection</em>, not physical arrival. It tells the Plant
 * which requested target should be considered for this loop and why that target was chosen.
 * Completion still belongs to {@link Plant#atTarget()} and {@link Plant#atTarget(double)}, because
 * only the Plant knows its feedback tolerance, hardware guards, and applied target.
 * Framework-created resolutions also carry private winning-command evidence so
 * {@link ScalarTasks} can distinguish a logical command from a same-valued overlay or fallback
 * without adding another public status concept.</p>
 */
public final class PlantTargetResolution {

    /**
     * High-level reason a requested target was produced.
     */
    public enum Kind {
        /** A plain exact scalar value was used. */
        EXACT,
        /** An alternative was selected from an advanced request. */
        PLANNED_CANDIDATE,
        /** A legal physical representative of an equivalent-position family was selected. */
        EQUIVALENT_POSITION,
        /** The target came from a fallback value. */
        FALLBACK,
        /** The target held a previously produced target. */
        HOLD_LAST_TARGET,
        /** The target latched a measured position. */
        HOLD_MEASURED_TARGET,
        /** No requested target was available. Final resolvers bound to Plants should avoid this. */
        UNAVAILABLE
    }

    private final boolean hasTarget;
    private final double target;
    private final Kind kind;
    private final boolean satisfiesIntent;
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

    private PlantTargetResolution(boolean hasTarget,
                                  double target,
                                  Kind kind,
                                  boolean satisfiesIntent,
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
        this.satisfiesIntent = satisfiesIntent;
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
     * Creates a simple exact target resolution.
     */
    public static PlantTargetResolution exact(double target, String reason) {
        requireFinite(target, "target");
        return new PlantTargetResolution(true, target, Kind.EXACT, true, false, false,
                "exact", 1.0, Double.NaN, LoopTimestamp.unavailable(),
                clean(reason, "exact target"), null, false, Double.NaN);
    }

    /** Creates a planned alternative after the framework planner validates its metadata. */
    static PlantTargetResolution planned(double target,
                                         PlantTargetRequest.Alternative alternative,
                                         double selectedAgeSec,
                                         boolean clampedByPlanner,
                                         String reason) {
        requireFinite(target, "target");
        Objects.requireNonNull(alternative, "alternative");
        if (alternative.observed()
                && (!Double.isFinite(selectedAgeSec) || selectedAgeSec < 0.0)) {
            throw new IllegalArgumentException(
                    "selectedAgeSec for an observed target alternative must be finite and >= 0, "
                            + "got " + selectedAgeSec);
        }
        return new PlantTargetResolution(true, target, Kind.PLANNED_CANDIDATE,
                !clampedByPlanner,
                false,
                clampedByPlanner,
                alternative.id(),
                alternative.quality(),
                alternative.observed() ? selectedAgeSec : Double.NaN,
                alternative.observed() ? alternative.timestamp() : LoopTimestamp.unavailable(),
                clean(reason, "planned target"), null, false, Double.NaN);
    }

    /**
     * Creates a fallback target resolution.
     */
    public static PlantTargetResolution fallback(double target, String reason) {
        requireFinite(target, "target");
        return new PlantTargetResolution(true, target, Kind.FALLBACK, false, true, false,
                "fallback", 1.0, Double.NaN, LoopTimestamp.unavailable(),
                clean(reason, "fallback target"), null, false, Double.NaN);
    }

    /**
     * Creates a hold-last target resolution.
     */
    public static PlantTargetResolution holdLast(double target, String reason) {
        requireFinite(target, "target");
        return new PlantTargetResolution(true, target, Kind.HOLD_LAST_TARGET, false, true, false,
                "hold-last", 1.0, Double.NaN, LoopTimestamp.unavailable(),
                clean(reason, "holding last target"), null, false, Double.NaN);
    }

    /**
     * Creates a hold-measured target resolution.
     */
    public static PlantTargetResolution holdMeasured(double target, String reason) {
        requireFinite(target, "target");
        return new PlantTargetResolution(true, target, Kind.HOLD_MEASURED_TARGET,
                false, true, false,
                "hold-measured", 1.0, Double.NaN, LoopTimestamp.unavailable(),
                clean(reason, "holding measured target"), null, false, Double.NaN);
    }

    /**
     * Creates a resolution with no commandable target.
     */
    public static PlantTargetResolution unavailable(String reason) {
        return new PlantTargetResolution(false, Double.NaN, Kind.UNAVAILABLE,
                false, false, false,
                "", Double.NaN, Double.NaN, LoopTimestamp.unavailable(),
                clean(reason, "target unavailable"), null, false, Double.NaN);
    }

    /** Attach internal evidence that this resolution selected one graph-owned command value. */
    PlantTargetResolution withSelectedCommand(ScalarTarget commandTarget, double logicalValue) {
        Objects.requireNonNull(commandTarget, "commandTarget");
        requireFinite(logicalValue, "logicalValue");
        if (!hasTarget) {
            throw new IllegalStateException("Cannot select a command path without a target");
        }
        return copy(target, kind, reason, commandTarget, true, logicalValue);
    }

    /** Attach internal evidence that this graph's command path did not produce the final target. */
    PlantTargetResolution withoutSelectedCommand(ScalarTarget commandTarget) {
        return copy(target, kind, reason, Objects.requireNonNull(commandTarget, "commandTarget"),
                false, Double.NaN);
    }

    /**
     * Returns this resolution with a selected physical equivalent while retaining provenance.
     */
    PlantTargetResolution withEquivalentTarget(double equivalentTarget, String equivalentReason) {
        requireFinite(equivalentTarget, "equivalentTarget");
        if (!hasTarget) {
            throw new IllegalStateException("Cannot select an equivalent position without a target");
        }
        Kind resolvedKind = satisfiesIntent && !usedFallback && !clampedByPlanner
                ? Kind.EQUIVALENT_POSITION
                : kind;
        return copy(equivalentTarget, resolvedKind,
                clean(equivalentReason, "selected equivalent position"),
                resolutionCommandTarget, commandPathSelected, logicalCommandValue);
    }

    /** True when this resolution reports the winning-path relation for {@code commandTarget}. */
    boolean reportsCommandResolutionFor(ScalarTarget commandTarget) {
        return resolutionCommandTarget == commandTarget;
    }

    /** True when this resolution represents the supplied value from the supplied command path. */
    boolean satisfiesCommand(ScalarTarget commandTarget, double logicalValue) {
        return reportsCommandResolutionFor(commandTarget)
                && commandPathSelected
                && logicalCommandValue == logicalValue
                && hasTarget
                && satisfiesIntent
                && !usedFallback
                && !clampedByPlanner;
    }

    private PlantTargetResolution copy(double copiedTarget,
                                       Kind copiedKind,
                                       String copiedReason,
                                       ScalarTarget copiedCommandTarget,
                                       boolean copiedCommandPathSelected,
                                       double copiedLogicalCommandValue) {
        return new PlantTargetResolution(hasTarget, copiedTarget, copiedKind, satisfiesIntent,
                usedFallback, clampedByPlanner, selectedCandidateId, selectedQuality,
                selectedAgeSec, selectedTimestamp, copiedReason, copiedCommandTarget,
                copiedCommandPathSelected, copiedLogicalCommandValue);
    }

    /**
     * Returns true when this resolution contains a finite requested target value.
     */
    public boolean hasTarget() {
        return hasTarget;
    }

    /**
     * Returns the requested target value in Plant units.
     *
     * @throws IllegalStateException if {@link #hasTarget()} is false
     */
    public double target() {
        if (!hasTarget) {
            throw new IllegalStateException("PlantTargetResolution has no target: " + reason);
        }
        return target;
    }

    /**
     * Returns why this resolution was produced.
     */
    public Kind kind() {
        return kind;
    }

    /**
     * Returns true when the target satisfies the active logical intent, rather than a fallback,
     * hold, or planner clamp.
     */
    public boolean satisfiesIntent() {
        return satisfiesIntent;
    }

    /**
     * Returns true when fallback or hold policy produced the target instead of the active intent.
     */
    public boolean usedFallback() {
        return usedFallback;
    }

    /**
     * Returns true when the planner clamped an unreachable requested value into the legal range.
     */
    public boolean clampedByPlanner() {
        return clampedByPlanner;
    }

    /**
     * Returns the identifier of the selected request alternative, useful in telemetry.
     */
    public String selectedCandidateId() {
        return selectedCandidateId;
    }

    /**
     * Returns quality metadata copied from the selected request alternative.
     */
    public double selectedQuality() {
        return selectedQuality;
    }

    /**
     * Returns the age of the selected observation at resolution, in seconds.
     *
     * <p>Returns {@link Double#NaN} unless this resolution selected an observed alternative.</p>
     */
    public double selectedAgeSec() {
        return selectedAgeSec;
    }

    /**
     * Returns the epoch-safe timestamp of the selected observation.
     *
     * <p>Returns {@link LoopTimestamp#unavailable()} unless this resolution selected an observed
     * alternative.</p>
     */
    public LoopTimestamp selectedTimestamp() {
        return selectedTimestamp;
    }

    /**
     * Returns a short human-readable explanation for debug telemetry.
     */
    public String reason() {
        return reason;
    }

    private static void requireFinite(double value, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite, got " + value);
        }
    }

    private static String clean(String text, String fallback) {
        return text == null || text.trim().isEmpty() ? fallback : text.trim();
    }

    @Override
    public String toString() {
        return hasTarget
                ? "PlantTargetResolution{" + kind + ", target=" + target
                        + ", candidate='" + selectedCandidateId + "', reason='" + reason + "'}"
                : "PlantTargetResolution{UNAVAILABLE, reason='" + reason + "'}";
    }
}
