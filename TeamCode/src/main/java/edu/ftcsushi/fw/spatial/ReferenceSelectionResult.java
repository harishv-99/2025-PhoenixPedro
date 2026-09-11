package edu.ftcsushi.fw.spatial;

import java.util.Objects;

import edu.ftcsushi.fw.sensing.observation.TargetSelectionResult;
import edu.ftcsushi.fw.sensing.vision.apriltag.TagSelectionResult;

/**
 * Immutable selection provenance shared by spatial queries and drive guidance.
 *
 * <p>This value retains the exact domain result; it does not sample a source, solve geometry,
 * refresh a timestamp, or manufacture a common physical identity. A selected tag, a selected
 * observation, and a computed approach have different evidence and expiry rules. Inspect the
 * typed result for those rules and the accompanying spatial solution for the evidence actually
 * used by that solve. A selection can remain inspectable when geometry is unavailable.</p>
 */
public final class ReferenceSelectionResult {
    /** The retained result's domain, not its current validity or the solve's sensor authority. */
    public enum Kind {
        /** No selection process; also used before a runtime has sampled a reference. */
        NONE,
        /** A tag identity and its original selection/preview evidence. */
        APRIL_TAG,
        /** A geometric choice from one captured observation frame, not tracked identity. */
        OBSERVED_TARGET,
        /** An observed or explicitly committed desired robot-center field pose. */
        APPROACH
    }

    private static final ReferenceSelectionResult NONE =
            new ReferenceSelectionResult(Kind.NONE, null, null, null);

    private final Kind kind;
    private final TagSelectionResult aprilTag;
    private final TargetSelectionResult observedTarget;
    private final ApproachResult2d approach;

    private ReferenceSelectionResult(Kind kind, TagSelectionResult aprilTag,
                                     TargetSelectionResult observedTarget, ApproachResult2d approach) {
        this.kind = kind;
        this.aprilTag = aprilTag;
        this.observedTarget = observedTarget;
        this.approach = approach;
    }

    /** Returns the shared immutable absence of a selection process or sampled reference. */
    public static ReferenceSelectionResult none() { return NONE; }

    /** Retains the exact immutable tag result, including an unavailable selection's evidence. */
    public static ReferenceSelectionResult aprilTag(TagSelectionResult result) {
        return new ReferenceSelectionResult(Kind.APRIL_TAG,
                Objects.requireNonNull(result, "tag selection result"), null, null);
    }

    /** Retains the exact frame selection, original capture time, freshness policy, and reason. */
    public static ReferenceSelectionResult observedTarget(TargetSelectionResult result) {
        return new ReferenceSelectionResult(Kind.OBSERVED_TARGET, null,
                Objects.requireNonNull(result, "observed target selection result"), null);
    }

    /** Retains the exact approach and its observed-versus-committed evidence contract. */
    public static ReferenceSelectionResult approach(ApproachResult2d result) {
        return new ReferenceSelectionResult(Kind.APPROACH, null, null,
                Objects.requireNonNull(result, "approach result"));
    }

    /** Returns the retained result's domain, independently of selection or geometry availability. */
    public Kind kind() { return kind; }

    /**
     * Whether the retained result contains a choice or approach, not whether it is usable now.
     * Solved geometry and the typed result's evidence rules remain separate from this fact.
     */
    public boolean hasSelection() {
        switch (kind) {
            case APRIL_TAG: return aprilTag.hasSelection;
            case OBSERVED_TARGET: return observedTarget.hasSelection();
            case APPROACH: return approach.hasApproach();
            default: return false;
        }
    }

    /**
     * Returns the retained tag result, including its stable producer ID and held-selection facts.
     * @throws IllegalStateException unless {@link #kind()} is {@link Kind#APRIL_TAG}
     */
    public TagSelectionResult aprilTag() {
        requireKind(Kind.APRIL_TAG);
        return aprilTag;
    }

    /**
     * Returns the retained observation selection without inventing a tag or tracked-object ID.
     * @throws IllegalStateException unless {@link #kind()} is {@link Kind#OBSERVED_TARGET}
     */
    public TargetSelectionResult observedTarget() {
        requireKind(Kind.OBSERVED_TARGET);
        return observedTarget;
    }

    /**
     * Returns the retained computed approach, not arrival or capture confirmation.
     * @throws IllegalStateException unless {@link #kind()} is {@link Kind#APPROACH}
     */
    public ApproachResult2d approach() {
        requireKind(Kind.APPROACH);
        return approach;
    }

    /** Rejects cross-domain reads instead of returning an unrelated or fabricated empty value. */
    private void requireKind(Kind expected) {
        if (kind != expected) {
            throw new IllegalStateException("Reference selection kind is " + kind
                    + ", not " + expected + "; inspect kind() before reading its typed result");
        }
    }

    /** Returns compact cached provenance without sampling any source or current clock. */
    @Override public String toString() {
        return "ReferenceSelectionResult{kind=" + kind + ", hasSelection=" + hasSelection() + '}';
    }
}
