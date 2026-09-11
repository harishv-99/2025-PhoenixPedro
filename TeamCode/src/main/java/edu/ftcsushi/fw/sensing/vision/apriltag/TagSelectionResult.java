package edu.ftcsushi.fw.sensing.vision.apriltag;

import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.Set;

import edu.ftcsushi.fw.core.time.LoopTimestamp;

/**
 * Immutable selected identity, current preview, and evidence snapshot. A held identity is intent,
 * not continued visibility. The frozen selection decision is separate from current geometry:
 * consumers must never reinterpret its old observation as a new sighting.
 */
public final class TagSelectionResult {
    /** Choice that would win now, or null. May name a different tag from the held selection. */
    public final TagSelectionChoice previewChoice;
    /** Decision that established this selection; frozen while held, null for an authored ID. */
    public final TagSelectionChoice selectionDecision;
    /** Current usable geometry for the selected ID, or null when unavailable. */
    public final TagSelectionCandidate currentSelectedCandidate;
    public final boolean hasPreview;
    public final int previewTagId;
    public final boolean hasSelection;
    public final int selectedTagId;
    public final boolean latched;
    /** True only for current actual observed geometry, never for field-pose calculations. */
    public final boolean hasFreshSelectedObservation;
    /** Current actual observation or {@link AprilTagObservation#noTarget()}. */
    public final AprilTagObservation selectedObservation;
    /** Eligible IDs actually reported by a fresh camera frame, not inferred from field pose. */
    public final Set<Integer> visibleCandidateIds;
    /** Frame evidence time; unavailable means visibility is UNKNOWN, not an observed empty view. */
    public final LoopTimestamp visibilityTimestamp;

    TagSelectionResult(TagSelectionChoice previewChoice, int selectedTagId, boolean latched,
                       TagSelectionChoice selectionDecision,
                       TagSelectionCandidate currentSelectedCandidate,
                       Set<Integer> visibleCandidateIds, LoopTimestamp visibilityTimestamp) {
        this.previewChoice = previewChoice;
        this.hasPreview = previewChoice != null;
        this.previewTagId = hasPreview ? previewChoice.candidate.tagId : -1;
        this.selectedTagId = selectedTagId;
        this.hasSelection = selectedTagId >= 0;
        this.latched = hasSelection && latched;
        this.selectionDecision = selectionDecision;
        this.currentSelectedCandidate = currentSelectedCandidate;
        this.hasFreshSelectedObservation = currentSelectedCandidate != null
                && currentSelectedCandidate.evidenceKind == TagSelectionCandidate.EvidenceKind.OBSERVED
                && currentSelectedCandidate.observation != null;
        this.selectedObservation = hasFreshSelectedObservation
                ? currentSelectedCandidate.observation : AprilTagObservation.noTarget();
        this.visibleCandidateIds = Collections.unmodifiableSet(new LinkedHashSet<>(visibleCandidateIds));
        this.visibilityTimestamp = visibilityTimestamp;
    }

    /** No identity, decision, geometry, or visibility evidence. */
    public static TagSelectionResult none() {
        return new TagSelectionResult(null, -1, false, null, null,
                Collections.emptySet(), LoopTimestamp.unavailable());
    }

    /** Authored identity only: no fabricated policy decision, geometry, or visibility. */
    public static TagSelectionResult forTagId(int tagId) {
        if (tagId < 0) throw new IllegalArgumentException("tagId must be non-negative");
        return new TagSelectionResult(null, tagId, false, null, null,
                Collections.emptySet(), LoopTimestamp.unavailable());
    }

    @Override public String toString() {
        return "TagSelectionResult{previewTagId=" + previewTagId + ", selectedTagId=" + selectedTagId
                + ", latched=" + latched + ", currentSelectedCandidate=" + currentSelectedCandidate
                + ", visibleCandidateIds=" + visibleCandidateIds + '}';
    }
}
