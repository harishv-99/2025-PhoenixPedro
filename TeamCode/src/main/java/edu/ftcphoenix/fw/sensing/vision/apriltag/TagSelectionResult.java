package edu.ftcphoenix.fw.sensing.vision.apriltag;

import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.Set;

/**
 * Immutable snapshot of a selector's preview and selected-tag state.
 *
 * <p>Phoenix intentionally distinguishes between two ideas:</p>
 * <ul>
 *   <li><b>Preview:</b> which tag would win <em>right now</em> if the selector needed to choose?</li>
 *   <li><b>Selection:</b> which tag is the robot currently committed to?</li>
 * </ul>
 *
 * <p>This matters for sticky aim-assist workflows. Before the driver enables aiming, telemetry can
 * preview the likely winner. Once enabled, the selector can latch that winner while still exposing
 * whether the selected tag is freshly visible this loop.</p>
 */
public final class TagSelectionResult {

    public final boolean hasPreview;
    public final int previewTagId;
    public final AprilTagObservation previewObservation;

    public final boolean hasSelection;
    public final int selectedTagId;
    public final boolean latched;
    public final boolean hasFreshSelectedObservation;
    public final AprilTagObservation selectedObservation;

    public final Set<Integer> visibleCandidateIds;

    public final String policyName;
    public final String reason;
    public final double metricValue;

    /**
     * Creates a full immutable selection snapshot.
     *
     * <p>The snapshot deliberately separates preview from committed selection so UI/telemetry can
     * answer both “what would win if enabled right now?” and “what tag is this behavior actually
     * committed to?”</p>
     */
    public TagSelectionResult(boolean hasPreview,
                              int previewTagId,
                              AprilTagObservation previewObservation,
                              boolean hasSelection,
                              int selectedTagId,
                              boolean latched,
                              boolean hasFreshSelectedObservation,
                              AprilTagObservation selectedObservation,
                              Set<Integer> visibleCandidateIds,
                              String policyName,
                              String reason,
                              double metricValue) {
        this.hasPreview = hasPreview;
        this.previewTagId = hasPreview ? previewTagId : -1;
        this.previewObservation = previewObservation != null ? previewObservation : AprilTagObservation.noTarget(Double.POSITIVE_INFINITY);
        this.hasSelection = hasSelection;
        this.selectedTagId = hasSelection ? selectedTagId : -1;
        this.latched = latched;
        this.hasFreshSelectedObservation = hasFreshSelectedObservation;
        this.selectedObservation = selectedObservation != null ? selectedObservation : AprilTagObservation.noTarget(Double.POSITIVE_INFINITY);
        LinkedHashSet<Integer> ids = new LinkedHashSet<Integer>();
        if (visibleCandidateIds != null) {
            ids.addAll(visibleCandidateIds);
        }
        this.visibleCandidateIds = Collections.unmodifiableSet(ids);
        this.policyName = (policyName == null || policyName.isEmpty()) ? "policy" : policyName;
        this.reason = (reason == null || reason.isEmpty()) ? this.policyName : reason;
        this.metricValue = metricValue;
    }

    /**
     * Returns a snapshot with no preview winner and no committed selection.
     */
    public static TagSelectionResult none(Set<Integer> visibleCandidateIds) {
        return new TagSelectionResult(
                false,
                -1,
                AprilTagObservation.noTarget(Double.POSITIVE_INFINITY),
                false,
                -1,
                false,
                false,
                AprilTagObservation.noTarget(Double.POSITIVE_INFINITY),
                visibleCandidateIds,
                "none",
                "no selection",
                Double.NaN
        );
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        return "TagSelectionResult{"
                + "hasPreview=" + hasPreview
                + ", previewTagId=" + previewTagId
                + ", hasSelection=" + hasSelection
                + ", selectedTagId=" + selectedTagId
                + ", latched=" + latched
                + ", hasFreshSelectedObservation=" + hasFreshSelectedObservation
                + ", visibleCandidateIds=" + visibleCandidateIds
                + ", policyName='" + policyName + '\''
                + ", reason='" + reason + '\''
                + ", metricValue=" + metricValue
                + '}';
    }
}
