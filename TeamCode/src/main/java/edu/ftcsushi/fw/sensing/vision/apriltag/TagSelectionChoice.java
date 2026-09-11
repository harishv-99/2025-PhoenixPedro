package edu.ftcsushi.fw.sensing.vision.apriltag;

import java.util.Objects;

/**
 * Explanation of a {@link TagSelectionPolicy} decision.
 */
public final class TagSelectionChoice {

    /**
     * Winning candidate from this policy invocation's supplied list.
     */
    public final TagSelectionCandidate candidate;
    /**
     * Stable policy name for telemetry/debug.
     */
    public final String policyName;
    /**
     * Human-readable explanation, usually matching the policy's ranking rule.
     */
    public final String reason;
    /**
     * Primary numeric metric used to rank the winning observation.
     */
    public final double metricValue;

    /**
     * Creates an explanation of one policy decision.
     *
     * @param candidate winning candidate; must be an instance from the supplied candidate list
     * @param policyName  stable policy identifier for telemetry/debug
     * @param reason      human-readable explanation of the ranking rule that won
     * @param metricValue primary numeric metric used by the policy
     */
    public TagSelectionChoice(TagSelectionCandidate candidate,
                              String policyName,
                              String reason,
                              double metricValue) {
        this.candidate = Objects.requireNonNull(candidate, "candidate");
        if (!Double.isFinite(metricValue)) {
            throw new IllegalArgumentException("metricValue must be finite");
        }
        this.policyName = (policyName == null || policyName.isEmpty()) ? "policy" : policyName;
        this.reason = (reason == null || reason.isEmpty()) ? this.policyName : reason;
        this.metricValue = metricValue;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        return "TagSelectionChoice{tagId=" + candidate.tagId
                + ", policyName='" + policyName + '\''
                + ", reason='" + reason + '\''
                + ", metricValue=" + metricValue
                + '}';
    }
}
