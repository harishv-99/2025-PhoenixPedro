package edu.ftcphoenix.fw.sensing.vision.apriltag;

import java.util.Objects;

/**
 * Explanation of a {@link TagSelectionPolicy} decision.
 */
public final class TagSelectionChoice {

    /**
     * Winning observation.
     */
    public final AprilTagObservation observation;
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
     * @param observation winning observation; must contain a target
     * @param policyName  stable policy identifier for telemetry/debug
     * @param reason      human-readable explanation of the ranking rule that won
     * @param metricValue primary numeric metric used by the policy
     */
    public TagSelectionChoice(AprilTagObservation observation,
                              String policyName,
                              String reason,
                              double metricValue) {
        this.observation = Objects.requireNonNull(observation, "observation");
        if (!observation.hasTarget) {
            throw new IllegalArgumentException("observation must contain a target");
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
        return "TagSelectionChoice{observation=" + observation.id
                + ", policyName='" + policyName + '\''
                + ", reason='" + reason + '\''
                + ", metricValue=" + metricValue
                + '}';
    }
}
