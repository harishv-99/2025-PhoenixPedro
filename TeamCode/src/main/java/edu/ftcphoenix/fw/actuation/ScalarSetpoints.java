package edu.ftcphoenix.fw.actuation;

/**
 * Entry-point helpers for scalar setpoint planning above {@link Plant}s.
 */
public final class ScalarSetpoints {
    private ScalarSetpoints() {
    }

    /**
     * Starts building a {@link ScalarSetpointPlanner}.
     */
    public static ScalarSetpointPlanner.Builder plan() {
        return ScalarSetpointPlanner.builder();
    }
}
