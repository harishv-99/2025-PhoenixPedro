package edu.ftcphoenix.robots.phoenix;

/** Human-acknowledged Phoenix calibration evidence. */
public final class PhoenixCalibrationConfig {

    /** Whether Phoenix's Pinpoint axis directions have been physically verified. */
    public boolean pinpointAxesVerified = false;

    /** Whether Phoenix's Pinpoint pod offsets have been physically calibrated. */
    public boolean pinpointPodOffsetsCalibrated = false;

    private PhoenixCalibrationConfig() {
    }

    /**
     * Returns a fresh conservative software baseline with no physical acknowledgement asserted.
     *
     * @return fresh mutable calibration-evidence draft
     */
    public static PhoenixCalibrationConfig defaults() {
        return new PhoenixCalibrationConfig();
    }
}
