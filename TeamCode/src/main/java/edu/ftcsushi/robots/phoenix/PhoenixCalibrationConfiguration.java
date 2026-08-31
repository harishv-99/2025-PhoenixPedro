package edu.ftcsushi.robots.phoenix;

/** Checked-in calibration acknowledgements for the current Phoenix robot. */
final class PhoenixCalibrationConfiguration {

    private PhoenixCalibrationConfiguration() {
    }

    /**
     * Returns a fresh draft containing Phoenix's current human-reviewed acknowledgements.
     *
     * <p>This is the edit location for those acknowledgements. Software cannot establish either
     * physical fact; update a value only after the corresponding robot calibration procedure.</p>
     *
     * @return fresh mutable current Phoenix calibration configuration
     */
    static PhoenixCalibrationConfig current() {
        PhoenixCalibrationConfig config = PhoenixCalibrationConfig.defaults();
        config.pinpointAxesVerified = true;
        config.pinpointPodOffsetsCalibrated = false;
        return config;
    }
}
