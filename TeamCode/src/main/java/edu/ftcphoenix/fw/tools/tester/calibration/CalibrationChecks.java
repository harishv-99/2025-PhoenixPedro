package edu.ftcphoenix.fw.tools.tester.calibration;

import edu.ftcphoenix.fw.ftc.localization.PinpointOdometryPredictor;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;

/**
 * Shared heuristics for common tester-menu calibration status checks.
 *
 * <p>The goal is to keep robot-side menu wiring thin: robot code should mainly pass in the current
 * config objects and any explicit human-acknowledgement flags, while the framework owns the common
 * "does this look calibrated enough to continue?" logic.</p>
 */
public final class CalibrationChecks {

    private static final double ZERO_TOL = 1e-6;

    private CalibrationChecks() {
    }

    /**
     * Returns whether the supplied camera mount looks non-identity.
     *
     * <p>This is a heuristic: identity can be a valid real mount in theory, but in practice it is
     * usually the placeholder value teams leave in config before calibration.</p>
     */
    public static boolean cameraMountLooksSolved(CameraMountConfig mount) {
        if (mount == null) {
            return false;
        }

        return !isLikelyIdentity(mount);
    }

    /**
     * Status helper for camera-mount calibration.
     */
    public static CalibrationStatus cameraMount(CameraMountConfig mount) {
        if (cameraMountLooksSolved(mount)) {
            return CalibrationStatus.complete("camera mount looks calibrated");
        }
        return CalibrationStatus.incomplete("camera mount still looks like the identity placeholder");
    }

    /**
     * Status helper for an explicit "I verified the Pinpoint axes" acknowledgement.
     */
    public static CalibrationStatus pinpointAxes(boolean verified) {
        if (verified) {
            return CalibrationStatus.complete("Pinpoint axis directions were explicitly verified");
        }
        return CalibrationStatus.incomplete("run the axis-direction tester and then set the verification flag");
    }

    /**
     * Returns whether the Pinpoint offsets look non-default.
     */
    public static boolean pinpointOffsetsLookConfigured(PinpointOdometryPredictor.Config cfg) {
        if (cfg == null) {
            return false;
        }

        return Math.abs(cfg.forwardPodOffsetLeftInches) > ZERO_TOL
                || Math.abs(cfg.strafePodOffsetForwardInches) > ZERO_TOL;
    }

    /**
     * Returns whether the Pinpoint offsets are considered ready enough for the next calibration step.
     *
     * <p>An explicit robot-side acknowledgement wins. Otherwise a non-default offset pair counts as
     * "ready" so teams are not blocked if they copied the numbers in but forgot to flip a separate
     * boolean.</p>
     */
    public static boolean pinpointOffsetsReady(PinpointOdometryPredictor.Config cfg, boolean explicitlyVerified) {
        return explicitlyVerified || pinpointOffsetsLookConfigured(cfg);
    }

    /**
     * Status helper for Pinpoint pod offsets.
     */
    public static CalibrationStatus pinpointOffsets(PinpointOdometryPredictor.Config cfg,
                                                    boolean explicitlyVerified) {
        if (explicitlyVerified) {
            return CalibrationStatus.complete("Pinpoint pod offsets were explicitly confirmed");
        }

        if (pinpointOffsetsLookConfigured(cfg)) {
            return CalibrationStatus.complete("Pinpoint pod offsets look non-default (verify the flag when convenient)");
        }

        return CalibrationStatus.incomplete("Pinpoint pod offsets still look like the default 0 / 0 values");
    }

    /**
     * Whether it is reasonable to enable AprilTag-assisted calibration features.
     */
    public static boolean canUseAprilTagAssist(CameraMountConfig mount) {
        return cameraMountLooksSolved(mount);
    }

    private static boolean isLikelyIdentity(CameraMountConfig mount) {
        if (mount == null || mount.robotToCameraPose() == null) {
            return true;
        }

        return Math.abs(mount.robotToCameraPose().xInches) < ZERO_TOL
                && Math.abs(mount.robotToCameraPose().yInches) < ZERO_TOL
                && Math.abs(mount.robotToCameraPose().zInches) < ZERO_TOL
                && Math.abs(mount.robotToCameraPose().yawRad) < ZERO_TOL
                && Math.abs(mount.robotToCameraPose().pitchRad) < ZERO_TOL
                && Math.abs(mount.robotToCameraPose().rollRad) < ZERO_TOL;
    }
}
