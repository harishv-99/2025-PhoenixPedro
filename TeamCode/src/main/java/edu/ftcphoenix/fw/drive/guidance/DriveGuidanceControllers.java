package edu.ftcphoenix.fw.drive.guidance;

import edu.ftcphoenix.fw.core.math.MathUtil;
import edu.ftcphoenix.fw.drive.DriveSignal;

/**
 * Small, unitless P-style controllers used by drive guidance overlays.
 *
 * <p>These controllers operate in “DriveSignal units” (typically [-1, +1]). They are not
 * kinematics-aware and intentionally avoid physical units (ips, rad/s) so they can be used
 * directly with Phoenix TeleOp driving.</p>
 */
final class DriveGuidanceControllers {

    private DriveGuidanceControllers() {
    }

    /**
     * Compute a translation command (axial/lateral) from a robot-frame translation error.
     */
    static DriveSignal translationCmd(double forwardErrorIn, double leftErrorIn, DriveGuidancePlan.Tuning tuning) {
        double ax = tuning.kPTranslate * forwardErrorIn;
        double lat = tuning.kPTranslate * leftErrorIn;

        // Clamp translation magnitude to maxTranslateCmd (preserve direction).
        double mag = Math.hypot(ax, lat);
        if (mag > tuning.maxTranslateCmd && mag > 1e-9) {
            double s = tuning.maxTranslateCmd / mag;
            ax *= s;
            lat *= s;
        }
        return new DriveSignal(ax, lat, 0.0);
    }

    /**
     * Compute an omega command from a bearing error (radians).
     */
    static double omegaCmd(double bearingErrorRad, DriveGuidancePlan.Tuning tuning) {
        if (Math.abs(bearingErrorRad) <= tuning.aimDeadbandRad) {
            return 0.0;
        }
        double om = tuning.kPAim * bearingErrorRad;
        return MathUtil.clamp(om, -tuning.maxOmegaCmd, +tuning.maxOmegaCmd);
    }
}
