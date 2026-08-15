package edu.ftcphoenix.fw.drive.guidance;

import edu.ftcphoenix.fw.core.math.MathUtil;
import edu.ftcphoenix.fw.drive.DriveSignal;

/**
 * Small P-style controllers used by drive guidance overlays.
 *
 * <p>Translation gain maps error in inches to normalized translation command, while aim gain maps
 * error in radians to normalized omega command. The controllers are not kinematics-aware and do
 * not command physical velocity units such as inches per second or radians per second.</p>
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
        if (mag > tuning.maxTranslateCmd) {
            if (!Double.isFinite(mag)
                    && Double.isFinite(forwardErrorIn)
                    && Double.isFinite(leftErrorIn)
                    && tuning.kPTranslate > 0.0) {
                return boundedTranslationFromFiniteError(
                        forwardErrorIn,
                        leftErrorIn,
                        tuning.maxTranslateCmd
                );
            }
            double s = tuning.maxTranslateCmd / mag;
            ax *= s;
            lat *= s;
        }
        return new DriveSignal(ax, lat, 0.0);
    }

    /**
     * Recovers a bounded command direction when finite multiplication or vector magnitude
     * overflows. Scaling by the largest error component keeps the normalization finite without
     * imposing an arbitrary upper bound on a valid gain or error.
     */
    private static DriveSignal boundedTranslationFromFiniteError(double forwardErrorIn,
                                                                 double leftErrorIn,
                                                                 double maxTranslateCmd) {
        double largestError = Math.max(Math.abs(forwardErrorIn), Math.abs(leftErrorIn));
        if (largestError == 0.0) {
            return DriveSignal.zero();
        }

        double scaledForward = forwardErrorIn / largestError;
        double scaledLeft = leftErrorIn / largestError;
        double scaledMagnitude = Math.hypot(scaledForward, scaledLeft);
        double commandScale = maxTranslateCmd / scaledMagnitude;
        return new DriveSignal(
                scaledForward * commandScale,
                scaledLeft * commandScale,
                0.0
        );
    }

    /**
     * Compute an omega command from a bearing error (radians).
     */
    static double omegaCmd(double bearingErrorRad, DriveGuidancePlan.Tuning tuning) {
        if (Math.abs(bearingErrorRad) <= tuning.aimDeadbandRad) {
            return 0.0;
        }
        double om = tuning.kPAim * bearingErrorRad;

        // Stiction assist: if we want to turn (outside deadband) but the computed command is
        // extremely small, bump it to a minimum magnitude so the drivetrain actually moves.
        // This is especially important for "tight" aim deadbands where kP*error can fall
        // below motor/controller deadbands.
        double min = tuning.minOmegaCmd;
        if (min > 0.0 && Double.isFinite(min) && Math.abs(om) < min) {
            // A valid positive gain can still underflow to signed zero. The bearing error is the
            // already-known correction direction whenever execution reaches this outside-deadband
            // branch, so retain that sign rather than deriving it from the underflowed product.
            om = Math.copySign(min, bearingErrorRad);
        }

        return MathUtil.clamp(om, -tuning.maxOmegaCmd, +tuning.maxOmegaCmd);
    }
}
