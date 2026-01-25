package edu.ftcphoenix.fw.drive;

import edu.ftcphoenix.fw.core.math.MathUtil;

/**
 * Robot-centric chassis velocity command.
 *
 * <p>This is the "real units" companion to {@link DriveSignal}:
 * {@code DriveSignal} is a normalized, unitless command typically in [-1, +1],
 * while {@code ChassisSpeeds} expresses intent in physical units.</p>
 *
 * <h2>Frame conventions</h2>
 * <ul>
 *   <li>All components are <b>robot-centric</b>.</li>
 *   <li>{@code vxRobotIps > 0} drives forward (+X).</li>
 *   <li>{@code vyRobotIps > 0} strafes left (+Y).</li>
 *   <li>{@code omegaRobotRadPerSec > 0} rotates CCW (turn left, viewed from above).</li>
 * </ul>
 */
public final class ChassisSpeeds {

    /**
     * Forward velocity in the robot frame, in inches/sec (+ forward).
     */
    public final double vxRobotIps;

    /**
     * Leftward velocity in the robot frame, in inches/sec (+ left).
     */
    public final double vyRobotIps;

    /**
     * Angular velocity about +Z, in rad/sec (+ CCW).
     */
    public final double omegaRobotRadPerSec;

    private static final ChassisSpeeds ZERO = new ChassisSpeeds(0.0, 0.0, 0.0);

    /**
     * @return a zero-velocity command.
     */
    public static ChassisSpeeds zero() {
        return ZERO;
    }

    /**
     * Construct a robot-centric chassis velocity command.
     *
     * @param vxRobotIps          forward velocity in the robot frame (inches/sec; + forward)
     * @param vyRobotIps          leftward velocity in the robot frame (inches/sec; + left)
     * @param omegaRobotRadPerSec angular velocity about +Z (rad/sec; + CCW)
     */
    public ChassisSpeeds(double vxRobotIps, double vyRobotIps, double omegaRobotRadPerSec) {
        this.vxRobotIps = vxRobotIps;
        this.vyRobotIps = vyRobotIps;
        this.omegaRobotRadPerSec = omegaRobotRadPerSec;
    }

    /**
     * Return a new command with translation and rotation scaled independently.
     *
     * @param translationScale scale applied to {@link #vxRobotIps} and {@link #vyRobotIps}
     * @param omegaScale       scale applied to {@link #omegaRobotRadPerSec}
     * @return a new scaled chassis velocity command
     */
    public ChassisSpeeds scaled(double translationScale, double omegaScale) {
        return new ChassisSpeeds(
                vxRobotIps * translationScale,
                vyRobotIps * translationScale,
                omegaRobotRadPerSec * omegaScale
        );
    }

    /**
     * Clamp each component independently to the given absolute maxima.
     *
     * @param maxVxAbsIps          maximum absolute value allowed for {@link #vxRobotIps} (inches/sec)
     * @param maxVyAbsIps          maximum absolute value allowed for {@link #vyRobotIps} (inches/sec)
     * @param maxOmegaAbsRadPerSec maximum absolute value allowed for {@link #omegaRobotRadPerSec} (rad/sec)
     * @return a new clamped chassis velocity command
     */
    public ChassisSpeeds clamped(double maxVxAbsIps, double maxVyAbsIps, double maxOmegaAbsRadPerSec) {
        return new ChassisSpeeds(
                MathUtil.clamp(vxRobotIps, -Math.abs(maxVxAbsIps), Math.abs(maxVxAbsIps)),
                MathUtil.clamp(vyRobotIps, -Math.abs(maxVyAbsIps), Math.abs(maxVyAbsIps)),
                MathUtil.clamp(omegaRobotRadPerSec, -Math.abs(maxOmegaAbsRadPerSec), Math.abs(maxOmegaAbsRadPerSec))
        );
    }


    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        return "ChassisSpeeds{" +
                "vxRobotIps=" + vxRobotIps +
                ", vyRobotIps=" + vyRobotIps +
                ", omegaRobotRadPerSec=" + omegaRobotRadPerSec +
                '}';
    }

}
