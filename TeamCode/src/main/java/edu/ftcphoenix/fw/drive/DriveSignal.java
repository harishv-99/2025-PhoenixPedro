package edu.ftcphoenix.fw.drive;

import edu.ftcphoenix.fw.core.math.MathUtil;

/**
 * Immutable robot-centric drive command used throughout the Phoenix framework.
 *
 * <p>A {@code DriveSignal} encodes three degrees of freedom:</p>
 *
 * <ul>
 *   <li><b>axial</b>   – forward / backward</li>
 *   <li><b>lateral</b> – left / right (strafing)</li>
 *   <li><b>omega</b>   – rotation about the vertical axis (turning)</li>
 * </ul>
 *
 * <h2>Sign conventions</h2>
 *
 * <p>All components are <b>robot-centric</b>, defined from the robot's point of view
 * (not field-centric), and aligned with Phoenix pose conventions
 * ({@code Pose2d}/{@code Pose3d}: +X forward, +Y left, yaw CCW-positive):</p>
 *
 * <ul>
 *   <li><b>axial &gt; 0</b>   → drive <b>forward</b> (out the front of the robot)</li>
 *   <li><b>axial &lt; 0</b>   → drive <b>backward</b></li>
 *   <li><b>lateral &gt; 0</b> → strafe <b>left</b> (robot's left-hand side)</li>
 *   <li><b>lateral &lt; 0</b> → strafe <b>right</b></li>
 *   <li><b>omega &gt; 0</b>   → rotate <b>counter-clockwise</b> (turn left when viewed from above)</li>
 *   <li><b>omega &lt; 0</b>   → rotate <b>clockwise</b> (turn right)</li>
 * </ul>
 *
 * <p>
 * These conventions are implemented by {@link edu.ftcphoenix.fw.drive.MecanumDrivebase}.
 * Driver-facing inputs (gamepad sticks) are converted at the boundary (see
 * {@link edu.ftcphoenix.fw.drive.source.GamepadDriveSource}) so that:
 * </p>
 *
 * <ul>
 *   <li>pushing the left stick right still makes the robot strafe right (which becomes {@code lateral &lt; 0})</li>
 *   <li>pushing the right stick right still makes the robot turn right (which becomes {@code omega &lt; 0})</li>
 * </ul>
 *
 * <h2>Typical usage</h2>
 *
 * <p>In most code you do not construct {@code DriveSignal} manually. Instead you use a
 * {@link edu.ftcphoenix.fw.drive.DriveSource} (such as
 * {@link edu.ftcphoenix.fw.drive.source.GamepadDriveSource}) and feed its output into
 * a drivebase (such as {@link edu.ftcphoenix.fw.drive.MecanumDrivebase}):</p>
 *
 * <pre>{@code
 * public final class PhoenixRobot {
 *     private final MecanumDrivebase drivebase;
 *     private final DriveSource driveSource;
 *
 *     public PhoenixRobot(HardwareMap hw, Gamepads pads) {
 *         this.drivebase = FtcDrives.mecanum(hw);
 *         // Choose one of the GamepadDriveSource factories.
 *         this.driveSource = GamepadDriveSource.teleOpMecanum(pads);
 *     }
 *
 *     public void updateTeleOp(LoopClock clock) {
 *         DriveSignal signal = driveSource.get(clock).clamped();
 *         drivebase.update(clock);
 *         drivebase.drive(signal);
 *     }
 * }
 * }</pre>
 *
 * <p>
 * Higher-level behaviors (tag aiming, go-to-pose, etc.) are implemented as alternate
 * {@link edu.ftcphoenix.fw.drive.DriveSource} implementations that still produce
 * {@code DriveSignal} using these same sign conventions.
 * </p>
 */
public final class DriveSignal {
    /**
     * Axial (forward/backward) component of the command.
     *
     * <p>{@code axial > 0} drives forward.</p>
     */
    public final double axial;

    /**
     * Lateral (left/right strafe) component of the command.
     *
     * <p>{@code lateral > 0} strafes left.</p>
     */
    public final double lateral;

    /**
     * Rotational component of the command (about vertical axis).
     *
     * <p>{@code omega > 0} rotates counter-clockwise (turns left).</p>
     */
    public final double omega;

    /**
     * A constant zero signal (all components = 0).
     *
     * <p>
     * Prefer {@link #zero()} for call-site readability and consistency with other
     * "factory" helpers.
     * </p>
     */
    private static final DriveSignal ZERO = new DriveSignal(0.0, 0.0, 0.0);

    /**
     * Return a zero signal (all components = 0).
     *
     * <p>
     * Prefer this over referencing {@link #ZERO} directly so call sites read
     * consistently with other small factories like {@code noop()}.
     * </p>
     */
    public static DriveSignal zero() {
        return ZERO;
    }

    /**
     * Construct a new drive signal.
     *
     * @param axial   forward/backward command (robot-centric)
     * @param lateral left/right strafe command (robot-centric; + is left)
     * @param omega   rotational command (robot-centric; + is CCW)
     */
    public DriveSignal(double axial, double lateral, double omega) {
        this.axial = axial;
        this.lateral = lateral;
        this.omega = omega;
    }

    /**
     * Return a new signal with translation and rotation scaled independently.
     *
     * <p>This is most commonly used for TeleOp "slow mode" (fine control), where
     * you often want translation and rotation to slow down by different amounts.
     * For example, you might set translation to 0.35x while setting rotation to 0.20x
     * so turning stays extra gentle while lining up.
     * </p>
     *
     * <p>Scaling is applied as:</p>
     * <ul>
     *   <li>{@code axial'   = axial   * translationScale}</li>
     *   <li>{@code lateral' = lateral * translationScale}</li>
     *   <li>{@code omega'   = omega   * omegaScale}</li>
     * </ul>
     *
     * <p>Typical scales are in (0, 1], but no clamping is performed. If you want to
     * enforce a specific range, use {@link #clamped()} after scaling.</p>
     *
     * @param translationScale scale applied to {@link #axial} and {@link #lateral}
     * @param omegaScale       scale applied to {@link #omega}
     * @return a new scaled drive signal
     */
    public DriveSignal scaled(double translationScale, double omegaScale) {
        return new DriveSignal(
                axial * translationScale,
                lateral * translationScale,
                omega * omegaScale
        );
    }

    /**
     * Return a new signal with each component clamped to the given range.
     *
     * @param min minimum value (inclusive)
     * @param max maximum value (inclusive)
     */
    public DriveSignal clamped(double min, double max) {
        return new DriveSignal(
                MathUtil.clamp(axial, min, max),
                MathUtil.clamp(lateral, min, max),
                MathUtil.clamp(omega, min, max)
        );
    }

    /**
     * Return a new signal with each component clamped to [-1, +1].
     *
     * <p>This is useful before sending commands to a drivebase that expects
     * normalized commands.</p>
     */
    public DriveSignal clamped() {
        return clamped(-1.0, +1.0);
    }

    /**
     * Add another drive signal to this one, component-wise.
     *
     * <p>This can be used to blend small "assist" commands into a base
     * command. Be careful not to exceed your intended range before
     * clamping.</p>
     *
     * @param other signal to add; if null, this signal is returned
     */
    public DriveSignal plus(DriveSignal other) {
        if (other == null) {
            return this;
        }
        return new DriveSignal(
                this.axial + other.axial,
                this.lateral + other.lateral,
                this.omega + other.omega
        );
    }

    /**
     * Linearly interpolate between this signal and {@code other}.
     *
     * <p>{@code alpha} is clamped to [0, 1]:
     * <ul>
     *   <li>{@code alpha = 0} → returns this signal.</li>
     *   <li>{@code alpha = 1} → returns {@code other}.</li>
     *   <li>Values in between produce a blended command.</li>
     * </ul>
     * This is useful for "driver assist" behaviors where you want to blend
     * manual control with some automatic behavior.</p>
     *
     * @param other the target signal to blend toward; if null, this signal is returned
     * @param alpha blend factor in [0, 1] (values outside this range are clamped)
     * @return blended signal
     */
    public DriveSignal lerp(DriveSignal other, double alpha) {
        if (other == null) {
            return this;
        }
        double t = MathUtil.clamp(alpha, 0.0, 1.0);
        return new DriveSignal(
                this.axial + (other.axial - this.axial) * t,
                this.lateral + (other.lateral - this.lateral) * t,
                this.omega + (other.omega - this.omega) * t
        );
    }

    /**
     * Return a new drive signal with the axial component replaced.
     */
    public DriveSignal withAxial(double newAxial) {
        return new DriveSignal(newAxial, lateral, omega);
    }

    /**
     * Return a new drive signal with the lateral component replaced.
     */
    public DriveSignal withLateral(double newLateral) {
        return new DriveSignal(axial, newLateral, omega);
    }

    /**
     * Return a new drive signal with the omega component replaced.
     */
    public DriveSignal withOmega(double newOmega) {
        return new DriveSignal(axial, lateral, newOmega);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        return "DriveSignal{"
                + "axial=" + axial
                + ", lateral=" + lateral
                + ", omega=" + omega
                + '}';
    }
}
