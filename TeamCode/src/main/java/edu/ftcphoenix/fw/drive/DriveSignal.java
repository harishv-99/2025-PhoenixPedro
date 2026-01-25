package edu.ftcphoenix.fw.drive;

import edu.ftcphoenix.fw.util.MathUtil;

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
 * (not field-centric):</p>
 *
 * <ul>
 *   <li><b>axial &gt; 0</b>   → drive <b>forward</b> (out the front of the robot)</li>
 *   <li><b>axial &lt; 0</b>   → drive <b>backward</b></li>
 *   <li><b>lateral &gt; 0</b> → strafe <b>right</b> (robot's right-hand side)</li>
 *   <li><b>lateral &lt; 0</b> → strafe <b>left</b></li>
 *   <li><b>omega &gt; 0</b>   → rotate <b>clockwise</b> (turn right when viewed from above)</li>
 *   <li><b>omega &lt; 0</b>   → rotate <b>counter-clockwise</b> (turn left)</li>
 * </ul>
 *
 * <p>
 * These conventions are consistent with the standard mecanum wheel mixing used by
 * {@link MecanumDrivebase}, and with the default
 * {@link edu.ftcphoenix.fw.drive.source.GamepadDriveSource} mapping:
 * </p>
 *
 * <ul>
 *   <li>P1 left stick Y (up &gt; 0)   → {@code axial}</li>
 *   <li>P1 left stick X (right &gt; 0) → {@code lateral}</li>
 *   <li>P1 right stick X (right &gt; 0) → {@code omega}</li>
 * </ul>
 *
 * <h2>Typical usage</h2>
 *
 * <p>In most code you do not construct {@code DriveSignal} manually. Instead you use a
 * {@link DriveSource} (such as
 * {@link edu.ftcphoenix.fw.drive.source.GamepadDriveSource}) and feed its output into
 * a drivebase (such as {@link MecanumDrivebase}):</p>
 *
 * <pre>{@code
 * public final class PhoenixRobot {
 *     private final MecanumDrivebase drivebase;
 *     private final DriveSource driveSource;
 *
 *     public PhoenixRobot(HardwareMap hw, Gamepads pads) {
 *         // Build a mecanum drivebase with standard HW naming.
 *         this.drivebase = Drives.mecanum(hw);
 *
 *         // Use the standard TeleOp drive mapping:
 *         //   - left stick: axial (forward/back) + lateral (strafe)
 *         //   - right stick X: omega (turn)
 *         this.driveSource = GamepadDriveSource.teleOpMecanumStandard(pads);
 *     }
 *
 *     public void updateTeleOp(LoopClock clock) {
 *         double dt = clock.dtSec();
 *
 *         // Get a drive command from the current drive source.
 *         DriveSignal signal = driveSource.get(clock).clamped();
 *
 *         // Apply to the drivebase.
 *         drivebase.drive(signal);
 *         drivebase.update(clock);
 *     }
 * }
 * }</pre>
 *
 * <p>
 * Higher-level behaviors (e.g., TagAim, autonomous paths) are implemented as alternate
 * {@link DriveSource} implementations that still produce
 * {@code DriveSignal} using these same sign conventions.
 * </p>
 */
public final class DriveSignal {
    /**
     * Axial (forward/backward) component of the command.
     */
    public final double axial;

    /**
     * Lateral (left/right strafe) component of the command.
     */
    public final double lateral;

    /**
     * Rotational component of the command (about vertical axis).
     */
    public final double omega;

    /**
     * A constant zero signal (all components = 0).
     *
     * <p>Useful as a default or "stop" command.</p>
     */
    public static final DriveSignal ZERO = new DriveSignal(0.0, 0.0, 0.0);

    /**
     * Construct a new drive signal.
     *
     * @param axial   forward/backward command
     * @param lateral left/right command
     * @param omega   rotational command
     */
    public DriveSignal(double axial, double lateral, double omega) {
        this.axial = axial;
        this.lateral = lateral;
        this.omega = omega;
    }

    /**
     * Return a new signal with each component multiplied by the given scale.
     *
     * <p>This is commonly used for "slow mode" or fine control, where you
     * want to uniformly shrink all components of the command.</p>
     */
    public DriveSignal scaled(double scale) {
        return new DriveSignal(
                axial * scale,
                lateral * scale,
                omega * scale
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

    @Override
    public String toString() {
        return "DriveSignal{" +
                "axial=" + axial +
                ", lateral=" + lateral +
                ", omega=" + omega +
                '}';
    }
}
