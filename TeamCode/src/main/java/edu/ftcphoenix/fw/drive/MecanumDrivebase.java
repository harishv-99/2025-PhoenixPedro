package edu.ftcphoenix.fw.drive;

import edu.ftcphoenix.fw.debug.DebugSink;
import edu.ftcphoenix.fw.hal.PowerOutput;
import edu.ftcphoenix.fw.util.LoopClock;
import edu.ftcphoenix.fw.util.MathUtil;

/**
 * Simple open-loop mecanum mixer.
 *
 * <p>Maps a high-level {@link DriveSignal} to four wheel power commands.</p>
 *
 * <h2>Sign conventions</h2>
 *
 * <p>Assumes an X-configured mecanum drivetrain (rollers pointing inwards when
 * viewed from above), with all inversion handled at the hardware level (e.g. via
 * FTC SDK {@code setDirection(REVERSE)} when constructing {@link PowerOutput}s).
 * In that configuration, the {@link DriveSignal} components have the following
 * robot-centric meaning:</p>
 *
 * <ul>
 *   <li><b>axial &gt; 0</b>   &rarr; drive forward</li>
 *   <li><b>axial &lt; 0</b>   &rarr; drive backward</li>
 *   <li><b>lateral &gt; 0</b> &rarr; strafe right</li>
 *   <li><b>lateral &lt; 0</b> &rarr; strafe left</li>
 *   <li><b>omega &gt; 0</b>   &rarr; rotate clockwise (turn right, viewed from above)</li>
 *   <li><b>omega &lt; 0</b>   &rarr; rotate counter-clockwise (turn left)</li>
 * </ul>
 *
 * <p>The internal mixer uses the standard mecanum equations:</p>
 *
 * <pre>
 * fl = axial + lateral + omega
 * fr = axial - lateral - omega
 * bl = axial - lateral + omega
 * br = axial + lateral - omega
 * </pre>
 *
 * <h2>Normalization</h2>
 *
 * <p>
 * After mixing, the wheel powers are <b>normalized</b> if any magnitude exceeds 1.0,
 * by dividing all four by {@code max(1, |fl|, |fr|, |bl|, |br|)}. This preserves the
 * intended direction and ratios even at full-stick inputs, instead of independently
 * clamping each wheel. A final clamp to [-1, +1] is applied for numerical safety.
 * </p>
 *
 * <p>
 * The {@link MecanumConfig} allows optional scaling of the three drive components
 * (axial, lateral, omega) <em>and</em> optional per-axis rate limiting of how
 * quickly those commands may change over time.
 * </p>
 *
 * <h2>Typical usage</h2>
 *
 * <pre>{@code
 * public final class PhoenixRobot {
 *     private final MecanumDrivebase drivebase;
 *     private final DriveSource driveSource;
 *
 *     public PhoenixRobot(HardwareMap hw, Gamepads pads) {
 *         this.drivebase = Drives.mecanum(hw);
 *         this.driveSource = GamepadDriveSource.teleOpMecanumStandard(pads);
 *     }
 *
 *     public void updateTeleOp(LoopClock clock) {
 *         // Get a drive command from the current drive source.
 *         DriveSignal signal = driveSource.get(clock).clamped();
 *
 *         // Apply to the drivebase.
 *         drivebase.drive(signal);
 *         drivebase.update(clock);
 *     }
 * }
 * }</pre>
 */
public final class MecanumDrivebase {

    private final PowerOutput fl;
    private final PowerOutput fr;
    private final PowerOutput bl;
    private final PowerOutput br;

    private final MecanumConfig cfg;

    // Last commanded drive components (after scaling and rate limiting).
    private double lastAxialCmd;
    private double lastLateralCmd;
    private double lastOmegaCmd;

    // Last commanded wheel powers (after normalization/clamping).
    private double lastFlPower;
    private double lastFrPower;
    private double lastBlPower;
    private double lastBrPower;

    // Last dt (seconds) used for rate limiting.
    private double lastDtSec;

    /**
     * Construct a new mecanum drivebase.
     *
     * @param flPower power output for the front-left wheel (non-null)
     * @param frPower power output for the front-right wheel (non-null)
     * @param blPower power output for the back-left wheel (non-null)
     * @param brPower power output for the back-right wheel (non-null)
     * @param cfg     configuration for scaling and rate limiting (non-null)
     */
    public MecanumDrivebase(PowerOutput flPower,
                            PowerOutput frPower,
                            PowerOutput blPower,
                            PowerOutput brPower,
                            MecanumConfig cfg) {
        this.fl = flPower;
        this.fr = frPower;
        this.bl = blPower;
        this.br = brPower;
        this.cfg = cfg != null ? cfg : MecanumConfig.defaults();
    }

    /**
     * Command the drivebase using a {@link DriveSignal}.
     *
     * <p>
     * This method:
     * </p>
     * <ol>
     *   <li>Scales {@code axial}, {@code lateral}, {@code omega} by config limits.</li>
     *   <li>Optionally rate-limits each component based on {@link #update(LoopClock)}.</li>
     *   <li>Computes mecanum wheel powers using the standard mix.</li>
     *   <li>Normalizes wheel powers if any magnitude exceeds 1.0.</li>
     *   <li>Clamps wheel powers to [-1, +1] and sends them to the hardware.</li>
     * </ol>
     *
     * @param s drive command; if {@code null}, this is treated as a stop command
     */
    public void drive(DriveSignal s) {
        if (s == null) {
            // Treat null as a stop command for robustness.
            stop();
            return;
        }

        // 1) Apply per-axis scaling from the config.
        double desiredAxial = s.axial * cfg.maxAxial;
        double desiredLateral = s.lateral * cfg.maxLateral;
        double desiredOmega = s.omega * cfg.maxOmega;

        // 2) Optionally apply per-axis rate limiting based on lastDtSec.
        double dt = lastDtSec;
        double axialCmd = limitRate(desiredAxial, lastAxialCmd, cfg.maxAxialRatePerSec, dt);
        double lateralCmd = limitRate(desiredLateral, lastLateralCmd, cfg.maxLateralRatePerSec, dt);
        double omegaCmd = limitRate(desiredOmega, lastOmegaCmd, cfg.maxOmegaRatePerSec, dt);

        lastAxialCmd = axialCmd;
        lastLateralCmd = lateralCmd;
        lastOmegaCmd = omegaCmd;

        // 3) Basic mecanum mixing with the (possibly rate-limited) components.
        //    Sign conventions (robot-centric):
        //      axial   > 0 -> forward
        //      lateral > 0 -> right
        //      omega   > 0 -> clockwise (turn right, viewed from above)
        double flP = axialCmd + lateralCmd + omegaCmd;
        double frP = axialCmd - lateralCmd - omegaCmd;
        double blP = axialCmd - lateralCmd + omegaCmd;
        double brP = axialCmd + lateralCmd - omegaCmd;

        // 4) Normalize wheel powers if any exceeds |1.0| to preserve direction.
        double maxMag = Math.max(
                1.0,
                Math.max(
                        Math.max(Math.abs(flP), Math.abs(frP)),
                        Math.max(Math.abs(blP), Math.abs(brP))
                )
        );
        flP /= maxMag;
        frP /= maxMag;
        blP /= maxMag;
        brP /= maxMag;

        // 5) Final clamp (mainly for numerical safety) and apply,
        //    while tracking last commanded values.
        lastFlPower = MathUtil.clamp(flP, -1.0, 1.0);
        lastFrPower = MathUtil.clamp(frP, -1.0, 1.0);
        lastBlPower = MathUtil.clamp(blP, -1.0, 1.0);
        lastBrPower = MathUtil.clamp(brP, -1.0, 1.0);

        fl.setPower(lastFlPower);
        fr.setPower(lastFrPower);
        bl.setPower(lastBlPower);
        br.setPower(lastBrPower);
    }

    /**
     * Internal helper to limit the rate of change of a command.
     *
     * <p>
     * If {@code maxRatePerSec} is &lt;= 0 or {@code dtSec} is &lt;= 0, this
     * method returns {@code desired} unchanged.
     * </p>
     */
    private static double limitRate(double desired,
                                    double previous,
                                    double maxRatePerSec,
                                    double dtSec) {
        if (maxRatePerSec <= 0.0 || dtSec <= 0.0) {
            return desired;
        }
        double maxDelta = maxRatePerSec * dtSec;
        double delta = desired - previous;

        if (delta > maxDelta) {
            return previous + maxDelta;
        } else if (delta < -maxDelta) {
            return previous - maxDelta;
        }
        return desired;
    }

    /**
     * Update loop timing information used for rate limiting.
     *
     * <p>
     * Call this once per loop before calling {@link #drive(DriveSignal)} if
     * you want rate limiting to be based on the most recent dt.
     * </p>
     *
     * @param clock loop timing helper (may be {@code null})
     */
    public void update(LoopClock clock) {
        if (clock == null) {
            return;
        }
        lastDtSec = clock.dtSec();
    }

    /**
     * Immediately stop all four drive outputs and reset last command
     * bookkeeping.
     */
    public void stop() {
        lastAxialCmd = 0.0;
        lastLateralCmd = 0.0;
        lastOmegaCmd = 0.0;

        lastFlPower = 0.0;
        lastFrPower = 0.0;
        lastBlPower = 0.0;
        lastBrPower = 0.0;

        fl.setPower(0.0);
        fr.setPower(0.0);
        bl.setPower(0.0);
        br.setPower(0.0);
    }

    // ------------------------------------------------------------------------
    // Debug / inspection helpers
    // ------------------------------------------------------------------------

    /**
     * Dump internal state to a {@link DebugSink}.
     *
     * <p>
     * This is intended for one-off debugging and tuning. Callers can choose
     * any prefix they like; nested callers often use dotted paths such as
     * {@code "drive.mecanum"}.
     * </p>
     *
     * <p>
     * This method is defensive: if {@code dbg} is {@code null}, it does
     * nothing. Framework classes consistently follow this pattern so callers
     * may freely pass {@code null} when they do not care about debug output.
     * </p>
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix != null && !prefix.isEmpty()) ? prefix + "." : "";

        dbg.addData(p + "lastFlPower", lastFlPower);
        dbg.addData(p + "lastFrPower", lastFrPower);
        dbg.addData(p + "lastBlPower", lastBlPower);
        dbg.addData(p + "lastBrPower", lastBrPower);

        dbg.addData(p + "lastAxialCmd", lastAxialCmd);
        dbg.addData(p + "lastLateralCmd", lastLateralCmd);
        dbg.addData(p + "lastOmegaCmd", lastOmegaCmd);

        dbg.addData(p + "lastDtSec", lastDtSec);
    }

    // ------------------------------------------------------------------------
    // Accessors
    // ------------------------------------------------------------------------

    /**
     * @return last commanded (scaled and rate-limited) axial command.
     */
    public double getLastAxialCmd() {
        return lastAxialCmd;
    }

    /**
     * @return last commanded (scaled and rate-limited) lateral command.
     */
    public double getLastLateralCmd() {
        return lastLateralCmd;
    }

    /**
     * @return last commanded (scaled and rate-limited) omega command.
     */
    public double getLastOmegaCmd() {
        return lastOmegaCmd;
    }

    /**
     * @return last commanded (normalized &amp; clamped) power for front-left wheel.
     */
    public double getLastFlPower() {
        return lastFlPower;
    }

    /**
     * @return last commanded (normalized &amp; clamped) power for front-right wheel.
     */
    public double getLastFrPower() {
        return lastFrPower;
    }

    /**
     * @return last commanded (normalized &amp; clamped) power for back-left wheel.
     */
    public double getLastBlPower() {
        return lastBlPower;
    }

    /**
     * @return last commanded (normalized &amp; clamped) power for back-right wheel.
     */
    public double getLastBrPower() {
        return lastBrPower;
    }
}
