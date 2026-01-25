package edu.ftcphoenix.fw.drive;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.math.MathUtil;
import edu.ftcphoenix.fw.core.time.LoopClock;

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
 * robot-centric meaning, aligned with Phoenix pose conventions
 * ({@code Pose2d}/{@code Pose3d}: +X forward, +Y left, yaw CCW-positive):</p>
 *
 * <ul>
 *   <li><b>axial &gt; 0</b>   &rarr; drive forward</li>
 *   <li><b>axial &lt; 0</b>   &rarr; drive backward</li>
 *   <li><b>lateral &gt; 0</b> &rarr; strafe left</li>
 *   <li><b>lateral &lt; 0</b> &rarr; strafe right</li>
 *   <li><b>omega &gt; 0</b>   &rarr; rotate counter-clockwise (turn left, viewed from above)</li>
 *   <li><b>omega &lt; 0</b>   &rarr; rotate clockwise (turn right)</li>
 * </ul>
 *
 * <p>The internal mixer uses the standard mecanum equations:</p>
 *
 * <pre>
 * fl = axial - lateral - omega
 * fr = axial + lateral + omega
 * bl = axial + lateral - omega
 * br = axial - lateral + omega
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
 * <h2>Typical usage</h2>
 *
 * <pre>{@code
 * MecanumDrivebase.Config cfg = MecanumDrivebase.Config.defaults();
 * cfg.maxOmega = 0.8;                 // example: slightly slower rotation
 * cfg.maxLateralRatePerSec = 4.0;     // example: smoother strafing
 *
 * MecanumDrivebase drive = new MecanumDrivebase(fl, fr, bl, br, cfg);
 *
 * // In the main loop:
 * drive.update(clock);        // update dt used for rate limiting
 * drive.drive(signal);        // applies motor power immediately
 * }</pre>
 */
public final class MecanumDrivebase {

    /**
     * Configuration for {@link MecanumDrivebase}.
     *
     * <p>This is a simple <strong>mutable data object</strong> following the
     * Phoenix convention:</p>
     *
     * <ul>
     *   <li>Start from {@link #defaults()}.</li>
     *   <li>Override the fields you care about during robot initialization.</li>
     *   <li>Pass the config into {@link MecanumDrivebase} at construction time.</li>
     * </ul>
     *
     * <p><b>Important:</b> {@link MecanumDrivebase} makes a defensive copy of the
     * config at construction time. Mutating a {@code Config} instance after passing
     * it into a drivebase will not affect that already-created drivebase.</p>
     */
    public static final class Config {

        // --------------------------------------------------------------------
        // Per-axis scaling (high-level DriveSignal components)
        // --------------------------------------------------------------------

        /**
         * Scale applied to {@link DriveSignal#axial} before mixing.
         *
         * <p>Default: {@code 1.0} (no scaling).</p>
         */
        public double maxAxial = 1.0;

        /**
         * Scale applied to {@link DriveSignal#lateral} before mixing.
         *
         * <p>Default: {@code 1.0} (no scaling).</p>
         */
        public double maxLateral = 1.0;

        /**
         * Scale applied to {@link DriveSignal#omega} before mixing.
         *
         * <p>Default: {@code 1.0} (no scaling).</p>
         */
        public double maxOmega = 1.0;

        // --------------------------------------------------------------------
        // Physical-speed mapping (used by drive(ChassisSpeeds))
        // --------------------------------------------------------------------

        /**
         * Approximate maximum forward speed of the robot at full command, in inches/sec.
         *
         * <p>Used only when converting a {@link ChassisSpeeds} command into a normalized
         * {@link DriveSignal}. This is a <b>best-effort mapping</b>, not closed-loop velocity
         * control.</p>
         */
        public double maxVxInchesPerSec = 40.0;

        /**
         * Approximate maximum leftward strafe speed of the robot at full command, in inches/sec.
         *
         * <p>Used only when converting a {@link ChassisSpeeds} command into a normalized
         * {@link DriveSignal}.</p>
         */
        public double maxVyInchesPerSec = 40.0;

        /**
         * Approximate maximum angular speed at full command, in rad/sec.
         *
         * <p>Used only when converting a {@link ChassisSpeeds} command into a normalized
         * {@link DriveSignal}.</p>
         */
        public double maxOmegaRadPerSec = Math.toRadians(180.0);

        // --------------------------------------------------------------------
        // Optional per-axis rate limiting (advanced)
        // --------------------------------------------------------------------

        /**
         * Maximum change in axial command per second (command units/sec).
         *
         * <p>Domain: any value &gt; 0 enables a limit; {@code <= 0} disables it.</p>
         *
         * <p>Default: {@code 0.0} (no limit).</p>
         */
        public double maxAxialRatePerSec = 0.0;

        /**
         * Maximum change in lateral command per second (command units/sec).
         *
         * <p>Domain: any value &gt; 0 enables a limit; {@code <= 0} disables it.</p>
         *
         * <p>
         * Default: {@code 4.0}. A typical Phoenix tuning might set this to a small
         * value like {@code 4.0} to smooth strafing.
         * </p>
         */
        public double maxLateralRatePerSec = 4.0;

        /**
         * Maximum change in omega command per second (command units/sec).
         *
         * <p>Domain: any value &gt; 0 enables a limit; {@code <= 0} disables it.</p>
         *
         * <p>Default: {@code 0.0} (no limit).</p>
         */
        public double maxOmegaRatePerSec = 0.0;

        private Config() {
            // Defaults assigned in field initializers.
        }

        /**
         * Create a new config instance with Phoenix defaults.
         */
        public static Config defaults() {
            return new Config();
        }

        /**
         * Create a deep copy of this config.
         */
        public Config copy() {
            Config c = new Config();
            c.maxAxial = this.maxAxial;
            c.maxLateral = this.maxLateral;
            c.maxOmega = this.maxOmega;

            c.maxVxInchesPerSec = this.maxVxInchesPerSec;
            c.maxVyInchesPerSec = this.maxVyInchesPerSec;
            c.maxOmegaRadPerSec = this.maxOmegaRadPerSec;

            c.maxAxialRatePerSec = this.maxAxialRatePerSec;
            c.maxLateralRatePerSec = this.maxLateralRatePerSec;
            c.maxOmegaRatePerSec = this.maxOmegaRatePerSec;
            return c;
        }
    }

    private final PowerOutput fl;
    private final PowerOutput fr;
    private final PowerOutput bl;
    private final PowerOutput br;

    private final Config cfg;

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
     * @param cfg     configuration for scaling and rate limiting (may be {@code null})
     */
    public MecanumDrivebase(PowerOutput flPower,
                            PowerOutput frPower,
                            PowerOutput blPower,
                            PowerOutput brPower,
                            Config cfg) {
        this.fl = flPower;
        this.fr = frPower;
        this.bl = blPower;
        this.br = brPower;

        // Defensive copy so callers can't change behavior by mutating cfg later.
        this.cfg = (cfg != null ? cfg.copy() : Config.defaults());
    }


    /**
     * Command the drivebase using a {@link ChassisSpeeds} velocity intent.
     *
     * <p>This is a <b>best-effort</b> mapping from physical units to a normalized
     * {@link DriveSignal}. It does <em>not</em> perform closed-loop velocity control.
     * Battery voltage, carpet, friction, and load will change the actual achieved speeds.</p>
     *
     * <p>All components are robot-centric, aligned with Phoenix pose conventions
     * (+X forward, +Y left, yaw CCW-positive).</p>
     *
     * <p>Saturation policy: if any component would exceed its configured maximum, all
     * components are scaled by the same factor so the command preserves its direction
     * in (vx, vy, omega) space.</p>
     *
     * <p><b>Actuation timing:</b> this method ultimately calls {@link #drive(DriveSignal)},
     * which <b>immediately</b> sends wheel power commands to the hardware outputs.</p>
     *
     * <p><b>Rate limiting:</b> if you enable rate limiting in {@link Config}, call
     * {@link #update(LoopClock)} once per loop <b>before</b> calling this method so the
     * current loop's {@code dtSec} is used.</p>
     *
     * @param speeds desired chassis speeds (robot-centric) (must not be {@code null})
     * @throws IllegalStateException if any of the speed-mapping max values are <= 0
     */
    public void drive(ChassisSpeeds speeds) {
        Objects.requireNonNull(speeds, "speeds");

        double maxVx = cfg.maxVxInchesPerSec;
        double maxVy = cfg.maxVyInchesPerSec;
        double maxOmega = cfg.maxOmegaRadPerSec;

        if (maxVx <= 0.0 || maxVy <= 0.0 || maxOmega <= 0.0) {
            throw new IllegalStateException("Invalid speed mapping config: maxVxInchesPerSec, maxVyInchesPerSec, and maxOmegaRadPerSec must all be > 0");
        }

        // Convert physical units -> normalized command space.
        double axial = speeds.vxRobotIps / maxVx;
        double lateral = speeds.vyRobotIps / maxVy;
        double omega = speeds.omegaRobotRadPerSec / maxOmega;

        // Preserve-direction saturation in command space.
        double maxMag = Math.max(1.0, Math.max(Math.abs(axial), Math.max(Math.abs(lateral), Math.abs(omega))));
        axial /= maxMag;
        lateral /= maxMag;
        omega /= maxMag;

        drive(new DriveSignal(axial, lateral, omega));
    }

    /**
     * Command the drivebase using a {@link DriveSignal}.
     *
     * <p>
     * This method:
     * </p>
     * <ol>
     *   <li>Scales {@code axial}, {@code lateral}, {@code omega} by config scales.</li>
     *   <li>Optionally rate-limits each component based on the most recent {@link #update(LoopClock)}.</li>
     *   <li>Computes mecanum wheel powers using the standard mix.</li>
     *   <li>Normalizes wheel powers if any magnitude exceeds 1.0.</li>
     *   <li>Clamps wheel powers to [-1, +1] and sends them to the hardware.</li>
     * </ol>
     *
     * <p><b>Actuation timing:</b> this method <b>immediately</b> sends wheel power commands
     * to the hardware outputs (via {@link PowerOutput#setPower(double)}). It does not
     * "latch" the command for a later update.</p>
     *
     * <p><b>Rate limiting:</b> if rate limiting is enabled in {@link Config}, call
     * {@link #update(LoopClock)} once per loop <b>before</b> calling this method so the
     * current loop's {@code dtSec} is used.</p>
     *
     * @param s drive command (must not be {@code null})
     */
    public void drive(DriveSignal s) {
        Objects.requireNonNull(s, "s");

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
        //      lateral > 0 -> left
        //      omega   > 0 -> counter-clockwise (turn left, viewed from above)
        double flP = axialCmd - lateralCmd - omegaCmd;
        double frP = axialCmd + lateralCmd + omegaCmd;
        double blP = axialCmd + lateralCmd - omegaCmd;
        double brP = axialCmd - lateralCmd + omegaCmd;

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
     * This method only records loop timing ({@code dtSec}) for optional rate limiting.
     * It does <b>not</b> command motors.
     * </p>
     *
     * <p>
     * Call this once per loop <b>before</b> calling {@link #drive(DriveSignal)} (or
     * {@link #drive(ChassisSpeeds)}) if you want rate limiting to use the most recent dt.
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
     * Immediately stop all four drive outputs and reset last command bookkeeping.
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
