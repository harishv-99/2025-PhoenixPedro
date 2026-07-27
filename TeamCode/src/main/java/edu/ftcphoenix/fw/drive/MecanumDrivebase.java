package edu.ftcphoenix.fw.drive;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.math.MathUtil;

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
 * <h2>Typical FTC usage</h2>
 *
 * <pre>{@code
 * FtcDrives.MecanumConfig config = FtcDrives.MecanumConfig.defaults();
 * config.drivebase.maxOmega = 0.8; // cap rotation before wheel mixing
 *
 * MecanumDrivebase drive = FtcDrives.mecanum(hardwareMap, config);
 *
 * // Shape behavior upstream, then let the drivebase mix and apply it:
 * DriveSource manual = gamepadDrive.rateLimited(4.0, 4.0, 6.0);
 * drive.drive(manual.get(clock).clamped());
 * }</pre>
 *
 * <p>The public constructor that accepts four {@link PowerOutput}s is the hardware-neutral seam for
 * custom output ownership, simulation, and focused tests. Ordinary FTC robots should use the
 * {@code FtcDrives.mecanum(...)} factory so complete-group validation and first-command mode
 * preflight remain intact.</p>
 */
public final class MecanumDrivebase implements DriveCommandSink {

    /**
     * Configuration for {@link MecanumDrivebase}.
     *
     * <p>This is a simple <strong>mutable data object</strong> following the
     * Phoenix convention:</p>
     *
     * <ul>
     *   <li>Start from {@link #defaults()}.</li>
     *   <li>Override the fields you care about during robot initialization.</li>
     *   <li>For an FTC robot, place it in {@code FtcDrives.MecanumConfig} and pass that complete
     *       config to {@code FtcDrives.mecanum(...)}.</li>
     *   <li>Pass it directly to {@link MecanumDrivebase} only when using the advanced
     *       hardware-neutral output-injection seam.</li>
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
         * <p>Must be finite and in [{@code 0.0}, {@code 1.0}]. Default: {@code 1.0}
         * (no scaling).</p>
         */
        public double maxAxial = 1.0;

        /**
         * Scale applied to {@link DriveSignal#lateral} before mixing.
         *
         * <p>Must be finite and in [{@code 0.0}, {@code 1.0}]. Default: {@code 1.0}
         * (no scaling).</p>
         */
        public double maxLateral = 1.0;

        /**
         * Scale applied to {@link DriveSignal#omega} before mixing.
         *
         * <p>Must be finite and in [{@code 0.0}, {@code 1.0}]. Default: {@code 1.0}
         * (no scaling).</p>
         */
        public double maxOmega = 1.0;

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
         * Creates a validated deep copy of this config.
         *
         * @throws IllegalArgumentException if a scale is non-finite or outside [{@code 0.0},
         * {@code 1.0}]
         */
        public Config copy() {
            requireNormalizedScale("maxAxial", maxAxial);
            requireNormalizedScale("maxLateral", maxLateral);
            requireNormalizedScale("maxOmega", maxOmega);

            Config c = new Config();
            c.maxAxial = this.maxAxial;
            c.maxLateral = this.maxLateral;
            c.maxOmega = this.maxOmega;
            return c;
        }

        private static void requireNormalizedScale(String fieldName, double value) {
            if (!Double.isFinite(value) || value < 0.0 || value > 1.0) {
                throw new IllegalArgumentException(
                        "MecanumDrivebase.Config." + fieldName
                                + " must be finite and in [0.0, 1.0], got " + value);
            }
        }
    }

    private final PowerOutput fl;
    private final PowerOutput fr;
    private final PowerOutput bl;
    private final PowerOutput br;

    private final Config cfg;

    // Last commanded drive components after drivebase scaling.
    private double lastAxialCmd;
    private double lastLateralCmd;
    private double lastOmegaCmd;

    // Last commanded wheel powers (after normalization/clamping).
    private double lastFlPower;
    private double lastFrPower;
    private double lastBlPower;
    private double lastBrPower;

    /**
     * Construct a new mecanum drivebase.
     *
     * @param flPower power output for the front-left wheel (non-null)
     * @param frPower power output for the front-right wheel (non-null)
     * @param blPower power output for the back-left wheel (non-null)
     * @param brPower power output for the back-right wheel (non-null)
     * @param cfg     configuration for drivebase scaling (non-null)
     * @throws NullPointerException if any output or {@code cfg} is {@code null}
     * @throws IllegalArgumentException if a configured scale is non-finite or outside
     * [{@code 0.0}, {@code 1.0}]
     */
    public MecanumDrivebase(PowerOutput flPower,
                            PowerOutput frPower,
                            PowerOutput blPower,
                            PowerOutput brPower,
                            Config cfg) {
        this.fl = Objects.requireNonNull(flPower, "flPower");
        this.fr = Objects.requireNonNull(frPower, "frPower");
        this.bl = Objects.requireNonNull(blPower, "blPower");
        this.br = Objects.requireNonNull(brPower, "brPower");

        // Defensive copy so callers can't change behavior by mutating cfg later.
        this.cfg = Objects.requireNonNull(cfg, "cfg").copy();
    }

    /**
     * Command the drivebase using a {@link DriveSignal}.
     *
     * <p>
     * This method:
     * </p>
     * <ol>
     *   <li>Scales {@code axial}, {@code lateral}, {@code omega} by config scales.</li>
     *   <li>Computes mecanum wheel powers using the standard mix.</li>
     *   <li>Normalizes wheel powers if any magnitude exceeds 1.0.</li>
     *   <li>Clamps wheel powers to [-1, +1] and sends them to the hardware.</li>
     * </ol>
     *
     * <p><b>Actuation timing:</b> this method <b>immediately</b> sends wheel power commands
     * to the hardware outputs (via {@link PowerOutput#setPower(double)}). It does not
     * "latch" the command for a later update.</p>
     *
     * <p><b>Rate limiting:</b> {@code MecanumDrivebase} is intentionally a sink, not a behavior
     * shaper. Use {@link DriveSource#rateLimited(double, double)} before calling this method.</p>
     *
     * @param s drive command (must not be {@code null})
     */
    @Override
    public void drive(DriveSignal s) {
        Objects.requireNonNull(s, "s");

        // 1) Apply drivebase-level scaling from the config.
        double axialCmd = s.axial * cfg.maxAxial;
        double lateralCmd = s.lateral * cfg.maxLateral;
        double omegaCmd = s.omega * cfg.maxOmega;

        lastAxialCmd = axialCmd;
        lastLateralCmd = lateralCmd;
        lastOmegaCmd = omegaCmd;

        // 2) Basic mecanum mixing with the scaled components.
        //    Sign conventions (robot-centric):
        //      axial   > 0 -> forward
        //      lateral > 0 -> left
        //      omega   > 0 -> counter-clockwise (turn left, viewed from above)
        double flP = axialCmd - lateralCmd - omegaCmd;
        double frP = axialCmd + lateralCmd + omegaCmd;
        double blP = axialCmd + lateralCmd - omegaCmd;
        double brP = axialCmd - lateralCmd + omegaCmd;

        // 3) Normalize wheel powers if any exceeds |1.0| to preserve direction.
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

        // 4) Final clamp (mainly for numerical safety) and apply,
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
     * Immediately stop all four drive outputs through their lifecycle stop hooks and reset last
     * command bookkeeping.
     */
    @Override
    public void stop() {
        lastAxialCmd = 0.0;
        lastLateralCmd = 0.0;
        lastOmegaCmd = 0.0;

        lastFlPower = 0.0;
        lastFrPower = 0.0;
        lastBlPower = 0.0;
        lastBrPower = 0.0;

        fl.stop();
        fr.stop();
        bl.stop();
        br.stop();
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
    }

    // ------------------------------------------------------------------------
    // Accessors
    // ------------------------------------------------------------------------

    /**
     * @return last commanded scaled axial command.
     */
    public double getLastAxialCmd() {
        return lastAxialCmd;
    }

    /**
     * @return last commanded scaled lateral command.
     */
    public double getLastLateralCmd() {
        return lastLateralCmd;
    }

    /**
     * @return last commanded scaled omega command.
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
