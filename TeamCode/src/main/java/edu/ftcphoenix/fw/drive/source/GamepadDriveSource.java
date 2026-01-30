package edu.ftcphoenix.fw.drive.source;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.DriveSource;
import edu.ftcphoenix.fw.input.GamepadDevice;
import edu.ftcphoenix.fw.input.Gamepads;

/**
 * {@link DriveSource} that maps gamepad inputs to a robot-centric {@link DriveSignal}
 * for TeleOp driving (e.g., mecanum).
 *
 * <h2>What this class is responsible for</h2>
 * <ul>
 *   <li>Mapping gamepad axes to drive intent (axial / lateral / omega).</li>
 *   <li>Stick shaping (deadband + exponent) and scaling (max translate / max omega).</li>
 *   <li><em>Slow mode</em> is handled externally using {@link DriveSource#scaledWhen} so it can be
 *       applied consistently to any {@link DriveSource} (not just stick drive).</li>
 * </ul>
 *
 * <h2>Phoenix sign conventions</h2>
 * <p>{@link DriveSignal} uses Phoenix conventions:</p>
 * <ul>
 *   <li>{@code axial > 0}   → forward</li>
 *   <li>{@code lateral > 0} → left</li>
 *   <li>{@code omega > 0}   → CCW (turn left)</li>
 * </ul>
 *
 * <p>
 * Standard FTC stick intuition is typically “stick right means right / clockwise”.
 * This class preserves that driver intuition by converting signs at the boundary:
 * </p>
 * <ul>
 *   <li>Left stick X: raw +right becomes {@code lateral < 0} (right strafe) → inverted</li>
 *   <li>Right stick X: raw +clockwise becomes {@code omega < 0} (clockwise) → inverted</li>
 * </ul>
 *
 * <h2>Recommended usage</h2>
 *
 * <pre>{@code
 * Gamepads pads = Gamepads.create(gamepad1, gamepad2);
 *
 * // Default mapping + shaping, no slow-mode (avoids button conflicts by default).
 * DriveSource drive = GamepadDriveSource.teleOpMecanum(pads);
 *
 * // Default mapping + shaping + slow mode on RB (with practical default slow scales).
 * DriveSource driveSlow = GamepadDriveSource.teleOpMecanumSlowRb(pads);
 *
 * // Or: explicit config (for custom shaping) plus a slow-mode wrapper.
 * GamepadDriveSource.Config cfg = GamepadDriveSource.Config.defaults();
 * DriveSource driveCustom = GamepadDriveSource.teleOpMecanum(pads, cfg)
 *         .scaledWhen(pads.p1().rightBumper(), 0.35, 0.20);
 * }</pre>
 *
 * <h2>Note on shaping</h2>
 * <p>
 * Stick shaping uses {@link ScalarSource#shaped(double, double, double, double)} with min/max
 * of {@code [-1, +1]} because this class is mapping gamepad sticks.
 * </p>
 */
public final class GamepadDriveSource implements DriveSource {

    /**
     * Configuration for TeleOp stick shaping.
     *
     * <p>
     * This is a mutable data object. {@link GamepadDriveSource} makes a defensive copy
     * when constructed.
     * </p>
     */
    public static final class Config {

        /**
         * Symmetric deadband radius in [0, 1]. Default: 0.05.
         *
         * <p>
         * Values with {@code |v| <= deadband} are treated as 0. Values outside the deadband
         * are normalized before the exponent is applied.
         * </p>
         */
        public double deadband = 0.05;

        /**
         * Exponent for translation (axial + lateral). Default: 1.5.
         *
         * <p>Values &gt; 1 soften near center and keep full-scale at the edges.</p>
         */
        public double translateExpo = 1.5;

        /**
         * Exponent for rotation (omega). Default: 1.5.
         */
        public double rotateExpo = 1.5;

        /**
         * Max translation scale (applied after shaping). Default: 1.0.
         */
        public double translateScale = 1.0;

        /**
         * Max rotation scale (applied after shaping). Default: 1.0.
         */
        public double rotateScale = 1.0;

        private Config() {
            // Defaults set via field initializers.
        }

        /**
         * Default shaping (Phoenix defaults).
         */
        public static Config defaults() {
            return new Config();
        }

        /**
         * Deep copy of this config.
         */
        public Config copy() {
            Config c = new Config();
            c.deadband = this.deadband;
            c.translateExpo = this.translateExpo;
            c.rotateExpo = this.rotateExpo;
            c.translateScale = this.translateScale;
            c.rotateScale = this.rotateScale;

            return c;
        }
    }

    // Raw axes (sampled for debug).
    private final ScalarSource axisLateralRaw; // raw +right (typical)
    private final ScalarSource axisAxialRaw;   // +forward (per GamepadDevice)
    private final ScalarSource axisOmegaRaw;   // raw +clockwise (typical)

    // Shaped axes in Phoenix DriveSignal conventions.
    private final ScalarSource axisAxialCmd;
    private final ScalarSource axisLateralCmd; // +left
    private final ScalarSource axisOmegaCmd;   // +CCW

    private final Config cfg;

    private DriveSignal lastSignal = DriveSignal.zero();

    // Cached raw axis samples from the most recent get(clock) call (for debug).
    private double lastLateralRaw = 0.0;
    private double lastAxialRaw = 0.0;
    private double lastOmegaRaw = 0.0;

    // ------------------------------------------------------------------------
    // Recommended entry points
    // ------------------------------------------------------------------------

    /**
     * Mecanum TeleOp mapping using Phoenix defaults (no slow mode).
     *
     * <p>
     * Mapping:
     * <ul>
     *   <li>P1 left stick Y → axial</li>
     *   <li>P1 left stick X → lateral</li>
     *   <li>P1 right stick X → omega</li>
     * </ul>
     * </p>
     */
    public static DriveSource teleOpMecanum(Gamepads pads) {
        if (pads == null) {
            throw new IllegalArgumentException("Gamepads is required");
        }
        return teleOpMecanum(pads, Config.defaults());
    }

    /**
     * Mecanum TeleOp mapping with custom config (stick shaping only).
     *
     * <p>
     * Most teams should start with {@link Config#defaults()}. If you want slow mode,
     * apply it externally using {@link DriveSource#scaledWhen(edu.ftcphoenix.fw.core.source.BooleanSource, double, double)}.
     * </p>
     */
    public static DriveSource teleOpMecanum(Gamepads pads, Config cfg) {
        if (pads == null) {
            throw new IllegalArgumentException("Gamepads is required");
        }
        if (cfg == null) {
            throw new IllegalArgumentException("GamepadDriveSource.Config is required");
        }

        GamepadDevice p1 = pads.p1();
        return new GamepadDriveSource(
                p1.leftX(),
                p1.leftY(),
                p1.rightX(),
                cfg
        );
    }

    /**
     * Convenience factory: {@link #teleOpMecanum(Gamepads)} plus slow mode on P1 RB.
     *
     * <p>This method exists purely as a beginner-friendly starting point. Internally it is just
     * {@code teleOpMecanum(pads).scaledWhen(RB, 0.35, 0.20)}.</p>
     */
    public static DriveSource teleOpMecanumSlowRb(Gamepads pads) {
        if (pads == null) {
            throw new IllegalArgumentException("Gamepads is required");
        }
        BooleanSource rb = pads.p1().rightBumper();
        return teleOpMecanum(pads)
                .scaledWhen(rb, 0.35, 0.20);
    }

    /**
     * Convenience: create a drive source from arbitrary axes.
     *
     * <p>This is handy for “microdrive” or “nudge” control where you want to drive using
     * dpad buttons (converted to axes) or triggers, but still reuse the same shaping and scaling
     * logic as normal stick drive.</p>
     */
    public static DriveSource fromAxes(ScalarSource lateralRaw, ScalarSource axialRaw, ScalarSource omegaRaw, Config cfg) {
        return new GamepadDriveSource(lateralRaw, axialRaw, omegaRaw, cfg);
    }

    // ------------------------------------------------------------------------
    // Construction
    // ------------------------------------------------------------------------

    /**
     * Core constructor: map three raw axes into a drive signal using {@link Config}.
     *
     * <p>
     * Most callers should use one of the {@code teleOpMecanum(...)} helpers.
     * </p>
     *
     * @param axisLateralRaw raw lateral axis (typically +right)
     * @param axisAxialRaw   axial axis (+forward per {@link GamepadDevice})
     * @param axisOmegaRaw   raw omega axis (typically +clockwise / turn-right)
     * @param cfg            stick shaping configuration (defensively copied)
     */
    public GamepadDriveSource(ScalarSource axisLateralRaw,
                              ScalarSource axisAxialRaw,
                              ScalarSource axisOmegaRaw,
                              Config cfg) {
        if (axisLateralRaw == null) {
            throw new IllegalArgumentException("axisLateralRaw is required");
        }
        if (axisAxialRaw == null) {
            throw new IllegalArgumentException("axisAxialRaw is required");
        }
        if (axisOmegaRaw == null) {
            throw new IllegalArgumentException("axisOmegaRaw is required");
        }
        if (cfg == null) {
            throw new IllegalArgumentException("GamepadDriveSource.Config is required");
        }

        this.axisLateralRaw = axisLateralRaw;
        this.axisAxialRaw = axisAxialRaw;
        this.axisOmegaRaw = axisOmegaRaw;
        this.cfg = cfg.copy();

        // Build shaped command axes (pre-built wrappers, no per-loop allocation).
        //
        // Notes on sign:
        // - DriveSignal.lateral is +left, but stick X raw is +right → invert.
        // - DriveSignal.omega is +CCW, but stick X raw is +clockwise → invert.
        ScalarSource axial = this.axisAxialRaw
                .shaped(this.cfg.deadband, this.cfg.translateExpo, -1.0, 1.0)
                .scaled(this.cfg.translateScale);

        ScalarSource lateralLeft = this.axisLateralRaw
                .shaped(this.cfg.deadband, this.cfg.translateExpo, -1.0, 1.0)
                .scaled(this.cfg.translateScale)
                .inverted();

        ScalarSource omegaCcw = this.axisOmegaRaw
                .shaped(this.cfg.deadband, this.cfg.rotateExpo, -1.0, 1.0)
                .scaled(this.cfg.rotateScale)
                .inverted();

        this.axisAxialCmd = axial;
        this.axisLateralCmd = lateralLeft;
        this.axisOmegaCmd = omegaCcw;
    }

    // ------------------------------------------------------------------------
    // DriveSource implementation
    // ------------------------------------------------------------------------

    /**
     * {@inheritDoc}
     */
    @Override
    public DriveSignal get(LoopClock clock) {
        // Sample raw axes for debug. These are the calibrated (but unshaped) values from GamepadDevice.
        lastLateralRaw = axisLateralRaw.getAsDouble(clock);
        lastAxialRaw = axisAxialRaw.getAsDouble(clock);
        lastOmegaRaw = axisOmegaRaw.getAsDouble(clock);

        // Sample shaped command axes.
        double ax = axisAxialCmd.getAsDouble(clock);
        double lat = axisLateralCmd.getAsDouble(clock);
        double om = axisOmegaCmd.getAsDouble(clock);

        DriveSignal out = new DriveSignal(ax, lat, om);
        lastSignal = out;
        return out;
    }


    // ------------------------------------------------------------------------
    // Debug support
    // ------------------------------------------------------------------------

    /**
     * Dump internal state to a {@link DebugSink}.
     *
     * <p>Because {@link ScalarSource} requires a {@link LoopClock} to sample, this method reports
     * the most recent raw values cached during {@link #get(LoopClock)}.</p>
     *
     * @param dbg    debug sink to write to (may be {@code null})
     * @param prefix key prefix for all entries (may be {@code null} or empty)
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "sticks" : prefix;

        dbg.addLine(p + ": GamepadDriveSource");

        dbg.addData(p + ".axis.lateral.raw", lastLateralRaw);
        dbg.addData(p + ".axis.axial.raw", lastAxialRaw);
        dbg.addData(p + ".axis.omega.raw", lastOmegaRaw);

        dbg.addData(p + ".last.axial", lastSignal.axial);
        dbg.addData(p + ".last.lateral", lastSignal.lateral);
        dbg.addData(p + ".last.omega", lastSignal.omega);

        dbg.addData(p + ".cfg.deadband", cfg.deadband);
        dbg.addData(p + ".cfg.translateExpo", cfg.translateExpo);
        dbg.addData(p + ".cfg.rotateExpo", cfg.rotateExpo);
        dbg.addData(p + ".cfg.translateScale", cfg.translateScale);
        dbg.addData(p + ".cfg.rotateScale", cfg.rotateScale);
    }

    /**
     * Last computed command from this source.
     */
    public DriveSignal getLastSignal() {
        return lastSignal;
    }
}