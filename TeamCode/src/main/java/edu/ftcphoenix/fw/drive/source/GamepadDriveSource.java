package edu.ftcphoenix.fw.drive.source;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.DriveSource;

/**
 * {@link DriveSource} that maps explicit gamepad-style axes to a robot-centric {@link DriveSignal}
 * for TeleOp driving (for example, mecanum).
 *
 * <h2>What this class is responsible for</h2>
 * <ul>
 *   <li>Mapping <em>explicitly supplied axes</em> to drive intent (axial / lateral / omega).</li>
 *   <li>Stick shaping (deadband + exponent) and scaling (max translate / max omega).</li>
 *   <li>Nothing about button choices, driver slots, or slow-mode policy.</li>
 * </ul>
 *
 * <p>
 * That last point is intentional. The framework treats button bindings and operator semantics as
 * robot-owned policy, not as part of this primitive. A future robot may use different gamepads,
 * different axes, split-driver control, trigger-based turning, or non-gamepad sources entirely.
 * {@code GamepadDriveSource} should still be reusable in all of those cases.
 * </p>
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
 * Standard FTC stick intuition is typically “stick right means right / clockwise”. This class
 * preserves that driver intuition by converting signs at the boundary:
 * </p>
 * <ul>
 *   <li>lateral raw +right becomes {@code lateral < 0} (right strafe) → inverted</li>
 *   <li>omega raw +clockwise becomes {@code omega < 0} (clockwise) → inverted</li>
 * </ul>
 *
 * <h2>Recommended usage</h2>
 *
 * <pre>{@code
 * GamepadDevice driver = gamepads.p1();
 * GamepadDriveSource.Config cfg = GamepadDriveSource.Config.defaults();
 *
 * DriveSource manual = new GamepadDriveSource(
 *         driver.leftX(),
 *         driver.leftY(),
 *         driver.rightX(),
 *         cfg
 * ).scaledWhen(driver.rightBumper(), 0.35, 0.20);
 * }</pre>
 *
 * <p>
 * The important part is that the robot code explicitly chooses the axes and any slow-mode button.
 * This class only performs axis-to-command mapping.
 * </p>
 *
 * <h2>Note on shaping</h2>
 * <p>
 * Stick shaping uses {@link ScalarSource#shaped(double, double, double, double)} with min/max of
 * {@code [-1, +1]} because this class is mapping normalized controller-style axes.
 * </p>
 */
public final class GamepadDriveSource implements DriveSource {

    /**
     * Configuration for TeleOp stick shaping.
     *
     * <p>
     * This is a mutable data object. {@link GamepadDriveSource} makes a defensive copy when it is
     * constructed.
     * </p>
     */
    public static final class Config {

        /**
         * Symmetric deadband radius in [0, 1]. Default: 0.05.
         *
         * <p>
         * Values with {@code |v| <= deadband} are treated as 0. Values outside the deadband are
         * normalized before the exponent is applied.
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
         * Max translation scale applied after shaping. Default: 1.0.
         */
        public double translateScale = 1.0;

        /**
         * Max rotation scale applied after shaping. Default: 1.0.
         */
        public double rotateScale = 1.0;

        private Config() {
            // Defaults set via field initializers.
        }

        /**
         * Creates a config populated with Phoenix defaults.
         *
         * @return new mutable config initialized to the framework defaults
         */
        public static Config defaults() {
            return new Config();
        }

        /**
         * Creates a deep copy of this config.
         *
         * @return copied config whose fields can be edited independently
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
    private final ScalarSource axisAxialRaw;   // +forward in robot-driving intuition
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

    /**
     * Core constructor: map three raw axes into a drive signal using {@link Config}.
     *
     * @param axisLateralRaw raw lateral axis (typically +right)
     * @param axisAxialRaw axial axis (typically +forward)
     * @param axisOmegaRaw raw omega axis (typically +clockwise / turn-right)
     * @param cfg stick-shaping configuration; defensively copied
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
        // - DriveSignal.lateral is +left, but stick X raw is usually +right → invert.
        // - DriveSignal.omega is +CCW, but stick turn raw is usually +clockwise → invert.
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

    /**
     * Samples the explicit axes and returns the current robot-centric drive command.
     *
     * @param clock shared loop clock used to sample the underlying sources
     * @return current robot-centric drive signal after shaping, scaling, and sign conversion
     */
    @Override
    public DriveSignal get(LoopClock clock) {
        // Sample raw axes for debug. These are the calibrated (but unshaped) values from the
        // upstream sources.
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

    /**
     * Dumps internal state to a {@link DebugSink}.
     *
     * <p>
     * Because {@link ScalarSource} requires a {@link LoopClock} to sample, this method reports the
     * most recent raw values cached during {@link #get(LoopClock)}.
     * </p>
     *
     * @param dbg debug sink to write to; ignored when {@code null}
     * @param prefix key prefix for all entries; may be {@code null} or empty
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
     * Returns the most recently computed drive command.
     *
     * @return last command produced by {@link #get(LoopClock)}; initially {@link DriveSignal#zero()}
     */
    public DriveSignal getLastSignal() {
        return lastSignal;
    }
}
