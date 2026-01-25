package edu.ftcphoenix.fw.drive.source;

/**
 * Configuration for stick shaping in {@link GamepadDriveSource}.
 *
 * <p>
 * This class is a simple <strong>mutable data object</strong> that controls how
 * raw stick values (in [-1, +1]) are turned into the axial / lateral / omega
 * components of a high-level drive command.
 * </p>
 *
 * <p>
 * Typical usage is:
 * </p>
 *
 * <pre>{@code
 * // Start from Phoenix defaults.
 * GamepadDriveSourceConfig cfg = GamepadDriveSourceConfig.defaults();
 *
 * // Optional: tweak deadband and rotation expo.
 * cfg.deadband = 0.08;
 * cfg.rotateExpo = 1.2;
 *
 * // Pass the config to a helper that builds your drive source.
 * DriveSource drive = GamepadDriveSource.teleOpMecanum(
 *         pads,
 *         cfg,
 *         pads.p1().rightBumper(),  // slow-mode button (optional)
 *         0.30                      // slow-mode scale (optional)
 * );
 * }</pre>
 *
 * <h2>Shaping model</h2>
 *
 * <p>
 * For each axis, {@link GamepadDriveSource} applies the following steps:
 * </p>
 *
 * <ol>
 *   <li>Apply a symmetric deadband around zero ({@link #deadband}).</li>
 *   <li>Normalize the remaining magnitude to [0, 1].</li>
 *   <li>Apply an exponent ({@link #translateExpo} or {@link #rotateExpo}).</li>
 *   <li>Restore the sign and apply a scale ({@link #translateScale} or {@link #rotateScale}).</li>
 * </ol>
 *
 * <p>
 * Intuitively:
 * </p>
 *
 * <ul>
 *   <li>Deadband removes small stick noise around center.</li>
 *   <li>Expo &gt; 1.0 makes the response gentler near center, but still allows full output.</li>
 *   <li>Scale &lt; 1.0 simply reduces the maximum output in that axis.</li>
 * </ul>
 *
 * <h2>Mutability and usage pattern</h2>
 *
 * <p>
 * This class is intentionally mutable and designed to be configured during
 * robot initialization:
 * </p>
 *
 * <pre>{@code
 * GamepadDriveSourceConfig cfg = GamepadDriveSourceConfig.defaults();
 * cfg.deadband = 0.05;
 * cfg.translateExpo = 1.5;
 * cfg.rotateExpo = 1.5;
 *
 * DriveSource drive = GamepadDriveSource.teleOpMecanum(pads, cfg, null, 1.0);
 * }</pre>
 *
 * <p>
 * Implementations that accept a {@link GamepadDriveSourceConfig} (such as the helpers in
 * {@link GamepadDriveSource}) are expected to make a <em>defensive copy</em> of
 * the config at construction time. Changing the fields of a {@code GamepadDriveSourceConfig}
 * instance <strong>after</strong> you pass it into a drive source will
 * <strong>not</strong> affect that already-created source.
 * </p>
 */
public final class GamepadDriveSourceConfig {

    // ------------------------------------------------------------------------
    // Deadband and shaping exponents
    // ------------------------------------------------------------------------

    /**
     * Symmetric deadband radius in [0, 1].
     *
     * <p>
     * Any raw stick magnitude whose absolute value is {@code <= deadband} will
     * be treated as zero. Values above the deadband are rescaled into [0, 1]
     * before the exponent is applied.
     * </p>
     *
     * <p>Default: {@code 0.05}.</p>
     */
    public double deadband = 0.05;

    /**
     * Exponent used for axial and lateral (translation) shaping.
     *
     * <p>
     * A value of {@code 1.0} leaves the response linear. Values &gt; 1.0 make
     * the response gentler near center and steeper near the edges, which often
     * feels more controllable for drivers.
     * </p>
     *
     * <p>Default: {@code 1.5}.</p>
     */
    public double translateExpo = 1.5;

    /**
     * Exponent used for rotational (omega) shaping.
     *
     * <p>
     * A value of {@code 1.0} leaves the response linear. Values &gt; 1.0 make
     * rotation more gentle near center while still allowing full-speed turns.
     * </p>
     *
     * <p>Default: {@code 1.5}.</p>
     */
    public double rotateExpo = 1.5;

    // ------------------------------------------------------------------------
    // Output scaling
    // ------------------------------------------------------------------------

    /**
     * Maximum scale for axial and lateral (translation) outputs.
     *
     * <p>
     * This is applied after shaping. A value of {@code 1.0} allows full
     * translation output; values &lt; 1.0 reduce the maximum output in both
     * axial and lateral directions.
     * </p>
     *
     * <p>Default: {@code 1.0}.</p>
     */
    public double translateScale = 1.0;

    /**
     * Maximum scale for rotational (omega) output.
     *
     * <p>
     * This is applied after shaping. A value of {@code 1.0} allows full
     * rotation output; values &lt; 1.0 reduce the maximum rotational speed.
     * </p>
     *
     * <p>Default: {@code 1.0}.</p>
     */
    public double rotateScale = 1.0;

    // ------------------------------------------------------------------------
    // Construction helpers
    // ------------------------------------------------------------------------

    /**
     * Private constructor to force use of {@link #defaults()}.
     *
     * <p>
     * The Phoenix philosophy for config objects is:
     * </p>
     *
     * <ul>
     *   <li>Use {@link #defaults()} to get a fresh config with standard values.</li>
     *   <li>Mutate the fields you care about during robot initialization.</li>
     *   <li>Pass the config into the helper or component that needs it.</li>
     * </ul>
     *
     * <p>
     * This keeps all configs starting from a known, well-documented baseline.
     * </p>
     */
    private GamepadDriveSourceConfig() {
        // Defaults are assigned directly in field initializers above.
    }

    /**
     * Create a new {@link GamepadDriveSourceConfig} with Phoenix default values.
     *
     * <p>
     * Defaults are chosen to be safe and intuitive for typical FTC robots:
     * small deadband, mild translation/rotation expo, and full-scale outputs.
     * </p>
     *
     * @return a new config instance with default values
     */
    public static GamepadDriveSourceConfig defaults() {
        return new GamepadDriveSourceConfig();
    }

    /**
     * Create a deep copy of this config.
     *
     * <p>
     * This is useful when you want to start from a base configuration and
     * tweak a few fields without mutating the original object.
     * </p>
     *
     * <pre>{@code
     * GamepadDriveSourceConfig base = GamepadDriveSourceConfig.defaults();
     * base.deadband = 0.05;
     *
     * GamepadDriveSourceConfig copy = base.copy();
     * copy.rotateExpo = 1.2;
     * }</pre>
     *
     * @return a new {@link GamepadDriveSourceConfig} with the same field values
     */
    public GamepadDriveSourceConfig copy() {
        GamepadDriveSourceConfig c = new GamepadDriveSourceConfig();
        c.deadband = this.deadband;
        c.translateExpo = this.translateExpo;
        c.rotateExpo = this.rotateExpo;
        c.translateScale = this.translateScale;
        c.rotateScale = this.rotateScale;
        return c;
    }
}
