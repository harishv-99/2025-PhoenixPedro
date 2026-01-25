package edu.ftcphoenix.fw.drive;

/**
 * Configuration for {@link MecanumDrivebase}.
 *
 * <p>
 * This class is a simple <strong>mutable data object</strong> that controls how
 * a high-level {@link DriveSignal} is mapped into wheel power commands.
 * Typical usage is:
 * </p>
 *
 * <pre>{@code
 * // Start from the Phoenix defaults.
 * MecanumConfig cfg = MecanumConfig.defaults();
 *
 * // Optional: tweak per-axis scaling (example: slightly slower rotation).
 * cfg.maxOmega = 0.8;
 *
 * // Optional: tweak rate limits (example: smoother lateral / strafe).
 * cfg.maxLateralRatePerSec = 4.0;
 *
 * // Pass the config into your drivebase or wiring helper.
 * MecanumDrivebase drive = new MecanumDrivebase(fl, fr, bl, br, cfg);
 * }</pre>
 *
 * <h2>Scaling semantics</h2>
 *
 * <p>
 * The three scale factors {@link #maxAxial}, {@link #maxLateral}, and
 * {@link #maxOmega} are applied directly to the components of a
 * {@link DriveSignal} before the mecanum mixing:
 * </p>
 *
 * <ul>
 *   <li>{@link #maxAxial}: scales forward/back commands.</li>
 *   <li>{@link #maxLateral}: scales strafe left/right commands.</li>
 *   <li>{@link #maxOmega}: scales rotation commands.</li>
 * </ul>
 *
 * <p>
 * For example, if {@code maxLateral = 0.7}, then a full strafe command
 * ({@code lateral = +1.0}) will be treated as {@code lateral = +0.7} before
 * being mixed into wheel powers. This is a simple way to “de-tune” a robot
 * that feels too aggressive on one axis.
 * </p>
 *
 * <h2>Rate limiting semantics (optional)</h2>
 *
 * <p>
 * The three rate limit fields {@link #maxAxialRatePerSec},
 * {@link #maxLateralRatePerSec}, and {@link #maxOmegaRatePerSec} are intended
 * to control how quickly the commanded drive signal is allowed to change over
 * time. They are expressed in “command units per second.” A value
 * {@code <= 0} means “no limit” for that axis.
 * </p>
 *
 * <p>
 * For example, if {@code maxLateralRatePerSec = 4.0}, then the lateral command
 * will be limited to changing by at most {@code 4.0} per second. At ~50 Hz,
 * that corresponds to going from 0 to full strafe in roughly a quarter-second,
 * which can make mecanum drivetrains feel less “tippy.”
 * </p>
 *
 * <p>
 * <strong>Note:</strong> the exact way these rate limits are applied is
 * implemented inside {@link MecanumDrivebase}. This config class simply
 * carries the desired parameters.
 * </p>
 *
 * <h2>Mutability and usage pattern</h2>
 *
 * <p>
 * This class is intentionally mutable and designed to be configured during
 * robot initialization:
 * </p>
 *
 * <pre>{@code
 * MecanumConfig cfg = MecanumConfig.defaults();
 * cfg.maxAxial = 0.9;
 * cfg.maxLateralRatePerSec = 3.0;
 *
 * MecanumDrivebase drive = new MecanumDrivebase(fl, fr, bl, br, cfg);
 * }</pre>
 *
 * <p>
 * Implementations that accept a {@link MecanumConfig} (such as
 * {@link MecanumDrivebase} or helpers in {@link Drives}) are expected to make
 * a <em>defensive copy</em> of the config at construction time. Changing the
 * fields of a {@code MecanumConfig} instance <strong>after</strong> you pass
 * it into a drivebase will <strong>not</strong> affect that already-created
 * drivebase.
 * </p>
 */
public final class MecanumConfig {

    // ------------------------------------------------------------------------
    // Per-axis scaling (high-level DriveSignal components)
    // ------------------------------------------------------------------------

    /**
     * Maximum scale for axial (forward/back) commands.
     *
     * <p>Default: {@code 1.0} (no scaling).</p>
     */
    public double maxAxial = 1.0;

    /**
     * Maximum scale for lateral (strafe left/right) commands.
     *
     * <p>Default: {@code 1.0} (no scaling).</p>
     */
    public double maxLateral = 1.0;

    /**
     * Maximum scale for rotational (omega) commands.
     *
     * <p>Default: {@code 1.0} (no scaling).</p>
     */
    public double maxOmega = 1.0;

    // ------------------------------------------------------------------------
    // Optional per-axis rate limiting (advanced)
    // ------------------------------------------------------------------------

    /**
     * Maximum change in axial command per second.
     *
     * <p>
     * Units: “command units per second” in the same [-1, +1] domain as
     * {@link DriveSignal#axial}. A value {@code <= 0} means “no limit.”
     * </p>
     *
     * <p>Default: {@code 0.0} (no axial rate limit).</p>
     */
    public double maxAxialRatePerSec = 0.0;

    /**
     * Maximum change in lateral command per second.
     *
     * <p>
     * Units: “command units per second” in the same [-1, +1] domain as
     * {@link DriveSignal#lateral}. A value {@code <= 0} means “no limit.”
     * </p>
     *
     * <p>
     * Default: {@code 4.0}. A typical Phoenix tuning might set this to a
     * small value like {@code 4.0} to smooth strafing.
     * </p>
     */
    public double maxLateralRatePerSec = 4.0;

    /**
     * Maximum change in rotational command per second.
     *
     * <p>
     * Units: “command units per second” in the same [-1, +1] domain as
     * {@link DriveSignal#omega}. A value {@code <= 0} means “no limit.”
     * </p>
     *
     * <p>Default: {@code 0.0} (no omega rate limit).</p>
     */
    public double maxOmegaRatePerSec = 0.0;

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
     *   <li>Pass the config into the drivebase or helper that needs it.</li>
     * </ul>
     *
     * <p>
     * This keeps all configs starting from a known, well-documented baseline.
     * </p>
     */
    private MecanumConfig() {
        // Defaults are assigned directly in field initializers above.
    }

    /**
     * Create a new {@link MecanumConfig} with Phoenix default values.
     *
     * <p>
     * Defaults are chosen to be safe and intuitive:
     * </p>
     *
     * <ul>
     *   <li>{@link #maxAxial}   = 1.0</li>
     *   <li>{@link #maxLateral} = 1.0</li>
     *   <li>{@link #maxOmega}   = 1.0</li>
     *   <li>{@link #maxAxialRatePerSec}   = 0.0 (no limit)</li>
     *   <li>{@link #maxLateralRatePerSec} = 0.0 (no limit by default)</li>
     *   <li>{@link #maxOmegaRatePerSec}   = 0.0 (no limit)</li>
     * </ul>
     *
     * <p>
     * Most robots can start with {@code defaults()} unchanged, and only tune
     * individual fields as needed.
     * </p>
     *
     * @return a new config instance with default values
     */
    public static MecanumConfig defaults() {
        return new MecanumConfig();
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
     * MecanumConfig base = MecanumConfig.defaults();
     * base.maxOmega = 0.8;
     *
     * MecanumConfig copy = base.copy();
     * copy.maxLateral = 0.7;
     * }</pre>
     *
     * @return a new {@link MecanumConfig} with the same field values
     */
    public MecanumConfig copy() {
        MecanumConfig c = new MecanumConfig();
        c.maxAxial = this.maxAxial;
        c.maxLateral = this.maxLateral;
        c.maxOmega = this.maxOmega;

        c.maxAxialRatePerSec = this.maxAxialRatePerSec;
        c.maxLateralRatePerSec = this.maxLateralRatePerSec;
        c.maxOmegaRatePerSec = this.maxOmegaRatePerSec;

        return c;
    }
}
