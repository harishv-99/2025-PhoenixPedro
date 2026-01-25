package edu.ftcphoenix.fw.drive.source;

import edu.ftcphoenix.fw.debug.DebugSink;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.DriveSource;
import edu.ftcphoenix.fw.input.Axis;
import edu.ftcphoenix.fw.input.Button;
import edu.ftcphoenix.fw.input.GamepadDevice;
import edu.ftcphoenix.fw.input.Gamepads;
import edu.ftcphoenix.fw.util.LoopClock;
import edu.ftcphoenix.fw.util.MathUtil;

/**
 * {@link DriveSource} implementation that maps gamepad sticks to a {@link DriveSignal}
 * for common FTC TeleOp use cases (mecanum / holonomic).
 *
 * <p>
 * This class is the <em>single</em> recommended place to put:
 * </p>
 *
 * <ul>
 *   <li>Stick mapping (which axis controls axial / lateral / turn).</li>
 *   <li>Deadband, expo, and scaling for the sticks (via {@link GamepadDriveSourceConfig}).</li>
 *   <li>Slow-mode behavior controlled by a button (optional).</li>
 * </ul>
 *
 * <h2>Stick mapping and sign conventions</h2>
 *
 * <p>The standard mecanum TeleOp helpers in this class use:</p>
 *
 * <ul>
 *   <li>P1 left stick Y ({@link GamepadDevice#leftY()}): {@code axial}</li>
 *   <li>P1 left stick X ({@link GamepadDevice#leftX()}): {@code lateral}</li>
 *   <li>P1 right stick X ({@link GamepadDevice#rightX()}): {@code omega}</li>
 * </ul>
 *
 * <p>With {@link GamepadDevice} and {@link DriveSignal} conventions:</p>
 *
 * <ul>
 *   <li>P1 left stick <b>up</b>   &rarr; {@code axial > 0}   &rarr; drive forward</li>
 *   <li>P1 left stick <b>down</b> &rarr; {@code axial < 0}   &rarr; drive backward</li>
 *   <li>P1 left stick <b>right</b> &rarr; {@code lateral > 0} &rarr; strafe right</li>
 *   <li>P1 left stick <b>left</b>  &rarr; {@code lateral < 0} &rarr; strafe left</li>
 *   <li>P1 right stick <b>right</b> &rarr; {@code omega > 0}   &rarr; rotate clockwise (turn right)</li>
 *   <li>P1 right stick <b>left</b>  &rarr; {@code omega < 0}   &rarr; rotate counter-clockwise (turn left)</li>
 * </ul>
 *
 * <h2>Typical usage</h2>
 *
 * <pre>{@code
 * Gamepads pads = Gamepads.create(gamepad1, gamepad2);
 *
 * // Standard stick mapping + shaping + slow mode on P1 right bumper.
 * DriveSource drive = GamepadDriveSource.teleOpMecanumStandard(pads);
 *
 * MecanumDrivebase drivebase = Drives.mecanum(hardwareMap);
 *
 * // In loop():
 * clock.update(getRuntime());
 *
 * DriveSignal cmd = drive.get(clock);   // or drive.get(clock).clamped()
 * drivebase.drive(cmd);
 * drivebase.update(clock);
 * }</pre>
 *
 * <h2>Configuration</h2>
 *
 * <p>
 * Stick shaping parameters live in {@link GamepadDriveSourceConfig}. The recommended pattern:
 * </p>
 *
 * <pre>{@code
 * GamepadDriveSourceConfig cfg = GamepadDriveSourceConfig.defaults();
 * cfg.deadband = 0.08;
 * cfg.rotateExpo = 1.2;
 *
 * DriveSource drive = GamepadDriveSource.teleOpMecanum(
 *         pads,
 *         cfg,
 *         pads.p1().rightBumper(),  // slow-mode button (optional)
 *         0.30                      // slow-mode scale (optional)
 * );
 * }</pre>
 *
 * <p>
 * Implementations that accept a {@link GamepadDriveSourceConfig} (such as
 * {@link #teleOpMecanum(Gamepads, GamepadDriveSourceConfig, Button, double)}) make a
 * <em>defensive copy</em> of the config at construction time. Changing the
 * fields of a {@code GamepadDriveSourceConfig} instance <strong>after</strong> you pass it
 * into a drive source will <strong>not</strong> affect that already-created
 * source.
 * </p>
 *
 * <h2>Generators vs wrappers</h2>
 *
 * <p>
 * {@code GamepadDriveSource} is a <strong>generator</strong>: it reads stick
 * inputs and directly produces a complete {@link DriveSignal} in robot-centric
 * coordinates ({@code axial / lateral / omega}). Feature-specific wrappers like a tag
 * aiming source should wrap a base {@link DriveSource} produced by this class,
 * but this class itself does not wrap other drive sources.
 * </p>
 */
public final class GamepadDriveSource implements DriveSource {

    private final Axis axisLateral;
    private final Axis axisAxial;
    private final Axis axisOmega;

    private final GamepadDriveSourceConfig cfg;

    // Optional slow-mode configuration.
    private final Button slowButton; // may be null
    private final double slowScale;  // only used when slowButton != null

    // Last output for debug/telemetry.
    private DriveSignal lastSignal = DriveSignal.ZERO;

    // ------------------------------------------------------------------------
    // Static helpers (recommended entry points)
    // ------------------------------------------------------------------------

    /**
     * Simple mecanum TeleOp mapping using Phoenix defaults and no slow mode.
     *
     * <p>Uses:</p>
     *
     * <ul>
     *   <li>P1 left stick Y (up &gt; 0) for axial (forward/back).</li>
     *   <li>P1 left stick X (right &gt; 0) for lateral (strafe).</li>
     *   <li>P1 right stick X (right &gt; 0) for omega (rotation).</li>
     *   <li>{@link GamepadDriveSourceConfig#defaults()} for deadband/expo/scale.</li>
     * </ul>
     *
     * @param pads gamepad wrapper created from FTC {@code gamepad1}, {@code gamepad2}
     * @return a {@link DriveSource} that reads P1 sticks and produces drive commands
     */
    public static DriveSource teleOpMecanum(Gamepads pads) {
        if (pads == null) {
            throw new IllegalArgumentException("Gamepads is required");
        }
        return teleOpMecanum(pads, GamepadDriveSourceConfig.defaults(), null, 1.0);
    }

    /**
     * Mecanum TeleOp mapping with custom stick config and no slow mode.
     *
     * @param pads gamepad wrapper created from FTC {@code gamepad1}, {@code gamepad2}
     * @param cfg  stick shaping configuration (deadband, expo, scales)
     * @return a {@link DriveSource} that reads P1 sticks and produces drive commands
     */
    public static DriveSource teleOpMecanum(Gamepads pads, GamepadDriveSourceConfig cfg) {
        if (pads == null) {
            throw new IllegalArgumentException("Gamepads is required");
        }
        if (cfg == null) {
            throw new IllegalArgumentException("GamepadDriveSourceConfig is required");
        }
        return teleOpMecanum(pads, cfg, null, 1.0);
    }

    /**
     * Full-control mecanum TeleOp mapping with custom config and optional slow mode.
     *
     * <p>
     * This method is the most flexible entry point for TeleOp stick wiring. It
     * uses P1 left stick X/Y for translation and P1 right stick X for rotation.
     * The supplied {@link GamepadDriveSourceConfig} controls deadband/expo/scales, and the
     * optional {@code slowButton} + {@code slowScale} control slow-mode behavior.
     * </p>
     *
     * @param pads       gamepad wrapper created from FTC {@code gamepad1}, {@code gamepad2}
     * @param cfg        stick shaping configuration (will be defensively copied)
     * @param slowButton button that enables slow mode while pressed (may be {@code null})
     * @param slowScale  scale applied to all components when slow mode is active
     *                   (must be in (0,1] if {@code slowButton} is non-null)
     * @return a {@link DriveSource} that reads P1 sticks and produces drive commands
     */
    public static DriveSource teleOpMecanum(Gamepads pads,
                                            GamepadDriveSourceConfig cfg,
                                            Button slowButton,
                                            double slowScale) {
        if (pads == null) {
            throw new IllegalArgumentException("Gamepads is required");
        }
        if (cfg == null) {
            throw new IllegalArgumentException("GamepadDriveSourceConfig is required");
        }
        GamepadDevice p1 = pads.p1();
        return new GamepadDriveSource(
                p1.leftX(),   // lateral: + = right
                p1.leftY(),   // axial:   + = forward (stick up)
                p1.rightX(),  // omega:   + = clockwise (stick right)
                cfg,
                slowButton,
                slowScale
        );
    }

    /**
     * Phoenix standard mecanum TeleOp mapping with default shaping and slow mode.
     *
     * <p>Uses:</p>
     *
     * <ul>
     *   <li>P1 left stick Y (up &gt; 0) for axial (forward/back).</li>
     *   <li>P1 left stick X (right &gt; 0) for lateral (strafe).</li>
     *   <li>P1 right stick X (right &gt; 0) for omega (rotation).</li>
     *   <li>{@link GamepadDriveSourceConfig#defaults()} for deadband/expo/scale.</li>
     *   <li>P1 right bumper as a slow-mode button.</li>
     *   <li>Slow-mode scale of {@code 0.30} (30% speed).</li>
     * </ul>
     *
     * @param pads gamepad wrapper created from FTC {@code gamepad1}, {@code gamepad2}
     * @return a {@link DriveSource} ready to plug into a drivebase
     */
    public static DriveSource teleOpMecanumStandard(Gamepads pads) {
        if (pads == null) {
            throw new IllegalArgumentException("Gamepads is required");
        }
        GamepadDriveSourceConfig cfg = GamepadDriveSourceConfig.defaults();
        Button slow = pads.p1().rightBumper();
        double slowScale = 0.30;
        return teleOpMecanum(pads, cfg, slow, slowScale);
    }

    // ------------------------------------------------------------------------
    // Constructors
    // ------------------------------------------------------------------------

    /**
     * Core constructor: map three axes + optional slow button into a drive signal.
     *
     * <p>
     * Most callers should use one of the {@code teleOpMecanum(...)} helpers
     * instead of calling this directly.
     * </p>
     *
     * @param axisLateral axis used for lateral motion (strafe)
     * @param axisAxial   axis used for axial motion (forward/back)
     * @param axisOmega   axis used for rotation (turn)
     * @param cfg         stick shaping configuration (will be defensively copied)
     * @param slowButton  button that enables slow mode while pressed
     *                    (may be {@code null} for no slow mode)
     * @param slowScale   scale applied to all components when slow mode is active
     *                    (must be in (0,1] if {@code slowButton} is non-null)
     */
    public GamepadDriveSource(Axis axisLateral,
                              Axis axisAxial,
                              Axis axisOmega,
                              GamepadDriveSourceConfig cfg,
                              Button slowButton,
                              double slowScale) {
        if (axisLateral == null) {
            throw new IllegalArgumentException("axisLateral is required");
        }
        if (axisAxial == null) {
            throw new IllegalArgumentException("axisAxial is required");
        }
        if (axisOmega == null) {
            throw new IllegalArgumentException("axisOmega is required");
        }
        if (cfg == null) {
            throw new IllegalArgumentException("GamepadDriveSourceConfig is required");
        }
        if (slowButton != null) {
            if (!(slowScale > 0.0 && slowScale <= 1.0)) {
                throw new IllegalArgumentException("slowScale must be in (0,1] when slowButton is non-null");
            }
        }

        this.axisLateral = axisLateral;
        this.axisAxial = axisAxial;
        this.axisOmega = axisOmega;
        this.cfg = cfg.copy(); // defensive copy

        this.slowButton = slowButton;
        this.slowScale = (slowButton != null) ? slowScale : 1.0;
    }

    // ------------------------------------------------------------------------
    // DriveSource implementation
    // ------------------------------------------------------------------------

    /**
     * Compute a drive signal from the configured sticks.
     *
     * <p>
     * {@link LoopClock} is accepted to match the {@link DriveSource}
     * interface, but this implementation does not currently use dt; all
     * shaping here is purely positional. Any time-based smoothing is expected
     * to be handled by the drivebase (for example, via {@code MecanumConfig}
     * in {@code MecanumDrivebase}).
     * </p>
     *
     * <p>The resulting {@link DriveSignal} respects the standard sign
     * conventions:</p>
     *
     * <ul>
     *   <li>{@code axial > 0}   → forward</li>
     *   <li>{@code lateral > 0} → strafe right</li>
     *   <li>{@code omega > 0}   → rotate clockwise (turn right)</li>
     * </ul>
     */
    @Override
    public DriveSignal get(LoopClock clock) {
        double rawLat = axisLateral.get();
        double rawAx = axisAxial.get();
        double rawOm = axisOmega.get();

        double lat = shape(rawLat, cfg.deadband, cfg.translateExpo, cfg.translateScale);
        double ax = shape(rawAx, cfg.deadband, cfg.translateExpo, cfg.translateScale);
        double om = shape(rawOm, cfg.deadband, cfg.rotateExpo, cfg.rotateScale);

        double modeScale = 1.0;
        if (slowButton != null && slowButton.isHeld()) {
            modeScale = slowScale;
        }

        DriveSignal out = new DriveSignal(
                ax * modeScale,
                lat * modeScale,
                om * modeScale
        );
        lastSignal = out;
        return out;
    }

    // ------------------------------------------------------------------------
    // Internal shaping helper
    // ------------------------------------------------------------------------

    /**
     * Apply deadband, shaping exponent, and scaling to a raw stick value.
     *
     * <p>Steps:</p>
     *
     * <ol>
     *   <li>Apply symmetric deadband around zero (using absolute value).</li>
     *   <li>Normalize the remaining magnitude to [0,1].</li>
     *   <li>Apply exponent (1 = linear, &gt;1 = more gentle near center).</li>
     *   <li>Restore sign and apply scale.</li>
     * </ol>
     *
     * @param x        raw stick value in [-1, +1]
     * @param deadband deadband radius (0..1)
     * @param expo     shaping exponent (&gt;= 1 recommended)
     * @param scale    output scale (typically &lt;= 1)
     * @return shaped output in [-scale, +scale]
     */
    private static double shape(double x,
                                double deadband,
                                double expo,
                                double scale) {
        double ax = Math.abs(x);
        if (ax <= deadband) {
            return 0.0;
        }

        // Map [deadband, 1] → [0, 1]
        double norm = (ax - deadband) / (1.0 - deadband);
        norm = MathUtil.clamp01(norm);

        // Apply exponent; expo = 1 → linear, >1 → more gentle near center.
        if (expo < 1.0) {
            expo = 1.0; // avoid amplification near center
        }
        double shaped = Math.pow(norm, expo);

        // Restore sign and apply scale.
        double sign = (x >= 0.0) ? 1.0 : -1.0;
        return sign * scale * shaped;
    }

    // ------------------------------------------------------------------------
    // Debug support
    // ------------------------------------------------------------------------

    /**
     * Dump internal state to a {@link DebugSink}.
     *
     * <p>
     * This is intended for one-off debugging and tuning. Callers can choose
     * any prefix they like; nested callers often use dotted paths such as
     * {@code "drive.sticks"}.
     * </p>
     *
     * <p>
     * This method is <b>defensive</b>: if {@code dbg} is {@code null}, it does
     * nothing. This allows callers to pass in a {@code NullDebugSink} or
     * conditionally pass a real sink.
     * </p>
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

        // Current raw axis readings (sampling now).
        dbg.addData(p + ".axis.lateral.raw", axisLateral.get());
        dbg.addData(p + ".axis.axial.raw", axisAxial.get());
        dbg.addData(p + ".axis.omega.raw", axisOmega.get());

        // Last shaped output.
        dbg.addData(p + ".last.axial", lastSignal.axial);
        dbg.addData(p + ".last.lateral", lastSignal.lateral);
        dbg.addData(p + ".last.omega", lastSignal.omega);

        // Shaping params.
        dbg.addData(p + ".cfg.deadband", cfg.deadband);
        dbg.addData(p + ".cfg.translateExpo", cfg.translateExpo);
        dbg.addData(p + ".cfg.rotateExpo", cfg.rotateExpo);
        dbg.addData(p + ".cfg.translateScale", cfg.translateScale);
        dbg.addData(p + ".cfg.rotateScale", cfg.rotateScale);

        // Slow mode configuration and state.
        dbg.addData(p + ".slow.configured", slowButton != null);
        if (slowButton != null) {
            dbg.addData(p + ".slow.scale", slowScale);
            dbg.addData(p + ".slow.pressed", slowButton.isHeld());
        }
    }

    /**
     * Last computed command from this source.
     *
     * <p>
     * Mainly useful for debugging / telemetry. This is updated on each
     * call to {@link #get(LoopClock)}.
     * </p>
     *
     * @return last {@link DriveSignal} produced by this source
     */
    public DriveSignal getLastSignal() {
        return lastSignal;
    }
}
