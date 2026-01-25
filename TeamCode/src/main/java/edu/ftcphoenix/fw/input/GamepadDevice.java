package edu.ftcphoenix.fw.input;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.lang.reflect.Field;

import edu.ftcphoenix.fw.core.debug.DebugSink;

/**
 * Thin wrapper around an FTC {@link Gamepad} that exposes:
 * <ul>
 *     <li>Axes ({@link Axis}) for sticks and triggers.</li>
 *     <li>Buttons ({@link Button}) for digital inputs (with edge detection).</li>
 * </ul>
 *
 * <h2>Axis conventions</h2>
 * Axes use a <b>human-friendly</b> convention:
 * <ul>
 *     <li><b>leftX</b>:  -1.0 = full left,  +1.0 = full right</li>
 *     <li><b>leftY</b>:  -1.0 = full down, +1.0 = full up</li>
 *     <li><b>rightX</b>: -1.0 = full left,  +1.0 = full right</li>
 *     <li><b>rightY</b>: -1.0 = full down, +1.0 = full up</li>
 *     <li>Triggers: 0.0 = released, 1.0 = fully pressed</li>
 * </ul>
 *
 * <p>
 * This inverts the raw FTC {@link Gamepad} Y-axis values (where pushing a stick
 * up yields a <em>negative</em> value and down yields a <em>positive</em> value) so that
 * <b>"up" is always positive</b> in Phoenix code. X-axis values are passed through as-is.
 * </p>
 *
 * <h2>Axis calibration, rescaling, and deadband</h2>
 * <p>
 * Some gamepads do not return an exact 0.0 when the sticks are released, or may drift
 * slightly over time. {@code GamepadDevice} supports two layers of error correction:
 * </p>
 * <ul>
 *     <li><b>Center offsets</b> – a per-axis "neutral" value that is treated as 0.0.
 *     This is set by {@link #calibrate()} and recentered so that the current readings
 *     become the logical center.</li>
 *     <li><b>Rescaling</b> – after recentering, axes are linearly rescaled so that the
 *     full mechanical travel still maps to the full logical range:
 *     <ul>
 *         <li>Sticks: [-1.0, +1.0]</li>
 *         <li>Triggers: [0.0, 1.0]</li>
 *     </ul>
 *     For example, if a stick is slightly off-center at rest, pushing it fully to the
 *     edge will still yield values near -1.0 or +1.0 after calibration.</li>
 *     <li><b>Deadband</b> – any corrected value whose magnitude is below
 *     {@link #axisDeadband()} is treated as exactly 0.0. This filters out small noise
 *     around center even after calibration and rescaling.</li>
 * </ul>
 *
 * <p>
 * On construction, {@link #calibrate()} is called automatically, so in the typical case
 * users do not need to call any extra methods. The calibration method remains public so
 * you can re-calibrate later if a gamepad starts to drift during a match.
 * </p>
 */
public final class GamepadDevice {

    /**
     * Default deadband applied to all axes after calibration. Values whose absolute
     * magnitude is below this threshold are treated as exactly 0.0.
     */
    public static final double DEFAULT_AXIS_DEADBAND = 0.02;

    private static final double MIN_SCALE = 1e-3; // protect against division by zero

    /**
     * Optional Gamepad fields vary by controller/SDK:
     * <ul>
     *   <li>Xbox often uses {@code back}/{@code start}</li>
     *   <li>PlayStation often uses {@code share}/{@code options}</li>
     * </ul>
     *
     * <p>We use reflection so this wrapper compiles even if some fields do not exist.</p>
     */
    private static final Field F_BACK = fieldOrNull("back");
    private static final Field F_START = fieldOrNull("start");
    private static final Field F_SHARE = fieldOrNull("share");
    private static final Field F_OPTIONS = fieldOrNull("options");

    private static Field fieldOrNull(String name) {
        try {
            Field f = Gamepad.class.getField(name);
            f.setAccessible(true);
            return f;
        } catch (Exception ignored) {
            return null;
        }
    }

    private static boolean readBooleanAny(Gamepad gp, Field... fields) {
        for (Field f : fields) {
            if (f == null) continue;
            try {
                if (f.getBoolean(gp)) return true;
            } catch (Exception ignored) {
                // fall through
            }
        }
        return false;
    }

    private final Gamepad gp;

    // Per-axis center offsets in "human-friendly" coordinates (after Y inversion).
    private double leftXCenter = 0.0;
    private double leftYCenter = 0.0;
    private double rightXCenter = 0.0;
    private double rightYCenter = 0.0;
    private double leftTriggerCenter = 0.0;
    private double rightTriggerCenter = 0.0;

    // Deadband applied to all corrected axis values.
    private double axisDeadband = DEFAULT_AXIS_DEADBAND;

    // Axes
    private final Axis leftX;
    private final Axis leftY;
    private final Axis rightX;
    private final Axis rightY;
    private final Axis leftTrigger;
    private final Axis rightTrigger;

    // Common 2D stick helpers
    private final Axis leftStickMagnitude;
    private final Axis rightStickMagnitude;

    // Buttons
    private final Button a;
    private final Button b;
    private final Button x;
    private final Button y;
    private final Button leftBumper;
    private final Button rightBumper;
    private final Button dpadUp;
    private final Button dpadDown;
    private final Button dpadLeft;
    private final Button dpadRight;

    // Stick click buttons
    private final Button leftStickButton;
    private final Button rightStickButton;

    // Menu/system buttons (controller dependent naming)
    private final Button back;
    private final Button start;

    /**
     * Create a {@link GamepadDevice} wrapper around an FTC {@link Gamepad}.
     *
     * <p>The constructor calls {@link #calibrate()} automatically, treating the current stick/trigger
     * positions as neutral. In most robots, you will construct this in {@code init()} and then call
     * {@link #calibrate()} again only if a controller starts drifting.</p>
     *
     * @param gp FTC SDK gamepad instance (for example {@code gamepad1} or {@code gamepad2})
     * @throws NullPointerException if {@code gp} is {@code null}
     */
    public GamepadDevice(Gamepad gp) {
        this.gp = gp;

        // Axes: build them in terms of raw + calibration + deadband.
        // X passes through, Y is inverted so that "up" is positive.
        this.leftX = Axis.of(() -> applyDeadband(calibratedStick(rawLeftX(), leftXCenter)));
        this.leftY = Axis.of(() -> applyDeadband(calibratedStick(rawLeftY(), leftYCenter)));
        this.rightX = Axis.of(() -> applyDeadband(calibratedStick(rawRightX(), rightXCenter)));
        this.rightY = Axis.of(() -> applyDeadband(calibratedStick(rawRightY(), rightYCenter)));

        this.leftTrigger = Axis.of(() -> applyDeadband(calibratedTrigger(rawLeftTrigger(), leftTriggerCenter)));
        this.rightTrigger = Axis.of(() -> applyDeadband(calibratedTrigger(rawRightTrigger(), rightTriggerCenter)));

        // 2D stick magnitudes (after calibration + deadband).
        this.leftStickMagnitude = Axis.magnitude(this.leftX, this.leftY);
        this.rightStickMagnitude = Axis.magnitude(this.rightX, this.rightY);

        // Buttons
        this.a = Button.of(() -> gp.a);
        this.b = Button.of(() -> gp.b);
        this.x = Button.of(() -> gp.x);
        this.y = Button.of(() -> gp.y);

        this.leftBumper = Button.of(() -> gp.left_bumper);
        this.rightBumper = Button.of(() -> gp.right_bumper);

        this.dpadUp = Button.of(() -> gp.dpad_up);
        this.dpadDown = Button.of(() -> gp.dpad_down);
        this.dpadLeft = Button.of(() -> gp.dpad_left);
        this.dpadRight = Button.of(() -> gp.dpad_right);

        // Stick click buttons
        this.leftStickButton = Button.of(() -> gp.left_stick_button);
        this.rightStickButton = Button.of(() -> gp.right_stick_button);

        // Menu/system buttons (controller-dependent naming across SDK/controller mappings)
        this.back = Button.of(() -> readBooleanAny(gp, F_BACK, F_SHARE));
        this.start = Button.of(() -> readBooleanAny(gp, F_START, F_OPTIONS));

        // Automatically treat the current stick/trigger positions as neutral and
        // configure scaling so we still reach the full logical range.
        calibrate();
    }

    // ---------------------------------------------------------------------------------------------
    // Raw axis helpers in human-friendly coordinates
    // ---------------------------------------------------------------------------------------------

    /**
     * Raw left stick X: -1.0 = left, +1.0 = right.
     */
    private double rawLeftX() {
        return gp.left_stick_x;
    }

    /**
     * Raw left stick Y: -1.0 = down, +1.0 = up.
     * The FTC SDK reports up as negative, so we invert here.
     */
    private double rawLeftY() {
        return -gp.left_stick_y;
    }

    /**
     * Raw right stick X: -1.0 = left, +1.0 = right.
     */
    private double rawRightX() {
        return gp.right_stick_x;
    }

    /**
     * Raw right stick Y: -1.0 = down, +1.0 = up (FTC SDK inverted; we flip sign).
     */
    private double rawRightY() {
        return -gp.right_stick_y;
    }

    /**
     * Raw left trigger: 0.0 = released, 1.0 = fully pressed.
     */
    private double rawLeftTrigger() {
        return gp.left_trigger;
    }

    /**
     * Raw right trigger: 0.0 = released, 1.0 = fully pressed.
     */
    private double rawRightTrigger() {
        return gp.right_trigger;
    }

    // ---------------------------------------------------------------------------------------------
    // Calibration math
    // ---------------------------------------------------------------------------------------------

    /**
     * Calibrates a stick axis (range [-1, +1]) given the current raw value and stored center.
     *
     * <p>We do a piecewise linear rescaling so that:</p>
     * <ul>
     *     <li>{@code raw == center} maps to 0.0</li>
     *     <li>{@code raw == +1.0} maps to +1.0</li>
     *     <li>{@code raw == -1.0} maps to -1.0</li>
     * </ul>
     */
    private double calibratedStick(double raw, double center) {
        if (raw == center) {
            return 0.0;
        }

        final double scale;
        if (raw > center) {
            // Map [center, +1] → [0, +1]
            scale = Math.max(MIN_SCALE, 1.0 - center);
        } else {
            // Map [-1, center] → [-1, 0]
            scale = Math.max(MIN_SCALE, 1.0 + center);
        }

        double value = (raw - center) / scale;

        // Clamp to [-1, +1] in case of slight overshoot.
        if (value > 1.0) value = 1.0;
        if (value < -1.0) value = -1.0;
        return value;
    }

    /**
     * Calibrates a trigger axis (range [0, 1]) given the current raw value and stored center.
     *
     * <p>{@code center} is treated as logical 0.0, then we rescale so {@code raw==1.0 → 1.0}.</p>
     */
    private double calibratedTrigger(double raw, double center) {
        double scale = Math.max(MIN_SCALE, 1.0 - center);
        double value = (raw - center) / scale;

        if (value < 0.0) value = 0.0;
        if (value > 1.0) value = 1.0;
        return value;
    }

    /**
     * Applies the current deadband to a corrected axis value.
     * Values whose absolute magnitude is below {@link #axisDeadband} are treated as 0.0.
     */
    private double applyDeadband(double value) {
        return (Math.abs(value) < axisDeadband) ? 0.0 : value;
    }

    // ---------------------------------------------------------------------------------------------
    // Public API: axes
    // ---------------------------------------------------------------------------------------------

    /**
     * Left stick X axis: -1.0 = full left, +1.0 = full right.
     */
    public Axis leftX() {
        return leftX;
    }

    /**
     * Left stick Y axis: -1.0 = full down, +1.0 = full up (Phoenix convention).
     */
    public Axis leftY() {
        return leftY;
    }

    /**
     * Right stick X axis: -1.0 = full left, +1.0 = full right.
     */
    public Axis rightX() {
        return rightX;
    }

    /**
     * Right stick Y axis: -1.0 = full down, +1.0 = full up (Phoenix convention).
     */
    public Axis rightY() {
        return rightY;
    }

    /**
     * Magnitude of the left stick vector (x,y) after calibration and deadband: {@code hypot(leftX, leftY)}.
     */
    public Axis leftStickMagnitude() {
        return leftStickMagnitude;
    }

    /**
     * Magnitude of the right stick vector (x,y) after calibration and deadband: {@code hypot(rightX, rightY)}.
     */
    public Axis rightStickMagnitude() {
        return rightStickMagnitude;
    }

    /**
     * Left trigger axis: 0.0 = released, 1.0 = fully pressed (after calibration and deadband).
     */
    public Axis leftTrigger() {
        return leftTrigger;
    }

    /**
     * Right trigger axis: 0.0 = released, 1.0 = fully pressed (after calibration and deadband).
     */
    public Axis rightTrigger() {
        return rightTrigger;
    }

    // ---------------------------------------------------------------------------------------------
    // Public API: buttons
    // ---------------------------------------------------------------------------------------------

    /**
     * The A button.
     *
     * @return a {@link Button} with edge detection
     */
    public Button a() {
        return a;
    }

    /**
     * The B button.
     *
     * @return a {@link Button} with edge detection
     */
    public Button b() {
        return b;
    }

    /**
     * The X button.
     *
     * @return a {@link Button} with edge detection
     */
    public Button x() {
        return x;
    }

    /**
     * The Y button.
     *
     * @return a {@link Button} with edge detection
     */
    public Button y() {
        return y;
    }

    /**
     * Left bumper.
     */
    public Button leftBumper() {
        return leftBumper;
    }

    /**
     * Right bumper.
     */
    public Button rightBumper() {
        return rightBumper;
    }

    /**
     * Alias for {@link #leftBumper()} to match common “LB/RB” naming in examples.
     */
    public Button lb() {
        return leftBumper;
    }

    /**
     * Alias for {@link #rightBumper()} to match common “LB/RB” naming in examples.
     */
    public Button rb() {
        return rightBumper;
    }

    /**
     * D-pad up.
     *
     * @return a {@link Button} with edge detection
     */
    public Button dpadUp() {
        return dpadUp;
    }

    /**
     * D-pad down.
     *
     * @return a {@link Button} with edge detection
     */
    public Button dpadDown() {
        return dpadDown;
    }

    /**
     * D-pad left.
     *
     * @return a {@link Button} with edge detection
     */
    public Button dpadLeft() {
        return dpadLeft;
    }

    /**
     * D-pad right.
     *
     * @return a {@link Button} with edge detection
     */
    public Button dpadRight() {
        return dpadRight;
    }

    /**
     * Left stick button (press the left stick).
     */
    public Button leftStickButton() {
        return leftStickButton;
    }

    /**
     * Right stick button (press the right stick).
     */
    public Button rightStickButton() {
        return rightStickButton;
    }

    /**
     * Alias for {@link #leftStickButton()} (common "LS" naming).
     */
    public Button ls() {
        return leftStickButton;
    }

    /**
     * Alias for {@link #rightStickButton()} (common "RS" naming).
     */
    public Button rs() {
        return rightStickButton;
    }

    /**
     * Back / View / Share (controller dependent).
     * <p>On PlayStation-style mappings this typically corresponds to “Share”.</p>
     */
    public Button back() {
        return back;
    }

    /**
     * Start / Options (controller dependent).
     * <p>On PlayStation-style mappings this typically corresponds to “Options”.</p>
     */
    public Button start() {
        return start;
    }

    // ---------------------------------------------------------------------------------------------
    // Calibration / deadband configuration
    // ---------------------------------------------------------------------------------------------

    /**
     * Calibrates all analog axes (sticks and triggers) by treating their <b>current</b>
     * positions as neutral.
     *
     * <p>This method is called once automatically from the constructor.</p>
     */
    public void calibrate() {
        leftXCenter = rawLeftX();
        leftYCenter = rawLeftY();
        rightXCenter = rawRightX();
        rightYCenter = rawRightY();
        leftTriggerCenter = rawLeftTrigger();
        rightTriggerCenter = rawRightTrigger();
    }

    /**
     * Sets the deadband used for all axes.
     *
     * <p>Any corrected axis value (after subtracting calibration offset and rescaling)
     * whose absolute magnitude is below this threshold will be reported as exactly 0.0.</p>
     *
     * @param deadband new deadband threshold in the range [0.0, 1.0].
     */
    public void setAxisDeadband(double deadband) {
        this.axisDeadband = Math.max(0.0, Math.min(1.0, deadband));
    }

    /**
     * Return the current axis deadband.
     *
     * @return deadband threshold in {@code [0.0, 1.0]}
     */
    public double axisDeadband() {
        return axisDeadband;
    }

    // ---------------------------------------------------------------------------------------------
    // Debugging
    // ---------------------------------------------------------------------------------------------

    /**
     * Emits diagnostic information about this {@code GamepadDevice} into the provided {@link DebugSink}.
     *
     * @param dbg    debug sink (may be {@code null}; if null, no output is produced)
     * @param prefix base key prefix, e.g. {@code "p1.gamepad"}
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) return;

        String p = (prefix == null || prefix.isEmpty()) ? "gamepad" : prefix;

        double rawLX = rawLeftX();
        double rawLY = rawLeftY();
        double rawRX = rawRightX();
        double rawRY = rawRightY();
        double rawLT = rawLeftTrigger();
        double rawRT = rawRightTrigger();

        double calLX = calibratedStick(rawLX, leftXCenter);
        double calLY = calibratedStick(rawLY, leftYCenter);
        double calRX = calibratedStick(rawRX, rightXCenter);
        double calRY = calibratedStick(rawRY, rightYCenter);
        double calLT = calibratedTrigger(rawLT, leftTriggerCenter);
        double calRT = calibratedTrigger(rawRT, rightTriggerCenter);

        double outLX = applyDeadband(calLX);
        double outLY = applyDeadband(calLY);
        double outRX = applyDeadband(calRX);
        double outRY = applyDeadband(calRY);
        double outLT = applyDeadband(calLT);
        double outRT = applyDeadband(calRT);

        dbg.addLine(p + ": GamepadDevice");
        dbg.addData(p + ".deadband", axisDeadband);

        dbg.addData(p + ".leftX.raw", rawLX)
                .addData(p + ".leftX.center", leftXCenter)
                .addData(p + ".leftX.calib", calLX)
                .addData(p + ".leftX.out", outLX);

        dbg.addData(p + ".leftY.raw", rawLY)
                .addData(p + ".leftY.center", leftYCenter)
                .addData(p + ".leftY.calib", calLY)
                .addData(p + ".leftY.out", outLY);

        dbg.addData(p + ".rightX.raw", rawRX)
                .addData(p + ".rightX.center", rightXCenter)
                .addData(p + ".rightX.calib", calRX)
                .addData(p + ".rightX.out", outRX);

        dbg.addData(p + ".rightY.raw", rawRY)
                .addData(p + ".rightY.center", rightYCenter)
                .addData(p + ".rightY.calib", calRY)
                .addData(p + ".rightY.out", outRY);

        dbg.addData(p + ".leftStick.mag", Math.hypot(outLX, outLY));
        dbg.addData(p + ".rightStick.mag", Math.hypot(outRX, outRY));

        dbg.addData(p + ".leftTrigger.raw", rawLT)
                .addData(p + ".leftTrigger.center", leftTriggerCenter)
                .addData(p + ".leftTrigger.calib", calLT)
                .addData(p + ".leftTrigger.out", outLT);

        dbg.addData(p + ".rightTrigger.raw", rawRT)
                .addData(p + ".rightTrigger.center", rightTriggerCenter)
                .addData(p + ".rightTrigger.calib", calRT)
                .addData(p + ".rightTrigger.out", outRT);
    }
}