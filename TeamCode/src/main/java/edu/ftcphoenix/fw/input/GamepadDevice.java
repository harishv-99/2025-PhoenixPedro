package edu.ftcphoenix.fw.input;

import com.qualcomm.robotcore.hardware.Gamepad;
import edu.ftcphoenix.fw.debug.DebugSink;

/**
 * Thin wrapper around an FTC {@link Gamepad} that exposes:
 * <ul>
 *     <li>Axes ({@link Axis}) for sticks and triggers.</li>
 *     <li>Buttons ({@link Button}) for all digital inputs.</li>
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
 *
 * <h2>Typical usage</h2>
 * <p>
 * You normally do not construct {@code GamepadDevice} yourself. Instead, you use
 * {@link Gamepads} as a manager for both controllers and update it once per loop:
 * </p>
 *
 * <pre>{@code
 * public final class PhoenixRobot {
 *     private final Gamepads pads;
 *     private final Bindings bindings = new Bindings();
 *
 *     public PhoenixRobot(HardwareMap hw, Gamepads pads) {
 *         this.pads = pads;
 *         configureBindings();
 *     }
 *
 *     private void configureBindings() {
 *         GamepadDevice p1 = pads.p1();
 *
 *         // Example: while A is held, run intake forward; stop on release.
 *         bindings.whileHeld(p1.a(), () -> intakePlant.setTarget(+1.0));
 *         bindings.onRelease(p1.a(), () -> intakePlant.setTarget(0.0));
 *     }
 *
 *     public void updateTeleOp(LoopClock clock) {
 *         double dt = clock.dtSec();
 *
 *         // In your OpMode loop:
 *         //   pads.update(dt);
 *         //   bindings.update(dt);
 *         //
 *         // You can read stick axes directly if needed:
 *         //
 *         // GamepadDevice p1 = pads.p1();
 *         // double forward = p1.leftY().get();   // +1.0 when stick is pushed up
 *         // double strafe  = p1.leftX().get();   // +1.0 to the right
 *         // double turn    = p1.rightX().get();  // +1.0 to the right
 *     }
 * }
 *
 * // OpMode skeleton:
 * public class MyTeleOp extends OpMode {
 *     private final LoopClock clock = new LoopClock();
 *     private Gamepads pads;
 *     private PhoenixRobot robot;
 *
 *     @Override
 *     public void init() {
 *         pads = Gamepads.create(gamepad1, gamepad2);
 *         robot = new PhoenixRobot(hardwareMap, pads);
 *         clock.reset(getRuntime());
 *     }
 *
 *     @Override
 *     public void loop() {
 *         clock.update(getRuntime());
 *         double dt = clock.dtSec();
 *
 *         pads.update(dt);
 *         robot.bindings().update(dt);
 *         robot.updateTeleOp(clock);
 *     }
 * }
 * }</pre>
 *
 * <p>
 * The intent is that all raw controller access goes through {@link Gamepads} /
 * {@code GamepadDevice}, and all "which button does what" logic lives in a
 * single {@code configureBindings()} method on your robot.
 * </p>
 */
public final class GamepadDevice {
    /**
     * Default deadband applied to all axes after calibration. Values whose absolute
     * magnitude is below this threshold are treated as exactly 0.0.
     */
    public static final double DEFAULT_AXIS_DEADBAND = 0.02;

    private static final double MIN_SCALE = 1e-3; // protect against division by zero

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

    // Buttons (add/keep whatever you already expose here)
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
    // ... other buttons as in your existing file ...

    public GamepadDevice(Gamepad gp) {
        this.gp = gp;

        // Axes: build them in terms of raw + calibration + deadband.
        // X passes through, Y is inverted so that "up" is positive.
        this.leftX  = Axis.of(() -> applyDeadband(calibratedStick(rawLeftX(),  leftXCenter)));
        this.leftY  = Axis.of(() -> applyDeadband(calibratedStick(rawLeftY(),  leftYCenter)));
        this.rightX = Axis.of(() -> applyDeadband(calibratedStick(rawRightX(), rightXCenter)));
        this.rightY = Axis.of(() -> applyDeadband(calibratedStick(rawRightY(), rightYCenter)));

        this.leftTrigger  = Axis.of(() -> applyDeadband(calibratedTrigger(rawLeftTrigger(),  leftTriggerCenter)));
        this.rightTrigger = Axis.of(() -> applyDeadband(calibratedTrigger(rawRightTrigger(), rightTriggerCenter)));

        // Buttons – same mapping as before.
        this.a = Button.of(() -> gp.a);
        this.b = Button.of(() -> gp.b);
        this.x = Button.of(() -> gp.x);
        this.y = Button.of(() -> gp.y);

        this.leftBumper  = Button.of(() -> gp.left_bumper);
        this.rightBumper = Button.of(() -> gp.right_bumper);

        this.dpadUp    = Button.of(() -> gp.dpad_up);
        this.dpadDown  = Button.of(() -> gp.dpad_down);
        this.dpadLeft  = Button.of(() -> gp.dpad_left);
        this.dpadRight = Button.of(() -> gp.dpad_right);

        // ... initialize the rest of your buttons exactly as before ...

        // Automatically treat the current stick/trigger positions as neutral and
        // configure scaling so we still reach the full logical range.
        calibrate();
    }

    // --- Raw axis helpers in human-friendly coordinates ---

    /**
     * Raw left stick X in human-friendly coordinates: -1.0 = left, +1.0 = right.
     */
    private double rawLeftX() {
        return gp.left_stick_x;
    }

    /**
     * Raw left stick Y in human-friendly coordinates: -1.0 = down, +1.0 = up.
     * The FTC SDK reports up as negative, so we invert here.
     */
    private double rawLeftY() {
        return -gp.left_stick_y;
    }

    /**
     * Raw right stick X in human-friendly coordinates: -1.0 = left, +1.0 = right.
     */
    private double rawRightX() {
        return gp.right_stick_x;
    }

    /**
     * Raw right stick Y in human-friendly coordinates: -1.0 = down, +1.0 = up.
     */
    private double rawRightY() {
        return -gp.right_stick_y;
    }

    /**
     * Raw left trigger in human-friendly coordinates: 0.0 = released, 1.0 = fully pressed.
     */
    private double rawLeftTrigger() {
        return gp.left_trigger;
    }

    /**
     * Raw right trigger in human-friendly coordinates: 0.0 = released, 1.0 = fully pressed.
     */
    private double rawRightTrigger() {
        return gp.right_trigger;
    }

    // --- Calibration math ---

    /**
     * Calibrates a stick axis (range [-1, +1]) given the current raw value and
     * stored center.
     *
     * <p>
     * We do a piecewise linear rescaling so that:
     * </p>
     * <ul>
     *     <li>{@code raw == center} maps to 0.0</li>
     *     <li>{@code raw == +1.0} maps to +1.0</li>
     *     <li>{@code raw == -1.0} maps to -1.0</li>
     * </ul>
     *
     * <p>
     * Positive and negative sides can have slightly different scale factors if
     * the center is not exactly 0.0, but the mapping is continuous at the center.
     * </p>
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

        // Clamp to [-1, +1] in case of slight overshoot or odd gamepad behaviour.
        if (value > 1.0) value = 1.0;
        if (value < -1.0) value = -1.0;
        return value;
    }

    /**
     * Calibrates a trigger axis (range [0, +1]) given the current raw value and
     * stored center.
     *
     * <p>
     * We treat {@code center} as the logical 0.0 and rescale so that:
     * </p>
     * <ul>
     *     <li>{@code raw == center} maps to 0.0</li>
     *     <li>{@code raw == +1.0} maps to +1.0</li>
     * </ul>
     *
     * <p>
     * Values below the center (which can occur due to noise) are clamped at 0.0.
     * </p>
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
     * <p>
     * For triggers (which are non-negative), this simply means very small values
     * near zero are forced to 0.0.
     * </p>
     */
    private double applyDeadband(double value) {
        return (Math.abs(value) < axisDeadband) ? 0.0 : value;
    }

    // --- Public API: axes ---

    /** Left stick X axis: -1.0 = full left, +1.0 = full right. */
    public Axis leftX() {
        return leftX;
    }

    /**
     * Left stick Y axis: -1.0 = full down, +1.0 = full up.
     *
     * <p>This inverts the raw FTC {@link Gamepad#left_stick_y} (which is
     * negative when pushed up) so that pushing the stick up yields a
     * <b>positive</b> value and pushing it down yields a <b>negative</b> value.
     * The reported value is then:
     * </p>
     * <ul>
     *     <li>recentred using the current calibration offset (set by {@link #calibrate()}),</li>
     *     <li>rescaled so that full travel still maps to [-1, +1], and</li>
     *     <li>filtered through the current deadband.</li>
     * </ul>
     */
    public Axis leftY() {
        return leftY;
    }

    /** Right stick X axis: -1.0 = full left, +1.0 = full right. */
    public Axis rightX() {
        return rightX;
    }

    /**
     * Right stick Y axis: -1.0 = full down, +1.0 = full up.
     *
     * <p>This inverts the raw FTC {@link Gamepad#right_stick_y} (which is
     * negative when pushed up) so that pushing the stick up yields a
     * <b>positive</b> value and pushing it down yields a <b>negative</b> value.
     * The reported value is then:
     * </p>
     * <ul>
     *     <li>recentred using the current calibration offset (set by {@link #calibrate()}),</li>
     *     <li>rescaled so that full travel still maps to [-1, +1], and</li>
     *     <li>filtered through the current deadband.</li>
     * </ul>
     */
    public Axis rightY() {
        return rightY;
    }

    /** Left trigger axis: 0.0 = released, 1.0 = fully pressed (after calibration and deadband). */
    public Axis leftTrigger() {
        return leftTrigger;
    }

    /** Right trigger axis: 0.0 = released, 1.0 = fully pressed (after calibration and deadband). */
    public Axis rightTrigger() {
        return rightTrigger;
    }

    // --- Public API: buttons ---

    public Button a() { return a; }
    public Button b() { return b; }
    public Button x() { return x; }
    public Button y() { return y; }

    public Button leftBumper() { return leftBumper; }
    public Button rightBumper() { return rightBumper; }

    public Button dpadUp() { return dpadUp; }
    public Button dpadDown() { return dpadDown; }
    public Button dpadLeft() { return dpadLeft; }
    public Button dpadRight() { return dpadRight; }

    // ... expose the rest of your buttons as in the existing implementation ...

    // --- Calibration / deadband configuration ---

    /**
     * Calibrates all analog axes (sticks and triggers) by treating their <b>current</b>
     * positions as neutral.
     *
     * <p>
     * This method is called once automatically from the constructor, so in the
     * typical case users do not need to call it themselves. It remains public so
     * you can re-calibrate later (for example, if a gamepad starts to drift during
     * a match or you suspect it was bumped during {@code init()}).
     * </p>
     *
     * <p>
     * Internally this method:
     * </p>
     * <ul>
     *     <li>Reads the current raw values for left/right sticks and triggers,
     *     after converting them into human-friendly coordinates.</li>
     *     <li>Saves those readings as per-axis "center" offsets.</li>
     *     <li>Subsequent calls to {@link Axis#get()} for these axes will:
     *         <ul>
     *             <li>subtract this center value,</li>
     *             <li>rescale so that full travel still maps to the full logical range, and</li>
     *             <li>apply the configured deadband.</li>
     *         </ul>
     *     </li>
     * </ul>
     */
    public void calibrate() {
        leftXCenter        = rawLeftX();
        leftYCenter        = rawLeftY();
        rightXCenter       = rawRightX();
        rightYCenter       = rawRightY();
        leftTriggerCenter  = rawLeftTrigger();
        rightTriggerCenter = rawRightTrigger();
    }

    /**
     * Sets the deadband used for all axes.
     *
     * <p>
     * Any corrected axis value (after subtracting the calibration offset and rescaling)
     * whose absolute magnitude is below this threshold will be reported as exactly 0.0.
     * Use a slightly larger deadband if your gamepad is noisy around center.
     * </p>
     *
     * @param deadband new deadband threshold in the range [0.0, 1.0].
     */
    public void setAxisDeadband(double deadband) {
        this.axisDeadband = Math.max(0.0, Math.min(1.0, deadband));
    }

    /**
     * Returns the current axis deadband.
     */
    public double axisDeadband() {
        return axisDeadband;
    }

    // --- Debugging ---

    /**
     * Emits diagnostic information about the current state of this
     * {@code GamepadDevice} into the provided {@link DebugSink}.
     *
     * <p>
     * The dump includes, for each axis:
     * </p>
     * <ul>
     *     <li>Raw stick and trigger readings in human-friendly coordinates.</li>
     *     <li>Current center offsets used for calibration.</li>
     *     <li>Calibrated values before deadband.</li>
     *     <li>Final values after deadband.</li>
     *     <li>The current deadband setting.</li>
     * </ul>
     *
     * <p>
     * This is intended for logging or telemetry when debugging controller issues
     * such as drift, asymmetry, or unexpected scaling.
     * </p>
     *
     * @param dbg    debug sink (may be {@code null}; if null, no output is produced)
     * @param prefix base key prefix, e.g. {@code "p1.gamepad"}
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "gamepad" : prefix;

        double rawLX  = rawLeftX();
        double rawLY  = rawLeftY();
        double rawRX  = rawRightX();
        double rawRY  = rawRightY();
        double rawLT  = rawLeftTrigger();
        double rawRT  = rawRightTrigger();

        double calLX  = calibratedStick(rawLX, leftXCenter);
        double calLY  = calibratedStick(rawLY, leftYCenter);
        double calRX  = calibratedStick(rawRX, rightXCenter);
        double calRY  = calibratedStick(rawRY, rightYCenter);
        double calLT  = calibratedTrigger(rawLT, leftTriggerCenter);
        double calRT  = calibratedTrigger(rawRT, rightTriggerCenter);

        double outLX  = applyDeadband(calLX);
        double outLY  = applyDeadband(calLY);
        double outRX  = applyDeadband(calRX);
        double outRY  = applyDeadband(calRY);
        double outLT  = applyDeadband(calLT);
        double outRT  = applyDeadband(calRT);

        dbg.addLine(p + ": GamepadDevice");

        dbg.addData(p + ".deadband", axisDeadband);

        // Left stick X
        dbg.addData(p + ".leftX.raw", rawLX)
                .addData(p + ".leftX.center", leftXCenter)
                .addData(p + ".leftX.calib", calLX)
                .addData(p + ".leftX.out", outLX);

        // Left stick Y
        dbg.addData(p + ".leftY.raw", rawLY)
                .addData(p + ".leftY.center", leftYCenter)
                .addData(p + ".leftY.calib", calLY)
                .addData(p + ".leftY.out", outLY);

        // Right stick X
        dbg.addData(p + ".rightX.raw", rawRX)
                .addData(p + ".rightX.center", rightXCenter)
                .addData(p + ".rightX.calib", calRX)
                .addData(p + ".rightX.out", outRX);

        // Right stick Y
        dbg.addData(p + ".rightY.raw", rawRY)
                .addData(p + ".rightY.center", rightYCenter)
                .addData(p + ".rightY.calib", calRY)
                .addData(p + ".rightY.out", outRY);

        // Left trigger
        dbg.addData(p + ".leftTrigger.raw", rawLT)
                .addData(p + ".leftTrigger.center", leftTriggerCenter)
                .addData(p + ".leftTrigger.calib", calLT)
                .addData(p + ".leftTrigger.out", outLT);

        // Right trigger
        dbg.addData(p + ".rightTrigger.raw", rawRT)
                .addData(p + ".rightTrigger.center", rightTriggerCenter)
                .addData(p + ".rightTrigger.calib", calRT)
                .addData(p + ".rightTrigger.out", outRT);
    }
}
