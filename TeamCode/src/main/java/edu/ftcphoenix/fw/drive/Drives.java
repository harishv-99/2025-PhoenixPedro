package edu.ftcphoenix.fw.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

import edu.ftcphoenix.fw.adapters.ftc.FtcHardware;
import edu.ftcphoenix.fw.hal.PowerOutput;

/**
 * High-level helpers for creating common drivebases.
 *
 * <h2>Beginner entrypoint for mecanum drive</h2>
 *
 * <p>For most teams, this class should be the <b>main way</b> to construct
 * a {@link MecanumDrivebase}. In beginner TeleOp code, you will usually see
 * just a single line:</p>
 *
 * <pre>{@code
 * MecanumDrivebase drive = Drives.mecanum(hardwareMap);
 * }</pre>
 *
 * <p>This call:</p>
 * <ul>
 *   <li>Assumes the <b>standard motor names</b>:
 *       <ul>
 *         <li>{@link #DEFAULT_FRONT_LEFT_MOTOR_NAME}</li>
 *         <li>{@link #DEFAULT_FRONT_RIGHT_MOTOR_NAME}</li>
 *         <li>{@link #DEFAULT_BACK_LEFT_MOTOR_NAME}</li>
 *         <li>{@link #DEFAULT_BACK_RIGHT_MOTOR_NAME}</li>
 *       </ul>
 *   </li>
 *   <li>Applies a sensible default inversion pattern (typically right side
 *       inverted).</li>
 *   <li>Uses {@link MecanumConfig#defaults()} for all drive tuning.</li>
 * </ul>
 *
 * <p>If the robot does not drive correctly with this default setup, teams
 * can <b>flip motors</b> without touching any low-level details:</p>
 *
 * <pre>{@code
 * // Same standard names, but custom inversion:
 * MecanumDrivebase drive = Drives.mecanum(
 *         hardwareMap,
 *         true,  // invert frontLeftMotor
 *         true,  // invert frontRightMotor
 *         true,  // invert backLeftMotor
 *         false  // invert backRightMotor
 * );
 * }</pre>
 *
 * <p>More advanced teams can:</p>
 *
 * <ul>
 *   <li>Pass a custom {@link MecanumConfig} to enable rate limiting or other
 *       tuning options.</li>
 *   <li>Use the overloads that accept custom motor names if they do not
 *       follow the standard naming convention.</li>
 *   <li>Bypass this helper entirely and construct {@link MecanumDrivebase}
 *       directly with {@link FtcHardware#motorPower} and a custom config.</li>
 * </ul>
 *
 * <p>However, for teaching and most examples, <b>prefer using
 * {@link #mecanum(HardwareMap)} or the simple overloads here</b>. This keeps
 * robot code focused on behavior (how the robot should move) instead of
 * wiring details.</p>
 */
public final class Drives {

    private Drives() {
        // utility class; no instances
    }

    // ======================================================================
    // Standard mecanum motor name constants
    // ======================================================================

    /**
     * Default front-left motor name used by {@link #mecanum(HardwareMap)} helpers.
     */
    public static final String DEFAULT_FRONT_LEFT_MOTOR_NAME = "frontLeftMotor";

    /**
     * Default front-right motor name used by {@link #mecanum(HardwareMap)} helpers.
     */
    public static final String DEFAULT_FRONT_RIGHT_MOTOR_NAME = "frontRightMotor";

    /**
     * Default back-left motor name used by {@link #mecanum(HardwareMap)} helpers.
     */
    public static final String DEFAULT_BACK_LEFT_MOTOR_NAME = "backLeftMotor";

    /**
     * Default back-right motor name used by {@link #mecanum(HardwareMap)} helpers.
     */
    public static final String DEFAULT_BACK_RIGHT_MOTOR_NAME = "backRightMotor";

    // ======================================================================
    // Mecanum drive helpers
    // ======================================================================

    /**
     * Create a mecanum drivebase using the <b>standard motor names</b> and a
     * typical inversion pattern, with {@link MecanumConfig#defaults()}.
     *
     * <ul>
     *   <li>{@link #DEFAULT_FRONT_LEFT_MOTOR_NAME}:  not inverted</li>
     *   <li>{@link #DEFAULT_FRONT_RIGHT_MOTOR_NAME}: inverted</li>
     *   <li>{@link #DEFAULT_BACK_LEFT_MOTOR_NAME}:   not inverted</li>
     *   <li>{@link #DEFAULT_BACK_RIGHT_MOTOR_NAME}:  inverted</li>
     * </ul>
     *
     * <p>If your robot does not drive correctly with this default pattern,
     * use {@link #mecanum(HardwareMap, boolean, boolean, boolean, boolean)}
     * and flip the booleans as needed.</p>
     *
     * @param hw FTC hardware map
     */
    public static MecanumDrivebase mecanum(HardwareMap hw) {
        return mecanum(hw,
                false,  // frontLeftMotor inverted?
                true,   // frontRightMotor inverted?
                false,  // backLeftMotor inverted?
                true,   // backRightMotor inverted?
                MecanumConfig.defaults());
    }

    /**
     * Create a mecanum drivebase using the <b>standard motor names</b>,
     * but with explicit inversion flags for each motor, and
     * {@link MecanumConfig#defaults()}.
     *
     * <p>This is the simplest way for students to "flip" motors while
     * still following the standard naming convention.</p>
     *
     * @param hw               FTC hardware map
     * @param invertFrontLeft  invert {@link #DEFAULT_FRONT_LEFT_MOTOR_NAME}?
     * @param invertFrontRight invert {@link #DEFAULT_FRONT_RIGHT_MOTOR_NAME}?
     * @param invertBackLeft   invert {@link #DEFAULT_BACK_LEFT_MOTOR_NAME}?
     * @param invertBackRight  invert {@link #DEFAULT_BACK_RIGHT_MOTOR_NAME}?
     */
    public static MecanumDrivebase mecanum(HardwareMap hw,
                                           boolean invertFrontLeft,
                                           boolean invertFrontRight,
                                           boolean invertBackLeft,
                                           boolean invertBackRight) {
        return mecanum(hw,
                invertFrontLeft,
                invertFrontRight,
                invertBackLeft,
                invertBackRight,
                MecanumConfig.defaults());
    }

    /**
     * Create a mecanum drivebase with <b>custom motor names</b> and inversion,
     * using {@link MecanumConfig#defaults()}.
     *
     * <p>Use this overload if your motor names do not match the standard
     * {@link #DEFAULT_FRONT_LEFT_MOTOR_NAME},
     * {@link #DEFAULT_FRONT_RIGHT_MOTOR_NAME},
     * {@link #DEFAULT_BACK_LEFT_MOTOR_NAME},
     * {@link #DEFAULT_BACK_RIGHT_MOTOR_NAME} convention.</p>
     */
    public static MecanumDrivebase mecanum(HardwareMap hw,
                                           String flName, boolean flInverted,
                                           String frName, boolean frInverted,
                                           String blName, boolean blInverted,
                                           String brName, boolean brInverted) {
        return mecanum(hw,
                flName, flInverted,
                frName, frInverted,
                blName, blInverted,
                brName, brInverted,
                MecanumConfig.defaults());
    }

    // ------------------------------------------------------------------
    // Overloads that accept a custom MecanumConfig (for rate limiting, etc.)
    // ------------------------------------------------------------------

    /**
     * Create a mecanum drivebase using standard names, default inversion,
     * and a custom {@link MecanumConfig}.
     *
     * <p>Use this when you want to adjust things like rate limiting but are
     * happy with the default motor naming convention.</p>
     */
    public static MecanumDrivebase mecanum(HardwareMap hw,
                                           MecanumConfig config) {
        return mecanum(hw,
                false,  // frontLeftMotor inverted?
                true,   // frontRightMotor inverted?
                false,  // backLeftMotor inverted?
                true,   // backRightMotor inverted?
                config);
    }

    /**
     * Create a mecanum drivebase using standard names, explicit inversion
     * flags, and a custom {@link MecanumConfig}.
     *
     * <p>This is the "flip a few booleans and tweak rate limiting" entrypoint
     * for students whose robots don't behave with the defaults.</p>
     */
    public static MecanumDrivebase mecanum(HardwareMap hw,
                                           boolean invertFrontLeft,
                                           boolean invertFrontRight,
                                           boolean invertBackLeft,
                                           boolean invertBackRight,
                                           MecanumConfig config) {
        return mecanum(hw,
                DEFAULT_FRONT_LEFT_MOTOR_NAME, invertFrontLeft,
                DEFAULT_FRONT_RIGHT_MOTOR_NAME, invertFrontRight,
                DEFAULT_BACK_LEFT_MOTOR_NAME, invertBackLeft,
                DEFAULT_BACK_RIGHT_MOTOR_NAME, invertBackRight,
                config);
    }

    /**
     * Create a mecanum drivebase with custom names, inversion, and config.
     *
     * <p>This is the most flexible overload; advanced teams can use it to
     * plug in any combination of hardware naming and {@link MecanumConfig}
     * tuning.</p>
     */
    public static MecanumDrivebase mecanum(HardwareMap hw,
                                           String flName, boolean flInverted,
                                           String frName, boolean frInverted,
                                           String blName, boolean blInverted,
                                           String brName, boolean brInverted,
                                           MecanumConfig config) {
        PowerOutput fl = FtcHardware.motorPower(hw, flName, flInverted);
        PowerOutput fr = FtcHardware.motorPower(hw, frName, frInverted);
        PowerOutput bl = FtcHardware.motorPower(hw, blName, blInverted);
        PowerOutput br = FtcHardware.motorPower(hw, brName, brInverted);

        MecanumConfig cfg = (config != null) ? config : MecanumConfig.defaults();
        return new MecanumDrivebase(fl, fr, bl, br, cfg);
    }

    // ----------------------------------------------------------------------
    // Other drive helpers can live here as needed.
    // ----------------------------------------------------------------------
}
