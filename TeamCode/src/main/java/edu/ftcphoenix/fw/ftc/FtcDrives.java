package edu.ftcphoenix.fw.ftc;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.drive.MecanumDrivebase;

/**
 * FTC-boundary helpers for creating and configuring common drivebases.
 *
 * <p>
 * This class lives in {@code fw.ftc} on purpose: any API that touches FTC SDK types like
 * {@link HardwareMap} or {@link DcMotor} belongs in the FTC boundary layer.
 * The resulting {@link MecanumDrivebase} is framework-generic and lives in {@code fw.drive}.
 * </p>
 *
 * <h2>Beginner entrypoint for mecanum drive</h2>
 *
 * <p>For most teams, this class should be the <b>main way</b> to construct
 * a {@link MecanumDrivebase}. In beginner TeleOp code, you will usually see
 * just a single line:</p>
 *
 * <pre>{@code
 * MecanumDrivebase drive = FtcDrives.mecanum(hardwareMap);
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
 *   <li>Applies a sensible default direction pattern (typically right side
 *       {@link Direction#REVERSE}).</li>
 *   <li>Uses {@link MecanumDrivebase.Config#defaults()} for all drive tuning.</li>
 * </ul>
 *
 * <p>If the robot does not drive correctly with this default setup, teams
 * can <b>flip motors</b> without touching any low-level details by changing
 * the per-motor {@link Direction} values:</p>
 *
 * <pre>{@code
 * // Same standard names, but custom directions:
 * MecanumDrivebase drive = FtcDrives.mecanum(
 *         hardwareMap,
 *         Direction.REVERSE,  // frontLeftMotor direction
 *         Direction.REVERSE,  // frontRightMotor direction
 *         Direction.FORWARD,  // backLeftMotor direction
 *         Direction.REVERSE   // backRightMotor direction
 * );
 * }</pre>
 *
 * <p>More advanced teams can:</p>
 *
 * <ul>
 *   <li>Pass a custom {@link MecanumDrivebase.Config} to enable rate limiting or other
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
public final class FtcDrives {

    private FtcDrives() {
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


    /**
     * Hardware wiring for a mecanum drivetrain.
     *
     * <p>This bundles motor names and directions into a single object so robot configs and testers
     * can pass wiring around without picking among multiple factory overloads.</p>
     */
    public static final class MecanumWiringConfig {

        /**
         * Motor config name for the front-left motor.
         */
        public String frontLeftName = DEFAULT_FRONT_LEFT_MOTOR_NAME;

        /**
         * Motor config name for the front-right motor.
         */
        public String frontRightName = DEFAULT_FRONT_RIGHT_MOTOR_NAME;

        /**
         * Motor config name for the back-left motor.
         */
        public String backLeftName = DEFAULT_BACK_LEFT_MOTOR_NAME;

        /**
         * Motor config name for the back-right motor.
         */
        public String backRightName = DEFAULT_BACK_RIGHT_MOTOR_NAME;

        /**
         * Logical direction for the front-left motor.
         */
        public Direction frontLeftDirection = Direction.FORWARD;

        /**
         * Logical direction for the front-right motor.
         */
        public Direction frontRightDirection = Direction.REVERSE;

        /**
         * Logical direction for the back-left motor.
         */
        public Direction backLeftDirection = Direction.FORWARD;

        /**
         * Logical direction for the back-right motor.
         */
        public Direction backRightDirection = Direction.REVERSE;

        private MecanumWiringConfig() {
            // Defaults assigned in field initializers.
        }

        /**
         * Returns a new wiring config instance populated with Phoenix's default motor names and directions.
         */
        public static MecanumWiringConfig defaults() {
            return new MecanumWiringConfig();
        }

        /**
         * Deep copy of this wiring config.
         */
        public MecanumWiringConfig copy() {
            MecanumWiringConfig c = new MecanumWiringConfig();
            c.frontLeftName = this.frontLeftName;
            c.frontRightName = this.frontRightName;
            c.backLeftName = this.backLeftName;
            c.backRightName = this.backRightName;
            c.frontLeftDirection = this.frontLeftDirection;
            c.frontRightDirection = this.frontRightDirection;
            c.backLeftDirection = this.backLeftDirection;
            c.backRightDirection = this.backRightDirection;
            return c;
        }
    }

    /**
     * Creates a mecanum drivebase from a {@link MecanumWiringConfig} bundle.
     *
     * <p>This is a convenience overload that uses {@link MecanumDrivebase.Config#defaults()}.</p>
     */
    public static MecanumDrivebase mecanum(HardwareMap hw, MecanumWiringConfig wiring) {
        return mecanum(hw, wiring, MecanumDrivebase.Config.defaults());
    }

    /**
     * Creates a mecanum drivebase from a {@link MecanumWiringConfig} bundle and a drive config.
     *
     * @param hw     hardware map
     * @param wiring motor names/directions
     * @param config mecanum drive config (if null, {@link MecanumDrivebase.Config#defaults()} is used)
     */
    public static MecanumDrivebase mecanum(HardwareMap hw, MecanumWiringConfig wiring, MecanumDrivebase.Config config) {
        if (wiring == null) {
            throw new IllegalArgumentException("wiring is required");
        }
        return mecanum(
                hw,
                wiring.frontLeftName,
                wiring.frontLeftDirection,
                wiring.frontRightName,
                wiring.frontRightDirection,
                wiring.backLeftName,
                wiring.backLeftDirection,
                wiring.backRightName,
                wiring.backRightDirection,
                config
        );
    }

    // ======================================================================
    // Motor behavior helpers (FTC-specific)
    // ======================================================================

    /**
     * Set {@link DcMotor.ZeroPowerBehavior} on the four drivetrain motors described by {@code wiring}.
     *
     * <p>
     * This is intentionally kept in {@code fw.ftc} so robot code does not need to talk to
     * FTC SDK motor objects directly.
     * </p>
     *
     * @param hw       FTC hardware map
     * @param wiring   mecanum wiring bundle (motor names)
     * @param behavior desired behavior when commanded power is zero
     */
    public static void setZeroPowerBehavior(HardwareMap hw, MecanumWiringConfig wiring, DcMotor.ZeroPowerBehavior behavior) {
        if (hw == null) {
            throw new IllegalArgumentException("HardwareMap is required");
        }
        if (wiring == null) {
            throw new IllegalArgumentException("wiring is required");
        }
        if (behavior == null) {
            throw new IllegalArgumentException("behavior is required");
        }

        hw.get(DcMotor.class, wiring.frontLeftName).setZeroPowerBehavior(behavior);
        hw.get(DcMotor.class, wiring.frontRightName).setZeroPowerBehavior(behavior);
        hw.get(DcMotor.class, wiring.backLeftName).setZeroPowerBehavior(behavior);
        hw.get(DcMotor.class, wiring.backRightName).setZeroPowerBehavior(behavior);
    }

    /**
     * Convenience helper: set drivetrain motors to BRAKE (if {@code brake} is true) or FLOAT.
     */
    public static void setDriveBrake(HardwareMap hw, MecanumWiringConfig wiring, boolean brake) {
        setZeroPowerBehavior(
                hw,
                wiring,
                brake ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT
        );
    }

    /**
     * Convenience helper using {@link MecanumWiringConfig#defaults()}.
     */
    public static void setDriveBrake(HardwareMap hw, boolean brake) {
        setDriveBrake(hw, MecanumWiringConfig.defaults(), brake);
    }

    // ======================================================================
    // Mecanum drive helpers
    // ======================================================================

    /**
     * Create a mecanum drivebase using the <b>standard motor names</b> and a typical direction pattern.
     *
     * <p>This overload uses {@link MecanumDrivebase.Config#defaults()} and the conventional
     * left/right direction pairing:</p>
     *
     * <ul>
     *   <li>{@link #DEFAULT_FRONT_LEFT_MOTOR_NAME}:  {@link Direction#FORWARD}</li>
     *   <li>{@link #DEFAULT_FRONT_RIGHT_MOTOR_NAME}: {@link Direction#REVERSE}</li>
     *   <li>{@link #DEFAULT_BACK_LEFT_MOTOR_NAME}:   {@link Direction#FORWARD}</li>
     *   <li>{@link #DEFAULT_BACK_RIGHT_MOTOR_NAME}:  {@link Direction#REVERSE}</li>
     * </ul>
     *
     * @param hw FTC {@link HardwareMap} used to look up configured motors
     * @return a new {@link MecanumDrivebase} wired to the four configured motors
     * @throws IllegalArgumentException if {@code hw} is {@code null}
     */
    public static MecanumDrivebase mecanum(HardwareMap hw) {
        return mecanum(
                hw,
                Direction.FORWARD,
                Direction.REVERSE,
                Direction.FORWARD,
                Direction.REVERSE,
                MecanumDrivebase.Config.defaults()
        );
    }

    /**
     * Create a mecanum drivebase using the <b>standard motor names</b>, but explicit directions.
     *
     * <p>This overload still uses {@link MecanumDrivebase.Config#defaults()}.</p>
     *
     * @param hw                  FTC {@link HardwareMap} used to look up configured motors
     * @param frontLeftDirection  logical direction for {@link #DEFAULT_FRONT_LEFT_MOTOR_NAME}
     * @param frontRightDirection logical direction for {@link #DEFAULT_FRONT_RIGHT_MOTOR_NAME}
     * @param backLeftDirection   logical direction for {@link #DEFAULT_BACK_LEFT_MOTOR_NAME}
     * @param backRightDirection  logical direction for {@link #DEFAULT_BACK_RIGHT_MOTOR_NAME}
     * @return a new {@link MecanumDrivebase} wired to the four configured motors
     * @throws IllegalArgumentException if any argument is {@code null}
     */
    public static MecanumDrivebase mecanum(HardwareMap hw,
                                           Direction frontLeftDirection,
                                           Direction frontRightDirection,
                                           Direction backLeftDirection,
                                           Direction backRightDirection) {
        return mecanum(
                hw,
                frontLeftDirection,
                frontRightDirection,
                backLeftDirection,
                backRightDirection,
                MecanumDrivebase.Config.defaults()
        );
    }

    /**
     * Create a mecanum drivebase using <b>custom motor names</b> and directions.
     *
     * <p>This overload uses {@link MecanumDrivebase.Config#defaults()}.</p>
     *
     * @param hw          FTC {@link HardwareMap} used to look up configured motors
     * @param flName      configured device name for the front-left motor
     * @param flDirection logical direction for the front-left motor
     * @param frName      configured device name for the front-right motor
     * @param frDirection logical direction for the front-right motor
     * @param blName      configured device name for the back-left motor
     * @param blDirection logical direction for the back-left motor
     * @param brName      configured device name for the back-right motor
     * @param brDirection logical direction for the back-right motor
     * @return a new {@link MecanumDrivebase} wired to the four configured motors
     * @throws IllegalArgumentException if {@code hw} is {@code null}, any name is {@code null}, or any direction is {@code null}
     */
    public static MecanumDrivebase mecanum(HardwareMap hw,
                                           String flName, Direction flDirection,
                                           String frName, Direction frDirection,
                                           String blName, Direction blDirection,
                                           String brName, Direction brDirection) {
        return mecanum(
                hw,
                flName, flDirection,
                frName, frDirection,
                blName, blDirection,
                brName, brDirection,
                MecanumDrivebase.Config.defaults()
        );
    }

    // ------------------------------------------------------------------
    // Overloads that accept a custom MecanumDrivebase.Config (rate limiting, etc.)
    // ------------------------------------------------------------------

    /**
     * Create a mecanum drivebase using standard motor names, default directions, and a custom config.
     *
     * <p>This is the easiest way to enable optional tuning (for example, drive rate limiting)
     * while still using the standard wiring assumptions.</p>
     *
     * @param hw     FTC {@link HardwareMap} used to look up configured motors
     * @param config configuration/tuning for the drivebase; if {@code null}, defaults are used
     * @return a new {@link MecanumDrivebase} wired to the standard four motors
     * @throws IllegalArgumentException if {@code hw} is {@code null}
     */
    public static MecanumDrivebase mecanum(HardwareMap hw,
                                           MecanumDrivebase.Config config) {
        return mecanum(
                hw,
                Direction.FORWARD,
                Direction.REVERSE,
                Direction.FORWARD,
                Direction.REVERSE,
                config
        );
    }

    /**
     * Create a mecanum drivebase using standard motor names, explicit directions, and a custom config.
     *
     * @param hw                  FTC {@link HardwareMap} used to look up configured motors
     * @param frontLeftDirection  logical direction for {@link #DEFAULT_FRONT_LEFT_MOTOR_NAME}
     * @param frontRightDirection logical direction for {@link #DEFAULT_FRONT_RIGHT_MOTOR_NAME}
     * @param backLeftDirection   logical direction for {@link #DEFAULT_BACK_LEFT_MOTOR_NAME}
     * @param backRightDirection  logical direction for {@link #DEFAULT_BACK_RIGHT_MOTOR_NAME}
     * @param config              configuration/tuning for the drivebase; if {@code null}, defaults are used
     * @return a new {@link MecanumDrivebase} wired to the standard four motors
     * @throws IllegalArgumentException if {@code hw} is {@code null} or any direction is {@code null}
     */
    public static MecanumDrivebase mecanum(HardwareMap hw,
                                           Direction frontLeftDirection,
                                           Direction frontRightDirection,
                                           Direction backLeftDirection,
                                           Direction backRightDirection,
                                           MecanumDrivebase.Config config) {
        return mecanum(
                hw,
                DEFAULT_FRONT_LEFT_MOTOR_NAME, frontLeftDirection,
                DEFAULT_FRONT_RIGHT_MOTOR_NAME, frontRightDirection,
                DEFAULT_BACK_LEFT_MOTOR_NAME, backLeftDirection,
                DEFAULT_BACK_RIGHT_MOTOR_NAME, backRightDirection,
                config
        );
    }

    /**
     * Create a mecanum drivebase with custom motor names, directions, and config.
     *
     * <p>This is the most general mecanum factory in {@link FtcDrives}. It is useful when:</p>
     * <ul>
     *   <li>Your robot uses non-standard motor names in the FTC Robot Configuration.</li>
     *   <li>You want full control over motor directions.</li>
     *   <li>You want to pass a custom {@link MecanumDrivebase.Config} (or {@code null} to use defaults).</li>
     * </ul>
     *
     * @param hw          FTC {@link HardwareMap} used to look up configured motors
     * @param flName      configured device name for the front-left motor
     * @param flDirection logical direction for the front-left motor
     * @param frName      configured device name for the front-right motor
     * @param frDirection logical direction for the front-right motor
     * @param blName      configured device name for the back-left motor
     * @param blDirection logical direction for the back-left motor
     * @param brName      configured device name for the back-right motor
     * @param brDirection logical direction for the back-right motor
     * @param config      configuration/tuning for the drivebase; if {@code null}, defaults are used
     * @return a new {@link MecanumDrivebase}
     * @throws IllegalArgumentException if {@code hw} is {@code null}, any name is {@code null}, or any direction is {@code null}
     */
    public static MecanumDrivebase mecanum(HardwareMap hw,
                                           String flName, Direction flDirection,
                                           String frName, Direction frDirection,
                                           String blName, Direction blDirection,
                                           String brName, Direction brDirection,
                                           MecanumDrivebase.Config config) {
        if (hw == null) {
            throw new IllegalArgumentException("HardwareMap is required");
        }
        if (flName == null || frName == null || blName == null || brName == null) {
            throw new IllegalArgumentException("All motor names are required");
        }
        if (flDirection == null || frDirection == null || blDirection == null || brDirection == null) {
            throw new IllegalArgumentException("All motor directions are required");
        }

        PowerOutput fl = FtcHardware.motorPower(hw, flName, flDirection);
        PowerOutput fr = FtcHardware.motorPower(hw, frName, frDirection);
        PowerOutput bl = FtcHardware.motorPower(hw, blName, blDirection);
        PowerOutput br = FtcHardware.motorPower(hw, brName, brDirection);

        MecanumDrivebase.Config cfg = (config != null) ? config : MecanumDrivebase.Config.defaults();
        return new MecanumDrivebase(fl, fr, bl, br, cfg);
    }

    // ----------------------------------------------------------------------
    // Other drive helpers can live here as needed.
    // ----------------------------------------------------------------------
}
