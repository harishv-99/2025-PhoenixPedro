package edu.ftcphoenix.fw.examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.DriveSource;
import edu.ftcphoenix.fw.drive.Drives;
import edu.ftcphoenix.fw.drive.MecanumDrivebase;
import edu.ftcphoenix.fw.drive.source.GamepadDriveSource;
import edu.ftcphoenix.fw.input.Gamepads;
import edu.ftcphoenix.fw.util.LoopClock;

/**
 * <h1>Example 01: Basic Mecanum TeleOp</h1>
 *
 * <p>This is the <b>first</b> Phoenix example to read and run.</p>
 *
 * <h2>Big ideas</h2>
 *
 * <ul>
 *   <li>Use <b>beginner helpers</b> whenever possible:
 *     <ul>
 *       <li>{@link Drives#mecanum} to wire the drive motors.</li>
 *       <li>{@link Gamepads} to wrap FTC gamepads.</li>
 *       <li>{@link GamepadDriveSource#teleOpMecanumStandard(Gamepads)} for
 *           stick → mecanum mapping.</li>
 *     </ul>
 *   </li>
 *   <li>Follow a simple, consistent pattern inside {@link #loop()}:
 *     <ol>
 *       <li>Update the {@link LoopClock}.</li>
 *       <li>Update inputs (gamepads).</li>
 *       <li>Compute a {@link DriveSignal} from sticks.</li>
 *       <li>Send the command to the {@link MecanumDrivebase} and update it.</li>
 *       <li>Send telemetry.</li>
 *     </ol>
 *   </li>
 * </ul>
 *
 * <h2>What this example does <b>not</b> include (on purpose)</h2>
 *
 * <ul>
 *   <li>No shooter, intake, or other mechanisms.</li>
 *   <li>No tasks or macros – the robot just responds directly to the sticks.</li>
 *   <li>No tuning – we use {@code MecanumConfig.defaults()} internally
 *       through {@link Drives#mecanum}.</li>
 * </ul>
 *
 * <h2>Motor naming convention</h2>
 *
 * <p>{@link Drives#mecanum} assumes these motor names in the FTC
 * Robot Configuration:</p>
 *
 * <ul>
 *   <li>{@link Drives#DEFAULT_FRONT_LEFT_MOTOR_NAME DEFAULT_FRONT_LEFT_MOTOR_NAME}:
 *       {@code "frontLeftMotor"}</li>
 *   <li>{@link Drives#DEFAULT_FRONT_RIGHT_MOTOR_NAME DEFAULT_FRONT_RIGHT_MOTOR_NAME}:
 *       {@code "frontRightMotor"}</li>
 *   <li>{@link Drives#DEFAULT_BACK_LEFT_MOTOR_NAME DEFAULT_BACK_LEFT_MOTOR_NAME}:
 *       {@code "backLeftMotor"}</li>
 *   <li>{@link Drives#DEFAULT_BACK_RIGHT_MOTOR_NAME DEFAULT_BACK_RIGHT_MOTOR_NAME}:
 *       {@code "backRightMotor"}</li>
 * </ul>
 *
 * <p>If your robot moves in a strange direction (for example, strafes the
 * wrong way), you can switch to one of the other
 * {@code Drives.mecanum(...)} overloads and flip the inversion booleans
 * without touching low-level motor code.</p>
 *
 * <h2>Controls (teleOpMecanumStandard)</h2>
 *
 * <ul>
 *   <li>P1 left stick X: strafe left/right (lateral)</li>
 *   <li>P1 left stick Y: drive forward/back (axial)</li>
 *   <li>P1 right stick X: rotate left/right (omega)</li>
 *   <li>P1 right bumper: slow mode (reduced speed for precision)</li>
 * </ul>
 *
 * <h2>Reading order for examples</h2>
 *
 * <p>After this file, look at:</p>
 * <ol>
 *   <li><b>Example 02: ShooterBasic</b> – adds a shooter + transfer + pusher
 *       using {@code Actuators} and {@code Plants} but keeps the same
 *       {@link #loop()} structure.</li>
 *   <li>Later examples add macros, interpolation tables, and autonomous
 *       on top of this pattern.</li>
 * </ol>
 */
@TeleOp(name = "FW Ex 01: Mecanum Basic", group = "Framework Examples")
@Disabled
public final class TeleOp_01_MecanumBasic extends OpMode {

    // ----------------------------------------------------------------------
    // Framework plumbing
    // ----------------------------------------------------------------------

    /**
     * Tracks time between loops.
     */
    private final LoopClock clock = new LoopClock();

    /**
     * Wrapper around FTC gamepad1 / gamepad2.
     */
    private Gamepads gamepads;

    /**
     * Our mecanum drivebase, created via {@link Drives#mecanum}.
     */
    private MecanumDrivebase drivebase;

    /**
     * Converts sticks → {@link DriveSignal} using a standard TeleOp mapping.
     */
    private DriveSource stickDrive;

    /**
     * Last commanded drive signal, for telemetry.
     */
    private DriveSignal lastDrive = new DriveSignal(0.0, 0.0, 0.0);

    // ----------------------------------------------------------------------
    // OpMode lifecycle
    // ----------------------------------------------------------------------

    @Override
    public void init() {
        // 1) Wrap FTC gamepads.
        gamepads = Gamepads.create(gamepad1, gamepad2);

        // 2) Create a mecanum drivebase using the beginner-friendly helper.
        //
        // This single line:
        //   - Uses default motor names (frontLeftMotor, frontRightMotor, ...).
        //   - Applies a standard inversion pattern.
        //   - Uses the default MecanumConfig.
        drivebase = Drives.mecanum(hardwareMap);

        // 3) Use the standard TeleOp stick mapping for mecanum.
        stickDrive = GamepadDriveSource.teleOpMecanumStandard(gamepads);

        telemetry.addLine("FW Example 01: Mecanum Basic");
        telemetry.addLine("Left stick: drive, Right stick: turn, RB: slow mode");
        telemetry.update();
    }

    @Override
    public void start() {
        // Reset the clock to the current runtime when the OpMode starts.
        clock.reset(getRuntime());
    }

    @Override
    public void loop() {
        // --- 1) Clock ---
        clock.update(getRuntime());
        double dtSec = clock.dtSec();

        // --- 2) Inputs ---
        // In later examples, we may also read sensors here.
        gamepads.update(dtSec);

        // --- 3) Logic: sticks → drive signal ---
        DriveSignal cmd = stickDrive.get(clock).clamped();
        lastDrive = cmd;

        // --- 4) Actuation: send to drivebase and update it ---
        drivebase.drive(cmd);
        drivebase.update(clock);

        // --- 5) Telemetry ---
        telemetry.addLine("FW Example 01: Mecanum Basic");
        telemetry.addLine("Drive (axial / lateral / omega)")
                .addData("axial", lastDrive.axial)
                .addData("lateral", lastDrive.lateral)
                .addData("omega", lastDrive.omega);
        telemetry.update();
    }

    @Override
    public void stop() {
        drivebase.stop();
    }
}
