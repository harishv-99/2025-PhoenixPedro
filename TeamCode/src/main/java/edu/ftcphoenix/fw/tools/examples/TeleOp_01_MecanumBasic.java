package edu.ftcphoenix.fw.tools.examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.debug.NullDebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.DriveSource;
import edu.ftcphoenix.fw.drive.MecanumDrivebase;
import edu.ftcphoenix.fw.drive.source.GamepadDriveSource;
import edu.ftcphoenix.fw.ftc.FtcDrives;
import edu.ftcphoenix.fw.ftc.FtcTelemetryDebugSink;
import edu.ftcphoenix.fw.input.Gamepads;

/**
 * <h1>Example 01: Basic Mecanum TeleOp</h1>
 *
 * <p>Shows how to wire a mecanum drivebase and read sticks using Phoenix helpers.</p>
 *
 * <ul>
 *   <li><b>Goal:</b> Keep the loop simple and non-blocking.</li>
 *   <li><b>Pattern:</b>
 *     <ol>
 *       <li>Update the {@link LoopClock}.</li>
 *       <li>Update inputs (gamepads).</li>
 *       <li>Compute a {@link DriveSignal} from sticks.</li>
 *       <li>Update the {@link MecanumDrivebase} and send the command.</li>
 *       <li>Send telemetry.</li>
 *     </ol>
 *   </li>
 * </ul>
 */
@TeleOp(name = "FW Example 01: Mecanum Basic", group = "FW Examples")
@Disabled
public final class TeleOp_01_MecanumBasic extends OpMode {

    private final LoopClock clock = new LoopClock();

    // Debug/telemetry adapter (DebugSink -> FTC Telemetry)
    private static final boolean DEBUG = true;
    private DebugSink dbg = NullDebugSink.INSTANCE;

    private Gamepads gamepads;

    private MecanumDrivebase drivebase;
    private DriveSource stickDrive;

    private DriveSignal lastDrive = DriveSignal.zero();

    /**
     * {@inheritDoc}
     */
    @Override
    public void init() {
        dbg = DEBUG ? new FtcTelemetryDebugSink(telemetry) : NullDebugSink.INSTANCE;

        clock.reset(getRuntime());

        gamepads = Gamepads.create(gamepad1, gamepad2);

        // Beginner-friendly mecanum wiring (uses default motor names and inversions).
        drivebase = FtcDrives.mecanum(hardwareMap);

        // Standard TeleOp stick mapping (includes slow-mode button).
        stickDrive = GamepadDriveSource.teleOpMecanumSlowRb(gamepads);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void start() {
        clock.reset(getRuntime());
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void loop() {
        // --- 1) Clock ---
        clock.update(getRuntime());

        // --- 2) Inputs ---
        gamepads.update(clock);

        // --- 3) Logic: sticks -> drive signal ---
        DriveSignal cmd = stickDrive.get(clock).clamped();
        lastDrive = cmd;

        // --- 4) Actuation: update drivebase timing, then send command ---
        drivebase.update(clock);
        drivebase.drive(cmd);

        // --- 5) Required telemetry (driver-facing) ---
        // This is the information you should not lose even when debug output is disabled.
        telemetry.addLine("FW Example 01: Mecanum Basic");
        telemetry.addData("drive.cmd", cmd);
        telemetry.addData("debug.enabled", DEBUG);

        // --- 6) Optional debug (can be disabled without breaking required telemetry) ---
        // Use DebugSink + debugDump() for verbose internal state.
        if (DEBUG) {
            dbg.addLine("--- debugDump() (optional) ---");

            clock.debugDump(dbg, "clock");
            gamepads.debugDump(dbg, "pads");

            stickDrive.debugDump(dbg, "driveSrc");
            drivebase.debugDump(dbg, "drivebase");
        }

        telemetry.update();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void stop() {
        drivebase.stop();
    }
}
