package edu.ftcsushi.robots.phoenix.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.robots.phoenix.PhoenixAlliance;

/**
 * Main Phoenix TeleOp Driver Station entry.
 *
 * <p>This is an ordinary managed Phoenix program. The framework owns FTC lifecycle forwarding,
 * the shared clock, binding and output heartbeats, one telemetry commit, and fail-stop cleanup.
 * Robot-specific construction remains here, while operator meanings and subsystem behavior stay
 * in their robot-owned classes.</p>
 *
 * <p>Gamepad 1 selects RED or BLUE through one INIT screen; RED is the documented standalone
 * default and FTC START freezes that choice. After localization is declared, one fresh final Auto
 * snapshot may restore pose and seed the still-editable alliance draft. Missing, stale, or
 * already-consumed state leaves both normal pose initialization and the RED draft intact. This
 * OpMode never commits a separate telemetry frame.</p>
 */
@TeleOp(name = "Phoenix TeleOp", group = "Phoenix")
public final class PhoenixTeleOp extends FtcRobotOpMode {

    /** Declare Phoenix's complete managed TeleOp graph once during FTC INIT. */
    @Override
    protected void configure(RobotProgram program) {
        new PhoenixTeleOpProgram(this, program, PhoenixAlliance.RED);
    }
}
