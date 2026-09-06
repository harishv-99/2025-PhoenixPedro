package edu.ftcsushi.robots.examples.basicsensing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;

/** Disabled, no-motion host that shows one switch through the managed service and presenter phases. */
@TeleOp(name = "FW Basic: Read a switch", group = "FW Examples")
@Disabled
public final class BasicSwitchTeleOp extends FtcRobotOpMode {

    /** Uses the lesson's sole configuration edit point and declares its graph once during INIT. */
    @Override
    protected void configure(RobotProgram program) {
        BasicSwitchService.Config config = BasicSwitchService.Config.defaults();
        declare(program, hardwareMap, config);
    }

    /** Shares the exact production graph with the deterministic, test-only managed host. */
    static BasicSwitchService declare(
            RobotProgram program, HardwareMap hardwareMap, BasicSwitchService.Config config) {
        Objects.requireNonNull(program, "program is required");
        BasicSwitchService limitSwitch = program.service(
                new BasicSwitchService(hardwareMap, config));
        program.presenter((clock, telemetry) -> {
            BasicSwitchService.Status status = limitSwitch.status();
            telemetry.addData("switch.observed", status.observed);
            telemetry.addData("switch.rawPressed", status.rawPressed);
            telemetry.addData("switch.pressed", status.pressed);
        });
        return limitSwitch;
    }
}
