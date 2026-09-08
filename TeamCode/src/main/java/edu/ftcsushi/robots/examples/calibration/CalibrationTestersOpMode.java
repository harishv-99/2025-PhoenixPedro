package edu.ftcsushi.robots.examples.calibration;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcsushi.fw.ftc.FtcTeleOpTesterOpMode;
import edu.ftcsushi.fw.tools.tester.TeleOpTester;

/**
 * Disabled independent diagnostic entry with physical gamepads and Driver Station telemetry.
 *
 * <p>Copy this teaching graph into the adopting robot's package and review its canonical facts
 * before enabling an entry. The tester host owns the only clock and all selected-child lifecycle;
 * do not run a match robot graph beside it or add another update loop. The default suite has no
 * camera or powered drive. Removing Disabled is not a hardware-safety approval.</p>
 */
@TeleOp(name = "FW Example: Calibration Testers", group = "FW Examples")
@Disabled
public final class CalibrationTestersOpMode extends FtcTeleOpTesterOpMode {
    @Override
    protected TeleOpTester createTester() {
        return CalibrationTesters.create(CalibrationRobotProfile.current());
    }
}
