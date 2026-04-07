package edu.ftcphoenix.robots.phoenix.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcphoenix.fw.ftc.FtcTeleOpTesterOpMode;
import edu.ftcphoenix.fw.tools.tester.StandardTesters;
import edu.ftcphoenix.fw.tools.tester.TeleOpTester;
import edu.ftcphoenix.fw.tools.tester.TesterSuite;
import edu.ftcphoenix.robots.phoenix.tester.PhoenixRobotTesters;

/**
 * Driver Station menu entry for the organized Phoenix tester tree.
 */
@TeleOp(name = "Phoenix: Testers", group = "Phoenix")
public final class PhoenixTestersOpMode extends FtcTeleOpTesterOpMode {

    /**
     * Creates the FTC OpMode instance.
     */
    public PhoenixTestersOpMode() {
    }

    /**
     * Builds the top-level tester menu shown in the Driver Station.
     *
     * @return tester root containing Phoenix-specific tools followed by the shared framework tools
     */
    @Override
    protected TeleOpTester createTester() {
        TesterSuite suite = new TesterSuite()
                .setTitle("Phoenix Tester Home")
                .setHelp("Guide first for a fresh robot. Dpad: select | A: enter | BACK: back");

        // Put the Phoenix-specific guide and robot-configured tools first.
        PhoenixRobotTesters.register(suite);
        StandardTesters.register(suite);
        return suite;
    }
}
