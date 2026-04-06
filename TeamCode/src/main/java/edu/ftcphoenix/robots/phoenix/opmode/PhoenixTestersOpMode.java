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
