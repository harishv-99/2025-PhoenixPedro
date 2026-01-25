package edu.ftcphoenix.robots.phoenix.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcphoenix.fw.ftc.FtcTeleOpTesterOpMode;
import edu.ftcphoenix.fw.tools.tester.StandardTesters;
import edu.ftcphoenix.fw.tools.tester.TeleOpTester;
import edu.ftcphoenix.fw.tools.tester.TesterSuite;
import edu.ftcphoenix.robots.phoenix.tester.PhoenixRobotTesters;

/**
 * Driver Station menu entry for running the Phoenix tester suite.
 *
 * <p>Select this TeleOp, then use the on-screen menu to choose a tester.</p>
 */
@TeleOp(name = "Phoenix: Testers", group = "Phoenix")
public final class PhoenixTestersOpMode extends FtcTeleOpTesterOpMode {

    @Override
    protected TeleOpTester createTester() {
        TesterSuite suite = StandardTesters.createSuite();
        PhoenixRobotTesters.register(suite);
        return suite;
    }
}
