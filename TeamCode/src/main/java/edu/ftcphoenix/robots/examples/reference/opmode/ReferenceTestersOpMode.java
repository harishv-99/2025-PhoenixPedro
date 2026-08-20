package edu.ftcphoenix.robots.examples.reference.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcphoenix.fw.ftc.FtcTeleOpTesterOpMode;
import edu.ftcphoenix.fw.tools.tester.TeleOpTester;
import edu.ftcphoenix.robots.examples.reference.tester.ReferenceRobotTesters;

/** Disabled Driver Station entry for the locked reference experiment tree. */
@TeleOp(name = "FW Reference: Experiments", group = "FW Examples")
@Disabled
public final class ReferenceTestersOpMode extends FtcTeleOpTesterOpMode {
    @Override
    protected TeleOpTester createTester() {
        return ReferenceRobotTesters.create();
    }
}
