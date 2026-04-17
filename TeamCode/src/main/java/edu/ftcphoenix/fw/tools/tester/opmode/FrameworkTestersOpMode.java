package edu.ftcphoenix.fw.tools.tester.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcphoenix.fw.ftc.FtcTeleOpTesterOpMode;
import edu.ftcphoenix.fw.tools.tester.StandardTesters;
import edu.ftcphoenix.fw.tools.tester.TeleOpTester;

/**
 * Ready-to-run Driver Station entrypoint for the framework-owned tester tree.
 *
 * <p>This is the no-robot-glue path: teams that copy only the {@code fw} portion of Phoenix still
 * get a usable tester menu immediately, without first writing their own {@link FtcTeleOpTesterOpMode}
 * wrapper. Robot projects that have their own configured tester trees can keep embedding the shared
 * framework categories through {@link StandardTesters#register(edu.ftcphoenix.fw.tools.tester.TesterSuite)}.</p>
 */
@TeleOp(name = "FW: Testers", group = "Framework Testers")
public final class FrameworkTestersOpMode extends FtcTeleOpTesterOpMode {

    /**
     * Creates the FTC OpMode instance.
     */
    public FrameworkTestersOpMode() {
    }

    /**
     * Builds the framework-owned tester home shown in the Driver Station.
     */
    @Override
    protected TeleOpTester createTester() {
        return StandardTesters.createSuite();
    }
}
