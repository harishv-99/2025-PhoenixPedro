package edu.ftcphoenix.robots.examples.starter;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import edu.ftcphoenix.fw.ftc.FtcRobotOpMode;
import edu.ftcphoenix.fw.ftc.RobotProgram;

/** Tiny declarative Auto that uses the same intake capability as the starter TeleOp. */
@Autonomous(name = "FW Starter: Auto", group = "FW Examples")
@Disabled
public final class StarterAuto extends FtcRobotOpMode {

    private static final double COLLECT_DURATION_SEC = 0.75;

    /** FTC construction path using the checked-in starter profile. */
    public StarterAuto() {
        // FTC constructs OpModes through their public no-argument constructor.
    }

    @Override
    protected void configure(RobotProgram program) {
        StarterProfile profile = StarterProfile.current();
        new StarterRobot(hardwareMap)
                .declareAuto(program, profile, COLLECT_DURATION_SEC);
    }
}
