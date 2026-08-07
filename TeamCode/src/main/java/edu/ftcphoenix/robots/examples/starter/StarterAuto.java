package edu.ftcphoenix.robots.examples.starter;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.util.Objects;

import edu.ftcphoenix.fw.ftc.FtcRobotOpMode;
import edu.ftcphoenix.fw.ftc.RobotProgram;

/** Tiny declarative Auto that uses the same intake capability as the starter TeleOp. */
@Autonomous(name = "FW Starter: Auto", group = "FW Examples")
@Disabled
public final class StarterAuto extends FtcRobotOpMode {

    private static final double COLLECT_DURATION_SEC = 0.75;

    private final StarterProfile profile;

    /** FTC construction path using the checked-in starter profile. */
    public StarterAuto() {
        this(StarterProfile.current());
    }

    /** Package-private profile seam for focused host tests. */
    StarterAuto(StarterProfile profile) {
        this.profile = Objects.requireNonNull(profile, "profile").copy();
    }

    @Override
    protected void configure(RobotProgram program) {
        new StarterRobot(hardwareMap, profile)
                .declareAuto(program, COLLECT_DURATION_SEC);
    }
}
