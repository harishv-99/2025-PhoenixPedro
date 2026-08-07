package edu.ftcphoenix.robots.examples.starter;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Objects;

import edu.ftcphoenix.fw.ftc.FtcRobotOpMode;
import edu.ftcphoenix.fw.ftc.RobotProgram;

/** Thin declarative FTC host for the modern starter TeleOp reference. */
@TeleOp(name = "FW Starter: TeleOp", group = "FW Examples")
@Disabled
public final class StarterTeleOp extends FtcRobotOpMode {

    private final StarterProfile profile;

    /** FTC construction path using the checked-in starter profile. */
    public StarterTeleOp() {
        this(StarterProfile.current());
    }

    /** Package-private profile seam for focused host tests. */
    StarterTeleOp(StarterProfile profile) {
        this.profile = Objects.requireNonNull(profile, "profile").copy();
    }

    @Override
    protected void configure(RobotProgram program) {
        new StarterRobot(hardwareMap, profile).declareTeleOp(program, gamepad1);
    }
}
