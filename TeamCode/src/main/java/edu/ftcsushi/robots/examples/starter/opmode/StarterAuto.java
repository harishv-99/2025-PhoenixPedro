package edu.ftcsushi.robots.examples.starter.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.util.Objects;

import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.robots.examples.starter.capability.intake.StarterIntake;
import edu.ftcsushi.robots.examples.starter.robot.StarterProfile;
import edu.ftcsushi.robots.examples.starter.robot.StarterRobot;

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
        StarterIntake intake = new StarterRobot(hardwareMap).declareAuto(program, profile);
        program.rootTask(oneTimedCollect(intake));
    }

    /**
     * Builds the fresh single-use routine selected by this Auto.
     *
     * @param intake capability that creates the timed collection Task
     * @return fresh Task that collects for the configured duration, then requests STOPPED
     * @throws NullPointerException if {@code intake} is {@code null}
     */
    static Task oneTimedCollect(StarterIntake intake) {
        return Objects.requireNonNull(intake, "intake")
                .collectForSeconds(COLLECT_DURATION_SEC);
    }
}
