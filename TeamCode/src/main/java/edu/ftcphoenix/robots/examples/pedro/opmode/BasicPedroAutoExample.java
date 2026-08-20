package edu.ftcphoenix.robots.examples.pedro.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.util.Objects;
import java.util.function.Function;

import edu.ftcphoenix.fw.ftc.FtcRobotOpMode;
import edu.ftcphoenix.fw.ftc.RobotProgram;
import edu.ftcphoenix.robots.examples.pedro.robot.BasicPedroAutoRobot;
import edu.ftcphoenix.robots.examples.pedro.robot.BasicPedroProfile;
import edu.ftcphoenix.robots.phoenix.PhoenixMatchHandoff;

/**
 * Disabled, compiling FTC host for the independent basic Pedro Auto reference.
 *
 * <p>The adjacent {@link BasicPedroProfile} keeps the example's local software baseline and
 * false-by-default motion permission visible without borrowing Phoenix hardware configuration or
 * project Pedro constants. Production and diagnostic Phoenix-season entries instead extend
 * {@code PhoenixAutoOpMode}; this generic example uses the same underlying
 * {@link FtcRobotOpMode}/{@link RobotProgram} grammar without constructing the season robot. The
 * Phoenix match-handoff clear remains one deliberate diagnostic safety dependency, not a hardware
 * configuration source.</p>
 */
@Autonomous(name = "FW Pedro Auto: Basic Reference", group = "Framework Examples")
@Disabled
public final class BasicPedroAutoExample extends FtcRobotOpMode {

    private final Function<RobotProgram, BasicPedroAutoRobot> testRobotFactory;
    private BasicPedroAutoRobot robot;

    /** Creates the disabled Driver Station entry using the independent local example profile. */
    public BasicPedroAutoExample() {
        testRobotFactory = null;
    }

    /** Test-only construction seam; it is deliberately not a public extension API. */
    BasicPedroAutoExample(
            Function<RobotProgram, BasicPedroAutoRobot> testRobotFactory
    ) {
        this.testRobotFactory = Objects.requireNonNull(testRobotFactory, "testRobotFactory");
    }

    /** Construct robot-specific owners and declare their managed roles during FTC INIT. */
    @Override
    protected void configure(RobotProgram program) {
        // This disabled diagnostic host must not preserve a match Auto snapshot for a later TeleOp.
        PhoenixMatchHandoff.clear();

        robot = testRobotFactory == null
                ? new BasicPedroAutoRobot(
                        program,
                        hardwareMap,
                        BasicPedroProfile.current()
                )
                : Objects.requireNonNull(
                        testRobotFactory.apply(program),
                        "testRobotFactory.apply(program)"
                );
    }
}
