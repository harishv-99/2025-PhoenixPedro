package edu.ftcphoenix.robots.examples.pedro;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;
import java.util.function.Function;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.ftc.FtcRobotOpMode;
import edu.ftcphoenix.fw.ftc.RobotProgram;
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
        program.presenter(this::presentPlacementAndWarning);
        program.presenter(this::presentRouteAndRootStatus);
    }

    /** Add the required placement and test warning without clearing or committing the frame. */
    private void presentPlacementAndWarning(LoopClock clock, Telemetry frame) {
        frame.addLine("Basic Pedro Auto reference: DISABLED TEST ONLY");
        frame.addLine("Verify directions, Pinpoint calibration, clear space, and STOP first.");
        Pose start = robot.pedroStartPose();
        frame.addData(
                "example.expectedPhysicalStartPedro",
                "x=%.1f in, y=%.1f in, heading=%.1f deg",
                start.getX(),
                start.getY(),
                Math.toDegrees(start.getHeading())
        );
    }

    /** Add retained route and root status without advancing either owner. */
    private void presentRouteAndRootStatus(LoopClock clock, Telemetry frame) {
        frame.addData("example.routeStatus", robot.latestRouteStatus());
        frame.addData("example.rootComplete", robot.isRootComplete());
        frame.addData("example.rootOutcome", robot.rootOutcome());
    }
}
