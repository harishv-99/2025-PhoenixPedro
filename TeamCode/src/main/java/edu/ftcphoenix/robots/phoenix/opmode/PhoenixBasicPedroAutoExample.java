package edu.ftcphoenix.robots.phoenix.opmode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Objects;
import java.util.function.Function;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.ftc.FtcRobotOpMode;
import edu.ftcphoenix.fw.ftc.RobotProgram;
import edu.ftcphoenix.fw.integrations.pedro.PedroPathingRuntime;
import edu.ftcphoenix.robots.examples.pedro.BasicPedroAutoMechanism;
import edu.ftcphoenix.robots.examples.pedro.BasicPedroAutoRobot;
import edu.ftcphoenix.robots.phoenix.PhoenixMatchHandoff;
import edu.ftcphoenix.robots.phoenix.PhoenixProfile;

/**
 * Disabled, compiling FTC host for the independent basic Pedro Auto reference.
 *
 * <p>This is the only example file tied to Phoenix hardware configuration. A new robot replaces
 * this host wiring while retaining the small path, routine, capability, and managed-program
 * declaration shape.</p>
 */
@Autonomous(name = "FW Pedro Auto: Basic Reference", group = "Framework Examples")
@Disabled
public final class PhoenixBasicPedroAutoExample extends FtcRobotOpMode {

    private final Function<RobotProgram, BasicPedroAutoRobot> testRobotFactory;
    private BasicPedroAutoRobot robot;
    private PedroPathingRuntime pedroRuntime;

    /** Creates the Driver Station entry using this repository's real Phoenix configuration. */
    public PhoenixBasicPedroAutoExample() {
        testRobotFactory = null;
    }

    /** Test-only construction seam; it is deliberately not a public extension API. */
    PhoenixBasicPedroAutoExample(
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
                ? createProductionRobot(program)
                : Objects.requireNonNull(
                        testRobotFactory.apply(program),
                        "testRobotFactory.apply(program)"
                );
        program.presenter(this::presentPlacementAndWarning);
        program.presenter(this::presentRouteAndRootStatus);
    }

    private BasicPedroAutoRobot createProductionRobot(RobotProgram program) {
        PhoenixProfile profile = PhoenixProfile.current().copy();
        BasicPedroAutoMechanism.Config mechanismConfig =
                BasicPedroAutoMechanism.Config.of(
                        profile.scoring.nameMotorIntake,
                        profile.scoring.directionMotorIntake,
                        profile.scoring.intakeMotorPower
                );
        PedroPathingRuntime builtRuntime = Constants.createPhoenixAutoRuntime(
                hardwareMap,
                profile
        );

        // BasicPedroAutoRobot registers the runtime service before invoking this mechanism
        // factory, so every later construction failure is covered by program cleanup.
        BasicPedroAutoRobot builtRobot = new BasicPedroAutoRobot(
                program,
                builtRuntime,
                () -> new BasicPedroAutoMechanism(hardwareMap, mechanismConfig)
        );
        pedroRuntime = builtRuntime;
        return builtRobot;
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
        if (pedroRuntime != null) {
            frame.addData(
                    "example.routeStatus",
                    pedroRuntime.driveAdapter().getLatestRouteStatus()
            );
        }
        frame.addData("example.rootComplete", robot.isRootComplete());
        frame.addData("example.rootOutcome", robot.rootOutcome());
    }
}
