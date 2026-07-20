package edu.ftcphoenix.robots.phoenix.opmode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Objects;
import java.util.function.Supplier;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.core.lifecycle.CleanupActions;
import edu.ftcphoenix.fw.ftc.FtcActuators;
import edu.ftcphoenix.fw.integrations.pedro.PedroPathingRuntime;
import edu.ftcphoenix.robots.examples.pedro.BasicPedroAutoMechanism;
import edu.ftcphoenix.robots.examples.pedro.BasicPedroAutoPaths;
import edu.ftcphoenix.robots.examples.pedro.BasicPedroAutoRobot;
import edu.ftcphoenix.robots.phoenix.PhoenixMatchHandoff;
import edu.ftcphoenix.robots.phoenix.PhoenixProfile;

/**
 * Disabled, compiling FTC host for the independent basic Pedro Auto reference.
 *
 * <p>This is the only example file tied to Phoenix hardware configuration. A new robot replaces
 * this host wiring while retaining the small paths, routine, capability, and lifecycle shape.</p>
 */
@Autonomous(name = "FW Pedro Auto: Basic Reference", group = "Framework Examples")
@Disabled
public final class PhoenixBasicPedroAutoExample extends OpMode {

    private final Supplier<BasicPedroAutoRobot> testRobotFactory;
    private BasicPedroAutoRobot robot;
    private PedroPathingRuntime pedroRuntime;
    private BasicPedroAutoPaths paths;

    /** Creates the Driver Station entry using this repository's real Phoenix configuration. */
    public PhoenixBasicPedroAutoExample() {
        testRobotFactory = null;
    }

    /** Test-only construction seam; it is deliberately not a public extension API. */
    PhoenixBasicPedroAutoExample(Supplier<BasicPedroAutoRobot> testRobotFactory) {
        this.testRobotFactory = Objects.requireNonNull(testRobotFactory, "testRobotFactory");
    }

    /** Constructs the complete example graph during FTC INIT. */
    @Override
    public void init() {
        // This disabled diagnostic host must not preserve a match Auto snapshot for a later TeleOp.
        PhoenixMatchHandoff.clear();
        try {
            if (robot != null) {
                throw new IllegalStateException("Basic Pedro Auto is already initialized");
            }
            robot = testRobotFactory == null
                    ? createProductionRobot()
                    : Objects.requireNonNull(testRobotFactory.get(), "testRobotFactory.get()");
            emitPlacementTelemetry();
        } catch (RuntimeException initFailure) {
            throw failStop(initFailure);
        }
    }

    /** Keeps the required physical start and test warning visible before START. */
    @Override
    public void init_loop() {
        try {
            emitPlacementTelemetry();
        } catch (RuntimeException telemetryFailure) {
            throw failStop(telemetryFailure);
        }
    }

    /** Forwards the exact FTC START runtime to the composition root. */
    @Override
    public void start() {
        requireRobot().start(getRuntime());
    }

    /** Advances one complete clock/localization/Pedro/Task/Plant loop. */
    @Override
    public void loop() {
        try {
            BasicPedroAutoRobot activeRobot = requireRobot();
            activeRobot.update(getRuntime());
            if (telemetry != null) {
                telemetry.addData("example.autoIdle", activeRobot.isAutoIdle());
                if (pedroRuntime != null) {
                    telemetry.addData(
                            "example.routeStatus",
                            pedroRuntime.driveAdapter().getLatestRouteStatus()
                    );
                }
                telemetry.update();
            }
        } catch (RuntimeException loopFailure) {
            throw failStop(loopFailure);
        }
    }

    /** Delegates all owned cancellation and physical stops to the composition root. */
    @Override
    public void stop() {
        if (robot != null) {
            robot.stop();
        }
    }

    private BasicPedroAutoRobot createProductionRobot() {
        PhoenixProfile profile = PhoenixProfile.current().copy();
        PedroPathingRuntime builtRuntime = Constants.createPhoenixAutoRuntime(
                hardwareMap,
                profile
        );
        Plant builtIntakePlant = null;
        BasicPedroAutoMechanism builtMechanism = null;
        try {
            BasicPedroAutoPaths builtPaths = new BasicPedroAutoPaths(builtRuntime);
            builtIntakePlant = FtcActuators.plant(hardwareMap)
                    .motor(
                            profile.scoring.nameMotorIntake,
                            profile.scoring.directionMotorIntake
                    )
                    .power()
                    .targetedByDefaultWritable(0.0)
                    .build();
            builtMechanism = new BasicPedroAutoMechanism(
                    builtIntakePlant,
                    profile.scoring.intakeMotorPower
            );
            BasicPedroAutoRobot builtRobot = new BasicPedroAutoRobot(
                    builtRuntime,
                    builtPaths,
                    builtMechanism
            );
            pedroRuntime = builtRuntime;
            paths = builtPaths;
            return builtRobot;
        } catch (RuntimeException constructionFailure) {
            throw cleanupPartialConstruction(
                    constructionFailure,
                    builtMechanism,
                    builtIntakePlant,
                    builtRuntime
            );
        }
    }

    private void emitPlacementTelemetry() {
        if (telemetry == null) {
            return;
        }
        telemetry.addLine("Basic Pedro Auto reference: DISABLED TEST ONLY");
        telemetry.addLine("Verify directions, Pinpoint calibration, clear space, and STOP first.");
        if (paths != null) {
            Pose start = paths.pedroStartPose();
            telemetry.addData(
                    "example.expectedPhysicalStartPedro",
                    "x=%.1f in, y=%.1f in, heading=%.1f deg",
                    start.getX(),
                    start.getY(),
                    Math.toDegrees(start.getHeading())
            );
        }
        telemetry.update();
    }

    private BasicPedroAutoRobot requireRobot() {
        if (robot == null) {
            throw new IllegalStateException("Call init() before the Basic Pedro Auto lifecycle");
        }
        return robot;
    }

    private static RuntimeException cleanupPartialConstruction(
            RuntimeException primary,
            BasicPedroAutoMechanism mechanism,
            Plant intakePlant,
            PedroPathingRuntime runtime) {
        Runnable driveCleanup = new Runnable() {
            @Override
            public void run() {
                runtime.driveAdapter().stop();
            }
        };
        if (mechanism != null) {
            return CleanupActions.attemptAllAfterFailure(
                    primary,
                    mechanism::stop,
                    driveCleanup
            );
        }
        if (intakePlant != null) {
            return CleanupActions.attemptAllAfterFailure(
                    primary,
                    intakePlant::stop,
                    driveCleanup
            );
        }
        return CleanupActions.attemptAllAfterFailure(primary, driveCleanup);
    }

    private RuntimeException failStop(RuntimeException primary) {
        if (robot == null) {
            return primary;
        }
        return CleanupActions.attemptAllAfterFailure(primary, robot::stop);
    }
}
