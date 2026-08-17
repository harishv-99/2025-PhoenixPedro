package edu.ftcphoenix.robots.phoenix.opmode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Objects;

import edu.ftcphoenix.fw.core.lifecycle.CleanupActions;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.ftc.RobotProgram;
import edu.ftcphoenix.fw.integrations.pedro.PedroPathingRuntime;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.Tasks;
import edu.ftcphoenix.robots.phoenix.PhoenixAlliance;
import edu.ftcphoenix.robots.phoenix.PhoenixCapabilities;
import edu.ftcphoenix.robots.phoenix.PhoenixMatchHandoff;
import edu.ftcphoenix.robots.phoenix.PhoenixProfile;
import edu.ftcphoenix.robots.phoenix.PhoenixReadiness;
import edu.ftcphoenix.robots.phoenix.PhoenixRobot;
import edu.ftcphoenix.robots.phoenix.autonomous.PhoenixAutoSpec;
import edu.ftcphoenix.robots.phoenix.autonomous.pedro.PhoenixPedroAutoContext;
import edu.ftcphoenix.robots.phoenix.autonomous.pedro.PhoenixPedroAutoRoutineFactory;
import edu.ftcphoenix.robots.phoenix.autonomous.pedro.PhoenixPedroPathFactory;

/** Internal one-path declaration of a complete managed Phoenix Pedro autonomous program. */
final class PhoenixAutoProgram {

    private final PhoenixAutoPrestart prestart;
    private final PedroPathingRuntime pedroRuntime;
    private final PhoenixPedroPathFactory pathFactory;
    private final PhoenixProfile profile;
    private final PhoenixCapabilities capabilities;
    private final Task rootTask;

    private PhoenixPedroPathFactory.Paths paths;

    /**
     * Declare one Phoenix Auto graph, first mapping Pedro data without effects and then crossing
     * the sole effectful {@link PedroPathingRuntime#create} hardware-construction boundary.
     */
    PhoenixAutoProgram(PhoenixAutoOpMode host,
                       RobotProgram program,
                       PhoenixAutoSetup setup) {
        PhoenixAutoOpMode requiredHost = Objects.requireNonNull(host, "host");
        RobotProgram requiredProgram = Objects.requireNonNull(program, "program");
        PhoenixAutoSetup requiredSetup = Objects.requireNonNull(setup, "setup");

        // Clear first so even a construction failure cannot leave a prior match snapshot visible.
        PhoenixMatchHandoff.clear();
        profile = PhoenixProfile.current().copy();
        prestart = requiredProgram.prestart(new PhoenixAutoPrestart(
                requiredSetup,
                profile,
                requiredHost.gamepad1,
                requiredHost.gamepad2
        ));

        PhoenixRobot robot = new PhoenixRobot(
                requiredHost.hardwareMap,
                requiredHost.telemetry,
                requiredHost.gamepad1,
                requiredHost.gamepad2,
                profile
        );
        pedroRuntime = PedroPathingRuntime.create(
                requiredHost.hardwareMap,
                Constants.phoenixAutoRuntimeConfig(profile)
        );

        if (requiredSetup.purpose() == PhoenixReadiness.AutoPurpose.MATCH_AUTO) {
            registerMatchHandoffOrStopPedro(requiredHost, requiredProgram);
        }

        // declareAuto transfers the Pedro drive owner into RobotProgram before constructing any
        // later Phoenix hardware owner, so every subsequent failure has one cleanup transaction.
        robot.declareAuto(
                requiredProgram,
                pedroRuntime.driveAdapter(),
                pedroRuntime.motionPredictor(),
                prestart.eligibleScoringTagIds(),
                BooleanSource.constant(true),
                BooleanSource.constant(false),
                () -> pedroRuntime.setStartingPose(prestart.frozenStartingPose())
        );
        capabilities = robot.capabilities();
        pathFactory = new PhoenixPedroPathFactory(pedroRuntime, profile.auto);

        if (requiredSetup.selectionMode() == PhoenixAutoSetup.SelectionMode.FIXED) {
            rootTask = buildRoutine(prestart.fixedSpecForEagerBuild());
        } else {
            rootTask = Tasks.buildAtStart(
                    "phoenix.selectedAuto",
                    () -> buildRoutine(prestart.frozenSpec())
            );
        }
        requiredProgram.rootTask(rootTask);

        requiredProgram.presenter(prestart::present);
        requiredProgram.presenter(this::presentRoute);
        requiredProgram.presenter(robot.autoPresenter(rootTask));
    }

    private Task buildRoutine(PhoenixAutoSpec spec) {
        PhoenixAutoSpec requiredSpec = Objects.requireNonNull(spec, "spec");
        PhoenixPedroPathFactory.Paths builtPaths = pathFactory.build(
                requiredSpec,
                capabilities
        );
        paths = builtPaths;
        return PhoenixPedroAutoRoutineFactory.build(new PhoenixPedroAutoContext(
                requiredSpec,
                profile,
                capabilities,
                pedroRuntime.driveAdapter(),
                pathFactory,
                builtPaths
        ));
    }

    private void registerMatchHandoffOrStopPedro(PhoenixAutoOpMode host,
                                                  RobotProgram program) {
        try {
            program.stopHandoff(
                    () -> new MatchHandoffCapture(
                            pedroRuntime.motionPredictor().getEstimate(),
                            prestart.frozenSpec().alliance
                    ),
                    capture -> PhoenixMatchHandoff.publishFromAuto(
                            host,
                            capture.finalPose,
                            capture.alliance
                    ),
                    PhoenixMatchHandoff::clear
            );
        } catch (RuntimeException registrationFailure) {
            throw CleanupActions.attemptAllAfterFailure(
                    registrationFailure,
                    pedroRuntime.driveAdapter()::stop
            );
        }
    }

    private void presentRoute(LoopClock clock, Telemetry telemetry) {
        Objects.requireNonNull(clock, "clock");
        if (telemetry == null) {
            return;
        }
        PhoenixPedroPathFactory.Paths builtPaths = paths;
        telemetry.addData(
                "auto.path",
                builtPaths == null ? "<builds once at START>" : builtPaths.label
        );
        telemetry.addData(
                "auto.routeStatus",
                pedroRuntime.driveAdapter().getLatestRouteStatus()
        );
    }

    /** Captures both match facts before cleanup so publication reads no live Auto owner. */
    private static final class MatchHandoffCapture {
        private final PoseEstimate finalPose;
        private final PhoenixAlliance alliance;

        private MatchHandoffCapture(
                PoseEstimate finalPose,
                PhoenixAlliance alliance
        ) {
            this.finalPose = finalPose;
            this.alliance = Objects.requireNonNull(
                    alliance,
                    "Phoenix Auto handoff alliance is required"
            );
        }
    }
}
