package edu.ftcsushi.robots.phoenix.opmode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Objects;

import edu.ftcsushi.fw.core.lifecycle.CleanupActions;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.integrations.pedro.PedroPathingRuntime;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.Tasks;
import edu.ftcsushi.robots.phoenix.PhoenixAlliance;
import edu.ftcsushi.robots.phoenix.PhoenixAutoConfig;
import edu.ftcsushi.robots.phoenix.PhoenixCapabilities;
import edu.ftcsushi.robots.phoenix.PhoenixMatchHandoff;
import edu.ftcsushi.robots.phoenix.PhoenixProfile;
import edu.ftcsushi.robots.phoenix.PhoenixReadiness;
import edu.ftcsushi.robots.phoenix.PhoenixRobot;
import edu.ftcsushi.robots.phoenix.autonomous.PhoenixAutoSpec;
import edu.ftcsushi.robots.phoenix.autonomous.pedro.PhoenixPedroAutoContext;
import edu.ftcsushi.robots.phoenix.autonomous.pedro.PhoenixPedroAutoRoutineFactory;
import edu.ftcsushi.robots.phoenix.autonomous.pedro.PhoenixPedroPathFactory;

/** Internal one-path declaration of a complete managed Phoenix Pedro autonomous program. */
final class PhoenixAutoProgram {

    private final PhoenixAutoPrestart prestart;
    private final PedroPathingRuntime pedroRuntime;
    private final PhoenixPedroPathFactory pathFactory;
    private final PhoenixAutoConfig autoConfig;
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
        PhoenixProfile profile = PhoenixProfile.current();
        PhoenixHardwareOwnershipPreflight.requireDistinctMotorOwners(profile);
        autoConfig = Objects.requireNonNull(
                profile.auto,
                "PhoenixProfile.auto"
        ).copy();
        prestart = requiredProgram.prestart(new PhoenixAutoPrestart(
                requiredSetup,
                profile.calibration,
                profile.targeting,
                profile.fixedAprilTagLayout,
                requiredHost.gamepad1,
                requiredHost.gamepad2
        ));

        PhoenixRobot robot = new PhoenixRobot(requiredHost.hardwareMap);
        pedroRuntime = PedroPathingRuntime.create(
                requiredHost.hardwareMap,
                Constants.phoenixAutoRuntimeConfig(
                        profile.localization == null ? null : profile.localization.predictor,
                        profile.drive == null ? null : profile.drive.wiring,
                        profile.drive != null && profile.drive.enableZeroPowerBrake
                )
        );

        if (requiredSetup.purpose() == PhoenixReadiness.AutoPurpose.MATCH_AUTO) {
            registerMatchHandoffOrStopPedro(requiredHost, requiredProgram);
        }

        // declareAuto transfers the Pedro drive owner into RobotProgram before constructing any
        // later Phoenix hardware owner, so every subsequent failure has one cleanup transaction.
        capabilities = robot.declareAuto(
                requiredProgram,
                profile,
                pedroRuntime.driveAdapter(),
                pedroRuntime.motionPredictor(),
                prestart.eligibleScoringTagIds(),
                BooleanSource.constant(true),
                BooleanSource.constant(false),
                () -> pedroRuntime.setStartingPose(prestart.frozenStartingPose())
        );
        pathFactory = new PhoenixPedroPathFactory(pedroRuntime, autoConfig);

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
                autoConfig,
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
