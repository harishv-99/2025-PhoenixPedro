package edu.ftcphoenix.robots.examples.pedro;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;
import java.util.function.Supplier;

import edu.ftcphoenix.fw.core.lifecycle.CleanupActions;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.drive.route.RouteStatus;
import edu.ftcphoenix.fw.ftc.RobotProgram;
import edu.ftcphoenix.fw.integrations.pedro.PedroPathingRuntime;
import edu.ftcphoenix.fw.localization.AbsolutePoseEstimator;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;

/**
 * Declares the complete basic Pedro Auto reference in one managed {@link RobotProgram}.
 *
 * <p>This object retains only read-only status used by presenters. The framework-owned program
 * owns the LoopClock, TaskRunner, START/update state, fixed phase order, and terminal cleanup.</p>
 */
public final class BasicPedroAutoRobot {

    private final Pose pedroStartPose;
    private final Task rootTask;
    private final Supplier<RouteStatus> latestRouteStatus;

    /**
     * Construct and declare the complete review-gated basic Pedro Auto graph.
     *
     * <p>The profile is consumed synchronously and retained nowhere. Its motion permission and sole
     * cross-owner motor collision are checked before hardware effects. The Pedro runtime snapshots
     * and validates its own Config, then its service is registered before paths or intake
     * construction so every later {@link RuntimeException} is covered by managed cleanup. The
     * intake owner independently snapshots and validates its own Config.</p>
     *
     * @param program framework-created declaration surface received by the OpMode
     * @param hardwareMap FTC hardware registry used only by the two owner-local construction paths
     * @param profile fresh complete basic Pedro configuration; retained nowhere
     */
    public BasicPedroAutoRobot(RobotProgram program,
                               HardwareMap hardwareMap,
                               BasicPedroProfile profile) {
        RobotProgram requiredProgram = Objects.requireNonNull(program, "program");
        HardwareMap requiredHardwareMap = Objects.requireNonNull(hardwareMap, "hardwareMap");
        BasicPedroProfile activeProfile = Objects.requireNonNull(profile, "profile");
        requireMotionAllowed(activeProfile.allowRobotMotion);

        PedroPathingRuntime.Config pedroConfig = Objects.requireNonNull(
                activeProfile.pedro,
                "BasicPedroProfile.pedro is required"
        );
        BasicPedroAutoMechanism.Config intakeConfig = Objects.requireNonNull(
                activeProfile.intake,
                "BasicPedroProfile.intake is required"
        );
        requireDistinctMotorOwners(pedroConfig, intakeConfig);

        PedroPathingRuntime runtime = PedroPathingRuntime.create(
                requiredHardwareMap,
                pedroConfig
        );

        AbsolutePoseEstimator localization = Objects.requireNonNull(
                runtime.motionPredictor(),
                "runtime.motionPredictor()"
        );
        DriveCommandSink autoDrive = Objects.requireNonNull(
                runtime.driveAdapter(),
                "runtime.driveAdapter()"
        );
        PedroAutoService service = registerServiceOrStop(
                requiredProgram,
                new PedroAutoService(localization, autoDrive)
        );

        BasicPedroAutoPaths paths = new BasicPedroAutoPaths(runtime);
        Pose declaredStartPose = paths.pedroStartPose();
        service.applyStartingPoseWith(() ->
                runtime.setStartingPose(copyPose(declaredStartPose)));

        BasicPedroAutoMechanism mechanism = new BasicPedroAutoMechanism(
                requiredHardwareMap,
                intakeConfig
        );
        registerOutputOrStop(requiredProgram, mechanism);

        Task declaredRoot = BasicPedroAutoRoutine.build(
                runtime.driveAdapter(),
                paths.practiceRoute(),
                mechanism
        );
        requiredProgram.rootTask(declaredRoot);

        pedroStartPose = copyPose(declaredStartPose);
        rootTask = declaredRoot;
        latestRouteStatus = runtime.driveAdapter()::getLatestRouteStatus;
    }

    /**
     * Hardware-neutral component seam for deterministic lifecycle tests.
     *
     * <p>The service becomes program-owned before the mechanism factory is invoked exactly once.
     * A later factory failure therefore exercises the same managed drive-cleanup boundary as the
     * ordinary runtime path without creating a second FTC/runtime configuration recipe.</p>
     */
    BasicPedroAutoRobot(RobotProgram program,
                        AbsolutePoseEstimator localization,
                        DriveCommandSink autoDrive,
                        Runnable applyStartingPose,
                        Supplier<BasicPedroAutoMechanism> mechanismFactory,
                        Task rootTask) {
        RobotProgram requiredProgram = Objects.requireNonNull(program, "program");
        DriveCommandSink requiredAutoDrive = Objects.requireNonNull(autoDrive, "autoDrive");
        PedroAutoService service = registerServiceOrStop(
                requiredProgram,
                new PedroAutoService(
                        Objects.requireNonNull(localization, "localization"),
                        requiredAutoDrive
                )
        );
        service.applyStartingPoseWith(
                Objects.requireNonNull(applyStartingPose, "applyStartingPose")
        );

        Supplier<BasicPedroAutoMechanism> requiredMechanismFactory =
                Objects.requireNonNull(mechanismFactory, "mechanismFactory");
        BasicPedroAutoMechanism mechanism = Objects.requireNonNull(
                requiredMechanismFactory.get(),
                "mechanismFactory.get()"
        );
        registerOutputOrStop(requiredProgram, mechanism);

        Task requiredRootTask = Objects.requireNonNull(rootTask, "rootTask");
        requiredProgram.rootTask(requiredRootTask);
        pedroStartPose = new Pose(0.0, 0.0, 0.0);
        this.rootTask = requiredRootTask;
        latestRouteStatus = () -> RouteStatus.NOT_STARTED;
    }

    /** Return a defensive copy of the declared Pedro-coordinate physical starting pose. */
    public Pose pedroStartPose() {
        return copyPose(pedroStartPose);
    }

    /** Return whether the declared root routine is terminal. */
    public boolean isRootComplete() {
        return rootTask.isComplete();
    }

    /** Return the root routine's current retained outcome without advancing it. */
    public TaskOutcome rootOutcome() {
        return Objects.requireNonNull(
                rootTask.getOutcome(),
                "basic Pedro root Task returned null outcome"
        );
    }

    /** Return the latest retained route status for the same-package additive presenter. */
    RouteStatus latestRouteStatus() {
        return Objects.requireNonNull(
                latestRouteStatus.get(),
                "basic Pedro runtime returned null latest route status"
        );
    }

    private static void requireMotionAllowed(boolean allowed) {
        if (!allowed) {
            throw new IllegalStateException(
                    "BasicPedroProfile.allowRobotMotion must be true before Basic Pedro Auto may "
                            + "construct motion-capable hardware owners. Review the complete active "
                            + "physical configuration before permitting motion, then verify small "
                            + "supervised motion and physical STOP."
            );
        }
    }

    private static void requireDistinctMotorOwners(
            PedroPathingRuntime.Config pedro,
            BasicPedroAutoMechanism.Config intake) {
        if (pedro.mecanumConstants == null) {
            return;
        }
        String intakeName = intake.motorName;
        requireDistinctMotorOwner(
                intakeName,
                "BasicPedroProfile.intake.motorName",
                pedro.mecanumConstants.leftFrontMotorName,
                "BasicPedroProfile.pedro.mecanumConstants.leftFrontMotorName"
        );
        requireDistinctMotorOwner(
                intakeName,
                "BasicPedroProfile.intake.motorName",
                pedro.mecanumConstants.leftRearMotorName,
                "BasicPedroProfile.pedro.mecanumConstants.leftRearMotorName"
        );
        requireDistinctMotorOwner(
                intakeName,
                "BasicPedroProfile.intake.motorName",
                pedro.mecanumConstants.rightFrontMotorName,
                "BasicPedroProfile.pedro.mecanumConstants.rightFrontMotorName"
        );
        requireDistinctMotorOwner(
                intakeName,
                "BasicPedroProfile.intake.motorName",
                pedro.mecanumConstants.rightRearMotorName,
                "BasicPedroProfile.pedro.mecanumConstants.rightRearMotorName"
        );
    }

    private static void requireDistinctMotorOwner(String firstName,
                                                  String firstPath,
                                                  String secondName,
                                                  String secondPath) {
        if (isBlank(firstName) || isBlank(secondName)) {
            return;
        }
        String firstKey = firstName.trim();
        String secondKey = secondName.trim();
        if (firstKey.equals(secondKey)) {
            throw new IllegalStateException(
                    "Basic Pedro motor ownership collision: " + firstPath + " and " + secondPath
                            + " both resolve to FTC hardware key \"" + firstKey
                            + "\". Configure distinct motor names."
            );
        }
    }

    private static boolean isBlank(String value) {
        return value == null || value.trim().isEmpty();
    }

    private static PedroAutoService registerServiceOrStop(RobotProgram program,
                                                           PedroAutoService service) {
        try {
            return program.service(service);
        } catch (RuntimeException registrationFailure) {
            throw CleanupActions.attemptAllAfterFailure(
                    registrationFailure,
                    service::stop
            );
        }
    }

    private static void registerOutputOrStop(RobotProgram program,
                                             BasicPedroAutoMechanism mechanism) {
        try {
            program.output(mechanism);
        } catch (RuntimeException registrationFailure) {
            throw CleanupActions.attemptAllAfterFailure(
                    registrationFailure,
                    mechanism::stop
            );
        }
    }

    private static Pose copyPose(Pose source) {
        Pose required = Objects.requireNonNull(source, "pedroStartPose");
        return new Pose(required.getX(), required.getY(), required.getHeading());
    }

    /** Owns exact-start pose application, localization, Pedro heartbeat, and drive cleanup. */
    private static final class PedroAutoService implements RobotProgram.Service {
        private final AbsolutePoseEstimator localization;
        private final DriveCommandSink autoDrive;
        private Runnable applyStartingPose;

        private PedroAutoService(AbsolutePoseEstimator localization,
                                 DriveCommandSink autoDrive) {
            this.localization = localization;
            this.autoDrive = autoDrive;
        }

        private void applyStartingPoseWith(Runnable action) {
            if (applyStartingPose != null) {
                throw new IllegalStateException("Pedro starting pose action is already declared");
            }
            applyStartingPose = Objects.requireNonNull(action, "applyStartingPose");
        }

        @Override
        public void start(LoopClock clock) {
            Runnable startAction = Objects.requireNonNull(
                    applyStartingPose,
                    "Pedro starting pose action was not declared"
            );
            startAction.run();
            localization.update(clock);
            autoDrive.update(clock);
        }

        @Override
        public void update(LoopClock clock) {
            localization.update(clock);
            autoDrive.update(clock);
        }

        @Override
        public void stop() {
            autoDrive.stop();
        }
    }
}
