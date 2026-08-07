package edu.ftcphoenix.robots.examples.pedro;

import com.pedropathing.geometry.Pose;

import java.util.Objects;
import java.util.function.Supplier;

import edu.ftcphoenix.fw.core.lifecycle.CleanupActions;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
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

    /**
     * Declare the validated Pedro service, mechanism output, and one fresh routine root.
     *
     * <p>The service is registered before paths or mechanism construction so a later failure is
     * cleanup-covered by the already-retained program. The mechanism factory is invoked exactly
     * once only after that ownership transfer; it must construct an ordinary mechanism that cleans
     * any internally created Plant if its own constructor fails.</p>
     *
     * @param program framework-created declaration surface received by the OpMode
     * @param runtime validated Pedro runtime
     * @param mechanismFactory one-shot factory for the privately Plant-owning mechanism
     */
    public BasicPedroAutoRobot(RobotProgram program,
                               PedroPathingRuntime runtime,
                               Supplier<BasicPedroAutoMechanism> mechanismFactory) {
        RobotProgram requiredProgram = Objects.requireNonNull(program, "program");
        PedroPathingRuntime requiredRuntime = Objects.requireNonNull(runtime, "runtime");

        AbsolutePoseEstimator localization = Objects.requireNonNull(
                requiredRuntime.motionPredictor(),
                "runtime.motionPredictor()"
        );
        DriveCommandSink autoDrive = Objects.requireNonNull(
                requiredRuntime.driveAdapter(),
                "runtime.driveAdapter()"
        );
        PedroAutoService service = registerServiceOrStop(
                requiredProgram,
                new PedroAutoService(localization, autoDrive)
        );

        BasicPedroAutoPaths paths = new BasicPedroAutoPaths(requiredRuntime);
        Pose declaredStartPose = paths.pedroStartPose();
        service.applyStartingPoseWith(() ->
                requiredRuntime.setStartingPose(copyPose(declaredStartPose)));

        Supplier<BasicPedroAutoMechanism> requiredMechanismFactory =
                Objects.requireNonNull(mechanismFactory, "mechanismFactory");
        BasicPedroAutoMechanism mechanism = Objects.requireNonNull(
                requiredMechanismFactory.get(),
                "mechanismFactory.get()"
        );
        registerOutputOrStop(requiredProgram, mechanism);

        Task declaredRoot = BasicPedroAutoRoutine.build(
                requiredRuntime.driveAdapter(),
                paths.practiceRoute(),
                mechanism
        );
        requiredProgram.rootTask(declaredRoot);

        pedroStartPose = copyPose(declaredStartPose);
        rootTask = declaredRoot;
    }

    /** Narrow component declaration seam retained package-private for deterministic fake tests. */
    BasicPedroAutoRobot(RobotProgram program,
                        AbsolutePoseEstimator localization,
                        DriveCommandSink autoDrive,
                        Runnable applyStartingPose,
                        BasicPedroAutoMechanism mechanism,
                        Task rootTask) {
        RobotProgram requiredProgram = Objects.requireNonNull(program, "program");
        PedroAutoService service = new PedroAutoService(
                Objects.requireNonNull(localization, "localization"),
                Objects.requireNonNull(autoDrive, "autoDrive")
        );
        service.applyStartingPoseWith(
                Objects.requireNonNull(applyStartingPose, "applyStartingPose")
        );
        registerServiceOrStop(requiredProgram, service);

        BasicPedroAutoMechanism requiredMechanism = Objects.requireNonNull(
                mechanism,
                "mechanism"
        );
        registerOutputOrStop(requiredProgram, requiredMechanism);

        Task requiredRootTask = Objects.requireNonNull(rootTask, "rootTask");
        requiredProgram.rootTask(requiredRootTask);
        pedroStartPose = new Pose(0.0, 0.0, 0.0);
        this.rootTask = requiredRootTask;
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
