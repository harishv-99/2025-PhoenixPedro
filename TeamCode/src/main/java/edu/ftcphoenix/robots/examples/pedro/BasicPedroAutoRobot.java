package edu.ftcphoenix.robots.examples.pedro;

import java.util.Objects;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.integrations.pedro.PedroPathingRuntime;
import edu.ftcphoenix.fw.localization.AbsolutePoseEstimator;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskRunner;

/**
 * Composition root for the basic Pedro Auto reference.
 *
 * <p>It owns one clock, one localization update, one recurring drive heartbeat, one root Task,
 * one mechanism realization, and their complete shutdown order.</p>
 */
public final class BasicPedroAutoRobot {

    private final LoopClock clock = new LoopClock();
    private final AbsolutePoseEstimator localization;
    private final DriveCommandSink autoDrive;
    private final Runnable applyStartingPose;
    private final BasicPedroAutoMechanism mechanism;
    private final Task rootTask;
    private final TaskRunner autoRunner = new TaskRunner();

    private boolean startAttempted;
    private boolean started;
    private boolean stopped;

    /**
     * Wires the validated Pedro runtime, fixed paths, mechanism, and matching routine together.
     */
    public BasicPedroAutoRobot(PedroPathingRuntime runtime,
                               BasicPedroAutoPaths paths,
                               BasicPedroAutoMechanism mechanism) {
        PedroPathingRuntime requiredRuntime = Objects.requireNonNull(runtime, "runtime");
        BasicPedroAutoPaths requiredPaths = Objects.requireNonNull(paths, "paths");
        this.mechanism = Objects.requireNonNull(mechanism, "mechanism");
        localization = Objects.requireNonNull(
                requiredRuntime.motionPredictor(),
                "runtime.motionPredictor()"
        );
        autoDrive = Objects.requireNonNull(
                requiredRuntime.driveAdapter(),
                "runtime.driveAdapter()"
        );
        applyStartingPose = new Runnable() {
            @Override
            public void run() {
                requiredRuntime.setStartingPose(requiredPaths.pedroStartPose());
            }
        };
        rootTask = BasicPedroAutoRoutine.build(
                requiredRuntime.driveAdapter(),
                requiredPaths.practiceRoute(),
                this.mechanism
        );
    }

    /** Narrow component constructor retained package-private for deterministic fake tests. */
    BasicPedroAutoRobot(AbsolutePoseEstimator localization,
                        DriveCommandSink autoDrive,
                        Runnable applyStartingPose,
                        BasicPedroAutoMechanism mechanism,
                        Task rootTask) {
        this.localization = Objects.requireNonNull(localization, "localization");
        this.autoDrive = Objects.requireNonNull(autoDrive, "autoDrive");
        this.applyStartingPose = Objects.requireNonNull(
                applyStartingPose,
                "applyStartingPose"
        );
        this.mechanism = Objects.requireNonNull(mechanism, "mechanism");
        this.rootTask = Objects.requireNonNull(rootTask, "rootTask");
    }

    /**
     * Applies the declared start pose, resets the shared clock, and queues the root routine.
     *
     * <p>The Task is intentionally not advanced here. The first regular loop refreshes
     * localization and the Pedro heartbeat before the runner starts the route.</p>
     */
    public void start(double runtimeSec) {
        requireFinite(runtimeSec, "runtimeSec");
        if (startAttempted) {
            throw new IllegalStateException(
                    "BasicPedroAutoRobot start() may be called only once per OpMode"
            );
        }
        if (stopped) {
            throw new IllegalStateException("BasicPedroAutoRobot cannot start after stop()");
        }
        startAttempted = true;

        try {
            applyStartingPose.run();
            if (stopped) {
                throw new IllegalStateException(
                        "BasicPedroAutoRobot was stopped while applying its starting pose"
                );
            }
            clock.reset(runtimeSec);
            autoRunner.enqueue(rootTask);
            started = true;
        } catch (RuntimeException startFailure) {
            failStop(startFailure);
            throw startFailure;
        }
    }

    /** Advances the complete Auto graph once in its documented ownership order. */
    public void update(double runtimeSec) {
        if (stopped) {
            return;
        }
        if (!started) {
            throw new IllegalStateException(
                    "BasicPedroAutoRobot update() requires start(runtimeSec) first"
            );
        }

        try {
            requireFinite(runtimeSec, "runtimeSec");
            clock.update(runtimeSec);
            localization.update(clock);
            autoDrive.update(clock);
            autoRunner.update(clock);
            mechanism.update(clock);
        } catch (RuntimeException updateFailure) {
            failStop(updateFailure);
            throw updateFailure;
        }
    }

    /** Returns whether no root work is queued or active. */
    public boolean isAutoIdle() {
        return autoRunner.isIdle();
    }

    /** Returns whether this composition root has completed its one-shot shutdown. */
    public boolean isStopped() {
        return stopped;
    }

    /**
     * Cancels behavior, stops the mechanism, and immediately stops drive, attempting every owner.
     */
    public void stop() {
        if (stopped) {
            return;
        }
        stopped = true;
        started = false;

        RuntimeException failure = null;
        failure = attempt(failure, new Runnable() {
            @Override
            public void run() {
                autoRunner.cancelAndClear();
            }
        });
        failure = attempt(failure, new Runnable() {
            @Override
            public void run() {
                mechanism.stop();
            }
        });
        failure = attempt(failure, new Runnable() {
            @Override
            public void run() {
                autoDrive.stop();
            }
        });
        if (failure != null) {
            throw failure;
        }
    }

    private void failStop(RuntimeException primaryFailure) {
        try {
            stop();
        } catch (RuntimeException cleanupFailure) {
            if (cleanupFailure != primaryFailure) {
                primaryFailure.addSuppressed(cleanupFailure);
            }
        }
    }

    private static RuntimeException attempt(RuntimeException first, Runnable action) {
        try {
            action.run();
        } catch (RuntimeException failure) {
            if (first == null) {
                return failure;
            }
            if (failure != first) {
                first.addSuppressed(failure);
            }
        }
        return first;
    }

    private static void requireFinite(double value, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite, got " + value);
        }
    }
}
