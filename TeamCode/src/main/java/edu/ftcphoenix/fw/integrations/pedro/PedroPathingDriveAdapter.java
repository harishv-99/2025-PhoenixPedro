package edu.ftcphoenix.fw.integrations.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

import java.util.Objects;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.route.RouteFollower;

/**
 * Framework adapter that bridges Pedro Pathing into Phoenix's small route/drive seams.
 *
 * <p>This adapter intentionally plays three closely related roles for one Pedro Follower:</p>
 * <ul>
 *   <li>{@link RouteFollower}&lt;{@link PathChain}&gt; so Phoenix tasks can sequence Pedro routes.</li>
 *   <li>{@link DriveCommandSink} so Phoenix guidance tasks (such as aim tasks) can temporarily
 *       command the same follower directly using normalized robot-centric drive signals.</li>
 *   <li>the cycle-aware owner of Pedro's recurring heartbeat and immediate stopped state.</li>
 * </ul>
 *
 * <p>The robot composition root must call {@link #update(LoopClock)} every Auto loop, even while a
 * mechanism or wait Task is active. Route and guidance Tasks may also call the same hook; repeated
 * calls in one {@link LoopClock#cycle()} are no-ops. This keeps hold-end, pose, callbacks, manual
 * drive, and stopped-state updates alive without advancing Pedro twice.</p>
 *
 * <p>Typical usage:</p>
 * <pre>{@code
 * Follower follower = Constants.createFollower(hardwareMap);
 * PedroPathingDriveAdapter adapter = new PedroPathingDriveAdapter(follower);
 * robot.initAuto(adapter); // composition root owns every-loop update + final stop
 *
 * Task auto = Tasks.sequence(
 *         RouteTasks.follow(adapter, outboundPath, new RouteTask.Config()),
 *         aimPlan.task(adapter, aimCfg),
 *         Tasks.runOnce(scoring::requestSingleShot)
 * );
 * robot.enqueueAuto(auto);
 * }</pre>
 */
public final class PedroPathingDriveAdapter implements RouteFollower<PathChain>, DriveCommandSink {

    /** Internal Pedro behavior selected for the next owned heartbeat. */
    private enum Mode {
        FOLLOWER,
        MANUAL_START_PENDING,
        MANUAL
    }

    private final Follower follower;
    private DriveSignal requestedManualDrive = DriveSignal.zero();
    private Mode mode = Mode.FOLLOWER;
    private long lastUpdateCycle = Long.MIN_VALUE;
    private boolean heartbeatInProgress = false;
    private boolean breakAfterHeartbeat = false;

    /**
     * Creates a Phoenix adapter around one Pedro {@link Follower} instance.
     *
     * @param follower Pedro follower to wrap
     */
    public PedroPathingDriveAdapter(Follower follower) {
        this.follower = Objects.requireNonNull(follower, "follower");
    }

    /**
     * Returns the wrapped Pedro follower for route building or read-only advanced inspection.
     *
     * <p>Runtime lifecycle calls such as follow, manual drive, update, cancellation, and stop must
     * go through this adapter so its one-heartbeat and stopped-state guarantees remain intact.</p>
     */
    public Follower follower() {
        return follower;
    }

    /**
     * Advances the wrapped Follower at most once in the supplied Phoenix loop cycle.
     *
     * <p>The cycle is recorded before entering Pedro, so a callback that reenters this method cannot
     * create a second update. Pedro 2.1.2's {@code startTeleopDrive()} performs one update itself;
     * that hidden update is deliberately counted as the heartbeat for a manual-mode transition.</p>
     *
     * <p>If Pedro throws, the current cycle remains consumed, the adapter best-effort stops the
     * Follower, and the original failure is rethrown with any stop failure suppressed.</p>
     *
     * @param clock shared Phoenix clock for the current OpMode cycle; must not be null
     */
    @Override
    public void update(LoopClock clock) {
        LoopClock currentClock = Objects.requireNonNull(
                clock,
                "PedroPathingDriveAdapter.update(clock) requires the shared LoopClock"
        );
        if (heartbeatInProgress || lastUpdateCycle == currentClock.cycle()) {
            return;
        }

        lastUpdateCycle = currentClock.cycle();
        heartbeatInProgress = true;
        RuntimeException failure = null;

        try {
            if (mode == Mode.MANUAL_START_PENDING) {
                // Pedro 2.1.2 calls Follower.update() internally here. Do not update again.
                follower.startTeleopDrive();
                if (!breakAfterHeartbeat && mode == Mode.MANUAL_START_PENDING) {
                    applyRequestedManualDrive();
                    mode = Mode.MANUAL;
                }
            } else if (mode == Mode.MANUAL) {
                applyRequestedManualDrive();
                follower.update();
            } else {
                follower.update();
            }
        } catch (RuntimeException updateFailure) {
            failure = updateFailure;
            stageStoppedRequest();
        }

        boolean mustBreak = breakAfterHeartbeat || failure != null;
        breakAfterHeartbeat = false;
        if (mustBreak) {
            failure = breakFollower(failure);
            if (failure != null) {
                stageStoppedRequest();
            }
        }

        heartbeatInProgress = false;
        if (failure != null) {
            throw failure;
        }
    }

    /**
     * Starts a route using Pedro's configured default hold-end policy.
     *
     * @param route Pedro path chain to start; must not be null
     */
    @Override
    public void follow(PathChain route) {
        requireRouteStartOutsideHeartbeat();
        PathChain requestedRoute = Objects.requireNonNull(route, "route");
        prepareFollowerModeForRouteStart();
        try {
            follower.followPath(requestedRoute);
        } catch (RuntimeException startFailure) {
            failRouteStartClosed(startFailure);
        }
    }

    /**
     * Begins following a route with an explicit Pedro hold-end choice.
     *
     * @param route path chain to follow
     * @param holdEnd whether Pedro should hold the final point when the path completes
     */
    public void follow(PathChain route, boolean holdEnd) {
        requireRouteStartOutsideHeartbeat();
        PathChain requestedRoute = Objects.requireNonNull(route, "route");
        prepareFollowerModeForRouteStart();
        try {
            follower.followPath(requestedRoute, holdEnd);
        } catch (RuntimeException startFailure) {
            failRouteStartClosed(startFailure);
        }
    }

    @Override
    public boolean isBusy() {
        return follower.isBusy();
    }

    @Override
    public void cancel() {
        stop();
    }

    /**
     * Stages a clamped robot-centric manual command for the owned Pedro heartbeat.
     *
     * <p>Changing from route/hold mode interrupts that behavior immediately, then the next owned
     * heartbeat enters Pedro manual mode. Pedro's hidden transition update applies zero; the staged
     * command is retained for the following heartbeat rather than updating twice in one cycle.</p>
     */
    @Override
    public void drive(DriveSignal signal) {
        DriveSignal cmd = (signal != null) ? signal.clamped() : DriveSignal.zero();
        requestedManualDrive = cmd;
        if (mode == Mode.FOLLOWER) {
            mode = Mode.MANUAL_START_PENDING;
            interruptFollowerNow();
        }
    }

    /**
     * Stops physical drive output immediately and keeps later heartbeats in stable zero-manual mode.
     *
     * <p>The operation is idempotent. If it is requested from inside a Pedro callback, the adapter
     * stops once immediately and again after the enclosing vendor update returns, preventing a
     * later write in that update from becoming the final motor command.</p>
     */
    @Override
    public void stop() {
        stageStoppedRequest();
        interruptFollowerNow();
    }

    /**
     * Select route/hold behavior before Pedro initializes callbacks for a new route.
     *
     * <p>The ordering is deliberate: a callback initializer may request stop/manual behavior, and
     * that reentrant safety request must win instead of being overwritten after
     * {@code followPath(...)} returns.</p>
     */
    private void prepareFollowerModeForRouteStart() {
        mode = Mode.FOLLOWER;
        requestedManualDrive = DriveSignal.zero();
    }

    /** Replace any pending motion with the persistent zero-manual stopped request. */
    private void stageStoppedRequest() {
        requestedManualDrive = DriveSignal.zero();
        mode = Mode.MANUAL_START_PENDING;
    }

    /** Send the staged normalized robot-centric vector into Pedro's manual vector calculator. */
    private void applyRequestedManualDrive() {
        follower.setTeleOpDrive(
                requestedManualDrive.axial,
                requestedManualDrive.lateral,
                requestedManualDrive.omega,
                true
        );
    }

    /** Interrupt the current Pedro behavior immediately and remember callback-time cleanup. */
    private void interruptFollowerNow() {
        if (heartbeatInProgress) {
            breakAfterHeartbeat = true;
        }
        RuntimeException failure = breakFollower(null);
        if (failure != null) {
            stageStoppedRequest();
            throw failure;
        }
    }

    /** Best-effort Pedro break that preserves an existing failure as the primary exception. */
    private RuntimeException breakFollower(RuntimeException primaryFailure) {
        try {
            follower.breakFollowing();
        } catch (RuntimeException stopFailure) {
            if (primaryFailure == null) {
                return stopFailure;
            }
            if (stopFailure != primaryFailure) {
                primaryFailure.addSuppressed(stopFailure);
            }
        }
        return primaryFailure;
    }

    /** Fail closed when Pedro partially starts a route and then throws. */
    private void failRouteStartClosed(RuntimeException startFailure) {
        stageStoppedRequest();
        RuntimeException failure = breakFollower(startFailure);
        throw failure;
    }

    /** Reject route replacement from inside a vendor heartbeat; ROUTE-02 owns replacement policy. */
    private void requireRouteStartOutsideHeartbeat() {
        if (heartbeatInProgress) {
            throw new IllegalStateException(
                    "Cannot start a Pedro route from inside Follower.update(); enqueue a fresh "
                            + "RouteTask so the route starts after the current loop heartbeat"
            );
        }
    }
}
