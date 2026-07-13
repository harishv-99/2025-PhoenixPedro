package edu.ftcphoenix.fw.integrations.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import java.util.Objects;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.route.RouteExecution;
import edu.ftcphoenix.fw.drive.route.RouteFollower;
import edu.ftcphoenix.fw.drive.route.RouteStatus;

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
 * <p>Each route start returns a per-start {@link RouteExecution}. During the owned
 * heartbeat this adapter distinguishes visible endpoint completion from Pedro timeout/stall,
 * interruption, replacement, failure, and unexplained terminal transitions. Raw Follower
 * lifecycle mutation is unsupported because it bypasses that retained status.</p>
 *
 * <p>Typical usage:</p>
 * <pre>{@code
 * PedroPathingRuntime runtime =
 *         Constants.createPhoenixAutoRuntime(hardwareMap, profile);
 * PedroPathingDriveAdapter adapter = runtime.driveAdapter();
 * robot.initAuto(adapter, runtime.motionPredictor());
 * runtime.setStartingPose(pedroStartPose);
 * // Phoenix now owns every-loop adapter update and final stop.
 *
 * Task auto = Tasks.sequence(
 *         RouteTasks.follow(adapter, outboundPath, new RouteTask.Config()),
 *         aimPlan.task(adapter, aimCfg),
 *         Tasks.runOnce(scoring::requestSingleShot)
 * );
 * robot.enqueueAuto(auto);
 * }</pre>
 * <p>This example demonstrates lifecycle and Task composition only. Generic sequences do not stop
 * automatically after an abnormal route result; robot-owned policy must gate later aiming,
 * scoring, or other position-dependent work.</p>
 */
public final class PedroPathingDriveAdapter implements RouteFollower<PathChain>, DriveCommandSink {

    /** Internal hook used by a production Pedro runtime immediately before its owned heartbeat. */
    interface HeartbeatPreparation {
        void prepare(LoopClock clock);
    }

    /** Internal Pedro behavior selected for the next owned heartbeat. */
    private enum Mode {
        FOLLOWER,
        MANUAL_START_PENDING,
        MANUAL
    }

    private final Follower follower;
    private final HeartbeatPreparation heartbeatPreparation;
    private DriveSignal requestedManualDrive = DriveSignal.zero();
    private Mode mode = Mode.FOLLOWER;
    private long lastUpdateCycle = Long.MIN_VALUE;
    private boolean heartbeatInProgress = false;
    private boolean breakAfterHeartbeat = false;
    private boolean routeStartInProgress = false;
    private PedroRouteExecution latestRouteExecution;

    /**
     * Creates a Phoenix adapter around one Pedro {@link Follower} instance.
     *
     * @param follower Pedro follower to wrap
     */
    public PedroPathingDriveAdapter(Follower follower) {
        this(follower, null);
    }

    /**
     * Internal constructor that binds a passive localization preparation step to this heartbeat.
     *
     * <p>The public adapter remains usable with an independently configured Follower. Production
     * runtimes that share Phoenix localization use this package-private path so the passive Pedro
     * localizer receives the same {@link LoopClock} immediately before the one vendor update.</p>
     */
    PedroPathingDriveAdapter(Follower follower,
                             HeartbeatPreparation heartbeatPreparation) {
        this.follower = Objects.requireNonNull(follower, "follower");
        this.heartbeatPreparation = heartbeatPreparation;
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
     * Returns the latest route attempt's backend-neutral status.
     *
     * <p>The status remains available after the route Task advances, and it never exposes Pedro
     * route types. Before any route has started this returns {@link RouteStatus#NOT_STARTED}.</p>
     *
     * @return current or retained terminal status of the most recently started route
     */
    public RouteStatus getLatestRouteStatus() {
        return latestRouteExecution != null
                ? latestRouteExecution.status()
                : RouteStatus.NOT_STARTED;
    }

    /**
     * Advances the wrapped Follower at most once in the supplied Phoenix loop cycle.
     *
     * <p>The cycle is recorded before entering Pedro, so a callback that reenters this method cannot
     * create a second update. Pedro 2.1.2's {@code startTeleopDrive()} performs one update itself;
     * that hidden update is deliberately counted as the heartbeat for a manual-mode transition.
     * A route-initializer callback reentry is deferred without consuming the cycle, allowing the
     * composition root's normal heartbeat to run after route start returns.</p>
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
        if (routeStartInProgress
                || heartbeatInProgress
                || lastUpdateCycle == currentClock.cycle()) {
            return;
        }

        lastUpdateCycle = currentClock.cycle();
        heartbeatInProgress = true;
        RuntimeException failure = null;
        try {
            try {
                RouteHeartbeatSnapshot routeSnapshot = captureActiveRouteSnapshot();
                if (heartbeatPreparation != null) {
                    heartbeatPreparation.prepare(currentClock);
                }
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
                classifyRouteAfterHeartbeat(routeSnapshot);
            } catch (RuntimeException updateFailure) {
                failure = updateFailure;
                finishActiveRoute(RouteStatus.FAILED);
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
            if (failure != null) {
                throw failure;
            }
        } finally {
            heartbeatInProgress = false;
        }
    }

    /**
     * Starts a route using Pedro's configured default hold-end policy.
     *
     * @param route Pedro path chain to start; must not be null
     * @return per-start execution handle for this exact route start
     */
    @Override
    public RouteExecution follow(PathChain route) {
        return startRoute(route, null);
    }

    /**
     * Begins following a route with an explicit Pedro hold-end choice.
     *
     * @param route path chain to follow
     * @param holdEnd whether Pedro should hold the final point when the path completes
     * @return per-start execution handle for this exact route start
     */
    public RouteExecution follow(PathChain route, boolean holdEnd) {
        return startRoute(route, Boolean.valueOf(holdEnd));
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
            finishActiveRoute(RouteStatus.INTERRUPTED);
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
        finishActiveRoute(RouteStatus.INTERRUPTED);
        stageStoppedRequest();
        interruptFollowerNow();
    }

    /** Start one checked route while retaining identity before Pedro initializes callbacks. */
    private RouteExecution startRoute(PathChain route, Boolean holdEnd) {
        requireRouteStartOutsideHeartbeat();
        PedroRouteExecution execution = null;
        routeStartInProgress = true;
        try {
            PathChain requestedRoute = Objects.requireNonNull(route, "route");
            if (requestedRoute.size() <= 0) {
                throw new IllegalArgumentException(
                        "PedroPathingDriveAdapter.follow(route) requires at least one Path"
                );
            }

            execution = new PedroRouteExecution(requestedRoute);
            finishActiveRoute(RouteStatus.REPLACED);
            prepareFollowerModeForRouteStart();
            latestRouteExecution = execution;
            if (holdEnd == null) {
                follower.followPath(requestedRoute);
            } else {
                follower.followPath(requestedRoute, holdEnd.booleanValue());
            }

            if (execution.isIntegrationActive()) {
                PathChain currentRoute = follower.getCurrentPathChain();
                if (currentRoute != requestedRoute) {
                    finishRouteAndStop(
                            execution,
                            currentRoute != null
                                    ? RouteStatus.REPLACED
                                    : RouteStatus.UNKNOWN_TERMINAL
                    );
                } else if (!follower.isBusy()) {
                    finishRouteAndStop(execution, RouteStatus.UNKNOWN_TERMINAL);
                }
            }
            return execution;
        } catch (RuntimeException startFailure) {
            if (execution != null) {
                execution.finish(RouteStatus.FAILED);
            }
            if (execution == null || latestRouteExecution != execution) {
                finishActiveRoute(RouteStatus.FAILED);
            }
            stageStoppedRequest();
            throw breakFollower(startFailure);
        } finally {
            routeStartInProgress = false;
        }
    }

    /** Capture the exact route and segment state before entering the owned vendor heartbeat. */
    private RouteHeartbeatSnapshot captureActiveRouteSnapshot() {
        PedroRouteExecution execution = latestRouteExecution;
        if (execution == null || !execution.isIntegrationActive() || mode != Mode.FOLLOWER) {
            return null;
        }
        return new RouteHeartbeatSnapshot(
                execution,
                follower.isBusy(),
                follower.getCurrentPath(),
                follower.getChainIndex()
        );
    }

    /** Classify route progress before Pedro clears the evidence behind its busy flag. */
    private void classifyRouteAfterHeartbeat(RouteHeartbeatSnapshot snapshot) {
        if (snapshot == null
                || snapshot.execution != latestRouteExecution
                || !snapshot.execution.isIntegrationActive()) {
            return;
        }

        PathChain currentChain = follower.getCurrentPathChain();
        if (currentChain != snapshot.execution.route) {
            finishRouteAndStop(
                    snapshot.execution,
                    currentChain != null
                            ? RouteStatus.REPLACED
                            : RouteStatus.UNKNOWN_TERMINAL
            );
            return;
        }

        if (follower.isBusy()) {
            int currentIndex = follower.getChainIndex();
            if (currentIndex > snapshot.chainIndex) {
                if (currentIndex != snapshot.chainIndex + 1
                        || snapshot.path == null
                        || !snapshot.path.isAtParametricEnd()) {
                    finishRouteAndStop(
                            snapshot.execution,
                            RouteStatus.FOLLOWER_TIMEOUT_OR_STALL
                    );
                }
                return;
            }
            if (currentIndex < snapshot.chainIndex
                    || (currentIndex == snapshot.chainIndex
                    && snapshot.path != null
                    && follower.getCurrentPath() != snapshot.path)) {
                finishRouteAndStop(snapshot.execution, RouteStatus.UNKNOWN_TERMINAL);
            }
            return;
        }

        if (!snapshot.wasBusy) {
            finishRouteAndStop(snapshot.execution, RouteStatus.UNKNOWN_TERMINAL);
        } else if (endpointConstraintsSatisfied(snapshot.execution)) {
            snapshot.execution.finish(RouteStatus.COMPLETED);
        } else {
            finishRouteAndStop(
                    snapshot.execution,
                    RouteStatus.FOLLOWER_TIMEOUT_OR_STALL
            );
        }
    }

    /** Return whether Pedro visibly achieved every final-path completion requirement. */
    private boolean endpointConstraintsSatisfied(PedroRouteExecution execution) {
        if (!execution.finalPath.isAtParametricEnd()) {
            return false;
        }
        double finalT = execution.finalPath.getClosestPointTValue();
        Pose closestFinalPoint = execution.finalPath.getPoint(finalT);
        double closestFinalHeading = execution.route.getClosestPointHeadingGoal(
                new PathChain.PathT(execution.route.size() - 1, finalT)
        );
        Pose actualPose = follower.getPose();
        double translationError = Math.hypot(
                actualPose.getX() - closestFinalPoint.getX(),
                actualPose.getY() - closestFinalPoint.getY()
        );
        double headingDifference = actualPose.getHeading() - closestFinalHeading;
        double headingError = Math.abs(Math.atan2(
                Math.sin(headingDifference),
                Math.cos(headingDifference)
        ));
        return follower.getVelocity().getMagnitude()
                < execution.finalPath.getPathEndVelocityConstraint()
                && translationError
                < execution.finalPath.getPathEndTranslationalConstraint()
                && headingError < execution.finalPath.getPathEndHeadingConstraint();
    }

    /** Retain one terminal status without changing a newer route execution. */
    private void finishActiveRoute(RouteStatus status) {
        PedroRouteExecution execution = latestRouteExecution;
        if (execution != null) {
            execution.finish(status);
        }
    }

    /** Retain an abnormal terminal status and request immediate plus stable physical stop. */
    private void finishRouteAndStop(PedroRouteExecution execution, RouteStatus status) {
        if (execution != latestRouteExecution || !execution.isIntegrationActive()) {
            return;
        }
        execution.finish(status);
        stageStoppedRequest();
        if (heartbeatInProgress) {
            breakAfterHeartbeat = true;
            return;
        }
        RuntimeException failure = breakFollower(null);
        if (failure != null) {
            throw failure;
        }
    }

    /** Apply execution-scoped cancellation without stopping a newer route. */
    private void cancelExecution(PedroRouteExecution execution) {
        if (execution != latestRouteExecution || !execution.isIntegrationActive()) {
            return;
        }
        execution.finish(RouteStatus.CANCELLED);
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

    /** Reject recursive or heartbeat-time route replacement before Pedro can corrupt ownership. */
    private void requireRouteStartOutsideHeartbeat() {
        if (heartbeatInProgress || routeStartInProgress) {
            throw new IllegalStateException(
                    "Cannot start a Pedro route from inside Follower.update() or a route-start "
                            + "callback; enqueue a fresh RouteTask so the route starts after the "
                            + "current Pedro lifecycle call"
            );
        }
    }

    /** Immutable pre-heartbeat evidence for one active Pedro route generation. */
    private static final class RouteHeartbeatSnapshot {
        final PedroRouteExecution execution;
        final boolean wasBusy;
        final Path path;
        final int chainIndex;

        RouteHeartbeatSnapshot(PedroRouteExecution execution,
                               boolean wasBusy,
                               Path path,
                               int chainIndex) {
            this.execution = execution;
            this.wasBusy = wasBusy;
            this.path = path;
            this.chainIndex = chainIndex;
        }
    }

    /** Pedro-owned implementation of one generation-safe backend-neutral execution handle. */
    private final class PedroRouteExecution extends RouteExecution {
        final PathChain route;
        final Path finalPath;
        private RouteStatus retainedStatus = RouteStatus.ACTIVE;

        PedroRouteExecution(PathChain route) {
            this.route = route;
            this.finalPath = Objects.requireNonNull(
                    route.getPath(route.size() - 1),
                    "Pedro route final Path"
            );
        }

        @Override
        protected RouteStatus integrationStatus() {
            return retainedStatus;
        }

        @Override
        protected void cancelActive() {
            cancelExecution(this);
        }

        boolean isIntegrationActive() {
            return retainedStatus == RouteStatus.ACTIVE;
        }

        void finish(RouteStatus status) {
            RouteStatus terminalStatus = Objects.requireNonNull(status, "status");
            if (terminalStatus == RouteStatus.NOT_STARTED
                    || terminalStatus == RouteStatus.ACTIVE
                    || terminalStatus == RouteStatus.TASK_TIMEOUT) {
                throw new IllegalArgumentException(
                        "Pedro integration cannot retain non-integration terminal status "
                                + terminalStatus
                );
            }
            if (isIntegrationActive()) {
                retainedStatus = terminalStatus;
            }
        }
    }
}
