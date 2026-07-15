package edu.ftcphoenix.robots.phoenix.autonomous.pedro;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.route.RouteStatus;
import edu.ftcphoenix.fw.drive.route.RouteTask;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;
import edu.ftcphoenix.robots.phoenix.PhoenixCapabilities;

/**
 * Coordinates one Phoenix Pedro outbound, scoring, and return/park routine with explicit policy.
 *
 * <p>This implementation is deliberately package-private and is constructed only by private
 * helpers in {@link PhoenixPedroAutoRoutineFactory}. It keeps season policy out of generic Task and
 * route APIs while preserving the precise {@link RouteStatus} of each route attempt.</p>
 *
 * <p>The three child Tasks are retained as one single-use graph. A completed outbound route permits
 * scoring; an outbound timeout skips scoring and selects the return route as a fallback; every
 * other abnormal outbound ending aborts. Scoring timeout still permits the return route while
 * retaining the degraded outcome. The return route never starts a replacement after failure.</p>
 */
final class PhoenixPedroAutoRoutineTask implements Task {

    private enum Phase {
        OUTBOUND,
        SCORING,
        RETURN_OR_PARK,
        DONE
    }

    private enum Decision {
        CONTINUE,
        FALLBACK,
        ABORT
    }

    private final String routineName;
    private final RouteTask<?> outboundRoute;
    private final Task scoringAttempt;
    private final RouteTask<?> returnOrParkRoute;
    private final PhoenixCapabilities.Scoring scoring;

    private boolean startAttempted;
    private boolean started;
    private boolean complete;
    private boolean flywheelDisableAttempted;
    private Phase phase = Phase.OUTBOUND;
    private Decision decision = Decision.CONTINUE;
    private Task activeChild;
    private TaskOutcome accumulatedOutcome = TaskOutcome.SUCCESS;
    private TaskOutcome finalOutcome = TaskOutcome.NOT_DONE;
    private RouteStatus lastRouteStatus = RouteStatus.NOT_STARTED;
    private RouteStatus triggeringRouteStatus;
    private String trigger = "NONE";

    /**
     * Creates one private Phoenix routine graph from three distinct, fresh child Tasks.
     *
     * @param routineName human-readable routine label used in current-task telemetry
     * @param outboundRoute route that must complete before scoring may begin
     * @param scoringAttempt Phoenix scoring attempt to run after successful outbound completion
     * @param returnOrParkRoute live-start return/park route used after scoring or as timeout fallback
     * @param scoring capability owner whose route-enabled flywheel request this routine disables
     */
    PhoenixPedroAutoRoutineTask(String routineName,
                                RouteTask<?> outboundRoute,
                                Task scoringAttempt,
                                RouteTask<?> returnOrParkRoute,
                                PhoenixCapabilities.Scoring scoring) {
        this.routineName = requireRoutineName(routineName);
        this.outboundRoute = requireRole(outboundRoute, "outboundRoute", routineName);
        this.scoringAttempt = requireRole(scoringAttempt, "scoringAttempt", routineName);
        this.returnOrParkRoute = requireRole(
                returnOrParkRoute,
                "returnOrParkRoute",
                routineName
        );
        this.scoring = requireRole(scoring, "scoring", routineName);
        requireDistinctChildren(this.outboundRoute, this.scoringAttempt, this.returnOrParkRoute);
    }

    @Override
    public void start(LoopClock clock) {
        markStartAttempt();
        started = true;
        phase = Phase.OUTBOUND;
        activeChild = outboundRoute;
        activeChild.start(clock);
        if (!complete) {
            advanceCompletedPhases(clock);
        }
    }

    @Override
    public void update(LoopClock clock) {
        if (!started) {
            throw new IllegalStateException(
                    "Phoenix Pedro routine '" + routineName + "' cannot be updated before "
                            + "start(clock). Enqueue the fresh Task returned by "
                            + "PhoenixPedroAutoRoutineFactory.build(...)."
            );
        }
        if (complete) {
            return;
        }

        Task child = activeChild;
        if (child == null) {
            throw malformedState("active phase " + phase + " has no active child");
        }

        boolean childWasComplete = child.isComplete();
        if (complete) {
            return;
        }
        if (activeChild != child) {
            // A child lifecycle callback re-entered this coordinator and selected another phase.
            // The nested call owns that transition; this stale outer update must do nothing.
            return;
        }
        if (childWasComplete) {
            advanceCompletedPhases(clock);
            return;
        }

        child.update(clock);
        if (!complete && activeChild == child) {
            advanceCompletedPhases(clock);
        }
    }

    @Override
    public void cancel() {
        if (!started || complete) {
            return;
        }

        Task childToCancel = activeChild;
        complete = true;
        phase = Phase.DONE;
        decision = Decision.ABORT;
        trigger = "DIRECT_CANCEL";
        finalOutcome = TaskOutcome.CANCELLED;

        RuntimeException firstFailure = null;
        try {
            if (childToCancel != null) {
                childToCancel.cancel();
            }
        } catch (RuntimeException childFailure) {
            firstFailure = childFailure;
        }

        try {
            disableFlywheelOnce();
        } catch (RuntimeException cleanupFailure) {
            firstFailure = retainFailure(firstFailure, cleanupFailure);
        }

        if (firstFailure != null) {
            throw firstFailure;
        }
    }

    @Override
    public boolean isComplete() {
        return complete;
    }

    @Override
    public TaskOutcome getOutcome() {
        return complete ? finalOutcome : TaskOutcome.NOT_DONE;
    }

    @Override
    public String getDebugName() {
        return routineName
                + "[phase=" + phase
                + ",decision=" + decision
                + ",trigger=" + trigger
                + "]";
    }

    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "phoenixAuto" : prefix;
        dbg.addData(p + ".name", getDebugName())
                .addData(p + ".routine", routineName)
                .addData(p + ".phase", phase)
                .addData(p + ".decision", decision)
                .addData(p + ".trigger", trigger)
                .addData(p + ".triggerRouteStatus",
                        triggeringRouteStatus == null ? "NONE" : triggeringRouteStatus)
                .addData(p + ".lastRouteStatus", lastRouteStatus)
                .addData(p + ".accumulatedOutcome", accumulatedOutcome)
                .addData(p + ".complete", complete)
                .addData(p + ".outcome", getOutcome())
                .addData(p + ".flywheelDisableAttempted", flywheelDisableAttempted);

        Task child = activeChild;
        if (child != null) {
            dbg.addData(p + ".activeChild", child.getDebugName())
                    .addData(p + ".activeChildOutcome", child.getOutcome());
            child.debugDump(dbg, p + ".active");
        }
    }

    /** Advance through child phases that completed at start or during the current update. */
    private void advanceCompletedPhases(LoopClock clock) {
        while (!complete) {
            Task completedChild = activeChild;
            if (completedChild == null) {
                throw malformedState("active phase " + phase + " has no active child");
            }
            boolean childComplete = completedChild.isComplete();
            if (complete) {
                return;
            }
            if (activeChild != completedChild) {
                // Reentrant policy already changed the phase. Inspect only the newly retained
                // child and never apply a stale child's terminal result twice.
                continue;
            }
            if (!childComplete) {
                return;
            }

            switch (phase) {
                case OUTBOUND:
                    handleOutboundTerminal(clock);
                    break;
                case SCORING:
                    handleScoringTerminal(clock);
                    break;
                case RETURN_OR_PARK:
                    handleReturnTerminal();
                    break;
                case DONE:
                default:
                    throw malformedState("completed loop reached terminal phase " + phase);
            }
        }
    }

    /** Apply Phoenix's explicit policy to the exact outbound route result. */
    private void handleOutboundTerminal(LoopClock clock) {
        RouteStatus status = requireTerminalRouteStatus(outboundRoute, "outboundRoute");
        lastRouteStatus = status;
        switch (status) {
            case COMPLETED:
                decision = Decision.CONTINUE;
                startPhase(Phase.SCORING, scoringAttempt, clock);
                return;
            case FOLLOWER_TIMEOUT_OR_STALL:
            case TASK_TIMEOUT:
                retainTimeout();
                selectRouteDecision(Decision.FALLBACK, status);
                disableFlywheelOnce();
                if (!complete
                        && phase == Phase.OUTBOUND
                        && activeChild == outboundRoute) {
                    startPhase(Phase.RETURN_OR_PARK, returnOrParkRoute, clock);
                }
                return;
            case INTERRUPTED:
            case REPLACED:
            case CANCELLED:
            case FAILED:
            case UNKNOWN_TERMINAL:
                selectRouteDecision(Decision.ABORT, status);
                finish(TaskOutcome.CANCELLED);
                disableFlywheelOnce();
                return;
            case NOT_STARTED:
            case ACTIVE:
            default:
                throw malformedRouteStatus("outboundRoute", status);
        }
    }

    /** Preserve scoring truth while choosing whether the live-start return route may begin. */
    private void handleScoringTerminal(LoopClock clock) {
        TaskOutcome outcome = requireCompletedOutcome(scoringAttempt, "scoringAttempt");
        // A custom child may re-enter robot policy while reporting its terminal outcome. Direct
        // cancellation is already terminal and must not be overwritten or start a return route.
        if (complete || phase != Phase.SCORING || activeChild != scoringAttempt) {
            return;
        }
        switch (outcome) {
            case SUCCESS:
                decision = Decision.CONTINUE;
                disableFlywheelOnce();
                if (!complete
                        && phase == Phase.SCORING
                        && activeChild == scoringAttempt) {
                    startPhase(Phase.RETURN_OR_PARK, returnOrParkRoute, clock);
                }
                return;
            case TIMEOUT:
                retainTimeout();
                decision = Decision.FALLBACK;
                trigger = "SCORING_TIMEOUT";
                disableFlywheelOnce();
                if (!complete
                        && phase == Phase.SCORING
                        && activeChild == scoringAttempt) {
                    startPhase(Phase.RETURN_OR_PARK, returnOrParkRoute, clock);
                }
                return;
            case CANCELLED:
            case UNKNOWN:
                decision = Decision.ABORT;
                trigger = "SCORING_" + outcome;
                finish(TaskOutcome.CANCELLED);
                disableFlywheelOnce();
                return;
            case NOT_DONE:
            default:
                throw malformedOutcome("scoringAttempt", outcome);
        }
    }

    /** Finish with the exact return result and any earlier retained degradation. */
    private void handleReturnTerminal() {
        RouteStatus status = requireTerminalRouteStatus(returnOrParkRoute, "returnOrParkRoute");
        lastRouteStatus = status;
        switch (status) {
            case COMPLETED:
                finish(accumulatedOutcome);
                return;
            case FOLLOWER_TIMEOUT_OR_STALL:
            case TASK_TIMEOUT:
                retainTimeout();
                if (triggeringRouteStatus == null) {
                    selectRouteDecision(Decision.ABORT, status);
                }
                finish(TaskOutcome.TIMEOUT);
                return;
            case INTERRUPTED:
            case REPLACED:
            case CANCELLED:
            case FAILED:
            case UNKNOWN_TERMINAL:
                selectRouteDecision(Decision.ABORT, status);
                finish(TaskOutcome.CANCELLED);
                return;
            case NOT_STARTED:
            case ACTIVE:
            default:
                throw malformedRouteStatus("returnOrParkRoute", status);
        }
    }

    /** Start a selected child only after its phase and identity are visible to reentrant cleanup. */
    private void startPhase(Phase nextPhase, Task nextChild, LoopClock clock) {
        phase = nextPhase;
        activeChild = nextChild;
        nextChild.start(clock);
    }

    /** Retain timeout degradation until a later cancellation-like result takes precedence. */
    private void retainTimeout() {
        accumulatedOutcome = TaskOutcome.TIMEOUT;
    }

    /** Record a route-derived fallback or abort decision for current-task telemetry. */
    private void selectRouteDecision(Decision selected, RouteStatus triggerStatus) {
        decision = selected;
        triggeringRouteStatus = triggerStatus;
        trigger = triggerStatus.name();
    }

    /** Make the routine terminal before any cleanup action that may throw. */
    private void finish(TaskOutcome outcome) {
        complete = true;
        phase = Phase.DONE;
        finalOutcome = outcome;
    }

    /** Disable only the flywheel request owned by this routine's outbound callback. */
    private void disableFlywheelOnce() {
        if (flywheelDisableAttempted) {
            return;
        }
        flywheelDisableAttempted = true;
        scoring.setFlywheelEnabled(false);
    }

    /** Read and validate one completed RouteTask's retained backend-neutral status. */
    private RouteStatus requireTerminalRouteStatus(RouteTask<?> route, String role) {
        RouteStatus status = route.getRouteStatus();
        if (status == null || status == RouteStatus.NOT_STARTED || status == RouteStatus.ACTIVE) {
            throw malformedRouteStatus(role, status);
        }
        return status;
    }

    /** Read and validate one completed non-route child outcome. */
    private TaskOutcome requireCompletedOutcome(Task child, String role) {
        TaskOutcome outcome = child.getOutcome();
        if (outcome == null || outcome == TaskOutcome.NOT_DONE) {
            throw malformedOutcome(role, outcome);
        }
        return outcome;
    }

    /** Record the one permitted start attempt before any child or capability effect. */
    private void markStartAttempt() {
        if (startAttempted) {
            throw new IllegalStateException(
                    "Phoenix Pedro routine '" + routineName + "' is single-use and "
                            + "start(clock) was called more than once. Build a fresh Task graph "
                            + "with PhoenixPedroAutoRoutineFactory.build(...)."
            );
        }
        startAttempted = true;
    }

    private IllegalStateException malformedRouteStatus(String role, RouteStatus status) {
        return new IllegalStateException(
                "Phoenix Pedro routine '" + routineName + "' completed " + role
                        + " with invalid terminal status " + status + ". The retained RouteTask "
                        + "must report a terminal RouteStatus before policy is selected."
        );
    }

    private IllegalStateException malformedOutcome(String role, TaskOutcome outcome) {
        return new IllegalStateException(
                "Phoenix Pedro routine '" + routineName + "' completed " + role
                        + " with invalid terminal outcome " + outcome + ". Return SUCCESS, "
                        + "TIMEOUT, CANCELLED, or UNKNOWN from the fresh child Task graph."
        );
    }

    private IllegalStateException malformedState(String detail) {
        return new IllegalStateException(
                "Phoenix Pedro routine '" + routineName + "' has malformed lifecycle state: "
                        + detail + ". Build a fresh Task graph with "
                        + "PhoenixPedroAutoRoutineFactory.build(...)."
        );
    }

    private void requireDistinctChildren(Task outbound, Task scoringTask, Task returnTask) {
        if (outbound == scoringTask) {
            throw duplicateRole("outboundRoute", "scoringAttempt");
        }
        if (outbound == returnTask) {
            throw duplicateRole("outboundRoute", "returnOrParkRoute");
        }
        if (scoringTask == returnTask) {
            throw duplicateRole("scoringAttempt", "returnOrParkRoute");
        }
    }

    private IllegalArgumentException duplicateRole(String firstRole, String secondRole) {
        return new IllegalArgumentException(
                "Phoenix Pedro routine '" + routineName + "' requires distinct Task instances, "
                        + "but " + firstRole + " and " + secondRole + " reference the same Task. "
                        + "Build each role as a fresh Task from its existing factory."
        );
    }

    private static String requireRoutineName(String value) {
        if (value == null || value.trim().isEmpty()) {
            throw new IllegalArgumentException(
                    "routineName is required for Phoenix Pedro route-policy telemetry"
            );
        }
        return value;
    }

    private static <T> T requireRole(T value, String role, String routineName) {
        if (value == null) {
            throw new IllegalArgumentException(
                    "Phoenix Pedro routine '" + routineName + "' requires " + role
                            + ". Build a fresh Task graph with the existing routine factory."
            );
        }
        return value;
    }

    /** Retain one cleanup failure and suppress later distinct failures on it. */
    private static RuntimeException retainFailure(RuntimeException first,
                                                  RuntimeException later) {
        if (first == null) {
            return later;
        }
        if (later != first) {
            first.addSuppressed(later);
        }
        return first;
    }
}
