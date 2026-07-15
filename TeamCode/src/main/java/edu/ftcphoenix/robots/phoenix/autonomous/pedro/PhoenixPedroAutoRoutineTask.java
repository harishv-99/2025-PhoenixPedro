package edu.ftcphoenix.robots.phoenix.autonomous.pedro;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.route.RouteStatus;
import edu.ftcphoenix.fw.drive.route.RouteTask;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;
import edu.ftcphoenix.fw.task.Tasks;

/**
 * Coordinates one bounded Phoenix pre-park phase and one live-start return/park route.
 *
 * <p>The pre-park policy is isolated in {@link PhoenixPedroPreParkTask} and bounded by one generic
 * {@link Tasks#withTimeout(Task, double)} decorator. A natural pre-park success, a local PHX-03
 * timeout, or a successful match-time cutoff permits the one return/park Task to start. Direct
 * cancellation, cleanup/lifecycle failure, and cancellation-like pre-park results never start it.
 * Once the return/park Task begins, the completed timeout decorator no longer owns it, so crossing
 * the match threshold cannot interrupt or restart an already-running park.</p>
 */
final class PhoenixPedroAutoRoutineTask implements Task {

    private enum Phase {
        PRE_PARK,
        RETURN_OR_PARK,
        DONE
    }

    private enum Decision {
        CONTINUE,
        FALLBACK,
        ABORT
    }

    private final String routineName;
    private final PhoenixPedroPreParkTask prePark;
    private final Task boundedPrePark;
    private final RouteTask<?> returnOrParkRoute;
    private final double parkTakeoverElapsedSec;

    private boolean startAttempted;
    private boolean started;
    private boolean updateInProgress;
    private long lastUpdateCycle = Long.MIN_VALUE;
    private Phase phase = Phase.PRE_PARK;
    private Decision decision = Decision.CONTINUE;
    private Task activeChild;
    private TaskOutcome accumulatedOutcome = TaskOutcome.SUCCESS;
    private TaskOutcome finalOutcome = TaskOutcome.NOT_DONE;
    private RouteStatus lastRouteStatus = RouteStatus.NOT_STARTED;
    private boolean matchTimeCutoff;
    private String trigger = "NOT_STARTED";

    /**
     * Creates the one private root graph behind {@link PhoenixPedroAutoRoutineFactory#build}.
     */
    PhoenixPedroAutoRoutineTask(String routineName,
                                PhoenixPedroPreParkTask prePark,
                                double parkTakeoverElapsedSec,
                                RouteTask<?> returnOrParkRoute) {
        this.routineName = requireRoutineName(routineName);
        this.prePark = requireRole(prePark, "prePark", routineName);
        this.returnOrParkRoute = requireRole(
                returnOrParkRoute,
                "returnOrParkRoute",
                routineName
        );
        if (!Double.isFinite(parkTakeoverElapsedSec) || parkTakeoverElapsedSec <= 0.0) {
            throw new IllegalArgumentException(
                    "Phoenix Pedro routine '" + routineName
                            + "' parkTakeoverElapsedSec must be finite and > 0, got "
                            + parkTakeoverElapsedSec
            );
        }
        if (prePark.directlyContains(returnOrParkRoute)) {
            throw new IllegalArgumentException(
                    "Phoenix Pedro routine '" + routineName + "' requires a distinct live-start "
                            + "returnOrParkRoute; it cannot reuse a pre-park child Task. Build every "
                            + "route role from a fresh RouteTasks factory call."
            );
        }
        this.parkTakeoverElapsedSec = parkTakeoverElapsedSec;
        this.boundedPrePark = Tasks.withTimeout(prePark, parkTakeoverElapsedSec);
    }

    @Override
    public void start(LoopClock clock) {
        markStartAttempt();
        Objects.requireNonNull(clock, "Phoenix Pedro routine start clock is required");
        started = true;
        phase = Phase.PRE_PARK;
        decision = Decision.CONTINUE;
        activeChild = boundedPrePark;
        accumulatedOutcome = TaskOutcome.SUCCESS;
        finalOutcome = TaskOutcome.NOT_DONE;
        lastRouteStatus = RouteStatus.NOT_STARTED;
        matchTimeCutoff = false;
        trigger = "PRE_PARK_ARMED";
        lastUpdateCycle = Long.MIN_VALUE;
        boundedPrePark.start(clock);
    }

    @Override
    public void update(LoopClock clock) {
        if (!started) {
            throw new IllegalStateException(
                    "Phoenix Pedro routine '" + routineName + "' cannot be updated before "
                            + "start(clock). Start the fresh Task returned by "
                            + "PhoenixPedroAutoRoutineFactory.build(...)."
            );
        }
        if (phase == Phase.DONE) {
            return;
        }
        Objects.requireNonNull(clock, "Phoenix Pedro routine update clock is required");
        if (updateInProgress || lastUpdateCycle == clock.cycle()) {
            return;
        }

        lastUpdateCycle = clock.cycle();
        updateInProgress = true;
        try {
            advanceCompletedPhases(clock);
            if (phase == Phase.DONE) {
                return;
            }

            Task child = activeChild;
            if (child == null) {
                throw malformedState("active phase " + phase + " has no active child");
            }
            child.update(clock);
            if (phase != Phase.DONE && activeChild == child) {
                advanceCompletedPhases(clock);
            }
        } finally {
            updateInProgress = false;
        }
    }

    @Override
    public void cancel() {
        if (!started || phase == Phase.DONE) {
            return;
        }

        Task childToCancel = activeChild;
        finish(TaskOutcome.CANCELLED, Decision.ABORT, "DIRECT_CANCEL");
        if (childToCancel != null) {
            childToCancel.cancel();
        }
    }

    @Override
    public boolean isComplete() {
        return phase == Phase.DONE;
    }

    @Override
    public TaskOutcome getOutcome() {
        return phase == Phase.DONE ? finalOutcome : TaskOutcome.NOT_DONE;
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
                .addData(p + ".matchTimeCutoff", matchTimeCutoff)
                .addData(p + ".parkTakeoverElapsedSec", parkTakeoverElapsedSec)
                .addData(p + ".lastRouteStatus", lastRouteStatus)
                .addData(p + ".preParkOutcome", prePark.getOutcome())
                .addData(p + ".preParkTrigger", prePark.terminalTrigger())
                .addData(p + ".accumulatedOutcome", accumulatedOutcome)
                .addData(p + ".complete", isComplete())
                .addData(p + ".outcome", getOutcome());

        if (activeChild != null) {
            dbg.addData(p + ".activeChild", activeChild.getDebugName())
                    .addData(p + ".activeChildOutcome", activeChild.getOutcome());
            activeChild.debugDump(dbg, p + ".active");
        }
        prePark.debugDump(dbg, p + ".prePark");
    }

    /** Advance through children that completed at start or during the current update. */
    private void advanceCompletedPhases(LoopClock clock) {
        while (phase != Phase.DONE) {
            Task child = activeChild;
            if (child == null) {
                throw malformedState("active phase " + phase + " has no active child");
            }
            boolean childComplete = child.isComplete();
            if (phase == Phase.DONE || activeChild != child) {
                return;
            }
            if (!childComplete) {
                return;
            }

            switch (phase) {
                case PRE_PARK:
                    handlePreParkTerminal(clock);
                    break;
                case RETURN_OR_PARK:
                    handleReturnTerminal();
                    break;
                case DONE:
                default:
                    throw malformedState("completed-loop reached terminal phase " + phase);
            }
        }
    }

    /** Permit one park only after an explicitly allowed pre-park ending. */
    private void handlePreParkTerminal(LoopClock clock) {
        TaskOutcome boundedOutcome = requireCompletedOutcome(boundedPrePark, "boundedPrePark");
        if (phase != Phase.PRE_PARK || activeChild != boundedPrePark) {
            return;
        }
        TaskOutcome preParkOutcome = requireCompletedOutcome(prePark, "prePark");
        if (phase != Phase.PRE_PARK || activeChild != boundedPrePark) {
            return;
        }
        lastRouteStatus = prePark.lastRouteStatus();

        switch (boundedOutcome) {
            case SUCCESS:
                if (preParkOutcome != TaskOutcome.SUCCESS) {
                    throw inconsistentPreParkOutcomes(boundedOutcome, preParkOutcome);
                }
                decision = Decision.CONTINUE;
                trigger = prePark.terminalTrigger();
                startReturnOrPark(clock);
                return;

            case TIMEOUT:
                if (preParkOutcome == TaskOutcome.TIMEOUT) {
                    accumulatedOutcome = TaskOutcome.TIMEOUT;
                    decision = Decision.FALLBACK;
                    trigger = prePark.terminalTrigger();
                    matchTimeCutoff = prePark.activeCancellationObserved();
                    startReturnOrPark(clock);
                    return;
                }
                if (preParkOutcome == TaskOutcome.CANCELLED) {
                    if (!prePark.permitsParkAfterActiveCancellation()) {
                        finish(TaskOutcome.CANCELLED, Decision.ABORT, prePark.terminalTrigger());
                        return;
                    }
                    accumulatedOutcome = TaskOutcome.TIMEOUT;
                    decision = Decision.FALLBACK;
                    matchTimeCutoff = true;
                    trigger = "MATCH_TIME_CUTOFF";
                    startReturnOrPark(clock);
                    return;
                }
                throw inconsistentPreParkOutcomes(boundedOutcome, preParkOutcome);

            case CANCELLED:
            case UNKNOWN:
                finish(TaskOutcome.CANCELLED, Decision.ABORT, prePark.terminalTrigger());
                return;

            case NOT_DONE:
            default:
                throw malformedOutcome("boundedPrePark", boundedOutcome);
        }
    }

    /** Publish the park phase before its live-start route factory can reenter. */
    private void startReturnOrPark(LoopClock clock) {
        phase = Phase.RETURN_OR_PARK;
        activeChild = returnOrParkRoute;
        try {
            returnOrParkRoute.start(clock);
        } catch (RuntimeException startFailure) {
            // RouteTask terminalizes itself before rethrowing a construction/follower-start
            // failure. Terminalize this root too so TaskRunner's fail-stop cancellation cannot
            // overwrite the exact B-phase reason retained for telemetry.
            RouteStatus status = returnOrParkRoute.getRouteStatus();
            lastRouteStatus = status;
            String failureTrigger = status == null ? "RETURN_OR_PARK_START_FAILURE" : status.name();
            finish(TaskOutcome.CANCELLED, Decision.ABORT, failureTrigger);
            throw startFailure;
        }
    }

    /** Finish with the exact final-route result and any retained pre-park degradation. */
    private void handleReturnTerminal() {
        RouteStatus status = requireTerminalRouteStatus(returnOrParkRoute, "returnOrParkRoute");
        if (phase != Phase.RETURN_OR_PARK || activeChild != returnOrParkRoute) {
            return;
        }
        lastRouteStatus = status;
        switch (status) {
            case COMPLETED:
                finish(accumulatedOutcome, decision, trigger);
                return;
            case FOLLOWER_TIMEOUT_OR_STALL:
            case TASK_TIMEOUT:
                accumulatedOutcome = TaskOutcome.TIMEOUT;
                finish(TaskOutcome.TIMEOUT, Decision.ABORT, status.name());
                return;
            case INTERRUPTED:
            case REPLACED:
            case CANCELLED:
            case FAILED:
            case UNKNOWN_TERMINAL:
                finish(TaskOutcome.CANCELLED, Decision.ABORT, status.name());
                return;
            case NOT_STARTED:
            case ACTIVE:
            default:
                throw malformedRouteStatus("returnOrParkRoute", status);
        }
    }

    /** Become terminal before cleanup callbacks can throw or reenter. */
    private void finish(TaskOutcome outcome, Decision finalDecision, String finalTrigger) {
        finalOutcome = outcome;
        decision = finalDecision;
        trigger = finalTrigger;
        phase = Phase.DONE;
    }

    private RouteStatus requireTerminalRouteStatus(RouteTask<?> route, String role) {
        RouteStatus status = route.getRouteStatus();
        if (status == null || status == RouteStatus.NOT_STARTED || status == RouteStatus.ACTIVE) {
            throw malformedRouteStatus(role, status);
        }
        return status;
    }

    private TaskOutcome requireCompletedOutcome(Task task, String role) {
        TaskOutcome outcome = task.getOutcome();
        if (outcome == null || outcome == TaskOutcome.NOT_DONE) {
            throw malformedOutcome(role, outcome);
        }
        return outcome;
    }

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

    private IllegalStateException inconsistentPreParkOutcomes(TaskOutcome boundedOutcome,
                                                              TaskOutcome preParkOutcome) {
        return new IllegalStateException(
                "Phoenix Pedro routine '" + routineName + "' observed inconsistent pre-park "
                        + "terminal outcomes: boundedPrePark=" + boundedOutcome
                        + ", prePark=" + preParkOutcome + ". A timeout decorator may retain the "
                        + "child outcome or report TIMEOUT after cancelling an active child; no "
                        + "other pairing may start the return/park route."
        );
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
                        + "TIMEOUT, CANCELLED, or UNKNOWN from every completed Task."
        );
    }

    private IllegalStateException malformedState(String detail) {
        return new IllegalStateException(
                "Phoenix Pedro routine '" + routineName + "' has malformed lifecycle state: "
                        + detail + ". Build a fresh Task graph with "
                        + "PhoenixPedroAutoRoutineFactory.build(...)."
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
}
