package edu.ftcphoenix.robots.phoenix.autonomous.pedro;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.drive.route.RouteStatus;
import edu.ftcphoenix.fw.drive.route.RouteTask;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;
import edu.ftcphoenix.robots.phoenix.PhoenixCapabilities;

/**
 * Runs the portion of one Phoenix Pedro routine that is allowed to consume the pre-park budget.
 *
 * <p>This package-private policy owner preserves PHX-03's route/scoring decisions while giving the
 * outer routine one precise Task to bound with {@code Tasks.withTimeout(...)}. A successful
 * outbound route permits scoring. An outbound timeout skips scoring and completes this Task with
 * {@link TaskOutcome#TIMEOUT}. Scoring success or timeout likewise ends the pre-park phase with a
 * park-eligible outcome. Cancellation-like route results and scoring cancellation/unknown results
 * fail closed with {@link TaskOutcome#CANCELLED}.</p>
 *
 * <p>{@link #start(LoopClock)} only arms the phase. The outbound route starts on the first update
 * from a later loop cycle, after Phoenix's localization, targeting, and Pedro heartbeat phases have
 * run. An update in the start cycle is deliberately a no-op.</p>
 */
final class PhoenixPedroPreParkTask implements Task {

    private enum Phase {
        ARMED,
        OUTBOUND,
        SCORING,
        DONE
    }

    private final String routineName;
    private final RouteTask<?> outboundRoute;
    private final Task scoringAttempt;
    private final PhoenixCapabilities.Scoring scoring;
    private final DriveCommandSink driveSink;

    private boolean startAttempted;
    private boolean started;
    private boolean updateInProgress;
    private boolean fullCleanupAttempted;
    private boolean flywheelDisableAttempted;
    private boolean activeCancellationObserved;
    private boolean parkPermittedAfterActiveCancellation;
    private long armedCycle;
    private long lastUpdateCycle = Long.MIN_VALUE;
    private Phase phase = Phase.ARMED;
    private Task activeChild;
    private TaskOutcome outcome = TaskOutcome.NOT_DONE;
    private RouteStatus lastRouteStatus = RouteStatus.NOT_STARTED;
    private String trigger = "NOT_STARTED";

    PhoenixPedroPreParkTask(String routineName,
                            RouteTask<?> outboundRoute,
                            Task scoringAttempt,
                            PhoenixCapabilities.Scoring scoring,
                            DriveCommandSink driveSink) {
        this.routineName = requireRoutineName(routineName);
        this.outboundRoute = requireRole(outboundRoute, "outboundRoute", routineName);
        this.scoringAttempt = requireRole(scoringAttempt, "scoringAttempt", routineName);
        this.scoring = requireRole(scoring, "scoring", routineName);
        this.driveSink = requireRole(driveSink, "driveSink", routineName);
        if (outboundRoute == scoringAttempt) {
            throw duplicateRole("outboundRoute", "scoringAttempt");
        }
    }

    @Override
    public void start(LoopClock clock) {
        markStartAttempt();
        Objects.requireNonNull(clock, "Phoenix Pedro pre-park start clock is required");
        started = true;
        phase = Phase.ARMED;
        activeChild = null;
        outcome = TaskOutcome.NOT_DONE;
        lastRouteStatus = RouteStatus.NOT_STARTED;
        trigger = "ARMED";
        activeCancellationObserved = false;
        parkPermittedAfterActiveCancellation = false;
        armedCycle = clock.cycle();
        lastUpdateCycle = Long.MIN_VALUE;
    }

    @Override
    public void update(LoopClock clock) {
        if (!started) {
            throw new IllegalStateException(
                    "Phoenix Pedro pre-park phase '" + routineName + "' cannot be updated before "
                            + "start(clock). Start the fresh Task returned by "
                            + "PhoenixPedroAutoRoutineFactory.build(...)."
            );
        }
        if (phase == Phase.DONE) {
            return;
        }
        Objects.requireNonNull(clock, "Phoenix Pedro pre-park update clock is required");
        if (updateInProgress || lastUpdateCycle == clock.cycle()) {
            return;
        }

        // startAuto() arms the root at FTC START. Do not begin route behavior until a later loop,
        // after the composition root has refreshed localization, targeting, and Pedro once.
        if (phase == Phase.ARMED && clock.cycle() == armedCycle) {
            return;
        }

        lastUpdateCycle = clock.cycle();
        updateInProgress = true;
        try {
            if (phase == Phase.ARMED) {
                startPhase(Phase.OUTBOUND, outboundRoute, clock);
                advanceCompletedPhases(clock);
                return;
            }

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

        Phase cancelledPhase = phase;
        Task childToCancel = activeChild;

        // Publish the ordinary cutoff ending before reading any child callback. A child that became
        // terminal between Phoenix's Pedro/scoring heartbeat and this timeout cancellation may
        // narrow that decision below; cancellation-like evidence must suppress the park.
        activeCancellationObserved = true;
        parkPermittedAfterActiveCancellation = true;
        finish(TaskOutcome.CANCELLED, "ACTIVE_CANCEL");

        RuntimeException firstFailure = null;
        firstFailure = classifyTerminalChildAtCancellation(firstFailure, cancelledPhase);
        firstFailure = cancelChild(firstFailure, childToCancel);
        firstFailure = clearAutoOwnedState(firstFailure);
        if (firstFailure != null) {
            throw firstFailure;
        }
    }

    @Override
    public boolean isComplete() {
        return phase == Phase.DONE;
    }

    @Override
    public TaskOutcome getOutcome() {
        return phase == Phase.DONE ? outcome : TaskOutcome.NOT_DONE;
    }

    @Override
    public String getDebugName() {
        return routineName + ".prePark[phase=" + phase + ",trigger=" + trigger + "]";
    }

    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "phoenixAuto.prePark" : prefix;
        dbg.addData(p + ".name", getDebugName())
                .addData(p + ".routine", routineName)
                .addData(p + ".phase", phase)
                .addData(p + ".trigger", trigger)
                .addData(p + ".lastRouteStatus", lastRouteStatus)
                .addData(p + ".armedCycle", armedCycle)
                .addData(p + ".lastUpdateCycle", lastUpdateCycle)
                .addData(p + ".fullCleanupAttempted", fullCleanupAttempted)
                .addData(p + ".flywheelDisableAttempted", flywheelDisableAttempted)
                .addData(p + ".complete", isComplete())
                .addData(p + ".outcome", getOutcome());
        if (activeChild != null) {
            dbg.addData(p + ".activeChild", activeChild.getDebugName())
                    .addData(p + ".activeChildOutcome", activeChild.getOutcome());
            activeChild.debugDump(dbg, p + ".active");
        }
    }

    String terminalTrigger() {
        return trigger;
    }

    RouteStatus lastRouteStatus() {
        return lastRouteStatus;
    }

    boolean permitsParkAfterActiveCancellation() {
        return activeCancellationObserved && parkPermittedAfterActiveCancellation;
    }

    boolean activeCancellationObserved() {
        return activeCancellationObserved;
    }

    boolean directlyContains(Task task) {
        return task == outboundRoute || task == scoringAttempt;
    }

    /** Advance through children that completed during start or the current update. */
    private void advanceCompletedPhases(LoopClock clock) {
        while (phase != Phase.DONE && phase != Phase.ARMED) {
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
                case OUTBOUND:
                    handleOutboundTerminal(clock);
                    break;
                case SCORING:
                    handleScoringTerminal();
                    break;
                case ARMED:
                case DONE:
                default:
                    throw malformedState("completed-loop reached phase " + phase);
            }
        }
    }

    private void handleOutboundTerminal(LoopClock clock) {
        RouteStatus status = requireTerminalRouteStatus(outboundRoute, "outboundRoute");
        if (phase != Phase.OUTBOUND || activeChild != outboundRoute) {
            return;
        }
        lastRouteStatus = status;
        switch (status) {
            case COMPLETED:
                trigger = "OUTBOUND_COMPLETED";
                startPhase(Phase.SCORING, scoringAttempt, clock);
                return;
            case FOLLOWER_TIMEOUT_OR_STALL:
            case TASK_TIMEOUT:
                finish(TaskOutcome.TIMEOUT, status.name());
                disableFlywheelOnce();
                return;
            case INTERRUPTED:
            case REPLACED:
            case CANCELLED:
            case FAILED:
            case UNKNOWN_TERMINAL:
                finish(TaskOutcome.CANCELLED, status.name());
                disableFlywheelOnce();
                return;
            case NOT_STARTED:
            case ACTIVE:
            default:
                throw malformedRouteStatus("outboundRoute", status);
        }
    }

    private void handleScoringTerminal() {
        TaskOutcome scoringOutcome = requireCompletedOutcome(scoringAttempt, "scoringAttempt");
        if (phase != Phase.SCORING || activeChild != scoringAttempt) {
            return;
        }
        switch (scoringOutcome) {
            case SUCCESS:
                finish(TaskOutcome.SUCCESS, "SCORING_SUCCESS");
                disableFlywheelOnce();
                return;
            case TIMEOUT:
                finish(TaskOutcome.TIMEOUT, "SCORING_TIMEOUT");
                disableFlywheelOnce();
                return;
            case CANCELLED:
            case UNKNOWN:
                finish(TaskOutcome.CANCELLED, "SCORING_" + scoringOutcome);
                disableFlywheelOnce();
                return;
            case NOT_DONE:
            default:
                throw malformedOutcome("scoringAttempt", scoringOutcome);
        }
    }

    /** Publish the next child before invoking its reentrant start hook. */
    private void startPhase(Phase nextPhase, Task nextChild, LoopClock clock) {
        phase = nextPhase;
        activeChild = nextChild;
        nextChild.start(clock);
    }

    /** Become terminal before any cleanup callback can throw or reenter. */
    private void finish(TaskOutcome terminalOutcome, String terminalTrigger) {
        outcome = terminalOutcome;
        trigger = terminalTrigger;
        phase = Phase.DONE;
    }

    /** Active cancellation attempts every owned safety action exactly once. */
    private RuntimeException clearAutoOwnedState(RuntimeException firstFailure) {
        if (fullCleanupAttempted) {
            return firstFailure;
        }
        fullCleanupAttempted = true;

        firstFailure = attempt(firstFailure, new Runnable() {
            @Override
            public void run() {
                scoring.cancelTransientActions();
            }
        });
        firstFailure = attempt(firstFailure, new Runnable() {
            @Override
            public void run() {
                scoring.setIntakeEnabled(false);
            }
        });
        firstFailure = attempt(firstFailure, new Runnable() {
            @Override
            public void run() {
                scoring.setShootingEnabled(false);
            }
        });
        firstFailure = attempt(firstFailure, new Runnable() {
            @Override
            public void run() {
                scoring.setEjectEnabled(false);
            }
        });
        if (!flywheelDisableAttempted) {
            flywheelDisableAttempted = true;
            firstFailure = attempt(firstFailure, new Runnable() {
                @Override
                public void run() {
                    scoring.setFlywheelEnabled(false);
                }
            });
        }
        return attempt(firstFailure, new Runnable() {
            @Override
            public void run() {
                driveSink.stop();
            }
        });
    }

    private RuntimeException cancelChild(RuntimeException firstFailure, final Task child) {
        if (child == null) {
            return firstFailure;
        }
        return attempt(firstFailure, new Runnable() {
            @Override
            public void run() {
                child.cancel();
            }
        });
    }

    /**
     * Preserve terminal evidence produced by the current-cycle heartbeat before the outer timeout
     * could update this policy Task. Success still permits the bounded fallback; cancellation-like
     * evidence fails closed and prevents B from starting.
     */
    private RuntimeException classifyTerminalChildAtCancellation(RuntimeException firstFailure,
                                                                  Phase cancelledPhase) {
        try {
            switch (cancelledPhase) {
                case ARMED:
                    return firstFailure;
                case OUTBOUND:
                    classifyOutboundAtCancellation();
                    return firstFailure;
                case SCORING:
                    classifyScoringAtCancellation();
                    return firstFailure;
                case DONE:
                default:
                    return firstFailure;
            }
        } catch (RuntimeException classificationFailure) {
            parkPermittedAfterActiveCancellation = false;
            return retainFailure(firstFailure, classificationFailure);
        }
    }

    private void classifyOutboundAtCancellation() {
        RouteStatus status = outboundRoute.getRouteStatus();
        if (status == null) {
            throw malformedRouteStatus("outboundRoute", null);
        }
        lastRouteStatus = status;
        switch (status) {
            case NOT_STARTED:
            case ACTIVE:
            case COMPLETED:
                return;
            case FOLLOWER_TIMEOUT_OR_STALL:
            case TASK_TIMEOUT:
                finish(TaskOutcome.TIMEOUT, status.name());
                return;
            case INTERRUPTED:
            case REPLACED:
            case CANCELLED:
            case FAILED:
            case UNKNOWN_TERMINAL:
                parkPermittedAfterActiveCancellation = false;
                finish(TaskOutcome.CANCELLED, status.name());
                return;
            default:
                throw malformedRouteStatus("outboundRoute", status);
        }
    }

    private void classifyScoringAtCancellation() {
        boolean scoringComplete = scoringAttempt.isComplete();
        TaskOutcome scoringOutcome = scoringAttempt.getOutcome();
        if (scoringOutcome == null
                || (scoringComplete && scoringOutcome == TaskOutcome.NOT_DONE)
                || (!scoringComplete && scoringOutcome != TaskOutcome.NOT_DONE)) {
            throw inconsistentCancellationSnapshot(
                    "scoringAttempt",
                    scoringComplete,
                    scoringOutcome
            );
        }
        if (!scoringComplete) {
            return;
        }
        switch (scoringOutcome) {
            case SUCCESS:
                return;
            case TIMEOUT:
                finish(TaskOutcome.TIMEOUT, "SCORING_TIMEOUT");
                return;
            case CANCELLED:
            case UNKNOWN:
                parkPermittedAfterActiveCancellation = false;
                finish(TaskOutcome.CANCELLED, "SCORING_" + scoringOutcome);
                return;
            case NOT_DONE:
            default:
                throw malformedOutcome("scoringAttempt", scoringOutcome);
        }
    }

    /** Disable the route/scoring-owned flywheel request after every natural pre-park ending. */
    private void disableFlywheelOnce() {
        if (flywheelDisableAttempted) {
            return;
        }
        flywheelDisableAttempted = true;
        scoring.setFlywheelEnabled(false);
    }

    private RouteStatus requireTerminalRouteStatus(RouteTask<?> route, String role) {
        RouteStatus status = route.getRouteStatus();
        if (status == null || status == RouteStatus.NOT_STARTED || status == RouteStatus.ACTIVE) {
            throw malformedRouteStatus(role, status);
        }
        return status;
    }

    private TaskOutcome requireCompletedOutcome(Task task, String role) {
        TaskOutcome childOutcome = task.getOutcome();
        if (childOutcome == null || childOutcome == TaskOutcome.NOT_DONE) {
            throw malformedOutcome(role, childOutcome);
        }
        return childOutcome;
    }

    private void markStartAttempt() {
        if (startAttempted) {
            throw new IllegalStateException(
                    "Phoenix Pedro pre-park phase '" + routineName + "' is single-use and "
                            + "start(clock) was called more than once. Build a fresh Task graph "
                            + "with PhoenixPedroAutoRoutineFactory.build(...)."
            );
        }
        startAttempted = true;
    }

    private IllegalStateException malformedRouteStatus(String role, RouteStatus status) {
        return new IllegalStateException(
                "Phoenix Pedro pre-park phase '" + routineName + "' completed " + role
                        + " with invalid terminal status " + status + ". The retained RouteTask "
                        + "must report a terminal RouteStatus before policy is selected."
        );
    }

    private IllegalStateException malformedOutcome(String role, TaskOutcome childOutcome) {
        return new IllegalStateException(
                "Phoenix Pedro pre-park phase '" + routineName + "' completed " + role
                        + " with invalid terminal outcome " + childOutcome + ". Return SUCCESS, "
                        + "TIMEOUT, CANCELLED, or UNKNOWN from the fresh child Task graph."
        );
    }

    private IllegalStateException inconsistentCancellationSnapshot(String role,
                                                                    boolean childComplete,
                                                                    TaskOutcome childOutcome) {
        return new IllegalStateException(
                "Phoenix Pedro pre-park phase '" + routineName + "' observed inconsistent "
                        + role + " lifecycle while applying active cancellation: isComplete="
                        + childComplete + ", outcome=" + childOutcome + ". An active Task must "
                        + "report NOT_DONE, while a terminal Task must report SUCCESS, TIMEOUT, "
                        + "CANCELLED, or UNKNOWN. Fix the fresh child Task graph."
        );
    }

    private IllegalStateException malformedState(String detail) {
        return new IllegalStateException(
                "Phoenix Pedro pre-park phase '" + routineName
                        + "' has malformed lifecycle state: " + detail + ". Build a fresh Task "
                        + "graph with PhoenixPedroAutoRoutineFactory.build(...)."
        );
    }

    private IllegalArgumentException duplicateRole(String firstRole, String secondRole) {
        return new IllegalArgumentException(
                "Phoenix Pedro pre-park phase '" + routineName + "' requires distinct Task "
                        + "instances, but " + firstRole + " and " + secondRole
                        + " reference the same Task. Build each role as a fresh Task."
        );
    }

    private static RuntimeException attempt(RuntimeException firstFailure, Runnable action) {
        try {
            action.run();
        } catch (RuntimeException laterFailure) {
            return retainFailure(firstFailure, laterFailure);
        }
        return firstFailure;
    }

    private static RuntimeException retainFailure(RuntimeException firstFailure,
                                                  RuntimeException laterFailure) {
        if (firstFailure == null) {
            return laterFailure;
        }
        if (laterFailure != firstFailure) {
            firstFailure.addSuppressed(laterFailure);
        }
        return firstFailure;
    }

    private static String requireRoutineName(String value) {
        if (value == null || value.trim().isEmpty()) {
            throw new IllegalArgumentException(
                    "routineName is required for Phoenix Pedro pre-park telemetry"
            );
        }
        return value;
    }

    private static <T> T requireRole(T value, String role, String routineName) {
        if (value == null) {
            throw new IllegalArgumentException(
                    "Phoenix Pedro pre-park phase '" + routineName + "' requires " + role
                            + ". Build a fresh Task graph with the existing routine factory."
            );
        }
        return value;
    }
}
