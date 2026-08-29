package edu.ftcphoenix.robots.examples.pedro.adaptive;

import com.pedropathing.paths.PathChain;

import java.util.Objects;
import java.util.function.Supplier;

import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.route.RouteFollower;
import edu.ftcphoenix.fw.drive.route.RouteStatus;
import edu.ftcphoenix.fw.drive.route.RouteTask;
import edu.ftcphoenix.fw.drive.route.RouteTasks;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.Tasks;
import edu.ftcphoenix.robots.examples.pedro.capability.intake.BasicPedroAutoMechanism;

/**
 * Coordinates one timestamped collection route, its intake lifetime, and one optional return.
 *
 * <p>The collection route freezes exactly one cached
 * {@link AdaptiveCollectionVisionService.Decision} when its start-time factory runs. An
 * unavailable decision is still a valid frozen input: {@link AdaptiveCollectionPaths} builds the
 * explicitly configured fallback route and {@link Status#usedFallback()} retains that provenance.
 * This owner does not resample vision after the collection route starts.</p>
 *
 * <p>An untimed semantic wait is the deadline for the outbound phase. It checks the retained
 * collection route status first, then the path's {@code nearEnd} milestone, then the conjunction
 * of {@code safeToLeave} and the supplied inventory source. The route and bounded intake are
 * companions, so a permitted early exit records its reason before both companions receive active,
 * cancellation-safe cleanup. Only normal endpoint completion, near-end progress, or full inventory
 * after safe progress may start the live-pose return route.</p>
 *
 * <p>{@link #task()} is the one stable single-use root for this attempt. Construct a fresh attempt
 * whenever the behavior should run again. {@link #status()} returns an immutable read-only
 * snapshot and never samples vision, inventory, a live route execution, or hardware. The root
 * Task's broad outcome describes whether the selected generic policy branch ran; the status's
 * exact exit reason and separate route statuses remain the authoritative attempt facts.</p>
 */
public final class AdaptiveCollectionAttempt {

    /** Data-only Task timeout configuration for one collection attempt. */
    public static final class Config {
        /**
         * Positive finite timeout for the collection route in seconds; the intake companion uses
         * the same bound so it cannot outlive the route budget.
         */
        public double collectionRouteTimeoutSec;

        /** Positive finite timeout for the live-pose return route in seconds. */
        public double returnRouteTimeoutSec;

        private Config() {
            // Use defaults() to begin from one complete software-valid example configuration.
        }

        /**
         * Returns a fresh software-valid baseline; the values are not physical safety claims.
         */
        public static Config defaults() {
            Config config = new Config();
            config.collectionRouteTimeoutSec = 4.0;
            config.returnRouteTimeoutSec = 4.0;
            return config;
        }
    }

    /** Why the collection phase ended, retained independently from later route cancellation. */
    public enum ExitReason {
        /** No collection exit has been selected yet. */
        NOT_FINISHED,

        /** The exact collection route execution reached its intended endpoint. */
        ROUTE_COMPLETED,

        /** The collection path published its semantic near-end milestone. */
        NEAR_END,

        /** Inventory became full after the path had published that leaving was safe. */
        INVENTORY_FULL_AFTER_SAFE,

        /** The follower reported its own timeout or stall safeguard. */
        FOLLOWER_TIMEOUT_OR_STALL,

        /** The collection Route Task's explicit timeout elapsed. */
        TASK_TIMEOUT,

        /** External policy or a callback interrupted the collection execution. */
        INTERRUPTED,

        /** A newer route replaced the exact collection execution. */
        REPLACED,

        /** The attempt or exact collection execution was actively cancelled. */
        CANCELLED,

        /** Route construction, route execution, or attempt-policy observation failed. */
        FAILED,

        /** The follower stopped without enough evidence to classify the ending. */
        UNKNOWN_TERMINAL
    }

    /** Immutable presenter snapshot for one adaptive collection attempt. */
    public static final class Status {
        private final boolean complete;
        private final AdaptiveCollectionVisionService.Decision frozenDecision;
        private final boolean safeToLeave;
        private final boolean nearEnd;
        private final boolean inventoryFull;
        private final ExitReason exitReason;
        private final RouteStatus collectionRouteStatus;
        private final RouteStatus returnRouteStatus;

        private Status(boolean complete,
                       AdaptiveCollectionVisionService.Decision frozenDecision,
                       boolean safeToLeave,
                       boolean nearEnd,
                       boolean inventoryFull,
                       ExitReason exitReason,
                       RouteStatus collectionRouteStatus,
                       RouteStatus returnRouteStatus) {
            this.complete = complete;
            this.frozenDecision = frozenDecision;
            this.safeToLeave = safeToLeave;
            this.nearEnd = nearEnd;
            this.inventoryFull = inventoryFull;
            this.exitReason = exitReason;
            this.collectionRouteStatus = collectionRouteStatus;
            this.returnRouteStatus = returnRouteStatus;
        }

        /** Returns whether the stable attempt Task has reached a terminal state. */
        public boolean complete() {
            return complete;
        }

        /** Returns whether the collection start-time factory has frozen its one vision decision. */
        public boolean decisionFrozen() {
            return frozenDecision != null;
        }

        /**
         * Returns the exact immutable decision frozen for collection route construction.
         *
         * @throws IllegalStateException if the collection route has not reached its start-time
         *                               factory
         */
        public AdaptiveCollectionVisionService.Decision frozenDecision() {
            if (frozenDecision == null) {
                throw new IllegalStateException(
                        "Adaptive collection decision is not frozen until the collection route "
                                + "starts. Check decisionFrozen() first."
                );
            }
            return frozenDecision;
        }

        /**
         * Returns whether the frozen unavailable decision selected the configured fallback route.
         *
         * @throws IllegalStateException if the collection decision is not frozen yet
         */
        public boolean usedFallback() {
            return !frozenDecision().hasSelection();
        }

        /** Returns the cached semantic safe-to-leave milestone. */
        public boolean safeToLeave() {
            return safeToLeave;
        }

        /** Returns the cached semantic near-end milestone. */
        public boolean nearEnd() {
            return nearEnd;
        }

        /** Returns the last inventory value sampled by the attempt deadline. */
        public boolean inventoryFull() {
            return inventoryFull;
        }

        /** Returns the exact retained collection-phase policy reason. */
        public ExitReason exitReason() {
            return exitReason;
        }

        /** Returns the current or terminal status retained for the exact collection route. */
        public RouteStatus collectionRouteStatus() {
            return collectionRouteStatus;
        }

        /** Returns the current or terminal status retained for the distinct return route. */
        public RouteStatus returnRouteStatus() {
            return returnRouteStatus;
        }
    }

    private static final String COLLECTION_ROUTE_NAME = "adaptiveCollection.collection";
    private static final String RETURN_ROUTE_NAME = "adaptiveCollection.return";

    private final Supplier<AdaptiveCollectionVisionService.Decision> decisionSource;
    private final AdaptiveCollectionPaths paths;
    private final BooleanSource inventoryFull;
    private final AdaptiveCollectionPaths.Milestones milestones;
    private final RouteTask<PathChain> collectionRoute;
    private final RouteTask<PathChain> returnRoute;
    private final Task rootTask;

    private AdaptiveCollectionVisionService.Decision frozenDecision;
    private boolean inventoryFullCached;
    private ExitReason exitReason = ExitReason.NOT_FINISHED;
    private RouteStatus collectionRouteStatus = RouteStatus.NOT_STARTED;
    private RouteStatus returnRouteStatus = RouteStatus.NOT_STARTED;

    /**
     * Constructs one complete, single-use adaptive collection attempt.
     *
     * <p>The mutable configuration is copied and validated before any Task is allocated. Task
     * construction does not read vision, inventory, current pose, or hardware. Both paths remain
     * start-built: collection freezes its decision and pose once, and return reads live pose only
     * after the collection companions have finished cancellation.</p>
     *
     * @param vision cached timestamped selection owner
     * @param paths Pedro geometry and semantic-milestone owner
     * @param inventoryFull cached robot-owned inventory fact
     * @param intake cancellation-safe Basic Pedro intake capability
     * @param config complete data-only attempt configuration
     */
    public AdaptiveCollectionAttempt(AdaptiveCollectionVisionService vision,
                                     AdaptiveCollectionPaths paths,
                                     BooleanSource inventoryFull,
                                     BasicPedroAutoMechanism intake,
                                     Config config) {
        this(decisionSourceOf(vision), paths, inventoryFull, intake, config);
    }

    /** Hardware-neutral package seam; production callers use the sole public owner constructor. */
    AdaptiveCollectionAttempt(
            Supplier<AdaptiveCollectionVisionService.Decision> decisionSource,
            AdaptiveCollectionPaths paths,
            BooleanSource inventoryFull,
            BasicPedroAutoMechanism intake,
            Config config) {
        Config source = Objects.requireNonNull(
                config,
                "AdaptiveCollectionAttempt.Config is required"
        );
        double copiedCollectionTimeoutSec = source.collectionRouteTimeoutSec;
        double copiedReturnTimeoutSec = source.returnRouteTimeoutSec;
        requirePositiveFinite(
                "collectionRouteTimeoutSec",
                copiedCollectionTimeoutSec
        );
        requirePositiveFinite("returnRouteTimeoutSec", copiedReturnTimeoutSec);

        this.decisionSource = Objects.requireNonNull(decisionSource, "decisionSource");
        this.paths = Objects.requireNonNull(paths, "paths");
        this.inventoryFull = Objects.requireNonNull(inventoryFull, "inventoryFull");
        BasicPedroAutoMechanism requiredIntake = Objects.requireNonNull(intake, "intake");
        milestones = paths.newMilestones();

        RouteFollower<PathChain> routeFollower = paths.routeFollower();
        collectionRoute = RouteTasks.followBuiltAtStart(
                COLLECTION_ROUTE_NAME,
                routeFollower,
                this::buildCollectionRoute,
                copiedCollectionTimeoutSec
        );
        returnRoute = RouteTasks.followBuiltAtStart(
                RETURN_ROUTE_NAME,
                routeFollower,
                this::buildReturnRoute,
                copiedReturnTimeoutSec
        );

        Task collectionDeadline = Tasks.waitUntil(new CollectionExitSource());
        Task collectionPhase = Tasks.parallelDeadline(
                collectionDeadline,
                collectionRoute,
                requiredIntake.collectTask(copiedCollectionTimeoutSec)
        );
        Task conditionalReturn = Tasks.buildAtStart(
                "adaptiveCollection.returnPolicy",
                this::buildReturnPolicy
        );
        rootTask = Tasks.sequence(collectionPhase, conditionalReturn);
    }

    /**
     * Returns this attempt's one stable single-use root Task.
     *
     * <p>Construct a new {@code AdaptiveCollectionAttempt} instead of starting this Task twice.</p>
     */
    public Task task() {
        return rootTask;
    }

    /**
     * Returns one immutable, side-effect-free status snapshot.
     *
     * <p>While a route is active, this method returns only state already observed by the Task
     * graph. Once a route is known terminal, reading its retained {@link RouteTask} status cannot
     * poll or advance the follower. This lets a direct outer cancellation truthfully appear as
     * {@link ExitReason#CANCELLED} without allowing a presenter to drive route policy.</p>
     */
    public Status status() {
        boolean rootComplete = rootTask.isComplete();
        RouteStatus exactCollectionStatus = collectionRouteStatus;
        if (rootComplete || exitReason != ExitReason.NOT_FINISHED) {
            exactCollectionStatus = collectionRoute.getRouteStatus();
        }
        RouteStatus exactReturnStatus = returnRouteStatus;
        if (rootComplete) {
            exactReturnStatus = returnRoute.getRouteStatus();
        }

        ExitReason snapshotExitReason = exitReason;
        if (snapshotExitReason == ExitReason.NOT_FINISHED && rootComplete) {
            ExitReason exactTerminalReason = exitReasonFor(exactCollectionStatus);
            if (exactTerminalReason != null) {
                snapshotExitReason = exactTerminalReason;
            }
        }

        return new Status(
                rootComplete,
                frozenDecision,
                milestones.safeToLeave(),
                milestones.nearEnd(),
                inventoryFullCached,
                snapshotExitReason,
                exactCollectionStatus,
                exactReturnStatus
        );
    }

    /** Freeze one cached decision and build one collection path at the Route Task start boundary. */
    private PathChain buildCollectionRoute() {
        AdaptiveCollectionVisionService.Decision decision = Objects.requireNonNull(
                decisionSource.get(),
                "adaptive collection decisionSource returned null"
        );
        frozenDecision = decision;
        PathChain route = paths.buildCollectionFromCurrentPose(decision, milestones);
        collectionRouteStatus = RouteStatus.ACTIVE;
        return route;
    }

    /** Build the return once from the current Pedro pose when its Route Task actually starts. */
    private PathChain buildReturnRoute() {
        PathChain route = paths.buildReturnFromCurrentPose();
        returnRouteStatus = RouteStatus.ACTIVE;
        return route;
    }

    /** Select one return graph only after the collection deadline finished companion cleanup. */
    private Task buildReturnPolicy() {
        if (!permitsReturn(exitReason)) {
            return Tasks.noop();
        }
        return returnRoute;
    }

    /** Clock-aware robot-policy source used by the framework's ordinary untimed wait Task. */
    private final class CollectionExitSource implements BooleanSource {
        @Override
        public boolean getAsBoolean(LoopClock clock) {
            Objects.requireNonNull(clock, "adaptive collection deadline clock");
            try {
                RouteStatus observedRouteStatus = collectionRoute.getRouteStatus();
                collectionRouteStatus = observedRouteStatus;

                ExitReason terminalRouteReason = exitReasonFor(observedRouteStatus);
                if (terminalRouteReason != null) {
                    exitReason = terminalRouteReason;
                    return true;
                }
                if (milestones.nearEnd()) {
                    exitReason = ExitReason.NEAR_END;
                    return true;
                }

                boolean observedInventoryFull = inventoryFull.getAsBoolean(clock);
                inventoryFullCached = observedInventoryFull;
                if (milestones.safeToLeave() && observedInventoryFull) {
                    exitReason = ExitReason.INVENTORY_FULL_AFTER_SAFE;
                    return true;
                }
                return false;
            } catch (RuntimeException failure) {
                exitReason = ExitReason.FAILED;
                throw failure;
            }
        }
    }

    /** Map one exact collection route ending to the attempt policy vocabulary. */
    private static ExitReason exitReasonFor(RouteStatus routeStatus) {
        if (routeStatus == null) {
            return ExitReason.FAILED;
        }
        switch (routeStatus) {
            case NOT_STARTED:
            case ACTIVE:
                return null;
            case COMPLETED:
                return ExitReason.ROUTE_COMPLETED;
            case FOLLOWER_TIMEOUT_OR_STALL:
                return ExitReason.FOLLOWER_TIMEOUT_OR_STALL;
            case INTERRUPTED:
                return ExitReason.INTERRUPTED;
            case REPLACED:
                return ExitReason.REPLACED;
            case TASK_TIMEOUT:
                return ExitReason.TASK_TIMEOUT;
            case CANCELLED:
                return ExitReason.CANCELLED;
            case FAILED:
                return ExitReason.FAILED;
            case UNKNOWN_TERMINAL:
                return ExitReason.UNKNOWN_TERMINAL;
            default:
                throw new IllegalStateException(
                        "Unhandled adaptive collection RouteStatus " + routeStatus
                );
        }
    }

    /** Return is allowed only after one of the three explicitly successful collection endings. */
    private static boolean permitsReturn(ExitReason reason) {
        return reason == ExitReason.ROUTE_COMPLETED
                || reason == ExitReason.NEAR_END
                || reason == ExitReason.INVENTORY_FULL_AFTER_SAFE;
    }

    /** Captures the required production owner without deferring its null check to Task start. */
    private static Supplier<AdaptiveCollectionVisionService.Decision> decisionSourceOf(
            AdaptiveCollectionVisionService vision) {
        AdaptiveCollectionVisionService requiredVision = Objects.requireNonNull(vision, "vision");
        return requiredVision::decision;
    }

    /** Validate one Task-owned timeout without silently selecting an unbounded route factory. */
    private static void requirePositiveFinite(String fieldName, double value) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(
                    "AdaptiveCollectionAttempt.Config." + fieldName
                            + " must be finite and > 0 seconds, got " + value
            );
        }
    }
}
