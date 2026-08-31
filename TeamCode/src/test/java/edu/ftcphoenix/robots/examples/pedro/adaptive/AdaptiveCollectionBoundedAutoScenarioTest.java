package edu.ftcphoenix.robots.examples.pedro.adaptive;

import com.pedropathing.paths.PathChain;

import org.junit.Test;

import java.util.ArrayList;
import java.util.IdentityHashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.Plants;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.route.RouteExecution;
import edu.ftcphoenix.fw.drive.route.RouteFollower;
import edu.ftcphoenix.fw.drive.route.RouteStatus;
import edu.ftcphoenix.fw.drive.route.RouteTask;
import edu.ftcphoenix.fw.drive.route.RouteTasks;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;
import edu.ftcphoenix.fw.task.Tasks;
import edu.ftcphoenix.fw.testing.ManualLoopClock;
import edu.ftcphoenix.robots.examples.pedro.capability.intake.BasicPedroAutoMechanism;
import edu.ftcphoenix.robots.examples.pedro.capability.intake.BasicPedroMechanismTestFactory;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/**
 * Runs the bounded adaptive-Auto composition without FTC hardware or a Pedro follower.
 *
 * <p>The fixture deliberately builds the same graph an adopting Auto owns: preload and bounded
 * fresh attempts share one hard pre-park timeout, while one start-built park remains outside that
 * timeout. Exact attempt and route status objects are retained alongside the broad aggregate Task
 * outcomes.</p>
 */
public final class AdaptiveCollectionBoundedAutoScenarioTest {

    private static final int DEFAULT_MAX_ATTEMPTS = 3;
    private static final double DEFAULT_LATEST_NEW_ATTEMPT_SEC = 8.0;
    private static final double DEFAULT_PARK_TAKEOVER_SEC = 10.0;

    @Test
    public void softAdmissionCanDenyFirstOrLaterAttemptAfterPreloadUsesBudget() {
        Fixture firstDenied = new Fixture(
                false,
                DEFAULT_MAX_ATTEMPTS,
                5.0,
                DEFAULT_PARK_TAKEOVER_SEC
        );
        firstDenied.start();
        firstDenied.preload.completeSuccessfully();
        firstDenied.cycle(6.0);

        assertEquals(0, firstDenied.attemptFactoryCalls);
        assertEquals(TaskOutcome.SUCCESS, firstDenied.repeat.getOutcome());
        assertEquals(TaskOutcome.SUCCESS, firstDenied.boundedPrePark.getOutcome());
        assertEquals(RouteStatus.ACTIVE, firstDenied.park.getRouteStatus());
        assertEquals(1, firstDenied.parkBuildCount);

        Fixture laterDenied = new Fixture(true, DEFAULT_MAX_ATTEMPTS, 1.0, 10.0);
        laterDenied.start();
        laterDenied.completeCollection(1, 0.3);
        laterDenied.completeReturn(1, 0.3);
        laterDenied.cycle(0.5);

        assertEquals(1, laterDenied.attemptFactoryCalls);
        assertEquals(AdaptiveCollectionAttempt.ExitReason.ROUTE_COMPLETED,
                laterDenied.attempt(1).status().exitReason());
        assertEquals(RouteStatus.COMPLETED,
                laterDenied.attempt(1).status().returnRouteStatus());
        assertEquals(TaskOutcome.SUCCESS, laterDenied.repeat.getOutcome());
        assertEquals(TaskOutcome.SUCCESS, laterDenied.boundedPrePark.getOutcome());
        assertEquals(RouteStatus.ACTIVE, laterDenied.park.getRouteStatus());
    }

    @Test
    public void maximumAttemptCountStartsParkWithoutAThirdFactoryCall() {
        Fixture fixture = new Fixture(true, 2, 9.0, DEFAULT_PARK_TAKEOVER_SEC);
        fixture.start();

        fixture.completeCollection(1, 0.1);
        fixture.completeReturn(1, 0.1);
        fixture.cycle(0.1);
        fixture.completeCollection(2, 0.1);
        fixture.completeReturn(2, 0.1);

        assertEquals(2, fixture.attemptFactoryCalls);
        assertEquals(2, fixture.attempts.size());
        assertEquals(TaskOutcome.SUCCESS, fixture.repeat.getOutcome());
        assertEquals(TaskOutcome.SUCCESS, fixture.boundedPrePark.getOutcome());
        assertEquals(RouteStatus.ACTIVE, fixture.park.getRouteStatus());
        assertEquals(1, fixture.parkBuildCount);
    }

    @Test
    public void preloadSharesTheHardBudgetAndSettlesBeforeOneLivePoseParkBuild() {
        Fixture fixture = new Fixture(
                false,
                DEFAULT_MAX_ATTEMPTS,
                DEFAULT_LATEST_NEW_ATTEMPT_SEC,
                DEFAULT_PARK_TAKEOVER_SEC
        );
        fixture.start();
        fixture.currentPoseXInches = 31.5;
        fixture.cycle(DEFAULT_PARK_TAKEOVER_SEC);

        assertEquals(1, fixture.preload.cancelCount);
        assertEquals(0, fixture.attemptFactoryCalls);
        assertEquals(TaskOutcome.NOT_DONE, fixture.repeat.getOutcome());
        assertEquals(TaskOutcome.TIMEOUT, fixture.boundedPrePark.getOutcome());
        assertEquals(RouteStatus.ACTIVE, fixture.park.getRouteStatus());
        assertEquals(1, fixture.parkBuildCount);
        assertEquals(31.5, fixture.parkBuiltFromPoseXInches, 0.0);
        assertBefore(fixture.events, "cancel.preload", "build.park");
    }

    @Test
    public void hardTakeoverCancelsAnActiveCollectionBeforeBuildingPark() {
        Fixture fixture = new Fixture(
                true,
                DEFAULT_MAX_ATTEMPTS,
                DEFAULT_LATEST_NEW_ATTEMPT_SEC,
                DEFAULT_PARK_TAKEOVER_SEC
        );
        fixture.start();
        assertEquals(0.20, fixture.intakePlant.commandTarget().get(), 0.0);
        fixture.currentPoseXInches = 42.0;
        fixture.cycle(DEFAULT_PARK_TAKEOVER_SEC);

        AdaptiveCollectionAttempt activeAttempt = fixture.attempt(1);
        AdaptiveCollectionAttempt.Status attemptStatus = activeAttempt.status();
        assertEquals(TaskOutcome.CANCELLED, fixture.repeat.getOutcome());
        assertEquals(TaskOutcome.TIMEOUT, fixture.boundedPrePark.getOutcome());
        assertEquals(TaskOutcome.CANCELLED, activeAttempt.task().getOutcome());
        assertEquals(AdaptiveCollectionAttempt.ExitReason.CANCELLED,
                attemptStatus.exitReason());
        assertEquals(RouteStatus.CANCELLED, attemptStatus.collectionRouteStatus());
        assertEquals(RouteStatus.NOT_STARTED, attemptStatus.returnRouteStatus());
        assertEquals(0.0, fixture.intakePlant.commandTarget().get(), 0.0);
        assertTrue(fixture.parkSawIdleIntake);
        assertEquals(RouteStatus.ACTIVE, fixture.park.getRouteStatus());
        assertEquals(1, fixture.parkBuildCount);
        assertEquals(42.0, fixture.parkBuiltFromPoseXInches, 0.0);
        assertBefore(fixture.events, "cancel.attempt.1.collection", "build.park");
    }

    @Test
    public void hardTakeoverCancelsTheLaterAttemptsReturnBeforeBuildingPark() {
        Fixture fixture = new Fixture(
                true,
                DEFAULT_MAX_ATTEMPTS,
                DEFAULT_LATEST_NEW_ATTEMPT_SEC,
                DEFAULT_PARK_TAKEOVER_SEC
        );
        fixture.start();
        fixture.completeCollection(1, 0.1);
        fixture.completeReturn(1, 0.1);
        fixture.cycle(0.1);
        fixture.completeCollection(2, 0.1);

        fixture.currentPoseXInches = 57.0;
        fixture.cycle(DEFAULT_PARK_TAKEOVER_SEC - fixture.nowSec());

        AdaptiveCollectionAttempt.Status firstStatus = fixture.attempt(1).status();
        AdaptiveCollectionAttempt secondAttempt = fixture.attempt(2);
        AdaptiveCollectionAttempt.Status secondStatus = secondAttempt.status();
        assertTrue(firstStatus.complete());
        assertEquals(RouteStatus.COMPLETED, firstStatus.returnRouteStatus());
        assertEquals(2, fixture.attemptFactoryCalls);
        assertEquals(TaskOutcome.CANCELLED, fixture.repeat.getOutcome());
        assertEquals(TaskOutcome.TIMEOUT, fixture.boundedPrePark.getOutcome());
        assertEquals(TaskOutcome.CANCELLED, secondAttempt.task().getOutcome());
        assertEquals(AdaptiveCollectionAttempt.ExitReason.ROUTE_COMPLETED,
                secondStatus.exitReason());
        assertEquals(RouteStatus.COMPLETED, secondStatus.collectionRouteStatus());
        assertEquals(RouteStatus.CANCELLED, secondStatus.returnRouteStatus());
        assertEquals(RouteStatus.ACTIVE, fixture.park.getRouteStatus());
        assertEquals(1, fixture.parkBuildCount);
        assertEquals(57.0, fixture.parkBuiltFromPoseXInches, 0.0);
        assertBefore(fixture.events, "cancel.attempt.2.return", "build.park");
    }

    @Test
    public void earlyParkCanContinuePastTheFormerCutoffWithoutRestart() {
        Fixture fixture = new Fixture(true, DEFAULT_MAX_ATTEMPTS, 0.0, 10.0);
        fixture.currentPoseXInches = 4.0;
        fixture.start();

        assertEquals(TaskOutcome.SUCCESS, fixture.boundedPrePark.getOutcome());
        assertEquals(RouteStatus.ACTIVE, fixture.park.getRouteStatus());
        assertEquals(1, fixture.parkBuildCount);

        fixture.currentPoseXInches = 99.0;
        fixture.cycle(10.5);

        assertFalse(fixture.root.isComplete());
        assertEquals(RouteStatus.ACTIVE, fixture.park.getRouteStatus());
        assertEquals(1, fixture.parkBuildCount);
        assertEquals(4.0, fixture.parkBuiltFromPoseXInches, 0.0);
        assertEquals(0, fixture.execution("park").cancelCount);

        fixture.execution("park").integrationStatus = RouteStatus.COMPLETED;
        fixture.cycle(0.1);
        assertTrue(fixture.root.isComplete());
        assertEquals(TaskOutcome.SUCCESS, fixture.root.getOutcome());
        assertEquals(RouteStatus.COMPLETED, fixture.park.getRouteStatus());
    }

    @Test
    public void abnormalSettledAttemptStillParksButItsExactStatusBlocksAnotherAttempt() {
        Fixture fixture = new Fixture(
                true,
                DEFAULT_MAX_ATTEMPTS,
                DEFAULT_LATEST_NEW_ATTEMPT_SEC,
                DEFAULT_PARK_TAKEOVER_SEC
        );
        fixture.start();
        fixture.execution("attempt.1.collection").integrationStatus =
                RouteStatus.FOLLOWER_TIMEOUT_OR_STALL;
        fixture.cycle(0.1);
        fixture.cycle(0.1);

        AdaptiveCollectionAttempt attempt = fixture.attempt(1);
        AdaptiveCollectionAttempt.Status exactStatus = attempt.status();
        assertEquals(TaskOutcome.SUCCESS, attempt.task().getOutcome());
        assertEquals(AdaptiveCollectionAttempt.ExitReason.FOLLOWER_TIMEOUT_OR_STALL,
                exactStatus.exitReason());
        assertEquals(RouteStatus.FOLLOWER_TIMEOUT_OR_STALL,
                exactStatus.collectionRouteStatus());
        assertEquals(RouteStatus.NOT_STARTED, exactStatus.returnRouteStatus());
        assertEquals(1, fixture.attemptFactoryCalls);
        assertEquals(TaskOutcome.SUCCESS, fixture.repeat.getOutcome());
        assertEquals(TaskOutcome.SUCCESS, fixture.boundedPrePark.getOutcome());
        assertEquals(RouteStatus.ACTIVE, fixture.park.getRouteStatus());
    }

    @Test
    public void abnormalTaskOutcomeStillParksWithoutAdmittingAnotherAttempt() {
        Fixture fixture = new Fixture(
                true,
                DEFAULT_MAX_ATTEMPTS,
                DEFAULT_LATEST_NEW_ATTEMPT_SEC,
                DEFAULT_PARK_TAKEOVER_SEC
        );
        fixture.start();
        fixture.completeCollection(1, 0.1);
        fixture.execution("attempt.1.return").integrationStatus =
                RouteStatus.FOLLOWER_TIMEOUT_OR_STALL;
        fixture.cycle(0.1);

        AdaptiveCollectionAttempt attempt = fixture.attempt(1);
        AdaptiveCollectionAttempt.Status exactStatus = attempt.status();
        assertTrue(exactStatus.complete());
        assertEquals(AdaptiveCollectionAttempt.ExitReason.ROUTE_COMPLETED,
                exactStatus.exitReason());
        assertEquals(RouteStatus.COMPLETED, exactStatus.collectionRouteStatus());
        assertEquals(RouteStatus.FOLLOWER_TIMEOUT_OR_STALL,
                exactStatus.returnRouteStatus());
        assertEquals(TaskOutcome.TIMEOUT, attempt.task().getOutcome());
        assertEquals(TaskOutcome.TIMEOUT, fixture.repeat.getOutcome());
        assertEquals(TaskOutcome.TIMEOUT, fixture.boundedPrePark.getOutcome());
        assertEquals(1, fixture.attemptFactoryCalls);
        assertEquals(RouteStatus.ACTIVE, fixture.park.getRouteStatus());
        assertEquals(1, fixture.parkBuildCount);
    }

    @Test
    public void directCancellationAndFailedCleanupBothSuppressPark() {
        Fixture cancelled = new Fixture(
                true,
                DEFAULT_MAX_ATTEMPTS,
                DEFAULT_LATEST_NEW_ATTEMPT_SEC,
                DEFAULT_PARK_TAKEOVER_SEC
        );
        cancelled.start();
        cancelled.root.cancel();
        cancelled.root.cancel();

        assertTrue(cancelled.root.isComplete());
        assertEquals(TaskOutcome.CANCELLED, cancelled.root.getOutcome());
        assertEquals(TaskOutcome.CANCELLED, cancelled.boundedPrePark.getOutcome());
        assertEquals(TaskOutcome.CANCELLED, cancelled.repeat.getOutcome());
        assertEquals(AdaptiveCollectionAttempt.ExitReason.CANCELLED,
                cancelled.attempt(1).status().exitReason());
        assertEquals(RouteStatus.NOT_STARTED, cancelled.park.getRouteStatus());
        assertEquals(0, cancelled.parkBuildCount);

        Fixture failedCleanup = new Fixture(
                false,
                DEFAULT_MAX_ATTEMPTS,
                DEFAULT_LATEST_NEW_ATTEMPT_SEC,
                DEFAULT_PARK_TAKEOVER_SEC
        );
        RuntimeException cleanupFailure = new RuntimeException("preload cleanup failed");
        failedCleanup.preload.cancelFailure = cleanupFailure;
        failedCleanup.start();

        try {
            failedCleanup.cycle(DEFAULT_PARK_TAKEOVER_SEC);
            fail("expected timeout cleanup failure");
        } catch (RuntimeException expected) {
            assertSame(cleanupFailure, expected);
        }

        assertFalse(failedCleanup.root.isComplete());
        assertEquals(TaskOutcome.NOT_DONE, failedCleanup.boundedPrePark.getOutcome());
        assertEquals(RouteStatus.NOT_STARTED, failedCleanup.park.getRouteStatus());
        assertEquals(0, failedCleanup.parkBuildCount);
        assertEquals(1, failedCleanup.preload.cancelCount);

        failedCleanup.root.cancel();
        assertTrue(failedCleanup.root.isComplete());
        assertEquals(TaskOutcome.CANCELLED, failedCleanup.root.getOutcome());
        assertEquals(RouteStatus.NOT_STARTED, failedCleanup.park.getRouteStatus());
        assertEquals(0, failedCleanup.parkBuildCount);
    }

    /** Assert the exact cleanup event happened before the live park factory was sampled. */
    private static void assertBefore(List<String> events, String first, String second) {
        int firstIndex = events.indexOf(first);
        int secondIndex = events.indexOf(second);
        assertTrue("missing event " + first + " in " + events, firstIndex >= 0);
        assertTrue("missing event " + second + " in " + events, secondIndex >= 0);
        assertTrue("expected " + first + " before " + second + " in " + events,
                firstIndex < secondIndex);
    }

    /** One hardware-free composition root retaining every policy and diagnostic object. */
    private static final class Fixture {
        final ManualLoopClock time = new ManualLoopClock();
        final List<String> events = new ArrayList<String>();
        final Map<PathChain, String> routeLabels = new IdentityHashMap<PathChain, String>();
        final RecordingFollower follower = new RecordingFollower(routeLabels, events);
        final Plant intakePlant = Plants.fromOutputs()
                .power(new InertPowerOutput())
                .targetFromNewCommand(0.0)
                .build();
        final BasicPedroAutoMechanism intake =
                BasicPedroMechanismTestFactory.fromPlant(intakePlant);
        final ControlledTask preload;
        final List<AdaptiveCollectionAttempt> attempts =
                new ArrayList<AdaptiveCollectionAttempt>();
        final Task repeat;
        final Task preParkWork;
        final Task boundedPrePark;
        final RouteTask<PathChain> park;
        final Task root;

        AdaptiveCollectionAttempt lastAttempt;
        int attemptFactoryCalls;
        int parkBuildCount;
        double currentPoseXInches;
        double parkBuiltFromPoseXInches = Double.NaN;
        boolean parkSawIdleIntake;

        Fixture(boolean preloadCompletesOnStart,
                int maxAttempts,
                double latestNewAttemptSec,
                double parkTakeoverSec) {
            preload = new ControlledTask("preload", preloadCompletesOnStart, events);

            repeat = Tasks.repeatWhileSuccessful(
                    "adaptiveCollection.attempts",
                    maxAttempts,
                    clock -> clock.nowSec() < latestNewAttemptSec
                            && (lastAttempt == null || lastAttemptPermitsAnother()),
                    this::buildFreshAttemptTask
            );
            preParkWork = Tasks.sequence(preload, repeat);
            boundedPrePark = Tasks.withTimeout(preParkWork, parkTakeoverSec);
            park = RouteTasks.followBuiltAtStart(
                    "adaptiveCollection.park",
                    follower,
                    this::buildParkFromCurrentPose,
                    30.0
            );
            root = Tasks.sequence(boundedPrePark, park);
        }

        /** Mimic RobotProgram's START-cycle start plus first update on the shared clock. */
        void start() {
            root.start(time.clock());
            root.update(time.clock());
        }

        /** Advance exactly one managed loop cycle. */
        void cycle(double dtSec) {
            root.update(time.nextCycle(dtSec));
        }

        double nowSec() {
            return time.clock().nowSec();
        }

        AdaptiveCollectionAttempt attempt(int oneBasedIndex) {
            return attempts.get(oneBasedIndex - 1);
        }

        void completeCollection(int attemptNumber, double dtSec) {
            execution("attempt." + attemptNumber + ".collection").integrationStatus =
                    RouteStatus.COMPLETED;
            cycle(dtSec);
        }

        void completeReturn(int attemptNumber, double dtSec) {
            execution("attempt." + attemptNumber + ".return").integrationStatus =
                    RouteStatus.COMPLETED;
            cycle(dtSec);
        }

        ControlledExecution execution(String label) {
            ControlledExecution execution = follower.executionsByLabel.get(label);
            if (execution == null) {
                throw new AssertionError("No execution named " + label + "; events=" + events);
            }
            return execution;
        }

        /** Build one fresh real adaptive attempt and retain its exact status owner. */
        private Task buildFreshAttemptTask() {
            final int attemptNumber = ++attemptFactoryCalls;
            AdaptiveCollectionPaths paths = new AdaptiveCollectionPaths(
                    follower,
                    (decision, milestones) -> {
                        events.add("build.attempt." + attemptNumber + ".collection");
                        return route("attempt." + attemptNumber + ".collection");
                    },
                    () -> {
                        events.add("build.attempt." + attemptNumber + ".return");
                        return route("attempt." + attemptNumber + ".return");
                    }
            );
            AdaptiveCollectionAttempt.Config config =
                    AdaptiveCollectionAttempt.Config.defaults();
            config.collectionRouteTimeoutSec = 20.0;
            config.returnRouteTimeoutSec = 20.0;
            AdaptiveCollectionAttempt built = new AdaptiveCollectionAttempt(
                    () -> AdaptiveCollectionVisionService.Decision
                            .selectedForHardwareNeutralTest(12.0),
                    paths,
                    BooleanSource.of(() -> false),
                    intake,
                    config
            );
            attempts.add(built);
            lastAttempt = built;
            return built.task();
        }

        /** Admit another attempt only after a fully successful exact attempt and return. */
        private boolean lastAttemptPermitsAnother() {
            AdaptiveCollectionAttempt.Status status = lastAttempt.status();
            if (!status.complete() || status.returnRouteStatus() != RouteStatus.COMPLETED) {
                return false;
            }
            AdaptiveCollectionAttempt.ExitReason reason = status.exitReason();
            return reason == AdaptiveCollectionAttempt.ExitReason.ROUTE_COMPLETED
                    || reason == AdaptiveCollectionAttempt.ExitReason.NEAR_END
                    || reason == AdaptiveCollectionAttempt.ExitReason.INVENTORY_FULL_AFTER_SAFE;
        }

        /** Sample one live current-pose fact only after pre-park cleanup has settled. */
        private PathChain buildParkFromCurrentPose() {
            parkBuildCount++;
            parkBuiltFromPoseXInches = currentPoseXInches;
            parkSawIdleIntake = intakePlant.commandTarget().get() == 0.0;
            events.add("build.park");
            return route("park");
        }

        private PathChain route(String label) {
            PathChain route = new PathChain();
            routeLabels.put(route, label);
            return route;
        }
    }

    /** Exact-route fake that retains one independent execution per route identity. */
    private static final class RecordingFollower implements RouteFollower<PathChain> {
        private final Map<PathChain, String> routeLabels;
        private final List<String> events;
        final Map<String, ControlledExecution> executionsByLabel =
                new LinkedHashMap<String, ControlledExecution>();

        RecordingFollower(Map<PathChain, String> routeLabels, List<String> events) {
            this.routeLabels = routeLabels;
            this.events = events;
        }

        @Override
        public RouteExecution follow(PathChain route) {
            String label = routeLabels.get(route);
            if (label == null) {
                throw new AssertionError("follow received an unregistered route identity");
            }
            ControlledExecution execution = new ControlledExecution(label, events);
            if (executionsByLabel.put(label, execution) != null) {
                throw new AssertionError("duplicate route label " + label);
            }
            events.add("follow." + label);
            return execution;
        }
    }

    /** Per-start route status and cancellation handle used by {@link RecordingFollower}. */
    private static final class ControlledExecution extends RouteExecution {
        private final String label;
        private final List<String> events;
        RouteStatus integrationStatus = RouteStatus.ACTIVE;
        int cancelCount;

        ControlledExecution(String label, List<String> events) {
            this.label = label;
            this.events = events;
        }

        @Override
        protected RouteStatus integrationStatus() {
            return integrationStatus;
        }

        @Override
        protected void cancelActive() {
            cancelCount++;
            events.add("cancel." + label);
        }
    }

    /** Manual preload whose active cancellation can emulate a failed cooperative cleanup. */
    private static final class ControlledTask implements Task {
        private final String name;
        private final boolean completeOnStart;
        private final List<String> events;
        private boolean startAttempted;
        private boolean started;
        private boolean complete;
        private TaskOutcome outcome = TaskOutcome.NOT_DONE;
        int cancelCount;
        RuntimeException cancelFailure;

        ControlledTask(String name, boolean completeOnStart, List<String> events) {
            this.name = name;
            this.completeOnStart = completeOnStart;
            this.events = events;
        }

        @Override
        public void start(LoopClock clock) {
            if (startAttempted) {
                throw new IllegalStateException(name + " is single-use");
            }
            startAttempted = true;
            started = true;
            events.add("start." + name);
            if (completeOnStart) {
                complete = true;
                outcome = TaskOutcome.SUCCESS;
            }
        }

        @Override
        public void update(LoopClock clock) {
            if (!started) {
                throw new IllegalStateException(name + " cannot update before start");
            }
        }

        @Override
        public void cancel() {
            if (!started || complete) {
                return;
            }
            complete = true;
            outcome = TaskOutcome.CANCELLED;
            cancelCount++;
            events.add("cancel." + name);
            if (cancelFailure != null) {
                throw cancelFailure;
            }
        }

        @Override
        public boolean isComplete() {
            return complete;
        }

        @Override
        public TaskOutcome getOutcome() {
            return complete ? outcome : TaskOutcome.NOT_DONE;
        }

        void completeSuccessfully() {
            if (!started || complete) {
                throw new IllegalStateException(name + " must be active before completion");
            }
            complete = true;
            outcome = TaskOutcome.SUCCESS;
            events.add("complete." + name);
        }
    }

    /** Inert hardware-neutral final output used only to expose the intake command target. */
    private static final class InertPowerOutput implements PowerOutput {
        private double commandedPower;

        @Override
        public void setPower(double power) {
            commandedPower = power;
        }

        @Override
        public double getCommandedPower() {
            return commandedPower;
        }
    }
}
