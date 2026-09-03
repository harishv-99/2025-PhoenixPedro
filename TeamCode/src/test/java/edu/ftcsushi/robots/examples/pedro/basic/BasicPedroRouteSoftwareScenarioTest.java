package edu.ftcsushi.robots.examples.pedro.basic;

import com.pedropathing.paths.PathChain;

import org.junit.Test;

import edu.ftcsushi.fw.drive.route.RouteExecution;
import edu.ftcsushi.fw.drive.route.RouteFollower;
import edu.ftcsushi.fw.drive.route.RouteStatus;
import edu.ftcsushi.fw.drive.route.RouteTask;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;

/**
 * Hardware-free route-boundary questions for the one-route Pedro lesson.
 *
 * <p>The real {@link RouteTask} remains under test. Only Pedro's external follower/execution
 * boundary is replaced, so the scenario can author completion or missing-completion evidence
 * without pretending to simulate drivetrain motion, localization, route accuracy, or safety.</p>
 */
public final class BasicPedroRouteSoftwareScenarioTest {

    @Test
    public void completedExecutionProducesSuccessForTheExactAuthoredRoute() {
        // ARRANGE: the follower records which route started; it invents no motion or completion.
        PathChain authoredRoute = new PathChain();
        RecordingExecution execution = new RecordingExecution();
        RecordingFollower follower = new RecordingFollower(execution);
        ManualLoopClock time = new ManualLoopClock();
        RouteTask<PathChain> routeTask = BasicPedroAuto.routeTask(follower, authoredRoute);

        // REQUEST: starting the Task must start this exact, eagerly authored route once.
        routeTask.start(time.clock());
        assertSame(authoredRoute, follower.followedRoute);
        assertEquals(1, follower.followCount);
        assertEquals(RouteStatus.ACTIVE, routeTask.getRouteStatus());
        assertEquals(TaskOutcome.NOT_DONE, routeTask.getOutcome());

        // INJECT EVIDENCE: only the retained execution may say that its endpoint was reached.
        execution.integrationStatus = RouteStatus.COMPLETED;

        // HEARTBEAT: the real Route Task observes that external fact on the next test cycle.
        routeTask.update(time.nextCycle(0.02));

        // ASSERT: exact endpoint evidence maps to exact route status and Task success.
        assertEquals(RouteStatus.COMPLETED, routeTask.getRouteStatus());
        assertEquals(TaskOutcome.SUCCESS, routeTask.getOutcome());
        assertEquals(0, execution.cancelCount);

        // NEXT GATE: only a supervised robot run can establish path accuracy and clearance.
    }

    @Test
    public void missingCompletionTimesOutAndCancelsTheExactExecution() {
        // ARRANGE: this execution stays ACTIVE unless the test supplies another fact.
        RecordingExecution execution = new RecordingExecution();
        RecordingFollower follower = new RecordingFollower(execution);
        ManualLoopClock time = new ManualLoopClock();
        RouteTask<PathChain> routeTask =
                BasicPedroAuto.routeTask(follower, new PathChain());

        // REQUEST: begin one route attempt at this Task's own time boundary.
        routeTask.start(time.clock());

        // HEARTBEAT: no endpoint evidence arrives before the lesson's four-second Task limit.
        routeTask.update(time.nextCycle(4.0));

        // ASSERT: timeout remains distinct from success and cleans up this execution exactly once.
        assertEquals(RouteStatus.TASK_TIMEOUT, routeTask.getRouteStatus());
        assertEquals(TaskOutcome.TIMEOUT, routeTask.getOutcome());
        assertEquals(1, execution.cancelCount);

        // NEXT GATE: this proves timeout policy, not that a physical drivetrain stopped safely.
    }

    /** Minimal test double for the external route-start boundary. */
    private static final class RecordingFollower implements RouteFollower<PathChain> {
        private final RecordingExecution execution;
        private PathChain followedRoute;
        private int followCount;

        private RecordingFollower(RecordingExecution execution) {
            this.execution = execution;
        }

        @Override
        public RouteExecution follow(PathChain route) {
            followedRoute = route;
            followCount++;
            return execution;
        }
    }

    /** Minimal test double whose status is authored independently from route commands. */
    private static final class RecordingExecution extends RouteExecution {
        private RouteStatus integrationStatus = RouteStatus.ACTIVE;
        private int cancelCount;

        @Override
        protected RouteStatus integrationStatus() {
            return integrationStatus;
        }

        @Override
        protected void cancelActive() {
            cancelCount++;
        }
    }
}
