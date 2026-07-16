package edu.ftcphoenix.robots.examples.pedro;

import com.pedropathing.paths.PathChain;

import org.junit.Test;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.Plants;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.source.ScalarTarget;
import edu.ftcphoenix.fw.drive.route.RouteExecution;
import edu.ftcphoenix.fw.drive.route.RouteFollower;
import edu.ftcphoenix.fw.drive.route.RouteStatus;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/** Verifies the example's explicit success, timeout, and cancellation-like route policy. */
public final class BasicPedroAutoRoutineTest {

    @Test
    public void completedRouteRunsCapabilityAndReturnsItToIdle() {
        Fixture fixture = new Fixture();
        Task routine = fixture.newRoutine();

        routine.start(fixture.time.clock());
        fixture.execution.integrationStatus = RouteStatus.COMPLETED;
        routine.update(fixture.time.clock());

        assertEquals(1, fixture.follower.followCount);
        assertEquals(0.80, fixture.target.get(), 0.0);
        assertFalse(routine.isComplete());

        routine.update(fixture.time.nextCycle(0.51));
        assertTrue(routine.isComplete());
        assertEquals(TaskOutcome.SUCCESS, routine.getOutcome());
        assertEquals(0.0, fixture.target.get(), 0.0);
    }

    @Test
    public void taskTimeoutRunsOnlyTheSafeFallback() {
        Fixture fixture = new Fixture();
        fixture.target.set(0.45);
        Task routine = fixture.newRoutine();

        routine.start(fixture.time.clock());
        routine.update(fixture.time.nextCycle(4.01));

        assertEquals(1, fixture.execution.cancelCount);
        assertEquals(0.0, fixture.target.get(), 0.0);
        routine.update(fixture.time.nextCycle(0.0));
        assertTrue(routine.isComplete());
        assertEquals(TaskOutcome.SUCCESS, routine.getOutcome());
    }

    @Test
    public void cancellationLikeRouteEndingStartsNeitherContinuation() {
        Fixture fixture = new Fixture();
        fixture.target.set(0.33);
        Task routine = fixture.newRoutine();

        routine.start(fixture.time.clock());
        fixture.execution.integrationStatus = RouteStatus.FAILED;
        routine.update(fixture.time.clock());

        assertTrue(routine.isComplete());
        assertEquals(TaskOutcome.CANCELLED, routine.getOutcome());
        assertEquals(0.33, fixture.target.get(), 0.0);
        assertEquals(0, fixture.execution.cancelCount);
    }

    @Test
    public void directCancellationCancelsExactRouteWithoutStartingFallback() {
        Fixture fixture = new Fixture();
        fixture.target.set(0.27);
        Task routine = fixture.newRoutine();

        routine.start(fixture.time.clock());
        routine.cancel();
        routine.cancel();

        assertTrue(routine.isComplete());
        assertEquals(TaskOutcome.CANCELLED, routine.getOutcome());
        assertEquals(1, fixture.execution.cancelCount);
        assertEquals(0.27, fixture.target.get(), 0.0);
    }

    private static final class Fixture {
        final ManualLoopClock time = new ManualLoopClock();
        final ScalarTarget target = ScalarTarget.held(0.0);
        final Plant plant = Plants.power(new RecordingPowerOutput(), target);
        final BasicPedroAutoMechanism mechanism = new BasicPedroAutoMechanism(plant, 0.80);
        final FakeRouteExecution execution = new FakeRouteExecution();
        final FakeRouteFollower follower = new FakeRouteFollower(execution);

        Task newRoutine() {
            return BasicPedroAutoRoutine.build(follower, new PathChain(), mechanism);
        }
    }

    private static final class FakeRouteFollower implements RouteFollower<PathChain> {
        final FakeRouteExecution execution;
        int followCount;

        FakeRouteFollower(FakeRouteExecution execution) {
            this.execution = execution;
        }

        @Override
        public RouteExecution follow(PathChain route) {
            followCount++;
            return execution;
        }
    }

    private static final class FakeRouteExecution extends RouteExecution {
        RouteStatus integrationStatus = RouteStatus.ACTIVE;
        int cancelCount;

        @Override
        protected RouteStatus integrationStatus() {
            return integrationStatus;
        }

        @Override
        protected void cancelActive() {
            cancelCount++;
        }
    }

    private static final class RecordingPowerOutput implements PowerOutput {
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
