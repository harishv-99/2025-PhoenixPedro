package edu.ftcphoenix.robots.examples.pedro;

import org.junit.Test;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.PlantTargetStatus;
import edu.ftcphoenix.fw.actuation.PlantTargets;
import edu.ftcphoenix.fw.actuation.Plants;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.source.ScalarTarget;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the example capability's fresh Task and cancellation-safe target behavior. */
public final class BasicPedroAutoMechanismTest {

    @Test
    public void collectTasksAreFreshAndActiveCancellationRestoresIdle() {
        ScalarTarget target = ScalarTarget.create(0.0);
        RecordingPowerOutput output = new RecordingPowerOutput();
        Plant plant = Plants.power(output, target);
        BasicPedroAutoMechanism mechanism = new BasicPedroAutoMechanism(plant, 0.75);
        Task first = mechanism.collectTask(0.50);
        Task second = mechanism.collectTask(0.50);
        ManualLoopClock time = new ManualLoopClock();

        assertNotSame(first, second);
        first.start(time.clock());
        assertEquals(0.75, target.get(), 0.0);

        mechanism.update(time.clock());
        assertEquals(0.75, output.commandedPower, 0.0);

        first.cancel();
        first.cancel();
        assertTrue(first.isComplete());
        assertEquals(TaskOutcome.CANCELLED, first.getOutcome());
        assertEquals(0.0, target.get(), 0.0);

        mechanism.update(time.nextCycle(0.02));
        assertEquals(0.0, output.commandedPower, 0.0);
    }

    @Test
    public void constructorRejectsPlantWithoutCommandTarget() {
        Plant plant = Plants.power(
                new RecordingPowerOutput(),
                PlantTargets.exact(0.0)
        );

        try {
            new BasicPedroAutoMechanism(plant, 0.75);
            fail("expected Plant without a command target to fail");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("intakePlant to have a command target"));
        }
    }

    @Test
    public void constructorDerivesCommandOnceAndLaterBehaviorUsesCachedTarget() {
        CountingCommandPlant plant = new CountingCommandPlant();
        BasicPedroAutoMechanism mechanism = new BasicPedroAutoMechanism(plant, 0.75);

        mechanism.collectTask(0.50);
        mechanism.idleTask();
        mechanism.stop();

        assertEquals(1, plant.commandTargetCalls);
        assertEquals(0.0, plant.target.get(), 0.0);
    }

    @Test
    public void constructorRejectsPlantThatClaimsButReturnsNoCommandTarget() {
        CountingCommandPlant plant = new CountingCommandPlant(null);

        try {
            new BasicPedroAutoMechanism(plant, 0.75);
            fail("expected null command target to fail");
        } catch (NullPointerException expected) {
            assertTrue(expected.getMessage().contains("intakePlant.commandTarget()"));
        }
        assertEquals(1, plant.commandTargetCalls);
    }

    private static final class RecordingPowerOutput implements PowerOutput {
        double commandedPower;

        @Override
        public void setPower(double power) {
            commandedPower = power;
        }

        @Override
        public double getCommandedPower() {
            return commandedPower;
        }
    }

    private static final class CountingCommandPlant implements Plant {
        final ScalarTarget target;
        int commandTargetCalls;

        CountingCommandPlant() {
            this(ScalarTarget.create(0.25));
        }

        CountingCommandPlant(ScalarTarget target) {
            this.target = target;
        }

        @Override
        public void update(LoopClock clock) {
        }

        @Override
        public double getRequestedTarget() {
            return target.get();
        }

        @Override
        public double getAppliedTarget() {
            return target.get();
        }

        @Override
        public PlantTargetStatus getTargetStatus() {
            return PlantTargetStatus.ACCEPTED;
        }

        @Override
        public boolean hasCommandTarget() {
            return true;
        }

        @Override
        public ScalarTarget commandTarget() {
            commandTargetCalls++;
            return target;
        }

        @Override
        public void stop() {
        }
    }
}
