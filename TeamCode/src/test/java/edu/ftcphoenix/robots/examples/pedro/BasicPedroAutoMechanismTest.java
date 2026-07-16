package edu.ftcphoenix.robots.examples.pedro;

import org.junit.Test;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.Plants;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.source.ScalarTarget;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertTrue;

/** Verifies the example capability's fresh Task and cancellation-safe target behavior. */
public final class BasicPedroAutoMechanismTest {

    @Test
    public void collectTasksAreFreshAndActiveCancellationRestoresIdle() {
        ScalarTarget target = ScalarTarget.held(0.0);
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
}
