package edu.ftcsushi.robots.examples.pedro.adaptive;

import edu.ftcsushi.fw.testing.ftc.FtcTestHardware;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.junit.Test;

import edu.ftcsushi.fw.actuation.Plant;
import edu.ftcsushi.fw.actuation.PlantTargetStatus;
import edu.ftcsushi.fw.actuation.PlantTargets;
import edu.ftcsushi.fw.actuation.Plants;
import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.core.hal.PowerOutput;
import edu.ftcsushi.fw.core.source.ScalarTarget;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the example capability's fresh Task and cancellation-safe target behavior. */
public final class AdaptiveCollectionIntakeTest {

    @Test
    public void collectTasksAreFreshAndActiveCancellationRestoresIdle() {
        RecordingPowerOutput output = new RecordingPowerOutput();
        Plant plant = Plants.fromOutputs()
                .power(output)
                .targetFromNewCommand(0.0)
                .build();
        AdaptiveCollectionIntake mechanism = new AdaptiveCollectionIntake(plant);
        Task first = mechanism.collectTask(0.50);
        Task second = mechanism.collectTask(0.50);
        ManualLoopClock time = new ManualLoopClock();

        assertNotSame(first, second);
        first.start(time.clock());
        assertEquals(0.20, plant.commandTarget().get(), 0.0);

        mechanism.update(time.clock());
        assertEquals(0.20, output.commandedPower, 0.0);

        first.cancel();
        first.cancel();
        assertTrue(first.isComplete());
        assertEquals(TaskOutcome.CANCELLED, first.getOutcome());
        assertEquals(0.0, plant.commandTarget().get(), 0.0);

        mechanism.update(time.nextCycle(0.02));
        assertEquals(0.0, output.commandedPower, 0.0);
    }

    @Test
    public void constructorRejectsPlantWithoutCommandTarget() {
        Plant plant = Plants.fromOutputs()
                .power(new RecordingPowerOutput())
                .targetFromResolver(PlantTargets.exact(0.0))
                .build();

        try {
            new AdaptiveCollectionIntake(plant);
            fail("expected Plant without a command target to fail");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("intakePlant to have a command target"));
        }
    }

    @Test
    public void constructorValidatesCommandAndLaterBehaviorUsesStablePlantAccessor() {
        CountingCommandPlant plant = new CountingCommandPlant();
        AdaptiveCollectionIntake mechanism = new AdaptiveCollectionIntake(plant);

        assertEquals(1, plant.commandTargetCalls);

        Task collect = mechanism.collectTask(0.50);
        assertEquals(2, plant.commandTargetCalls);
        Task idle = mechanism.idleTask();
        assertEquals(3, plant.commandTargetCalls);

        ManualLoopClock time = new ManualLoopClock();
        collect.start(time.clock());
        assertEquals(0.20, plant.target.get(), 0.0);
        collect.cancel();
        idle.start(time.clock());
        assertEquals(0.0, plant.target.get(), 0.0);

        plant.target.set(0.40);
        mechanism.stop();

        assertEquals(3, plant.commandTargetCalls);
        assertEquals(0.40, plant.target.get(), 0.0);
    }

    @Test
    public void constructorRejectsPlantThatClaimsButReturnsNoCommandTarget() {
        CountingCommandPlant plant = new CountingCommandPlant(null);

        try {
            new AdaptiveCollectionIntake(plant);
            fail("expected null command target to fail");
        } catch (NullPointerException expected) {
            assertTrue(expected.getMessage().contains("intakePlant.commandTarget()"));
        }
        assertEquals(1, plant.commandTargetCalls);
    }

    @Test
    public void ordinaryConstructorSnapshotsConfigBeforeLaterTaskExecution() {
        AdaptiveCollectionIntake.Config config = AdaptiveCollectionIntake.Config.defaults();
        config.motorName = "  customIntake  ";
        config.direction = Direction.REVERSE;
        config.collectPower = -0.65;
        FtcTestHardware hardwareMap =
                new FtcTestHardware();
        FtcTestHardware.MotorProbe motor = hardwareMap.addMotor("customIntake");

        AdaptiveCollectionIntake mechanism = new AdaptiveCollectionIntake(hardwareMap, config);

        assertEquals(1, hardwareMap.lookupCalls());
        assertEquals("  customIntake  ", hardwareMap.lastLookupName());
        assertEquals(0, hardwareMap.totalMotorPowerWrites());
        assertEquals(DcMotorSimple.Direction.REVERSE, motor.direction());

        config.motorName = "missingAfterConstruction";
        config.direction = Direction.FORWARD;
        config.collectPower = 0.95;

        ManualLoopClock time = new ManualLoopClock();
        Task collect = mechanism.collectTask(0.50);
        collect.start(time.clock());
        mechanism.update(time.clock());
        assertEquals(-0.65, motor.power(), 0.0);
        assertEquals(1, hardwareMap.lookupCalls());
        assertEquals(DcMotorSimple.Direction.REVERSE, motor.direction());

        collect.cancel();
        mechanism.update(time.nextCycle(0.02));
        assertEquals(0.0, motor.power(), 0.0);
        mechanism.stop();
        assertEquals(0.0, motor.power(), 0.0);
    }

    @Test
    public void ordinaryOwnerRejectsEveryInvalidConfigFieldBeforeLookup() {
        assertInvalid(null, "AdaptiveCollectionIntake.Config is required");

        AdaptiveCollectionIntake.Config config = AdaptiveCollectionIntake.Config.defaults();
        config.motorName = null;
        assertInvalid(config, "AdaptiveCollectionIntake.Config.motorName", "got null");
        config = AdaptiveCollectionIntake.Config.defaults();
        config.motorName = "";
        assertInvalid(config, "AdaptiveCollectionIntake.Config.motorName", "non-blank");
        config = AdaptiveCollectionIntake.Config.defaults();
        config.motorName = " \t ";
        assertInvalid(config, "AdaptiveCollectionIntake.Config.motorName", "non-blank");

        config = AdaptiveCollectionIntake.Config.defaults();
        config.direction = null;
        assertInvalid(config, "AdaptiveCollectionIntake.Config.direction", "got null");

        assertInvalidCollectPower(Double.NaN);
        assertInvalidCollectPower(Double.POSITIVE_INFINITY);
        assertInvalidCollectPower(Double.NEGATIVE_INFINITY);
        assertInvalidCollectPower(0.0);
        assertInvalidCollectPower(-0.0);
        assertInvalidCollectPower(Math.nextUp(1.0));
        assertInvalidCollectPower(Math.nextDown(-1.0));
    }

    @Test
    public void exactPowerBoundsAndEitherSignRemainValid() {
        assertAcceptedPower(1.0);
        assertAcceptedPower(-1.0);
        assertAcceptedPower(0.01);
        assertAcceptedPower(-0.01);
    }

    private static void assertInvalidCollectPower(double value) {
        AdaptiveCollectionIntake.Config config = AdaptiveCollectionIntake.Config.defaults();
        config.collectPower = value;
        assertInvalid(
                config,
                "AdaptiveCollectionIntake.Config.collectPower",
                "finite, nonzero, and in [-1.0, 1.0]",
                "got " + value);
    }

    private static void assertInvalid(AdaptiveCollectionIntake.Config config,
                                      String... messageFragments) {
        FtcTestHardware hardwareMap =
                new FtcTestHardware();
        FtcTestHardware.MotorProbe motor = hardwareMap.addMotor("intakeMotor");
        try {
            new AdaptiveCollectionIntake(hardwareMap, config);
            fail("expected invalid adaptive collection intake Config to fail");
        } catch (RuntimeException expected) {
            for (String fragment : messageFragments) {
                assertTrue(
                        "expected message fragment '" + fragment + "' in: "
                                + expected.getMessage(),
                        expected.getMessage() != null
                                && expected.getMessage().contains(fragment));
            }
        }
        assertEquals(0, hardwareMap.lookupCalls());
        assertEquals(0, hardwareMap.totalMotorPowerWrites());
        assertEquals(DcMotorSimple.Direction.FORWARD, motor.direction());
    }

    private static void assertAcceptedPower(double collectPower) {
        AdaptiveCollectionIntake.Config config = AdaptiveCollectionIntake.Config.defaults();
        config.collectPower = collectPower;
        FtcTestHardware hardwareMap =
                new FtcTestHardware();
        FtcTestHardware.MotorProbe motor = hardwareMap.addMotor(config.motorName);
        AdaptiveCollectionIntake mechanism = new AdaptiveCollectionIntake(hardwareMap, config);
        ManualLoopClock time = new ManualLoopClock();

        mechanism.collectTask(0.50).start(time.clock());
        mechanism.update(time.clock());

        assertEquals(collectPower, motor.power(), 0.0);
        mechanism.stop();
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
