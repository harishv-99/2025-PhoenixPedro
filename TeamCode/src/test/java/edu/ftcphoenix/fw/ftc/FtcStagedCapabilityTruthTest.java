package edu.ftcphoenix.fw.ftc;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.Test;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.Plants;
import edu.ftcphoenix.fw.actuation.PositionPlant;
import edu.ftcphoenix.fw.core.hal.Direction;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Proves device-managed FTC terminal continuations expose only their declared capabilities. */
public final class FtcStagedCapabilityTruthTest {

    @Test
    public void deviceManagedVelocityTailStaysNarrowThroughGuardsAndBuildSelection() {
        Plants.TargetStep<Plant> target = FtcActuators.plant(emptyHardwareMap())
                .motor("flywheel", Direction.FORWARD)
                .velocity()
                .deviceManaged()
                .unbounded()
                .nativeUnits()
                .velocityTolerance(0.0);

        assertVelocityTargetOnly(target);
        Plants.TargetGuardStep<Plant> guards = target.targetGuards();
        assertFalse(guards instanceof Plants.VelocityControlStep);
        assertIllegalArgument(() -> guards.maxTargetRate(Double.NaN));

        Plants.TargetStep<Plant> afterGuards = guards.maxTargetRate(10.0)
                .doneTargetGuards();
        assertVelocityTargetOnly(afterGuards);
        Plants.BuildStep<Plant> build = afterGuards.targetFromNewCommand(0.0);
        assertNotNull(build);
        assertFalse(build instanceof Plants.TargetStep);
        assertFalse(build instanceof Plants.VelocityControlStep);
    }

    @Test
    public void tunedDeviceManagedVelocityUsesTheSameNarrowTail() {
        Plants.TargetStep<Plant> target = FtcActuators.plant(emptyHardwareMap())
                .motor("flywheel", Direction.FORWARD)
                .velocity()
                .deviceManagedWithOverrides()
                .velocityPidf(1.0, 0.0, 0.0, 0.0)
                .unbounded()
                .nativeUnits()
                .velocityTolerance(0.0);

        assertVelocityTargetOnly(target);
    }

    @Test
    public void deviceManagedPositionTailExposesOnlySymmetricPowerThenStaysNarrow() {
        Plants.SymmetricOutputPowerPolicyStep<PositionPlant> outputPolicy =
                FtcActuators.plant(emptyHardwareMap())
                        .motor("lift", Direction.FORWARD)
                        .position()
                        .deviceManaged()
                        .nonPeriodic()
                        .unbounded()
                        .nativeUnits()
                        .alreadyReferenced()
                        .positionTolerance(0.0);

        assertSymmetricPositionPolicyOnly(outputPolicy);
        assertIllegalArgument(() -> outputPolicy.outputPowerLimitedTo(Double.NaN));

        Plants.TargetStep<PositionPlant> target = outputPolicy.outputPowerLimitedTo(0.5);
        assertPositionTargetOnly(target);
        Plants.TargetStep<PositionPlant> afterGuards = target.targetGuards()
                .holdLastTargetUnless("ready", clock -> true)
                .doneTargetGuards();
        assertPositionTargetOnly(afterGuards);
        Plants.BuildStep<PositionPlant> build = afterGuards.targetFromNewCommand(0.0);
        assertNotNull(build);
        assertFalse(build instanceof Plants.TargetStep);
        assertFalse(build instanceof Plants.PositionControlStep);
        assertFalse(build instanceof Plants.OutputPowerPolicyStep);
    }

    @Test
    public void tunedDeviceManagedPositionUsesTheSameSymmetricOnlyTail() {
        Plants.SymmetricOutputPowerPolicyStep<PositionPlant> outputPolicy =
                FtcActuators.plant(emptyHardwareMap())
                        .motor("lift", Direction.FORWARD)
                        .position()
                        .deviceManagedWithOverrides()
                        .outerPositionP(1.0)
                        .doneOverrides()
                        .nonPeriodic()
                        .unbounded()
                        .nativeUnits()
                        .alreadyReferenced()
                        .positionTolerance(0.0);

        assertSymmetricPositionPolicyOnly(outputPolicy);
    }

    private static HardwareMap emptyHardwareMap() {
        return new HardwareMap(null, null);
    }

    private static void assertVelocityTargetOnly(Plants.TargetStep<Plant> target) {
        assertNotNull(target);
        assertFalse(target instanceof Plants.VelocityControlStep);
        assertFalse(target instanceof Plants.VelocityDirectFeedbackStep);
        assertFalse(target instanceof Plants.VelocityProfiledFeedbackStep);
        assertFalse(target instanceof Plants.OutputPowerPolicyStep);
    }

    private static void assertSymmetricPositionPolicyOnly(
            Plants.SymmetricOutputPowerPolicyStep<PositionPlant> outputPolicy) {
        assertNotNull(outputPolicy);
        assertTrue(outputPolicy instanceof Plants.SymmetricOutputPowerPolicyStep);
        assertFalse(outputPolicy instanceof Plants.OutputPowerPolicyStep);
        assertFalse(outputPolicy instanceof Plants.PositionControlStep);
        assertFalse(outputPolicy instanceof Plants.PositionDirectFeedbackStep);
        assertFalse(outputPolicy instanceof Plants.PositionProfiledFeedbackStep);
    }

    private static void assertPositionTargetOnly(Plants.TargetStep<PositionPlant> target) {
        assertNotNull(target);
        assertFalse(target instanceof Plants.SymmetricOutputPowerPolicyStep);
        assertFalse(target instanceof Plants.OutputPowerPolicyStep);
        assertFalse(target instanceof Plants.PositionControlStep);
        assertFalse(target instanceof Plants.PositionDirectFeedbackStep);
        assertFalse(target instanceof Plants.PositionProfiledFeedbackStep);
    }

    private static void assertIllegalArgument(Runnable action) {
        try {
            action.run();
            fail("Expected IllegalArgumentException");
        } catch (IllegalArgumentException expected) {
            // Expected; the same stage is deliberately retried by each caller above.
        }
    }
}
