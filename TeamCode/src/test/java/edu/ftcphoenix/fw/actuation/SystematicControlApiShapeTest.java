package edu.ftcphoenix.fw.actuation;

import org.junit.Test;

import java.lang.reflect.Method;
import java.lang.reflect.ParameterizedType;
import java.lang.reflect.Type;
import java.util.function.DoubleUnaryOperator;

import edu.ftcphoenix.fw.core.control.PidController;
import edu.ftcphoenix.fw.core.control.PidfRegulator;
import edu.ftcphoenix.fw.core.control.ScalarRegulator;
import edu.ftcphoenix.fw.core.control.ScalarRegulators;
import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.hal.PositionOutput;
import edu.ftcphoenix.fw.core.hal.PowerLimitedPositionOutput;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.hal.VelocityOutput;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.ftc.FtcActuators;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

/** Locks the public staged grammar chosen by CTRL-02. */
public final class SystematicControlApiShapeTest {

    @Test
    public void portableRootsExposeOutputPowerOnlyWithTruthfulCapabilityEvidence()
            throws Exception {
        Method plainPosition = declared(
                Plants.FromOutputsStep.class,
                "deviceManagedPosition",
                PositionOutput.class,
                ScalarSource.class);
        assertGenericReturn(plainPosition,
                type(Plants.DeviceManagedPositionStep.class,
                        type(Plants.TargetStep.class, type(PositionPlant.class))));

        Method powerLimitedPosition = declared(
                Plants.FromOutputsStep.class,
                "deviceManagedPosition",
                PowerLimitedPositionOutput.class,
                ScalarSource.class);
        assertGenericReturn(powerLimitedPosition,
                type(Plants.DeviceManagedPositionStep.class,
                        type(Plants.SymmetricOutputPowerPolicyStep.class,
                                type(PositionPlant.class))));

        assertGenericReturn(declared(
                        Plants.FromOutputsStep.class,
                        "deviceManagedVelocity",
                        VelocityOutput.class,
                        ScalarSource.class),
                type(Plants.VelocityBoundsStep.class,
                        type(Plants.TargetStep.class, type(Plant.class))));
        assertGenericReturn(declared(
                        Plants.FromOutputsStep.class,
                        "regulatedPosition",
                        PowerOutput.class,
                        ScalarSource.class),
                type(Plants.PositionPeriodicityStep.class,
                        type(Plants.FeedbackPositionBoundsStep.class,
                                type(Plants.PositionControlStep.class))));
        assertGenericReturn(declared(
                        Plants.FromOutputsStep.class,
                        "regulatedVelocity",
                        PowerOutput.class,
                        ScalarSource.class),
                type(Plants.VelocityBoundsStep.class,
                        type(Plants.VelocityControlStep.class)));
    }

    @Test
    public void portableControlStagesHaveOnlyTheApprovedMethodsAndReturns() throws Exception {
        assertDeclaredMethodCount(Plants.SymmetricOutputPowerPolicyStep.class, 1);
        assertRawReturn(Plants.SymmetricOutputPowerPolicyStep.class,
                "outputPowerLimitedTo", Plants.TargetStep.class, double.class);

        assertDeclaredMethodCount(Plants.OutputPowerPolicyStep.class, 2);
        assertRawReturn(Plants.OutputPowerPolicyStep.class,
                "outputPowerLimitedTo", Plants.TargetStep.class, double.class, double.class);
        assertRawReturn(Plants.OutputPowerPolicyStep.class,
                "voltageCompensationFrom", Plants.OutputPowerAfterVoltageStep.class,
                ScalarSource.class, double.class, double.class, double.class);

        assertDeclaredMethodCount(Plants.OutputPowerAfterVoltageStep.class, 1);
        assertRawReturn(Plants.OutputPowerAfterVoltageStep.class,
                "outputPowerLimitedTo", Plants.TargetStep.class, double.class, double.class);

        assertDeclaredMethodCount(Plants.PositionControlStep.class, 3);
        assertRawReturn(Plants.PositionControlStep.class,
                "setpointFromAppliedTarget", Plants.PositionDirectFeedbackStep.class);
        assertRawReturn(Plants.PositionControlStep.class,
                "setpointFromTrapezoidalProfile", Plants.PositionProfiledFeedbackStep.class,
                double.class, double.class);
        assertGenericReturn(declared(Plants.PositionControlStep.class,
                        "controlFromCustomRegulator", ScalarRegulator.class),
                type(Plants.OutputPowerPolicyStep.class, type(PositionPlant.class)));

        assertPidFeedbackStage(Plants.PositionDirectFeedbackStep.class,
                Plants.PositionDirectPidStep.class);
        assertPidFeedbackStage(Plants.PositionProfiledFeedbackStep.class,
                Plants.PositionProfiledPidStep.class);
        assertPositionDirectPidStage();
        assertPositionProfiledPidStage();

        assertDeclaredMethodCount(Plants.VelocityControlStep.class, 3);
        assertRawReturn(Plants.VelocityControlStep.class,
                "setpointFromAppliedTarget", Plants.VelocityDirectFeedbackStep.class);
        assertRawReturn(Plants.VelocityControlStep.class,
                "setpointFromAccelerationLimitedProfile",
                Plants.VelocityProfiledFeedbackStep.class, double.class);
        assertGenericReturn(declared(Plants.VelocityControlStep.class,
                        "controlFromCustomRegulator", ScalarRegulator.class),
                type(Plants.OutputPowerPolicyStep.class, type(Plant.class)));

        assertPidFeedbackStage(Plants.VelocityDirectFeedbackStep.class,
                Plants.VelocityDirectPidStep.class);
        assertPidFeedbackStage(Plants.VelocityProfiledFeedbackStep.class,
                Plants.VelocityProfiledPidStep.class);
        assertVelocityDirectPidStage();
        assertVelocityProfiledPidStage();
    }

    @Test
    public void ftcOrdinaryAndOverrideBranchesHaveDistinctCompileTimeGrammar()
            throws Exception {
        assertDeclaredMethodCount(FtcActuators.MotorVelocityControlStep.class, 3);
        assertGenericReturn(declared(FtcActuators.MotorVelocityControlStep.class,
                        "deviceManaged"),
                type(Plants.VelocityBoundsStep.class,
                        type(Plants.TargetStep.class, type(Plant.class))));
        assertRawReturn(FtcActuators.MotorVelocityControlStep.class,
                "deviceManagedWithOverrides",
                FtcActuators.MotorDeviceManagedVelocityStep.class);
        assertRawReturn(FtcActuators.MotorVelocityControlStep.class,
                "regulated", FtcActuators.MotorRegulatedVelocityFeedbackStep.class);

        assertDeclaredMethodCount(FtcActuators.MotorDeviceManagedVelocityStep.class, 1);
        assertGenericReturn(declared(FtcActuators.MotorDeviceManagedVelocityStep.class,
                        "velocityPidf", double.class, double.class, double.class, double.class),
                type(Plants.VelocityBoundsStep.class,
                        type(Plants.TargetStep.class, type(Plant.class))));

        ExpectedType devicePositionContinuation = type(
                Plants.PositionPeriodicityStep.class,
                type(Plants.FeedbackPositionBoundsStep.class,
                        type(Plants.SymmetricOutputPowerPolicyStep.class,
                                type(PositionPlant.class))));
        assertDeclaredMethodCount(FtcActuators.MotorPositionControlStep.class, 3);
        assertGenericReturn(declared(FtcActuators.MotorPositionControlStep.class,
                "deviceManaged"), devicePositionContinuation);
        assertRawReturn(FtcActuators.MotorPositionControlStep.class,
                "deviceManagedWithOverrides",
                FtcActuators.MotorDeviceManagedPositionStep.class);
        assertRawReturn(FtcActuators.MotorPositionControlStep.class,
                "regulated", FtcActuators.MotorRegulatedPositionFeedbackStep.class);

        assertDeclaredMethodCount(FtcActuators.MotorDeviceManagedPositionStep.class, 4);
        assertRawReturn(FtcActuators.MotorDeviceManagedPositionStep.class,
                "outerPositionP", FtcActuators.MotorDeviceManagedPositionStep.class,
                double.class);
        assertRawReturn(FtcActuators.MotorDeviceManagedPositionStep.class,
                "innerVelocityPidf", FtcActuators.MotorDeviceManagedPositionStep.class,
                double.class, double.class, double.class, double.class);
        assertRawReturn(FtcActuators.MotorDeviceManagedPositionStep.class,
                "devicePositionToleranceTicks",
                FtcActuators.MotorDeviceManagedPositionStep.class, int.class);
        assertGenericReturn(declared(FtcActuators.MotorDeviceManagedPositionStep.class,
                "doneOverrides"), devicePositionContinuation);
    }

    @Test
    public void ftcRegulatedFeedbackEntriesHaveExactParallelGrammar() throws Exception {
        Class<?> motorVelocity = FtcActuators.MotorRegulatedVelocityFeedbackStep.class;
        ExpectedType velocityNext = type(
                Plants.VelocityBoundsStep.class,
                type(Plants.VelocityControlStep.class));
        assertDeclaredMethodCount(motorVelocity, 6);
        assertGenericReturn(declared(motorVelocity, "internalEncoder"), velocityNext);
        assertGenericReturn(declared(motorVelocity, "internalEncoder", String.class),
                velocityNext);
        assertGenericReturn(declared(motorVelocity, "averageInternalEncoders"), velocityNext);
        assertGenericReturn(declared(motorVelocity, "externalEncoder", String.class),
                velocityNext);
        assertGenericReturn(declared(motorVelocity,
                "externalEncoder", String.class, Direction.class), velocityNext);
        assertGenericReturn(declared(motorVelocity, "nativeFeedback", ScalarSource.class),
                velocityNext);

        ExpectedType positionNext = type(
                Plants.PositionPeriodicityStep.class,
                type(Plants.FeedbackPositionBoundsStep.class,
                        type(Plants.PositionControlStep.class)));
        Class<?> motorPosition = FtcActuators.MotorRegulatedPositionFeedbackStep.class;
        assertDeclaredMethodCount(motorPosition, 6);
        assertGenericReturn(declared(motorPosition, "internalEncoder"), positionNext);
        assertGenericReturn(declared(motorPosition, "internalEncoder", String.class),
                positionNext);
        assertGenericReturn(declared(motorPosition, "averageInternalEncoders"), positionNext);
        assertGenericReturn(declared(motorPosition, "externalEncoder", String.class),
                positionNext);
        assertGenericReturn(declared(motorPosition,
                "externalEncoder", String.class, Direction.class), positionNext);
        assertGenericReturn(declared(motorPosition, "nativeFeedback", ScalarSource.class),
                positionNext);

        Class<?> crServoPosition = FtcActuators.CrServoRegulatedPositionFeedbackStep.class;
        assertDeclaredMethodCount(crServoPosition, 3);
        assertGenericReturn(declared(crServoPosition, "externalEncoder", String.class),
                positionNext);
        assertGenericReturn(declared(crServoPosition,
                "externalEncoder", String.class, Direction.class), positionNext);
        assertGenericReturn(declared(crServoPosition, "nativeFeedback", ScalarSource.class),
                positionNext);
    }

    @Test
    public void onlyTheApprovedLegacyControllerFactoriesAreDeprecated() throws Exception {
        assertTrue(PidfRegulator.class.isAnnotationPresent(Deprecated.class));
        assertTrue(declared(ScalarRegulators.class, "pid", PidController.class)
                .isAnnotationPresent(Deprecated.class));
        assertTrue(declared(ScalarRegulators.class, "pidf",
                double.class, double.class, double.class, double.class)
                .isAnnotationPresent(Deprecated.class));
        assertTrue(declared(ScalarRegulators.class, "setpointFeedforward",
                ScalarRegulator.class, DoubleUnaryOperator.class)
                .isAnnotationPresent(Deprecated.class));

        assertFalse(ScalarRegulator.class.isAnnotationPresent(Deprecated.class));
        assertFalse(declared(ScalarRegulators.class, "voltageCompensated",
                ScalarRegulator.class, ScalarSource.class,
                double.class, double.class, double.class)
                .isAnnotationPresent(Deprecated.class));
        assertFalse(declared(ScalarRegulators.class, "outputLimited",
                ScalarRegulator.class, double.class, double.class)
                .isAnnotationPresent(Deprecated.class));
    }

    private static void assertPositionDirectPidStage() throws Exception {
        Class<?> stage = Plants.PositionDirectPidStep.class;
        assertDeclaredMethodCount(stage, 4);
        assertRawReturn(stage, "feedbackIntegralLimitedTo", stage,
                double.class, double.class);
        assertRawReturn(stage, "feedbackOutputLimitedTo", stage,
                double.class, double.class);
        assertRawReturn(stage, "feedforwardFromLift", Plants.OutputPowerPolicyStep.class,
                double.class);
        assertRawReturn(stage, "feedforwardFromArm", Plants.OutputPowerPolicyStep.class,
                double.class, double.class, double.class);
    }

    private static void assertPositionProfiledPidStage() throws Exception {
        Class<?> stage = Plants.PositionProfiledPidStep.class;
        assertDeclaredMethodCount(stage, 8);
        assertRawReturn(stage, "feedbackIntegralLimitedTo", stage,
                double.class, double.class);
        assertRawReturn(stage, "feedbackOutputLimitedTo", stage,
                double.class, double.class);
        assertRawReturn(stage, "feedforwardFromMotion", Plants.OutputPowerPolicyStep.class,
                double.class);
        assertRawReturn(stage, "feedforwardFromMotion", Plants.OutputPowerPolicyStep.class,
                double.class, double.class, double.class);
        assertRawReturn(stage, "feedforwardFromLift", Plants.OutputPowerPolicyStep.class,
                double.class);
        assertRawReturn(stage, "feedforwardFromLift", Plants.OutputPowerPolicyStep.class,
                double.class, double.class, double.class, double.class);
        assertRawReturn(stage, "feedforwardFromArm", Plants.OutputPowerPolicyStep.class,
                double.class, double.class, double.class);
        assertRawReturn(stage, "feedforwardFromArm", Plants.OutputPowerPolicyStep.class,
                double.class, double.class, double.class,
                double.class, double.class, double.class);
    }

    private static void assertVelocityDirectPidStage() throws Exception {
        Class<?> stage = Plants.VelocityDirectPidStep.class;
        assertDeclaredMethodCount(stage, 6);
        assertRawReturn(stage, "feedbackIntegralLimitedTo", stage,
                double.class, double.class);
        assertRawReturn(stage, "feedbackOutputLimitedTo", stage,
                double.class, double.class);
        assertRawReturn(stage, "feedforwardFromMotion", Plants.OutputPowerPolicyStep.class,
                double.class);
        assertRawReturn(stage, "feedforwardFromMotion", Plants.OutputPowerPolicyStep.class,
                double.class, double.class);
        assertRawReturn(stage, "feedforwardFromLift", Plants.OutputPowerPolicyStep.class,
                double.class);
        assertRawReturn(stage, "feedforwardFromLift", Plants.OutputPowerPolicyStep.class,
                double.class, double.class, double.class);
    }

    private static void assertVelocityProfiledPidStage() throws Exception {
        Class<?> stage = Plants.VelocityProfiledPidStep.class;
        assertDeclaredMethodCount(stage, 6);
        assertRawReturn(stage, "feedbackIntegralLimitedTo", stage,
                double.class, double.class);
        assertRawReturn(stage, "feedbackOutputLimitedTo", stage,
                double.class, double.class);
        assertRawReturn(stage, "feedforwardFromMotion", Plants.OutputPowerPolicyStep.class,
                double.class);
        assertRawReturn(stage, "feedforwardFromMotion", Plants.OutputPowerPolicyStep.class,
                double.class, double.class, double.class);
        assertRawReturn(stage, "feedforwardFromLift", Plants.OutputPowerPolicyStep.class,
                double.class);
        assertRawReturn(stage, "feedforwardFromLift", Plants.OutputPowerPolicyStep.class,
                double.class, double.class, double.class, double.class);
    }

    private static void assertPidFeedbackStage(Class<?> stage, Class<?> pidStage)
            throws Exception {
        assertDeclaredMethodCount(stage, 2);
        assertRawReturn(stage, "feedbackFromPid", pidStage, double.class);
        assertRawReturn(stage, "feedbackFromPid", pidStage,
                double.class, double.class, double.class);
    }

    private static void assertDeclaredMethodCount(Class<?> owner, int expected) {
        assertEquals(owner.getSimpleName(), expected, owner.getDeclaredMethods().length);
    }

    private static void assertRawReturn(Class<?> owner,
                                        String name,
                                        Class<?> expectedReturn,
                                        Class<?>... parameterTypes) throws Exception {
        assertSame(expectedReturn, declared(owner, name, parameterTypes).getReturnType());
    }

    private static Method declared(Class<?> owner,
                                   String name,
                                   Class<?>... parameterTypes) throws Exception {
        return owner.getDeclaredMethod(name, parameterTypes);
    }

    private static ExpectedType type(Class<?> raw, ExpectedType... arguments) {
        return new ExpectedType(raw, arguments);
    }

    private static void assertGenericReturn(Method method, ExpectedType expected) {
        assertType(method.toString(), method.getGenericReturnType(), expected);
    }

    private static void assertType(String context, Type actual, ExpectedType expected) {
        if (expected.arguments.length == 0) {
            assertSame(context, expected.raw, actual);
            return;
        }
        assertTrue(context + " should return a parameterized type",
                actual instanceof ParameterizedType);
        ParameterizedType parameterized = (ParameterizedType) actual;
        assertSame(context, expected.raw, parameterized.getRawType());
        Type[] actualArguments = parameterized.getActualTypeArguments();
        assertEquals(context, expected.arguments.length, actualArguments.length);
        for (int i = 0; i < actualArguments.length; i++) {
            assertType(context + " argument " + i, actualArguments[i], expected.arguments[i]);
        }
    }

    private static final class ExpectedType {
        private final Class<?> raw;
        private final ExpectedType[] arguments;

        private ExpectedType(Class<?> raw, ExpectedType[] arguments) {
            this.raw = raw;
            this.arguments = arguments;
        }
    }
}
