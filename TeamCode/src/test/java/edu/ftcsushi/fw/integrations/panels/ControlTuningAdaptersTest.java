package edu.ftcsushi.fw.integrations.panels;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.junit.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

import edu.ftcsushi.fw.actuation.Plant;
import edu.ftcsushi.fw.actuation.Plants;
import edu.ftcsushi.fw.actuation.PositionPlant;
import edu.ftcsushi.fw.actuation.ScalarRange;
import edu.ftcsushi.fw.core.hal.PowerOutput;
import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.ftc.FtcActuators;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Typed-adapter coverage against real Sushi standard-control Plants. */
public final class ControlTuningAdaptersTest {

    @Test
    public void velocityKvOnlyTopologyExposesOnlyActiveFieldsAndPartialReseedEvidence() {
        Plant plant = Plants.fromOutputs()
                .regulatedVelocity(new RecordingPowerOutput(), clock -> 2.0)
                .bounded(-20.0, 20.0)
                .nativeUnits()
                .velocityTolerance(0.1)
                .setpointFromAppliedTarget()
                .feedbackFromPid(0.1, 0.2, 0.3)
                .feedforwardFromMotion(0.4)
                .targetFromNewCommand(0.0)
                .build();
        ControlTuningModel.Session session = ControlTuningAdapters.claimVelocity(plant);
        assertEquals(Arrays.asList(
                        "feedback.kP", "feedback.kI", "feedback.kD", "feedforward.kV"),
                Arrays.asList(session.initialCandidate().values().keySet().toArray(
                        new String[0])));

        ManualLoopClock time = new ManualLoopClock();
        plant.update(time.clock());
        ControlTuningModel.Parameters candidate = withValues(
                session.initialCandidate(),
                "feedback.kP", 0.7,
                "feedforward.kV", 0.8);
        session.apply(candidate, time.clock());

        ControlTuningModel.Evidence reseeded = session.evidence(time.clock());
        assertEquals("false", reseeded.text().get("completeEvaluation"));
        assertEquals(2.0, reseeded.numeric().get("measurement"), 0.0);
        assertFalse(reseeded.numeric().containsKey("feedbackOutput"));
        assertEquals(candidate.values(), session.readbacks().get(0).parameters.values());
    }

    @Test
    public void fullLiftCandidateKeepsGravityMotionParameterMeanings() {
        Plant plant = Plants.fromOutputs()
                .regulatedVelocity(new RecordingPowerOutput(), clock -> 0.0)
                .bounded(-20.0, 20.0)
                .nativeUnits()
                .velocityTolerance(0.1)
                .setpointFromAccelerationLimitedProfile(10.0)
                .feedbackFromPid(0.1)
                .feedforwardFromLift(0.01, 0.02, 0.03, 0.04)
                .targetFromNewCommand(0.0)
                .build();
        ControlTuningModel.Session session = ControlTuningAdapters.claimVelocity(plant);
        assertEquals(Arrays.asList(
                        "feedback.kP", "feedback.kI", "feedback.kD",
                        "feedforward.kG", "feedforward.kS",
                        "feedforward.kV", "feedforward.kA"),
                Arrays.asList(session.initialCandidate().values().keySet().toArray(
                        new String[0])));
        ManualLoopClock time = new ManualLoopClock();
        plant.update(time.clock());
        ControlTuningModel.Parameters candidate = withValues(
                session.initialCandidate(),
                "feedforward.kG", 1.1,
                "feedforward.kS", 1.2,
                "feedforward.kV", 1.3,
                "feedforward.kA", 1.4);

        session.apply(candidate, time.clock());

        assertEquals(candidate.values(), session.readbacks().get(0).parameters.values());
    }

    @Test
    public void fullArmCandidateKeepsGravityMotionParameterMeanings() {
        PositionPlant plant = Plants.fromOutputs()
                .regulatedPosition(new RecordingPowerOutput(), clock -> 0.0)
                .nonPeriodic()
                .bounded(-20.0, 20.0)
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(0.1)
                .setpointFromTrapezoidalProfile(10.0, 20.0)
                .feedbackFromPid(0.1)
                .feedforwardFromArm(0.01, 0.0, 1.0, 0.02, 0.03, 0.04)
                .targetFromNewCommand(0.0)
                .build();
        ControlTuningModel.Session session = ControlTuningAdapters.claimPosition(plant);
        assertEquals(Arrays.asList(
                        "feedback.kP", "feedback.kI", "feedback.kD",
                        "feedforward.kG", "feedforward.kS",
                        "feedforward.kV", "feedforward.kA"),
                Arrays.asList(session.initialCandidate().values().keySet().toArray(
                        new String[0])));
        ManualLoopClock time = new ManualLoopClock();
        session.preparePositionHold(time.clock());
        plant.update(time.clock());
        ControlTuningModel.Parameters candidate = withValues(
                session.initialCandidate(),
                "feedforward.kG", 1.1,
                "feedforward.kS", 1.2,
                "feedforward.kV", 1.3,
                "feedforward.kA", 1.4);

        session.apply(candidate, time.clock());

        assertEquals(candidate.values(), session.readbacks().get(0).parameters.values());
    }

    @Test
    public void positionRecoveryReseedsProfileAtSameCycleMeasurementBeforeOutput() {
        final double[] measured = {20.0};
        RecordingPowerOutput output = new RecordingPowerOutput();
        PositionPlant plant = Plants.fromOutputs()
                .regulatedPosition(output, clock -> measured[0])
                .nonPeriodic()
                .bounded(-100.0, 100.0)
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(0.1)
                .setpointFromTrapezoidalProfile(50.0, 100.0)
                .feedbackFromPid(0.1)
                .targetFromNewCommand(20.0)
                .build();
        ControlTuningModel.Session session = ControlTuningAdapters.claimPosition(plant);
        ManualLoopClock time = new ManualLoopClock();
        assertEquals(20.0, session.preparePositionHold(time.clock()), 0.0);
        plant.update(time.clock());

        plant.commandTarget().set(80.0);
        plant.update(time.nextCycle(0.10));
        plant.update(time.nextCycle(0.10));
        assertTrue(session.evidence(time.clock()).numeric().get("setpointVelocity") > 0.0);

        measured[0] = 40.0;
        time.nextCycle(0.10);
        ControlTuningModel.PositionRecoveryHold hold = session.preparePositionRecoveryHold(
                ScalarRange.bounded(0.0, 100.0), time.clock());
        assertEquals(40.0, hold.measurement, 0.0);
        assertEquals(40.0, hold.holdTarget, 0.0);
        assertFalse(hold.clamped);
        assertEquals(40.0, plant.commandTarget().get(), 0.0);
        assertEquals(0.0,
                session.evidence(time.clock()).numeric().get("setpointVelocity"), 0.0);
        int writesBeforeRecoveryHeartbeat = output.writes;

        plant.update(time.clock());

        assertEquals(writesBeforeRecoveryHeartbeat + 1, output.writes);
        assertEquals(40.0,
                session.evidence(time.clock()).numeric().get("setpointPosition"), 0.0);
        assertEquals(0.0,
                session.evidence(time.clock()).numeric().get("setpointVelocity"), 0.0);

        plant.stop();
        session.restoreInitial(time.clock());
    }

    @Test
    public void periodicArmRequiresGravityModelToRepeatOverPlantPeriod() {
        PositionPlant compatible = periodicArmPlant(360.0, Math.PI / 180.0);
        ControlTuningModel.Session accepted = ControlTuningAdapters.claimPosition(compatible);
        assertTrue(accepted.topology().contains("ARM"));

        PositionPlant incompatible = periodicArmPlant(360.0, Math.PI / 200.0);
        try {
            ControlTuningAdapters.claimPosition(incompatible);
            fail("expected incompatible periodic arm rejection");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage(),
                    expected.getMessage().contains("integer multiple of 2*pi"));
        }
    }

    @Test
    public void divergentFtcGroupDraftSeedRequiresOnlyOneSuccessfulInitialApply() {
        TestHardwareMap map = new TestHardwareMap();
        map.addMotor("left", pidf(
                1.0, 2.0, 3.0, 4.0, MotorControlAlgorithm.LegacyPID));
        map.addMotor("right", pidf(
                1.0, 2.0, 3.0, 4.0, MotorControlAlgorithm.PIDF));
        Plant plant = FtcActuators.plant(map)
                .motor("left", Direction.FORWARD)
                .andMotor("right", Direction.REVERSE)
                .scale(-1.0)
                .velocity()
                .deviceManaged()
                .bounded(-100.0, 100.0)
                .nativeUnits()
                .velocityTolerance(1.0)
                .targetFromNewCommand(0.0)
                .build();
        ControlTuningModel.Session session = ControlTuningAdapters.claimVelocity(plant);
        assertTrue(session.requiresInitialApply());

        ControlTuningModel.Parameters outsideFtcDomain = withValues(
                session.initialCandidate(), "velocity.kP", 40000.0);
        try {
            session.validate(outsideFtcDomain);
            fail("expected FTC controller-domain rejection");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("REV conversion domain"));
        }
        assertTrue("validation must not consume the first required shared apply",
                session.requiresInitialApply());

        ManualLoopClock time = new ManualLoopClock();
        plant.update(time.clock());
        assertTrue(session.evidence(time.clock()).text().containsKey("initialCandidateSeed"));
        assertTrue(session.readyForReconfiguration(time.clock()));
        session.apply(session.initialCandidate(), time.clock());

        assertFalse(session.requiresInitialApply());
        assertFalse(session.evidence(time.clock()).text().containsKey("initialCandidateSeed"));
        for (ControlTuningModel.Readback readback : session.readbacks()) {
            assertEquals(session.initialCandidate().values(), readback.parameters.values());
        }
    }

    @Test
    public void groupedVelocityCompletionRejectsOpposingMemberErrorsHiddenByAverage() {
        TestHardwareMap map = new TestHardwareMap();
        MotorProbe left = map.addMotor("left", pidf(1.0, 0.0, 0.0, 0.0));
        MotorProbe right = map.addMotor("right", pidf(1.0, 0.0, 0.0, 0.0));
        left.velocityMeasurement = 12.0;
        right.velocityMeasurement = -8.0;
        Plant plant = FtcActuators.plant(map)
                .motor("left", Direction.FORWARD)
                .andMotor("right", Direction.REVERSE)
                .scale(-1.0)
                .velocity()
                .deviceManaged()
                .bounded(-100.0, 100.0)
                .nativeUnits()
                .velocityTolerance(1.0)
                .targetFromNewCommand(10.0)
                .build();
        ControlTuningModel.Session session = ControlTuningAdapters.claimVelocity(plant);
        ManualLoopClock time = new ManualLoopClock();

        plant.update(time.clock());

        assertEquals(10.0, plant.getMeasurement(), 0.0);
        assertTrue(plant.atTarget(10.0));
        assertFalse(session.experimentAtTarget(true, time.clock()));
    }

    @Test
    public void ftcCleanupSkipsUntouchedSessionAndRestoresSuccessfulApplyOnce() {
        TestHardwareMap map = new TestHardwareMap();
        MotorProbe motor = map.addMotor("motor", pidf(1.0, 0.0, 0.0, 0.0));
        Plant plant = FtcActuators.plant(map)
                .motor("motor", Direction.FORWARD)
                .velocity()
                .deviceManaged()
                .bounded(-100.0, 100.0)
                .nativeUnits()
                .velocityTolerance(1.0)
                .targetFromNewCommand(0.0)
                .build();
        ControlTuningModel.Session session = ControlTuningAdapters.claimVelocity(plant);

        session.restoreInitial(new ManualLoopClock().clock());
        assertEquals("untouched cleanup must not manufacture a controller write",
                0, motor.generalSetWrites);

        ManualLoopClock time = new ManualLoopClock();
        plant.update(time.clock());
        session.apply(withValues(session.initialCandidate(), "velocity.kP", 2.0), time.clock());
        assertEquals(1, motor.velocitySetWrites);
        session.restoreInitial(time.clock());
        assertEquals(1, motor.generalSetWrites);
        session.restoreInitial(time.clock());
        assertEquals("successful restore clears the adapter obligation",
                1, motor.generalSetWrites);
    }

    @Test
    public void partiallyFailedFtcGroupApplyStillRequiresRestoration() {
        TestHardwareMap map = new TestHardwareMap();
        MotorProbe left = map.addMotor("left", pidf(1.0, 0.0, 0.0, 0.0));
        MotorProbe right = map.addMotor("right", pidf(1.0, 0.0, 0.0, 0.0));
        Plant plant = FtcActuators.plant(map)
                .motor("left", Direction.FORWARD)
                .andMotor("right", Direction.REVERSE)
                .scale(-1.0)
                .velocity()
                .deviceManaged()
                .bounded(-100.0, 100.0)
                .nativeUnits()
                .velocityTolerance(1.0)
                .targetFromNewCommand(0.0)
                .build();
        ControlTuningModel.Session session = ControlTuningAdapters.claimVelocity(plant);
        ManualLoopClock time = new ManualLoopClock();
        plant.update(time.clock());
        right.velocitySetFailure = new IllegalStateException("second setter failed");

        try {
            session.apply(withValues(session.initialCandidate(), "velocity.kP", 2.0), time.clock());
            fail("expected partial grouped apply failure");
        } catch (RuntimeException expected) {
            // Expected; restoration assertions below prove the adapter retained the obligation.
        }
        assertEquals(1, left.velocitySetWrites);
        session.restoreInitial(time.clock());
        assertEquals(1, left.generalSetWrites);
        assertEquals(1, right.generalSetWrites);
    }

    private static PositionPlant periodicArmPlant(double period, double radiansPerPlantUnit) {
        return Plants.fromOutputs()
                .regulatedPosition(new RecordingPowerOutput(), clock -> 0.0)
                .periodic(period)
                .bounded(-720.0, 720.0)
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(0.1)
                .setpointFromAppliedTarget()
                .feedbackFromPid(0.1)
                .feedforwardFromArm(0.2, 0.0, radiansPerPlantUnit)
                .targetFromNewCommand(0.0)
                .build();
    }

    private static ControlTuningModel.Parameters withValues(
            ControlTuningModel.Parameters baseline,
            Object... namesAndValues) {
        LinkedHashMap<String, Double> values =
                new LinkedHashMap<String, Double>(baseline.values());
        for (int index = 0; index < namesAndValues.length; index += 2) {
            values.put((String) namesAndValues[index], (Double) namesAndValues[index + 1]);
        }
        return new ControlTuningModel.Parameters(values);
    }

    private static final class RecordingPowerOutput implements PowerOutput {
        double commandedPower = Double.NaN;
        int writes;

        @Override
        public void setPower(double power) {
            commandedPower = power;
            writes++;
        }

        @Override
        public double getCommandedPower() {
            return commandedPower;
        }
    }

    private static PIDFCoefficients pidf(double p, double i, double d, double f) {
        return pidf(p, i, d, f, MotorControlAlgorithm.PIDF);
    }

    private static PIDFCoefficients pidf(
            double p, double i, double d, double f, MotorControlAlgorithm algorithm) {
        return new PIDFCoefficients(p, i, d, f, algorithm);
    }

    private static final class TestHardwareMap extends HardwareMap {
        private final Map<String, MotorProbe> motors = new HashMap<String, MotorProbe>();

        TestHardwareMap() {
            super(null, null);
        }

        MotorProbe addMotor(String name, PIDFCoefficients velocityPidf) {
            MotorProbe probe = new MotorProbe(name, velocityPidf);
            motors.put(name, probe);
            return probe;
        }

        @Override
        public <T> T get(Class<? extends T> type, String name) {
            MotorProbe probe = motors.get(name);
            if (probe == null || !type.isInstance(probe.motor)) {
                throw new IllegalArgumentException("No test motor named " + name);
            }
            return type.cast(probe.motor);
        }
    }

    private static final class MotorProbe {
        private final String name;
        private final DcMotorEx motor;
        private PIDFCoefficients velocityPidf;
        private double velocityMeasurement;
        private int velocitySetWrites;
        private int generalSetWrites;
        private RuntimeException velocitySetFailure;

        MotorProbe(String name, PIDFCoefficients velocityPidf) {
            this.name = name;
            this.velocityPidf = new PIDFCoefficients(velocityPidf);
            motor = (DcMotorEx) Proxy.newProxyInstance(
                    DcMotorEx.class.getClassLoader(),
                    new Class<?>[]{DcMotorEx.class},
                    this::invoke);
        }

        private Object invoke(Object proxy, Method method, Object[] args) {
            String operation = method.getName();
            if (method.getDeclaringClass() == Object.class) {
                if ("equals".equals(operation)) return proxy == args[0];
                if ("hashCode".equals(operation)) return System.identityHashCode(proxy);
                if ("toString".equals(operation)) return "MotorProbe(" + name + ')';
            }
            if ("getPIDFCoefficients".equals(operation)) {
                return new PIDFCoefficients(velocityPidf);
            }
            if ("setVelocityPIDFCoefficients".equals(operation)) {
                if (velocitySetFailure != null) {
                    RuntimeException failure = velocitySetFailure;
                    velocitySetFailure = null;
                    throw failure;
                }
                velocitySetWrites++;
                velocityPidf = pidf(
                        (double) args[0], (double) args[1],
                        (double) args[2], (double) args[3]);
                return null;
            }
            if ("setPIDFCoefficients".equals(operation)) {
                generalSetWrites++;
                velocityPidf = new PIDFCoefficients((PIDFCoefficients) args[1]);
                return null;
            }
            if ("getVelocity".equals(operation)) return velocityMeasurement;
            if ("setVelocity".equals(operation)) return null;
            if ("setDirection".equals(operation)) return null;
            if ("getDirection".equals(operation)) return DcMotorSimple.Direction.FORWARD;
            if ("setMode".equals(operation)) return null;
            if ("getMode".equals(operation)) return DcMotor.RunMode.RUN_USING_ENCODER;
            if ("setPower".equals(operation)) return null;
            if ("getPower".equals(operation)) return 0.0;
            if ("getManufacturer".equals(operation)) return HardwareDevice.Manufacturer.Other;
            if ("getDeviceName".equals(operation)) return "test motor";
            if ("getConnectionInfo".equals(operation)) return "test";
            if ("getVersion".equals(operation)) return 1;
            return defaultValue(method.getReturnType());
        }
    }

    private static Object defaultValue(Class<?> type) {
        if (!type.isPrimitive()) return null;
        if (type == boolean.class) return false;
        if (type == byte.class) return (byte) 0;
        if (type == short.class) return (short) 0;
        if (type == int.class) return 0;
        if (type == long.class) return 0L;
        if (type == float.class) return 0.0f;
        if (type == double.class) return 0.0;
        if (type == char.class) return '\0';
        return null;
    }
}
