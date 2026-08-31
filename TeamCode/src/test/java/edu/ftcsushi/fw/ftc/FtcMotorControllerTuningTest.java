package edu.ftcsushi.fw.ftc;

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
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.ftcsushi.fw.actuation.Plant;
import edu.ftcsushi.fw.actuation.PositionPlant;
import edu.ftcsushi.fw.actuation.PositionPlantTuning;
import edu.ftcsushi.fw.actuation.ScalarRange;
import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.core.time.LoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Focused TUNE-03 coverage for completed-Plant-derived FTC controller capabilities. */
public final class FtcMotorControllerTuningTest {

    @Test
    public void velocityFactoryAcceptsSingleAndGroupAndPermanentlySingleClaims() {
        TestHardwareMap map = new TestHardwareMap();
        MotorProbe left = map.addMotor("left", pidf(1, 2, 3, 4, MotorControlAlgorithm.LegacyPID));
        MotorProbe right = map.addMotor("right", pidf(5, 6, 7, 8, MotorControlAlgorithm.PIDF));
        Plant plant = velocityGroup(map, 2.0, 3.0);

        FtcMotorVelocityControl handle = FtcMotorControllers.velocityControl(plant);

        assertEquals(2, handle.memberCount());
        assertSame(handle.plantTargetRange(), handle.plantTargetRange());
        assertEquals(-100.0, handle.plantTargetRange().minValue, 0.0);
        assertEquals(100.0, handle.plantTargetRange().maxValue, 0.0);
        assertConfiguration(handle.initialConfigurations().get(0), "left", left.velocity());
        assertConfiguration(handle.initialConfigurations().get(1), "right", right.velocity());
        IllegalStateException duplicate = assertThrows(
                IllegalStateException.class,
                () -> FtcMotorControllers.velocityControl(plant));
        assertContains(duplicate, "already has");
    }

    @Test
    public void publicApiShapeUsesCompletedPlantFactoriesAndNoPublicHandleConstructors()
            throws Exception {
        Method velocity = FtcMotorControllers.class.getMethod("velocityControl", Plant.class);
        assertEquals(FtcMotorVelocityControl.class, velocity.getReturnType());
        Method position = FtcMotorControllers.class.getMethod(
                "positionControl", PositionPlant.class);
        assertEquals(FtcMotorPositionControl.class, position.getReturnType());
        for (Method method : FtcMotorControllers.class.getDeclaredMethods()) {
            assertFalse("Legacy velocityPidf factory must remain absent",
                    "velocityPidf".equals(method.getName()));
        }
        assertEquals(0, FtcMotorVelocityControl.class.getConstructors().length);
        assertEquals(0, FtcMotorPositionControl.class.getConstructors().length);

        Method recovery = FtcMotorPositionControl.class.getMethod(
                "prepareRecoveryHoldWithin", ScalarRange.class, LoopClock.class);
        assertEquals(PositionPlantTuning.RecoveryHold.class, recovery.getReturnType());
    }

    @Test
    public void velocityConstructionCandidateExistsOnlyForAnExplicitSharedOverride() {
        TestHardwareMap plainMap = new TestHardwareMap();
        plainMap.addMotor("plain", pidf(1, 2, 3, 4, MotorControlAlgorithm.PIDF));
        FtcMotorVelocityControl plain = FtcMotorControllers.velocityControl(
                singleVelocity(plainMap, "plain"));
        assertFalse(plain.constructionCandidate().isPresent());

        TestHardwareMap configuredMap = new TestHardwareMap();
        configuredMap.addMotor("left", pidf(1, 0, 0, 0, MotorControlAlgorithm.LegacyPID));
        configuredMap.addMotor("right", pidf(2, 0, 0, 0, MotorControlAlgorithm.LegacyPID));
        Plant configuredPlant = FtcActuators.plant(configuredMap)
                .motor("left", Direction.FORWARD)
                .andMotor("right", Direction.REVERSE)
                .velocity()
                .deviceManagedWithOverrides()
                .velocityPidf(9, 8, 7, 6)
                .bounded(-100, 100)
                .nativeUnits()
                .velocityTolerance(1)
                .targetFromNewCommand(0)
                .build();
        FtcMotorVelocityControl configured =
                FtcMotorControllers.velocityControl(configuredPlant);

        assertTrue(configured.constructionCandidate().isPresent());
        assertEquals(FtcMotorVelocityControl.Candidate.of(9, 8, 7, 6),
                configured.constructionCandidate().get());
    }

    @Test
    public void recognizedVelocityCaptureFailureIsStateFailureAndConsumesClaim() {
        TestHardwareMap map = new TestHardwareMap();
        MotorProbe motor = map.addMotor(
                "flywheel", pidf(1, 0, 0, 0, MotorControlAlgorithm.PIDF));
        Plant plant = singleVelocity(map, "flywheel");
        motor.pidfReadFailure = new IllegalStateException("controller offline");

        IllegalStateException captureFailure = assertThrows(
                IllegalStateException.class,
                () -> FtcMotorControllers.velocityControl(plant));
        assertContains(captureFailure, "could not capture");

        motor.pidfReadFailure = null;
        IllegalStateException consumed = assertThrows(
                IllegalStateException.class,
                () -> FtcMotorControllers.velocityControl(plant));
        assertContains(consumed, "already has");
    }

    @Test
    public void velocityControllerMustBeClaimedBeforeFirstHeartbeatOrStop() {
        TestHardwareMap updatedMap = new TestHardwareMap();
        MotorProbe updatedMotor = updatedMap.addMotor(
                "updated", pidf(1, 0, 0, 0, MotorControlAlgorithm.PIDF));
        Plant updated = singleVelocity(updatedMap, "updated");
        LoopClock clock = new LoopClock();
        clock.reset(0.0);
        updated.update(clock);

        IllegalStateException late = assertThrows(
                IllegalStateException.class,
                () -> FtcMotorControllers.velocityControl(updated));
        assertContains(late, "before the Plant's first update(clock)");
        assertEquals(0, updatedMotor.pidfReads);

        TestHardwareMap stoppedMap = new TestHardwareMap();
        MotorProbe stoppedMotor = stoppedMap.addMotor(
                "stopped", pidf(1, 0, 0, 0, MotorControlAlgorithm.PIDF));
        Plant stopped = singleVelocity(stoppedMap, "stopped");
        stopped.stop();

        IllegalStateException stoppedClaim = assertThrows(
                IllegalStateException.class,
                () -> FtcMotorControllers.velocityControl(stopped));
        assertContains(stoppedClaim, "before the Plant is stopped");
        assertEquals(0, stoppedMotor.pidfReads);
    }

    @Test
    public void stoppedVelocityPlantNeverPublishesEvidenceOrAcceptsApplyButCanRestore() {
        TestHardwareMap map = new TestHardwareMap();
        MotorProbe left = map.addMotor(
                "left", pidf(1, 0, 0, 0, MotorControlAlgorithm.PIDF));
        MotorProbe right = map.addMotor(
                "right", pidf(1, 0, 0, 0, MotorControlAlgorithm.PIDF));
        Plant plant = velocityGroup(map, 1.0, 1.0);
        FtcMotorVelocityControl handle = FtcMotorControllers.velocityControl(plant);
        LoopClock clock = new LoopClock();
        clock.reset(0.0);
        plant.stop();
        plant.update(clock);

        assertThrows(IllegalStateException.class, () -> handle.evidence(clock));
        assertThrows(IllegalStateException.class,
                () -> handle.apply(FtcMotorVelocityControl.Candidate.of(2, 3, 4, 5), clock));
        assertEquals(0, left.velocityPidfWrites);
        assertEquals(0, right.velocityPidfWrites);

        handle.restoreInitial();
        assertEquals(1, left.generalSetWrites);
        assertEquals(1, right.generalSetWrites);
    }

    @Test
    public void velocityEvidenceReusesPlantMemoizedSourcesAndMapsEveryMemberTolerance() {
        TestHardwareMap map = new TestHardwareMap();
        MotorProbe left = map.addMotor("left", pidf(1, 0, 0, 0, MotorControlAlgorithm.PIDF));
        MotorProbe right = map.addMotor("right", pidf(1, 0, 0, 0, MotorControlAlgorithm.PIDF));
        left.velocityMeasurement = 5.0;
        right.velocityMeasurement = -5.0;
        Plant plant = velocityGroup(map, 2.0, 3.0);
        FtcMotorVelocityControl handle = FtcMotorControllers.velocityControl(plant);
        LoopClock clock = new LoopClock();
        clock.reset(0.0);

        IllegalStateException early = assertThrows(
                IllegalStateException.class,
                () -> handle.evidence(clock));
        assertContains(early, "update(clock)");
        assertEquals(0, left.velocityReads);
        assertEquals(0, right.velocityReads);

        plant.update(clock);
        List<FtcMotorVelocityControl.MemberEvidence> evidence = handle.evidence(clock);
        List<FtcMotorVelocityControl.MemberEvidence> repeated = handle.evidence(clock);

        assertEquals(1, left.velocityReads);
        assertEquals(1, right.velocityReads);
        assertEquals(2, evidence.size());
        assertEquals(2, repeated.size());
        assertRawEquals(0.0, evidence.get(0).nativeCommandedTarget());
        assertRawEquals(-0.0, evidence.get(1).nativeCommandedTarget());
        assertEquals(5.0, evidence.get(0).nativeMeasurement(), 0.0);
        assertEquals(-5.0, evidence.get(1).nativeMeasurement(), 0.0);
        assertEquals(-5.0, evidence.get(0).nativeError(), 0.0);
        assertEquals(5.0, evidence.get(1).nativeError(), 0.0);
        assertEquals(6.0, evidence.get(0).nativeTolerance(), 0.0);
        assertEquals(6.0, evidence.get(1).nativeTolerance(), 0.0);
        assertTrue(evidence.get(0).withinMappedPlantTolerance());
        assertTrue(evidence.get(1).withinMappedPlantTolerance());
    }

    @Test
    public void groupedVelocityApplyRequiresEveryChildColdBeforeAnySetter() {
        TestHardwareMap map = new TestHardwareMap();
        MotorProbe left = map.addMotor("left", pidf(1, 0, 0, 0, MotorControlAlgorithm.PIDF));
        MotorProbe right = map.addMotor("right", pidf(1, 0, 0, 0, MotorControlAlgorithm.PIDF));
        right.velocityMeasurement = -7.0;
        Plant plant = velocityGroup(map, 2.0, 3.0);
        FtcMotorVelocityControl handle = FtcMotorControllers.velocityControl(plant);
        LoopClock clock = new LoopClock();
        clock.reset(0.0);
        plant.update(clock);

        IllegalStateException notCold = assertThrows(
                IllegalStateException.class,
                () -> handle.apply(FtcMotorVelocityControl.Candidate.of(9, 8, 7, 6), clock));

        assertContains(notCold, "right");
        assertEquals(0, left.velocityPidfWrites);
        assertEquals(0, right.velocityPidfWrites);
        assertFalse(handle.isTerminallyUncertain());

        right.velocityMeasurement = -5.0;
        clock.update(0.02);
        plant.update(clock);
        handle.apply(FtcMotorVelocityControl.Candidate.of(9, 8, 7, 6), clock);
        assertEquals(1, left.velocityPidfWrites);
        assertEquals(1, right.velocityPidfWrites);
        assertEquals(9.0,
                handle.readbackConfigurations().get(0).pidf().getKP(), 0.0);

        plant.commandTarget().set(10.0);
        clock.update(0.04);
        plant.update(clock);
        IllegalStateException targetNotZero = assertThrows(
                IllegalStateException.class,
                () -> handle.apply(FtcMotorVelocityControl.Candidate.of(2, 0, 0, 0), clock));
        assertContains(targetNotZero, "applied target");
        assertEquals(1, left.velocityPidfWrites);
        assertEquals(1, right.velocityPidfWrites);
    }

    @Test
    public void singleVelocityApplyCanBeHotAndRestorePreservesExactAlgorithm() {
        TestHardwareMap map = new TestHardwareMap();
        PIDFCoefficients initial = pidf(-0.0, 2, 3, 4, MotorControlAlgorithm.LegacyPID);
        MotorProbe motor = map.addMotor("flywheel", initial);
        Plant plant = singleVelocity(map, "flywheel");
        FtcMotorVelocityControl handle = FtcMotorControllers.velocityControl(plant);
        LoopClock clock = new LoopClock();
        clock.reset(0.0);
        plant.commandTarget().set(1000.0);
        plant.update(clock);

        handle.apply(FtcMotorVelocityControl.Candidate.of(5, 6, 7, 8), clock);
        assertEquals(5.0, handle.readbackConfigurations().get(0).pidf().getKP(), 0.0);

        handle.restoreInitial();
        assertPidf(initial, motor.velocity());
        assertEquals(MotorControlAlgorithm.LegacyPID,
                handle.readbackConfigurations().get(0).pidf().algorithm());
        assertFalse(handle.isTerminallyUncertain());
    }

    @Test
    public void groupedVelocityFailureLatchesTerminalAndRestoreAttemptsEveryMember() {
        TestHardwareMap map = new TestHardwareMap();
        PIDFCoefficients leftInitial = pidf(1, 2, 3, 4, MotorControlAlgorithm.LegacyPID);
        PIDFCoefficients rightInitial = pidf(5, 6, 7, 8, MotorControlAlgorithm.PIDF);
        MotorProbe left = map.addMotor("left", leftInitial);
        MotorProbe right = map.addMotor("right", rightInitial);
        Plant plant = velocityGroup(map, 1.0, 0.0);
        FtcMotorVelocityControl handle = FtcMotorControllers.velocityControl(plant);
        LoopClock clock = new LoopClock();
        clock.reset(0.0);
        plant.update(clock);
        right.velocitySetFailure = new IllegalStateException("transport");

        IllegalStateException failure = assertThrows(
                IllegalStateException.class,
                () -> handle.apply(FtcMotorVelocityControl.Candidate.of(9, 9, 9, 9), clock));
        assertContains(failure, "partially changed");
        assertTrue(handle.isTerminallyUncertain());
        assertEquals(1, left.velocityPidfWrites);
        assertEquals(0, right.velocityPidfWrites);
        assertThrows(IllegalStateException.class,
                () -> handle.apply(FtcMotorVelocityControl.Candidate.of(1, 1, 1, 1), clock));

        handle.restoreInitial();
        assertPidf(leftInitial, left.velocity());
        assertPidf(rightInitial, right.velocity());
        assertEquals(1, left.generalSetWrites);
        assertEquals(1, right.generalSetWrites);
        assertTrue(handle.isTerminallyUncertain());
    }

    @Test
    public void positionHandleOwnsOnlyTheSingleMotorCascadeAndPreparesHoldWithoutOutput() {
        TestHardwareMap map = new TestHardwareMap();
        PIDFCoefficients outerInitial = pidf(-0.0, 11, 12, 13, MotorControlAlgorithm.LegacyPID);
        PIDFCoefficients innerInitial = pidf(2, 3, 4, 5, MotorControlAlgorithm.PIDF);
        MotorProbe motor = map.addMotor("arm", innerInitial);
        motor.configurations.put(DcMotor.RunMode.RUN_TO_POSITION,
                new PIDFCoefficients(outerInitial));
        motor.positionMeasurement = 42;
        PositionPlant plant = singlePosition(map, "arm");
        FtcMotorPositionControl handle = FtcMotorControllers.positionControl(plant);
        LoopClock clock = new LoopClock();
        clock.reset(0.0);

        assertTrue(handle.hasExactCommandTarget());
        assertTrue(handle.isReferenced());
        assertEquals("arm", handle.motorName());
        assertRawEquals(42.0, handle.prepareHoldAtCurrent(clock));
        assertRawEquals(42.0, plant.commandTarget().getAsDouble(clock));
        assertEquals(0, motor.targetWrites);
        assertEquals(0, motor.powerWrites);
        assertEquals(0, motor.modeWrites);

        handle.apply(FtcMotorPositionControl.Candidate.of(9, 8, 7, 6, 5));
        assertEquals(9.0,
                handle.readbackConfiguration().outerPosition().getKP(), 0.0);
        assertEquals(8.0,
                handle.readbackConfiguration().innerVelocity().getKP(), 0.0);

        handle.restoreInitial();
        assertPidf(outerInitial, motor.position());
        assertPidf(innerInitial, motor.velocity());
        assertFalse(handle.isTerminallyUncertain());
    }

    @Test
    public void positionRecoveryHoldUsesSameCycleMeasurementAndWritesNoControllerOrOutput() {
        TestHardwareMap map = new TestHardwareMap();
        MotorProbe motor = map.addMotor(
                "arm", pidf(1, 0, 0, 0, MotorControlAlgorithm.PIDF));
        motor.positionMeasurement = 25;
        PositionPlant plant = singlePosition(map, "arm");
        FtcMotorPositionControl handle = FtcMotorControllers.positionControl(plant);
        LoopClock clock = new LoopClock();
        clock.reset(0.0);
        handle.prepareHoldAtCurrent(clock);

        motor.positionMeasurement = 80;
        clock.update(0.02);
        PositionPlantTuning.RecoveryHold recovery = handle.prepareRecoveryHoldWithin(
                ScalarRange.bounded(-50, 50), clock);

        assertEquals(80.0, recovery.measurement(), 0.0);
        assertEquals(50.0, recovery.holdTarget(), 0.0);
        assertTrue(recovery.wasClamped());
        assertEquals(50.0, plant.commandTarget().getAsDouble(clock), 0.0);
        assertEquals(2, motor.positionReads);
        assertEquals(0, motor.targetWrites);
        assertEquals(0, motor.powerWrites);
        assertEquals(0, motor.modeWrites);
        assertEquals(0, motor.velocityPidfAttempts);
        assertEquals(0, motor.generalSetWrites);
    }

    @Test
    public void positionOutputMeasurementSearchAndControllerReuseOneResolvedMotorIdentity() {
        TestHardwareMap map = new TestHardwareMap();
        MotorProbe motor = map.addMotor(
                "arm", pidf(1, 0, 0, 0, MotorControlAlgorithm.PIDF));
        motor.positionMeasurement = 33;

        PositionPlant plant = singlePosition(map, "arm");
        assertEquals(1, map.lookupCount);
        FtcMotorPositionControl handle = FtcMotorControllers.positionControl(plant);
        LoopClock clock = new LoopClock();
        clock.reset(0.0);

        assertEquals(33.0, handle.prepareHoldAtCurrent(clock), 0.0);
        assertEquals(1, map.lookupCount);
        assertEquals(1, motor.positionReads);
    }

    @Test
    public void stoppedPositionPlantRejectsApplyButLeavesExactRestoreAvailable() {
        TestHardwareMap map = new TestHardwareMap();
        MotorProbe motor = map.addMotor(
                "arm", pidf(1, 2, 3, 4, MotorControlAlgorithm.LegacyPID));
        PositionPlant plant = singlePosition(map, "arm");
        FtcMotorPositionControl handle = FtcMotorControllers.positionControl(plant);
        plant.stop();

        assertThrows(IllegalStateException.class,
                () -> handle.apply(FtcMotorPositionControl.Candidate.of(5, 6, 7, 8, 9)));
        assertEquals(0, motor.velocityPidfAttempts);

        handle.restoreInitial();
        assertEquals(2, motor.generalSetWrites);
    }

    @Test
    public void positionFactoryRejectsGroupedAndOtherShapesBeforeReadback() {
        TestHardwareMap map = new TestHardwareMap();
        MotorProbe left = map.addMotor("left", pidf(1, 0, 0, 0, MotorControlAlgorithm.PIDF));
        MotorProbe right = map.addMotor("right", pidf(1, 0, 0, 0, MotorControlAlgorithm.PIDF));
        PositionPlant grouped = FtcActuators.plant(map)
                .motor("left", Direction.FORWARD)
                .andMotor("right", Direction.REVERSE)
                .position()
                .deviceManaged()
                .nonPeriodic()
                .bounded(-100, 100)
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(1.0)
                .targetFromNewCommand(0.0)
                .build();
        left.pidfReads = 0;
        right.pidfReads = 0;

        IllegalArgumentException failure = assertThrows(
                IllegalArgumentException.class,
                () -> FtcMotorControllers.positionControl(grouped));
        assertContains(failure, "single-motor");
        assertEquals(0, left.pidfReads);
        assertEquals(0, right.pidfReads);
    }

    @Test
    public void periodicSingleMotorPositionPlantRetainsTheSameControllerCapability() {
        TestHardwareMap map = new TestHardwareMap();
        map.addMotor("tray", pidf(1, 0, 0, 0, MotorControlAlgorithm.PIDF));
        PositionPlant plant = FtcActuators.plant(map)
                .motor("tray", Direction.FORWARD)
                .position()
                .deviceManaged()
                .periodic(360.0)
                .bounded(0.0, 360.0)
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(1.0)
                .targetFromNewCommand(0.0)
                .build();

        FtcMotorPositionControl handle = FtcMotorControllers.positionControl(plant);

        assertEquals("tray", handle.motorName());
        assertEquals(0.0, handle.plantTargetRange().minValue, 0.0);
        assertEquals(360.0, handle.plantTargetRange().maxValue, 0.0);
    }

    @Test
    public void positionApplyFailureIsTerminalAndRestoreRecoversBothExactModes() {
        TestHardwareMap map = new TestHardwareMap();
        PIDFCoefficients outerInitial =
                pidf(3, 4, 5, 6, MotorControlAlgorithm.LegacyPID);
        PIDFCoefficients innerInitial =
                pidf(7, 8, 9, 10, MotorControlAlgorithm.LegacyPID);
        MotorProbe motor = map.addMotor("arm", innerInitial);
        motor.configurations.put(
                DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(outerInitial));
        FtcMotorPositionControl handle = FtcMotorControllers.positionControl(
                singlePosition(map, "arm"));
        motor.velocitySetFailure = new IllegalStateException("inner setter failed");

        IllegalStateException failure = assertThrows(
                IllegalStateException.class,
                () -> handle.apply(FtcMotorPositionControl.Candidate.of(1, 2, 3, 4, 5)));
        assertContains(failure, "partially changed");
        assertTrue(handle.isTerminallyUncertain());
        assertThrows(IllegalStateException.class,
                () -> handle.apply(FtcMotorPositionControl.Candidate.of(1, 2, 3, 4, 5)));

        handle.restoreInitial();
        assertPidf(outerInitial, motor.position());
        assertPidf(innerInitial, motor.velocity());
        assertTrue(handle.isTerminallyUncertain());
    }

    @Test
    public void duplicateResolvedMotorIdentityIsRejectedBeforeDirectionOrControllerEffects() {
        TestHardwareMap map = new TestHardwareMap();
        MotorProbe shared = map.addMotor("left", pidf(1, 0, 0, 0, MotorControlAlgorithm.PIDF));
        map.alias("right", shared);

        IllegalStateException failure = assertThrows(
                IllegalStateException.class,
                () -> velocityGroup(map, 1.0, 1.0));

        assertContains(failure, "same DcMotorEx object");
        assertEquals(0, shared.directionWrites);
        assertEquals(0, shared.pidfReads);
        assertEquals(0, shared.velocityPidfAttempts);
    }

    @Test
    public void groupedConstructionOverrideFailureRestoresEveryCapturedVelocityConfiguration() {
        TestHardwareMap map = new TestHardwareMap();
        PIDFCoefficients leftInitial = pidf(1, 2, 3, 4, MotorControlAlgorithm.LegacyPID);
        PIDFCoefficients rightInitial = pidf(5, 6, 7, 8, MotorControlAlgorithm.PIDF);
        MotorProbe left = map.addMotor("left", leftInitial);
        MotorProbe right = map.addMotor("right", rightInitial);
        right.velocitySetFailure = new IllegalStateException("write failed");

        IllegalStateException failure = assertThrows(
                IllegalStateException.class,
                () -> FtcActuators.plant(map)
                        .motor("left", Direction.FORWARD)
                        .andMotor("right", Direction.REVERSE)
                        .velocity()
                        .deviceManagedWithOverrides()
                        .velocityPidf(9, 9, 9, 9)
                        .bounded(-100, 100)
                        .nativeUnits()
                        .velocityTolerance(1)
                        .targetFromNewCommand(0)
                        .build());

        assertContains(failure, "every captured controller restoration was attempted");
        assertPidf(leftInitial, left.velocity());
        assertPidf(rightInitial, right.velocity());
        assertEquals(1, left.generalSetWrites);
        assertEquals(1, right.generalSetWrites);
        assertEquals(0, left.directionWrites);
        assertEquals(0, right.directionWrites);
    }

    @Test
    public void groupedConstructionOverrideFailureRestoresEveryPositionSetting() {
        TestHardwareMap map = new TestHardwareMap();
        PIDFCoefficients leftOuter = pidf(1, 2, 3, 4, MotorControlAlgorithm.LegacyPID);
        PIDFCoefficients leftInner = pidf(5, 6, 7, 8, MotorControlAlgorithm.PIDF);
        PIDFCoefficients rightOuter = pidf(9, 10, 11, 12, MotorControlAlgorithm.PIDF);
        PIDFCoefficients rightInner = pidf(13, 14, 15, 16, MotorControlAlgorithm.LegacyPID);
        MotorProbe left = map.addMotor("left", leftInner);
        MotorProbe right = map.addMotor("right", rightInner);
        left.configurations.put(DcMotor.RunMode.RUN_TO_POSITION,
                new PIDFCoefficients(leftOuter));
        right.configurations.put(DcMotor.RunMode.RUN_TO_POSITION,
                new PIDFCoefficients(rightOuter));
        left.targetTolerance = 17;
        right.targetTolerance = 18;
        right.velocitySetFailure = new IllegalStateException("inner write failed");

        IllegalStateException failure = assertThrows(
                IllegalStateException.class,
                () -> FtcActuators.plant(map)
                        .motor("left", Direction.FORWARD)
                        .andMotor("right", Direction.REVERSE)
                        .position()
                        .deviceManagedWithOverrides()
                        .outerPositionP(20)
                        .innerVelocityPidf(21, 22, 23, 24)
                        .devicePositionToleranceTicks(25)
                        .doneOverrides()
                        .nonPeriodic()
                        .bounded(-100, 100)
                        .nativeUnits()
                        .alreadyReferenced()
                        .positionTolerance(1)
                        .targetFromNewCommand(0)
                        .build());

        assertContains(failure, "every captured controller restoration was attempted");
        assertPidf(leftOuter, left.position());
        assertPidf(leftInner, left.velocity());
        assertEquals(17, left.targetTolerance);
        assertPidf(rightOuter, right.position());
        assertPidf(rightInner, right.velocity());
        assertEquals(18, right.targetTolerance);
        assertEquals(2, left.generalSetWrites);
        assertEquals(2, right.generalSetWrites);
        assertEquals(0, left.directionWrites);
        assertEquals(0, right.directionWrites);
    }

    @Test
    public void laterPositionPlantConstructionFailureStillRestoresEveryCapturedSetting() {
        TestHardwareMap map = new TestHardwareMap();
        PIDFCoefficients leftOuter = pidf(1, 2, 3, 4, MotorControlAlgorithm.LegacyPID);
        PIDFCoefficients leftInner = pidf(5, 6, 7, 8, MotorControlAlgorithm.PIDF);
        PIDFCoefficients rightOuter = pidf(9, 10, 11, 12, MotorControlAlgorithm.PIDF);
        PIDFCoefficients rightInner = pidf(13, 14, 15, 16, MotorControlAlgorithm.LegacyPID);
        MotorProbe left = map.addMotor("left", leftInner);
        MotorProbe right = map.addMotor("right", rightInner);
        left.configurations.put(DcMotor.RunMode.RUN_TO_POSITION,
                new PIDFCoefficients(leftOuter));
        right.configurations.put(DcMotor.RunMode.RUN_TO_POSITION,
                new PIDFCoefficients(rightOuter));
        left.targetTolerance = 17;
        right.targetTolerance = 18;
        // Position output configuration succeeds first; the later search-power adapter then fails.
        right.directionFailureAttempt = 2;
        right.directionSetFailure =
                new IllegalStateException("late power-output direction failed");

        IllegalStateException failure = assertThrows(
                IllegalStateException.class,
                () -> FtcActuators.plant(map)
                        .motor("left", Direction.FORWARD)
                        .andMotor("right", Direction.REVERSE)
                        .position()
                        .deviceManagedWithOverrides()
                        .outerPositionP(20)
                        .innerVelocityPidf(21, 22, 23, 24)
                        .devicePositionToleranceTicks(25)
                        .doneOverrides()
                        .nonPeriodic()
                        .bounded(-100, 100)
                        .nativeUnits()
                        .alreadyReferenced()
                        .positionTolerance(1)
                        .targetFromNewCommand(0)
                        .build());

        assertContains(failure, "late power-output direction failed");
        assertPidf(leftOuter, left.position());
        assertPidf(leftInner, left.velocity());
        assertEquals(17, left.targetTolerance);
        assertPidf(rightOuter, right.position());
        assertPidf(rightInner, right.velocity());
        assertEquals(18, right.targetTolerance);
        assertEquals(2, left.generalSetWrites);
        assertEquals(2, right.generalSetWrites);
    }

    private static Plant velocityGroup(TestHardwareMap map,
                                       double nativePerPlantUnit,
                                       double plantTolerance) {
        return FtcActuators.plant(map)
                .motor("left", Direction.FORWARD)
                .andMotor("right", Direction.REVERSE)
                .scale(-1.0)
                .velocity()
                .deviceManaged()
                .bounded(-100.0, 100.0)
                .scaleToNative(nativePerPlantUnit)
                .velocityTolerance(plantTolerance)
                .targetFromNewCommand(0.0)
                .build();
    }

    private static Plant singleVelocity(TestHardwareMap map, String name) {
        return FtcActuators.plant(map)
                .motor(name, Direction.FORWARD)
                .velocity()
                .deviceManaged()
                .bounded(-2000.0, 2000.0)
                .nativeUnits()
                .velocityTolerance(10.0)
                .targetFromNewCommand(0.0)
                .build();
    }

    private static PositionPlant singlePosition(TestHardwareMap map, String name) {
        return FtcActuators.plant(map)
                .motor(name, Direction.FORWARD)
                .position()
                .deviceManaged()
                .nonPeriodic()
                .bounded(-100.0, 100.0)
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(1.0)
                .targetFromNewCommand(0.0)
                .build();
    }

    private static PIDFCoefficients pidf(double p,
                                         double i,
                                         double d,
                                         double f,
                                         MotorControlAlgorithm algorithm) {
        return new PIDFCoefficients(p, i, d, f, algorithm);
    }

    private static void assertConfiguration(
            FtcMotorVelocityControl.MemberConfiguration actual,
            String name,
            PIDFCoefficients expected) {
        assertEquals(name, actual.motorName());
        assertPidf(expected, actual.pidf());
    }

    private static void assertPidf(PIDFCoefficients expected,
                                   FtcMotorPidfConfiguration actual) {
        assertNotNull(actual);
        assertRawEquals(expected.p, actual.getKP());
        assertRawEquals(expected.i, actual.getKI());
        assertRawEquals(expected.d, actual.getKD());
        assertRawEquals(expected.f, actual.getKF());
        assertEquals(expected.algorithm, actual.algorithm());
    }

    private static void assertPidf(PIDFCoefficients expected, PIDFCoefficients actual) {
        assertNotNull(actual);
        assertRawEquals(expected.p, actual.p);
        assertRawEquals(expected.i, actual.i);
        assertRawEquals(expected.d, actual.d);
        assertRawEquals(expected.f, actual.f);
        assertEquals(expected.algorithm, actual.algorithm);
    }

    private static void assertRawEquals(double expected, double actual) {
        assertEquals(Double.doubleToRawLongBits(expected), Double.doubleToRawLongBits(actual));
    }

    private static void assertContains(Throwable failure, String expected) {
        assertNotNull(failure.getMessage());
        assertTrue("Expected <" + failure.getMessage() + "> to contain <" + expected + ">",
                failure.getMessage().contains(expected));
    }

    private static <T extends Throwable> T assertThrows(Class<T> type, Runnable action) {
        try {
            action.run();
            fail("Expected " + type.getSimpleName());
            return null;
        } catch (Throwable failure) {
            if (!type.isInstance(failure)) {
                throw new AssertionError(
                        "Expected " + type.getSimpleName() + " but got " + failure, failure);
            }
            return type.cast(failure);
        }
    }

    private static final class TestHardwareMap extends HardwareMap {
        private final Map<String, MotorProbe> motors = new HashMap<>();
        private int lookupCount;

        private TestHardwareMap() { super(null, null); }

        private MotorProbe addMotor(String name, PIDFCoefficients velocity) {
            MotorProbe probe = new MotorProbe(name, velocity);
            motors.put(name, probe);
            return probe;
        }

        private void alias(String name, MotorProbe probe) { motors.put(name, probe); }

        @Override
        public <T> T get(Class<? extends T> type, String name) {
            lookupCount++;
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
        private final EnumMap<DcMotor.RunMode, PIDFCoefficients> configurations =
                new EnumMap<>(DcMotor.RunMode.class);
        private double velocityMeasurement;
        private int positionMeasurement;
        private int targetTolerance = 5;
        private int pidfReads;
        private int velocityReads;
        private int positionReads;
        private int velocityPidfAttempts;
        private int velocityPidfWrites;
        private int generalSetWrites;
        private int directionWrites;
        private int modeWrites;
        private int powerWrites;
        private int targetWrites;
        private RuntimeException velocitySetFailure;
        private RuntimeException pidfReadFailure;
        private int directionFailureAttempt = -1;
        private RuntimeException directionSetFailure;

        private MotorProbe(String name, PIDFCoefficients velocity) {
            this.name = name;
            configurations.put(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(velocity));
            configurations.put(DcMotor.RunMode.RUN_TO_POSITION,
                    pidf(0, 0, 0, 0, MotorControlAlgorithm.PIDF));
            motor = (DcMotorEx) Proxy.newProxyInstance(
                    DcMotorEx.class.getClassLoader(),
                    new Class<?>[]{DcMotorEx.class},
                    this::invoke);
        }

        private PIDFCoefficients velocity() {
            return new PIDFCoefficients(configurations.get(DcMotor.RunMode.RUN_USING_ENCODER));
        }

        private PIDFCoefficients position() {
            return new PIDFCoefficients(configurations.get(DcMotor.RunMode.RUN_TO_POSITION));
        }

        private Object invoke(Object proxy, Method method, Object[] args) {
            String operation = method.getName();
            if (method.getDeclaringClass() == Object.class) {
                if ("equals".equals(operation)) return proxy == args[0];
                if ("hashCode".equals(operation)) return System.identityHashCode(proxy);
                if ("toString".equals(operation)) return "MotorProbe(" + name + ')';
            }
            if ("getPIDFCoefficients".equals(operation)) {
                if (pidfReadFailure != null) throw pidfReadFailure;
                pidfReads++;
                return new PIDFCoefficients(configurations.get((DcMotor.RunMode) args[0]));
            }
            if ("setPIDFCoefficients".equals(operation)) {
                generalSetWrites++;
                configurations.put(
                        (DcMotor.RunMode) args[0],
                        new PIDFCoefficients((PIDFCoefficients) args[1]));
                return null;
            }
            if ("setVelocityPIDFCoefficients".equals(operation)) {
                velocityPidfAttempts++;
                if (velocitySetFailure != null) throw velocitySetFailure;
                velocityPidfWrites++;
                configurations.put(
                        DcMotor.RunMode.RUN_USING_ENCODER,
                        pidf((double) args[0], (double) args[1], (double) args[2],
                                (double) args[3], MotorControlAlgorithm.PIDF));
                return null;
            }
            if ("setPositionPIDFCoefficients".equals(operation)) {
                configurations.put(
                        DcMotor.RunMode.RUN_TO_POSITION,
                        pidf((double) args[0], 0, 0, 0, MotorControlAlgorithm.PIDF));
                return null;
            }
            if ("getTargetPositionTolerance".equals(operation)) return targetTolerance;
            if ("setTargetPositionTolerance".equals(operation)) {
                targetTolerance = (int) args[0];
                return null;
            }
            if ("getVelocity".equals(operation)) {
                velocityReads++;
                return velocityMeasurement;
            }
            if ("getCurrentPosition".equals(operation)) {
                positionReads++;
                return positionMeasurement;
            }
            if ("setDirection".equals(operation)) {
                directionWrites++;
                if (directionWrites == directionFailureAttempt) throw directionSetFailure;
                return null;
            }
            if ("getDirection".equals(operation)) return DcMotorSimple.Direction.FORWARD;
            if ("setMode".equals(operation)) { modeWrites++; return null; }
            if ("getMode".equals(operation)) return DcMotor.RunMode.RUN_USING_ENCODER;
            if ("setPower".equals(operation)) { powerWrites++; return null; }
            if ("getPower".equals(operation)) return 0.0;
            if ("setTargetPosition".equals(operation)) { targetWrites++; return null; }
            if ("getTargetPosition".equals(operation)) return 0;
            if ("setVelocity".equals(operation)) return null;
            if ("getZeroPowerBehavior".equals(operation)) return DcMotor.ZeroPowerBehavior.FLOAT;
            if ("isBusy".equals(operation)) return false;
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
