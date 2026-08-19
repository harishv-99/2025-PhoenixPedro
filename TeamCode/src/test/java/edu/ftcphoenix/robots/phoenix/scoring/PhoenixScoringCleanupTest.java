package edu.ftcphoenix.robots.phoenix.scoring;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.localization.AbsolutePoseEstimator;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcphoenix.robots.phoenix.PhoenixProfile;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Policy, private-Plant construction, and cleanup coverage for PhoenixScoring. */
public final class PhoenixScoringCleanupTest {

    @Test
    public void configSurfaceIsDefaultsOnlyWithAllThirtyFiveFields() {
        assertEquals(35, PhoenixScoring.Config.class.getFields().length);
        Constructor<?>[] constructors = PhoenixScoring.Config.class.getDeclaredConstructors();
        assertEquals(1, constructors.length);
        assertTrue(Modifier.isPrivate(constructors[0].getModifiers()));

        Method[] methods = PhoenixScoring.Config.class.getDeclaredMethods();
        int publicMethodCount = 0;
        for (Method method : methods) {
            if (Modifier.isPublic(method.getModifiers())) {
                publicMethodCount++;
                assertEquals("defaults", method.getName());
                assertTrue(Modifier.isStatic(method.getModifiers()));
            }
        }
        assertEquals(1, publicMethodCount);
        for (Field field : PhoenixScoring.Config.class.getFields()) {
            assertTrue(Modifier.isPublic(field.getModifiers()));
        }

        PhoenixScoring.Config config = PhoenixScoring.Config.defaults();
        assertEquals("intakeMotor", config.nameMotorIntake);
        assertEquals(Direction.FORWARD, config.directionMotorIntake);
        assertEquals("intakeTransfer", config.nameCrServoIntakeTransfer);
        assertEquals(Direction.REVERSE, config.directionCrServoIntakeTransfer);
        assertEquals("shooterTransferLeft", config.nameCrServoShooterTransferLeft);
        assertEquals(Direction.REVERSE, config.directionCrServoShooterTransferLeft);
        assertEquals("shooterTransferRight", config.nameCrServoShooterTransferRight);
        assertEquals(Direction.FORWARD, config.directionCrServoShooterTransferRight);
        assertEquals(0.65, config.shooterTransferLeftScale, 0.0);
        assertEquals("shooterMotor", config.nameMotorShooterWheel);
        assertEquals(Direction.FORWARD, config.directionMotorShooterWheel);
        assertEquals(700.0, config.velocityMin, 0.0);
        assertEquals(2000.0, config.velocityMax, 0.0);
        assertEquals(50.0, config.velocityToleranceNative, 0.0);
        assertFalse(config.applyFlywheelVelocityPIDF);
        assertEquals(0.0, config.flywheelVelKp, 0.0);
        assertEquals(0.0, config.flywheelVelKi, 0.0);
        assertEquals(0.0, config.flywheelVelKd, 0.0);
        assertEquals(0.0, config.flywheelVelKf, 0.0);
        assertEquals(50.0, config.velocityToleranceBelowNative, 0.0);
        assertEquals(50.0, config.velocityToleranceAboveNative, 0.0);
        assertEquals(0.10, config.readyPredictLeadSec, 0.0);
        assertEquals(0.03, config.readyStableSec, 0.0);
        assertEquals(1.0, config.intakeMotorPower, 0.0);
        assertEquals(1.0, config.intakeTransferPower, 0.0);
        assertEquals(0.5, config.intakeShooterTransferHoldBackPower, 0.0);
        assertEquals(1.0, config.ejectMotorPower, 0.0);
        assertEquals(1.0, config.ejectTransferPower, 0.0);
        assertEquals(1.0, config.ejectShooterTransferPower, 0.0);
        assertEquals(1.0, config.shootFeedPower, 0.0);
        assertEquals(0.22, config.shootFeedPulseSec, 0.0);
        assertEquals(0.06, config.shootFeedCooldownSec, 0.0);
        assertEquals(1.0, config.feedScaleIntakeMotor, 0.0);
        assertEquals(1.0, config.feedScaleIntakeTransfer, 0.0);
        assertEquals(1.0, config.feedScaleShooterTransfer, 0.0);
    }

    @Test
    public void queuedShotsRunAsDistinctPulsesAndDrainInOrder() {
        PolicyFixture fixture = new PolicyFixture();
        fixture.scoring.setFlywheelEnabled(true);
        fixture.scoring.requestShots(2);

        fixture.tick(0.0);
        assertFeedState(fixture, "SHOOT", 2, 1, true, fixture.config.shootFeedPower);

        fixture.tick(0.20);
        assertFeedState(fixture, "SHOOT", 2, 1, true, 0.0);

        fixture.tick(0.30);
        assertFeedState(fixture, "SHOOT", 1, 1, false, 0.0);

        fixture.tick(0.31);
        assertFeedState(fixture, "SHOOT", 1, 0, true, fixture.config.shootFeedPower);

        fixture.tick(0.51);
        fixture.tick(0.61);
        fixture.tick(0.62);
        assertFeedState(fixture, "IDLE", 0, 0, false, 0.0);
        assertFalse(fixture.scoring.hasPendingShots());
    }

    @Test
    public void continuousShootingKeepsOnePulseBufferedUntilRequestFalls() {
        PolicyFixture fixture = new PolicyFixture();
        fixture.scoring.setFlywheelEnabled(true);
        fixture.scoring.setShootingEnabled(true);

        fixture.tick(0.0);
        assertFeedState(fixture, "SHOOT", 1, 0, true, fixture.config.shootFeedPower);

        fixture.tick(0.20);
        fixture.tick(0.30);
        assertEquals(0, fixture.scoring.status().feedBacklog);

        fixture.tick(0.31);
        assertFeedState(fixture, "SHOOT", 1, 0, true, fixture.config.shootFeedPower);

        fixture.scoring.setShootingEnabled(false);
        fixture.tick(0.32);
        assertFeedState(fixture, "IDLE", 0, 0, false, 0.0);
        assertFalse(fixture.scoring.status().shootingRequested);
        assertFalse(fixture.scoring.hasPendingShots());
    }

    @Test
    public void cancellationAndIntakeTransitionClearAllTransientShotWork() {
        PolicyFixture fixture = new PolicyFixture();
        fixture.scoring.setFlywheelEnabled(true);
        fixture.scoring.requestShots(3);

        fixture.tick(0.0);
        assertEquals(3, fixture.scoring.status().feedBacklog);
        assertEquals(fixture.config.shootFeedPower, fixture.intakeMotor().commandedPower, 0.0);

        fixture.scoring.cancelTransientActions();
        fixture.tick(0.01);
        assertFeedState(fixture, "IDLE", 0, 0, false, 0.0);

        fixture.scoring.requestSingleShot();
        fixture.tick(0.02);
        assertFeedState(fixture, "SHOOT", 1, 0, true, fixture.config.shootFeedPower);

        fixture.scoring.setIntakeEnabled(true);
        fixture.tick(0.03);
        assertFeedState(fixture, "INTAKE", 0, 0, false, 0.0);
        assertEquals(fixture.config.intakeMotorPower, fixture.intakeMotor().commandedPower, 0.0);
        assertEquals(
                fixture.config.intakeTransferPower,
                fixture.intakeTransfer().commandedPower,
                0.0);
        assertEquals(
                -fixture.config.intakeShooterTransferHoldBackPower,
                fixture.shooterTransferRight().commandedPower,
                0.0);
        assertFalse(fixture.scoring.hasPendingShots());
    }

    @Test
    public void ejectOverridesShootingAndFlywheelThenContinuousShootingResumes() {
        PolicyFixture fixture = new PolicyFixture();
        fixture.scoring.setIntakeEnabled(true);
        fixture.scoring.setFlywheelEnabled(true);
        fixture.scoring.setShootingEnabled(true);

        fixture.tick(0.0);
        assertFeedState(fixture, "SHOOT", 1, 0, true, fixture.config.shootFeedPower);
        assertTrue(fixture.scoring.status().flywheelEnabled);

        fixture.scoring.setEjectEnabled(true);
        fixture.tick(0.01);
        assertFeedState(fixture, "EJECT", 0, 0, false, 0.0);
        assertTrue(fixture.scoring.status().flywheelRequested);
        assertFalse(fixture.scoring.status().flywheelEnabled);
        assertEquals(-fixture.config.ejectMotorPower, fixture.intakeMotor().commandedPower, 0.0);
        assertEquals(
                -fixture.config.ejectTransferPower,
                fixture.intakeTransfer().commandedPower,
                0.0);
        assertEquals(
                -fixture.config.ejectShooterTransferPower,
                fixture.shooterTransferRight().commandedPower,
                0.0);

        fixture.scoring.setEjectEnabled(false);
        fixture.tick(0.02);
        assertFeedState(fixture, "SHOOT", 1, 0, true, fixture.config.shootFeedPower);
        assertTrue(fixture.scoring.status().flywheelEnabled);
    }

    @Test
    public void freshFlywheelReadinessStartsWaitingFeedInTheSameScoringUpdate() {
        PolicyFixture fixture = new PolicyFixture();
        fixture.flywheel().measuredVelocity = 0.0;
        fixture.scoring.setFlywheelEnabled(true);
        fixture.scoring.requestSingleShot();

        fixture.tick(0.0);
        assertFalse(fixture.scoring.status().ready);
        assertFeedState(fixture, "SHOOT", 1, 0, true, 0.0);

        fixture.flywheel().measuredVelocity = fixture.config.velocityMin;
        fixture.tick(0.02);
        assertTrue(fixture.scoring.status().ready);
        assertFeedState(fixture, "SHOOT", 1, 0, true, fixture.config.shootFeedPower);
        assertEquals(fixture.config.shootFeedPower, fixture.intakeMotor().commandedPower, 0.0);
        assertEquals(
                fixture.config.shootFeedPower,
                fixture.shooterTransferRight().commandedPower,
                0.0);
    }

    @Test
    public void productionAndExclusiveTuningFactoryShareTheFlywheelRecipe() {
        PhoenixProfile profile = PhoenixProfile.current();
        PhoenixScoring.Config config = PhoenixScoring.Config.defaults();
        config.applyFlywheelVelocityPIDF = true;
        config.flywheelVelKp = 1.25;
        config.flywheelVelKi = 2.5;
        config.flywheelVelKd = 3.75;
        config.flywheelVelKf = 4.5;
        config.velocityMax = 1200.0;

        TestHardwareMap productionHardware = TestHardwareMap.forScoring(config);
        PhoenixScoring production = new PhoenixScoring(
                productionHardware,
                config,
                targetingFor(profile));

        TestHardwareMap tuningHardware = TestHardwareMap.forFlywheel(config);
        Plant tuning = PhoenixScoring.createFlywheelPlantForTuning(tuningHardware, config);

        DeviceState productionFlywheel = productionHardware.state(config.nameMotorShooterWheel);
        DeviceState tuningFlywheel = tuningHardware.state(config.nameMotorShooterWheel);
        assertEquals(productionFlywheel.direction, tuningFlywheel.direction);
        assertEquals(1, productionFlywheel.velocityPidfWrites);
        assertEquals(1, tuningFlywheel.velocityPidfWrites);
        assertDoubleArrayEquals(productionFlywheel.velocityPidf, tuningFlywheel.velocityPidf);

        // Both owners snapshot the accepted recipe rather than retaining this mutable caller data.
        config.velocityMax = 1.0;
        config.flywheelVelKp = 99.0;

        LoopClock clock = new LoopClock();
        clock.reset(0.0);
        production.setSelectedVelocityNative(900.0);
        production.setFlywheelEnabled(true);
        production.update(clock);
        tuning.commandTarget().set(900.0);
        tuning.update(clock);

        assertEquals(900.0, productionFlywheel.commandedVelocity, 0.0);
        assertEquals(
                productionFlywheel.commandedVelocity,
                tuningFlywheel.commandedVelocity,
                0.0);
        assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, productionFlywheel.motorMode);
        assertEquals(productionFlywheel.motorMode, tuningFlywheel.motorMode);

        production.stop();
        tuning.stop();
    }

    @Test
    public void productionPreflightRejectsMalformedAndEverySameFamilyCollisionBeforeLookup() {
        PhoenixProfile profile = PhoenixProfile.current();

        PhoenixScoring.Config malformed = PhoenixScoring.Config.defaults();
        malformed.nameMotorIntake = "   ";
        malformed.directionMotorIntake = null;
        assertProductionPreflightFailure(
                profile,
                malformed,
                "nameMotorIntake"
        );

        PhoenixScoring.Config motorCollision = PhoenixScoring.Config.defaults();
        motorCollision.nameMotorShooterWheel = "  " + motorCollision.nameMotorIntake + "  ";
        assertProductionPreflightFailure(profile, motorCollision, "nameMotorShooterWheel");

        PhoenixScoring.Config firstServoCollision = PhoenixScoring.Config.defaults();
        firstServoCollision.nameCrServoShooterTransferLeft =
                " " + firstServoCollision.nameCrServoIntakeTransfer + " ";
        assertProductionPreflightFailure(
                profile,
                firstServoCollision,
                "nameCrServoShooterTransferLeft"
        );

        PhoenixScoring.Config secondServoCollision = PhoenixScoring.Config.defaults();
        secondServoCollision.nameCrServoShooterTransferRight =
                " " + secondServoCollision.nameCrServoIntakeTransfer + " ";
        assertProductionPreflightFailure(
                profile,
                secondServoCollision,
                "nameCrServoShooterTransferRight"
        );

        PhoenixScoring.Config thirdServoCollision = PhoenixScoring.Config.defaults();
        thirdServoCollision.nameCrServoShooterTransferRight =
                " " + thirdServoCollision.nameCrServoShooterTransferLeft + " ";
        assertProductionPreflightFailure(
                profile,
                thirdServoCollision,
                "nameCrServoShooterTransferRight"
        );

        PhoenixScoring.Config caseSensitive = PhoenixScoring.Config.defaults();
        caseSensitive.nameMotorShooterWheel = caseSensitive.nameMotorIntake.toUpperCase();
        TestHardwareMap hardwareMap = TestHardwareMap.forScoring(caseSensitive);
        PhoenixScoring scoring = new PhoenixScoring(
                hardwareMap,
                caseSensitive,
                targetingFor(profile)
        );
        assertTrue(hardwareMap.lookupCount > 0);
        scoring.stop();
    }

    @Test
    public void identityCrossFieldsPrecedeEveryLaterDirectionWithoutHardwareLookup() {
        PhoenixProfile profile = PhoenixProfile.current();

        PhoenixScoring.Config intakeLeft = PhoenixScoring.Config.defaults();
        intakeLeft.nameCrServoShooterTransferLeft = intakeLeft.nameCrServoIntakeTransfer;
        intakeLeft.directionCrServoShooterTransferLeft = null;
        assertProductionPreflightFailure(profile, intakeLeft, "nameCrServoIntakeTransfer");

        PhoenixScoring.Config intakeRight = PhoenixScoring.Config.defaults();
        intakeRight.nameCrServoShooterTransferRight = intakeRight.nameCrServoIntakeTransfer;
        intakeRight.directionCrServoShooterTransferRight = null;
        assertProductionPreflightFailure(profile, intakeRight, "nameCrServoIntakeTransfer");

        PhoenixScoring.Config leftRight = PhoenixScoring.Config.defaults();
        leftRight.nameCrServoShooterTransferRight =
                leftRight.nameCrServoShooterTransferLeft;
        leftRight.directionCrServoShooterTransferRight = null;
        assertProductionPreflightFailure(
                profile,
                leftRight,
                "nameCrServoShooterTransferLeft"
        );

        PhoenixScoring.Config motorPair = PhoenixScoring.Config.defaults();
        motorPair.nameMotorShooterWheel = motorPair.nameMotorIntake;
        motorPair.directionMotorShooterWheel = null;
        assertProductionPreflightFailure(profile, motorPair, "nameMotorIntake");
    }

    @Test
    public void disabledPidfIsDormantAndEnabledRevDomainRejectsBeforeLookup() {
        PhoenixProfile profile = PhoenixProfile.current();
        PhoenixScoring.Config dormant = PhoenixScoring.Config.defaults();
        dormant.flywheelVelKp = Double.NaN;
        dormant.flywheelVelKi = Double.POSITIVE_INFINITY;
        dormant.flywheelVelKd = Double.NEGATIVE_INFINITY;
        dormant.flywheelVelKf = Double.NaN;
        TestHardwareMap dormantHardware = TestHardwareMap.forScoring(dormant);
        PhoenixScoring scoring = new PhoenixScoring(
                dormantHardware,
                dormant,
                targetingFor(profile)
        );
        assertEquals(0, dormantHardware.state(dormant.nameMotorShooterWheel).velocityPidfWrites);
        scoring.stop();

        PhoenixScoring.Config active = PhoenixScoring.Config.defaults();
        active.applyFlywheelVelocityPIDF = true;
        active.flywheelVelKp = Math.nextUp(Integer.MAX_VALUE / 65536.0);
        assertProductionPreflightFailure(profile, active, "flywheelVelKp");
    }

    @Test
    public void productionDomainsRejectNonfiniteOutOfRangeAndCrossFieldValues() {
        PhoenixProfile profile = PhoenixProfile.current();

        PhoenixScoring.Config config = PhoenixScoring.Config.defaults();
        config.directionCrServoShooterTransferLeft = null;
        assertProductionPreflightFailure(profile, config,
                "directionCrServoShooterTransferLeft");

        config = PhoenixScoring.Config.defaults();
        config.shooterTransferLeftScale = Math.nextUp(1.0);
        assertProductionPreflightFailure(profile, config, "shooterTransferLeftScale");

        config = PhoenixScoring.Config.defaults();
        config.velocityMin = -1.0;
        assertProductionPreflightFailure(profile, config, "velocityMin");

        config = PhoenixScoring.Config.defaults();
        config.velocityMax = Double.POSITIVE_INFINITY;
        assertProductionPreflightFailure(profile, config, "velocityMax");

        config = PhoenixScoring.Config.defaults();
        config.velocityMin = 1000.0;
        config.velocityMax = 999.0;
        config.velocityToleranceNative = -1.0;
        assertProductionPreflightFailure(profile, config, "velocityMin");

        config = PhoenixScoring.Config.defaults();
        config.velocityToleranceBelowNative = -1.0;
        assertProductionPreflightFailure(profile, config, "velocityToleranceBelowNative");

        config = PhoenixScoring.Config.defaults();
        config.readyPredictLeadSec = Double.NaN;
        assertProductionPreflightFailure(profile, config, "readyPredictLeadSec");

        config = PhoenixScoring.Config.defaults();
        config.intakeMotorPower = Math.nextUp(1.0);
        assertProductionPreflightFailure(profile, config, "intakeMotorPower");

        config = PhoenixScoring.Config.defaults();
        config.shootFeedPulseSec = -1.0;
        assertProductionPreflightFailure(profile, config, "shootFeedPulseSec");

        config = PhoenixScoring.Config.defaults();
        config.feedScaleShooterTransfer = Double.NaN;
        assertProductionPreflightFailure(profile, config, "feedScaleShooterTransfer");

        double coefficientBound = Integer.MAX_VALUE / 65536.0;
        PhoenixScoring.Config boundaries = PhoenixScoring.Config.defaults();
        boundaries.shooterTransferLeftScale = -1.0;
        boundaries.velocityMin = 0.0;
        boundaries.velocityMax = 0.0;
        boundaries.velocityToleranceNative = 0.0;
        boundaries.applyFlywheelVelocityPIDF = true;
        boundaries.flywheelVelKp = -coefficientBound;
        boundaries.flywheelVelKi = coefficientBound;
        boundaries.flywheelVelKd = -coefficientBound;
        boundaries.flywheelVelKf = coefficientBound;
        boundaries.velocityToleranceBelowNative = 0.0;
        boundaries.velocityToleranceAboveNative = 0.0;
        boundaries.readyPredictLeadSec = 0.0;
        boundaries.readyStableSec = 0.0;
        boundaries.intakeMotorPower = 0.0;
        boundaries.intakeTransferPower = 1.0;
        boundaries.intakeShooterTransferHoldBackPower = 0.0;
        boundaries.ejectMotorPower = 1.0;
        boundaries.ejectTransferPower = 0.0;
        boundaries.ejectShooterTransferPower = 1.0;
        boundaries.shootFeedPower = 0.0;
        boundaries.shootFeedPulseSec = 0.0;
        boundaries.shootFeedCooldownSec = 0.0;
        boundaries.feedScaleIntakeMotor = -1.0;
        boundaries.feedScaleIntakeTransfer = 1.0;
        boundaries.feedScaleShooterTransfer = -1.0;
        TestHardwareMap boundaryHardware = TestHardwareMap.forScoring(boundaries);
        PhoenixScoring accepted = new PhoenixScoring(
                boundaryHardware,
                boundaries,
                targetingFor(profile)
        );
        assertTrue(boundaryHardware.lookupCount > 0);
        accepted.stop();
    }

    @Test
    public void exclusiveTunerValidatesOnlyItsExactActiveSliceBeforeLookup() {
        PhoenixScoring.Config config = PhoenixScoring.Config.defaults();
        config.nameMotorIntake = null;
        config.directionMotorIntake = null;
        config.nameCrServoIntakeTransfer = null;
        config.directionCrServoIntakeTransfer = null;
        config.nameCrServoShooterTransferLeft = null;
        config.directionCrServoShooterTransferLeft = null;
        config.nameCrServoShooterTransferRight = null;
        config.directionCrServoShooterTransferRight = null;
        config.shooterTransferLeftScale = Double.NaN;
        config.velocityToleranceBelowNative = Double.NaN;
        config.velocityToleranceAboveNative = Double.NaN;
        config.readyPredictLeadSec = Double.NaN;
        config.readyStableSec = Double.NaN;
        config.intakeMotorPower = Double.NaN;
        config.intakeTransferPower = Double.NaN;
        config.intakeShooterTransferHoldBackPower = Double.NaN;
        config.ejectMotorPower = Double.NaN;
        config.ejectTransferPower = Double.NaN;
        config.ejectShooterTransferPower = Double.NaN;
        config.shootFeedPower = Double.NaN;
        config.shootFeedPulseSec = Double.NaN;
        config.shootFeedCooldownSec = Double.NaN;
        config.feedScaleIntakeMotor = Double.NaN;
        config.feedScaleIntakeTransfer = Double.NaN;
        config.feedScaleShooterTransfer = Double.NaN;

        TestHardwareMap acceptedHardware = TestHardwareMap.forFlywheel(config);
        Plant plant = PhoenixScoring.createFlywheelPlantForTuning(acceptedHardware, config);
        assertTrue(acceptedHardware.lookupCount > 0);
        plant.stop();

        config.velocityToleranceNative = Double.NaN;
        TestHardwareMap rejectedHardware = TestHardwareMap.forFlywheel(config);
        try {
            PhoenixScoring.createFlywheelPlantForTuning(rejectedHardware, config);
            fail("expected invalid active tuner tolerance to fail");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("velocityToleranceNative"));
        }
        assertEquals(0, rejectedHardware.lookupCount);
    }

    @Test
    public void eachLaterPlantConstructionFailureStopsEveryCompletedPlantInStableOrder() {
        PhoenixProfile profile = PhoenixProfile.current();
        PhoenixScoring.Config config = PhoenixScoring.Config.defaults();

        assertConstructionRollback(
                profile,
                config,
                config.nameCrServoIntakeTransfer,
                Arrays.asList("intake.power")
        );
        assertConstructionRollback(
                profile,
                config,
                config.nameCrServoShooterTransferRight,
                Arrays.asList("intake.power", "intakeTransfer.power")
        );
        assertConstructionRollback(
                profile,
                config,
                config.nameMotorShooterWheel,
                Arrays.asList(
                        "intake.power",
                        "intakeTransfer.power",
                        "shooterTransferRight.power",
                        "shooterTransferLeft.power"
                )
        );
    }

    @Test
    public void constructionRollbackRetainsPrimaryAndSuppressesCleanupFailuresInOrder() {
        PhoenixProfile profile = PhoenixProfile.current();
        PhoenixScoring.Config config = PhoenixScoring.Config.defaults();
        TestHardwareMap hardwareMap = TestHardwareMap.forScoring(config);
        hardwareMap.remove(config.nameMotorShooterWheel);

        RuntimeException intakeFailure = new RuntimeException("intake rollback failure");
        RuntimeException intakeTransferFailure =
                new RuntimeException("intake-transfer rollback failure");
        RuntimeException shooterTransferFailure =
                new RuntimeException("shooter-transfer rollback failure");
        hardwareMap.state(config.nameMotorIntake).zeroPowerFailure = intakeFailure;
        hardwareMap.state(config.nameCrServoIntakeTransfer).zeroPowerFailure =
                intakeTransferFailure;
        hardwareMap.state(config.nameCrServoShooterTransferRight).zeroPowerFailure =
                shooterTransferFailure;

        try {
            new PhoenixScoring(hardwareMap, config, targetingFor(profile));
            fail("expected missing flywheel construction to fail");
        } catch (RuntimeException primaryFailure) {
            assertTrue(primaryFailure.getMessage().contains(config.nameMotorShooterWheel));
            assertEquals(3, primaryFailure.getSuppressed().length);
            assertSame(intakeFailure, primaryFailure.getSuppressed()[0]);
            assertSame(intakeTransferFailure, primaryFailure.getSuppressed()[1]);
            assertSame(shooterTransferFailure, primaryFailure.getSuppressed()[2]);
        }

        assertEquals(
                Arrays.asList(
                        "intake.power",
                        "intakeTransfer.power",
                        "shooterTransferRight.power",
                        "shooterTransferLeft.power"
                ),
                hardwareMap.stopEvents
        );
    }

    @Test
    public void stopIsTerminalIdempotentAndAttemptsEveryPlantAfterFailures() {
        PhoenixProfile profile = PhoenixProfile.current();
        PhoenixScoring.Config config = PhoenixScoring.Config.defaults();
        TestHardwareMap hardwareMap = TestHardwareMap.forScoring(config);
        PhoenixScoring scoring = new PhoenixScoring(
                hardwareMap,
                config,
                targetingFor(profile));

        RuntimeException flywheelFailure = new RuntimeException("flywheel stop failure");
        RuntimeException intakeFailure = new RuntimeException("intake stop failure");
        RuntimeException intakeTransferFailure = new RuntimeException("intake-transfer stop failure");
        RuntimeException shooterTransferFailure = new RuntimeException("shooter-transfer stop failure");
        hardwareMap.state(config.nameMotorShooterWheel).zeroVelocityFailure = flywheelFailure;
        hardwareMap.state(config.nameMotorIntake).zeroPowerFailure = intakeFailure;
        hardwareMap.state(config.nameCrServoIntakeTransfer).zeroPowerFailure =
                intakeTransferFailure;
        hardwareMap.state(config.nameCrServoShooterTransferRight).zeroPowerFailure =
                shooterTransferFailure;

        try {
            scoring.stop();
            fail("expected injected stop failures");
        } catch (RuntimeException actual) {
            assertSame(flywheelFailure, actual);
            assertEquals(3, actual.getSuppressed().length);
            assertSame(intakeFailure, actual.getSuppressed()[0]);
            assertSame(intakeTransferFailure, actual.getSuppressed()[1]);
            assertSame(shooterTransferFailure, actual.getSuppressed()[2]);
        }

        assertEquals(
                Arrays.asList(
                        "flywheel.velocity",
                        "intake.power",
                        "intakeTransfer.power",
                        "shooterTransferRight.power",
                        "shooterTransferLeft.power"
                ),
                hardwareMap.stopEvents
        );
        assertFalse(scoring.status().flywheelEnabled);
        assertFalse(scoring.status().flywheelRequested);

        List<String> firstStopEvents = new ArrayList<String>(hardwareMap.stopEvents);
        scoring.stop();
        assertEquals(firstStopEvents, hardwareMap.stopEvents);

        LoopClock clock = new LoopClock();
        clock.reset(0.0);
        try {
            scoring.update(clock);
            fail("expected update after terminal stop to fail");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("after stop"));
        }
    }

    private static void assertConstructionRollback(PhoenixProfile profile,
                                                   PhoenixScoring.Config config,
                                                   String missingName,
                                                   List<String> expectedStopEvents) {
        TestHardwareMap hardwareMap = TestHardwareMap.forScoring(config);
        hardwareMap.remove(missingName);

        try {
            new PhoenixScoring(hardwareMap, config, targetingFor(profile));
            fail("expected missing device construction failure for " + missingName);
        } catch (RuntimeException expected) {
            assertTrue(expected.getMessage().contains(missingName));
            assertEquals(0, expected.getSuppressed().length);
        }

        assertEquals(expectedStopEvents, hardwareMap.stopEvents);
    }

    private static void assertProductionPreflightFailure(PhoenixProfile profile,
                                                         PhoenixScoring.Config config,
                                                         String expectedPath) {
        TestHardwareMap hardwareMap = TestHardwareMap.forScoring(config);
        try {
            new PhoenixScoring(hardwareMap, config, targetingFor(profile));
            fail("expected PhoenixScoring.Config preflight failure for " + expectedPath);
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("PhoenixScoring.Config"));
            assertTrue(expected.getMessage().contains(expectedPath));
        }
        assertEquals("configuration must fail before the first hardware lookup", 0,
                hardwareMap.lookupCount);
    }

    private static void assertFeedState(PolicyFixture fixture,
                                        String expectedMode,
                                        int expectedBacklog,
                                        int expectedQueued,
                                        boolean expectedActive,
                                        double expectedOutput) {
        assertEquals(expectedMode, fixture.scoring.status().mode);
        assertEquals(expectedBacklog, fixture.scoring.status().feedBacklog);
        assertEquals(expectedQueued, fixture.scoring.status().feedQueued);
        assertEquals(expectedActive, fixture.scoring.status().feedActive);
        assertEquals(expectedOutput, fixture.scoring.status().feedOutput, 0.0);
    }

    private static final class PolicyFixture {
        final PhoenixProfile profile = PhoenixProfile.current();
        final PhoenixScoring.Config config = PhoenixScoring.Config.defaults();
        final TestHardwareMap hardwareMap;
        final PhoenixTargeting targeting;
        final PhoenixScoring scoring;
        final LoopClock clock = new LoopClock();
        boolean firstTick = true;

        PolicyFixture() {
            profile.targeting.aimReadyDebounceSec = 0.0;
            config.readyPredictLeadSec = 0.0;
            config.readyStableSec = 0.0;
            config.shootFeedPower = 0.73;
            config.shootFeedPulseSec = 0.10;
            config.shootFeedCooldownSec = 0.05;
            config.intakeMotorPower = 0.31;
            config.intakeTransferPower = 0.41;
            config.intakeShooterTransferHoldBackPower = 0.21;
            config.ejectMotorPower = 0.51;
            config.ejectTransferPower = 0.52;
            config.ejectShooterTransferPower = 0.53;

            hardwareMap = TestHardwareMap.forScoring(config);
            flywheel().measuredVelocity = config.velocityMin;
            int scoringTagId = profile.targeting.scoringTargets.keySet().iterator().next();
            targeting = new PhoenixTargeting(
                    profile.targeting,
                    profile.localization.estimation.aprilTags.fieldPoseSolver,
                    new CurrentFrameAprilTagSensor(scoringTagId),
                    CameraMountConfig.identity(),
                    new NoPoseEstimator(),
                    profile.fixedAprilTagLayout,
                    Source.constant(profile.targeting.scoringTargets.keySet()),
                    BooleanSource.constant(true),
                    BooleanSource.constant(false)
            );
            scoring = new PhoenixScoring(hardwareMap, config, targeting);
        }

        void tick(double nowSec) {
            if (firstTick) {
                clock.reset(nowSec);
                firstTick = false;
            } else {
                clock.update(nowSec);
            }
            targeting.update(clock);
            scoring.update(clock);
        }

        DeviceState intakeMotor() {
            return hardwareMap.state(config.nameMotorIntake);
        }

        DeviceState intakeTransfer() {
            return hardwareMap.state(config.nameCrServoIntakeTransfer);
        }

        DeviceState shooterTransferRight() {
            return hardwareMap.state(config.nameCrServoShooterTransferRight);
        }

        DeviceState flywheel() {
            return hardwareMap.state(config.nameMotorShooterWheel);
        }
    }

    private static PhoenixTargeting targetingFor(PhoenixProfile profile) {
        return new PhoenixTargeting(
                profile.targeting,
                profile.localization.estimation.aprilTags.fieldPoseSolver,
                new EmptyAprilTagSensor(),
                CameraMountConfig.identity(),
                new NoPoseEstimator(),
                profile.fixedAprilTagLayout,
                Source.constant(profile.targeting.scoringTargets.keySet()),
                BooleanSource.constant(true),
                BooleanSource.constant(false)
        );
    }

    private static void assertDoubleArrayEquals(double[] expected, double[] actual) {
        assertEquals(expected.length, actual.length);
        for (int index = 0; index < expected.length; index++) {
            assertEquals(expected[index], actual[index], 0.0);
        }
    }

    private static final class EmptyAprilTagSensor implements AprilTagSensor {
        @Override
        public AprilTagDetections get(LoopClock clock) {
            return AprilTagDetections.none();
        }
    }

    private static final class CurrentFrameAprilTagSensor implements AprilTagSensor {
        private final int tagId;

        CurrentFrameAprilTagSensor(int tagId) {
            this.tagId = tagId;
        }

        @Override
        public AprilTagDetections get(LoopClock clock) {
            return AprilTagDetections.fromFrame(
                    clock.nowTimestamp(),
                    Collections.singletonList(AprilTagObservation.target(
                            tagId,
                            new Pose3d(36.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                    ))
            );
        }
    }

    private static final class NoPoseEstimator implements AbsolutePoseEstimator {
        private final PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());

        @Override
        public void update(LoopClock clock) {
            // No localization state is needed for cleanup tests.
        }

        @Override
        public PoseEstimate getEstimate() {
            return estimate;
        }
    }

    private static final class TestHardwareMap extends HardwareMap {
        private final Map<String, HardwareDevice> devices = new HashMap<String, HardwareDevice>();
        private final Map<String, DeviceState> states = new HashMap<String, DeviceState>();
        private final List<String> stopEvents = new ArrayList<String>();
        int lookupCount;

        private TestHardwareMap() {
            super(null, null);
        }

        static TestHardwareMap forScoring(PhoenixScoring.Config config) {
            TestHardwareMap map = new TestHardwareMap();
            map.add(config.nameMotorIntake, DcMotorEx.class, "intake");
            map.add(config.nameMotorShooterWheel, DcMotorEx.class, "flywheel");
            map.add(config.nameCrServoIntakeTransfer, CRServo.class, "intakeTransfer");
            map.add(
                    config.nameCrServoShooterTransferRight,
                    CRServo.class,
                    "shooterTransferRight"
            );
            map.add(
                    config.nameCrServoShooterTransferLeft,
                    CRServo.class,
                    "shooterTransferLeft"
            );
            return map;
        }

        static TestHardwareMap forFlywheel(PhoenixScoring.Config config) {
            TestHardwareMap map = new TestHardwareMap();
            map.add(config.nameMotorShooterWheel, DcMotorEx.class, "flywheel");
            return map;
        }

        private <T extends HardwareDevice> void add(String name, Class<T> type, String label) {
            DeviceState state = new DeviceState(label, stopEvents);
            states.put(name, state);
            devices.put(name, deviceProxy(type, state));
        }

        DeviceState state(String name) {
            DeviceState state = states.get(name);
            if (state == null) {
                throw new AssertionError("No test state for " + name);
            }
            return state;
        }

        void remove(String name) {
            devices.remove(name);
        }

        @Override
        public <T> T get(Class<? extends T> type, String name) {
            lookupCount++;
            HardwareDevice device = devices.get(name);
            if (device == null || !type.isInstance(device)) {
                throw new IllegalArgumentException("No test " + type.getSimpleName() + " named " + name);
            }
            return type.cast(device);
        }
    }

    private static final class DeviceState {
        final String label;
        final List<String> stopEvents;
        DcMotor.RunMode motorMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
        double commandedPower;
        double commandedVelocity;
        double measuredVelocity;
        double[] velocityPidf = new double[]{0.0, 0.0, 0.0, 0.0};
        MotorControlAlgorithm velocityPidfAlgorithm = MotorControlAlgorithm.PIDF;
        int velocityPidfWrites;
        RuntimeException zeroPowerFailure;
        RuntimeException zeroVelocityFailure;

        DeviceState(String label, List<String> stopEvents) {
            this.label = label;
            this.stopEvents = stopEvents;
        }
    }

    private static <T extends HardwareDevice> T deviceProxy(Class<T> type, DeviceState state) {
        InvocationHandler handler = new InvocationHandler() {
            @Override
            public Object invoke(Object proxy, Method method, Object[] args) {
                String name = method.getName();
                if (method.getDeclaringClass() == Object.class) {
                    if ("equals".equals(name)) {
                        return proxy == args[0];
                    }
                    if ("hashCode".equals(name)) {
                        return System.identityHashCode(proxy);
                    }
                    if ("toString".equals(name)) {
                        return "TestDevice(" + state.label + ")";
                    }
                }
                if ("getManufacturer".equals(name)) {
                    return HardwareDevice.Manufacturer.Other;
                }
                if ("getDeviceName".equals(name)) {
                    return state.label;
                }
                if ("getConnectionInfo".equals(name)) {
                    return "test";
                }
                if ("getVersion".equals(name)) {
                    return 1;
                }
                if ("setMode".equals(name)) {
                    state.motorMode = (DcMotor.RunMode) args[0];
                    return null;
                }
                if ("getMode".equals(name)) {
                    return state.motorMode;
                }
                if ("setDirection".equals(name)) {
                    state.direction = (DcMotorSimple.Direction) args[0];
                    return null;
                }
                if ("getDirection".equals(name)) {
                    return state.direction;
                }
                if ("setVelocityPIDFCoefficients".equals(name)) {
                    state.velocityPidf = new double[]{
                            (Double) args[0],
                            (Double) args[1],
                            (Double) args[2],
                            (Double) args[3]
                    };
                    state.velocityPidfAlgorithm = MotorControlAlgorithm.PIDF;
                    state.velocityPidfWrites++;
                    return null;
                }
                if ("getPIDFCoefficients".equals(name)) {
                    return new PIDFCoefficients(
                            state.velocityPidf[0],
                            state.velocityPidf[1],
                            state.velocityPidf[2],
                            state.velocityPidf[3],
                            state.velocityPidfAlgorithm);
                }
                if ("setPIDFCoefficients".equals(name)) {
                    PIDFCoefficients value = (PIDFCoefficients) args[1];
                    state.velocityPidf = new double[]{value.p, value.i, value.d, value.f};
                    state.velocityPidfAlgorithm = value.algorithm;
                    state.velocityPidfWrites++;
                    return null;
                }
                if ("setPower".equals(name)) {
                    double power = (Double) args[0];
                    state.commandedPower = power;
                    if (power == 0.0) {
                        state.stopEvents.add(state.label + ".power");
                        if (state.zeroPowerFailure != null) {
                            throw state.zeroPowerFailure;
                        }
                    }
                    return null;
                }
                if ("getPower".equals(name)) {
                    return state.commandedPower;
                }
                if ("setVelocity".equals(name)) {
                    double velocity = (Double) args[0];
                    state.commandedVelocity = velocity;
                    if (velocity == 0.0) {
                        state.stopEvents.add(state.label + ".velocity");
                        if (state.zeroVelocityFailure != null) {
                            throw state.zeroVelocityFailure;
                        }
                    }
                    return null;
                }
                if ("getVelocity".equals(name)) {
                    return state.measuredVelocity;
                }
                return defaultValue(method.getReturnType());
            }
        };
        return type.cast(Proxy.newProxyInstance(
                type.getClassLoader(),
                new Class<?>[]{type},
                handler
        ));
    }

    private static Object defaultValue(Class<?> type) {
        if (!type.isPrimitive()) {
            return null;
        }
        if (type == boolean.class) {
            return false;
        }
        if (type == byte.class) {
            return (byte) 0;
        }
        if (type == short.class) {
            return (short) 0;
        }
        if (type == int.class) {
            return 0;
        }
        if (type == long.class) {
            return 0L;
        }
        if (type == float.class) {
            return 0.0f;
        }
        if (type == double.class) {
            return 0.0;
        }
        if (type == char.class) {
            return '\0';
        }
        return null;
    }
}
