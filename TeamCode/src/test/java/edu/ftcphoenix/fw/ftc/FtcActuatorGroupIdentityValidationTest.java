package edu.ftcphoenix.fw.ftc;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.junit.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.Locale;
import java.util.Map;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.Plants;
import edu.ftcphoenix.fw.actuation.PositionPlant;
import edu.ftcphoenix.fw.core.control.ScalarRegulator;
import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies SDK-equivalent, side-effect-free identity validation for FTC actuator groups. */
public final class FtcActuatorGroupIdentityValidationTest {

    private static final double EPSILON = 1.0e-9;
    private static final ScalarRegulator ZERO_REGULATOR =
            (setpoint, measurement, clock) -> 0.0;

    @Test
    public void motorGroupsRejectBlankAndDuplicateMembersBeforeEveryControlDomain() {
        for (MotorDomain domain : MotorDomain.values()) {
            assertFirstMotorRejected(domain, " \t ", "motor", "blank");
            assertLaterMotorRejectedAndOriginalRemainsUsable(
                    domain, " \t ", "motor", "blank");
            assertLaterMotorRejectedAndOriginalRemainsUsable(
                    domain, "left", "motor", "left");
            assertLaterMotorRejectedAndOriginalRemainsUsable(
                    domain, " \tleft ", "motor", "left");
        }
    }

    @Test
    public void standardServoGroupsRejectBlankAndDuplicateMembersBeforeLookup() {
        assertFirstServoRejected(" \t ", "servo", "blank");
        assertLaterServoRejectedAndOriginalRemainsUsable(" \t ", "servo", "blank");
        assertLaterServoRejectedAndOriginalRemainsUsable("claw", "servo", "claw");
        assertLaterServoRejectedAndOriginalRemainsUsable(" \tclaw ", "servo", "claw");
    }

    @Test
    public void crServoGroupsRejectBlankAndDuplicateMembersBeforePowerOrPosition() {
        for (CrServoDomain domain : CrServoDomain.values()) {
            assertFirstCrServoRejected(domain, " \t ", "CR servo", "blank");
            assertLaterCrServoRejectedAndOriginalRemainsUsable(
                    domain, " \t ", "CR servo", "blank");
            assertLaterCrServoRejectedAndOriginalRemainsUsable(
                    domain, "feed", "CR servo", "feed");
            assertLaterCrServoRejectedAndOriginalRemainsUsable(
                    domain, " \tfeed ", "CR servo", "feed");
        }
    }

    @Test
    public void actuatorFamilySelectionIsSingleUseButInvalidInputDoesNotPoisonRoot() {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        hardwareMap.addMotor("left");
        FtcActuators.StartStep root = FtcActuators.plant(hardwareMap);

        Throwable invalid = assertThrows(
                IllegalArgumentException.class,
                () -> root.servo(" \t ", Direction.FORWARD));
        assertContainsIgnoreCase(invalid, "servo", "blank");
        hardwareMap.effects.assertNone();

        FtcActuators.MotorSingleStep motor = root.motor("left", Direction.FORWARD);
        Throwable secondBranch = assertThrows(
                IllegalStateException.class,
                () -> root.crServo("feed", Direction.FORWARD));
        assertContainsIgnoreCase(secondBranch, "already selected", "new", "recipe");
        hardwareMap.effects.assertNone();

        buildMotor(motor, MotorDomain.POWER);
        assertEquals(1, hardwareMap.effects.lookupCalls);
        assertEquals(1, hardwareMap.effects.directionWrites);
    }

    @Test
    public void nullChecksKeepNameBeforeDirectionAndRejectedAddsDoNotMutateMotorGroup() {
        TestHardwareMap firstNameMap = new TestHardwareMap();
        Throwable firstName = assertThrows(
                NullPointerException.class,
                () -> FtcActuators.plant(firstNameMap).motor(null, null));
        assertContains(firstName, "name");
        firstNameMap.effects.assertNone();

        TestHardwareMap firstDirectionMap = new TestHardwareMap();
        Throwable firstDirection = assertThrows(
                NullPointerException.class,
                () -> FtcActuators.plant(firstDirectionMap)
                        .motor("left", null));
        assertContains(firstDirection, "direction");
        firstDirectionMap.effects.assertNone();

        TestHardwareMap laterMap = new TestHardwareMap();
        laterMap.addMotor("left");
        FtcActuators.MotorSingleStep retained = FtcActuators.plant(laterMap)
                .motor("left", Direction.FORWARD);

        Throwable laterName = assertThrows(
                NullPointerException.class,
                () -> retained.andMotor(null, null));
        assertContains(laterName, "name");
        Throwable laterDirection = assertThrows(
                NullPointerException.class,
                () -> retained.andMotor("right", null));
        assertContains(laterDirection, "direction");
        laterMap.effects.assertNone();

        buildMotor(retained, MotorDomain.POWER);
        assertEquals(1, laterMap.effects.lookupCalls);
        assertEquals(1, laterMap.effects.directionWrites);
    }

    @Test
    public void nullChecksKeepNameBeforeDirectionAndRejectedAddsDoNotMutateServoGroup() {
        TestHardwareMap firstNameMap = new TestHardwareMap();
        Throwable firstName = assertThrows(
                NullPointerException.class,
                () -> FtcActuators.plant(firstNameMap).servo(null, null));
        assertContains(firstName, "name");
        firstNameMap.effects.assertNone();

        TestHardwareMap firstDirectionMap = new TestHardwareMap();
        Throwable firstDirection = assertThrows(
                NullPointerException.class,
                () -> FtcActuators.plant(firstDirectionMap)
                        .servo("claw", null));
        assertContains(firstDirection, "direction");
        firstDirectionMap.effects.assertNone();

        TestHardwareMap laterMap = new TestHardwareMap();
        laterMap.addServo("claw");
        FtcActuators.ServoSingleStep retained = FtcActuators.plant(laterMap)
                .servo("claw", Direction.FORWARD);

        Throwable laterName = assertThrows(
                NullPointerException.class,
                () -> retained.andServo(null, null));
        assertContains(laterName, "name");
        Throwable laterDirection = assertThrows(
                NullPointerException.class,
                () -> retained.andServo("wrist", null));
        assertContains(laterDirection, "direction");
        laterMap.effects.assertNone();

        buildServo(retained);
        assertEquals(1, laterMap.effects.lookupCalls);
        assertEquals(1, laterMap.effects.directionWrites);
    }

    @Test
    public void nullChecksKeepNameBeforeDirectionAndRejectedAddsDoNotMutateCrServoGroup() {
        TestHardwareMap firstNameMap = new TestHardwareMap();
        Throwable firstName = assertThrows(
                NullPointerException.class,
                () -> FtcActuators.plant(firstNameMap).crServo(null, null));
        assertContains(firstName, "name");
        firstNameMap.effects.assertNone();

        TestHardwareMap firstDirectionMap = new TestHardwareMap();
        Throwable firstDirection = assertThrows(
                NullPointerException.class,
                () -> FtcActuators.plant(firstDirectionMap)
                        .crServo("feed", null));
        assertContains(firstDirection, "direction");
        firstDirectionMap.effects.assertNone();

        TestHardwareMap laterMap = new TestHardwareMap();
        laterMap.addCrServo("feed");
        FtcActuators.CrServoSingleStep retained = FtcActuators.plant(laterMap)
                .crServo("feed", Direction.FORWARD);

        Throwable laterName = assertThrows(
                NullPointerException.class,
                () -> retained.andCrServo(null, null));
        assertContains(laterName, "name");
        Throwable laterDirection = assertThrows(
                NullPointerException.class,
                () -> retained.andCrServo("indexer", null));
        assertContains(laterDirection, "direction");
        laterMap.effects.assertNone();

        buildCrServo(retained, CrServoDomain.POWER, null);
        assertEquals(1, laterMap.effects.lookupCalls);
        assertEquals(1, laterMap.effects.directionWrites);
    }

    @Test
    public void retainedTargetStageRejectsAliasAddBeforeDeferredHardwareResolution() {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        hardwareMap.addMotor("left");
        FtcActuators.MotorSingleStep retained = FtcActuators.plant(hardwareMap)
                .motor("left", Direction.FORWARD);

        Plants.TargetStep<Plant> resolvedPower = retained.power();
        int[] beforeRejectedAdd = hardwareMap.effects.snapshot();

        Throwable duplicate = assertThrows(
                IllegalArgumentException.class,
                () -> retained.andMotor(" left ", Direction.REVERSE));
        assertContainsIgnoreCase(duplicate, "motor", "left");
        assertArrayEquals(beforeRejectedAdd, hardwareMap.effects.snapshot());

        resolvedPower.targetFromNewCommand(0.0).build();
        assertEquals(1, hardwareMap.effects.lookupCalls);
        assertEquals(1, hardwareMap.effects.directionWrites);
    }

    @Test
    public void retainedVelocityAndPositionStagesRejectAddsBeforeTuningOrConfigurationEffects() {
        TestHardwareMap velocityMap = new TestHardwareMap();
        velocityMap.addMotor("left");
        FtcActuators.MotorSingleStep retainedVelocity = FtcActuators.plant(velocityMap)
                .motor("left", Direction.FORWARD);
        Plants.VelocityBoundsStep<Plants.TargetStep<Plant>> velocity = retainedVelocity.velocity()
                .deviceManagedWithOverrides()
                .velocityPidf(1.0, 2.0, 3.0, 4.0);

        assertThrows(IllegalArgumentException.class,
                () -> retainedVelocity.andMotor(" left ", Direction.REVERSE));
        velocityMap.effects.assertNone();
        velocity.unbounded()
                .nativeUnits()
                .velocityTolerance(0.0)
                .targetFromNewCommand(0.0)
                .build();
        assertEquals(1, velocityMap.effects.velocityPidfWrites);

        TestHardwareMap positionMap = new TestHardwareMap();
        positionMap.addMotor("lift");
        FtcActuators.MotorSingleStep retainedPosition = FtcActuators.plant(positionMap)
                .motor("lift", Direction.FORWARD);
        Plants.PositionPeriodicityStep<Plants.FeedbackPositionBoundsStep<
                Plants.SymmetricOutputPowerPolicyStep<PositionPlant>>> position =
                retainedPosition.position()
                .deviceManagedWithOverrides()
                .outerPositionP(7.0)
                .innerVelocityPidf(1.0, 2.0, 3.0, 4.0)
                .devicePositionToleranceTicks(12)
                .doneOverrides();

        assertThrows(IllegalArgumentException.class,
                () -> retainedPosition.andMotor("lift", Direction.REVERSE));
        positionMap.effects.assertNone();
        position.nonPeriodic()
                .unbounded()
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(0.0)
                .targetFromNewCommand(0.0)
                .build();
        assertEquals(1, positionMap.effects.positionPidfWrites);
        assertEquals(1, positionMap.effects.velocityPidfWrites);
        assertEquals(1, positionMap.effects.targetToleranceWrites);
    }

    @Test
    public void motorPowerGroupPrevalidatesListShapeAndEveryMemberBeforeLookup() {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        hardwareMap.addMotor("left");
        hardwareMap.addMotor("right");

        assertThrows(IllegalArgumentException.class,
                () -> FtcHardware.motorPowerGroup(
                        hardwareMap, null, Collections.singletonList(Direction.FORWARD)));
        assertThrows(IllegalArgumentException.class,
                () -> FtcHardware.motorPowerGroup(
                        hardwareMap, Collections.singletonList("left"), null));
        assertThrows(IllegalArgumentException.class,
                () -> FtcHardware.motorPowerGroup(
                        hardwareMap, Collections.emptyList(), Collections.emptyList()));
        assertThrows(IllegalArgumentException.class,
                () -> FtcHardware.motorPowerGroup(
                        hardwareMap,
                        Arrays.asList("left", "right"),
                        Collections.singletonList(Direction.FORWARD)));
        assertThrows(IllegalArgumentException.class,
                () -> FtcHardware.motorPowerGroup(
                        hardwareMap,
                        Arrays.asList("left", null),
                        Arrays.asList(Direction.FORWARD, Direction.REVERSE)));
        assertThrows(IllegalArgumentException.class,
                () -> FtcHardware.motorPowerGroup(
                        hardwareMap,
                        Arrays.asList("left", "right"),
                        Arrays.asList(Direction.FORWARD, null)));
        assertThrows(IllegalArgumentException.class,
                () -> FtcHardware.motorPowerGroup(
                        hardwareMap,
                        Arrays.asList("left", " \t "),
                        Arrays.asList(Direction.FORWARD, Direction.REVERSE)));
        assertThrows(IllegalArgumentException.class,
                () -> FtcHardware.motorPowerGroup(
                        hardwareMap,
                        Arrays.asList("left", " left "),
                        Arrays.asList(Direction.FORWARD, Direction.REVERSE)));

        hardwareMap.effects.assertNone();
    }

    @Test
    public void sdkTrimmedLookupAcceptsWhitespaceAndCaseDistinctGroupMembers() {
        TestHardwareMap motorMap = new TestHardwareMap();
        motorMap.addMotor("left");
        motorMap.addMotor("Left");
        buildMotor(
                FtcActuators.plant(motorMap)
                        .motor(" left ", Direction.FORWARD)
                        .andMotor("\tLeft\t", Direction.REVERSE),
                MotorDomain.POWER);
        assertEquals(2, motorMap.effects.lookupCalls);
        assertEquals(2, motorMap.effects.directionWrites);

        TestHardwareMap servoMap = new TestHardwareMap();
        servoMap.addServo("claw");
        servoMap.addServo("Claw");
        buildServo(FtcActuators.plant(servoMap)
                .servo(" claw ", Direction.FORWARD)
                .andServo("\tClaw\t", Direction.REVERSE));
        assertEquals(2, servoMap.effects.lookupCalls);
        assertEquals(2, servoMap.effects.directionWrites);

        TestHardwareMap crServoMap = new TestHardwareMap();
        crServoMap.addCrServo("feed");
        crServoMap.addCrServo("Feed");
        buildCrServo(FtcActuators.plant(crServoMap)
                        .crServo(" feed ", Direction.FORWARD)
                        .andCrServo("\tFeed\t", Direction.REVERSE),
                CrServoDomain.POWER,
                null);
        assertEquals(2, crServoMap.effects.lookupCalls);
        assertEquals(2, crServoMap.effects.directionWrites);
    }

    @Test
    public void externalVelocityFeedbackMayReusePoweredMotorWithTrimEquivalentSpelling() {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        MotorProbe flywheel = hardwareMap.addMotor("flywheel");
        flywheel.positionTicks = 100;
        Plant plant = FtcActuators.plant(hardwareMap)
                .motor("flywheel", Direction.FORWARD)
                .velocity()
                .regulated()
                .externalEncoder(" \tflywheel ")
                .unbounded()
                .nativeUnits()
                .velocityTolerance(0.0)
                .controlFromCustomRegulator(ZERO_REGULATOR)
                .targetFromNewCommand(0.0)
                .build();

        plant.update(new ManualLoopClock().clock());

        assertEquals(2, hardwareMap.effects.lookupCalls);
        assertEquals(1, flywheel.positionReads);
        assertEquals(0, flywheel.velocityReads);
    }

    @Test
    public void externalPositionFeedbackMayReusePoweredMotorWithTrimEquivalentSpelling() {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        MotorProbe lift = hardwareMap.addMotor("lift");
        lift.positionTicks = 321;
        PositionPlant plant = FtcActuators.plant(hardwareMap)
                .motor("lift", Direction.FORWARD)
                .position()
                .regulated()
                .externalEncoder(" \tlift ")
                .nonPeriodic()
                .unbounded()
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(0.0)
                .controlFromCustomRegulator(ZERO_REGULATOR)
                .targetFromNewCommand(0.0)
                .build();

        plant.update(new ManualLoopClock().clock());

        assertEquals(321.0, plant.getMeasurement(), EPSILON);
        assertEquals(2, hardwareMap.effects.lookupCalls);
        assertEquals(1, lift.positionReads);
    }

    @Test
    public void namedInternalVelocityEncoderUsesSdkEquivalentTrimmedIdentity() {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        MotorProbe left = hardwareMap.addMotor("left");
        MotorProbe right = hardwareMap.addMotor("right");
        left.velocityTicksPerSec = 2468.0;
        right.velocityTicksPerSec = 999.0;
        Plant plant = FtcActuators.plant(hardwareMap)
                .motor(" \tleft ", Direction.FORWARD)
                .andMotor("right", Direction.REVERSE)
                .velocity()
                .regulated()
                .internalEncoder("left")
                .unbounded()
                .nativeUnits()
                .velocityTolerance(0.0)
                .controlFromCustomRegulator(ZERO_REGULATOR)
                .targetFromNewCommand(0.0)
                .build();

        plant.update(new ManualLoopClock().clock());

        assertEquals(2468.0, plant.getMeasurement(), EPSILON);
        assertEquals(1, left.velocityReads);
        assertEquals(0, right.velocityReads);
    }

    @Test
    public void namedInternalPositionEncoderUsesSdkEquivalentTrimmedIdentity() {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        MotorProbe left = hardwareMap.addMotor("left");
        MotorProbe right = hardwareMap.addMotor("right");
        left.positionTicks = 135;
        right.positionTicks = 975;
        PositionPlant plant = FtcActuators.plant(hardwareMap)
                .motor("left", Direction.FORWARD)
                .andMotor(" \tright ", Direction.REVERSE)
                .position()
                .regulated()
                .internalEncoder("right")
                .nonPeriodic()
                .unbounded()
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(0.0)
                .controlFromCustomRegulator(ZERO_REGULATOR)
                .targetFromNewCommand(0.0)
                .build();

        plant.update(new ManualLoopClock().clock());

        assertEquals(975.0, plant.getMeasurement(), EPSILON);
        assertEquals(0, left.positionReads);
        assertEquals(1, right.positionReads);
    }

    @Test
    public void separatePlantsMayDeliberatelyReuseOneConfiguredName() {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        hardwareMap.addMotor("intake");

        buildMotor(FtcActuators.plant(hardwareMap)
                .motor("intake", Direction.FORWARD), MotorDomain.POWER);
        buildMotor(FtcActuators.plant(hardwareMap)
                .motor(" intake ", Direction.FORWARD), MotorDomain.POWER);

        assertEquals(2, hardwareMap.effects.lookupCalls);
        assertEquals(2, hardwareMap.effects.directionWrites);
    }

    @Test
    public void mecanumConstructionRejectsBlankAndTrimDuplicateNamesBeforeEffects() {
        TestHardwareMap wiringMap = defaultDriveMap();
        FtcDrives.MecanumWiringConfig wiring = FtcDrives.MecanumWiringConfig.defaults();
        wiring.frontRightName = " frontLeftMotor ";
        FtcDrives.MecanumConfig wiringConfig = FtcDrives.MecanumConfig.defaults();
        wiringConfig.wiring = wiring;

        Throwable wiringFailure = assertThrows(
                IllegalArgumentException.class,
                () -> FtcDrives.mecanum(wiringMap, wiringConfig));
        assertContainsIgnoreCase(wiringFailure, "motor", "front");
        wiringMap.effects.assertNone();

        TestHardwareMap customMap = defaultDriveMap();
        FtcDrives.MecanumConfig blankConfig = FtcDrives.MecanumConfig.defaults();
        blankConfig.wiring.frontRightName = " \t ";
        Throwable blankFailure = assertThrows(
                IllegalArgumentException.class,
                () -> FtcDrives.mecanum(customMap, blankConfig));
        assertContainsIgnoreCase(blankFailure, "motor", "blank");
        customMap.effects.assertNone();
    }

    @Test
    public void mecanumConstructionRejectsMissingNestedConfigsBeforeEffects() {
        TestHardwareMap missingWiringMap = defaultDriveMap();
        FtcDrives.MecanumConfig missingWiring = FtcDrives.MecanumConfig.defaults();
        missingWiring.wiring = null;

        Throwable wiringFailure = assertThrows(
                NullPointerException.class,
                () -> FtcDrives.mecanum(missingWiringMap, missingWiring));
        assertContainsIgnoreCase(wiringFailure, "wiring");
        missingWiringMap.effects.assertNone();

        TestHardwareMap missingDrivebaseMap = defaultDriveMap();
        FtcDrives.MecanumConfig missingDrivebase = FtcDrives.MecanumConfig.defaults();
        missingDrivebase.drivebase = null;

        Throwable drivebaseFailure = assertThrows(
                NullPointerException.class,
                () -> FtcDrives.mecanum(missingDrivebaseMap, missingDrivebase));
        assertContainsIgnoreCase(drivebaseFailure, "drivebase");
        missingDrivebaseMap.effects.assertNone();
    }

    @Test
    public void mecanumConstructionRejectsInvalidDriveScalesBeforeEffects() {
        double[] invalidValues = {
                Double.NaN,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY,
                -0.01,
                1.01
        };
        for (double invalidValue : invalidValues) {
            assertInvalidMecanumScale("maxAxial", invalidValue);
            assertInvalidMecanumScale("maxLateral", invalidValue);
            assertInvalidMecanumScale("maxOmega", invalidValue);
        }
    }

    @Test
    public void mecanumConstructionResolvesCompleteGroupBeforeConfigurationEffects() {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        hardwareMap.addMotor(FtcDrives.DEFAULT_FRONT_LEFT_MOTOR_NAME);
        hardwareMap.addMotor(FtcDrives.DEFAULT_FRONT_RIGHT_MOTOR_NAME);
        hardwareMap.addMotor(FtcDrives.DEFAULT_BACK_LEFT_MOTOR_NAME);

        Throwable failure = assertThrows(
                IllegalArgumentException.class,
                () -> FtcDrives.mecanum(hardwareMap));

        assertContainsIgnoreCase(failure, "backRightMotor");
        assertEquals(4, hardwareMap.effects.lookupCalls);
        assertEquals(0, hardwareMap.effects.directionWrites);
        assertEquals(0, hardwareMap.effects.zeroPowerBehaviorWrites);
        assertEquals(0, hardwareMap.effects.modeReads);
        assertEquals(0, hardwareMap.effects.modeWrites);
        assertEquals(0, hardwareMap.effects.powerWrites);
    }

    @Test
    public void validTrimmedMecanumNamesResolveOnceAndConfigureDirectionAndBrake() {
        TestHardwareMap constructionMap = defaultDriveMap();
        FtcDrives.MecanumConfig construction = FtcDrives.MecanumConfig.defaults();
        construction.wiring = whitespaceDriveWiring();

        FtcDrives.mecanum(constructionMap, construction);

        assertEquals(4, constructionMap.effects.lookupCalls);
        assertEquals(4, constructionMap.effects.directionWrites);
        assertEquals(4, constructionMap.effects.zeroPowerBehaviorWrites);
        assertEquals(0, constructionMap.effects.modeReads);
        assertEquals(0, constructionMap.effects.modeWrites);
        assertEquals(0, constructionMap.effects.powerWrites);
    }

    private static void assertFirstMotorRejected(MotorDomain domain,
                                                 String name,
                                                 String... messageFragments) {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        Throwable failure = assertThrows(
                IllegalArgumentException.class,
                () -> FtcActuators.plant(hardwareMap)
                        .motor(name, Direction.FORWARD));
        assertContainsIgnoreCase(failure, messageFragments);
        hardwareMap.effects.assertNone();
    }

    private static void assertLaterMotorRejectedAndOriginalRemainsUsable(
            MotorDomain domain,
            String rejectedName,
            String... messageFragments) {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        hardwareMap.addMotor("left");
        FtcActuators.MotorSingleStep retained = FtcActuators.plant(hardwareMap)
                .motor("left", Direction.FORWARD);

        Throwable failure = assertThrows(
                IllegalArgumentException.class,
                () -> retained.andMotor(rejectedName, Direction.REVERSE));
        assertContainsIgnoreCase(failure, messageFragments);
        hardwareMap.effects.assertNone();

        buildMotor(retained, domain);
        assertTrue("The original motor must remain buildable for " + domain,
                hardwareMap.effects.lookupCalls > 0);
    }

    private static void assertFirstServoRejected(String name, String... messageFragments) {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        Throwable failure = assertThrows(
                IllegalArgumentException.class,
                () -> FtcActuators.plant(hardwareMap)
                        .servo(name, Direction.FORWARD));
        assertContainsIgnoreCase(failure, messageFragments);
        hardwareMap.effects.assertNone();
    }

    private static void assertLaterServoRejectedAndOriginalRemainsUsable(
            String rejectedName,
            String... messageFragments) {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        hardwareMap.addServo("claw");
        FtcActuators.ServoSingleStep retained = FtcActuators.plant(hardwareMap)
                .servo("claw", Direction.FORWARD);

        Throwable failure = assertThrows(
                IllegalArgumentException.class,
                () -> retained.andServo(rejectedName, Direction.REVERSE));
        assertContainsIgnoreCase(failure, messageFragments);
        hardwareMap.effects.assertNone();

        buildServo(retained);
        assertEquals(1, hardwareMap.effects.lookupCalls);
    }

    private static void assertFirstCrServoRejected(CrServoDomain domain,
                                                   String name,
                                                   String... messageFragments) {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        Throwable failure = assertThrows(
                IllegalArgumentException.class,
                () -> FtcActuators.plant(hardwareMap)
                        .crServo(name, Direction.FORWARD));
        assertContainsIgnoreCase(failure, messageFragments);
        hardwareMap.effects.assertNone();
    }

    private static void assertLaterCrServoRejectedAndOriginalRemainsUsable(
            CrServoDomain domain,
            String rejectedName,
            String... messageFragments) {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        hardwareMap.addCrServo("feed");
        MotorProbe encoder = null;
        if (domain == CrServoDomain.POSITION) {
            encoder = hardwareMap.addMotor("encoder");
        }
        FtcActuators.CrServoSingleStep retained = FtcActuators.plant(hardwareMap)
                .crServo("feed", Direction.FORWARD);

        Throwable failure = assertThrows(
                IllegalArgumentException.class,
                () -> retained.andCrServo(rejectedName, Direction.REVERSE));
        assertContainsIgnoreCase(failure, messageFragments);
        hardwareMap.effects.assertNone();

        buildCrServo(retained, domain, encoder);
        assertTrue("The original CR servo must remain buildable for " + domain,
                hardwareMap.effects.lookupCalls > 0);
    }

    private static Plant buildMotor(FtcActuators.MotorSingleStep step, MotorDomain domain) {
        switch (domain) {
            case POWER:
                return step.power()
                        .targetFromNewCommand(0.0)
                        .build();
            case VELOCITY:
                return step.velocity()
                        .deviceManaged()
                        .unbounded()
                        .nativeUnits()
                        .velocityTolerance(0.0)
                        .targetFromNewCommand(0.0)
                        .build();
            case POSITION:
                return step.position()
                        .deviceManaged()
                        .nonPeriodic()
                        .unbounded()
                        .nativeUnits()
                        .alreadyReferenced()
                        .positionTolerance(0.0)
                        .targetFromNewCommand(0.0)
                        .build();
            default:
                throw new AssertionError("Unhandled domain " + domain);
        }
    }

    private static PositionPlant buildServo(FtcActuators.ServoSingleStep step) {
        return step.position()
                .nonPeriodic()
                .bounded(0.0, 1.0)
                .nativeUnits()
                .targetFromNewCommand(0.0)
                .build();
    }

    private static Plant buildCrServo(FtcActuators.CrServoSingleStep step,
                                      CrServoDomain domain,
                                      MotorProbe encoder) {
        if (domain == CrServoDomain.POWER) {
            return step.power()
                    .targetFromNewCommand(0.0)
                    .build();
        }
        if (encoder == null) {
            throw new AssertionError("Position test requires an encoder");
        }
        return step.position()
                .regulated()
                .externalEncoder("encoder")
                .nonPeriodic()
                .unbounded()
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(0.0)
                .controlFromCustomRegulator(ZERO_REGULATOR)
                .targetFromNewCommand(0.0)
                .build();
    }

    private static TestHardwareMap defaultDriveMap() {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        hardwareMap.addMotor(FtcDrives.DEFAULT_FRONT_LEFT_MOTOR_NAME);
        hardwareMap.addMotor(FtcDrives.DEFAULT_FRONT_RIGHT_MOTOR_NAME);
        hardwareMap.addMotor(FtcDrives.DEFAULT_BACK_LEFT_MOTOR_NAME);
        hardwareMap.addMotor(FtcDrives.DEFAULT_BACK_RIGHT_MOTOR_NAME);
        return hardwareMap;
    }

    private static FtcDrives.MecanumWiringConfig whitespaceDriveWiring() {
        FtcDrives.MecanumWiringConfig wiring = FtcDrives.MecanumWiringConfig.defaults();
        wiring.frontLeftName = " " + wiring.frontLeftName;
        wiring.frontRightName = wiring.frontRightName + " ";
        wiring.backLeftName = "\t" + wiring.backLeftName + "\t";
        wiring.backRightName = "\r\n" + wiring.backRightName + "\r\n";
        return wiring;
    }

    private static void assertInvalidMecanumScale(String fieldName, double invalidValue) {
        TestHardwareMap hardwareMap = defaultDriveMap();
        FtcDrives.MecanumConfig config = FtcDrives.MecanumConfig.defaults();
        if ("maxAxial".equals(fieldName)) {
            config.drivebase.maxAxial = invalidValue;
        } else if ("maxLateral".equals(fieldName)) {
            config.drivebase.maxLateral = invalidValue;
        } else if ("maxOmega".equals(fieldName)) {
            config.drivebase.maxOmega = invalidValue;
        } else {
            throw new AssertionError("Unhandled mecanum scale " + fieldName);
        }

        Throwable failure = assertThrows(
                IllegalArgumentException.class,
                () -> FtcDrives.mecanum(hardwareMap, config));

        assertContainsIgnoreCase(failure, fieldName, "finite", "[0.0, 1.0]");
        hardwareMap.effects.assertNone();
    }

    private static Throwable assertThrows(Class<? extends Throwable> expectedType,
                                          Runnable action) {
        try {
            action.run();
        } catch (Throwable actual) {
            assertTrue("Expected " + expectedType.getSimpleName() + " but got " + actual,
                    expectedType.isInstance(actual));
            return actual;
        }
        fail("Expected " + expectedType.getSimpleName());
        throw new AssertionError("unreachable");
    }

    private static void assertContains(Throwable failure, String fragment) {
        assertTrue("Expected message containing '" + fragment + "' but got: "
                        + failure.getMessage(),
                failure.getMessage() != null && failure.getMessage().contains(fragment));
    }

    private static void assertContainsIgnoreCase(Throwable failure, String... fragments) {
        String message = failure.getMessage();
        assertTrue("Expected a non-empty failure message", message != null && !message.isEmpty());
        String lowerMessage = message.toLowerCase(Locale.ROOT);
        for (String fragment : fragments) {
            assertTrue("Expected message containing '" + fragment + "' but got: " + message,
                    lowerMessage.contains(fragment.toLowerCase(Locale.ROOT)));
        }
    }

    private enum MotorDomain {
        POWER,
        VELOCITY,
        POSITION
    }

    private enum CrServoDomain {
        POWER,
        POSITION
    }

    /** In-memory HardwareMap with the SDK's trim-then-case-sensitive lookup behavior. */
    private static final class TestHardwareMap extends HardwareMap {
        private final Map<String, HardwareDevice> devices = new HashMap<>();
        private final Effects effects = new Effects();

        private TestHardwareMap() {
            super(null, null);
        }

        private MotorProbe addMotor(String name) {
            MotorProbe probe = new MotorProbe(effects);
            devices.put(name.trim(), probe.motor());
            return probe;
        }

        private void addServo(String name) {
            devices.put(name.trim(), new ServoProbe(effects).servo());
        }

        private void addCrServo(String name) {
            devices.put(name.trim(), new CrServoProbe(effects).servo());
        }

        @Override
        public <T> T get(Class<? extends T> type, String name) {
            effects.lookupCalls++;
            String effectiveName = name == null ? null : name.trim();
            HardwareDevice device = devices.get(effectiveName);
            if (device == null) {
                throw new IllegalArgumentException("No test device named " + name);
            }
            if (!type.isInstance(device)) {
                throw new IllegalArgumentException(
                        name + " is not a " + type.getSimpleName());
            }
            return type.cast(device);
        }
    }

    /** Aggregate hardware effects used to prove validation occurs before SDK interaction. */
    private static final class Effects {
        private int lookupCalls;
        private int directionWrites;
        private int modeReads;
        private int modeWrites;
        private int powerWrites;
        private int positionWrites;
        private int velocityWrites;
        private int targetPositionWrites;
        private int positionReads;
        private int velocityReads;
        private int positionPidfWrites;
        private int velocityPidfWrites;
        private int targetToleranceWrites;
        private int zeroPowerBehaviorWrites;

        private int[] snapshot() {
            return new int[]{
                    lookupCalls,
                    directionWrites,
                    modeReads,
                    modeWrites,
                    powerWrites,
                    positionWrites,
                    velocityWrites,
                    targetPositionWrites,
                    positionReads,
                    velocityReads,
                    positionPidfWrites,
                    velocityPidfWrites,
                    targetToleranceWrites,
                    zeroPowerBehaviorWrites
            };
        }

        private void assertNone() {
            assertArrayEquals(new int[snapshot().length], snapshot());
        }
    }

    /** Dynamic SDK motor whose reads, configuration, tuning, and commands are all observable. */
    private static final class MotorProbe {
        private final Effects effects;
        private final DcMotorEx motor;
        private int positionTicks;
        private double velocityTicksPerSec;
        private int positionReads;
        private int velocityReads;
        private DcMotor.RunMode mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;

        private MotorProbe(Effects effects) {
            this.effects = effects;
            motor = (DcMotorEx) Proxy.newProxyInstance(
                    DcMotorEx.class.getClassLoader(),
                    new Class<?>[]{DcMotorEx.class},
                    this::invoke);
        }

        private DcMotorEx motor() {
            return motor;
        }

        private Object invoke(Object proxy, Method method, Object[] args) {
            String name = method.getName();
            if (method.getDeclaringClass() == Object.class) {
                return objectMethod(proxy, name, args, "MotorProbe");
            }
            if ("setDirection".equals(name)) {
                effects.directionWrites++;
                return null;
            }
            if ("getDirection".equals(name)) {
                return DcMotorSimple.Direction.FORWARD;
            }
            if ("getMode".equals(name)) {
                effects.modeReads++;
                return mode;
            }
            if ("setMode".equals(name)) {
                effects.modeWrites++;
                mode = (DcMotor.RunMode) args[0];
                return null;
            }
            if ("setPower".equals(name)) {
                effects.powerWrites++;
                return null;
            }
            if ("getPower".equals(name)) {
                return 0.0;
            }
            if ("setVelocity".equals(name)) {
                effects.velocityWrites++;
                return null;
            }
            if ("getVelocity".equals(name)) {
                effects.velocityReads++;
                velocityReads++;
                return velocityTicksPerSec;
            }
            if ("setTargetPosition".equals(name)) {
                effects.targetPositionWrites++;
                return null;
            }
            if ("getTargetPosition".equals(name)) {
                return 0;
            }
            if ("getCurrentPosition".equals(name)) {
                effects.positionReads++;
                positionReads++;
                return positionTicks;
            }
            if ("setPositionPIDFCoefficients".equals(name)) {
                effects.positionPidfWrites++;
                return null;
            }
            if ("setVelocityPIDFCoefficients".equals(name)) {
                effects.velocityPidfWrites++;
                return null;
            }
            if ("setTargetPositionTolerance".equals(name)) {
                effects.targetToleranceWrites++;
                return null;
            }
            if ("setZeroPowerBehavior".equals(name)) {
                effects.zeroPowerBehaviorWrites++;
                return null;
            }
            if ("getZeroPowerBehavior".equals(name)) {
                return DcMotor.ZeroPowerBehavior.FLOAT;
            }
            if ("isBusy".equals(name)) {
                return false;
            }
            return hardwareDeviceOrDefault(method);
        }
    }

    /** Dynamic standard servo with observable configuration and command calls. */
    private static final class ServoProbe {
        private final Effects effects;
        private final Servo servo;

        private ServoProbe(Effects effects) {
            this.effects = effects;
            servo = (Servo) Proxy.newProxyInstance(
                    Servo.class.getClassLoader(),
                    new Class<?>[]{Servo.class},
                    this::invoke);
        }

        private Servo servo() {
            return servo;
        }

        private Object invoke(Object proxy, Method method, Object[] args) {
            String name = method.getName();
            if (method.getDeclaringClass() == Object.class) {
                return objectMethod(proxy, name, args, "ServoProbe");
            }
            if ("setDirection".equals(name)) {
                effects.directionWrites++;
                return null;
            }
            if ("getDirection".equals(name)) {
                return Servo.Direction.FORWARD;
            }
            if ("setPosition".equals(name)) {
                effects.positionWrites++;
                return null;
            }
            if ("getPosition".equals(name)) {
                return 0.0;
            }
            return hardwareDeviceOrDefault(method);
        }
    }

    /** Dynamic CR servo with observable configuration and command calls. */
    private static final class CrServoProbe {
        private final Effects effects;
        private final CRServo servo;

        private CrServoProbe(Effects effects) {
            this.effects = effects;
            servo = (CRServo) Proxy.newProxyInstance(
                    CRServo.class.getClassLoader(),
                    new Class<?>[]{CRServo.class},
                    this::invoke);
        }

        private CRServo servo() {
            return servo;
        }

        private Object invoke(Object proxy, Method method, Object[] args) {
            String name = method.getName();
            if (method.getDeclaringClass() == Object.class) {
                return objectMethod(proxy, name, args, "CrServoProbe");
            }
            if ("setDirection".equals(name)) {
                effects.directionWrites++;
                return null;
            }
            if ("getDirection".equals(name)) {
                return DcMotorSimple.Direction.FORWARD;
            }
            if ("setPower".equals(name)) {
                effects.powerWrites++;
                return null;
            }
            if ("getPower".equals(name)) {
                return 0.0;
            }
            return hardwareDeviceOrDefault(method);
        }
    }

    private static Object objectMethod(Object proxy,
                                       String methodName,
                                       Object[] args,
                                       String label) {
        if ("equals".equals(methodName)) {
            return proxy == args[0];
        }
        if ("hashCode".equals(methodName)) {
            return System.identityHashCode(proxy);
        }
        if ("toString".equals(methodName)) {
            return label;
        }
        return null;
    }

    private static Object hardwareDeviceOrDefault(Method method) {
        String name = method.getName();
        if ("getManufacturer".equals(name)) {
            return HardwareDevice.Manufacturer.Other;
        }
        if ("getDeviceName".equals(name)) {
            return "ACT-01 probe";
        }
        if ("getConnectionInfo".equals(name)) {
            return "test";
        }
        if ("getVersion".equals(name)) {
            return 1;
        }
        return defaultValue(method.getReturnType());
    }

    private static Object defaultValue(Class<?> returnType) {
        if (!returnType.isPrimitive()) {
            return null;
        }
        if (returnType == boolean.class) {
            return false;
        }
        if (returnType == byte.class) {
            return (byte) 0;
        }
        if (returnType == short.class) {
            return (short) 0;
        }
        if (returnType == int.class) {
            return 0;
        }
        if (returnType == long.class) {
            return 0L;
        }
        if (returnType == float.class) {
            return 0.0f;
        }
        if (returnType == double.class) {
            return 0.0;
        }
        if (returnType == char.class) {
            return '\0';
        }
        return null;
    }
}
