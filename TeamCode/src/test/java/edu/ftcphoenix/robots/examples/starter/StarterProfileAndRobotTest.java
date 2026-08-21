package edu.ftcphoenix.robots.examples.starter;

import edu.ftcphoenix.robots.examples.starter.support.StarterTestHardware;
import edu.ftcphoenix.robots.examples.starter.opmode.StarterAuto;
import edu.ftcphoenix.robots.examples.starter.capability.intake.StarterIntake;
import edu.ftcphoenix.robots.examples.starter.opmode.StarterTeleOp;
import edu.ftcphoenix.robots.examples.starter.robot.StarterProfile;
import edu.ftcphoenix.robots.examples.starter.robot.StarterRobot;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.ftc.FtcRobotOpMode;
import edu.ftcphoenix.fw.ftc.RobotProgram;

import static edu.ftcphoenix.robots.examples.starter.support.StarterTestHardware.fullTeleOpHardware;
import static edu.ftcphoenix.robots.examples.starter.support.StarterTestHardware.prepare;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies starter configuration consumption, declarations, and managed lifecycle cleanup. */
public final class StarterProfileAndRobotTest {

    @Test
    public void rootAndProductionHostsExposeOnlyTheApprovedConstructionSurface()
            throws Exception {
        assertTrue(Modifier.isPublic(StarterRobot.class.getModifiers()));
        assertTrue(Modifier.isFinal(StarterRobot.class.getModifiers()));

        Constructor<?>[] robotConstructors = StarterRobot.class.getDeclaredConstructors();
        assertEquals(1, robotConstructors.length);
        assertTrue(Modifier.isPublic(robotConstructors[0].getModifiers()));
        assertEquals(
                Arrays.asList(HardwareMap.class),
                Arrays.asList(robotConstructors[0].getParameterTypes()));

        Method teleOp = StarterRobot.class.getDeclaredMethod(
                "declareTeleOp",
                RobotProgram.class,
                StarterProfile.class,
                Gamepad.class);
        assertTrue(Modifier.isPublic(teleOp.getModifiers()));
        assertEquals(Void.TYPE, teleOp.getReturnType());
        assertEquals(1, declaredMethodCount(StarterRobot.class, "declareTeleOp"));

        Method auto = StarterRobot.class.getDeclaredMethod(
                "declareAuto",
                RobotProgram.class,
                StarterProfile.class);
        assertTrue(Modifier.isPublic(auto.getModifiers()));
        assertEquals(StarterIntake.class, auto.getReturnType());
        assertEquals(1, declaredMethodCount(StarterRobot.class, "declareAuto"));

        Field[] rootFields = StarterRobot.class.getDeclaredFields();
        assertEquals(1, rootFields.length);
        assertEquals(HardwareMap.class, rootFields[0].getType());
        assertTrue(Modifier.isPrivate(rootFields[0].getModifiers()));
        assertTrue(Modifier.isFinal(rootFields[0].getModifiers()));

        assertOnlyPublicNoArgConstructor(StarterTeleOp.class);
        assertOnlyPublicNoArgConstructor(StarterAuto.class);
        assertNoRetainedProfile(StarterTeleOp.class);
        assertNoRetainedProfile(StarterAuto.class);
    }

    @Test
    public void currentReturnsFreshCompleteSoftwareDefaultsWithMotionBlocked() {
        StarterProfile first = StarterProfile.current();
        StarterProfile second = StarterProfile.current();

        assertNotSame(first.intake, second.intake);
        assertNotSame(first.drive, second.drive);
        assertNotSame(first.drive.wiring, second.drive.wiring);
        assertNotSame(first.drive.drivebase, second.drive.drivebase);

        assertEquals("intakeMotor", first.intake.motorName);
        assertEquals(Direction.FORWARD, first.intake.direction);
        assertEquals(0.20, first.intake.collectPower, 0.0);
        assertEquals(-0.20, first.intake.ejectPower, 0.0);
        assertFalse(first.allowIntakeMotion);

        assertEquals("frontLeftMotor", first.drive.wiring.frontLeftName);
        assertEquals("frontRightMotor", first.drive.wiring.frontRightName);
        assertEquals("backLeftMotor", first.drive.wiring.backLeftName);
        assertEquals("backRightMotor", first.drive.wiring.backRightName);
        assertEquals(Direction.FORWARD, first.drive.wiring.frontLeftDirection);
        assertEquals(Direction.REVERSE, first.drive.wiring.frontRightDirection);
        assertEquals(Direction.FORWARD, first.drive.wiring.backLeftDirection);
        assertEquals(Direction.REVERSE, first.drive.wiring.backRightDirection);
        assertTrue(first.drive.enableZeroPowerBrake);
        assertEquals(0.25, first.drive.drivebase.maxAxial, 0.0);
        assertEquals(0.25, first.drive.drivebase.maxLateral, 0.0);
        assertEquals(0.20, first.drive.drivebase.maxOmega, 0.0);
        assertFalse(first.allowDriveMotion);
    }

    @Test
    public void permissionsRepresentAllFourStatesAndAutoNeverReadsDrive() {
        assertPermissionState(false, false);
        assertPermissionState(false, true);
        assertPermissionState(true, false);
        assertPermissionState(true, true);
    }

    @Test
    public void everyTrimEquivalentCrossOwnerCollisionFailsBeforeLookup() {
        String[] drivePaths = {
                "StarterProfile.drive.wiring.frontLeftName",
                "StarterProfile.drive.wiring.frontRightName",
                "StarterProfile.drive.wiring.backLeftName",
                "StarterProfile.drive.wiring.backRightName"
        };

        for (int index = 0; index < drivePaths.length; index++) {
            StarterProfile profile = readyProfile();
            setDriveName(profile, index, "  intake  ");
            StarterTestHardware.HardwareMapProbe hardwareMap =
                    new StarterTestHardware.HardwareMapProbe();
            TestStarterTeleOp mode = prepare(
                    new TestStarterTeleOp(profile),
                    hardwareMap,
                    new StarterTestHardware.TelemetryProbe(),
                    new Gamepad());

            IllegalStateException failure = expectIllegalState(mode::init);

            assertTrue(failure.getMessage().contains("StarterProfile.intake.motorName"));
            assertTrue(failure.getMessage().contains(drivePaths[index]));
            assertTrue(failure.getMessage().contains("\"intake\""));
            assertEquals(0, hardwareMap.lookupCalls());
        }
    }

    @Test
    public void caseDifferentMotorNamesRemainDistinctFtcKeys() {
        StarterProfile profile = readyProfile();
        profile.drive.wiring.frontLeftName = "Intake";
        StarterTestHardware.HardwareMapProbe hardwareMap = fullTeleOpHardware(profile);
        TestStarterTeleOp mode = prepare(
                new TestStarterTeleOp(profile),
                hardwareMap,
                new StarterTestHardware.TelemetryProbe(),
                new Gamepad());

        mode.init();

        assertEquals(5, hardwareMap.lookupCalls());
        mode.stop();
    }

    @Test
    public void activeIntakeOwnerSnapshotsItsConfigDuringInit() {
        StarterProfile profile = readyProfile();
        StarterTestHardware.HardwareMapProbe hardwareMap =
                new StarterTestHardware.HardwareMapProbe();
        StarterTestHardware.MotorProbe intakeMotor =
                hardwareMap.addMotor(profile.intake.motorName);
        TestStarterAuto mode = prepare(
                new TestStarterAuto(profile),
                hardwareMap,
                new StarterTestHardware.TelemetryProbe(),
                new Gamepad());

        mode.init();
        profile.intake.motorName = "changedAfterInit";
        profile.intake.direction = Direction.REVERSE;
        profile.intake.collectPower = -0.90;
        mode.start();

        assertEquals(1, hardwareMap.lookupCalls());
        assertEquals(0.65, intakeMotor.lastPower(), 0.0);
        mode.stop();
    }

    @Test
    public void invalidLaterDriveConfigCleansRegisteredIntakeAndPreservesFailure() {
        StarterProfile profile = readyProfile();
        profile.drive.drivebase.maxAxial = Double.NaN;
        StarterTestHardware.HardwareMapProbe hardwareMap =
                new StarterTestHardware.HardwareMapProbe();
        StarterTestHardware.MotorProbe intakeMotor =
                hardwareMap.addMotor(profile.intake.motorName);
        TestStarterTeleOp mode = prepare(
                new TestStarterTeleOp(profile),
                hardwareMap,
                new StarterTestHardware.TelemetryProbe(),
                new Gamepad());

        IllegalArgumentException failure = expectIllegalArgument(mode::init);

        assertEquals(
                "MecanumDrivebase.Config.maxAxial must be finite and in [0.0, 1.0], got NaN",
                failure.getMessage());
        assertEquals(1, hardwareMap.lookupCalls());
        assertEquals(0.0, intakeMotor.lastPower(), 0.0);
        assertEquals(1, intakeMotor.powerWrites());

        mode.loop();
        mode.stop();
        assertEquals(1, intakeMotor.powerWrites());
    }

    @Test
    public void missingLaterDriveHardwareCleansRegisteredIntakeAndPreservesFailure() {
        StarterProfile profile = readyProfile();
        profile.drive.wiring.frontLeftName = "missingFrontLeft";
        StarterTestHardware.HardwareMapProbe hardwareMap =
                new StarterTestHardware.HardwareMapProbe();
        StarterTestHardware.MotorProbe intakeMotor =
                hardwareMap.addMotor(profile.intake.motorName);
        TestStarterTeleOp mode = prepare(
                new TestStarterTeleOp(profile),
                hardwareMap,
                new StarterTestHardware.TelemetryProbe(),
                new Gamepad());

        IllegalArgumentException failure = expectIllegalArgument(mode::init);

        assertEquals("No test DcMotorEx named missingFrontLeft", failure.getMessage());
        assertEquals(2, hardwareMap.lookupCalls());
        assertEquals(0.0, intakeMotor.lastPower(), 0.0);
        assertEquals(1, intakeMotor.powerWrites());

        mode.loop();
        mode.stop();
        assertEquals(1, intakeMotor.powerWrites());
    }

    @Test
    public void teleOpDeclarationMapsControlsRealizesOutputsAndPresentsStatus() {
        StarterProfile profile = readyProfile();
        profile.intake.ejectPower = profile.intake.collectPower;
        List<String> events = new ArrayList<String>();
        StarterTestHardware.HardwareMapProbe hardwareMap =
                fullTeleOpHardware(profile, events);
        StarterTestHardware.TelemetryProbe telemetry =
                new StarterTestHardware.TelemetryProbe(events);
        Gamepad driver = new Gamepad();
        TestStarterTeleOp mode = prepare(
                new TestStarterTeleOp(profile),
                hardwareMap,
                telemetry,
                driver);

        mode.init();
        assertEquals(5, hardwareMap.lookupCalls());
        assertEquals(1, telemetry.updateCalls());
        assertEquals(
                Arrays.asList("intake.mode", "intake.appliedTargetPower"),
                telemetry.lastFrameKeys());
        assertEquals(2, telemetry.dataRowsAtLastUpdate());
        assertEquals(0.0,
                (Double) telemetry.dataValue("intake.appliedTargetPower"),
                0.0);

        mode.start();
        mode.loop();
        assertEquals(2, telemetry.updateCalls());
        assertEquals(2, telemetry.dataRowsAtLastUpdate());

        events.clear();
        driver.a = true;
        driver.left_stick_y = -1.0f;
        mode.loop();
        assertEquals(3, telemetry.updateCalls());
        assertEquals(2, telemetry.dataRowsAtLastUpdate());
        assertEquals(
                profile.intake.collectPower,
                (Double) telemetry.dataValue("intake.appliedTargetPower"),
                0.0);
        assertEquals(
                StarterIntake.Mode.COLLECT,
                telemetry.dataValue("intake.mode"));
        assertEquals(
                profile.intake.collectPower,
                hardwareMap.motor(profile.intake.motorName).lastPower(),
                0.0);
        assertEventBefore(events, "power:intake:", "power:frontLeft:");
        assertEventBefore(events, "power:frontLeft:", "telemetry.row:intake.mode");
        assertEventBefore(
                events,
                "telemetry.row:intake.appliedTargetPower",
                "telemetry.commit");

        events.clear();
        driver.a = false;
        driver.b = true;
        mode.loop();
        assertEquals(4, telemetry.updateCalls());
        assertEquals(
                StarterIntake.Mode.EJECT,
                telemetry.dataValue("intake.mode"));
        assertEquals(
                profile.intake.ejectPower,
                (Double) telemetry.dataValue("intake.appliedTargetPower"),
                0.0);
        assertEquals(
                profile.intake.ejectPower,
                hardwareMap.motor(profile.intake.motorName).lastPower(),
                0.0);

        mode.stop();
        assertAllMotorsStopped(hardwareMap, profile);
        int powerWritesAfterFirstStop = hardwareMap.totalPowerWrites();

        mode.stop();
        mode.loop();
        assertEquals(powerWritesAfterFirstStop, hardwareMap.totalPowerWrites());
        assertEquals(4, telemetry.updateCalls());
    }

    @Test
    public void activeLoopFailureFailStopsEveryConstructedOutput() {
        StarterProfile profile = readyProfile();
        StarterTestHardware.HardwareMapProbe hardwareMap = fullTeleOpHardware(profile);
        StarterTestHardware.TelemetryProbe telemetry =
                new StarterTestHardware.TelemetryProbe();
        TestStarterTeleOp mode = prepare(
                new TestStarterTeleOp(profile),
                hardwareMap,
                telemetry,
                new Gamepad());

        mode.init();
        mode.start();
        telemetry.failNextUpdate();

        IllegalStateException failure = expectIllegalState(mode::loop);

        assertTrue(failure.getMessage().contains("telemetry update failed"));
        assertAllMotorsStopped(hardwareMap, profile);
        int writesAfterFailure = hardwareMap.totalPowerWrites();
        mode.loop();
        assertEquals(writesAfterFailure, hardwareMap.totalPowerWrites());
    }

    @Test
    public void autoClientStartsOneFreshRootAndDeclarationPresentsOnlyCapabilityStatus() {
        StarterProfile profile = readyProfile();
        List<String> events = new ArrayList<String>();
        StarterTestHardware.HardwareMapProbe hardwareMap =
                new StarterTestHardware.HardwareMapProbe(events);
        StarterTestHardware.MotorProbe intakeMotor =
                hardwareMap.addMotor(profile.intake.motorName);
        StarterTestHardware.TelemetryProbe telemetry =
                new StarterTestHardware.TelemetryProbe(events);
        TestStarterAuto mode = prepare(
                new TestStarterAuto(profile),
                hardwareMap,
                telemetry,
                new Gamepad());

        mode.init();
        assertEquals(1, telemetry.updateCalls());
        assertEquals(
                Arrays.asList("intake.mode", "intake.appliedTargetPower"),
                telemetry.lastFrameKeys());
        assertEquals(2, telemetry.dataRowsAtLastUpdate());

        mode.start();
        assertEquals(profile.intake.collectPower, intakeMotor.lastPower(), 0.0);

        events.clear();
        mode.loop();
        assertEquals(2, telemetry.updateCalls());
        assertEquals(2, telemetry.dataRowsAtLastUpdate());
        assertEquals(
                profile.intake.collectPower,
                (Double) telemetry.dataValue("intake.appliedTargetPower"),
                0.0);
        assertEquals(
                StarterIntake.Mode.COLLECT,
                telemetry.dataValue("intake.mode"));
        assertEventBefore(events, "power:intake:", "telemetry.row:intake.mode");
        assertEventBefore(
                events,
                "telemetry.row:intake.appliedTargetPower",
                "telemetry.commit");

        mode.stop();
        assertEquals(0.0, intakeMotor.lastPower(), 0.0);
        int writesAfterFirstStop = intakeMotor.powerWrites();

        mode.stop();
        mode.loop();
        assertEquals(writesAfterFirstStop, intakeMotor.powerWrites());
        assertEquals(2, telemetry.updateCalls());
    }

    private static void assertPermissionState(boolean allowIntakeMotion,
                                              boolean allowDriveMotion) {
        StarterProfile autoProfile = readyProfile();
        autoProfile.allowIntakeMotion = allowIntakeMotion;
        autoProfile.allowDriveMotion = allowDriveMotion;
        autoProfile.drive = null;
        StarterTestHardware.HardwareMapProbe autoMap =
                new StarterTestHardware.HardwareMapProbe();
        autoMap.addMotor(autoProfile.intake.motorName);
        TestStarterAuto autoMode = prepare(
                new TestStarterAuto(autoProfile),
                autoMap,
                new StarterTestHardware.TelemetryProbe(),
                new Gamepad());

        if (allowIntakeMotion) {
            autoMode.init();
            assertEquals(1, autoMap.lookupCalls());
            autoMode.stop();
        } else {
            IllegalStateException failure = expectIllegalState(autoMode::init);
            assertTrue(failure.getMessage().contains("StarterProfile.allowIntakeMotion"));
            assertEquals(0, autoMap.lookupCalls());
        }

        StarterProfile teleOpProfile = readyProfile();
        teleOpProfile.allowIntakeMotion = allowIntakeMotion;
        teleOpProfile.allowDriveMotion = allowDriveMotion;
        StarterTestHardware.HardwareMapProbe teleOpMap = allowIntakeMotion && allowDriveMotion
                ? fullTeleOpHardware(teleOpProfile)
                : new StarterTestHardware.HardwareMapProbe();
        TestStarterTeleOp teleOpMode = prepare(
                new TestStarterTeleOp(teleOpProfile),
                teleOpMap,
                new StarterTestHardware.TelemetryProbe(),
                new Gamepad());

        if (allowIntakeMotion && allowDriveMotion) {
            teleOpMode.init();
            assertEquals(5, teleOpMap.lookupCalls());
            teleOpMode.stop();
        } else {
            IllegalStateException failure = expectIllegalState(teleOpMode::init);
            String blockedPermission = allowIntakeMotion
                    ? "StarterProfile.allowDriveMotion"
                    : "StarterProfile.allowIntakeMotion";
            assertTrue(failure.getMessage().contains(blockedPermission));
            assertEquals(0, teleOpMap.lookupCalls());
        }
    }

    private static StarterProfile readyProfile() {
        StarterProfile profile = StarterProfile.current();
        profile.intake.motorName = "intake";
        profile.intake.direction = Direction.FORWARD;
        profile.intake.collectPower = 0.65;
        profile.intake.ejectPower = -0.45;
        profile.allowIntakeMotion = true;

        profile.drive.wiring.frontLeftName = "frontLeft";
        profile.drive.wiring.frontRightName = "frontRight";
        profile.drive.wiring.backLeftName = "backLeft";
        profile.drive.wiring.backRightName = "backRight";
        profile.drive.wiring.frontLeftDirection = Direction.FORWARD;
        profile.drive.wiring.frontRightDirection = Direction.REVERSE;
        profile.drive.wiring.backLeftDirection = Direction.FORWARD;
        profile.drive.wiring.backRightDirection = Direction.REVERSE;
        profile.allowDriveMotion = true;
        return profile;
    }

    private static void setDriveName(StarterProfile profile, int index, String name) {
        switch (index) {
            case 0:
                profile.drive.wiring.frontLeftName = name;
                return;
            case 1:
                profile.drive.wiring.frontRightName = name;
                return;
            case 2:
                profile.drive.wiring.backLeftName = name;
                return;
            case 3:
                profile.drive.wiring.backRightName = name;
                return;
            default:
                throw new AssertionError("unsupported drive motor index " + index);
        }
    }

    private static void assertOnlyPublicNoArgConstructor(Class<?> type) {
        assertTrue(Modifier.isPublic(type.getModifiers()));
        assertTrue(Modifier.isFinal(type.getModifiers()));
        Constructor<?>[] constructors = type.getDeclaredConstructors();
        assertEquals(1, constructors.length);
        assertTrue(Modifier.isPublic(constructors[0].getModifiers()));
        assertEquals(0, constructors[0].getParameterTypes().length);
    }

    private static void assertNoRetainedProfile(Class<?> type) {
        for (Field field : type.getDeclaredFields()) {
            assertTrue(
                    type.getSimpleName() + " must not retain StarterProfile",
                    field.getType() != StarterProfile.class);
        }
    }

    private static int declaredMethodCount(Class<?> type, String name) {
        int count = 0;
        for (Method method : type.getDeclaredMethods()) {
            if (method.getName().equals(name)) {
                count++;
            }
        }
        return count;
    }

    private static void assertEventBefore(List<String> events,
                                          String earlierPrefix,
                                          String laterPrefix) {
        int earlier = eventIndex(events, earlierPrefix);
        int later = eventIndex(events, laterPrefix);
        assertTrue("Missing event prefix " + earlierPrefix + " in " + events, earlier >= 0);
        assertTrue("Missing event prefix " + laterPrefix + " in " + events, later >= 0);
        assertTrue("Expected " + earlierPrefix + " before " + laterPrefix + " in " + events,
                earlier < later);
    }

    private static int eventIndex(List<String> events, String prefix) {
        for (int index = 0; index < events.size(); index++) {
            if (events.get(index).startsWith(prefix)) {
                return index;
            }
        }
        return -1;
    }

    private static void assertAllMotorsStopped(
            StarterTestHardware.HardwareMapProbe hardwareMap,
            StarterProfile profile) {
        assertEquals(0.0, hardwareMap.motor(profile.intake.motorName).lastPower(), 0.0);
        assertEquals(0.0,
                hardwareMap.motor(profile.drive.wiring.frontLeftName).lastPower(),
                0.0);
        assertEquals(0.0,
                hardwareMap.motor(profile.drive.wiring.frontRightName).lastPower(),
                0.0);
        assertEquals(0.0,
                hardwareMap.motor(profile.drive.wiring.backLeftName).lastPower(),
                0.0);
        assertEquals(0.0,
                hardwareMap.motor(profile.drive.wiring.backRightName).lastPower(),
                0.0);
    }

    private static IllegalStateException expectIllegalState(Runnable action) {
        try {
            action.run();
            fail("Expected IllegalStateException");
            throw new AssertionError("unreachable");
        } catch (IllegalStateException expected) {
            return expected;
        }
    }

    private static IllegalArgumentException expectIllegalArgument(Runnable action) {
        try {
            action.run();
            fail("Expected IllegalArgumentException");
            throw new AssertionError("unreachable");
        } catch (IllegalArgumentException expected) {
            return expected;
        }
    }

    /** Test-only host that synchronously supplies a selected TeleOp profile during INIT. */
    private static final class TestStarterTeleOp extends FtcRobotOpMode {
        private final StarterProfile profile;

        private TestStarterTeleOp(StarterProfile profile) {
            this.profile = profile;
        }

        @Override
        protected void configure(RobotProgram program) {
            new StarterRobot(hardwareMap).declareTeleOp(program, profile, gamepad1);
        }
    }

    /** Test-only host that synchronously supplies a selected Auto profile during INIT. */
    private static final class TestStarterAuto extends FtcRobotOpMode {
        private final StarterProfile profile;

        private TestStarterAuto(StarterProfile profile) {
            this.profile = profile;
        }

        @Override
        protected void configure(RobotProgram program) {
            StarterIntake intake = new StarterRobot(hardwareMap).declareAuto(program, profile);
            program.rootTask(intake.collectForSeconds(0.75));
        }
    }
}
