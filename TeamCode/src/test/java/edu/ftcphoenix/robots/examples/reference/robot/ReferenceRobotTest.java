package edu.ftcphoenix.robots.examples.reference.robot;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;

import edu.ftcphoenix.fw.ftc.FtcRobotOpMode;
import edu.ftcphoenix.fw.ftc.RobotProgram;
import edu.ftcphoenix.robots.examples.reference.autonomous.ReferenceAutoRoutines;
import edu.ftcphoenix.robots.examples.reference.opmode.ReferenceAuto;
import edu.ftcphoenix.fw.testing.ftc.FtcTestHardware;
import edu.ftcphoenix.robots.examples.starter.support.StarterTestHardware;

import static edu.ftcphoenix.robots.examples.starter.support.StarterTestHardware.prepare;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies Reference composition surfaces, active-owner preflight, and Auto handoff. */
public final class ReferenceRobotTest {

    @Test
    public void rootReturnsCapabilitiesButOwnsNoAutoRoutine() throws Exception {
        Method teleOp = ReferenceRobot.class.getDeclaredMethod(
                "declareTeleOp", RobotProgram.class, ReferenceProfile.class, Gamepad.class);
        assertEquals(Void.TYPE, teleOp.getReturnType());

        Method auto = ReferenceRobot.class.getDeclaredMethod(
                "declareAuto", RobotProgram.class, ReferenceProfile.class);
        assertEquals(ReferenceCapabilities.class, auto.getReturnType());

        Constructor<?> constructor = ReferenceCapabilities.class.getDeclaredConstructors()[0];
        assertFalse(Modifier.isPublic(constructor.getModifiers()));
        assertEquals(2, ReferenceCapabilities.class.getDeclaredFields().length);
        assertOnlyPublicNoArgConstructor(ReferenceAuto.class);
    }

    @Test
    public void everyTeleOpMotorCollisionFailsBeforeLookupWithBothProfilePaths() {
        String[] paths = teleOpPaths();
        for (int second = 1; second < paths.length; second++) {
            for (int first = 0; first < second; first++) {
                ReferenceProfile profile = readyProfile();
                String key = motorName(profile, first);
                setMotorName(profile, second, "  " + key + "  ");
                FtcTestHardware hardware =
                        new FtcTestHardware();
                TestReferenceTeleOp mode = prepare(
                        new TestReferenceTeleOp(profile),
                        hardware,
                        new StarterTestHardware.TelemetryProbe(),
                        new Gamepad());

                IllegalStateException failure = expectIllegalState(mode::init);

                assertTrue(failure.getMessage().contains(paths[first]));
                assertTrue(failure.getMessage().contains(paths[second]));
                assertTrue(failure.getMessage().contains("\"" + key + "\""));
                assertEquals(0, hardware.lookupCalls());
            }
        }
    }

    @Test
    public void everyAutoMotorCollisionFailsBeforeLookupButUnusedDriveIsIgnored() {
        String[] paths = autoPaths();
        for (int second = 5; second < 7; second++) {
            for (int first = 4; first < second; first++) {
                ReferenceProfile profile = readyProfile();
                String key = motorName(profile, first);
                setMotorName(profile, second, " " + key + " ");
                FtcTestHardware hardware =
                        new FtcTestHardware();
                TestReferenceAuto mode = prepare(
                        new TestReferenceAuto(profile),
                        hardware,
                        new StarterTestHardware.TelemetryProbe(),
                        new Gamepad());

                IllegalStateException failure = expectIllegalState(mode::init);

                assertTrue(failure.getMessage().contains(paths[first - 4]));
                assertTrue(failure.getMessage().contains(paths[second - 4]));
                assertTrue(failure.getMessage().contains("\"" + key + "\""));
                assertEquals(0, hardware.lookupCalls());
            }
        }

        ReferenceProfile profile = readyProfile();
        profile.drive = null;
        profile.allowDriveMotion = false;
        FtcTestHardware hardware = fullAutoHardware(profile);
        TestReferenceAuto mode = prepare(
                new TestReferenceAuto(profile),
                hardware,
                new StarterTestHardware.TelemetryProbe(),
                new Gamepad());

        mode.init();
        assertTrue(hardware.lookupCalls() > 0);
        mode.stop();
    }

    @Test
    public void caseDifferentNamesRemainDistinctFtcKeys() {
        ReferenceProfile profile = readyProfile();
        profile.lift.motorName = "Shared";
        profile.launcher.leftFlywheelName = "shared";
        FtcTestHardware hardware = fullAutoHardware(profile);
        TestReferenceAuto mode = prepare(
                new TestReferenceAuto(profile),
                hardware,
                new StarterTestHardware.TelemetryProbe(),
                new Gamepad());

        mode.init();
        mode.stop();
    }

    @Test
    public void autoDeclarationDoesNotHideACompositionRootControlScript() {
        ReferenceProfile profile = readyProfile();
        FtcTestHardware hardware = fullAutoHardware(profile);
        TestReferenceDeclarationOnlyAuto mode = prepare(
                new TestReferenceDeclarationOnlyAuto(profile),
                hardware,
                new StarterTestHardware.TelemetryProbe(),
                new Gamepad());

        mode.init();
        mode.start();

        assertEquals(0.0, hardware.motor(profile.lift.motorName).power(), 0.0);
        mode.stop();
    }

    @Test
    public void laterLauncherConstructionFailureCleansTheRegisteredLiftOwner() {
        ReferenceProfile profile = readyProfile();
        FtcTestHardware hardware =
                new FtcTestHardware();
        hardware.addDigitalInput(profile.lift.bottomSwitchName);
        FtcTestHardware.MotorProbe liftMotor =
                hardware.addMotor(profile.lift.motorName);
        hardware.addMotor(profile.launcher.leftFlywheelName);
        hardware.addMotor(profile.launcher.rightFlywheelName);
        TestReferenceAuto mode = prepare(
                new TestReferenceAuto(profile),
                hardware,
                new StarterTestHardware.TelemetryProbe(),
                new Gamepad());

        try {
            mode.init();
            fail("Expected missing launcher hardware");
        } catch (IllegalArgumentException expected) {
            assertEquals(
                    "No test DigitalChannel named '" + profile.launcher.objectSensorName + "'",
                    expected.getMessage());
        }

        assertTrue(liftMotor.powerWrites() > 0);
        assertEquals(0.0, liftMotor.power(), 0.0);
        int writesAfterFailure = liftMotor.powerWrites();
        mode.stop();
        mode.loop();
        assertEquals(writesAfterFailure, liftMotor.powerWrites());
    }

    @Test
    public void nullTeleOpGamepadFailsBeforePermissionsCollisionOrHardwareLookup() {
        ReferenceProfile profile = readyProfile();
        profile.allowDriveMotion = false;
        FtcTestHardware hardware =
                new FtcTestHardware();
        TestReferenceTeleOp mode = prepare(
                new TestReferenceTeleOp(profile),
                hardware,
                new StarterTestHardware.TelemetryProbe(),
                null);

        try {
            mode.init();
            fail("Expected null gamepad1");
        } catch (NullPointerException expected) {
            assertTrue(expected.getMessage().contains("gamepad1"));
        }
        assertEquals(0, hardware.lookupCalls());
    }

    private static ReferenceProfile readyProfile() {
        ReferenceProfile profile = ReferenceProfile.current();
        profile.allowDriveMotion = true;
        profile.allowLiftMotion = true;
        profile.allowLauncherMotion = true;
        return profile;
    }

    private static FtcTestHardware fullAutoHardware(
            ReferenceProfile profile) {
        FtcTestHardware hardware =
                new FtcTestHardware();
        hardware.addDigitalInput(profile.lift.bottomSwitchName);
        hardware.addMotor(profile.lift.motorName);
        hardware.addDigitalInput(profile.launcher.objectSensorName);
        hardware.addMotor(profile.launcher.leftFlywheelName);
        hardware.addMotor(profile.launcher.rightFlywheelName);
        hardware.addCrServo(profile.launcher.transferName);
        hardware.addServo(profile.launcher.releaseServoName);
        return hardware;
    }

    private static String[] teleOpPaths() {
        return new String[]{
                "ReferenceProfile.drive.wiring.frontLeftName",
                "ReferenceProfile.drive.wiring.frontRightName",
                "ReferenceProfile.drive.wiring.backLeftName",
                "ReferenceProfile.drive.wiring.backRightName",
                "ReferenceProfile.lift.motorName",
                "ReferenceProfile.launcher.leftFlywheelName",
                "ReferenceProfile.launcher.rightFlywheelName"
        };
    }

    private static String[] autoPaths() {
        return new String[]{
                "ReferenceProfile.lift.motorName",
                "ReferenceProfile.launcher.leftFlywheelName",
                "ReferenceProfile.launcher.rightFlywheelName"
        };
    }

    private static String motorName(ReferenceProfile profile, int index) {
        switch (index) {
            case 0:
                return profile.drive.wiring.frontLeftName;
            case 1:
                return profile.drive.wiring.frontRightName;
            case 2:
                return profile.drive.wiring.backLeftName;
            case 3:
                return profile.drive.wiring.backRightName;
            case 4:
                return profile.lift.motorName;
            case 5:
                return profile.launcher.leftFlywheelName;
            case 6:
                return profile.launcher.rightFlywheelName;
            default:
                throw new AssertionError("unsupported motor index " + index);
        }
    }

    private static void setMotorName(ReferenceProfile profile, int index, String name) {
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
            case 4:
                profile.lift.motorName = name;
                return;
            case 5:
                profile.launcher.leftFlywheelName = name;
                return;
            case 6:
                profile.launcher.rightFlywheelName = name;
                return;
            default:
                throw new AssertionError("unsupported motor index " + index);
        }
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

    private static void assertOnlyPublicNoArgConstructor(Class<?> type) {
        Constructor<?>[] constructors = type.getDeclaredConstructors();
        assertEquals(1, constructors.length);
        assertTrue(Modifier.isPublic(constructors[0].getModifiers()));
        assertEquals(0, constructors[0].getParameterTypes().length);
    }

    private static final class TestReferenceTeleOp extends FtcRobotOpMode {
        private final ReferenceProfile profile;

        private TestReferenceTeleOp(ReferenceProfile profile) {
            this.profile = profile;
        }

        @Override
        protected void configure(RobotProgram program) {
            new ReferenceRobot(hardwareMap).declareTeleOp(program, profile, gamepad1);
        }
    }

    private static final class TestReferenceAuto extends FtcRobotOpMode {
        private final ReferenceProfile profile;

        private TestReferenceAuto(ReferenceProfile profile) {
            this.profile = profile;
        }

        @Override
        protected void configure(RobotProgram program) {
            ReferenceCapabilities capabilities =
                    new ReferenceRobot(hardwareMap).declareAuto(program, profile);
            program.rootTask(ReferenceAutoRoutines.homeMoveLowThenLaunch(capabilities));
        }
    }

    private static final class TestReferenceDeclarationOnlyAuto extends FtcRobotOpMode {
        private final ReferenceProfile profile;

        private TestReferenceDeclarationOnlyAuto(ReferenceProfile profile) {
            this.profile = profile;
        }

        @Override
        protected void configure(RobotProgram program) {
            new ReferenceRobot(hardwareMap).declareAuto(program, profile);
        }
    }
}
