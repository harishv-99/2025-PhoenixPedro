package edu.ftcphoenix.robots.examples.pedro.robot;

import edu.ftcphoenix.fw.testing.ftc.FtcTestHardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;

import java.lang.reflect.Proxy;

import edu.ftcphoenix.fw.ftc.FtcRobotOpMode;
import edu.ftcphoenix.fw.ftc.RobotProgram;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies Basic Pedro permission and cross-owner policy before any hardware effect. */
public final class BasicPedroAutoConfigurationTest {

    private static final String[] DRIVE_NAME_PATHS = {
            "BasicPedroProfile.pedro.mecanumConstants.leftFrontMotorName",
            "BasicPedroProfile.pedro.mecanumConstants.leftRearMotorName",
            "BasicPedroProfile.pedro.mecanumConstants.rightFrontMotorName",
            "BasicPedroProfile.pedro.mecanumConstants.rightRearMotorName"
    };

    @Test
    public void permissionBlocksMalformedDraftBeforeInspectingItOrTouchingHardware() {
        BasicPedroProfile profile = BasicPedroProfile.current();
        profile.pedro = null;
        profile.intake = null;
        FtcTestHardware hardwareMap = hardwareWithIntake();

        RuntimeException failure = expectInitFailure(profile, hardwareMap);

        assertTrue(failure.getMessage().contains("BasicPedroProfile.allowRobotMotion"));
        assertEquals(0, hardwareMap.lookupCalls());
        assertEquals(0, hardwareMap.totalMotorPowerWrites());
    }

    @Test
    public void nullActiveConfigObjectsFailBeforeRuntimeEffects() {
        BasicPedroProfile missingPedro = readyProfile();
        missingPedro.pedro = null;
        FtcTestHardware firstHardware = hardwareWithIntake();
        RuntimeException pedroFailure = expectInitFailure(missingPedro, firstHardware);
        assertTrue(pedroFailure.getMessage().contains("BasicPedroProfile.pedro"));
        assertEquals(0, firstHardware.lookupCalls());
        assertEquals(0, firstHardware.totalMotorPowerWrites());

        BasicPedroProfile missingIntake = readyProfile();
        missingIntake.intake = null;
        FtcTestHardware secondHardware = hardwareWithIntake();
        RuntimeException intakeFailure = expectInitFailure(missingIntake, secondHardware);
        assertTrue(intakeFailure.getMessage().contains("BasicPedroProfile.intake"));
        assertEquals(0, secondHardware.lookupCalls());
        assertEquals(0, secondHardware.totalMotorPowerWrites());
    }

    @Test
    public void everyExactAndTrimEquivalentCrossOwnerCollisionFailsBeforeEffects() {
        for (int index = 0; index < DRIVE_NAME_PATHS.length; index++) {
            assertCollision(index, "intakeMotor", "intakeMotor", "intakeMotor");
            assertCollision(index, "intakeMotor", "  intakeMotor  ", "intakeMotor");
            assertCollision(index, "  intakeMotor  ", "intakeMotor", "intakeMotor");
        }
    }

    @Test
    public void everyCaseDifferentDriveNamePassesCollisionPolicy() {
        for (int index = 0; index < DRIVE_NAME_PATHS.length; index++) {
            BasicPedroProfile profile = readyProfile();
            setDriveName(profile, index, "IntakeMotor");
            profile.pedro.pathConstraints = null;
            FtcTestHardware hardwareMap = hardwareWithIntake();

            RuntimeException failure = expectInitFailure(profile, hardwareMap);

            assertTrue(failure.getMessage().contains("pathConstraints"));
            assertEquals(0, hardwareMap.lookupCalls());
            assertEquals(0, hardwareMap.totalMotorPowerWrites());
        }
    }

    @Test
    public void runtimeInvalidDraftFailsBeforeLookupAndLeavesPedroGlobalUntouched() {
        BasicPedroProfile profile = readyProfile();
        profile.pedro.mecanumConstants.maxPower = Double.NaN;
        FtcTestHardware hardwareMap = hardwareWithIntake();
        com.pedropathing.paths.PathConstraints original =
                com.pedropathing.paths.PathConstraints.defaultConstraints;
        com.pedropathing.paths.PathConstraints poison =
                new com.pedropathing.paths.PathConstraints(
                        0.21, 22.0, 23.0, 0.24, 25.0, 2.6, 27, 2.8);
        com.pedropathing.paths.PathConstraints.defaultConstraints = poison;
        try {
            RuntimeException failure = expectInitFailure(profile, hardwareMap);

            assertTrue(failure.getMessage().contains("mecanumConstants.maxPower"));
            assertEquals(0, hardwareMap.lookupCalls());
            assertEquals(0, hardwareMap.totalMotorPowerWrites());
            assertSame(poison, com.pedropathing.paths.PathConstraints.defaultConstraints);
        } finally {
            com.pedropathing.paths.PathConstraints.defaultConstraints = original;
        }
    }

    private static void assertCollision(int driveIndex,
                                        String configuredIntakeName,
                                        String configuredDriveName,
                                        String effectiveName) {
        BasicPedroProfile profile = readyProfile();
        profile.intake.motorName = configuredIntakeName;
        setDriveName(profile, driveIndex, configuredDriveName);
        FtcTestHardware hardwareMap = hardwareWithIntake();

        RuntimeException failure = expectInitFailure(profile, hardwareMap);

        assertTrue(failure.getMessage().contains("BasicPedroProfile.intake.motorName"));
        assertTrue(failure.getMessage().contains(DRIVE_NAME_PATHS[driveIndex]));
        assertTrue(failure.getMessage().contains("\"" + effectiveName + "\""));
        assertEquals(0, hardwareMap.lookupCalls());
        assertEquals(0, hardwareMap.totalMotorPowerWrites());
    }

    private static BasicPedroProfile readyProfile() {
        BasicPedroProfile profile = BasicPedroProfile.current();
        profile.allowRobotMotion = true;
        return profile;
    }

    private static FtcTestHardware hardwareWithIntake() {
        FtcTestHardware hardwareMap =
                new FtcTestHardware();
        hardwareMap.addMotor("intakeMotor");
        return hardwareMap;
    }

    private static void setDriveName(BasicPedroProfile profile, int index, String value) {
        if (index == 0) {
            profile.pedro.mecanumConstants.leftFrontMotorName = value;
        } else if (index == 1) {
            profile.pedro.mecanumConstants.leftRearMotorName = value;
        } else if (index == 2) {
            profile.pedro.mecanumConstants.rightFrontMotorName = value;
        } else if (index == 3) {
            profile.pedro.mecanumConstants.rightRearMotorName = value;
        } else {
            throw new AssertionError("unsupported drive motor index " + index);
        }
    }

    private static RuntimeException expectInitFailure(
            BasicPedroProfile profile,
            FtcTestHardware hardwareMap) {
        ProductionDeclarationMode mode = new ProductionDeclarationMode(profile);
        mode.hardwareMap = hardwareMap;
        mode.telemetry = inertTelemetry();
        try {
            mode.init();
            fail("expected Basic Pedro production declaration to fail");
            throw new AssertionError("unreachable");
        } catch (RuntimeException expected) {
            return expected;
        }
    }

    private static final class ProductionDeclarationMode extends FtcRobotOpMode {
        private final BasicPedroProfile profile;

        private ProductionDeclarationMode(BasicPedroProfile profile) {
            this.profile = profile;
        }

        @Override
        protected void configure(RobotProgram program) {
            new BasicPedroAutoRobot(program, hardwareMap, profile);
        }
    }

    private static Telemetry inertTelemetry() {
        return (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                (proxy, method, args) -> defaultValue(method.getReturnType()));
    }

    private static Object defaultValue(Class<?> returnType) {
        if (returnType == boolean.class) {
            return true;
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
