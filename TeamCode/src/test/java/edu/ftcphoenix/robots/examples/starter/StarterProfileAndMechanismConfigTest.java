package edu.ftcphoenix.robots.examples.starter;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.ftc.FtcDrives;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Locks the Starter profile surface and the intake owner's Config/effect boundary. */
public final class StarterProfileAndMechanismConfigTest {

    @Test
    public void profileHasOnlyTheFourApprovedFieldsAndCurrentFactory() throws Exception {
        assertTrue(Modifier.isPublic(StarterProfile.class.getModifiers()));
        assertTrue(Modifier.isFinal(StarterProfile.class.getModifiers()));

        Constructor<?>[] constructors = StarterProfile.class.getDeclaredConstructors();
        assertEquals(1, constructors.length);
        assertEquals(0, constructors[0].getParameterTypes().length);
        assertTrue(Modifier.isPrivate(constructors[0].getModifiers()));

        Field[] fields = StarterProfile.class.getDeclaredFields();
        assertEquals(4, fields.length);
        assertPublicField(fields[0], "intake", StarterIntakeMechanism.Config.class);
        assertPublicField(fields[1], "allowIntakeMotion", boolean.class);
        assertPublicField(fields[2], "drive", FtcDrives.MecanumConfig.class);
        assertPublicField(fields[3], "allowDriveMotion", boolean.class);

        Method[] methods = StarterProfile.class.getDeclaredMethods();
        assertEquals(1, methods.length);
        Method current = StarterProfile.class.getDeclaredMethod("current");
        assertTrue(Modifier.isPublic(current.getModifiers()));
        assertTrue(Modifier.isStatic(current.getModifiers()));
        assertEquals(StarterProfile.class, current.getReturnType());
        assertEquals(0, current.getParameterTypes().length);
    }

    @Test
    public void currentReturnsIndependentCompleteConservativeExamples() {
        StarterProfile first = StarterProfile.current();
        StarterProfile second = StarterProfile.current();

        assertNotSame(first, second);
        assertNotSame(first.intake, second.intake);
        assertNotSame(first.drive, second.drive);
        assertNotSame(first.drive.wiring, second.drive.wiring);
        assertNotSame(first.drive.drivebase, second.drive.drivebase);
        assertCurrentValues(first);
        assertCurrentValues(second);

        first.intake.motorName = "changedIntake";
        first.intake.direction = Direction.REVERSE;
        first.intake.collectPower = 0.75;
        first.intake.ejectPower = -0.55;
        first.allowIntakeMotion = true;
        first.drive.wiring.frontLeftName = "changedFrontLeft";
        first.drive.wiring.frontRightDirection = Direction.FORWARD;
        first.drive.enableZeroPowerBrake = false;
        first.drive.drivebase.maxAxial = 0.90;
        first.allowDriveMotion = true;

        assertCurrentValues(second);
    }

    @Test
    public void mechanismAndConfigExposeOnlyTheApprovedConstructionPaths() throws Exception {
        assertTrue(Modifier.isPublic(StarterIntakeMechanism.class.getModifiers()));
        assertTrue(Modifier.isFinal(StarterIntakeMechanism.class.getModifiers()));

        Constructor<?>[] mechanismConstructors =
                StarterIntakeMechanism.class.getDeclaredConstructors();
        assertEquals(2, mechanismConstructors.length);
        Constructor<?> ordinary = StarterIntakeMechanism.class.getDeclaredConstructor(
                HardwareMap.class,
                StarterIntakeMechanism.Config.class);
        assertTrue(Modifier.isPublic(ordinary.getModifiers()));
        Constructor<?> neutral = StarterIntakeMechanism.class.getDeclaredConstructor(Plant.class);
        assertTrue(isPackagePrivate(neutral.getModifiers()));

        Class<?> configType = StarterIntakeMechanism.Config.class;
        int configModifiers = configType.getModifiers();
        assertTrue(Modifier.isPublic(configModifiers));
        assertTrue(Modifier.isStatic(configModifiers));
        assertTrue(Modifier.isFinal(configModifiers));
        Constructor<?>[] configConstructors = configType.getDeclaredConstructors();
        assertEquals(1, configConstructors.length);
        assertEquals(0, configConstructors[0].getParameterTypes().length);
        assertTrue(Modifier.isPrivate(configConstructors[0].getModifiers()));

        Field[] fields = configType.getDeclaredFields();
        assertEquals(4, fields.length);
        assertPublicField(fields[0], "motorName", String.class);
        assertPublicField(fields[1], "direction", Direction.class);
        assertPublicField(fields[2], "collectPower", double.class);
        assertPublicField(fields[3], "ejectPower", double.class);

        Set<String> publicMethods = new HashSet<String>();
        int publicMethodCount = 0;
        for (Method method : configType.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers())) {
                publicMethodCount++;
                publicMethods.add(method.getName());
            }
        }
        assertEquals(1, publicMethodCount);
        assertEquals(new HashSet<String>(Arrays.asList("defaults")), publicMethods);
        Method defaults = configType.getDeclaredMethod("defaults");
        assertTrue(Modifier.isStatic(defaults.getModifiers()));
        assertEquals(configType, defaults.getReturnType());
        assertEquals(0, defaults.getParameterTypes().length);
    }

    @Test
    public void configDefaultsAreFreshCompleteAndSoftwareValid() {
        StarterIntakeMechanism.Config first = StarterIntakeMechanism.Config.defaults();
        StarterIntakeMechanism.Config second = StarterIntakeMechanism.Config.defaults();

        assertNotSame(first, second);
        assertEquals("intakeMotor", first.motorName);
        assertEquals(Direction.FORWARD, first.direction);
        assertEquals(0.20, first.collectPower, 0.0);
        assertEquals(-0.20, first.ejectPower, 0.0);

        first.motorName = "changed";
        first.direction = Direction.REVERSE;
        first.collectPower = 0.70;
        first.ejectPower = -0.60;

        assertEquals("intakeMotor", second.motorName);
        assertEquals(Direction.FORWARD, second.direction);
        assertEquals(0.20, second.collectPower, 0.0);
        assertEquals(-0.20, second.ejectPower, 0.0);
    }

    @Test
    public void ordinaryConstructorCapturesEveryFieldBeforeHardwareEffects() {
        StarterIntakeMechanism.Config config = StarterIntakeMechanism.Config.defaults();
        config.motorName = "  customIntake  ";
        config.direction = Direction.REVERSE;
        config.collectPower = 0.65;
        config.ejectPower = -0.45;
        StarterTestHardware.HardwareMapProbe hardwareMap =
                new StarterTestHardware.HardwareMapProbe();
        StarterTestHardware.MotorProbe motor = hardwareMap.addMotor("customIntake");

        StarterIntakeMechanism intake = new StarterIntakeMechanism(hardwareMap, config);

        assertEquals(1, hardwareMap.lookupCalls());
        assertEquals(0, hardwareMap.totalPowerWrites());
        assertEquals(DcMotorSimple.Direction.REVERSE, motor.direction());

        config.motorName = "missingAfterConstruction";
        config.direction = Direction.FORWARD;
        config.collectPower = 0.95;
        config.ejectPower = -0.85;

        ManualLoopClock time = new ManualLoopClock();
        intake.setMode(StarterIntake.Mode.COLLECT);
        intake.update(time.clock());
        assertEquals(0.65, motor.lastPower(), 0.0);
        assertEquals(DcMotorSimple.Direction.REVERSE, motor.direction());

        intake.setMode(StarterIntake.Mode.EJECT);
        intake.update(time.nextCycle(0.02));
        assertEquals(-0.45, motor.lastPower(), 0.0);
        assertEquals(1, hardwareMap.lookupCalls());
    }

    @Test
    public void ownerRejectsEveryInvalidFieldBeforeHardwareLookup() {
        assertInvalid(null, "StarterIntakeMechanism.Config is required");

        StarterIntakeMechanism.Config config = StarterIntakeMechanism.Config.defaults();
        config.motorName = null;
        assertInvalid(config, "StarterIntakeMechanism.Config.motorName", "got null");
        config = StarterIntakeMechanism.Config.defaults();
        config.motorName = "";
        assertInvalid(config, "StarterIntakeMechanism.Config.motorName", "non-blank");
        config = StarterIntakeMechanism.Config.defaults();
        config.motorName = " \t ";
        assertInvalid(config, "StarterIntakeMechanism.Config.motorName", "non-blank");

        config = StarterIntakeMechanism.Config.defaults();
        config.direction = null;
        assertInvalid(config, "StarterIntakeMechanism.Config.direction", "got null");

        assertInvalidCollectPower(Double.NaN);
        assertInvalidCollectPower(Double.POSITIVE_INFINITY);
        assertInvalidCollectPower(Double.NEGATIVE_INFINITY);
        assertInvalidCollectPower(0.0);
        assertInvalidCollectPower(-0.0);
        assertInvalidCollectPower(1.01);
        assertInvalidCollectPower(-1.01);

        assertInvalidEjectPower(Double.NaN);
        assertInvalidEjectPower(Double.POSITIVE_INFINITY);
        assertInvalidEjectPower(Double.NEGATIVE_INFINITY);
        assertInvalidEjectPower(0.0);
        assertInvalidEjectPower(-0.0);
        assertInvalidEjectPower(1.01);
        assertInvalidEjectPower(-1.01);

        config = StarterIntakeMechanism.Config.defaults();
        config.ejectPower = config.collectPower;
        assertInvalid(
                config,
                "StarterIntakeMechanism.Config.collectPower",
                "StarterIntakeMechanism.Config.ejectPower",
                "must be different",
                "got 0.2 and 0.2");
    }

    @Test
    public void actionPowerBoundariesAndEitherSignRemainValid() {
        assertAcceptedPowers(1.0, -1.0);
        assertAcceptedPowers(-1.0, 1.0);
        assertAcceptedPowers(0.01, 0.02);
        assertAcceptedPowers(-0.01, -0.02);
    }

    private static void assertCurrentValues(StarterProfile profile) {
        assertEquals("intakeMotor", profile.intake.motorName);
        assertEquals(Direction.FORWARD, profile.intake.direction);
        assertEquals(0.20, profile.intake.collectPower, 0.0);
        assertEquals(-0.20, profile.intake.ejectPower, 0.0);
        assertFalse(profile.allowIntakeMotion);

        assertEquals("frontLeftMotor", profile.drive.wiring.frontLeftName);
        assertEquals("frontRightMotor", profile.drive.wiring.frontRightName);
        assertEquals("backLeftMotor", profile.drive.wiring.backLeftName);
        assertEquals("backRightMotor", profile.drive.wiring.backRightName);
        assertEquals(Direction.FORWARD, profile.drive.wiring.frontLeftDirection);
        assertEquals(Direction.REVERSE, profile.drive.wiring.frontRightDirection);
        assertEquals(Direction.FORWARD, profile.drive.wiring.backLeftDirection);
        assertEquals(Direction.REVERSE, profile.drive.wiring.backRightDirection);
        assertTrue(profile.drive.enableZeroPowerBrake);
        assertEquals(0.25, profile.drive.drivebase.maxAxial, 0.0);
        assertEquals(0.25, profile.drive.drivebase.maxLateral, 0.0);
        assertEquals(0.20, profile.drive.drivebase.maxOmega, 0.0);
        assertFalse(profile.allowDriveMotion);
    }

    private static void assertInvalidCollectPower(double value) {
        StarterIntakeMechanism.Config config = StarterIntakeMechanism.Config.defaults();
        config.collectPower = value;
        assertInvalid(
                config,
                "StarterIntakeMechanism.Config.collectPower",
                "finite, nonzero, and in [-1.0, 1.0]",
                "got " + value);
    }

    private static void assertInvalidEjectPower(double value) {
        StarterIntakeMechanism.Config config = StarterIntakeMechanism.Config.defaults();
        config.ejectPower = value;
        assertInvalid(
                config,
                "StarterIntakeMechanism.Config.ejectPower",
                "finite, nonzero, and in [-1.0, 1.0]",
                "got " + value);
    }

    private static void assertInvalid(StarterIntakeMechanism.Config config,
                                      String... messageFragments) {
        StarterTestHardware.HardwareMapProbe hardwareMap =
                new StarterTestHardware.HardwareMapProbe();
        StarterTestHardware.MotorProbe motor = hardwareMap.addMotor("intakeMotor");
        try {
            new StarterIntakeMechanism(hardwareMap, config);
            fail("Expected invalid intake Config to fail");
        } catch (RuntimeException expected) {
            for (String fragment : messageFragments) {
                assertTrue(
                        "Expected message fragment '" + fragment + "' in: "
                                + expected.getMessage(),
                        expected.getMessage() != null
                                && expected.getMessage().contains(fragment));
            }
        }
        assertEquals(0, hardwareMap.lookupCalls());
        assertEquals(0, hardwareMap.totalPowerWrites());
        assertEquals(DcMotorSimple.Direction.FORWARD, motor.direction());
    }

    private static void assertAcceptedPowers(double collectPower, double ejectPower) {
        StarterIntakeMechanism.Config config = StarterIntakeMechanism.Config.defaults();
        config.collectPower = collectPower;
        config.ejectPower = ejectPower;
        StarterTestHardware.HardwareMapProbe hardwareMap =
                new StarterTestHardware.HardwareMapProbe();
        StarterTestHardware.MotorProbe motor = hardwareMap.addMotor(config.motorName);
        StarterIntakeMechanism intake = new StarterIntakeMechanism(hardwareMap, config);
        ManualLoopClock time = new ManualLoopClock();

        intake.setMode(StarterIntake.Mode.COLLECT);
        intake.update(time.clock());
        assertEquals(collectPower, motor.lastPower(), 0.0);
        intake.setMode(StarterIntake.Mode.EJECT);
        intake.update(time.nextCycle(0.02));
        assertEquals(ejectPower, motor.lastPower(), 0.0);
    }

    private static void assertPublicField(Field field, String name, Class<?> type) {
        assertEquals(name, field.getName());
        assertEquals(type, field.getType());
        assertTrue(Modifier.isPublic(field.getModifiers()));
        assertFalse(Modifier.isStatic(field.getModifiers()));
    }

    private static boolean isPackagePrivate(int modifiers) {
        return !Modifier.isPublic(modifiers)
                && !Modifier.isProtected(modifiers)
                && !Modifier.isPrivate(modifiers);
    }
}
