package edu.ftcsushi.fw.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

import edu.ftcsushi.fw.core.hal.PowerOutput;
import edu.ftcsushi.fw.ftc.FtcDrives;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Protects API-05's single FTC factory layer and normalized direct-drive vocabulary. */
public final class DirectMecanumDriveApiTest {

    private static final double EPSILON = 1.0e-9;

    @Test
    public void ftcFactoryHasExactlyTheDefaultAndCompleteConfigOverloads() {
        Set<String> parameterShapes = new HashSet<>();
        for (Method method : FtcDrives.class.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers()) && "mecanum".equals(method.getName())) {
                parameterShapes.add(Arrays.toString(method.getParameterTypes()));
            }
            assertFalse("post-construction brake helper must not remain public",
                    Modifier.isPublic(method.getModifiers())
                            && ("setDriveBrake".equals(method.getName())
                            || "setZeroPowerBehavior".equals(method.getName())));
        }

        Set<String> expected = new HashSet<>(Arrays.asList(
                Arrays.toString(new Class<?>[]{HardwareMap.class}),
                Arrays.toString(new Class<?>[]{HardwareMap.class, FtcDrives.MecanumConfig.class})
        ));
        assertEquals(expected, parameterShapes);
    }

    @Test
    public void completeConfigOwnsOnlyWiringBrakeAndDrivebaseConfiguration() throws Exception {
        assertEquals(
                new HashSet<>(Arrays.asList("wiring", "enableZeroPowerBrake", "drivebase")),
                publicFieldNames(FtcDrives.MecanumConfig.class));

        Constructor<FtcDrives.MecanumConfig> constructor =
                FtcDrives.MecanumConfig.class.getDeclaredConstructor();
        assertTrue(Modifier.isPrivate(constructor.getModifiers()));

        FtcDrives.MecanumConfig original = FtcDrives.MecanumConfig.defaults();
        FtcDrives.MecanumConfig copy = original.copy();
        original.wiring.frontLeftName = "changed";
        original.enableZeroPowerBrake = false;
        original.drivebase.maxAxial = 0.25;

        assertEquals(FtcDrives.DEFAULT_FRONT_LEFT_MOTOR_NAME, copy.wiring.frontLeftName);
        assertTrue(copy.enableZeroPowerBrake);
        assertEquals(1.0, copy.drivebase.maxAxial, EPSILON);
    }

    @Test
    public void directDrivebaseSurfaceUsesOnlyNormalizedSignalsAndOneInjectedConstructor()
            throws Exception {
        Set<String> configFields = publicFieldNames(MecanumDrivebase.Config.class);
        assertEquals(new HashSet<>(Arrays.asList("maxAxial", "maxLateral", "maxOmega")),
                configFields);

        int declaredDriveMethods = 0;
        for (Method method : MecanumDrivebase.class.getDeclaredMethods()) {
            assertFalse("direct mecanum owner must not declare a fictitious heartbeat",
                    "update".equals(method.getName()));
            if ("drive".equals(method.getName())) {
                declaredDriveMethods++;
                assertEquals(Arrays.asList(DriveSignal.class),
                        Arrays.asList(method.getParameterTypes()));
            }
        }
        assertEquals(1, declaredDriveMethods);

        Constructor<?>[] constructors = MecanumDrivebase.class.getConstructors();
        assertEquals(1, constructors.length);
        assertEquals(Arrays.asList(
                        PowerOutput.class,
                        PowerOutput.class,
                        PowerOutput.class,
                        PowerOutput.class,
                        MecanumDrivebase.Config.class),
                Arrays.asList(constructors[0].getParameterTypes()));

        assertClassIsAbsent("edu.ftcsushi.fw.drive.ChassisSpeeds");
        assertClassIsAbsent("edu.ftcsushi.fw.ftc.drive.FtcMecanumDriveLane");
    }

    @Test
    public void injectedConstructorRejectsNullsAndSnapshotsItsConfig() {
        RecordingOutput fl = new RecordingOutput();
        RecordingOutput fr = new RecordingOutput();
        RecordingOutput bl = new RecordingOutput();
        RecordingOutput br = new RecordingOutput();
        MecanumDrivebase.Config config = MecanumDrivebase.Config.defaults();

        expectNullPointer(() -> new MecanumDrivebase(null, fr, bl, br, config), "flPower");
        expectNullPointer(() -> new MecanumDrivebase(fl, null, bl, br, config), "frPower");
        expectNullPointer(() -> new MecanumDrivebase(fl, fr, null, br, config), "blPower");
        expectNullPointer(() -> new MecanumDrivebase(fl, fr, bl, null, config), "brPower");
        expectNullPointer(() -> new MecanumDrivebase(fl, fr, bl, br, null), "cfg");

        config.maxAxial = 0.4;
        MecanumDrivebase drive = new MecanumDrivebase(fl, fr, bl, br, config);
        config.maxAxial = 0.1;

        drive.drive(new DriveSignal(1.0, 0.0, 0.0));

        assertEquals(0.4, fl.power, EPSILON);
        assertEquals(0.4, fr.power, EPSILON);
        assertEquals(0.4, bl.power, EPSILON);
        assertEquals(0.4, br.power, EPSILON);
    }

    @Test
    public void injectedConstructorRejectsInvalidNormalizedScalesBeforeOutputWrites() {
        RecordingOutput fl = new RecordingOutput();
        RecordingOutput fr = new RecordingOutput();
        RecordingOutput bl = new RecordingOutput();
        RecordingOutput br = new RecordingOutput();
        double[] invalidValues = {
                Double.NaN,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY,
                -0.01,
                1.01
        };

        for (double invalidValue : invalidValues) {
            MecanumDrivebase.Config axial = MecanumDrivebase.Config.defaults();
            axial.maxAxial = invalidValue;
            expectInvalidConfig(() -> new MecanumDrivebase(fl, fr, bl, br, axial), "maxAxial");

            MecanumDrivebase.Config lateral = MecanumDrivebase.Config.defaults();
            lateral.maxLateral = invalidValue;
            expectInvalidConfig(
                    () -> new MecanumDrivebase(fl, fr, bl, br, lateral),
                    "maxLateral");

            MecanumDrivebase.Config omega = MecanumDrivebase.Config.defaults();
            omega.maxOmega = invalidValue;
            expectInvalidConfig(() -> new MecanumDrivebase(fl, fr, bl, br, omega), "maxOmega");
        }

        assertEquals(0, fl.writeCount + fr.writeCount + bl.writeCount + br.writeCount);
    }

    private static Set<String> publicFieldNames(Class<?> type) {
        Set<String> names = new HashSet<>();
        for (Field field : type.getFields()) {
            if (field.getDeclaringClass() == type) {
                names.add(field.getName());
            }
        }
        return names;
    }

    private static void assertClassIsAbsent(String className) {
        try {
            Class<?> unexpected = Class.forName(className);
            fail("legacy class must be deleted: " + unexpected.getName());
        } catch (ClassNotFoundException expected) {
            assertNotNull(expected);
        }
    }

    private static void expectNullPointer(Runnable action, String messagePart) {
        try {
            action.run();
            fail("Expected NullPointerException containing " + messagePart);
        } catch (NullPointerException expected) {
            assertTrue(String.valueOf(expected.getMessage()).contains(messagePart));
        }
    }

    private static void expectInvalidConfig(Runnable action, String fieldName) {
        try {
            action.run();
            fail("Expected IllegalArgumentException containing " + fieldName);
        } catch (IllegalArgumentException expected) {
            String message = String.valueOf(expected.getMessage());
            assertTrue(message, message.contains(fieldName));
            assertTrue(message, message.contains("finite"));
            assertTrue(message, message.contains("[0.0, 1.0]"));
        }
    }

    private static final class RecordingOutput implements PowerOutput {
        private double power;
        private int writeCount;

        @Override
        public void setPower(double power) {
            writeCount++;
            this.power = power;
        }

        @Override
        public double getCommandedPower() {
            return power;
        }
    }
}
