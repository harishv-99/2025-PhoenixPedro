package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.junit.Test;

import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.robots.phoenix.PhoenixProfile;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies that Pedro physical configuration has one PhoenixProfile authority. */
public final class ConstantsTest {

    private static final double EPSILON = 1e-9;

    @Test
    public void productionMappingsCopyDriveAndPinpointPhysicalConfiguration() {
        PhoenixProfile profile = PhoenixProfile.current().copy();
        profile.drive.wiring.frontLeftName = "profile-fl";
        profile.drive.wiring.frontRightName = "profile-fr";
        profile.drive.wiring.backLeftName = "profile-bl";
        profile.drive.wiring.backRightName = "profile-br";
        profile.drive.wiring.frontLeftDirection = Direction.REVERSE;
        profile.drive.wiring.frontRightDirection = Direction.FORWARD;
        profile.drive.wiring.backLeftDirection = Direction.REVERSE;
        profile.drive.wiring.backRightDirection = Direction.FORWARD;
        profile.drive.zeroPowerBrake = true;

        profile.localization.predictor.hardwareMapName = "profile-pinpoint";
        profile.localization.predictor.forwardPodOffsetLeftInches = 3.25;
        profile.localization.predictor.strafePodOffsetForwardInches = -4.5;
        profile.localization.predictor.customEncoderResolutionTicksPerInch = 1234.5;
        profile.localization.predictor.forwardPodDirection =
                GoBildaPinpointDriver.EncoderDirection.REVERSED;
        profile.localization.predictor.strafePodDirection =
                GoBildaPinpointDriver.EncoderDirection.FORWARD;
        profile.localization.predictor.yawScalar = 1.001;

        Constants.validatePhysicalConfig(profile, true);
        MecanumConstants drive = Constants.mecanumConstantsFrom(profile);
        PinpointConstants pinpoint = Constants.pinpointConstantsFrom(profile);

        assertEquals("profile-fl", drive.leftFrontMotorName);
        assertEquals("profile-fr", drive.rightFrontMotorName);
        assertEquals("profile-bl", drive.leftRearMotorName);
        assertEquals("profile-br", drive.rightRearMotorName);
        assertEquals(DcMotorSimple.Direction.REVERSE, drive.leftFrontMotorDirection);
        assertEquals(DcMotorSimple.Direction.FORWARD, drive.rightFrontMotorDirection);
        assertEquals(DcMotorSimple.Direction.REVERSE, drive.leftRearMotorDirection);
        assertEquals(DcMotorSimple.Direction.FORWARD, drive.rightRearMotorDirection);
        assertTrue(drive.useBrakeModeInTeleOp);

        assertEquals("profile-pinpoint", pinpoint.hardwareMapName);
        assertEquals(3.25, pinpoint.forwardPodY, EPSILON);
        assertEquals(-4.5, pinpoint.strafePodX, EPSILON);
        assertTrue(pinpoint.customEncoderResolution.isPresent());
        assertEquals(1234.5, pinpoint.customEncoderResolution.getAsDouble(), EPSILON);
        assertEquals(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                pinpoint.forwardEncoderDirection);
        assertEquals(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                pinpoint.strafeEncoderDirection);
        assertTrue(pinpoint.yawScalar.isPresent());
        assertEquals(1.001, pinpoint.yawScalar.getAsDouble(), EPSILON);
    }

    @Test
    public void podPresetIsRetainedWhenNoCustomResolutionIsConfigured() {
        PhoenixProfile profile = PhoenixProfile.current().copy();
        profile.localization.predictor.customEncoderResolutionTicksPerInch = null;
        profile.localization.predictor.encoderPods =
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD;

        PinpointConstants pinpoint = Constants.pinpointConstantsFrom(profile);

        assertFalse(pinpoint.customEncoderResolution.isPresent());
        assertEquals(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD,
                pinpoint.encoderResolution);
    }

    @Test
    public void invalidPhysicalProfileFailsBeforeHardwareConstruction() {
        PhoenixProfile duplicateMotor = PhoenixProfile.current().copy();
        duplicateMotor.drive.wiring.frontRightName =
                duplicateMotor.drive.wiring.frontLeftName;
        assertRejects(
                () -> Constants.validatePhysicalConfig(duplicateMotor, true),
                "duplicates motor hardware name"
        );

        PhoenixProfile noControlledReset = PhoenixProfile.current().copy();
        noControlledReset.localization.predictor.enableResetOnInit = false;
        assertRejects(
                () -> Constants.validatePhysicalConfig(noControlledReset, true),
                "enableResetOnInit=true"
        );

        PhoenixProfile blankPinpoint = PhoenixProfile.current().copy();
        blankPinpoint.localization.predictor.hardwareMapName = "  ";
        assertRejects(
                () -> Constants.validatePhysicalConfig(blankPinpoint, true),
                "hardwareMapName"
        );

        PhoenixProfile invalidOffset = PhoenixProfile.current().copy();
        invalidOffset.localization.predictor.forwardPodOffsetLeftInches = Double.NaN;
        assertRejects(
                () -> Constants.validatePhysicalConfig(invalidOffset, true),
                "forwardPodOffsetLeftInches"
        );
    }

    private static void assertRejects(Runnable action, String expectedMessage) {
        try {
            action.run();
            fail("Expected rejection containing: " + expectedMessage);
        } catch (IllegalArgumentException expected) {
            assertTrue(
                    "Expected message containing '" + expectedMessage + "' but got: "
                            + expected.getMessage(),
                    expected.getMessage().contains(expectedMessage)
            );
        }
    }
}
