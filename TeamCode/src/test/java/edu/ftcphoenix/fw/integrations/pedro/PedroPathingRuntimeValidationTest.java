package edu.ftcphoenix.fw.integrations.pedro;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.paths.PathConstraints;

import org.junit.Test;

import edu.ftcphoenix.fw.ftc.localization.PinpointOdometryPredictor;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

public final class PedroPathingRuntimeValidationTest {

    @Test
    public void validPinnedDefaultsPassBoundaryValidation() {
        PedroPathingRuntime.validatePredictorConfig(
                PinpointOdometryPredictor.Config.defaults()
        );
        PedroPathingRuntime.validateFollowerConstants(new FollowerConstants());
        PedroPathingRuntime.validateMecanumConstants(new MecanumConstants());
        PedroPathingRuntime.validatePathConstraints(PathConstraints.defaultConstraints.copy());
    }

    @Test
    public void productionRequiresControlledPredictorInitReset() {
        PinpointOdometryPredictor.Config config =
                PinpointOdometryPredictor.Config.defaults().withResetOnInit(false);

        assertRejects(
                () -> PedroPathingRuntime.validatePredictorConfig(config),
                "enableResetOnInit=true"
        );
    }

    @Test
    public void motorNamesMustBePresentAndUnique() {
        MecanumConstants duplicate = new MecanumConstants()
                .leftFrontMotorName("drive")
                .leftRearMotorName("drive");
        assertRejects(
                () -> PedroPathingRuntime.validateMecanumConstants(duplicate),
                "duplicates motor hardware name"
        );

        MecanumConstants blank = new MecanumConstants().rightRearMotorName("  ");
        assertRejects(
                () -> PedroPathingRuntime.validateMecanumConstants(blank),
                "rightRearMotorName"
        );
    }

    @Test
    public void invalidVendorTuningFailsBeforeHardwareConstruction() {
        FollowerConstants followerConstants = new FollowerConstants();
        followerConstants.mass = Double.NaN;
        assertRejects(
                () -> PedroPathingRuntime.validateFollowerConstants(followerConstants),
                "followerConstants.mass"
        );

        MecanumConstants mecanumConstants = new MecanumConstants().maxPower(1.5);
        assertRejects(
                () -> PedroPathingRuntime.validateMecanumConstants(mecanumConstants),
                "mecanumConstants.maxPower"
        );

        PathConstraints pathConstraints = PathConstraints.defaultConstraints.copy();
        pathConstraints.setHeadingConstraint(Double.POSITIVE_INFINITY);
        assertRejects(
                () -> PedroPathingRuntime.validatePathConstraints(pathConstraints),
                "pathConstraints.headingConstraint"
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
