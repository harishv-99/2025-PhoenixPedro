package edu.ftcsushi.fw.integrations.pedro;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;

import edu.ftcsushi.fw.ftc.localization.PinpointOdometryPredictor;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;

public final class PedroPathingRuntimeConfigCopyTest {

    private static final String[] FOLLOWER_MUTABLE_OBJECT_FIELDS = {
            "coefficientsTranslationalPIDF",
            "integralTranslational",
            "coefficientsHeadingPIDF",
            "coefficientsDrivePIDF",
            "coefficientsSecondaryTranslationalPIDF",
            "integralSecondaryTranslational",
            "coefficientsSecondaryHeadingPIDF",
            "coefficientsSecondaryDrivePIDF",
            "predictiveBrakingCoefficients"
    };

    @Test
    public void defaultsUseFreshLiteralPathConstraintsNotPedroGlobal() {
        PathConstraints original = PathConstraints.defaultConstraints;
        PathConstraints poisoned = new PathConstraints(
                0.21,
                22.0,
                23.0,
                0.24,
                25.0,
                2.6,
                27,
                2.8
        );
        PathConstraints.defaultConstraints = poisoned;
        try {
            PedroPathingRuntime.Config first = PedroPathingRuntime.Config.defaults();
            PedroPathingRuntime.Config second = PedroPathingRuntime.Config.defaults();

            assertNotSame(first, second);
            assertNotSame(first.predictor, second.predictor);
            assertNotSame(first.followerConstants, second.followerConstants);
            assertNotSame(first.mecanumConstants, second.mecanumConstants);
            assertNotSame(first.pathConstraints, second.pathConstraints);
            assertNotSame(poisoned, first.pathConstraints);
            assertPathValues(first.pathConstraints, 0.995, 0.1, 0.1, 0.007,
                    100.0, 1.0, 10, 1.0);
            assertSame(PedroFieldTransform.decodeInvertedFtc(), first.fieldTransform);
            assertSame(poisoned, PathConstraints.defaultConstraints);
        } finally {
            PathConstraints.defaultConstraints = original;
        }
    }

    @Test
    public void copyReconstructsEveryPinnedMutableLeafWithoutBitDrift() throws Exception {
        PedroPathingRuntime.Config source = distinctDraft();
        source.validatedCopy("PedroPathingRuntimeConfigCopyTest.distinctDraft");
        PedroPathingRuntime.Config copy = source.copy();

        assertNotSame(source, copy);
        assertNotSame(source.predictor, copy.predictor);
        assertNotSame(source.followerConstants, copy.followerConstants);
        assertNotSame(source.mecanumConstants, copy.mecanumConstants);
        assertNotSame(source.pathConstraints, copy.pathConstraints);
        assertSame(source.fieldTransform, copy.fieldTransform);

        assertPublicPrimitiveBitsEqual(source.followerConstants, copy.followerConstants);
        for (String fieldName : FOLLOWER_MUTABLE_OBJECT_FIELDS) {
            Object sourceValue = FollowerConstants.class.getField(fieldName).get(
                    source.followerConstants
            );
            Object copiedValue = FollowerConstants.class.getField(fieldName).get(
                    copy.followerConstants
            );
            assertNotSame(fieldName, sourceValue, copiedValue);
            assertPublicPrimitiveBitsEqual(sourceValue, copiedValue);
        }

        assertPublicPrimitiveBitsEqual(source.mecanumConstants, copy.mecanumConstants);
        assertEquals(
                source.mecanumConstants.leftFrontMotorName,
                copy.mecanumConstants.leftFrontMotorName
        );
        assertEquals(
                source.mecanumConstants.leftRearMotorName,
                copy.mecanumConstants.leftRearMotorName
        );
        assertEquals(
                source.mecanumConstants.rightFrontMotorName,
                copy.mecanumConstants.rightFrontMotorName
        );
        assertEquals(
                source.mecanumConstants.rightRearMotorName,
                copy.mecanumConstants.rightRearMotorName
        );
        assertSame(
                source.mecanumConstants.leftFrontMotorDirection,
                copy.mecanumConstants.leftFrontMotorDirection
        );
        assertSame(
                source.mecanumConstants.leftRearMotorDirection,
                copy.mecanumConstants.leftRearMotorDirection
        );
        assertSame(
                source.mecanumConstants.rightFrontMotorDirection,
                copy.mecanumConstants.rightFrontMotorDirection
        );
        assertSame(
                source.mecanumConstants.rightRearMotorDirection,
                copy.mecanumConstants.rightRearMotorDirection
        );
        assertNotSame(source.mecanumConstants.frontLeftVector,
                copy.mecanumConstants.frontLeftVector);
        assertDoubleBitsEqual(
                source.mecanumConstants.frontLeftVector.getMagnitude(),
                copy.mecanumConstants.frontLeftVector.getMagnitude()
        );
        assertDoubleBitsEqual(
                source.mecanumConstants.frontLeftVector.getTheta(),
                copy.mecanumConstants.frontLeftVector.getTheta()
        );
        Vector sourceNormalized = source.mecanumConstants.frontLeftVector.normalize();
        Vector copiedNormalized = copy.mecanumConstants.frontLeftVector.normalize();
        assertDoubleBitsEqual(sourceNormalized.getXComponent(), copiedNormalized.getXComponent());
        assertDoubleBitsEqual(sourceNormalized.getYComponent(), copiedNormalized.getYComponent());

        assertEquals(source.predictor.hardwareMapName, copy.predictor.hardwareMapName);
        assertDoubleBitsEqual(
                source.predictor.forwardPodOffsetLeftInches,
                copy.predictor.forwardPodOffsetLeftInches
        );
        assertDoubleBitsEqual(
                source.predictor.strafePodOffsetForwardInches,
                copy.predictor.strafePodOffsetForwardInches
        );
        assertSame(source.predictor.encoderResolution, copy.predictor.encoderResolution);
        assertSame(source.predictor.forwardPodDirection, copy.predictor.forwardPodDirection);
        assertSame(source.predictor.strafePodDirection, copy.predictor.strafePodDirection);
        assertEquals(source.predictor.yawScalar, copy.predictor.yawScalar);
        assertDoubleBitsEqual(source.predictor.quality, copy.predictor.quality);
        assertPathValues(
                copy.pathConstraints,
                source.pathConstraints.getTValueConstraint(),
                source.pathConstraints.getVelocityConstraint(),
                source.pathConstraints.getTranslationalConstraint(),
                source.pathConstraints.getHeadingConstraint(),
                source.pathConstraints.getTimeoutConstraint(),
                source.pathConstraints.getBrakingStrength(),
                source.pathConstraints.getBEZIER_CURVE_SEARCH_LIMIT(),
                source.pathConstraints.getBrakingStart()
        );
    }

    @Test
    public void sourceAndCopyAreIsolatedInBothMutationDirections() {
        PedroPathingRuntime.Config source = distinctDraft();
        PedroPathingRuntime.Config copy = source.copy();

        double copiedMass = copy.followerConstants.mass;
        source.followerConstants.mass = 9001.0;
        assertDoubleBitsEqual(copiedMass, copy.followerConstants.mass);

        double sourceFilteredT = source.followerConstants.coefficientsDrivePIDF.T;
        copy.followerConstants.coefficientsDrivePIDF.T = 0.123456789;
        assertDoubleBitsEqual(
                sourceFilteredT,
                source.followerConstants.coefficientsDrivePIDF.T
        );

        double copiedVectorTheta = copy.mecanumConstants.frontLeftVector.getTheta();
        source.mecanumConstants.frontLeftVector.setTheta(1.2);
        assertDoubleBitsEqual(copiedVectorTheta, copy.mecanumConstants.frontLeftVector.getTheta());

        double sourceTimeout = source.pathConstraints.getTimeoutConstraint();
        copy.pathConstraints.setTimeoutConstraint(12345.0);
        assertDoubleBitsEqual(sourceTimeout, source.pathConstraints.getTimeoutConstraint());

        double copiedPredictorOffset = copy.predictor.forwardPodOffsetLeftInches;
        source.predictor.forwardPodOffsetLeftInches = 54321.0;
        assertDoubleBitsEqual(copiedPredictorOffset, copy.predictor.forwardPodOffsetLeftInches);
    }

    @Test
    public void rawCopyPreservesTopLevelAndNestedNullDrafts() {
        PedroPathingRuntime.Config topLevel = PedroPathingRuntime.Config.defaults();
        topLevel.predictor = null;
        topLevel.followerConstants = null;
        topLevel.mecanumConstants = null;
        topLevel.pathConstraints = null;
        topLevel.fieldTransform = null;
        PedroPathingRuntime.Config copiedTopLevel = topLevel.copy();
        assertNull(copiedTopLevel.predictor);
        assertNull(copiedTopLevel.followerConstants);
        assertNull(copiedTopLevel.mecanumConstants);
        assertNull(copiedTopLevel.pathConstraints);
        assertNull(copiedTopLevel.fieldTransform);

        PedroPathingRuntime.Config nested = PedroPathingRuntime.Config.defaults();
        nested.followerConstants.coefficientsDrivePIDF = null;
        nested.followerConstants.predictiveBrakingCoefficients = null;
        nested.mecanumConstants.frontLeftVector = null;
        nested.mecanumConstants.leftFrontMotorName = null;
        nested.mecanumConstants.rightRearMotorDirection = null;
        PedroPathingRuntime.Config copiedNested = nested.copy();
        assertNull(copiedNested.followerConstants.coefficientsDrivePIDF);
        assertNull(copiedNested.followerConstants.predictiveBrakingCoefficients);
        assertNull(copiedNested.mecanumConstants.frontLeftVector);
        assertNull(copiedNested.mecanumConstants.leftFrontMotorName);
        assertNull(copiedNested.mecanumConstants.rightRearMotorDirection);
    }

    private static PedroPathingRuntime.Config distinctDraft() {
        PedroPathingRuntime.Config draft = PedroPathingRuntime.Config.defaults();
        draft.predictor.hardwareMapName = "distinct odo";
        draft.predictor.forwardPodOffsetLeftInches = distinctDouble(1);
        draft.predictor.strafePodOffsetForwardInches = distinctDouble(2);
        draft.predictor.encoderResolution = PinpointOdometryPredictor.EncoderResolution
                .forGoBildaPod(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        draft.predictor.forwardPodDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        draft.predictor.strafePodDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        draft.predictor.yawScalar = distinctDouble(3);
        draft.predictor.quality = distinctFraction(4);

        draft.followerConstants.coefficientsTranslationalPIDF = pid(10);
        draft.followerConstants.integralTranslational = pid(20);
        draft.followerConstants.coefficientsHeadingPIDF = pid(30);
        draft.followerConstants.coefficientsDrivePIDF = filtered(40);
        draft.followerConstants.coefficientsSecondaryTranslationalPIDF = pid(50);
        draft.followerConstants.integralSecondaryTranslational = pid(60);
        draft.followerConstants.coefficientsSecondaryHeadingPIDF = pid(70);
        draft.followerConstants.coefficientsSecondaryDrivePIDF = filtered(80);
        draft.followerConstants.predictiveBrakingCoefficients = predictive(90);
        assignPrimitiveSentinels(draft.followerConstants, 100);
        draft.followerConstants.coefficientsDrivePIDF.T = distinctFraction(101);
        draft.followerConstants.coefficientsSecondaryDrivePIDF.T = distinctFraction(102);
        draft.followerConstants.predictiveBrakingCoefficients.maximumBrakingPower =
                distinctFraction(103);
        draft.followerConstants.usePredictiveBraking = true;
        draft.followerConstants.useSecondaryTranslationalPIDF = true;
        draft.followerConstants.useSecondaryHeadingPIDF = true;
        draft.followerConstants.useSecondaryDrivePIDF = true;
        draft.followerConstants.automaticHoldEnd = false;
        draft.followerConstants.forwardZeroPowerAcceleration = -18.125;
        draft.followerConstants.lateralZeroPowerAcceleration = -15.375;
        draft.followerConstants.stuckTValueLow = distinctFraction(104);
        draft.followerConstants.stuckTValueHigh = 0.875;

        assignPrimitiveSentinels(draft.mecanumConstants, 200);
        draft.mecanumConstants.maxPower = distinctFraction(201);
        draft.mecanumConstants.motorCachingThreshold = distinctFraction(202);
        draft.mecanumConstants.staticFrictionCoefficient = distinctFraction(203);
        draft.mecanumConstants.useBrakeModeInTeleOp = true;
        draft.mecanumConstants.useVoltageCompensation = true;
        draft.mecanumConstants.frontLeftVector = new Vector(7.25, 0.37);
        draft.mecanumConstants.leftFrontMotorName = "lf exact ";
        draft.mecanumConstants.leftRearMotorName = "lr exact";
        draft.mecanumConstants.rightFrontMotorName = "rf exact";
        draft.mecanumConstants.rightRearMotorName = "rr exact";
        draft.mecanumConstants.leftFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        draft.mecanumConstants.leftRearMotorDirection = DcMotorSimple.Direction.FORWARD;
        draft.mecanumConstants.rightFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        draft.mecanumConstants.rightRearMotorDirection = DcMotorSimple.Direction.REVERSE;

        draft.pathConstraints = new PathConstraints(
                distinctFraction(301),
                distinctDouble(302),
                distinctDouble(303),
                distinctDouble(304),
                distinctDouble(305),
                distinctDouble(306),
                307,
                distinctDouble(308)
        );
        return draft;
    }

    private static PIDFCoefficients pid(int base) {
        return new PIDFCoefficients(
                distinctDouble(base),
                distinctDouble(base + 1),
                distinctDouble(base + 2),
                distinctDouble(base + 3)
        );
    }

    private static FilteredPIDFCoefficients filtered(int base) {
        return new FilteredPIDFCoefficients(
                distinctDouble(base),
                distinctDouble(base + 1),
                distinctDouble(base + 2),
                distinctDouble(base + 3),
                distinctDouble(base + 4)
        );
    }

    private static PredictiveBrakingCoefficients predictive(int base) {
        PredictiveBrakingCoefficients value = new PredictiveBrakingCoefficients(
                distinctDouble(base),
                distinctDouble(base + 1),
                distinctDouble(base + 2)
        );
        value.maximumBrakingPower = distinctDouble(base + 3);
        return value;
    }

    private static void assignPrimitiveSentinels(Object target, int base) {
        int index = 0;
        for (Field field : target.getClass().getFields()) {
            if (Modifier.isStatic(field.getModifiers()) || !field.getType().isPrimitive()) {
                continue;
            }
            try {
                if (field.getType() == double.class) {
                    field.setDouble(target, distinctDouble(base + index));
                } else if (field.getType() == int.class) {
                    field.setInt(target, base + index);
                } else if (field.getType() == boolean.class) {
                    field.setBoolean(target, (index & 1) == 0);
                } else {
                    throw new AssertionError("Unexpected primitive field " + field);
                }
            } catch (IllegalAccessException impossible) {
                throw new AssertionError(impossible);
            }
            index++;
        }
    }

    private static void assertPublicPrimitiveBitsEqual(Object expected, Object actual)
            throws IllegalAccessException {
        for (Field field : expected.getClass().getFields()) {
            if (Modifier.isStatic(field.getModifiers()) || !field.getType().isPrimitive()) {
                continue;
            }
            if (field.getType() == double.class) {
                assertDoubleBitsEqual(field.getDouble(expected), field.getDouble(actual));
            } else if (field.getType() == int.class) {
                assertEquals(field.getInt(expected), field.getInt(actual));
            } else if (field.getType() == boolean.class) {
                assertEquals(field.getBoolean(expected), field.getBoolean(actual));
            } else {
                throw new AssertionError("Unexpected primitive field " + field);
            }
        }
    }

    private static double distinctDouble(int index) {
        return Double.longBitsToDouble(0x3ff0000000000000L + index);
    }

    private static double distinctFraction(int index) {
        return Double.longBitsToDouble(0x3fe0000000000000L + index);
    }

    private static void assertPathValues(PathConstraints actual,
                                         double t,
                                         double velocity,
                                         double translation,
                                         double heading,
                                         double timeout,
                                         double brakingStrength,
                                         int searchLimit,
                                         double brakingStart) {
        assertDoubleBitsEqual(t, actual.getTValueConstraint());
        assertDoubleBitsEqual(velocity, actual.getVelocityConstraint());
        assertDoubleBitsEqual(translation, actual.getTranslationalConstraint());
        assertDoubleBitsEqual(heading, actual.getHeadingConstraint());
        assertDoubleBitsEqual(timeout, actual.getTimeoutConstraint());
        assertDoubleBitsEqual(brakingStrength, actual.getBrakingStrength());
        assertEquals(searchLimit, actual.getBEZIER_CURVE_SEARCH_LIMIT());
        assertDoubleBitsEqual(brakingStart, actual.getBrakingStart());
    }

    private static void assertDoubleBitsEqual(double expected, double actual) {
        assertEquals(
                Long.toHexString(Double.doubleToRawLongBits(expected)),
                Long.toHexString(Double.doubleToRawLongBits(actual))
        );
    }
}
