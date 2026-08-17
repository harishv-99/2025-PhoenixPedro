package edu.ftcphoenix.fw.integrations.pedro;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathConstraints;

import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

public final class PedroPathingRuntimeConfigValidationTest {

    private static final String ROOT = PedroPathingRuntime.Config.class.getCanonicalName();

    @Test
    public void validatedCopyUsesCanonicalOrExplicitContextAndReturnsAnotherSnapshot() {
        PedroPathingRuntime.Config draft = PedroPathingRuntime.Config.defaults();
        PedroPathingRuntime.Config validated = draft.validatedCopy(null);
        assertNotSame(draft, validated);
        assertNotSame(draft.predictor, validated.predictor);
        assertNotSame(draft.followerConstants, validated.followerConstants);
        assertNotSame(draft.mecanumConstants, validated.mecanumConstants);
        assertNotSame(draft.pathConstraints, validated.pathConstraints);

        assertInvalid(
                c -> c.followerConstants.mass = Double.NaN,
                ROOT + ".followerConstants.mass",
                "finite",
                "NaN"
        );
        assertInvalidWithContext(
                "Custom.owner",
                c -> c.followerConstants.mass = Double.NaN,
                "Custom.owner.followerConstants.mass",
                "NaN"
        );
        assertInvalidWithContext(
                "  ",
                c -> c.followerConstants.mass = Double.NaN,
                ROOT + ".followerConstants.mass"
        );
    }

    @Test
    public void everyRequiredTopLevelAndNestedObjectRejectsNullWithOwnerPath()
            throws Exception {
        assertNullInvalid(c -> c.predictor = null, ROOT + ".predictor");
        assertNullInvalid(c -> c.followerConstants = null, ROOT + ".followerConstants");
        assertNullInvalid(c -> c.mecanumConstants = null, ROOT + ".mecanumConstants");
        assertNullInvalid(c -> c.pathConstraints = null, ROOT + ".pathConstraints");
        assertNullInvalid(c -> c.fieldTransform = null, ROOT + ".fieldTransform");

        for (Field field : FollowerConstants.class.getFields()) {
            if (field.getType() != PIDFCoefficients.class
                    && field.getType() != FilteredPIDFCoefficients.class
                    && field.getType() != PredictiveBrakingCoefficients.class) {
                continue;
            }
            assertNullInvalid(
                    c -> setField(field, c.followerConstants, null),
                    ROOT + ".followerConstants." + field.getName()
            );
        }

        for (Field field : MecanumConstants.class.getFields()) {
            if (field.getType() != String.class
                    && field.getType() != com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.class
                    && field.getType() != Vector.class) {
                continue;
            }
            assertNullInvalid(
                    c -> setField(field, c.mecanumConstants, null),
                    ROOT + ".mecanumConstants." + field.getName()
            );
        }
    }

    @Test
    public void everyPinnedFollowerNumericLeafRejectsNonFiniteValues() throws Exception {
        double[] invalidValues = {
                Double.NaN,
                Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY
        };
        for (Field ownerField : FollowerConstants.class.getFields()) {
            Class<?> ownerType = ownerField.getType();
            if (ownerType != PIDFCoefficients.class
                    && ownerType != FilteredPIDFCoefficients.class
                    && ownerType != PredictiveBrakingCoefficients.class) {
                continue;
            }
            for (Field leaf : ownerType.getFields()) {
                if (leaf.getType() != double.class) {
                    continue;
                }
                for (double invalid : invalidValues) {
                    assertInvalid(
                            c -> setDoubleField(
                                    leaf,
                                    getField(ownerField, c.followerConstants),
                                    invalid
                            ),
                            ROOT + ".followerConstants." + ownerField.getName()
                                    + "." + leaf.getName(),
                            Double.toString(invalid)
                    );
                }
            }
        }

        for (Field field : FollowerConstants.class.getFields()) {
            if (field.getType() != double.class) {
                continue;
            }
            for (double invalid : invalidValues) {
                assertInvalid(
                        c -> setDoubleField(field, c.followerConstants, invalid),
                        ROOT + ".followerConstants." + field.getName(),
                        Double.toString(invalid)
                );
            }
        }
    }

    @Test
    public void filteredAndPredictiveBrakingDomainsAreExact() {
        assertInvalid(
                c -> c.followerConstants.coefficientsDrivePIDF.T = -Double.MIN_VALUE,
                "coefficientsDrivePIDF.T",
                "[0.0, 1.0]"
        );
        assertValid(c -> c.followerConstants.coefficientsDrivePIDF.T = 0.0);
        assertValid(c -> c.followerConstants.coefficientsDrivePIDF.T = 1.0);
        assertInvalid(
                c -> c.followerConstants.coefficientsDrivePIDF.T = Math.nextUp(1.0),
                "coefficientsDrivePIDF.T"
        );
        assertInvalid(
                c -> c.followerConstants.coefficientsSecondaryDrivePIDF.T = -1.0,
                "coefficientsSecondaryDrivePIDF.T"
        );
        assertValid(c -> c.followerConstants.coefficientsSecondaryDrivePIDF.T = 0.0);
        assertValid(c -> c.followerConstants.coefficientsSecondaryDrivePIDF.T = 1.0);
        assertInvalid(
                c -> c.followerConstants.coefficientsSecondaryDrivePIDF.T = Math.nextUp(1.0),
                "coefficientsSecondaryDrivePIDF.T"
        );

        assertInvalid(
                c -> c.followerConstants.predictiveBrakingCoefficients.P = -Double.MIN_VALUE,
                "predictiveBrakingCoefficients.P"
        );
        assertInvalid(
                c -> c.followerConstants.predictiveBrakingCoefficients.kLinearBraking = -1.0,
                "kLinearBraking"
        );
        assertInvalid(
                c -> c.followerConstants.predictiveBrakingCoefficients.kQuadraticFriction = -1.0,
                "kQuadraticFriction"
        );
        assertValid(c -> {
            c.followerConstants.predictiveBrakingCoefficients.kLinearBraking = 0.0;
            c.followerConstants.predictiveBrakingCoefficients.kQuadraticFriction = 0.0;
        });
        assertInvalid(
                c -> c.followerConstants.predictiveBrakingCoefficients.maximumBrakingPower = 0.0,
                "maximumBrakingPower",
                "(0, 1]"
        );
        assertValid(c -> c.followerConstants.predictiveBrakingCoefficients
                .maximumBrakingPower = Double.MIN_VALUE);
        assertValid(c -> c.followerConstants.predictiveBrakingCoefficients
                .maximumBrakingPower = 1.0);
        assertInvalid(
                c -> c.followerConstants.predictiveBrakingCoefficients
                        .maximumBrakingPower = Math.nextUp(1.0),
                "maximumBrakingPower"
        );
        assertValid(c -> {
            c.followerConstants.usePredictiveBraking = false;
            c.followerConstants.predictiveBrakingCoefficients.P = 0.0;
        });
        assertInvalid(
                c -> {
                    c.followerConstants.usePredictiveBraking = true;
                    c.followerConstants.predictiveBrakingCoefficients.P = 0.0;
                },
                "predictiveBrakingCoefficients.P",
                "usePredictiveBraking=true",
                "0.0"
        );
        assertValid(c -> {
            c.followerConstants.usePredictiveBraking = true;
            c.followerConstants.predictiveBrakingCoefficients.P = Double.MIN_VALUE;
        });
    }

    @Test
    public void followerSwitchScalingSearchAndCompletionDomainsAreExact() {
        assertSwitchRule(
                c -> c.followerConstants.headingPIDFSwitch = 0.0,
                c -> c.followerConstants.useSecondaryHeadingPIDF = true,
                "headingPIDFSwitch",
                "useSecondaryHeadingPIDF"
        );
        assertSwitchRule(
                c -> c.followerConstants.drivePIDFSwitch = 0.0,
                c -> c.followerConstants.useSecondaryDrivePIDF = true,
                "drivePIDFSwitch",
                "useSecondaryDrivePIDF"
        );
        assertSwitchRule(
                c -> c.followerConstants.translationalPIDFSwitch = 0.0,
                c -> c.followerConstants.useSecondaryTranslationalPIDF = true,
                "translationalPIDFSwitch",
                "useSecondaryTranslationalPIDF"
        );

        assertInvalid(
                c -> c.followerConstants.holdPointTranslationalScaling = -1.0,
                "holdPointTranslationalScaling"
        );
        assertValid(c -> c.followerConstants.holdPointTranslationalScaling = 0.0);
        assertInvalid(
                c -> c.followerConstants.holdPointHeadingScaling = -1.0,
                "holdPointHeadingScaling"
        );
        assertValid(c -> c.followerConstants.holdPointHeadingScaling = 0.0);
        assertInvalid(
                c -> c.followerConstants.centripetalScaling = -1.0,
                "centripetalScaling"
        );
        assertValid(c -> c.followerConstants.centripetalScaling = 0.0);
        assertInvalid(
                c -> {
                    c.followerConstants.centripetalScaling = 2.0;
                    c.followerConstants.mass = Double.MAX_VALUE;
                },
                "centripetalScaling",
                "mass",
                "must remain finite",
                Double.toString(Double.MAX_VALUE)
        );

        assertInvalid(
                c -> c.followerConstants.turnHeadingErrorThreshold = 0.0,
                "turnHeadingErrorThreshold",
                "> 0"
        );
        assertValid(c -> c.followerConstants.turnHeadingErrorThreshold = Double.MIN_VALUE);
        assertInvalid(c -> c.followerConstants.mass = 0.0, ".mass", "> 0");
        assertValid(c -> c.followerConstants.mass = Double.MIN_VALUE);
        assertInvalid(
                c -> c.followerConstants.BEZIER_CURVE_SEARCH_LIMIT = 0,
                "BEZIER_CURVE_SEARCH_LIMIT",
                "got 0"
        );
        assertValid(c -> c.followerConstants.BEZIER_CURVE_SEARCH_LIMIT = 1);
    }

    @Test
    public void accelerationAndKalmanRepresentabilityRulesMatchPinnedSourceOrder() {
        assertInvalid(
                c -> c.followerConstants.forwardZeroPowerAcceleration = -0.0,
                "forwardZeroPowerAcceleration",
                "< 0"
        );
        assertInvalid(
                c -> c.followerConstants.lateralZeroPowerAcceleration = -0.0,
                "lateralZeroPowerAcceleration",
                "< 0"
        );
        assertValid(c -> {
            c.followerConstants.forwardZeroPowerAcceleration = -Double.MIN_VALUE;
            c.mecanumConstants.xVelocity = Double.MIN_VALUE;
            c.mecanumConstants.yVelocity = Double.MIN_VALUE;
        });
        assertValid(c -> c.followerConstants.lateralZeroPowerAcceleration =
                -Double.MIN_VALUE);
        assertInvalid(
                c -> c.followerConstants.forwardZeroPowerAcceleration = -Double.MAX_VALUE,
                "2.0 *",
                "forwardZeroPowerAcceleration",
                Double.toString(-Double.MAX_VALUE)
        );
        assertInvalid(
                c -> c.followerConstants.lateralZeroPowerAcceleration = -Double.MAX_VALUE,
                "2.0 *",
                "lateralZeroPowerAcceleration"
        );

        assertValid(c -> {
            c.followerConstants.driveKalmanFilterModelCovariance = 0.0;
            c.followerConstants.driveKalmanFilterDataCovariance = Double.MAX_VALUE;
        });
        assertInvalid(
                c -> c.followerConstants.driveKalmanFilterModelCovariance = -Double.MIN_VALUE,
                "driveKalmanFilterModelCovariance",
                ">= 0"
        );
        assertInvalid(
                c -> c.followerConstants.driveKalmanFilterDataCovariance = -Double.MIN_VALUE,
                "driveKalmanFilterDataCovariance",
                ">= 0"
        );
        assertValid(c -> {
            c.followerConstants.driveKalmanFilterModelCovariance = 1.0;
            c.followerConstants.driveKalmanFilterDataCovariance = 0.0;
        });
        assertValid(c -> {
            c.followerConstants.driveKalmanFilterModelCovariance = Double.MIN_VALUE;
            c.followerConstants.driveKalmanFilterDataCovariance = 0.0;
        });
        assertValid(c -> {
            c.followerConstants.driveKalmanFilterModelCovariance = 0.0;
            c.followerConstants.driveKalmanFilterDataCovariance = Double.MIN_VALUE;
        });
        assertInvalid(
                c -> {
                    c.followerConstants.driveKalmanFilterModelCovariance = 0.0;
                    c.followerConstants.driveKalmanFilterDataCovariance = 0.0;
                },
                "driveKalmanFilterModelCovariance",
                "driveKalmanFilterDataCovariance",
                "at least one positive",
                "0.0 and 0.0"
        );
        assertInvalid(
                c -> {
                    c.followerConstants.driveKalmanFilterModelCovariance = 1.0;
                    c.followerConstants.driveKalmanFilterDataCovariance = Double.MAX_VALUE;
                },
                "driveKalmanFilterModelCovariance",
                "driveKalmanFilterDataCovariance",
                "must remain finite",
                Double.toString(Double.MAX_VALUE)
        );
    }

    @Test
    public void stuckDetectionDomainsAndOrderingAreExact() {
        assertInvalid(c -> c.followerConstants.stuckVelocity = -1.0, "stuckVelocity");
        assertValid(c -> c.followerConstants.stuckVelocity = 0.0);
        assertInvalid(c -> c.followerConstants.stuckTimeout = -1.0, "stuckTimeout");
        assertValid(c -> c.followerConstants.stuckTimeout = 0.0);
        assertValid(c -> {
            c.followerConstants.stuckTValueLow = 0.0;
            c.followerConstants.stuckTValueHigh = 1.0;
        });
        assertInvalid(c -> c.followerConstants.stuckTValueLow = -Double.MIN_VALUE,
                "stuckTValueLow");
        assertInvalid(c -> c.followerConstants.stuckTValueHigh = Math.nextUp(1.0),
                "stuckTValueHigh");
        assertInvalid(
                c -> {
                    c.followerConstants.stuckTValueLow = 0.5;
                    c.followerConstants.stuckTValueHigh = 0.5;
                },
                "stuckTValueLow",
                "stuckTValueHigh",
                "0.5 and 0.5"
        );
    }

    @Test
    public void everyMecanumAndPathNumberRejectsNonFiniteValues() throws Exception {
        double[] invalidValues = {
                Double.NaN,
                Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY
        };
        for (Field field : MecanumConstants.class.getFields()) {
            if (field.getType() != double.class) {
                continue;
            }
            for (double invalid : invalidValues) {
                assertInvalid(
                        c -> setDoubleField(field, c.mecanumConstants, invalid),
                        ROOT + ".mecanumConstants." + field.getName(),
                        Double.toString(invalid)
                );
            }
        }

        PathMutation[] mutations = {
                (p, v) -> p.setTValueConstraint(v),
                (p, v) -> p.setVelocityConstraint(v),
                (p, v) -> p.setTranslationalConstraint(v),
                (p, v) -> p.setHeadingConstraint(v),
                (p, v) -> p.setTimeoutConstraint(v),
                (p, v) -> p.setBrakingStrength(v),
                (p, v) -> p.setBrakingStart(v)
        };
        String[] names = {
                "tValueConstraint",
                "velocityConstraint",
                "translationalConstraint",
                "headingConstraint",
                "timeoutConstraint",
                "brakingStrength",
                "brakingStart"
        };
        for (int i = 0; i < mutations.length; i++) {
            for (double invalid : invalidValues) {
                PathMutation mutation = mutations[i];
                assertInvalid(
                        c -> mutation.apply(c.pathConstraints, invalid),
                        ROOT + ".pathConstraints." + names[i],
                        Double.toString(invalid)
                );
            }
        }
    }

    @Test
    public void mecanumIdentityBoundsVoltageAndVectorBasisAreExact() {
        assertInvalid(
                c -> c.mecanumConstants.leftFrontMotorName = " \t ",
                "leftFrontMotorName",
                "non-whitespace"
        );
        assertInvalid(
                c -> {
                    c.mecanumConstants.leftFrontMotorName = "drive";
                    c.mecanumConstants.leftRearMotorName = " drive ";
                },
                "leftFrontMotorName='drive'",
                "leftRearMotorName=' drive '",
                "duplicates motor hardware name"
        );
        assertValid(c -> {
            c.mecanumConstants.leftFrontMotorName = "drive";
            c.mecanumConstants.leftRearMotorName = "Drive";
        });

        assertInvalid(c -> c.mecanumConstants.xVelocity = 0.0, "xVelocity", "> 0");
        assertInvalid(c -> c.mecanumConstants.yVelocity = -1.0, "yVelocity", "> 0");
        assertInvalid(c -> c.mecanumConstants.nominalVoltage = 0.0,
                "nominalVoltage", "> 0");
        assertValid(c -> c.mecanumConstants.xVelocity = Double.MIN_VALUE);
        assertValid(c -> c.mecanumConstants.yVelocity = Double.MIN_VALUE);
        assertValid(c -> c.mecanumConstants.nominalVoltage = Double.MIN_VALUE);
        assertInvalid(c -> c.mecanumConstants.maxPower = 0.0, "maxPower", "(0, 1]");
        assertValid(c -> c.mecanumConstants.maxPower = Double.MIN_VALUE);
        assertValid(c -> c.mecanumConstants.maxPower = 1.0);
        assertInvalid(c -> c.mecanumConstants.maxPower = Math.nextUp(1.0), "maxPower");

        assertValid(c -> c.mecanumConstants.motorCachingThreshold = 0.0);
        assertValid(c -> c.mecanumConstants.motorCachingThreshold = Math.nextDown(1.0));
        assertInvalid(c -> c.mecanumConstants.motorCachingThreshold = 1.0,
                "motorCachingThreshold", "[0.0, 1.0)");
        assertInvalid(c -> c.mecanumConstants.motorCachingThreshold = -Double.MIN_VALUE,
                "motorCachingThreshold");
        assertValid(c -> c.mecanumConstants.staticFrictionCoefficient = 0.0);
        assertValid(c -> c.mecanumConstants.staticFrictionCoefficient = Math.nextDown(1.0));
        assertInvalid(c -> c.mecanumConstants.staticFrictionCoefficient = -Double.MIN_VALUE,
                "staticFrictionCoefficient", "[0.0, 1.0)");
        assertInvalid(c -> c.mecanumConstants.staticFrictionCoefficient = 1.0,
                "staticFrictionCoefficient", "[0.0, 1.0)");

        double squareOverflow = Math.nextUp(Math.sqrt(Double.MAX_VALUE));
        assertValid(c -> {
            c.mecanumConstants.useVoltageCompensation = false;
            c.mecanumConstants.nominalVoltage = squareOverflow;
        });
        assertValid(c -> {
            c.mecanumConstants.useVoltageCompensation = true;
            c.mecanumConstants.nominalVoltage = Math.sqrt(Double.MAX_VALUE);
        });
        assertInvalid(
                c -> {
                    c.mecanumConstants.useVoltageCompensation = true;
                    c.mecanumConstants.nominalVoltage = squareOverflow;
                },
                "nominalVoltage",
                "useVoltageCompensation=true",
                "must remain finite"
        );

        assertInvalid(
                c -> c.mecanumConstants.frontLeftVector = new Vector(Double.NaN, 0.0),
                "frontLeftVector"
        );
        double[] nonFinite = {
                Double.NaN,
                Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY
        };
        for (double invalid : nonFinite) {
            assertInvalid(c -> {
                Vector vector = new Vector();
                vector.setOrthogonalComponents(invalid, 1.0);
                c.mecanumConstants.frontLeftVector = vector;
            }, "frontLeftVector.x", Double.toString(invalid));
            assertInvalid(c -> {
                Vector vector = new Vector();
                vector.setOrthogonalComponents(1.0, invalid);
                c.mecanumConstants.frontLeftVector = vector;
            }, "frontLeftVector", "finite", "got");
        }
        assertInvalid(
                c -> c.mecanumConstants.frontLeftVector = new Vector(),
                "frontLeftVector.magnitude",
                "> 0"
        );
        assertInvalid(
                c -> c.mecanumConstants.frontLeftVector = new Vector(1.0, 1e-20),
                "normalizedBasisDeterminant"
        );

        Vector collapsible = new Vector(1.0, 1e-20);
        Vector mirrored = new Vector(
                collapsible.getMagnitude(),
                2.0 * Math.PI - collapsible.getTheta()
        );
        double initialDeterminant = collapsible.cross(mirrored);
        assertTrue(initialDeterminant != 0.0);
        assertTrue(Double.isFinite(4.0 / Math.abs(initialDeterminant)));
        collapsible.rotateVector(1.0);
        mirrored.rotateVector(1.0);
        assertEquals(0.0, collapsible.cross(mirrored), 0.0);
    }

    @Test
    public void wheelBasisFloorIsInclusiveAndAcceptedRotationsRemainConditioned()
            throws Exception {
        Field floorField = PedroPathingRuntime.class.getDeclaredField(
                "MIN_NORMALIZED_WHEEL_BASIS_DETERMINANT"
        );
        floorField.setAccessible(true);
        double floor = floorField.getDouble(null);
        assertEquals(64.0 * Math.ulp(4.0 * Math.PI), floor, 0.0);
        Method validator = PedroPathingRuntime.class.getDeclaredMethod(
                "validateNormalizedWheelBasisDeterminant",
                double.class,
                String.class
        );
        validator.setAccessible(true);

        validator.invoke(null, floor, "test.basis");
        try {
            validator.invoke(null, Math.nextDown(floor), "test.basis");
            fail("Expected determinant below the conditioning floor to fail");
        } catch (InvocationTargetException expected) {
            assertTrue(expected.getCause() instanceof IllegalArgumentException);
            assertTrue(expected.getCause().getMessage().contains("test.basis"));
            assertTrue(expected.getCause().getMessage().contains(Double.toString(floor)));
        }

        double nearFloorTarget = floor * 2.0;
        Vector nearFloor = new Vector(1.0, 0.5 * Math.asin(nearFloorTarget));
        assertValid(c -> c.mecanumConstants.frontLeftVector = nearFloor);
        Vector defaults = new MecanumConstants().frontLeftVector;
        double[] headings = {
                0.0,
                1.0,
                Math.PI,
                Math.nextDown(2.0 * Math.PI)
        };
        for (double heading : headings) {
            assertConditionedAfterRotation(nearFloor, heading);
            assertConditionedAfterRotation(defaults, heading);
        }
    }

    @Test
    public void pathDomainsAndPinnedArithmeticCrossChecksAreExact() {
        assertInvalid(c -> c.pathConstraints.setTValueConstraint(0.0),
                "tValueConstraint", "(0, 1]");
        assertValid(c -> c.pathConstraints.setTValueConstraint(Double.MIN_VALUE));
        assertValid(c -> c.pathConstraints.setTValueConstraint(1.0));
        assertInvalid(c -> c.pathConstraints.setTValueConstraint(Math.nextUp(1.0)),
                "tValueConstraint");
        assertInvalid(c -> c.pathConstraints.setVelocityConstraint(-Double.MIN_VALUE),
                "velocityConstraint");
        assertValid(c -> c.pathConstraints.setVelocityConstraint(0.0));
        assertInvalid(c -> c.pathConstraints.setTranslationalConstraint(-1.0),
                "translationalConstraint");
        assertValid(c -> c.pathConstraints.setTranslationalConstraint(0.0));
        assertInvalid(c -> c.pathConstraints.setHeadingConstraint(-1.0),
                "headingConstraint");
        assertValid(c -> c.pathConstraints.setHeadingConstraint(0.0));
        assertInvalid(c -> c.pathConstraints.setTimeoutConstraint(-1.0),
                "timeoutConstraint");
        assertValid(c -> c.pathConstraints.setTimeoutConstraint(0.0));
        assertInvalid(c -> c.pathConstraints.setBrakingStart(-1.0), "brakingStart");
        assertValid(c -> c.pathConstraints.setBrakingStart(0.0));
        assertInvalid(c -> c.pathConstraints.setBrakingStrength(0.0),
                "brakingStrength", "> 0");
        assertValid(c -> c.pathConstraints.setBrakingStrength(Double.MIN_VALUE));
        assertInvalid(c -> c.pathConstraints.setBEZIER_CURVE_SEARCH_LIMIT(0),
                "BEZIER_CURVE_SEARCH_LIMIT", "got 0");
        assertValid(c -> c.pathConstraints.setBEZIER_CURVE_SEARCH_LIMIT(1));

        assertInvalid(
                c -> c.pathConstraints.setBrakingStrength(Double.MAX_VALUE),
                "forwardZeroPowerAcceleration",
                "brakingStrength",
                "must remain finite"
        );
        assertInvalid(
                c -> {
                    double acceleration = Math.abs(
                            c.followerConstants.forwardZeroPowerAcceleration
                    );
                    c.pathConstraints.setBrakingStrength(
                            (Double.MAX_VALUE / (acceleration * 4.0)) * 0.75
                    );
                },
                "2.0 *",
                "forwardZeroPowerAcceleration",
                "brakingStrength"
        );
        assertInvalid(
                c -> {
                    c.followerConstants.forwardZeroPowerAcceleration = -Double.MIN_VALUE;
                    c.mecanumConstants.xVelocity = Double.MIN_VALUE;
                    c.mecanumConstants.yVelocity = Double.MIN_VALUE;
                    c.pathConstraints.setBrakingStrength(Double.MAX_VALUE / 8.0);
                },
                "lateralZeroPowerAcceleration",
                "brakingStrength",
                "must remain finite"
        );
        assertInvalid(
                c -> {
                    c.followerConstants.forwardZeroPowerAcceleration = -Double.MIN_VALUE;
                    c.mecanumConstants.xVelocity = Double.MIN_VALUE;
                    c.mecanumConstants.yVelocity = Double.MIN_VALUE;
                    double acceleration = Math.abs(
                            c.followerConstants.lateralZeroPowerAcceleration
                    );
                    c.pathConstraints.setBrakingStrength(
                            (Double.MAX_VALUE / (acceleration * 4.0)) * 0.75
                    );
                },
                "2.0 *",
                "lateralZeroPowerAcceleration",
                "brakingStrength"
        );
        assertInvalid(
                c -> c.mecanumConstants.xVelocity =
                        Math.nextUp(Math.sqrt(Double.MAX_VALUE)),
                "xVelocity",
                "forwardZeroPowerAcceleration",
                "must remain finite"
        );
        assertInvalid(
                c -> c.mecanumConstants.yVelocity =
                        Math.nextUp(Math.sqrt(Double.MAX_VALUE)),
                "yVelocity",
                "forwardZeroPowerAcceleration",
                "must remain finite"
        );
        assertInvalid(
                c -> {
                    c.followerConstants.forwardZeroPowerAcceleration = -Double.MIN_VALUE;
                    c.mecanumConstants.xVelocity = Double.MIN_VALUE;
                    c.mecanumConstants.yVelocity = 1.0;
                },
                "yVelocity",
                "forwardZeroPowerAcceleration",
                "must remain finite"
        );
        assertInvalid(
                c -> {
                    c.mecanumConstants.xVelocity = 1e154;
                    c.pathConstraints.setBrakingStart(1e10);
                },
                "xVelocity",
                "brakingStart",
                "times"
        );
        assertInvalid(
                c -> {
                    c.mecanumConstants.xVelocity = Double.MIN_VALUE;
                    c.mecanumConstants.yVelocity = 1e154;
                    c.pathConstraints.setBrakingStart(1e10);
                },
                "yVelocity",
                "brakingStart",
                "times"
        );
    }

    private static void assertSwitchRule(ConfigMutation zeroSwitch,
                                         ConfigMutation enable,
                                         String switchName,
                                         String enableName) {
        assertValid(zeroSwitch);
        assertInvalid(c -> {
            zeroSwitch.apply(c);
            enable.apply(c);
        }, switchName, enableName + "=true", "0.0");
        assertInvalid(
                c -> setFollowerDouble(
                        c.followerConstants,
                        switchName,
                        -Double.MIN_VALUE
                ),
                switchName,
                ">= 0"
        );
        assertInvalid(c -> {
            enable.apply(c);
            setFollowerDouble(c.followerConstants, switchName, -Double.MIN_VALUE);
        }, switchName, ">= 0");
        assertValid(c -> {
            setFollowerDouble(c.followerConstants, switchName, Double.MIN_VALUE);
            enable.apply(c);
        });
    }

    private static void assertConditionedAfterRotation(Vector source, double heading) {
        Vector left = source.normalize();
        Vector mirrored = new Vector(left.getMagnitude(), 2.0 * Math.PI - left.getTheta());
        left.rotateVector(heading);
        mirrored.rotateVector(heading);
        double determinant = left.cross(mirrored);
        assertTrue("finite determinant at heading " + heading, Double.isFinite(determinant));
        assertTrue("nonzero determinant at heading " + heading, determinant != 0.0);
        assertTrue(Double.isFinite(4.0 / Math.abs(determinant)));
    }

    private static void assertValid(ConfigMutation mutation) {
        PedroPathingRuntime.Config config = PedroPathingRuntime.Config.defaults();
        mutation.apply(config);
        config.validatedCopy(null);
    }

    private static void assertInvalid(ConfigMutation mutation, String... fragments) {
        assertInvalidWithContext(null, mutation, fragments);
    }

    private static void assertInvalidWithContext(String context,
                                                 ConfigMutation mutation,
                                                 String... fragments) {
        PedroPathingRuntime.Config config = PedroPathingRuntime.Config.defaults();
        mutation.apply(config);
        try {
            config.validatedCopy(context);
            fail("Expected invalid configuration");
        } catch (IllegalArgumentException expected) {
            assertContains(expected, fragments);
        }
    }

    private static void assertNullInvalid(ConfigMutation mutation, String... fragments) {
        PedroPathingRuntime.Config config = PedroPathingRuntime.Config.defaults();
        mutation.apply(config);
        try {
            config.validatedCopy(null);
            fail("Expected null configuration collaborator");
        } catch (NullPointerException expected) {
            assertContains(expected, fragments);
        }
    }

    private static void assertContains(RuntimeException failure, String... fragments) {
        String message = failure.getMessage();
        assertTrue("Expected a diagnostic message", message != null);
        for (String fragment : fragments) {
            assertTrue(
                    "Expected message containing '" + fragment + "' but got: " + message,
                    message.contains(fragment)
            );
        }
    }

    private static Object getField(Field field, Object target) {
        try {
            return field.get(target);
        } catch (IllegalAccessException impossible) {
            throw new AssertionError(impossible);
        }
    }

    private static void setField(Field field, Object target, Object value) {
        try {
            field.set(target, value);
        } catch (IllegalAccessException impossible) {
            throw new AssertionError(impossible);
        }
    }

    private static void setDoubleField(Field field, Object target, double value) {
        try {
            field.setDouble(target, value);
        } catch (IllegalAccessException impossible) {
            throw new AssertionError(impossible);
        }
    }

    private static void setFollowerDouble(FollowerConstants constants,
                                          String fieldName,
                                          double value) {
        try {
            FollowerConstants.class.getField(fieldName).setDouble(constants, value);
        } catch (ReflectiveOperationException impossible) {
            throw new AssertionError(impossible);
        }
    }

    private interface ConfigMutation {
        void apply(PedroPathingRuntime.Config config);
    }

    private interface PathMutation {
        void apply(PathConstraints constraints, double value);
    }
}
