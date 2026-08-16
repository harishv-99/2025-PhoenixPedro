package edu.ftcphoenix.fw.spatial;

import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;

import org.junit.Test;

import edu.ftcphoenix.fw.core.geometry.Pose2d;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the deliberately narrow robot-frame rectangle contract. */
public final class RobotFrameRectangle2dTest {

    @Test
    public void publicSurfaceHasOnlyTwoFactoriesAndTwoLiteralPredicates() throws Exception {
        assertTrue(Modifier.isFinal(RobotFrameRectangle2d.class.getModifiers()));
        assertEquals(1, RobotFrameRectangle2d.class.getDeclaredConstructors().length);
        Constructor<?> constructor = RobotFrameRectangle2d.class.getDeclaredConstructors()[0];
        assertTrue(Modifier.isPrivate(constructor.getModifiers()));
        assertEquals(0, RobotFrameRectangle2d.class.getFields().length);
        assertEquals(0, RobotFrameRectangle2d.class.getDeclaredClasses().length);

        assertPublicStaticMethod(
                "centeredInches",
                RobotFrameRectangle2d.class,
                double.class,
                double.class
        );
        assertPublicStaticMethod(
                "fromRobotFrameBoundsInches",
                RobotFrameRectangle2d.class,
                double.class,
                double.class,
                double.class,
                double.class
        );
        assertPublicInstanceMethod(
                "fullyInside",
                boolean.class,
                AxisAlignedBoxRegion2d.class,
                Pose2d.class
        );
        assertPublicInstanceMethod(
                "hasAnyCornerInside",
                boolean.class,
                AxisAlignedBoxRegion2d.class,
                Pose2d.class
        );
        assertPublicInstanceMethod("toString", String.class);

        int publicDeclaredMethods = 0;
        for (Method method : RobotFrameRectangle2d.class.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers())) {
                publicDeclaredMethods++;
            }
        }
        assertEquals(5, publicDeclaredMethods);

        String diagnostic = RobotFrameRectangle2d.fromRobotFrameBoundsInches(
                -7.0, 11.0, -5.0, 3.0).toString();
        assertTrue(diagnostic, diagnostic.contains("minXInches=-7.0"));
        assertTrue(diagnostic, diagnostic.contains("maxXInches=11.0"));
        assertTrue(diagnostic, diagnostic.contains("minYInches=-5.0"));
        assertTrue(diagnostic, diagnostic.contains("maxYInches=3.0"));
    }

    @Test
    public void centeredRectangleUsesRobotForwardLengthAndLeftRightWidth() {
        AxisAlignedBoxRegion2d parkBox = new AxisAlignedBoxRegion2d(-10.0, 10.0, -10.0, 10.0);
        RobotFrameRectangle2d rectangle = RobotFrameRectangle2d.centeredInches(18.0, 16.0);

        assertTrue(rectangle.fullyInside(parkBox, Pose2d.zero()));
        assertTrue(rectangle.fullyInside(
                parkBox,
                new Pose2d(0.0, 0.0, Math.PI / 2.0)
        ));

        Pose2d diagonal = new Pose2d(0.0, 0.0, Math.PI / 4.0);
        assertFalse(rectangle.fullyInside(parkBox, diagonal));
        assertFalse(rectangle.hasAnyCornerInside(parkBox, diagonal));
    }

    @Test
    public void boundaryCountsAsInsideAndOneOutsideCornerDoesNotClaimFullContainment() {
        AxisAlignedBoxRegion2d parkBox = new AxisAlignedBoxRegion2d(-10.0, 10.0, -10.0, 10.0);
        RobotFrameRectangle2d rectangle = RobotFrameRectangle2d.centeredInches(18.0, 16.0);

        assertTrue(rectangle.fullyInside(parkBox, new Pose2d(1.0, 0.0, 0.0)));

        Pose2d justOutside = new Pose2d(1.0 + 1e-9, 0.0, 0.0);
        assertFalse(rectangle.fullyInside(parkBox, justOutside));
        assertTrue(rectangle.hasAnyCornerInside(parkBox, justOutside));
    }

    @Test
    public void explicitBoundsSupportTrackedOriginOutsideRectangle() {
        AxisAlignedBoxRegion2d parkBox = new AxisAlignedBoxRegion2d(-3.0, 3.0, -3.0, 3.0);
        RobotFrameRectangle2d rectangle =
                RobotFrameRectangle2d.fromRobotFrameBoundsInches(2.0, 8.0, -2.0, 2.0);

        Pose2d trackedOriginOutsideBox = new Pose2d(-5.0, 0.0, 0.0);
        assertFalse(parkBox.contains(trackedOriginOutsideBox));
        assertTrue(rectangle.fullyInside(parkBox, trackedOriginOutsideBox));

        assertFalse(rectangle.fullyInside(parkBox, Pose2d.zero()));
        assertTrue(rectangle.hasAnyCornerInside(parkBox, Pose2d.zero()));
    }

    @Test
    public void asymmetricOffsetBoundsUseCounterClockwisePositiveRotation() {
        RobotFrameRectangle2d rectangle =
                RobotFrameRectangle2d.fromRobotFrameBoundsInches(1.0, 4.0, -2.0, 3.0);
        Pose2d counterClockwise = new Pose2d(10.0, 20.0, Math.PI / 2.0);
        Pose2d clockwise = new Pose2d(10.0, 20.0, -Math.PI / 2.0);

        // At +90 degrees the four corners span field X=[7, 12], Y=[21, 24].
        AxisAlignedBoxRegion2d counterClockwiseBox =
                new AxisAlignedBoxRegion2d(7.0 - 1e-9, 12.0 + 1e-9,
                        21.0 - 1e-9, 24.0 + 1e-9);
        assertTrue(rectangle.fullyInside(counterClockwiseBox, counterClockwise));
        assertFalse(rectangle.hasAnyCornerInside(counterClockwiseBox, clockwise));

        // At -90 degrees those same authored bounds instead span X=[8, 13], Y=[16, 19].
        AxisAlignedBoxRegion2d clockwiseBox =
                new AxisAlignedBoxRegion2d(8.0 - 1e-9, 13.0 + 1e-9,
                        16.0 - 1e-9, 19.0 + 1e-9);
        assertTrue(rectangle.fullyInside(clockwiseBox, clockwise));
        assertFalse(rectangle.hasAnyCornerInside(clockwiseBox, counterClockwise));
    }

    @Test
    public void everyCornerCanBeTheOnlyInsideOrOnlyOutsideCorner() {
        RobotFrameRectangle2d rectangle =
                RobotFrameRectangle2d.fromRobotFrameBoundsInches(-2.0, 5.0, -1.0, 3.0);
        Pose2d fieldToRobot = new Pose2d(10.0, -4.0, Math.PI / 6.0);
        double[][] corners = transformedCorners(
                fieldToRobot,
                -2.0,
                5.0,
                -1.0,
                3.0
        );

        for (int corner = 0; corner < corners.length; corner++) {
            double x = corners[corner][0];
            double y = corners[corner][1];
            AxisAlignedBoxRegion2d onlyThisCorner =
                    new AxisAlignedBoxRegion2d(x - 0.01, x + 0.01, y - 0.01, y + 0.01);
            assertFalse("corner " + corner, rectangle.fullyInside(onlyThisCorner, fieldToRobot));
            assertTrue("corner " + corner,
                    rectangle.hasAnyCornerInside(onlyThisCorner, fieldToRobot));
        }

        double minX = corners[1][0];
        double maxX = corners[2][0];
        double minY = corners[0][1];
        double maxY = corners[3][1];
        AxisAlignedBoxRegion2d[] onlyThisCornerOutside = {
                new AxisAlignedBoxRegion2d(minX - 1.0, maxX + 1.0,
                        corners[0][1] + 0.01, maxY + 1.0),
                new AxisAlignedBoxRegion2d(corners[1][0] + 0.01, maxX + 1.0,
                        minY - 1.0, maxY + 1.0),
                new AxisAlignedBoxRegion2d(minX - 1.0, corners[2][0] - 0.01,
                        minY - 1.0, maxY + 1.0),
                new AxisAlignedBoxRegion2d(minX - 1.0, maxX + 1.0,
                        minY - 1.0, corners[3][1] - 0.01)
        };
        for (int corner = 0; corner < onlyThisCornerOutside.length; corner++) {
            AxisAlignedBoxRegion2d box = onlyThisCornerOutside[corner];
            assertFalse("corner " + corner, rectangle.fullyInside(box, fieldToRobot));
            assertTrue("corner " + corner, rectangle.hasAnyCornerInside(box, fieldToRobot));
        }
    }

    @Test
    public void anyCornerPredicateDoesNotPretendToBeGeneralOverlap() {
        AxisAlignedBoxRegion2d parkBox = new AxisAlignedBoxRegion2d(-1.0, 1.0, -1.0, 1.0);

        RobotFrameRectangle2d surrounding = RobotFrameRectangle2d.centeredInches(40.0, 40.0);
        assertFalse(surrounding.fullyInside(parkBox, Pose2d.zero()));
        assertFalse(surrounding.hasAnyCornerInside(parkBox, Pose2d.zero()));

        RobotFrameRectangle2d edgeCrossing = RobotFrameRectangle2d.centeredInches(40.0, 1.0);
        assertFalse(edgeCrossing.fullyInside(parkBox, Pose2d.zero()));
        assertFalse(edgeCrossing.hasAnyCornerInside(parkBox, Pose2d.zero()));
    }

    @Test
    public void constructionRejectsNonFiniteDegenerateAndMisorderedGeometry() {
        double[] nonFinite = {
                Double.NaN,
                Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY
        };
        for (double invalid : nonFinite) {
            expectIllegalArgument(
                    () -> RobotFrameRectangle2d.centeredInches(invalid, 1.0),
                    "lengthInches"
            );
            expectIllegalArgument(
                    () -> RobotFrameRectangle2d.centeredInches(1.0, invalid),
                    "widthInches"
            );
            for (int field = 0; field < 4; field++) {
                final int invalidField = field;
                expectIllegalArgument(
                        () -> rectangleWithOneBound(invalidField, invalid),
                        boundName(invalidField)
                );
            }
        }

        for (double invalidDimension : new double[]{0.0, -0.0, -1.0}) {
            expectIllegalArgument(
                    () -> RobotFrameRectangle2d.centeredInches(invalidDimension, 1.0),
                    "lengthInches"
            );
            expectIllegalArgument(
                    () -> RobotFrameRectangle2d.centeredInches(1.0, invalidDimension),
                    "widthInches"
            );
        }
        expectIllegalArgument(
                () -> RobotFrameRectangle2d.centeredInches(Double.MIN_VALUE, 1.0),
                "lengthInches"
        );
        expectIllegalArgument(
                () -> RobotFrameRectangle2d.centeredInches(1.0, Double.MIN_VALUE),
                "widthInches"
        );

        expectIllegalArgument(
                () -> RobotFrameRectangle2d.fromRobotFrameBoundsInches(1.0, 1.0, -1.0, 1.0),
                "minXInches"
        );
        expectIllegalArgument(
                () -> RobotFrameRectangle2d.fromRobotFrameBoundsInches(2.0, 1.0, -1.0, 1.0),
                "minXInches"
        );
        expectIllegalArgument(
                () -> RobotFrameRectangle2d.fromRobotFrameBoundsInches(-1.0, 1.0, 1.0, 1.0),
                "minYInches"
        );
        expectIllegalArgument(
                () -> RobotFrameRectangle2d.fromRobotFrameBoundsInches(-1.0, 1.0, 2.0, 1.0),
                "minYInches"
        );

        RobotFrameRectangle2d.fromRobotFrameBoundsInches(
                0.0,
                Double.MIN_VALUE,
                0.0,
                Double.MIN_VALUE
        );
        RobotFrameRectangle2d.centeredInches(Double.MAX_VALUE, Double.MAX_VALUE);
    }

    @Test
    public void nullArgumentsFailImmediatelyAndNonFiniteRuntimePosesFailClosed() {
        AxisAlignedBoxRegion2d parkBox = new AxisAlignedBoxRegion2d(-10.0, 10.0, -10.0, 10.0);
        RobotFrameRectangle2d rectangle = RobotFrameRectangle2d.centeredInches(18.0, 16.0);

        expectNullPointer(() -> rectangle.fullyInside(null, Pose2d.zero()), "fieldBox");
        expectNullPointer(() -> rectangle.fullyInside(parkBox, null), "fieldToRobot");
        expectNullPointer(
                () -> rectangle.hasAnyCornerInside(null, Pose2d.zero()),
                "fieldBox"
        );
        expectNullPointer(
                () -> rectangle.hasAnyCornerInside(parkBox, null),
                "fieldToRobot"
        );

        for (double invalid : new double[]{
                Double.NaN,
                Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY
        }) {
            assertPredicatesFailClosed(rectangle, parkBox, new Pose2d(invalid, 0.0, 0.0));
            assertPredicatesFailClosed(rectangle, parkBox, new Pose2d(0.0, invalid, 0.0));
            assertPredicatesFailClosed(rectangle, parkBox, new Pose2d(0.0, 0.0, invalid));
        }

        RobotFrameRectangle2d huge =
                RobotFrameRectangle2d.centeredInches(Double.MAX_VALUE, Double.MAX_VALUE);
        assertPredicatesFailClosed(
                huge,
                parkBox,
                new Pose2d(Double.MAX_VALUE, Double.MAX_VALUE, 0.0)
        );
    }

    private static void assertPublicStaticMethod(String name,
                                                 Class<?> returnType,
                                                 Class<?>... parameterTypes) throws Exception {
        Method method = RobotFrameRectangle2d.class.getDeclaredMethod(name, parameterTypes);
        assertTrue(Modifier.isPublic(method.getModifiers()));
        assertTrue(Modifier.isStatic(method.getModifiers()));
        assertEquals(returnType, method.getReturnType());
    }

    private static void assertPublicInstanceMethod(String name,
                                                   Class<?> returnType,
                                                   Class<?>... parameterTypes) throws Exception {
        Method method = RobotFrameRectangle2d.class.getDeclaredMethod(name, parameterTypes);
        assertTrue(Modifier.isPublic(method.getModifiers()));
        assertFalse(Modifier.isStatic(method.getModifiers()));
        assertEquals(returnType, method.getReturnType());
    }

    private static RobotFrameRectangle2d rectangleWithOneBound(int field, double value) {
        double[] bounds = {-1.0, 1.0, -1.0, 1.0};
        bounds[field] = value;
        return RobotFrameRectangle2d.fromRobotFrameBoundsInches(
                bounds[0],
                bounds[1],
                bounds[2],
                bounds[3]
        );
    }

    private static double[][] transformedCorners(Pose2d fieldToRobot,
                                                 double minXInches,
                                                 double maxXInches,
                                                 double minYInches,
                                                 double maxYInches) {
        double cos = Math.cos(fieldToRobot.headingRad);
        double sin = Math.sin(fieldToRobot.headingRad);
        double[][] robotCorners = {
                {minXInches, minYInches},
                {minXInches, maxYInches},
                {maxXInches, minYInches},
                {maxXInches, maxYInches}
        };
        double[][] fieldCorners = new double[robotCorners.length][2];
        for (int corner = 0; corner < robotCorners.length; corner++) {
            double robotX = robotCorners[corner][0];
            double robotY = robotCorners[corner][1];
            fieldCorners[corner][0] = fieldToRobot.xInches + cos * robotX - sin * robotY;
            fieldCorners[corner][1] = fieldToRobot.yInches + sin * robotX + cos * robotY;
        }
        return fieldCorners;
    }

    private static String boundName(int field) {
        return new String[]{"minXInches", "maxXInches", "minYInches", "maxYInches"}[field];
    }

    private static void assertPredicatesFailClosed(RobotFrameRectangle2d rectangle,
                                                   AxisAlignedBoxRegion2d parkBox,
                                                   Pose2d fieldToRobot) {
        assertFalse(rectangle.fullyInside(parkBox, fieldToRobot));
        assertFalse(rectangle.hasAnyCornerInside(parkBox, fieldToRobot));
    }

    private static void expectIllegalArgument(Runnable action, String messagePart) {
        try {
            action.run();
            fail("Expected IllegalArgumentException containing " + messagePart);
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains(messagePart));
        }
    }

    private static void expectNullPointer(Runnable action, String messagePart) {
        try {
            action.run();
            fail("Expected NullPointerException containing " + messagePart);
        } catch (NullPointerException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains(messagePart));
        }
    }
}
