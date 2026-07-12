package edu.ftcphoenix.fw.integrations.pedro;

import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;

import java.util.Objects;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.math.MathUtil;

/**
 * Named conversion between the Phoenix FTC field frame and Pedro's field frame.
 *
 * <p>This class deliberately owns the conversion instead of delegating to Pedro 2.1.2's FTC
 * coordinate implementations. That pinned release applies a signed quarter-turn inconsistently
 * for the inverse operation. All distances here are inches and all angular values are radians.</p>
 */
public final class PedroFieldTransform {

    private static final double TWO_PI = 2.0 * Math.PI;
    private static final double PEDRO_FIELD_CENTER_INCHES = 72.0;

    private static final PedroFieldTransform DECODE_INVERTED_FTC =
            new PedroFieldTransform("Decode inverted FTC");

    private final String name;

    private PedroFieldTransform(String name) {
        this.name = name;
    }

    /**
     * Returns the explicit transform selected by Phoenix Decode.
     *
     * <p>The mapping is:</p>
     * <pre>
     * xPedro = 72 - yPhoenix
     * yPedro = 72 + xPhoenix
     * hPedro = hPhoenix + pi/2
     * </pre>
     */
    public static PedroFieldTransform decodeInvertedFtc() {
        return DECODE_INVERTED_FTC;
    }

    /** Returns the human-readable identity of this field convention. */
    public String name() {
        return name;
    }

    /**
     * Converts a Phoenix FTC-field pose to an explicitly Pedro-coordinate pose.
     *
     * @param phoenixFieldToRobotPose Phoenix field-to-robot pose, in inches/radians
     * @return equivalent Pedro field pose with heading normalized to [0, 2pi)
     */
    public Pose phoenixFieldToPedroPose(Pose2d phoenixFieldToRobotPose) {
        Pose2d pose = requireFinitePhoenixPose(
                Objects.requireNonNull(phoenixFieldToRobotPose, "phoenixFieldToRobotPose"),
                "phoenixFieldToRobotPose"
        );
        return new Pose(
                PEDRO_FIELD_CENTER_INCHES - pose.yInches,
                PEDRO_FIELD_CENTER_INCHES + pose.xInches,
                wrapZeroToTwoPi(pose.headingRad + Math.PI / 2.0),
                PedroCoordinates.INSTANCE
        );
    }

    /**
     * Converts a Pedro-coordinate field pose into the Phoenix FTC field frame.
     *
     * @param pedroFieldPose pose explicitly tagged with {@link PedroCoordinates}
     * @return Phoenix field-to-robot pose, in inches/radians
     */
    public Pose2d pedroToPhoenixFieldPose(Pose pedroFieldPose) {
        Pose pose = requirePedroPose(pedroFieldPose, "pedroFieldPose");
        return new Pose2d(
                pose.getY() - PEDRO_FIELD_CENTER_INCHES,
                PEDRO_FIELD_CENTER_INCHES - pose.getX(),
                MathUtil.wrapToPi(pose.getHeading() - Math.PI / 2.0)
        );
    }

    /**
     * Rotates a Phoenix field-frame velocity into Pedro's field axes without translating it.
     *
     * <p>Pedro's {@link com.pedropathing.localization.Localizer} represents velocity with a
     * {@link Pose}: X/Y are inches per second and heading is radians per second.</p>
     */
    public Pose phoenixFieldVelocityToPedro(double phoenixFieldVelocityXInchesPerSec,
                                            double phoenixFieldVelocityYInchesPerSec,
                                            double angularVelocityRadPerSec) {
        requireFinite(phoenixFieldVelocityXInchesPerSec,
                "phoenixFieldVelocityXInchesPerSec");
        requireFinite(phoenixFieldVelocityYInchesPerSec,
                "phoenixFieldVelocityYInchesPerSec");
        requireFinite(angularVelocityRadPerSec, "angularVelocityRadPerSec");
        return new Pose(
                -phoenixFieldVelocityYInchesPerSec,
                phoenixFieldVelocityXInchesPerSec,
                angularVelocityRadPerSec,
                PedroCoordinates.INSTANCE
        );
    }

    /** Package-private inverse used to regression-test the velocity transform without a core type. */
    PhoenixFieldVelocity pedroFieldVelocityToPhoenix(Pose pedroFieldVelocity) {
        Pose velocity = requirePedroPose(pedroFieldVelocity, "pedroFieldVelocity");
        return new PhoenixFieldVelocity(
                velocity.getY(),
                -velocity.getX(),
                velocity.getHeading()
        );
    }

    private static Pose requirePedroPose(Pose pose, String name) {
        Pose value = Objects.requireNonNull(pose, name);
        if (value.getCoordinateSystem() != PedroCoordinates.INSTANCE) {
            throw new IllegalArgumentException(
                    name + " must be explicitly tagged with PedroCoordinates; got "
                            + value.getCoordinateSystem()
            );
        }
        requireFinite(value.getX(), name + ".x");
        requireFinite(value.getY(), name + ".y");
        requireFinite(value.getHeading(), name + ".heading");
        return value;
    }

    private static Pose2d requireFinitePhoenixPose(Pose2d pose, String name) {
        requireFinite(pose.xInches, name + ".xInches");
        requireFinite(pose.yInches, name + ".yInches");
        requireFinite(pose.headingRad, name + ".headingRad");
        return pose;
    }

    private static double wrapZeroToTwoPi(double angleRad) {
        requireFinite(angleRad, "angleRad");
        double wrapped = angleRad % TWO_PI;
        return wrapped < 0.0 ? wrapped + TWO_PI : wrapped;
    }

    private static void requireFinite(double value, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite, got " + value);
        }
    }

    /** Internal value used only to verify the inverse velocity mapping. */
    static final class PhoenixFieldVelocity {
        final double xInchesPerSec;
        final double yInchesPerSec;
        final double angularRadPerSec;

        private PhoenixFieldVelocity(double xInchesPerSec,
                                     double yInchesPerSec,
                                     double angularRadPerSec) {
            this.xInchesPerSec = xInchesPerSec;
            this.yInchesPerSec = yInchesPerSec;
            this.angularRadPerSec = angularRadPerSec;
        }
    }

    @Override
    public String toString() {
        return "PedroFieldTransform{" + name + '}';
    }
}
