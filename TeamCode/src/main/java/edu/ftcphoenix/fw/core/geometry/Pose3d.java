package edu.ftcphoenix.fw.core.geometry;

import java.util.Objects;

/**
 * Immutable 3D pose (6-DOF) in Phoenix's standard right-handed coordinate conventions.
 *
 * <p>A {@code Pose3d} represents a rigid transform: a 3D translation plus a 3D orientation.
 * It is used to describe relationships between coordinate frames (e.g. camera-to-tag,
 * robot-to-camera mount, field-to-robot).</p>
 *
 * <h2>Coordinate conventions</h2>
 * <ul>
 *   <li><strong>Units:</strong> distances are in inches, angles are in radians.</li>
 *   <li><strong>Axes:</strong>
 *     <ul>
 *       <li>{@code xInches}, {@code yInches}, {@code zInches} are the X/Y/Z translation
 *           components of the pose in <em>whatever frame</em> you are working in.</li>
 *       <li>{@code Pose3d} itself does not assign a universal real-world meaning to +X/+Y/+Z.
 *           That meaning comes from the coordinate frame the pose is expressed in.</li>
 *       <li><b>Robot/mechanism frames</b> in Phoenix are typically expressed as +X forward,
 *           +Y left, +Z up.</li>
 *       <li><b>Field-centric poses</b> in Phoenix are expressed in the <b>FTC Field Coordinate
 *           System</b> for the current season (see {@code TagLayout} and {@code PoseEstimate}).</li>
 *     </ul>
 *   </li>
 *   <li><strong>Rotations:</strong> right-hand rule about the +axis.</li>
 *   <li><strong>Yaw/Pitch/Roll meaning:</strong>
 *     <ul>
 *       <li>{@code yawRad}: rotation about +Z (turning left is positive)</li>
 *       <li>{@code pitchRad}: rotation about +Y</li>
 *       <li>{@code rollRad}: rotation about +X</li>
 *     </ul>
 *   </li>
 * </ul>
 *
 * <h2>Rotation composition</h2>
 * <p>This class uses the common yaw-pitch-roll (Z-Y-X) convention when converting to/from
 * a rotation matrix:</p>
 *
 * <pre>
 * R = Rz(yaw) · Ry(pitch) · Rx(roll)
 * </pre>
 *
 * <p>This is a standard, student-friendly convention, and it keeps {@link Pose2d#headingRad}
 * consistent with {@link #yawRad} (both are about +Z).</p>
 */
public final class Pose3d {

    /**
     * X translation in inches (frame +X).
     */
    public final double xInches;

    /**
     * Y translation in inches (frame +Y).
     */
    public final double yInches;

    /**
     * Z translation in inches (frame +Z).
     */
    public final double zInches;

    /**
     * Rotation about +Z in radians (CCW-positive).
     */
    public final double yawRad;

    /**
     * Rotation about +Y in radians (right-hand rule).
     */
    public final double pitchRad;

    /**
     * Rotation about +X in radians (right-hand rule).
     */
    public final double rollRad;

    /**
     * Constructs a {@code Pose3d} (distances in inches, angles in radians).
     *
     * @param xInches  X translation in inches (frame +X)
     * @param yInches  Y translation in inches (frame +Y)
     * @param zInches  Z translation in inches (frame +Z)
     * @param yawRad   rotation about +Z in radians (CCW-positive)
     * @param pitchRad rotation about +Y in radians
     * @param rollRad  rotation about +X in radians
     */
    public Pose3d(
            double xInches,
            double yInches,
            double zInches,
            double yawRad,
            double pitchRad,
            double rollRad
    ) {
        this.xInches = xInches;
        this.yInches = yInches;
        this.zInches = zInches;
        this.yawRad = yawRad;
        this.pitchRad = pitchRad;
        this.rollRad = rollRad;
    }

    /**
     * A constant zero transform (no translation, no rotation).
     *
     * <p>This is safe to share as a singleton because {@code Pose3d} is immutable.</p>
     */
    private static final Pose3d ZERO = new Pose3d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    /**
     * Returns the zero transform (no translation, no rotation).
     */
    public static Pose3d zero() {
        return ZERO;
    }

    /**
     * Translation component as a {@link Vec3} (inches, Phoenix axes).
     */
    public Vec3 translation() {
        return new Vec3(xInches, yInches, zInches);
    }

    /**
     * Rotation component as a {@link Mat3}, using Phoenix yaw-pitch-roll convention.
     */
    public Mat3 rotation() {
        return Mat3.fromYawPitchRoll(yawRad, pitchRad, rollRad);
    }

    /**
     * Euclidean length of the translation component in inches.
     */
    public double translationNormInches() {
        return translation().norm();
    }

    /**
     * Convenience: project this 6DOF pose into a planar {@link Pose2d}.
     *
     * <p>This keeps (x, y) and uses {@link #yawRad} as heading, dropping z/pitch/roll.</p>
     *
     * @return planar pose projection (x, y, yaw)
     */
    public Pose2d toPose2d() {
        return new Pose2d(xInches, yInches, yawRad);
    }

    /**
     * Composes this transform with {@code next} (apply {@code this}, then {@code next}).
     *
     * <p>If this pose is A→B and {@code next} is B→C, the result is A→C.</p>
     *
     * @param next transform to apply after this one (non-null)
     * @return composed transform
     */
    public Pose3d then(Pose3d next) {
        Objects.requireNonNull(next, "next");

        Mat3 rThis = rotation();
        Mat3 rNext = next.rotation();

        // R = R_this * R_next
        Mat3 rOut = rThis.mul(rNext);

        // t = t_this + R_this * t_next
        Vec3 tOut = translation().add(rThis.mul(next.translation()));

        Mat3.YawPitchRoll ypr = Mat3.toYawPitchRoll(rOut);
        return new Pose3d(tOut.x, tOut.y, tOut.z, ypr.yawRad, ypr.pitchRad, ypr.rollRad);
    }

    /**
     * Returns the inverse transform.
     *
     * <p>If this pose is A→B, the inverse is B→A.</p>
     */
    public Pose3d inverse() {
        Mat3 r = rotation();

        // R_inv = R^T (rotation matrices are orthonormal)
        Mat3 rInv = r.transpose();

        // t_inv = -(R^T * t)
        Vec3 tInv = rInv.mul(translation()).neg();

        Mat3.YawPitchRoll ypr = Mat3.toYawPitchRoll(rInv);
        return new Pose3d(tInv.x, tInv.y, tInv.z, ypr.yawRad, ypr.pitchRad, ypr.rollRad);
    }

    /**
     * Creates a new pose with the same rotation but a different translation.
     */
    public Pose3d withTranslation(double xInches, double yInches, double zInches) {
        return new Pose3d(xInches, yInches, zInches, yawRad, pitchRad, rollRad);
    }

    /**
     * Creates a new pose with the same translation but a different rotation.
     */
    public Pose3d withRotation(double yawRad, double pitchRad, double rollRad) {
        return new Pose3d(xInches, yInches, zInches, yawRad, pitchRad, rollRad);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        return "Pose3d{"
                + "xInches=" + xInches
                + ", yInches=" + yInches
                + ", zInches=" + zInches
                + ", yawRad=" + yawRad
                + ", pitchRad=" + pitchRad
                + ", rollRad=" + rollRad
                + '}';
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean equals(Object o) {
        if (!(o instanceof Pose3d)) return false;
        Pose3d other = (Pose3d) o;
        return Double.compare(xInches, other.xInches) == 0
                && Double.compare(yInches, other.yInches) == 0
                && Double.compare(zInches, other.zInches) == 0
                && Double.compare(yawRad, other.yawRad) == 0
                && Double.compare(pitchRad, other.pitchRad) == 0
                && Double.compare(rollRad, other.rollRad) == 0;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public int hashCode() {
        return Objects.hash(xInches, yInches, zInches, yawRad, pitchRad, rollRad);
    }
}
