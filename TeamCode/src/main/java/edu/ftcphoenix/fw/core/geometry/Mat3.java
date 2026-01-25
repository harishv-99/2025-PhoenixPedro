package edu.ftcphoenix.fw.core.geometry;

import java.util.Objects;

/**
 * Immutable 3x3 matrix, primarily used for rotations and basis changes.
 *
 * <h2>Design intent</h2>
 * <p>
 * Phoenix uses {@link Mat3} and {@link Vec3} as small, shared linear-algebra primitives so that
 * higher-level classes (like {@code Pose3d} and FTC adapter frame conversions) do not each re-define
 * their own matrix/vector helpers.
 * </p>
 *
 * <h2>Rotation convention</h2>
 * <p>
 * Phoenix expresses 3D orientation using yaw/pitch/roll angles in radians:
 * </p>
 * <ul>
 *   <li><b>yawRad</b>: rotation about +Z (CCW when viewed from +Z)</li>
 *   <li><b>pitchRad</b>: rotation about +Y</li>
 *   <li><b>rollRad</b>: rotation about +X</li>
 * </ul>
 *
 * <p>
 * This class implements the common yaw-pitch-roll composition:
 * </p>
 *
 * <pre>
 * R = Rz(yaw) · Ry(pitch) · Rx(roll)
 * </pre>
 *
 * <p>
 * This is consistent with {@link Pose2d} where heading is a rotation about +Z, and is a standard,
 * student-friendly convention.
 * </p>
 */
public final class Mat3 {

    public final double m00, m01, m02;
    public final double m10, m11, m12;
    public final double m20, m21, m22;

    /**
     * Create a matrix with explicit elements.
     */
    public Mat3(double m00, double m01, double m02,
                double m10, double m11, double m12,
                double m20, double m21, double m22) {
        this.m00 = m00;
        this.m01 = m01;
        this.m02 = m02;
        this.m10 = m10;
        this.m11 = m11;
        this.m12 = m12;
        this.m20 = m20;
        this.m21 = m21;
        this.m22 = m22;
    }

    /**
     * A constant identity matrix.
     *
     * <p>This is safe to share as a singleton because {@code Mat3} is immutable.</p>
     */
    private static final Mat3 IDENTITY = new Mat3(
            1, 0, 0,
            0, 1, 0,
            0, 0, 1
    );

    /**
     * Returns the identity matrix.
     */
    public static Mat3 identity() {
        return IDENTITY;
    }

    /**
     * Transpose this matrix.
     */
    public Mat3 transpose() {
        return new Mat3(
                m00, m10, m20,
                m01, m11, m21,
                m02, m12, m22
        );
    }

    /**
     * Multiply this matrix by another: {@code this * other}.
     */
    public Mat3 mul(Mat3 other) {
        Objects.requireNonNull(other, "other");
        return new Mat3(
                m00 * other.m00 + m01 * other.m10 + m02 * other.m20,
                m00 * other.m01 + m01 * other.m11 + m02 * other.m21,
                m00 * other.m02 + m01 * other.m12 + m02 * other.m22,

                m10 * other.m00 + m11 * other.m10 + m12 * other.m20,
                m10 * other.m01 + m11 * other.m11 + m12 * other.m21,
                m10 * other.m02 + m11 * other.m12 + m12 * other.m22,

                m20 * other.m00 + m21 * other.m10 + m22 * other.m20,
                m20 * other.m01 + m21 * other.m11 + m22 * other.m21,
                m20 * other.m02 + m21 * other.m12 + m22 * other.m22
        );
    }

    /**
     * Multiply this matrix by a vector: {@code this * v}.
     */
    public Vec3 mul(Vec3 v) {
        Objects.requireNonNull(v, "v");
        return new Vec3(
                m00 * v.x + m01 * v.y + m02 * v.z,
                m10 * v.x + m11 * v.y + m12 * v.z,
                m20 * v.x + m21 * v.y + m22 * v.z
        );
    }

    /**
     * Build a rotation matrix from yaw/pitch/roll using:
     *
     * <pre>
     * R = Rz(yaw) · Ry(pitch) · Rx(roll)
     * </pre>
     */
    public static Mat3 fromYawPitchRoll(double yawRad, double pitchRad, double rollRad) {
        double cy = Math.cos(yawRad);
        double sy = Math.sin(yawRad);
        double cp = Math.cos(pitchRad);
        double sp = Math.sin(pitchRad);
        double cr = Math.cos(rollRad);
        double sr = Math.sin(rollRad);

        // Derived from Rz(yaw) * Ry(pitch) * Rx(roll)
        return new Mat3(
                cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
                sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
                -sp, cp * sr, cp * cr
        );
    }

    /**
     * Extract yaw/pitch/roll from a rotation matrix consistent with:
     *
     * <pre>
     * R = Rz(yaw) · Ry(pitch) · Rx(roll)
     * </pre>
     *
     * <p>This handles the gimbal singularity when {@code cos(pitch)} is close to 0 by choosing
     * {@code roll = 0} and deriving yaw from the remaining terms.</p>
     */
    public static YawPitchRoll toYawPitchRoll(Mat3 R) {
        Objects.requireNonNull(R, "R");

        // For Z-Y-X: pitch = asin(-r20)
        double sp = clamp(-R.m20, -1.0, 1.0);
        double pitch = Math.asin(sp);

        double cp = Math.cos(pitch);

        double yaw;
        double roll;

        if (Math.abs(cp) > 1e-9) {
            // yaw = atan2(r10, r00), roll = atan2(r21, r22)
            yaw = Math.atan2(R.m10, R.m00);
            roll = Math.atan2(R.m21, R.m22);
        } else {
            // Gimbal lock: choose roll=0 and derive yaw from other terms.
            roll = 0.0;
            yaw = Math.atan2(-R.m01, R.m11);
        }

        return new YawPitchRoll(
                Pose2d.wrapToPi(yaw),
                Pose2d.wrapToPi(pitch),
                Pose2d.wrapToPi(roll)
        );
    }

    /**
     * Simple immutable yaw/pitch/roll tuple (radians).
     */
    public static final class YawPitchRoll {
        public final double yawRad;
        public final double pitchRad;
        public final double rollRad;

        /**
         * Construct a yaw/pitch/roll triple.
         *
         * @param yawRad   yaw angle in radians (CCW-positive)
         * @param pitchRad pitch angle in radians
         * @param rollRad  roll angle in radians
         */
        public YawPitchRoll(double yawRad, double pitchRad, double rollRad) {
            this.yawRad = yawRad;
            this.pitchRad = pitchRad;
            this.rollRad = rollRad;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public String toString() {
            return "YawPitchRoll{yawRad=" + yawRad + ", pitchRad=" + pitchRad + ", rollRad=" + rollRad + "}";
        }
    }

    private static double clamp(double v, double lo, double hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        return "Mat3{" +
                "m00=" + m00 + ", m01=" + m01 + ", m02=" + m02 +
                ", m10=" + m10 + ", m11=" + m11 + ", m12=" + m12 +
                ", m20=" + m20 + ", m21=" + m21 + ", m22=" + m22 +
                '}';
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean equals(Object o) {
        if (!(o instanceof Mat3)) return false;
        Mat3 other = (Mat3) o;
        return Double.compare(m00, other.m00) == 0
                && Double.compare(m01, other.m01) == 0
                && Double.compare(m02, other.m02) == 0
                && Double.compare(m10, other.m10) == 0
                && Double.compare(m11, other.m11) == 0
                && Double.compare(m12, other.m12) == 0
                && Double.compare(m20, other.m20) == 0
                && Double.compare(m21, other.m21) == 0
                && Double.compare(m22, other.m22) == 0;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public int hashCode() {
        return Objects.hash(m00, m01, m02, m10, m11, m12, m20, m21, m22);
    }
}
