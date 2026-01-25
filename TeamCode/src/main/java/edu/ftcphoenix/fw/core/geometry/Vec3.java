package edu.ftcphoenix.fw.core.geometry;

import java.util.Objects;

/**
 * Immutable 3D vector.
 *
 * <h2>Design intent</h2>
 * <p>
 * Phoenix uses {@code Vec3} and {@link Mat3} as small, shared linear-algebra primitives so that
 * higher-level classes (like {@link Pose3d} and FTC adapter frame conversions) do not each re-define
 * their own matrix/vector helpers.
 * </p>
 *
 * <h2>Units and frames</h2>
 * <p>
 * {@code Vec3} is frame-agnostic. The meaning of (x, y, z) depends on the caller.
 * When used with Phoenix-framed poses, Phoenix defines axes as:
 * </p>
 * <ul>
 *   <li><b>+X</b> forward</li>
 *   <li><b>+Y</b> left</li>
 *   <li><b>+Z</b> up</li>
 * </ul>
 *
 * <p>
 * When used with FTC SDK adapter conversions, the caller is responsible for using the correct
 * SDK frame conventions at the boundary, then converting into Phoenix framing.
 * </p>
 */
public final class Vec3 {

    /**
     * X component (unit depends on caller; often inches).
     */
    public final double x;

    /**
     * Y component (unit depends on caller; often inches).
     */
    public final double y;

    /**
     * Z component (unit depends on caller; often inches).
     */
    public final double z;

    /**
     * Construct an immutable vector.
     */
    public Vec3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    /**
     * A constant zero vector (0,0,0).
     *
     * <p>This is safe to share as a singleton because {@code Vec3} is immutable.</p>
     */
    private static final Vec3 ZERO = new Vec3(0.0, 0.0, 0.0);

    /**
     * Returns the zero vector (0,0,0).
     */
    public static Vec3 zero() {
        return ZERO;
    }

    /**
     * Add another vector.
     */
    public Vec3 add(Vec3 other) {
        Objects.requireNonNull(other, "other");
        return new Vec3(x + other.x, y + other.y, z + other.z);
    }

    /**
     * Subtract another vector.
     */
    public Vec3 sub(Vec3 other) {
        Objects.requireNonNull(other, "other");
        return new Vec3(x - other.x, y - other.y, z - other.z);
    }

    /**
     * Negate this vector.
     */
    public Vec3 neg() {
        return new Vec3(-x, -y, -z);
    }

    /**
     * Scale this vector by a scalar.
     */
    public Vec3 scale(double s) {
        return new Vec3(x * s, y * s, z * s);
    }

    /**
     * Dot product with another vector.
     */
    public double dot(Vec3 other) {
        Objects.requireNonNull(other, "other");
        return x * other.x + y * other.y + z * other.z;
    }

    /**
     * Cross product with another vector.
     */
    public Vec3 cross(Vec3 other) {
        Objects.requireNonNull(other, "other");
        return new Vec3(
                y * other.z - z * other.y,
                z * other.x - x * other.z,
                x * other.y - y * other.x
        );
    }

    /**
     * Squared Euclidean norm (length^2).
     */
    public double normSquared() {
        return x * x + y * y + z * z;
    }

    /**
     * Euclidean norm (length).
     */
    public double norm() {
        return Math.sqrt(normSquared());
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        return "Vec3{" + "x=" + x + ", y=" + y + ", z=" + z + '}';
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean equals(Object o) {
        if (!(o instanceof Vec3)) return false;
        Vec3 other = (Vec3) o;
        return Double.compare(x, other.x) == 0
                && Double.compare(y, other.y) == 0
                && Double.compare(z, other.z) == 0;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public int hashCode() {
        return Objects.hash(x, y, z);
    }
}
