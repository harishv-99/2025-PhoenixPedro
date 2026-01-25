package edu.ftcphoenix.fw.ftc;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.geometry.Mat3;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.field.TagLayout;

/**
 * A {@link TagLayout} backed by the FTC SDK's {@link AprilTagLibrary}.
 *
 * <p>This adapter converts FTC {@link AprilTagMetadata} entries into Phoenix {@link TagPose}
 * objects using the optional metadata fields:</p>
 *
 * <ul>
 *   <li>{@link AprilTagMetadata#fieldPosition} ({@link VectorF})</li>
 *   <li>{@link AprilTagMetadata#fieldOrientation} ({@link Quaternion})</li>
 * </ul>
 *
 * <p>The FTC SDK states these optional fields are expressed in the FTC Field Coordinate System.
 * Phoenix adopts the same field frame in {@link TagLayout}.</p>
 *
 * <h2>Units</h2>
 * <p>
 * FTC metadata includes a {@link AprilTagMetadata#distanceUnit}. Phoenix stores all field distances
 * in inches, so values are converted as needed.
 * </p>
 *
 * <h2>Orientation</h2>
 * <p>
 * Phoenix stores tag orientation as yaw/pitch/roll in a {@link Pose3d}, consistent with
 * {@link Mat3#fromYawPitchRoll(double, double, double)} and {@link Mat3#toYawPitchRoll(Mat3)}.
 * The FTC quaternion is converted to a rotation matrix and then converted to yaw/pitch/roll.
 * </p>
 *
 * <h2>Performance</h2>
 * <p>
 * This class snapshots the library contents once in the constructor, building an immutable map
 * of tag ID to {@link TagPose}. This avoids per-loop allocations and ensures that
 * {@link #get(int)} is a simple map lookup.
 * </p>
 *
 * <h2>Missing field metadata</h2>
 * <p>
 * If a tag exists in the FTC library but does not provide {@code fieldPosition} or
 * {@code fieldOrientation}, it is omitted from this layout (i.e., {@link #get(int)} returns
 * {@code null}).
 * </p>
 */
public final class FtcGameTagLayout implements TagLayout {

    private final AprilTagLibrary library;
    private final Map<Integer, TagPose> posesById;
    private final Set<Integer> ids;

    /**
     * Creates a Phoenix {@link TagLayout} backed by an FTC {@link AprilTagLibrary}.
     *
     * <p>Only tags that provide the optional FTC metadata fields
     * {@link AprilTagMetadata#fieldPosition} and {@link AprilTagMetadata#fieldOrientation}
     * are included.</p>
     *
     * @param library FTC AprilTag library (non-null)
     */
    public FtcGameTagLayout(AprilTagLibrary library) {
        this.library = Objects.requireNonNull(library, "library");

        Map<Integer, TagPose> tmp = new HashMap<>();
        for (AprilTagMetadata meta : library.getAllTags()) {
            TagPose pose = toTagPose(meta);
            if (pose != null) {
                tmp.put(pose.id, pose);
            }
        }

        this.posesById = Collections.unmodifiableMap(tmp);
        this.ids = Collections.unmodifiableSet(new HashSet<>(tmp.keySet()));
    }

    /**
     * Returns the underlying FTC library used to build this layout.
     *
     * <p>Exposed for debugging / telemetry (e.g., printing tag names or sizes) without forcing
     * FTC SDK types into core {@code fw.field} APIs.</p>
     */
    public AprilTagLibrary getLibrary() {
        return library;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public TagPose get(int id) {
        return posesById.get(id);
    }

    /** {@inheritDoc} */
    @Override
    public Set<Integer> ids() {
        return ids;
    }

    /**
     * Emit debug information about this tag layout.
     *
     * @param dbg    debug sink to write to; if {@code null}, this method does nothing
     * @param prefix key prefix to use; if {@code null} or empty, {@code "tagLayout"} is used
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "tagLayout" : prefix;

        dbg.addLine(p + ": FtcGameTagLayout");
        dbg.addData(p + ".tagCount", ids.size());
        dbg.addData(p + ".ids", ids.toString());
        dbg.addData(p + ".library.class", library.getClass().getSimpleName());
    }

    private static TagPose toTagPose(AprilTagMetadata meta) {
        if (meta == null) {
            return null;
        }

        VectorF pos = meta.fieldPosition;
        Quaternion q = meta.fieldOrientation;
        if (pos == null || q == null) {
            return null;
        }

        // Defensive: VectorF is expected to have 3 entries (x, y, z).
        final double xRaw;
        final double yRaw;
        final double zRaw;
        try {
            xRaw = pos.get(0);
            yRaw = pos.get(1);
            zRaw = pos.get(2);
        } catch (RuntimeException e) {
            return null;
        }

        DistanceUnit unit = (meta.distanceUnit != null) ? meta.distanceUnit : DistanceUnit.INCH;

        // Phoenix stores all field distances in inches.
        double xInches = unit.toInches(xRaw);
        double yInches = unit.toInches(yRaw);
        double zInches = unit.toInches(zRaw);

        // Convert quaternion -> rotation matrix -> yaw/pitch/roll.
        Mat3 rFieldToTag = rotationFromQuaternion(q);
        Mat3.YawPitchRoll ypr = Mat3.toYawPitchRoll(rFieldToTag);

        Pose3d fieldToTag = new Pose3d(xInches, yInches, zInches, ypr.yawRad, ypr.pitchRad, ypr.rollRad);
        return TagPose.ofPose(meta.id, fieldToTag);
    }

    /**
     * Convert an FTC quaternion (w,x,y,z) to a rotation matrix.
     *
     * <p>This uses the standard right-handed unit quaternion to DCM conversion.</p>
     *
     * @param q quaternion
     * @return rotation matrix
     */
    private static Mat3 rotationFromQuaternion(Quaternion q) {
        // FTC Quaternion components are (w, x, y, z).
        double w = q.w;
        double x = q.x;
        double y = q.y;
        double z = q.z;

        double xx = x * x;
        double yy = y * y;
        double zz = z * z;

        double xy = x * y;
        double xz = x * z;
        double yz = y * z;

        double wx = w * x;
        double wy = w * y;
        double wz = w * z;

        double m00 = 1.0 - 2.0 * (yy + zz);
        double m01 = 2.0 * (xy - wz);
        double m02 = 2.0 * (xz + wy);

        double m10 = 2.0 * (xy + wz);
        double m11 = 1.0 - 2.0 * (xx + zz);
        double m12 = 2.0 * (yz - wx);

        double m20 = 2.0 * (xz - wy);
        double m21 = 2.0 * (yz + wx);
        double m22 = 1.0 - 2.0 * (xx + yy);

        return new Mat3(
                m00, m01, m02,
                m10, m11, m12,
                m20, m21, m22
        );
    }
}
