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
 * {@link TagLayout} backed by the FTC SDK's {@link AprilTagLibrary}.
 *
 * <p>This is the normal bridge from FTC game metadata into Phoenix field-landmark metadata. It
 * reads every tag from the supplied library, converts the FTC pose/orientation representation into
 * Phoenix {@link Pose3d}, and stores the result as immutable field metadata.</p>
 *
 * <h2>Typical usage</h2>
 *
 * <pre>{@code
 * TagLayout layout = new FtcGameTagLayout(AprilTagGameDatabase.getCurrentGameTagLibrary());
 *
 * Pose3d fieldToTag5 = layout.requireFieldToTagPose(5);
 * }</pre>
 *
 * <p>Most robot code should treat this as static setup-time data and inject it anywhere field-fixed
 * AprilTag metadata is needed (localizers, guidance, calibration tools, etc.).</p>
 */
public final class FtcGameTagLayout implements TagLayout {

    private final AprilTagLibrary library;
    private final Map<Integer, Pose3d> posesById;
    private final Set<Integer> ids;

    /**
     * Converts an FTC SDK AprilTag library into immutable Phoenix field metadata.
     *
     * @param library FTC SDK library containing field tag metadata
     */
    public FtcGameTagLayout(AprilTagLibrary library) {
        this.library = Objects.requireNonNull(library, "library");

        Map<Integer, Pose3d> tmp = new HashMap<>();
        for (AprilTagMetadata meta : library.getAllTags()) {
            Pose3d pose = toFieldToTagPose(meta);
            if (pose != null) {
                tmp.put(meta.id, pose);
            }
        }

        this.posesById = Collections.unmodifiableMap(tmp);
        this.ids = Collections.unmodifiableSet(new HashSet<>(tmp.keySet()));
    }

    /**
     * Returns the underlying FTC SDK library.
     *
     * <p>This is mainly useful for advanced tools that want the original FTC metadata in addition
     * to the Phoenix-converted poses.</p>
     */
    public AprilTagLibrary getLibrary() {
        return library;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public Pose3d getFieldToTagPose(int id) {
        return posesById.get(id);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public Set<Integer> ids() {
        return ids;
    }

    /**
     * Emits a compact summary of the converted library.
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

    /**
     * Converts a single FTC SDK tag metadata record into a Phoenix {@code field -> tag} pose.
     *
     * <p>Returns {@code null} when the FTC metadata is incomplete or malformed.</p>
     */
    private static Pose3d toFieldToTagPose(AprilTagMetadata meta) {
        if (meta == null) {
            return null;
        }

        VectorF pos = meta.fieldPosition;
        Quaternion q = meta.fieldOrientation;
        if (pos == null || q == null) {
            return null;
        }

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
        double xInches = unit.toInches(xRaw);
        double yInches = unit.toInches(yRaw);
        double zInches = unit.toInches(zRaw);

        Mat3 rFieldToTag = rotationFromQuaternion(q);
        Mat3.YawPitchRoll ypr = Mat3.toYawPitchRoll(rFieldToTag);

        return new Pose3d(xInches, yInches, zInches, ypr.yawRad, ypr.pitchRad, ypr.rollRad);
    }

    /**
     * Converts an FTC SDK quaternion into a Phoenix rotation matrix.
     */
    private static Mat3 rotationFromQuaternion(Quaternion q) {
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
