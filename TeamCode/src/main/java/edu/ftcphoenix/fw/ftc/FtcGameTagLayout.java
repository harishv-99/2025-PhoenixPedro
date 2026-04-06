package edu.ftcphoenix.fw.ftc;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
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
 * <p>Important distinction:</p>
 * <ul>
 *   <li>The FTC SDK {@link AprilTagLibrary} may contain <em>every tag the detector should know
 *       about</em>, including tags that are useful for identification but are <em>not</em> safe to
 *       treat as fixed field landmarks.</li>
 *   <li>{@link TagLayout}, by contract, should contain only tags whose field placement is truly
 *       fixed and may therefore be promoted through localization and AprilTag→field-pose solving.</li>
 * </ul>
 *
 * <p>To keep that distinction explicit, this class is created through named factory methods rather
 * than a bare constructor. For official FTC games, prefer {@link #currentGameFieldFixed()}; for
 * advanced cases, use {@link #fromLibraryFixedIds(AprilTagLibrary, Set)} or
 * {@link #fromLibraryAllTags(AprilTagLibrary)}.</p>
 */
public final class FtcGameTagLayout implements TagLayout {

    private static final Set<Integer> DECODE_2025_2026_LIBRARY_IDS =
            immutableIds(20, 21, 22, 23, 24);

    private static final Set<Integer> DECODE_2025_2026_FIXED_FIELD_IDS =
            immutableIds(20, 24);

    private final AprilTagLibrary library;
    private final Map<Integer, Pose3d> posesById;
    private final Set<Integer> ids;
    private final Set<Integer> libraryIds;
    private final Set<Integer> excludedIds;
    private final String sourceDescription;

    private FtcGameTagLayout(AprilTagLibrary library,
                             Set<Integer> includedIds,
                             String sourceDescription) {
        this.library = Objects.requireNonNull(library, "library");
        this.sourceDescription = (sourceDescription != null && !sourceDescription.trim().isEmpty())
                ? sourceDescription.trim()
                : "unspecified";

        Set<Integer> requested = (includedIds != null)
                ? Collections.unmodifiableSet(new HashSet<Integer>(includedIds))
                : Collections.<Integer>emptySet();

        Set<Integer> sourceIds = allLibraryIds(library);
        validateRequestedIds(requested, sourceIds);

        Map<Integer, Pose3d> tmp = new HashMap<Integer, Pose3d>();
        for (AprilTagMetadata meta : library.getAllTags()) {
            if (meta == null || !requested.contains(meta.id)) {
                continue;
            }
            Pose3d pose = requireFieldToTagPose(meta);
            tmp.put(meta.id, pose);
        }

        this.posesById = Collections.unmodifiableMap(tmp);
        this.ids = Collections.unmodifiableSet(new HashSet<Integer>(tmp.keySet()));
        this.libraryIds = Collections.unmodifiableSet(new HashSet<Integer>(sourceIds));

        HashSet<Integer> excluded = new HashSet<Integer>(sourceIds);
        excluded.removeAll(this.ids);
        this.excludedIds = Collections.unmodifiableSet(excluded);
    }

    /**
     * Creates the framework's best-known fixed-field layout for the FTC SDK's current-game
     * library.
     *
     * <p>This intentionally fails fast for unknown current-game libraries instead of silently
     * treating every SDK tag as fixed. If FIRST changes the current game and Phoenix has not yet
     * been updated with the season's fixed-tag policy, the error message will tell the maintainer
     * exactly what to do.</p>
     */
    public static FtcGameTagLayout currentGameFieldFixed() {
        return officialGameFieldFixed(AprilTagGameDatabase.getCurrentGameTagLibrary());
    }

    /**
     * Creates a fixed-field layout for a known official FTC game library using Phoenix's curated
     * season knowledge.
     *
     * <p>Use this when you intentionally pass an archived official game library. If Phoenix does
     * not recognize the library's ID signature yet, this method throws with an actionable message
     * instead of guessing.</p>
     */
    public static FtcGameTagLayout officialGameFieldFixed(AprilTagLibrary library) {
        Objects.requireNonNull(library, "library");
        Set<Integer> libraryIds = allLibraryIds(library);

        if (libraryIds.equals(DECODE_2025_2026_LIBRARY_IDS)) {
            return fromLibraryFixedIds(
                    library,
                    DECODE_2025_2026_FIXED_FIELD_IDS,
                    "official FTC game (DECODE 2025-2026 fixed-field tags)"
            );
        }

        throw new IllegalArgumentException(
                "Phoenix does not have a built-in fixed-tag policy for FTC game library ids="
                        + libraryIds
                        + ". Use FtcGameTagLayout.fromLibraryFixedIds(...) for an explicit layout,"
                        + " or update FtcGameTagLayout.officialGameFieldFixed(...) for this season."
        );
    }

    /**
     * Creates a layout from an FTC SDK library by explicitly naming which IDs are safe to treat as
     * fixed field landmarks.
     */
    public static FtcGameTagLayout fromLibraryFixedIds(AprilTagLibrary library, Set<Integer> fixedIds) {
        return fromLibraryFixedIds(library, fixedIds, "explicit fixed-id set");
    }

    /**
     * Creates a layout from an FTC SDK library by explicitly naming which IDs are safe to treat as
     * fixed field landmarks, while also recording a human-readable source description.
     */
    public static FtcGameTagLayout fromLibraryFixedIds(AprilTagLibrary library,
                                                       Set<Integer> fixedIds,
                                                       String sourceDescription) {
        Objects.requireNonNull(fixedIds, "fixedIds");
        return new FtcGameTagLayout(library, fixedIds, sourceDescription);
    }

    /**
     * Creates a layout from every tag in the supplied library.
     *
     * <p>This is an advanced escape hatch. Use it only when you already know every tag in the
     * library is truly fixed in the field or on your practice rig. For official FTC games, prefer
     * {@link #currentGameFieldFixed()} or {@link #officialGameFieldFixed(AprilTagLibrary)}.</p>
     */
    public static FtcGameTagLayout fromLibraryAllTags(AprilTagLibrary library) {
        Objects.requireNonNull(library, "library");
        return new FtcGameTagLayout(library, allLibraryIds(library), "all tags from supplied library");
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
     * Returns every tag ID present in the source FTC SDK library, including IDs intentionally
     * excluded from this fixed-field layout.
     */
    public Set<Integer> libraryIds() {
        return libraryIds;
    }

    /**
     * Returns source-library IDs that were intentionally excluded from this fixed-field layout.
     *
     * <p>For official FTC games, these are usually tags whose placement is not deterministic enough
     * to trust for localization.</p>
     */
    public Set<Integer> excludedIds() {
        return excludedIds;
    }

    /**
     * Returns a human-readable description of how this layout was chosen.
     */
    public String sourceDescription() {
        return sourceDescription;
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
     * Emits a compact summary of the converted layout.
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "tagLayout" : prefix;

        dbg.addLine(p + ": FtcGameTagLayout");
        dbg.addData(p + ".sourceDescription", sourceDescription);
        dbg.addData(p + ".fixedTagCount", ids.size());
        dbg.addData(p + ".fixedIds", ids.toString());
        dbg.addData(p + ".sourceTagCount", libraryIds.size());
        dbg.addData(p + ".sourceIds", libraryIds.toString());
        dbg.addData(p + ".excludedIds", excludedIds.toString());
        dbg.addData(p + ".library.class", library.getClass().getSimpleName());
    }

    /**
     * Converts a single FTC SDK tag metadata record into a Phoenix {@code field -> tag} pose.
     *
     * <p>Throws when the FTC metadata is incomplete or malformed for a tag the caller explicitly
     * requested. This keeps bad field metadata from silently leaking into localization.</p>
     */
    private static Pose3d requireFieldToTagPose(AprilTagMetadata meta) {
        if (meta == null) {
            throw new IllegalArgumentException("AprilTag metadata must not be null");
        }

        VectorF pos = meta.fieldPosition;
        Quaternion q = meta.fieldOrientation;
        if (pos == null || q == null) {
            throw new IllegalArgumentException(
                    "AprilTag metadata for id=" + meta.id + " is missing field pose/orientation"
            );
        }

        final double xRaw;
        final double yRaw;
        final double zRaw;
        try {
            xRaw = pos.get(0);
            yRaw = pos.get(1);
            zRaw = pos.get(2);
        } catch (RuntimeException e) {
            throw new IllegalArgumentException(
                    "AprilTag metadata for id=" + meta.id + " has malformed fieldPosition",
                    e
            );
        }

        DistanceUnit unit = (meta.distanceUnit != null) ? meta.distanceUnit : DistanceUnit.INCH;
        double xInches = unit.toInches(xRaw);
        double yInches = unit.toInches(yRaw);
        double zInches = unit.toInches(zRaw);

        Mat3 rFieldToTag = rotationFromQuaternion(q);
        Mat3.YawPitchRoll ypr = Mat3.toYawPitchRoll(rFieldToTag);

        return new Pose3d(xInches, yInches, zInches, ypr.yawRad, ypr.pitchRad, ypr.rollRad);
    }

    private static void validateRequestedIds(Set<Integer> requested, Set<Integer> sourceIds) {
        HashSet<Integer> missing = new HashSet<Integer>(requested);
        missing.removeAll(sourceIds);
        if (!missing.isEmpty()) {
            throw new IllegalArgumentException(
                    "Requested fixed tag ids " + missing + " are not present in the supplied FTC library ids="
                            + sourceIds
            );
        }
    }

    private static Set<Integer> allLibraryIds(AprilTagLibrary library) {
        Objects.requireNonNull(library, "library");
        HashSet<Integer> ids = new HashSet<Integer>();
        for (AprilTagMetadata meta : library.getAllTags()) {
            if (meta != null) {
                ids.add(meta.id);
            }
        }
        return Collections.unmodifiableSet(ids);
    }

    private static Set<Integer> immutableIds(int... ids) {
        HashSet<Integer> set = new HashSet<Integer>();
        if (ids != null) {
            for (int id : ids) {
                set.add(id);
            }
        }
        return Collections.unmodifiableSet(set);
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

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        return "FtcGameTagLayout{"
                + "sourceDescription='" + sourceDescription + '\''
                + ", fixedIds=" + ids
                + ", excludedIds=" + excludedIds
                + '}';
    }
}
