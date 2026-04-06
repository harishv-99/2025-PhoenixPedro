package edu.ftcphoenix.fw.ftc;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.TreeSet;

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

    /**
     * Known ancillary FTC SDK sample-tag IDs that may be present in broad/default libraries.
     *
     * <p>These tags are useful for detector bring-up and sample OpModes, but they are not part of
     * an official game's field-fixed layout policy.</p>
     */
    private static final Set<Integer> KNOWN_ANCILLARY_LIBRARY_IDS = immutableIds(583, 584, 585, 586);

    private static final OfficialGamePolicy DECODE_2025_2026 = new OfficialGamePolicy(
            "DECODE 2025-2026",
            immutableIds(20, 21, 22, 23, 24),
            immutableIds(20, 24)
    );

    private static final List<OfficialGamePolicy> OFFICIAL_GAME_POLICIES = Collections.unmodifiableList(
            Arrays.asList(DECODE_2025_2026)
    );

    private static final class OfficialGamePolicy {
        final String seasonName;
        final Set<Integer> officialGameIds;
        final Set<Integer> fixedFieldIds;
        final Set<Integer> nonFixedGameIds;

        OfficialGamePolicy(String seasonName,
                           Set<Integer> officialGameIds,
                           Set<Integer> fixedFieldIds) {
            this.seasonName = Objects.requireNonNull(seasonName, "seasonName").trim();
            this.officialGameIds = orderedImmutableIds(Objects.requireNonNull(officialGameIds, "officialGameIds"));
            this.fixedFieldIds = orderedImmutableIds(Objects.requireNonNull(fixedFieldIds, "fixedFieldIds"));

            if (!this.officialGameIds.containsAll(this.fixedFieldIds)) {
                throw new IllegalArgumentException(
                        "Official game fixed-field IDs must be a subset of official game IDs"
                );
            }
            this.nonFixedGameIds = orderedImmutableIds(difference(this.officialGameIds, this.fixedFieldIds));
        }

        boolean matchesNormalizedLibrary(Set<Integer> normalizedLibraryIds) {
            return officialGameIds.equals(normalizedLibraryIds);
        }

        String sourceDescription() {
            return "official FTC game (" + seasonName + " fixed-field tags)";
        }
    }

    private final AprilTagLibrary library;
    private final Map<Integer, Pose3d> posesById;
    private final Set<Integer> ids;
    private final Set<Integer> libraryIds;
    private final Set<Integer> excludedIds;
    private final Set<Integer> officialGameIds;
    private final Set<Integer> nonFixedGameIds;
    private final Set<Integer> ancillaryLibraryIds;
    private final String sourceDescription;

    private FtcGameTagLayout(AprilTagLibrary library,
                             Set<Integer> includedIds,
                             Set<Integer> officialGameIds,
                             Set<Integer> nonFixedGameIds,
                             Set<Integer> ancillaryLibraryIds,
                             String sourceDescription) {
        this.library = Objects.requireNonNull(library, "library");
        this.sourceDescription = (sourceDescription != null && !sourceDescription.trim().isEmpty())
                ? sourceDescription.trim()
                : "unspecified";

        Set<Integer> sourceIds = allLibraryIds(library);
        Set<Integer> requested = orderedImmutableIds(includedIds);
        Set<Integer> officialIds = orderedImmutableIds(officialGameIds);
        Set<Integer> nonFixedIds = orderedImmutableIds(nonFixedGameIds);
        Set<Integer> ancillaryIds = orderedImmutableIds(ancillaryLibraryIds);

        validateRequestedIds(requested, sourceIds);
        validateRequestedIds(officialIds, sourceIds);
        validateRequestedIds(nonFixedIds, sourceIds);
        validateRequestedIds(ancillaryIds, sourceIds);

        if (!officialIds.containsAll(nonFixedIds)) {
            throw new IllegalArgumentException(
                    "nonFixedGameIds must be a subset of officialGameIds"
            );
        }

        HashSet<Integer> overlap = new HashSet<Integer>(officialIds);
        overlap.retainAll(ancillaryIds);
        if (!overlap.isEmpty()) {
            throw new IllegalArgumentException(
                    "Official game IDs and ancillary library IDs must be disjoint, overlap=" + overlap
            );
        }

        Map<Integer, Pose3d> tmp = new HashMap<Integer, Pose3d>();
        for (AprilTagMetadata meta : library.getAllTags()) {
            if (meta == null || !requested.contains(meta.id)) {
                continue;
            }
            Pose3d pose = requireFieldToTagPose(meta);
            tmp.put(meta.id, pose);
        }

        this.posesById = Collections.unmodifiableMap(tmp);
        this.ids = orderedImmutableIds(tmp.keySet());
        this.libraryIds = orderedImmutableIds(sourceIds);
        this.officialGameIds = officialIds;
        this.nonFixedGameIds = nonFixedIds;
        this.ancillaryLibraryIds = ancillaryIds;

        this.excludedIds = orderedImmutableIds(difference(sourceIds, this.ids));
    }

    /**
     * Creates the framework's best-known fixed-field layout for the FTC SDK's current-game
     * library.
     *
     * <p>This intentionally fails fast for unknown current-game libraries instead of silently
     * treating every SDK tag as fixed. If FIRST changes the current game and Phoenix has not yet
     * been updated with the season's fixed-tag policy, the error message will tell the maintainer
     * exactly what to do.</p>
     *
     * <p>The FTC SDK's current-game library may also include ancillary/sample tags used by SDK
     * examples. Those are normalized away before the official-game signature is matched so the
     * framework does not break just because the detector library is broader than the fixed-field
     * layout policy.</p>
     */
    public static FtcGameTagLayout currentGameFieldFixed() {
        return officialGameFieldFixed(AprilTagGameDatabase.getCurrentGameTagLibrary());
    }

    /**
     * Creates a fixed-field layout for a known official FTC game library using Phoenix's curated
     * season knowledge.
     *
     * <p>Use this when you intentionally pass an archived official game library. If Phoenix does
     * not recognize the library's normalized game-ID signature yet, this method throws with an
     * actionable message instead of guessing.</p>
     */
    public static FtcGameTagLayout officialGameFieldFixed(AprilTagLibrary library) {
        Objects.requireNonNull(library, "library");
        Set<Integer> rawLibraryIds = allLibraryIds(library);
        Set<Integer> ancillaryIds = orderedImmutableIds(intersection(rawLibraryIds, KNOWN_ANCILLARY_LIBRARY_IDS));
        Set<Integer> normalizedGameIds = orderedImmutableIds(difference(rawLibraryIds, ancillaryIds));

        for (OfficialGamePolicy policy : OFFICIAL_GAME_POLICIES) {
            if (policy.matchesNormalizedLibrary(normalizedGameIds)) {
                return new FtcGameTagLayout(
                        library,
                        policy.fixedFieldIds,
                        policy.officialGameIds,
                        policy.nonFixedGameIds,
                        ancillaryIds,
                        policy.sourceDescription()
                );
            }
        }

        throw new IllegalArgumentException(
                "Phoenix does not have a built-in fixed-tag policy for FTC game library ids="
                        + rawLibraryIds
                        + " (normalizedGameIds=" + normalizedGameIds
                        + ", ancillaryIds=" + ancillaryIds
                        + "). Use FtcGameTagLayout.fromLibraryFixedIds(...) for an explicit layout,"
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
        return new FtcGameTagLayout(
                library,
                fixedIds,
                Collections.<Integer>emptySet(),
                Collections.<Integer>emptySet(),
                Collections.<Integer>emptySet(),
                sourceDescription
        );
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
        return new FtcGameTagLayout(
                library,
                allLibraryIds(library),
                Collections.<Integer>emptySet(),
                Collections.<Integer>emptySet(),
                Collections.<Integer>emptySet(),
                "all tags from supplied library"
        );
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
     * <p>This union may include both:</p>
     * <ul>
     *   <li>official game IDs that Phoenix intentionally does not trust as fixed landmarks, and</li>
     *   <li>ancillary SDK IDs that are present in the detector library but are not part of the
     *       official field layout policy.</li>
     * </ul>
     */
    public Set<Integer> excludedIds() {
        return excludedIds;
    }

    /**
     * Returns whether this layout was created by a recognized official-game fixed-tag policy.
     */
    public boolean hasOfficialGamePolicy() {
        return !officialGameIds.isEmpty();
    }

    /**
     * Returns recognized official-game IDs from the source library, if this layout came from an
     * official-game policy. Otherwise returns an empty set.
     */
    public Set<Integer> officialGameIds() {
        return officialGameIds;
    }

    /**
     * Returns recognized official-game IDs that Phoenix intentionally excludes from the fixed-field
     * layout because they are not stable enough for localization / field-pose solving.
     */
    public Set<Integer> nonFixedGameIds() {
        return nonFixedGameIds;
    }

    /**
     * Returns ancillary source-library IDs (for example SDK sample tags) that were ignored while
     * matching the official-game policy.
     */
    public Set<Integer> ancillaryLibraryIds() {
        return ancillaryLibraryIds;
    }

    /**
     * Returns a human-readable description of how this layout was chosen.
     */
    public String sourceDescription() {
        return sourceDescription;
    }

    /**
     * Returns a compact human-readable summary of the fixed-tag policy used by this layout.
     */
    public String policySummaryLine() {
        if (!hasOfficialGamePolicy()) {
            return sourceDescription
                    + " | fixed=" + ids
                    + (excludedIds.isEmpty() ? "" : " | excluded=" + excludedIds);
        }

        return sourceDescription
                + " | fixed=" + ids
                + " | nonFixedGame=" + nonFixedGameIds
                + (ancillaryLibraryIds.isEmpty() ? "" : " | ancillary=" + ancillaryLibraryIds);
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
        dbg.addData(p + ".hasOfficialGamePolicy", hasOfficialGamePolicy());
        if (hasOfficialGamePolicy()) {
            dbg.addData(p + ".officialGameIds", officialGameIds.toString());
            dbg.addData(p + ".nonFixedGameIds", nonFixedGameIds.toString());
            dbg.addData(p + ".ancillaryLibraryIds", ancillaryLibraryIds.toString());
        }
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
        return orderedImmutableIds(ids);
    }

    private static Set<Integer> immutableIds(int... ids) {
        HashSet<Integer> set = new HashSet<Integer>();
        if (ids != null) {
            for (int id : ids) {
                set.add(id);
            }
        }
        return orderedImmutableIds(set);
    }

    private static Set<Integer> orderedImmutableIds(Set<Integer> ids) {
        TreeSet<Integer> sorted = new TreeSet<Integer>();
        if (ids != null) {
            sorted.addAll(ids);
        }
        return Collections.unmodifiableSet(new LinkedHashSet<Integer>(sorted));
    }

    private static Set<Integer> difference(Set<Integer> a, Set<Integer> b) {
        HashSet<Integer> out = new HashSet<Integer>();
        if (a != null) {
            out.addAll(a);
        }
        if (b != null) {
            out.removeAll(b);
        }
        return out;
    }

    private static Set<Integer> intersection(Set<Integer> a, Set<Integer> b) {
        HashSet<Integer> out = new HashSet<Integer>();
        if (a != null) {
            out.addAll(a);
        }
        if (b == null) {
            out.clear();
            return out;
        }
        out.retainAll(b);
        return out;
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
                + (hasOfficialGamePolicy() ? ", nonFixedGameIds=" + nonFixedGameIds : "")
                + (ancillaryLibraryIds.isEmpty() ? "" : ", ancillaryLibraryIds=" + ancillaryLibraryIds)
                + '}';
    }
}
