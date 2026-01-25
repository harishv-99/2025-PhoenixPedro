package edu.ftcphoenix.fw.field;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.geometry.Pose3d;

/**
 * Simple in-memory implementation of {@link TagLayout}.
 *
 * <p>Use this when you want to provide your own tag placements (for example, for a
 * custom field, practice setup, or unit tests), or when you want a small layout
 * without depending on an FTC-provided database.</p>
 *
 * <h2>Frame and units</h2>
 *
 * <p>All tag poses stored here are <b>field-centric</b> in the FTC field frame:</p>
 * <ul>
 *   <li>Origin: center of the field on the floor</li>
 *   <li>+Z: up from the floor</li>
 *   <li>Distances: inches</li>
 *   <li>Angles: radians</li>
 * </ul>
 *
 * <h2>Rotation conventions</h2>
 *
 * <p>Orientation matches {@link Pose3d}:</p>
 * <ul>
 *   <li><b>yaw</b> is rotation about +Z</li>
 *   <li><b>pitch</b> is rotation about +Y</li>
 *   <li><b>roll</b> is rotation about +X</li>
 * </ul>
 */
public final class SimpleTagLayout implements TagLayout {

    private final Map<Integer, TagPose> byId = new HashMap<>();

    /**
     * Creates an empty layout.
     */
    public SimpleTagLayout() {
    }

    /**
     * Create a layout pre-populated with the provided tag poses.
     *
     * @param tags tag poses to add (may be empty; null elements not allowed)
     * @return new {@link SimpleTagLayout}
     */
    public static SimpleTagLayout of(TagPose... tags) {
        SimpleTagLayout layout = new SimpleTagLayout();
        if (tags != null) {
            for (TagPose t : tags) {
                layout.add(t);
            }
        }
        return layout;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public TagPose get(int id) {
        return byId.get(id);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public Set<Integer> ids() {
        return Collections.unmodifiableSet(byId.keySet());
    }

    /**
     * Add (or replace) a tag pose by object.
     *
     * @param tagPose tag pose to store (non-null)
     * @return {@code this} for chaining
     */
    public SimpleTagLayout add(TagPose tagPose) {
        Objects.requireNonNull(tagPose, "tagPose");
        byId.put(tagPose.id, tagPose);
        return this;
    }

    /**
     * Add (or replace) a tag pose by pose object.
     *
     * @param id         AprilTag numeric ID code
     * @param fieldToTag tag center pose expressed in the FTC field frame (non-null)
     * @return {@code this} for chaining
     */
    public SimpleTagLayout addPose(int id, Pose3d fieldToTag) {
        return add(TagPose.ofPose(id, fieldToTag));
    }

    /**
     * Add (or replace) a tag pose by specifying translation and yaw/pitch/roll.
     *
     * @param id       AprilTag numeric ID code
     * @param xInches  tag center X in the FTC field frame (inches)
     * @param yInches  tag center Y in the FTC field frame (inches)
     * @param zInches  tag center Z (height above floor) in the FTC field frame (inches)
     * @param yawRad   tag yaw about +Z in the FTC field frame (radians)
     * @param pitchRad tag pitch about +Y in the FTC field frame (radians)
     * @param rollRad  tag roll about +X in the FTC field frame (radians)
     * @return {@code this} for chaining
     */
    public SimpleTagLayout add(
            int id,
            double xInches,
            double yInches,
            double zInches,
            double yawRad,
            double pitchRad,
            double rollRad
    ) {
        return add(TagPose.of(id, xInches, yInches, zInches, yawRad, pitchRad, rollRad));
    }

    /**
     * Remove a tag from the layout.
     *
     * @param id AprilTag numeric ID code
     * @return {@code true} if a tag was removed; {@code false} if it was not present
     */
    public boolean remove(int id) {
        return byId.remove(id) != null;
    }

    /**
     * Remove all tags from the layout.
     */
    public void clear() {
        byId.clear();
    }

    /**
     * Number of tags currently in the layout.
     */
    public int size() {
        return byId.size();
    }

    /**
     * Debug-dump all tag poses in this layout.
     *
     * <p>Framework convention: this is a no-op when {@code dbg == null}.</p>
     *
     * @param dbg    debug sink (may be null)
     * @param prefix prefix for keys (may be null/empty)
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "tagLayout" : prefix;

        dbg.addData(p + ".type", "SimpleTagLayout");
        dbg.addData(p + ".count", byId.size());
        dbg.addData(p + ".ids", ids().toString());

        // Dump each pose with stable key names.
        for (Integer id : ids()) {
            TagPose t = byId.get(id);
            if (t == null) continue;

            String k = p + ".tag[" + id + "]";
            Pose3d pose = t.fieldToTagPose();

            dbg.addData(k + ".xInches", pose.xInches);
            dbg.addData(k + ".yInches", pose.yInches);
            dbg.addData(k + ".zInches", pose.zInches);
            dbg.addData(k + ".yawRad", pose.yawRad);
            dbg.addData(k + ".pitchRad", pose.pitchRad);
            dbg.addData(k + ".rollRad", pose.rollRad);
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        return "SimpleTagLayout{ids=" + ids() + "}";
    }
}
