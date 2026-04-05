package edu.ftcphoenix.fw.field;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.geometry.Pose3d;

/**
 * Simple in-memory implementation of {@link TagLayout}.
 *
 * <p>{@link SimpleTagLayout} is the easiest way to author a handful of fixed tags for tests,
 * practice fields, calibration tools, or custom off-season games. It is intentionally mutable so
 * setup code can read fluently, while consumers still view it through the immutable
 * {@link TagLayout} interface.</p>
 *
 * <h2>Common usage</h2>
 *
 * <pre>{@code
 * TagLayout layout = new SimpleTagLayout()
 *         .add(5, 48.0, 0.0, 18.0, Math.PI, 0.0, 0.0)
 *         .add(6, 48.0, 24.0, 18.0, Math.PI, 0.0, 0.0);
 *
 * Pose3d fieldToTag5 = layout.requireFieldToTagPose(5);
 * }</pre>
 *
 * <p>Like all {@link TagLayout} implementations, poses are stored as full 6DOF {@link Pose3d}
 * values in FTC field coordinates.</p>
 */
public final class SimpleTagLayout implements TagLayout {

    private final Map<Integer, Pose3d> byId = new HashMap<>();

    /**
     * Creates an empty layout.
     */
    public SimpleTagLayout() {
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public Pose3d getFieldToTagPose(int id) {
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
     * Adds or replaces a tag placement using a pre-built pose object.
     *
     * @param id AprilTag numeric ID code (must be non-negative)
     * @param fieldToTagPose tag pose in the FTC field frame
     * @return {@code this} for fluent setup code
     */
    public SimpleTagLayout addPose(int id, Pose3d fieldToTagPose) {
        if (id < 0) {
            throw new IllegalArgumentException("id must be non-negative");
        }
        if (fieldToTagPose == null) {
            throw new IllegalArgumentException("fieldToTagPose must not be null");
        }
        byId.put(id, fieldToTagPose);
        return this;
    }

    /**
     * Adds or replaces a tag placement from the standard six pose values.
     *
     * <p>This is a convenience wrapper around {@link #addPose(int, Pose3d)}.</p>
     *
     * @param id AprilTag numeric ID code
     * @param xInches tag center X in the FTC field frame
     * @param yInches tag center Y in the FTC field frame
     * @param zInches tag center Z (height) in the FTC field frame
     * @param yawRad tag yaw in the FTC field frame
     * @param pitchRad tag pitch in the FTC field frame
     * @param rollRad tag roll in the FTC field frame
     * @return {@code this} for fluent setup code
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
        return addPose(id, new Pose3d(xInches, yInches, zInches, yawRad, pitchRad, rollRad));
    }

    /**
     * Removes a tag from the layout.
     *
     * @param id AprilTag numeric ID code
     * @return {@code true} if a tag was present and removed
     */
    public boolean remove(int id) {
        return byId.remove(id) != null;
    }

    /**
     * Removes all tags from the layout.
     */
    public void clear() {
        byId.clear();
    }

    /**
     * @return number of tag entries currently stored
     */
    public int size() {
        return byId.size();
    }

    /**
     * Emits a stable debug dump of the layout contents.
     *
     * <p>This is useful when validating a practice-field setup or confirming that a converted FTC
     * game layout contains the IDs you expect.</p>
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "tagLayout" : prefix;

        dbg.addData(p + ".type", "SimpleTagLayout");
        dbg.addData(p + ".count", byId.size());
        dbg.addData(p + ".ids", ids().toString());

        for (Integer id : ids()) {
            Pose3d pose = byId.get(id);
            if (pose == null) {
                continue;
            }

            String k = p + ".tag[" + id + "]";
            dbg.addData(k + ".xInches", pose.xInches);
            dbg.addData(k + ".yInches", pose.yInches);
            dbg.addData(k + ".zInches", pose.zInches);
            dbg.addData(k + ".yawRad", pose.yawRad);
            dbg.addData(k + ".pitchRad", pose.pitchRad);
            dbg.addData(k + ".rollRad", pose.rollRad);
        }
    }

    @Override
    public String toString() {
        return "SimpleTagLayout{ids=" + ids() + "}";
    }
}
