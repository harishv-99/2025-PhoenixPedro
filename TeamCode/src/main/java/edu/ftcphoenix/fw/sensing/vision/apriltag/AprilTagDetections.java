package edu.ftcphoenix.fw.sensing.vision.apriltag;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Objects;
import java.util.Set;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;

/**
 * Immutable snapshot of the AprilTag detections produced from one processed camera frame.
 *
 * <p>All observations in {@link #observations} come from the frame identified by
 * {@link #frameTimestamp()}. Keeping the reset epoch and time coordinate together prevents a
 * retained frame from becoming plausible again after {@link LoopClock#reset(double)}.</p>
 *
 * <h2>Why Phoenix returns all detections</h2>
 * <ul>
 *   <li><b>Selection should be explicit:</b> the framework does not hide a global “best tag”
 *       policy inside the sensor boundary.</li>
 *   <li><b>Localization may use more than one tag:</b> a pose estimator can combine several fixed
 *       tags from the same frame to reduce noise.</li>
 *   <li><b>Telemetry should be honest:</b> drivers can see which tags are visible, which tag a
 *       selector preview would choose, and which tag is currently latched for aiming.</li>
 * </ul>
 */
public final class AprilTagDetections {

    private static final AprilTagDetections NONE = new AprilTagDetections(
            LoopTimestamp.unavailable(),
            Collections.<AprilTagObservation>emptyList()
    );

    private final LoopTimestamp frameTimestamp;

    /** Immutable observations from the frame, expressed in Phoenix framing. */
    public final List<AprilTagObservation> observations;

    private AprilTagDetections(LoopTimestamp frameTimestamp,
                               List<AprilTagObservation> observations) {
        this.frameTimestamp = frameTimestamp;
        this.observations = observations;
    }

    /**
     * Returns an empty snapshot when there is no trustworthy processed frame.
     *
     * <p>This value has an unavailable frame timestamp and is never fresh.</p>
     */
    public static AprilTagDetections none() {
        return NONE;
    }

    /**
     * Creates one immutable frame snapshot.
     *
     * <p>Every observation must carry the same available, current-epoch frame timestamp. This
     * preserves the class's one-frame contract while camera backends still own how frame identity
     * is acquired.</p>
     *
     * @param frameTimestamp timestamp of the processed frame
     * @param observations   detections from that frame; may be empty but must not be null
     * @return immutable detections snapshot
     */
    public static AprilTagDetections of(LoopTimestamp frameTimestamp,
                                        List<AprilTagObservation> observations) {
        requireAvailableTimestamp(frameTimestamp);
        Objects.requireNonNull(observations, "observations");
        ArrayList<AprilTagObservation> copy =
                new ArrayList<AprilTagObservation>(observations.size());
        for (AprilTagObservation observation : observations) {
            AprilTagObservation obs = Objects.requireNonNull(
                    observation,
                    "observations must not contain null"
            );
            if (!obs.hasTarget) {
                throw new IllegalArgumentException(
                        "observations must contain detected targets, not noTarget()");
            }
            double offsetSec = obs.frameTimestamp().secondsSince(frameTimestamp);
            if (!Double.isFinite(offsetSec) || offsetSec != 0.0) {
                throw new IllegalArgumentException(
                        "every AprilTag observation must use the detections frameTimestamp");
            }
            copy.add(obs);
        }
        return new AprilTagDetections(
                frameTimestamp,
                Collections.unmodifiableList(copy)
        );
    }

    /** Returns this snapshot's camera-frame timestamp. */
    public LoopTimestamp frameTimestamp() {
        return frameTimestamp;
    }

    /**
     * Returns the current age of this snapshot's camera frame.
     *
     * @param clock the same stable loop clock that created the timestamp
     * @return non-negative age in seconds, or {@code NaN} when unavailable or reset-invalidated
     */
    public double frameAgeSec(LoopClock clock) {
        return frameTimestamp.ageSec(clock);
    }

    /** Returns whether this frame is no older than the inclusive maximum age. */
    public boolean isFresh(LoopClock clock, double maxAgeSec) {
        return frameTimestamp.isFresh(clock, maxAgeSec);
    }

    /** Returns the first fresh observation for {@code id}, or no target. */
    public AprilTagObservation forId(LoopClock clock, int id, double maxAgeSec) {
        if (!isFresh(clock, maxAgeSec)) {
            return AprilTagObservation.noTarget();
        }
        for (AprilTagObservation obs : observations) {
            if (obs.id == id) {
                return obs;
            }
        }
        return AprilTagObservation.noTarget();
    }

    /** Returns all fresh observations, preserving frame order. */
    public List<AprilTagObservation> freshObservations(LoopClock clock, double maxAgeSec) {
        if (!isFresh(clock, maxAgeSec)) {
            return Collections.emptyList();
        }
        return observations;
    }

    /** Returns all fresh observations whose IDs are contained in {@code idsOfInterest}. */
    public List<AprilTagObservation> freshMatching(LoopClock clock,
                                                   Set<Integer> idsOfInterest,
                                                   double maxAgeSec) {
        Objects.requireNonNull(idsOfInterest, "idsOfInterest");
        if (idsOfInterest.isEmpty() || !isFresh(clock, maxAgeSec) || observations.isEmpty()) {
            return Collections.emptyList();
        }
        ArrayList<AprilTagObservation> out = new ArrayList<AprilTagObservation>();
        for (AprilTagObservation obs : observations) {
            if (idsOfInterest.contains(obs.id)) {
                out.add(obs);
            }
        }
        return Collections.unmodifiableList(out);
    }

    /** Returns the set of fresh visible IDs. */
    public Set<Integer> visibleIds(LoopClock clock, double maxAgeSec) {
        if (!isFresh(clock, maxAgeSec) || observations.isEmpty()) {
            return Collections.emptySet();
        }
        LinkedHashSet<Integer> ids = new LinkedHashSet<Integer>();
        for (AprilTagObservation obs : observations) {
            ids.add(obs.id);
        }
        return Collections.unmodifiableSet(ids);
    }

    private static void requireAvailableTimestamp(LoopTimestamp timestamp) {
        if (timestamp == null || !timestamp.isAvailable()) {
            throw new IllegalArgumentException(
                    "frameTimestamp must be an available LoopTimestamp created by LoopClock");
        }
    }

    @Override
    public String toString() {
        return "AprilTagDetections{frameTimestamp=" + frameTimestamp
                + ", observations=" + observations + "}";
    }
}
