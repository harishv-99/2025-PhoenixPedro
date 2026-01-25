package edu.ftcphoenix.fw.sensing.vision.apriltag;

import java.util.Objects;
import java.util.Set;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.sensing.observation.TargetObservation2d;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.CameraMountLogic;

/**
 * Tracks the "best" AprilTag target across loops for a specific set of IDs.
 *
 * <h2>Role</h2>
 * <p>
 * {@code TagTarget} wraps an {@link AprilTagSensor} and remembers the latest
 * {@link AprilTagObservation} that matches a set of tag IDs and a freshness constraint.
 * </p>
 *
 * <p>
 * It is intended to be the single place in your robot code that answers:
 * </p>
 *
 * <ul>
 *   <li>Which tag are we currently tracking?</li>
 *   <li>How old is the observation?</li>
 *   <li>What is the bearing and range to the tag?</li>
 * </ul>
 *
 * <h2>Frame &amp; sign conventions</h2>
 * <p>
 * {@link AprilTagObservation} stores {@code cameraToTagPose} (camera→tag) in Phoenix framing:
 * +X forward, +Y left, +Z up.
 * </p>
 *
 * <ul>
 *   <li><b>Camera-centric bearing</b> is computed from {@code cameraToTagPose} as
 *       {@code atan2(left, forward)} in the camera frame:
 *       {@code bearingRad > 0} means the tag is to the <b>left</b>.</li>
 *   <li><b>Robot-centric bearing</b> (optional) accounts for camera offset using
 *       {@link CameraMountConfig} so the <b>robot center</b> faces the tag.</li>
 * </ul>
 *
 * <h2>Typical usage</h2>
 *
 * <pre>{@code
 * // Wiring in init():
 * AprilTagSensor tagSensor = FtcVision.aprilTags(hardwareMap, "Webcam 1");
 * TagTarget target = new TagTarget(tagSensor, Set.of(1, 2, 3), 0.5);
 *
 * // In loop():
 * target.update(clock);
 * if (target.hasTarget()) {
 *     double cameraBearing = target.bearingRad();
 *     double rangeLosInches = target.lineOfSightRangeInches();
 * }
 * }</pre>
 *
 * <h2>Per-cycle idempotency</h2>
 * <p>
 * {@link #update(LoopClock)} is idempotent by {@link LoopClock#cycle()} to prevent
 * accidental double polling (e.g., multiple layers calling update in the same loop).
 * </p>
 */
public final class TagTarget {

    private final AprilTagSensor sensor;
    private final Set<Integer> idsOfInterest;
    private final double maxAgeSec;

    // Last observation returned by the sensor for this ID set + age constraint.
    private AprilTagObservation lastObs =
            AprilTagObservation.noTarget(Double.POSITIVE_INFINITY);

    /**
     * Tracks which loop cycle we last updated for, to prevent double-polling in a single cycle.
     */
    private long lastUpdatedCycle = Long.MIN_VALUE;

    /**
     * Create a tag target tracker.
     *
     * @param sensor        AprilTag sensor backing this tracker; must not be null
     * @param idsOfInterest set of tag IDs this tracker cares about; must not be null or empty
     * @param maxAgeSec     maximum acceptable age in seconds; observations older than
     *                      this will be reported as {@code hasTarget == false}
     * @throws NullPointerException     if {@code sensor} or {@code idsOfInterest} is null
     * @throws IllegalArgumentException if {@code idsOfInterest} is empty
     * @throws IllegalArgumentException if {@code maxAgeSec} is negative
     */
    public TagTarget(AprilTagSensor sensor, Set<Integer> idsOfInterest, double maxAgeSec) {
        this.sensor = Objects.requireNonNull(sensor, "sensor is required");
        this.idsOfInterest = Objects.requireNonNull(idsOfInterest, "idsOfInterest is required");
        if (idsOfInterest.isEmpty()) {
            throw new IllegalArgumentException("idsOfInterest must not be empty");
        }
        if (maxAgeSec < 0.0) {
            throw new IllegalArgumentException("maxAgeSec must be non-negative");
        }
        this.maxAgeSec = maxAgeSec;
    }

    /**
     * Update the tracked target using the underlying sensor.
     *
     * <p>
     * Call this once per control loop, before reading {@link #last()},
     * {@link #hasTarget()}, {@link #bearingRad()}, etc.
     * </p>
     *
     * <p>This method is idempotent by {@link LoopClock#cycle()}.</p>
     *
     * @param clock loop clock (must not be {@code null})
     */
    public void update(LoopClock clock) {
        Objects.requireNonNull(clock, "clock is required");

        long c = clock.cycle();
        if (c == lastUpdatedCycle) {
            return;
        }
        lastUpdatedCycle = c;

        lastObs = sensor.best(idsOfInterest, maxAgeSec);
    }

    /**
     * Latest observation from this tracker.
     *
     * @return last observation returned by {@link #update(LoopClock)}, or an initial
     * "no target" observation if {@link #update(LoopClock)} has not yet been called
     */
    public AprilTagObservation last() {
        return lastObs;
    }

    /**
     * Whether the tracker currently has a valid target.
     *
     * @return {@code true} if the latest observation has a target (and met the age constraint)
     */
    public boolean hasTarget() {
        return lastObs.hasTarget;
    }

    /**
     * Camera-centric horizontal bearing to the tracked tag, in radians.
     *
     * <p>
     * This bearing is relative to the camera forward axis (derived from {@code cameraToTagPose}).
     * If your camera is offset and you want the <b>robot center</b> to face the tag,
     * use {@link #robotBearingRad(CameraMountConfig)}.
     * </p>
     *
     * @return bearing in radians (positive = left/CCW). Only meaningful when {@link #hasTarget()} is true.
     */
    public double bearingRad() {
        return lastObs.cameraBearingRad();
    }

    /**
     * Convenience alias for {@link #bearingRad()}.
     *
     * <p>Spells out that this is camera-centric bearing.</p>
     *
     * @return camera-centric bearing in radians (positive = left/CCW)
     */
    public double cameraBearingRad() {
        return bearingRad();
    }

    /**
     * Robot-centric bearing to the tracked tag, accounting for camera mount offset.
     *
     * <p>
     * This computes robot-centric bearing by applying the mount extrinsics:
     * {@code robotToTagPose = robotToCameraPose.then(cameraToTagPose)}, then computing
     * {@code atan2(left, forward)} in the robot frame.
     * </p>
     *
     * @param cameraMount robot→camera extrinsics; must not be null
     * @return robot-centric bearing (positive = left/CCW). Returns 0 if {@link #hasTarget()} is false.
     */
    public double robotBearingRad(CameraMountConfig cameraMount) {
        Objects.requireNonNull(cameraMount, "cameraMount is required");
        return CameraMountLogic.robotBearingRad(lastObs, cameraMount);
    }

    /**
     * Check whether the current camera-centric bearing is within a tolerance.
     *
     * @param toleranceRad tolerance in radians (must be {@code >= 0})
     * @return {@code true} if {@link #hasTarget()} and {@code |bearing| <= toleranceRad}
     * @throws IllegalArgumentException if {@code toleranceRad} is negative
     */
    public boolean isBearingWithin(double toleranceRad) {
        if (toleranceRad < 0.0) {
            throw new IllegalArgumentException("toleranceRad must be non-negative");
        }
        if (!lastObs.hasTarget) {
            return false;
        }
        return Math.abs(lastObs.cameraBearingRad()) <= toleranceRad;
    }

    /**
     * Check whether the current robot-centric bearing is within a tolerance.
     *
     * @param cameraMount robot→camera extrinsics (must not be {@code null})
     * @param toleranceRad tolerance in radians (must be {@code >= 0})
     * @return {@code true} if {@link #hasTarget()} and {@code |robotBearing| <= toleranceRad}
     * @throws NullPointerException if {@code cameraMount} is {@code null}
     * @throws IllegalArgumentException if {@code toleranceRad} is negative
     */
    public boolean isRobotBearingWithin(CameraMountConfig cameraMount, double toleranceRad) {
        Objects.requireNonNull(cameraMount, "cameraMount is required");
        if (toleranceRad < 0.0) {
            throw new IllegalArgumentException("toleranceRad must be non-negative");
        }
        if (!lastObs.hasTarget) {
            return false;
        }
        return Math.abs(CameraMountLogic.robotBearingRad(lastObs, cameraMount)) <= toleranceRad;
    }

    /**
     * Compute planar (XY) range to the tag in the camera frame.
     *
     * <p>This uses the forward/left components of the camera→tag pose:
     * {@code sqrt(forward^2 + left^2)}.</p>
     *
     * @return planar range in inches (camera frame)
     */
    public double rangeInches() {
        double f = lastObs.cameraForwardInches();
        double l = lastObs.cameraLeftInches();
        return Math.sqrt(f * f + l * l);
    }

    /**
     * Return the full line-of-sight range to the tag in the camera frame.
     *
     * @return range in inches along the camera→tag ray
     */
    public double lineOfSightRangeInches() {
        return lastObs.cameraRangeInches();
    }

    /**
     * Return the maximum observation age allowed by this tracker.
     *
     * @return maximum age in seconds; older observations are treated as no-target
     */
    public double maxAgeSec() {
        return maxAgeSec;
    }

    /**
     * Return the set of tag IDs this tracker considers valid targets.
     *
     * @return immutable/owned set of tag IDs of interest
     */
    public Set<Integer> idsOfInterest() {
        return idsOfInterest;
    }

    /**
     * Convert the current observation into a robot-relative planar observation.
     *
     * <p>This is the preferred representation for drive-assist and “driver guidance” code.
     * It accounts for camera offset using {@link CameraMountLogic}.</p>
     *
     * @param cameraMount robot→camera extrinsics (must not be {@code null})
     * @return a robot-relative observation; {@link TargetObservation2d#hasTarget} will be false if
     *         {@link #hasTarget()} is false
     */
    public TargetObservation2d toRobotObservation2d(CameraMountConfig cameraMount) {
        Objects.requireNonNull(cameraMount, "cameraMount is required");
        return CameraMountLogic.robotObservation2d(lastObs, cameraMount);
    }

    /**
     * Dump a snapshot of the current tracker state to a {@link DebugSink}.
     *
     * @param dbg debug sink; if {@code null}, this method does nothing
     * @param prefix key prefix to use for debug fields; if null/empty, {@code "tagTarget"} is used
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "tagTarget" : prefix;

        dbg.addLine(p + ": TagTarget");

        dbg.addData(p + ".ids", idsOfInterest.toString());
        dbg.addData(p + ".maxAgeSec", maxAgeSec);
        dbg.addData(p + ".sensor.class", sensor.getClass().getSimpleName());
        dbg.addData(p + ".lastUpdatedCycle", lastUpdatedCycle);

        AprilTagObservation o = lastObs;
        dbg.addData(p + ".obs.hasTarget", o.hasTarget);
        dbg.addData(p + ".obs.id", o.id);
        dbg.addData(p + ".obs.ageSec", o.ageSec);

        dbg.addData(p + ".obs.cameraForwardInches", o.cameraForwardInches());
        dbg.addData(p + ".obs.cameraLeftInches", o.cameraLeftInches());
        dbg.addData(p + ".obs.cameraUpInches", o.cameraUpInches());

        dbg.addData(p + ".obs.cameraBearingRad", o.cameraBearingRad());
        dbg.addData(p + ".obs.cameraRangeInches", o.cameraRangeInches());
        dbg.addData(p + ".obs.cameraPlanarRangeInches", rangeInches());
    }

    /**
     * Dump tracker state to a {@link DebugSink}, including robot-centric bearing if available.
     *
     * @param dbg debug sink; if {@code null}, this method does nothing
     * @param prefix key prefix to use for debug fields; if null/empty, {@code "tagTarget"} is used
     * @param cameraMount robot→camera extrinsics used to compute robot-centric bearing (must not be {@code null})
     * @throws NullPointerException if {@code cameraMount} is {@code null}
     */
    public void debugDump(DebugSink dbg, String prefix, CameraMountConfig cameraMount) {
        if (dbg == null) {
            return;
        }
        Objects.requireNonNull(cameraMount, "cameraMount is required");

        debugDump(dbg, prefix);

        String p = (prefix == null || prefix.isEmpty()) ? "tagTarget" : prefix;
        if (lastObs.hasTarget) {
            dbg.addData(p + ".obs.robotBearingRad", CameraMountLogic.robotBearingRad(lastObs, cameraMount));
        } else {
            dbg.addData(p + ".obs.robotBearingRad", 0.0);
        }
    }
}
