package edu.ftcsushi.fw.spatial;

import java.util.Objects;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.sensing.observation.TargetObservation2d;

/**
 * Immutable desired robot-center field pose supported by one target observation.
 *
 * <p>An approach is intent, not an observed object orientation, a route, arrival evidence, or a
 * clearance certificate. An observed result expires with its sighting. An explicit bounded
 * commitment retains a destination for a resting target without pretending the target was seen
 * again. Loop-clock resets invalidate both forms. Consumers still gate current localization and
 * must discard retained goals when their owner rebases the field coordinate system.</p>
 */
public final class ApproachResult2d {
    private final Pose2d fieldToRobotGoal;
    private final TargetObservation2d observation;
    private final double maxObservationAgeSec;
    private final LoopTimestamp committedAt;
    private final double commitmentDurationSec;
    private final String reason;

    private ApproachResult2d(Pose2d goal, TargetObservation2d observation, double maxAgeSec,
                             LoopTimestamp committedAt, double durationSec, String reason) {
        this.fieldToRobotGoal = goal;
        this.observation = observation;
        this.maxObservationAgeSec = maxAgeSec;
        this.committedAt = committedAt;
        this.commitmentDurationSec = durationSec;
        this.reason = reason;
    }

    /** Returns unavailable geometry; the reason describes the robot-owned rejection. */
    public static ApproachResult2d unavailable(String reason) {
        String message = Objects.requireNonNull(reason, "reason").trim();
        if (message.isEmpty()) throw new IllegalArgumentException("Approach reason must not be blank");
        return new ApproachResult2d(null, TargetObservation2d.none(), 0.0,
                LoopTimestamp.unavailable(), 0.0, message);
    }

    /**
     * Creates a computed robot-center goal while preserving its supporting observation.
     * This is the seam for robot-authored tag/object approach geometry, not a safety approval.
     */
    public static ApproachResult2d observedFieldPose(Pose2d fieldToRobotGoal,
                                                    TargetObservation2d observation,
                                                    double maxObservationAgeSec) {
        SpatialValidation.requireFinitePose2d("fieldToRobotGoal", fieldToRobotGoal);
        requireDuration("maxObservationAgeSec", maxObservationAgeSec, false);
        TargetObservation2d evidence = Objects.requireNonNull(observation, "observation");
        if (!evidence.hasFieldPosition()) return unavailable("Target has no capture-time field position");
        return new ApproachResult2d(fieldToRobotGoal, evidence, maxObservationAgeSec,
                LoopTimestamp.unavailable(), 0.0, "observed target approach");
    }

    /**
     * Positions a rigid tool at a stand-off from a located target, at the chosen robot heading.
     * Distances are inches; heading is field-relative, CCW-positive radians. The tool's local +X
     * points toward the target. Apply no additional tool offset to downstream robot-center guidance.
     */
    public static ApproachResult2d forTarget(TargetObservation2d observation,
                                             Pose2d robotToToolFrame,
                                             double standOffInches,
                                             double desiredRobotFieldHeadingRad,
                                             double maxObservationAgeSec) {
        Objects.requireNonNull(observation, "observation");
        SpatialValidation.requireFinitePose2d("robotToToolFrame", robotToToolFrame);
        requireDuration("standOffInches", standOffInches, false);
        SpatialValidation.requireFinite("desiredRobotFieldHeadingRad", desiredRobotFieldHeadingRad);
        requireDuration("maxObservationAgeSec", maxObservationAgeSec, false);
        if (!observation.hasFieldPosition()) return unavailable("Target has no capture-time field position");
        Pose2d bodyTarget = robotToToolFrame.then(new Pose2d(standOffInches, 0.0, 0.0));
        double heading = Pose2d.wrapToPi(desiredRobotFieldHeadingRad);
        double c = Math.cos(heading);
        double s = Math.sin(heading);
        Pose2d goal = new Pose2d(
                observation.fieldXInches - (c * bodyTarget.xInches - s * bodyTarget.yInches),
                observation.fieldYInches - (s * bodyTarget.xInches + c * bodyTarget.yInches),
                heading);
        if (!SpatialValidation.isFinite(goal)) return unavailable("Approach geometry overflowed");
        return observedFieldPose(goal, observation, maxObservationAgeSec);
    }

    /**
     * Freezes this fresh observed destination once for a bounded resting-target attempt.
     * The original sighting remains unchanged. Recommitting an already committed result is rejected
     * so repeated calls cannot extend a frozen destination's lifetime without a new observation.
     */
    public ApproachResult2d committedFor(LoopClock clock, double durationSec) {
        Objects.requireNonNull(clock, "clock");
        requireDuration("commitmentDurationSec", durationSec, true);
        if (isCommitted()) throw new IllegalStateException("Approach is already committed; reobserve before another attempt");
        if (!isUsable(clock)) return unavailable("Approach observation is unavailable or stale at commitment");
        return new ApproachResult2d(fieldToRobotGoal, observation, maxObservationAgeSec,
                clock.nowTimestamp(), durationSec, "bounded resting-target destination");
    }

    /** Whether this value contains geometry, independently of its current age. */
    public boolean hasApproach() { return fieldToRobotGoal != null; }

    /** Whether an explicit bounded commitment, rather than live observation freshness, owns expiry. */
    public boolean isCommitted() { return commitmentDurationSec > 0.0; }

    /** Checks observation/commitment age in the supplied clock and reset identity. */
    public boolean isUsable(LoopClock clock) {
        Objects.requireNonNull(clock, "clock");
        if (!hasApproach()) return false;
        double observationAge = observation.timestamp.ageSec(clock);
        if (!Double.isFinite(observationAge) || observationAge < -1.0e-6) return false;
        return isCommitted() ? committedAt.isFresh(clock, commitmentDurationSec)
                : observation.timestamp.isFresh(clock, maxObservationAgeSec);
    }

    /** Desired robot-center field pose; unavailable results reject this accessor. */
    public Pose2d fieldToRobotGoalPose() {
        if (!hasApproach()) throw new IllegalStateException(reason);
        return fieldToRobotGoal;
    }

    /** Original capture/projection evidence; never a refreshed observation. */
    public TargetObservation2d observation() { return observation; }

    /** Time of the explicit commitment, or unavailable for an observed result. */
    public LoopTimestamp committedAt() { return committedAt; }

    /** Human-readable provenance or unavailable reason. */
    public String reason() { return reason; }

    private static void requireDuration(String name, double value, boolean positive) {
        if (!Double.isFinite(value) || (positive ? value <= 0.0 : value < 0.0)) {
            throw new IllegalArgumentException(name + " must be finite and " + (positive ? "> 0" : ">= 0"));
        }
    }
}
