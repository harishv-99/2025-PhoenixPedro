package edu.ftcphoenix.fw.ftc.localization;

import java.util.Objects;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;

/**
 * Immutable same-cycle planar pose and velocity sample from a Pinpoint odometry owner.
 *
 * <p>The pose and translational velocity use the Phoenix field frame. Distances are inches,
 * velocities are inches per second, and angular values are radians or radians per second. The
 * unwrapped {@link #totalHeadingRad} counts only measured physical turns; deliberate pose rebases
 * must not change it.</p>
 *
 * <p>{@link #hasPose} and {@link #hasVelocity} are separate because a deliberate hardware reset,
 * recalibration, or invalid sensor response can make motion unavailable without inventing a zero
 * physical sample.</p>
 */
public final class PinpointKinematicSnapshot {

    /** Sentinel used when a snapshot was produced outside a {@link LoopClock} cycle. */
    public static final long NO_CYCLE = -1L;

    /** Latest robot pose in the Phoenix field frame; inspect {@link #hasPose} before using it. */
    public final Pose2d fieldToRobotPose;

    /** Whether {@link #fieldToRobotPose} is valid. */
    public final boolean hasPose;

    /** Whether the three velocity fields contain a measured physical velocity. */
    public final boolean hasVelocity;

    /** {@link LoopClock#cycle()} associated with this sample, or {@link #NO_CYCLE}. */
    public final long cycle;

    /** Epoch-safe sample time from the shared {@link LoopClock}. */
    public final LoopTimestamp timestamp;

    /** Field-frame +X velocity in inches per second. */
    public final double fieldVelocityXInchesPerSec;

    /** Field-frame +Y velocity in inches per second. */
    public final double fieldVelocityYInchesPerSec;

    /** CCW-positive yaw velocity in radians per second. */
    public final double angularVelocityRadPerSec;

    /** Unwrapped CCW-positive physical heading accumulated from Pinpoint samples, in radians. */
    public final double totalHeadingRad;

    private PinpointKinematicSnapshot(Pose2d fieldToRobotPose,
                                      boolean hasPose,
                                      boolean hasVelocity,
                                      long cycle,
                                      LoopTimestamp timestamp,
                                      double fieldVelocityXInchesPerSec,
                                      double fieldVelocityYInchesPerSec,
                                      double angularVelocityRadPerSec,
                                      double totalHeadingRad) {
        this.fieldToRobotPose = Objects.requireNonNull(fieldToRobotPose, "fieldToRobotPose");
        requireCycle(cycle);
        this.timestamp = Objects.requireNonNull(timestamp, "timestamp");
        requireFinite(totalHeadingRad, "totalHeadingRad");

        if (hasPose) {
            requireFinite(fieldToRobotPose.xInches, "fieldToRobotPose.xInches");
            requireFinite(fieldToRobotPose.yInches, "fieldToRobotPose.yInches");
            requireFinite(fieldToRobotPose.headingRad, "fieldToRobotPose.headingRad");
        }
        if (hasVelocity) {
            requireFinite(fieldVelocityXInchesPerSec, "fieldVelocityXInchesPerSec");
            requireFinite(fieldVelocityYInchesPerSec, "fieldVelocityYInchesPerSec");
            requireFinite(angularVelocityRadPerSec, "angularVelocityRadPerSec");
        }

        this.hasPose = hasPose;
        this.hasVelocity = hasVelocity;
        this.cycle = cycle;
        this.fieldVelocityXInchesPerSec = hasVelocity ? fieldVelocityXInchesPerSec : 0.0;
        this.fieldVelocityYInchesPerSec = hasVelocity ? fieldVelocityYInchesPerSec : 0.0;
        this.angularVelocityRadPerSec = hasVelocity ? angularVelocityRadPerSec : 0.0;
        this.totalHeadingRad = totalHeadingRad;
    }

    /** Create a snapshot with no valid pose or measured velocity. */
    static PinpointKinematicSnapshot unavailable(long cycle,
                                                  LoopTimestamp timestamp,
                                                  double totalHeadingRad) {
        return new PinpointKinematicSnapshot(
                Pose2d.zero(),
                false,
                false,
                cycle,
                timestamp,
                0.0,
                0.0,
                0.0,
                totalHeadingRad
        );
    }

    /** Create a sampled pose with either valid measured velocity or an explicit velocity gap. */
    static PinpointKinematicSnapshot sampled(Pose2d fieldToRobotPose,
                                             boolean hasVelocity,
                                             long cycle,
                                             LoopTimestamp timestamp,
                                             double fieldVelocityXInchesPerSec,
                                             double fieldVelocityYInchesPerSec,
                                             double angularVelocityRadPerSec,
                                             double totalHeadingRad) {
        return new PinpointKinematicSnapshot(
                fieldToRobotPose,
                true,
                hasVelocity,
                cycle,
                timestamp,
                fieldVelocityXInchesPerSec,
                fieldVelocityYInchesPerSec,
                angularVelocityRadPerSec,
                totalHeadingRad
        );
    }

    /**
     * Rebase the cached pose while preserving measured velocity and accumulated physical heading.
     *
     * <p>This is the correction-safe operation used by the Pinpoint owner after a deliberate
     * {@code setPose}: changing the coordinate estimate is not robot motion.</p>
     */
    PinpointKinematicSnapshot withRebasedPose(Pose2d fieldToRobotPose) {
        return new PinpointKinematicSnapshot(
                fieldToRobotPose,
                true,
                hasVelocity,
                cycle,
                timestamp,
                fieldVelocityXInchesPerSec,
                fieldVelocityYInchesPerSec,
                angularVelocityRadPerSec,
                totalHeadingRad
        );
    }

    /** Preserve the last pose/heading while marking physical velocity unavailable and zeroing it. */
    PinpointKinematicSnapshot withoutVelocity() {
        return new PinpointKinematicSnapshot(
                fieldToRobotPose,
                hasPose,
                false,
                cycle,
                timestamp,
                0.0,
                0.0,
                0.0,
                totalHeadingRad
        );
    }

    /** Return whether this snapshot belongs to the supplied loop cycle. */
    public boolean isCurrentFor(LoopClock clock) {
        return clock != null
                && cycle != NO_CYCLE
                && cycle == clock.cycle()
                && Double.isFinite(timestamp.ageSec(clock));
    }

    /** Return whether both pose and measured physical velocity are available. */
    public boolean hasUsableKinematics() {
        return hasPose && hasVelocity;
    }

    private static void requireCycle(long cycle) {
        if (cycle < NO_CYCLE) {
            throw new IllegalArgumentException("cycle must be >= " + NO_CYCLE + ", got " + cycle);
        }
    }

    private static void requireFinite(double value, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite, got " + value);
        }
    }

    @Override
    public String toString() {
        return "PinpointKinematicSnapshot{" +
                "fieldToRobotPose=" + fieldToRobotPose +
                ", hasPose=" + hasPose +
                ", hasVelocity=" + hasVelocity +
                ", cycle=" + cycle +
                ", timestamp=" + timestamp +
                ", fieldVelocityXInchesPerSec=" + fieldVelocityXInchesPerSec +
                ", fieldVelocityYInchesPerSec=" + fieldVelocityYInchesPerSec +
                ", angularVelocityRadPerSec=" + angularVelocityRadPerSec +
                ", totalHeadingRad=" + totalHeadingRad +
                '}';
    }
}
