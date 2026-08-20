package edu.ftcphoenix.fw.localization;

import java.util.Objects;

import edu.ftcphoenix.fw.core.time.LoopTimestamp;

/** Immutable evidence-bearing estimate of robot heading in a fixed field frame. */
public final class HeadingEstimate {

    /** Robot heading in radians, counter-clockwise positive from field +X. */
    public final double fieldHeadingRad;

    /** Whether {@link #fieldHeadingRad} is currently usable. */
    public final boolean hasHeading;

    /** Source-defined confidence in [0, 1]. */
    public final double quality;

    /** Epoch-safe capture time, or {@link LoopTimestamp#unavailable()}. */
    public final LoopTimestamp timestamp;

    /** Construct one immutable heading snapshot. */
    public HeadingEstimate(double fieldHeadingRad,
                           boolean hasHeading,
                           double quality,
                           LoopTimestamp timestamp) {
        this.fieldHeadingRad = fieldHeadingRad;
        this.hasHeading = hasHeading;
        this.quality = quality;
        this.timestamp = Objects.requireNonNull(timestamp, "timestamp is required");
    }

    /** Return an unavailable heading produced at the supplied time. */
    public static HeadingEstimate noHeading(LoopTimestamp timestamp) {
        return new HeadingEstimate(0.0, false, 0.0, timestamp);
    }

    @Override
    public String toString() {
        return hasHeading
                ? "HeadingEstimate{fieldHeadingRad=" + fieldHeadingRad
                + ", quality=" + quality + ", timestamp=" + timestamp + "}"
                : "HeadingEstimate{no heading, timestamp=" + timestamp + "}";
    }
}
