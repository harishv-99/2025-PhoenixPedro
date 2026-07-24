package edu.ftcphoenix.fw.core.time;

import java.util.Objects;

/**
 * Immutable capture time owned by one {@link LoopClock} reset epoch.
 *
 * <p>A timestamp deliberately keeps its clock identity, reset epoch, and seconds coordinate
 * together. Consumers therefore pass one value and use {@link #ageSec(LoopClock)},
 * {@link #isFresh(LoopClock, double)}, or {@link #secondsSince(LoopTimestamp)} instead of carrying
 * and comparing raw timestamp/epoch pairs.</p>
 *
 * <p>Valid timestamps are created only by {@link LoopClock#nowTimestamp()} and
 * {@link LoopClock#timestampSecondsAgo(double)}. Use {@link #unavailable()} when a containing value
 * has no truthful measurement time. There is intentionally no raw seconds or epoch accessor.</p>
 */
public final class LoopTimestamp {

    /** Numeric tolerance used only when comparing a capture with its clock's current time. */
    private static final double FUTURE_TOLERANCE_SEC = 1.0e-6;

    private static final LoopTimestamp UNAVAILABLE = new LoopTimestamp(null, 0L, Double.NaN);

    private final LoopClock owner;
    private final long epoch;
    private final double timeSec;

    private LoopTimestamp(LoopClock owner, long epoch, double timeSec) {
        this.owner = owner;
        this.epoch = epoch;
        this.timeSec = timeSec;
    }

    static LoopTimestamp captured(LoopClock owner, long epoch, double timeSec) {
        Objects.requireNonNull(owner, "owner");
        if (!Double.isFinite(timeSec)) {
            throw new IllegalArgumentException(
                    "LoopTimestamp capture time must be finite, got " + timeSec);
        }
        return new LoopTimestamp(owner, epoch, timeSec);
    }

    /**
     * Returns the shared sentinel for a value that has no truthful measurement time.
     */
    public static LoopTimestamp unavailable() {
        return UNAVAILABLE;
    }

    /**
     * Returns whether this value structurally contains a captured time.
     *
     * <p>This distinguishes {@link #unavailable()} only. A timestamp can still belong to a prior
     * reset epoch; use {@link #ageSec(LoopClock)} or {@link #isFresh(LoopClock, double)} when current
     * validity matters.</p>
     */
    public boolean isAvailable() {
        return owner != null;
    }

    /**
     * Returns this capture's age in the supplied clock's current reset epoch.
     *
     * <p>A capture up to one microsecond in the future is treated as current to absorb floating
     * point boundary noise. Unavailable, prior-epoch, or materially future timestamps return
     * {@link Double#NaN}.</p>
     *
     * @param clock the same stable clock that created this timestamp
     * @return non-negative age in seconds, or {@code NaN} when it is not valid now
     * @throws IllegalArgumentException if this timestamp belongs to a different clock
     */
    public double ageSec(LoopClock clock) {
        Objects.requireNonNull(clock, "clock");
        if (!isAvailable()) {
            return Double.NaN;
        }
        requireSameClock(clock);
        if (epoch != owner.timestampEpoch()) {
            return Double.NaN;
        }

        double nowSec = clock.nowSec();
        if (!Double.isFinite(nowSec)) {
            return Double.NaN;
        }
        double ageSec = nowSec - timeSec;
        if (!Double.isFinite(ageSec) || ageSec < -FUTURE_TOLERANCE_SEC) {
            return Double.NaN;
        }
        return Math.max(0.0, ageSec);
    }

    /**
     * Returns whether this capture is valid in the supplied clock's current epoch and no older
     * than the inclusive maximum age.
     *
     * @param clock     the same stable clock that created this timestamp
     * @param maxAgeSec inclusive finite, non-negative maximum age
     * @throws IllegalArgumentException if {@code maxAgeSec} is invalid or this timestamp belongs
     *                                  to a different clock
     */
    public boolean isFresh(LoopClock clock, double maxAgeSec) {
        if (!Double.isFinite(maxAgeSec) || maxAgeSec < 0.0) {
            throw new IllegalArgumentException(
                    "maxAgeSec must be finite and >= 0, got " + maxAgeSec);
        }
        double ageSec = ageSec(clock);
        return Double.isFinite(ageSec) && ageSec <= maxAgeSec;
    }

    /**
     * Returns the signed duration from {@code earlier} to this timestamp.
     *
     * <p>Positive means this timestamp is later. Unavailable timestamps, timestamps from different
     * reset epochs, or timestamps whose epoch is no longer current return {@link Double#NaN}.
     * There is no future-time tolerance here because this operation compares two captured values,
     * not a capture with the current clock.</p>
     *
     * @throws IllegalArgumentException if both timestamps are available but belong to different
     *                                  clocks
     */
    public double secondsSince(LoopTimestamp earlier) {
        Objects.requireNonNull(earlier, "earlier");
        if (!isAvailable() || !earlier.isAvailable()) {
            return Double.NaN;
        }
        if (owner != earlier.owner) {
            throw new IllegalArgumentException(
                    "Cannot compare LoopTimestamps from different LoopClock instances; "
                            + "use one stable LoopClock for the OpMode");
        }
        if (epoch != earlier.epoch || epoch != owner.timestampEpoch()) {
            return Double.NaN;
        }
        double elapsedSec = timeSec - earlier.timeSec;
        return Double.isFinite(elapsedSec) ? elapsedSec : Double.NaN;
    }

    private void requireSameClock(LoopClock clock) {
        if (owner != clock) {
            throw new IllegalArgumentException(
                    "LoopTimestamp belongs to a different LoopClock; "
                            + "pass the same stable LoopClock that created it");
        }
    }

    @Override
    public String toString() {
        if (!isAvailable()) {
            return "LoopTimestamp{unavailable}";
        }
        return "LoopTimestamp{captured}";
    }
}
