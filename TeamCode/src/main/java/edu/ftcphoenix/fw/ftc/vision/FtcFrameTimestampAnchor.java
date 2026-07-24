package edu.ftcphoenix.fw.ftc.vision;

import java.util.Objects;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;

/**
 * Anchors one stable FTC/vendor frame identity into the Phoenix loop-clock time domain.
 *
 * <p>Age-native FTC boundaries call {@link #anchor(LoopClock, Object, double)} whenever they
 * publish a result. Repeated reads of the same vendor identity receive the exact retained
 * timestamp rather than translating a changing relative age again. A clock reset invalidates and
 * blocks the retained identity. Vendor adapters also keep their own monotonic identity gate so an
 * older frame cannot return after newer post-reset data.</p>
 *
 * <p>This bridge deliberately owns no vendor identity policy. Webcam and Limelight adapters create
 * their own immutable identity values and reject vendor-specific regressions before calling it.</p>
 */
final class FtcFrameTimestampAnchor {

    private LoopClock ownerClock;
    private Object retainedIdentity;
    private Object resetBlockedIdentity;
    private LoopTimestamp retainedTimestamp = LoopTimestamp.unavailable();

    /**
     * Returns one retained timestamp for {@code stableIdentity}, anchoring {@code ageSec} only on
     * that identity's first valid publication.
     *
     * <p>A null identity or invalid age fails closed with an unavailable timestamp. Once a retained
     * timestamp becomes invalid in its owning clock (for example after {@link LoopClock#reset}),
     * the most recently reset-invalidated identity remains blocked even if one newer identity is
     * observed; changing only the reported age cannot make it current again. A later reset advances
     * that bounded barrier to the then-retained identity, while each vendor adapter's monotonic gate
     * rejects older replay. The same stable {@link LoopClock} instance must be used for this
     * anchor's lifetime.</p>
     *
     * @param clock shared OpMode loop clock
     * @param stableIdentity immutable vendor-frame/result identity, or {@code null} if unavailable
     * @param ageSec current finite, non-negative vendor-reported age of that identity
     * @return retained current-epoch frame timestamp, or an unavailable timestamp when timing
     *         cannot be trusted
     */
    LoopTimestamp anchor(LoopClock clock, Object stableIdentity, double ageSec) {
        LoopClock checkedClock = Objects.requireNonNull(clock, "clock");
        requireStableClock(checkedClock);
        blockRetainedIdentityIfTimestampInvalid(checkedClock);

        if (stableIdentity == null || !Double.isFinite(ageSec) || ageSec < 0.0) {
            return LoopTimestamp.unavailable();
        }

        if (stableIdentity.equals(retainedIdentity)) {
            return retainedTimestamp;
        }

        if (stableIdentity.equals(resetBlockedIdentity)) {
            return LoopTimestamp.unavailable();
        }

        LoopTimestamp anchored = checkedClock.timestampSecondsAgo(ageSec);
        retainedIdentity = stableIdentity;
        retainedTimestamp = anchored;
        return anchored;
    }

    private void blockRetainedIdentityIfTimestampInvalid(LoopClock clock) {
        if (retainedIdentity == null
                || Double.isFinite(retainedTimestamp.ageSec(clock))) {
            return;
        }

        // Detect the reset/time discontinuity before considering the incoming identity. Otherwise
        // A -> reset -> B would overwrite A, allowing a later repeat of A to be re-anchored.
        // One slot is sufficient here: each camera owner rejects same-generation identity
        // regressions before calling this helper, while this slot prevents the most recently
        // reset-invalidated frame from being re-anchored even after one newer frame arrives.
        resetBlockedIdentity = retainedIdentity;
        retainedIdentity = null;
        retainedTimestamp = LoopTimestamp.unavailable();
    }

    private void requireStableClock(LoopClock clock) {
        if (ownerClock == null) {
            ownerClock = clock;
            return;
        }
        if (ownerClock != clock) {
            throw new IllegalArgumentException(
                    "FtcFrameTimestampAnchor requires one stable LoopClock instance; "
                            + "pass the same OpMode clock on every read");
        }
    }
}
