package edu.ftcsushi.fw.actuation;

import java.util.Objects;

import edu.ftcsushi.fw.core.time.LoopClock;

/**
 * Package-private transactional cycle gate for one effectful Plant heartbeat.
 *
 * <p>The owning Plant claims a cycle before it samples collaborators or performs an effect. A
 * repeated successful update is a no-op, a repeated failed update rethrows the original failure,
 * and a reentrant update fails immediately. One gate binds to one {@link LoopClock} identity for
 * its lifetime.</p>
 */
final class PlantUpdateCycle {

    private enum State {
        IDLE,
        IN_PROGRESS,
        SUCCEEDED,
        FAILED
    }

    private final String owner;
    private LoopClock boundClock;
    private long claimedCycle;
    private State state = State.IDLE;
    private RuntimeException retainedFailure;

    PlantUpdateCycle(String owner) {
        String checked = Objects.requireNonNull(owner, "owner").trim();
        if (checked.isEmpty()) throw new IllegalArgumentException("owner must not be blank");
        this.owner = checked;
    }

    /**
     * Claim the supplied cycle before any update work begins.
     *
     * @return {@code true} for the first attempt, or {@code false} after same-cycle success
     * @throws IllegalStateException for reentrancy or a different clock identity
     * @throws RuntimeException the exact retained failure after a same-cycle failed attempt
     */
    boolean begin(LoopClock clock) {
        Objects.requireNonNull(clock, "clock");
        if (state == State.IN_PROGRESS) {
            throw new IllegalStateException(owner
                    + " update is reentrant; one Plant heartbeat must finish before another begins");
        }
        if (boundClock == null) {
            boundClock = clock;
        } else if (boundClock != clock) {
            throw new IllegalStateException(owner
                    + " is bound to one LoopClock for its lifecycle; received a different clock identity");
        }

        if (state != State.IDLE && claimedCycle == clock.cycle()) {
            if (state == State.FAILED) throw retainedFailure;
            if (state == State.SUCCEEDED) return false;
        }

        claimedCycle = clock.cycle();
        retainedFailure = null;
        state = State.IN_PROGRESS;
        return true;
    }

    /** Commit the current claimed cycle as successful. */
    void succeed() {
        if (state != State.IN_PROGRESS) {
            throw new IllegalStateException(owner
                    + " cannot complete an update that is not in progress");
        }
        state = State.SUCCEEDED;
    }

    /** Retain the exact failure for the rest of the claimed cycle. */
    void fail(RuntimeException failure) {
        Objects.requireNonNull(failure, "failure");
        if (state == State.IN_PROGRESS) {
            retainedFailure = failure;
            state = State.FAILED;
        }
    }

    /**
     * Whether this Plant already began an update in the supplied cycle.
     * This observation does not claim or bind an otherwise unused gate.
     */
    boolean wasAttemptedIn(LoopClock clock) {
        Objects.requireNonNull(clock, "clock");
        if (boundClock != null && boundClock != clock) {
            throw new IllegalStateException(owner
                    + " is bound to one LoopClock for its lifecycle; received a different clock identity");
        }
        return state != State.IDLE && claimedCycle == clock.cycle();
    }
}
