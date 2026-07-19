package edu.ftcphoenix.fw.tools.tester;

import java.util.Objects;

/**
 * Package-private lifecycle owner for a replaceable child {@link TeleOpTester}.
 *
 * <p>The session deliberately does not own factories, menus, telemetry, or FTC lifecycle state.
 * It only makes child retention and cleanup atomic for the two menu-style tester owners in this
 * package.</p>
 */
final class TesterChildSession {

    private enum State {
        EMPTY,
        ACTIVE,
        CLEANING,
        CLEANUP_BLOCKED,
        TERMINAL
    }

    static final class BackResult {
        private final boolean handled;
        private final RuntimeException failure;

        private BackResult(boolean handled, RuntimeException failure) {
            this.handled = handled;
            this.failure = failure;
        }

        boolean handled() {
            return handled;
        }

        RuntimeException failure() {
            return failure;
        }
    }

    private State state = State.EMPTY;
    private TeleOpTester active;
    private boolean terminalRequested;
    private RuntimeException lastFailure;

    boolean canActivate() {
        return state == State.EMPTY;
    }

    boolean hasActive() {
        return state == State.ACTIVE;
    }

    boolean cleanupBlocked() {
        return state == State.CLEANUP_BLOCKED;
    }

    boolean mustConsumeBackNavigation() {
        return state == State.CLEANING
                || state == State.CLEANUP_BLOCKED
                || state == State.TERMINAL;
    }

    RuntimeException lastFailure() {
        return lastFailure;
    }

    /**
     * Retain a new inactive child before its first lifecycle callback.
     */
    void retain(TeleOpTester child) {
        Objects.requireNonNull(child, "child");
        if (state != State.EMPTY) {
            throw new IllegalStateException(
                    "Cannot activate a tester child while its session is " + state + ".");
        }

        active = child;
        state = State.ACTIVE;
        lastFailure = null;
    }

    RuntimeException init(TesterContext ctx) {
        return invoke(child -> child.init(ctx));
    }

    RuntimeException initLoop(double dtSec) {
        return invoke(child -> child.initLoop(dtSec));
    }

    RuntimeException start() {
        return invoke(TeleOpTester::start);
    }

    RuntimeException loop(double dtSec) {
        return invoke(child -> child.loop(dtSec));
    }

    BackResult backPressed() {
        if (state != State.ACTIVE) {
            return new BackResult(false, null);
        }

        TeleOpTester child = active;
        try {
            return new BackResult(child.onBackPressed(), null);
        } catch (RuntimeException failure) {
            return new BackResult(false, failStop(failure));
        }
    }

    /**
     * Stop the active child so its owner may return to selection or launch a replacement.
     *
     * @return a cleanup failure, or {@code null} when cleanup completed or there was no child
     */
    RuntimeException stopForReplacement() {
        if (state != State.ACTIVE) {
            return null;
        }
        return detachAndStop(null, false);
    }

    /**
     * Permanently terminalize this session and stop its active child at most once.
     *
     * <p>Terminal state is published before calling the child, so a reentrant stop is a no-op.</p>
     *
     * @return a cleanup failure, or {@code null} when cleanup completed or was already attempted
     */
    RuntimeException stopTerminal() {
        if (state == State.TERMINAL) {
            return null;
        }
        if (state == State.CLEANING) {
            terminalRequested = true;
            return null;
        }
        if (state == State.CLEANUP_BLOCKED) {
            state = State.TERMINAL;
            return null;
        }
        if (state == State.EMPTY) {
            state = State.TERMINAL;
            return null;
        }

        terminalRequested = true;
        return detachAndStop(null, true);
    }

    private RuntimeException invoke(ChildCallback callback) {
        if (state != State.ACTIVE) {
            return null;
        }

        TeleOpTester child = active;
        try {
            callback.run(child);
            return null;
        } catch (RuntimeException failure) {
            return failStop(failure);
        }
    }

    private RuntimeException failStop(RuntimeException primaryFailure) {
        return detachAndStop(primaryFailure, false);
    }

    /**
     * Detach before invoking user cleanup. A cleanup callback can therefore reenter its owner
     * without observing or stopping the same child twice.
     */
    private RuntimeException detachAndStop(RuntimeException primaryFailure, boolean terminal) {
        TeleOpTester stopping = active;
        active = null;
        state = State.CLEANING;
        if (terminal) {
            terminalRequested = true;
        }

        RuntimeException result = primaryFailure;
        boolean cleanupReturned = false;
        try {
            stopping.stop();
            cleanupReturned = true;
        } catch (RuntimeException cleanupFailure) {
            if (result == null) {
                result = cleanupFailure;
            } else if (cleanupFailure != result) {
                result.addSuppressed(cleanupFailure);
            }
        } finally {
            if (terminalRequested) {
                state = State.TERMINAL;
            } else if (cleanupReturned) {
                state = State.EMPTY;
            } else {
                state = State.CLEANUP_BLOCKED;
            }
        }
        if (result != null) {
            lastFailure = result;
        }
        return result;
    }

    private interface ChildCallback {
        void run(TeleOpTester child);
    }
}
