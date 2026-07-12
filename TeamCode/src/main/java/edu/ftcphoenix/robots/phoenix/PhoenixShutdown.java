package edu.ftcphoenix.robots.phoenix;

/**
 * Guards Phoenix's one initialization and executes shutdown actions with best-effort cleanup.
 *
 * <p>This helper is deliberately package-private. {@link PhoenixRobot#stop()} remains the one
 * student-facing shutdown operation; this class only keeps ownership-graph replacement,
 * exact-once shutdown, and error-aggregation rules independently testable without constructing FTC
 * hardware.</p>
 */
final class PhoenixShutdown {

    private boolean initializationStarted;
    private boolean stopped;

    /**
     * Begin the one allowed mode initialization for this robot lifetime.
     *
     * <p>A repeated or cross-mode initialization is rejected before callers can overwrite the
     * active ownership graph. The existing initialized graph remains open and can still be stopped
     * normally.</p>
     *
     * @param operation initialization operation being attempted
     */
    void beginInitialization(String operation) {
        ensureOpen(operation);
        if (initializationStarted) {
            throw new IllegalStateException(
                    operation + " cannot run because this PhoenixRobot is already initialized; "
                            + "create a new PhoenixRobot for another mode or runtime"
            );
        }
        initializationStarted = true;
    }

    /**
     * Reject initialization after this robot lifetime has been stopped.
     *
     * @param operation initialization operation being attempted
     */
    void ensureOpen(String operation) {
        if (stopped) {
            throw new IllegalStateException(
                    operation + " cannot run after PhoenixRobot.stop(); create a new PhoenixRobot"
            );
        }
    }

    /**
     * Execute each non-null cleanup action in declaration order, at most once for this lifetime.
     *
     * <p>Every action is attempted even when an earlier action throws. The first runtime failure is
     * rethrown after cleanup finishes, with later distinct failures attached as suppressed
     * exceptions.</p>
     *
     * @param actions ordered cleanup actions; null entries represent owners that were not created
     */
    void run(Runnable... actions) {
        if (stopped) {
            return;
        }
        stopped = true;

        RuntimeException firstFailure = null;
        for (Runnable action : actions) {
            if (action == null) {
                continue;
            }
            try {
                action.run();
            } catch (RuntimeException failure) {
                if (firstFailure == null) {
                    firstFailure = failure;
                } else if (failure != firstFailure) {
                    firstFailure.addSuppressed(failure);
                }
            }
        }

        if (firstFailure != null) {
            throw firstFailure;
        }
    }
}
