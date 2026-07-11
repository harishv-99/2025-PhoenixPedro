package edu.ftcphoenix.robots.phoenix;

/**
 * Executes Phoenix's internal shutdown actions once while preserving best-effort cleanup.
 *
 * <p>This helper is deliberately package-private. {@link PhoenixRobot#stop()} remains the one
 * student-facing shutdown operation; this class only keeps its exact-once and error-aggregation
 * rules independently testable without constructing FTC hardware.</p>
 */
final class PhoenixShutdown {

    private boolean stopped;

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
