package edu.ftcsushi.fw.actuation;

/** Package-private terminal lifecycle latch shared by Plant runtimes. */
final class PlantLifecycle {
    private boolean stopped;

    /** Whether an owner update may begin or continue doing work. */
    boolean isActive() {
        return !stopped;
    }

    /**
     * Claim the one terminal transition before any external cleanup callback.
     *
     * @return {@code true} only for the caller that claimed the transition
     */
    boolean claimStop() {
        if (stopped) return false;
        stopped = true;
        return true;
    }

    /** Reject a position lifecycle operation that could mutate a stopped Plant. */
    void requireActive(String operation) {
        if (stopped) {
            throw new IllegalStateException(operation
                    + " cannot be used after Plant.stop(); construct a new Plant for another lifecycle");
        }
    }
}
