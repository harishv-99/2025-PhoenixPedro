package edu.ftcphoenix.fw.drive.route;

/**
 * Status and cancellation handle for exactly one route-following attempt.
 *
 * <p>An execution remains bound to the route start that created it even if the follower later
 * starts another route. Implementations retain a terminal {@link #status()} and ensure cancelling
 * an old execution cannot stop a replacement.</p>
 */
public abstract class RouteExecution {

    private RouteStatus taskTerminalStatus;

    /** Create a per-route execution from a route-integration implementation. */
    protected RouteExecution() {
    }

    /**
     * Returns this execution's current or retained terminal status.
     *
     * <p>The result must never be {@code null}. Once terminal, it must remain terminal and must not
     * reflect a later route started by the same follower. Reads are cheap and side-effect-free;
     * integrations classify transitions in their owned heartbeat rather than in this getter.</p>
     *
     * @return backend-neutral status for this exact route execution
     * @throws IllegalStateException if the integration hook returns {@code null} or a Task-owned
     * status
     */
    public final RouteStatus status() {
        if (taskTerminalStatus != null) {
            return taskTerminalStatus;
        }
        RouteStatus integrationStatus = integrationStatus();
        if (integrationStatus == null) {
            throw new IllegalStateException("RouteExecution.integrationStatus() returned null. "
                    + "Return a backend-neutral RouteStatus.");
        }
        if (integrationStatus == RouteStatus.NOT_STARTED) {
            throw new IllegalStateException("RouteExecution.integrationStatus() returned "
                    + "NOT_STARTED. RouteFollower.follow(...) must begin synchronously and return "
                    + "an ACTIVE execution or a retained integration terminal status.");
        }
        if (integrationStatus == RouteStatus.TASK_TIMEOUT) {
            throw new IllegalStateException("RouteExecution.integrationStatus() returned "
                    + "TASK_TIMEOUT. That status is owned by RouteTask; return the integration's "
                    + "own terminal reason instead.");
        }
        return integrationStatus;
    }

    /**
     * Cancel this execution if and only if it is active.
     *
     * <p>Terminal and repeated cancellation are side-effect-free. Active cancellation must make
     * this execution terminal as {@link RouteStatus#CANCELLED} before cleanup that may throw, and
     * it must never stop a newer replacement execution.</p>
     */
    public final void cancel() {
        RouteStatus currentStatus;
        try {
            currentStatus = status();
        } catch (RuntimeException statusFailure) {
            failClosed(statusFailure);
            throw statusFailure;
        }
        if (currentStatus != RouteStatus.ACTIVE) {
            return;
        }
        taskTerminalStatus = RouteStatus.CANCELLED;
        cancelActive();
    }

    /**
     * Return the integration-owned status for this exact route attempt.
     *
     * <p>Implementations must retain terminal states and must not return the state of a replacement
     * execution. This hook is a cheap, side-effect-free snapshot and must be safe to read more than
     * once in one loop cycle.</p>
     *
     * @return non-null integration-owned route status; never {@link RouteStatus#NOT_STARTED} or
     * {@link RouteStatus#TASK_TIMEOUT}
     */
    protected abstract RouteStatus integrationStatus();

    /**
     * Stop this exact execution after the base class has retained its Task-owned terminal status.
     *
     * <p>This hook is normally called for an active execution. It may also be called when the
     * integration's status hook itself failed, so implementations must recheck their exact run
     * identity and make terminal/repeated cleanup harmless. It must never stop a newer replacement
     * execution. Calling {@link #status()} inside this hook returns the retained
     * {@link RouteStatus#CANCELLED}, {@link RouteStatus#TASK_TIMEOUT}, or
     * {@link RouteStatus#FAILED} reason.</p>
     */
    protected abstract void cancelActive();

    /** Cancel after the owning RouteTask has just observed this execution as active. */
    final void cancelAfterActiveObservation() {
        if (taskTerminalStatus != null) {
            return;
        }
        taskTerminalStatus = RouteStatus.CANCELLED;
        cancelActive();
    }

    /** Mark a Task timeout before applying active execution cleanup. */
    final void cancelForTimeout() {
        if (taskTerminalStatus != null) {
            return;
        }
        taskTerminalStatus = RouteStatus.TASK_TIMEOUT;
        cancelActive();
    }

    /** Retain failed status and best-effort cleanup without trusting a broken status hook. */
    final void failClosed(RuntimeException primaryFailure) {
        if (taskTerminalStatus != null) {
            return;
        }
        taskTerminalStatus = RouteStatus.FAILED;
        try {
            cancelActive();
        } catch (RuntimeException cleanupFailure) {
            if (cleanupFailure != primaryFailure) {
                primaryFailure.addSuppressed(cleanupFailure);
            }
        }
    }
}
