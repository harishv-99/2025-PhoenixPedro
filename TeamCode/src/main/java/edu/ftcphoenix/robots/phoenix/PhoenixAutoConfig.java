package edu.ftcphoenix.robots.phoenix;

/**
 * Phoenix-owned autonomous timing, tolerance, and integration-route policy.
 *
 * <p>This mutable object is data only. {@link #defaults()} returns a complete software-valid
 * starting point, not evidence that route timing, clearance, aiming, or stopping behavior is safe
 * on the physical robot. Active consumers capture and validate only the fields they use; the
 * public custom-routine context carries a raw defensive snapshot of the cohesive policy.</p>
 */
public final class PhoenixAutoConfig {

    /**
     * Elapsed Auto time at which Phoenix abandons unfinished pre-park work and starts its
     * live-pose return/park policy, in seconds.
     */
    public double parkTakeoverElapsedSec = 25.0;

    /** Maximum time allowed for one route Task before Task-owned timeout, in seconds. */
    public double routeTimeoutSec = 4.0;

    /** Heading tolerance used by autonomous aim Tasks, in degrees. */
    public double aimHeadingToleranceDeg = 2.0;

    /** Overall timeout for an autonomous aim Task, in seconds. */
    public double aimTimeoutSec = 1.75;

    /** Maximum time an aim Task may run without usable guidance, in seconds. */
    public double aimMaxNoGuidanceSec = 0.75;

    /** Maximum wait for a selected scoring target before the attempt times out, in seconds. */
    public double waitForTargetSec = 0.75;

    /** Maximum wait for a requested shot to drain from the scoring queue, in seconds. */
    public double waitForShotCompleteSec = 2.5;

    /** Distance used by the checked-in Pedro integration placeholder path, in inches. */
    public double pedroIntegrationTestDistanceIn = 12.0;

    private PhoenixAutoConfig() {
        // Defaults set by field initializers.
    }

    /**
     * Returns a fresh complete software-valid Auto policy draft.
     *
     * <p>These defaults do not prove physical route safety, aiming performance, or clearance.</p>
     *
     * @return new mutable Auto configuration
     */
    public static PhoenixAutoConfig defaults() {
        return new PhoenixAutoConfig();
    }

    /**
     * Returns a raw field-for-field copy without validation.
     *
     * <p>Invalid or non-finite authored values remain intact so the active owner can report them
     * with its own field path and validation precedence.</p>
     *
     * @return independent mutable copy of this Auto configuration
     */
    public PhoenixAutoConfig copy() {
        PhoenixAutoConfig copy = new PhoenixAutoConfig();
        copy.parkTakeoverElapsedSec = parkTakeoverElapsedSec;
        copy.routeTimeoutSec = routeTimeoutSec;
        copy.aimHeadingToleranceDeg = aimHeadingToleranceDeg;
        copy.aimTimeoutSec = aimTimeoutSec;
        copy.aimMaxNoGuidanceSec = aimMaxNoGuidanceSec;
        copy.waitForTargetSec = waitForTargetSec;
        copy.waitForShotCompleteSec = waitForShotCompleteSec;
        copy.pedroIntegrationTestDistanceIn = pedroIntegrationTestDistanceIn;
        return copy;
    }
}
