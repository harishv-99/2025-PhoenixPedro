package edu.ftcphoenix.robots.phoenix;

/**
 * Immutable status snapshot for the scoring supervisor.
 */
public final class ScoringStatus {
    public final boolean intakeEnabled;
    public final boolean ejectRequested;
    public final boolean shootingRequested;
    public final boolean flywheelRequested;
    public final boolean shootActive;
    public final int feedBacklog;
    public final String mode;

    /**
     * Creates an immutable scoring status snapshot.
     *
     * @param intakeEnabled whether intake mode is currently latched on
     * @param ejectRequested whether eject/unjam mode is currently being requested
     * @param shootingRequested whether the caller is actively requesting shot execution
     * @param flywheelRequested whether the flywheel has been requested on by the caller
     * @param shootActive whether the scoring path is effectively in shooting mode this loop
     * @param feedBacklog requested feed-task backlog managed by the supervisor
     * @param mode human-readable feed mode label for telemetry/debug output
     */
    public ScoringStatus(boolean intakeEnabled,
                         boolean ejectRequested,
                         boolean shootingRequested,
                         boolean flywheelRequested,
                         boolean shootActive,
                         int feedBacklog,
                         String mode) {
        this.intakeEnabled = intakeEnabled;
        this.ejectRequested = ejectRequested;
        this.shootingRequested = shootingRequested;
        this.flywheelRequested = flywheelRequested;
        this.shootActive = shootActive;
        this.feedBacklog = feedBacklog;
        this.mode = mode;
    }
}
