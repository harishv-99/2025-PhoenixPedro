package edu.ftcphoenix.robots.phoenix;

/**
 * Immutable status snapshot for Phoenix's robot-specific drive-assist service.
 *
 * <p>
 * This snapshot explains how Phoenix is currently layering scoring-related drive assists on top of
 * manual driver input. It deliberately contains only derived assist state so telemetry and future
 * policy layers can reason about brace/assist behavior without inspecting internal latches or
 * overlay objects.
 * </p>
 */
public final class DriveAssistStatus {
    public final boolean autoAimRequested;
    public final boolean shootBraceEligible;
    public final boolean shootBraceEnabled;
    public final double manualTranslateMagnitude;

    /**
     * Creates an immutable drive-assist status snapshot.
     *
     * @param autoAimRequested         whether the driver/operator is currently requesting auto aim
     * @param shootBraceEligible       whether the current robot state allows the shoot-brace policy to engage
     * @param shootBraceEnabled        whether the shoot-brace pose-lock assist is currently active
     * @param manualTranslateMagnitude current manual translation-stick magnitude in the normalized [0, 1] range
     */
    public DriveAssistStatus(boolean autoAimRequested,
                             boolean shootBraceEligible,
                             boolean shootBraceEnabled,
                             double manualTranslateMagnitude) {
        this.autoAimRequested = autoAimRequested;
        this.shootBraceEligible = shootBraceEligible;
        this.shootBraceEnabled = shootBraceEnabled;
        this.manualTranslateMagnitude = manualTranslateMagnitude;
    }
}
