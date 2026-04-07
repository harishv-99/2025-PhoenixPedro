package edu.ftcphoenix.robots.phoenix;

/**
 * Immutable status snapshot for the shooter subsystem.
 *
 * <p>Outside code should read this snapshot instead of reaching into the subsystem's live state or
 * directly formatting FTC telemetry inside the mechanism owner.</p>
 */
public final class ShooterStatus {
    public final boolean flywheelEnabled;
    public final boolean pidfEnabled;
    public final String pidfWarning;
    public final double selectedVelocityNative;
    public final double flywheelTargetNative;
    public final double flywheelMeasuredNative;
    public final double flywheelErrorNative;
    public final double flywheelErrorAbsNative;
    public final double flywheelToleranceNative;
    public final double flywheelToleranceBelowNative;
    public final double flywheelToleranceAboveNative;
    public final double flywheelAccelNativePerSec;
    public final double flywheelAccelAbsNativePerSec;
    public final double readyLeadSec;
    public final double predictedFlywheelAbsNative;
    public final double predictedFlywheelErrorNative;
    public final boolean flywheelAtSetpoint;
    public final boolean ready;
    public final int feedBacklog;
    public final int feedQueued;
    public final boolean feedActive;
    public final double feedOutput;

    /**
     * Creates an immutable shooter status snapshot.
     *
     * @param flywheelEnabled              whether the flywheel is currently enabled
     * @param pidfEnabled                  whether motor-controller velocity PIDF was requested in config
     * @param pidfWarning                  warning text emitted while attempting to apply PIDF, or {@code null}
     * @param selectedVelocityNative       operator-selected flywheel target in motor native units
     * @param flywheelTargetNative         commanded flywheel target in native velocity units
     * @param flywheelMeasuredNative       measured flywheel velocity in native units
     * @param flywheelErrorNative          measured minus target flywheel velocity in native units
     * @param flywheelErrorAbsNative       absolute flywheel velocity error in native units
     * @param flywheelToleranceNative      plant setpoint tolerance in native units
     * @param flywheelToleranceBelowNative lower ready-band tolerance in native units
     * @param flywheelToleranceAboveNative upper ready-band tolerance in native units
     * @param flywheelAccelNativePerSec    measured flywheel acceleration in native units per second
     * @param flywheelAccelAbsNativePerSec absolute measured flywheel acceleration in native units per second
     * @param readyLeadSec                 prediction horizon used by the ready calculation, in seconds
     * @param predictedFlywheelAbsNative   predicted absolute flywheel speed at the feed horizon, in native units
     * @param predictedFlywheelErrorNative predicted absolute-speed error at the feed horizon, in native units
     * @param flywheelAtSetpoint           whether the flywheel plant currently reports at-setpoint
     * @param ready                        whether the Phoenix ready gate currently allows feeding
     * @param feedBacklog                  number of backlog slots still requested by the queue owner
     * @param feedQueued                   number of queued feed tasks waiting behind the active task
     * @param feedActive                   whether a feed task is currently driving the path
     * @param feedOutput                   current output value coming from the feed queue
     */
    public ShooterStatus(boolean flywheelEnabled,
                         boolean pidfEnabled,
                         String pidfWarning,
                         double selectedVelocityNative,
                         double flywheelTargetNative,
                         double flywheelMeasuredNative,
                         double flywheelErrorNative,
                         double flywheelErrorAbsNative,
                         double flywheelToleranceNative,
                         double flywheelToleranceBelowNative,
                         double flywheelToleranceAboveNative,
                         double flywheelAccelNativePerSec,
                         double flywheelAccelAbsNativePerSec,
                         double readyLeadSec,
                         double predictedFlywheelAbsNative,
                         double predictedFlywheelErrorNative,
                         boolean flywheelAtSetpoint,
                         boolean ready,
                         int feedBacklog,
                         int feedQueued,
                         boolean feedActive,
                         double feedOutput) {
        this.flywheelEnabled = flywheelEnabled;
        this.pidfEnabled = pidfEnabled;
        this.pidfWarning = pidfWarning;
        this.selectedVelocityNative = selectedVelocityNative;
        this.flywheelTargetNative = flywheelTargetNative;
        this.flywheelMeasuredNative = flywheelMeasuredNative;
        this.flywheelErrorNative = flywheelErrorNative;
        this.flywheelErrorAbsNative = flywheelErrorAbsNative;
        this.flywheelToleranceNative = flywheelToleranceNative;
        this.flywheelToleranceBelowNative = flywheelToleranceBelowNative;
        this.flywheelToleranceAboveNative = flywheelToleranceAboveNative;
        this.flywheelAccelNativePerSec = flywheelAccelNativePerSec;
        this.flywheelAccelAbsNativePerSec = flywheelAccelAbsNativePerSec;
        this.readyLeadSec = readyLeadSec;
        this.predictedFlywheelAbsNative = predictedFlywheelAbsNative;
        this.predictedFlywheelErrorNative = predictedFlywheelErrorNative;
        this.flywheelAtSetpoint = flywheelAtSetpoint;
        this.ready = ready;
        this.feedBacklog = feedBacklog;
        this.feedQueued = feedQueued;
        this.feedActive = feedActive;
        this.feedOutput = feedOutput;
    }
}
