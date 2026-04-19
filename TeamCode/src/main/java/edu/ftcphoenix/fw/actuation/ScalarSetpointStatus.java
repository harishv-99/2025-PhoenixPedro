package edu.ftcphoenix.fw.actuation;

/**
 * Runtime status produced by {@link ScalarSetpointPlanner}.
 *
 * <p>The status deliberately mirrors common {@link Plant} vocabulary: setpoint, measurement, error,
 * and at-setpoint. It also reports whether the original request was truly satisfied or merely
 * clamped to the closest reachable value.</p>
 */
public final class ScalarSetpointStatus {

    public final boolean hasSetpoint;
    public final double setpoint;
    public final double measurement;
    public final double error;
    public final boolean atSetpoint;
    public final boolean requestSatisfied;
    public final boolean reachable;
    public final boolean clamped;
    public final String selectedCandidateId;
    public final double selectedQuality;
    public final double selectedAgeSec;
    public final double selectedTimestampSec;
    public final String blockedReason;

    private ScalarSetpointStatus(boolean hasSetpoint,
                                 double setpoint,
                                 double measurement,
                                 double error,
                                 boolean atSetpoint,
                                 boolean requestSatisfied,
                                 boolean reachable,
                                 boolean clamped,
                                 String selectedCandidateId,
                                 double selectedQuality,
                                 double selectedAgeSec,
                                 double selectedTimestampSec,
                                 String blockedReason) {
        this.hasSetpoint = hasSetpoint;
        this.setpoint = setpoint;
        this.measurement = measurement;
        this.error = error;
        this.atSetpoint = atSetpoint;
        this.requestSatisfied = requestSatisfied;
        this.reachable = reachable;
        this.clamped = clamped;
        this.selectedCandidateId = selectedCandidateId;
        this.selectedQuality = selectedQuality;
        this.selectedAgeSec = selectedAgeSec;
        this.selectedTimestampSec = selectedTimestampSec;
        this.blockedReason = blockedReason;
    }

    /**
     * Creates a status with no commandable setpoint.
     */
    public static ScalarSetpointStatus blocked(double measurement, String reason) {
        return new ScalarSetpointStatus(false, Double.NaN, measurement, Double.NaN,
                false, false, false, false, "", Double.NaN, Double.NaN, Double.NaN,
                reason != null ? reason : "blocked");
    }

    /**
     * Creates a commandable setpoint status.
     */
    static ScalarSetpointStatus of(double setpoint,
                                   double measurement,
                                   boolean reachable,
                                   boolean clamped,
                                   ScalarSetpointCandidate candidate,
                                   double atSetpointTolerance,
                                   double requestSatisfiedTolerance) {
        double error = setpoint - measurement;
        boolean atSetpoint = Double.isFinite(error) && Math.abs(error) <= atSetpointTolerance;
        boolean requestSatisfied = reachable && !clamped && atSetpoint
                || reachable && !clamped && Math.abs(error) <= requestSatisfiedTolerance;
        return new ScalarSetpointStatus(true,
                setpoint,
                measurement,
                error,
                atSetpoint,
                requestSatisfied,
                reachable,
                clamped,
                candidate != null ? candidate.id : "hold",
                candidate != null ? candidate.quality : 1.0,
                candidate != null ? candidate.ageSec : 0.0,
                candidate != null ? candidate.timestampSec : Double.NaN,
                "");
    }

    /**
     * Returns true when the planner produced a scalar setpoint.
     */
    public boolean hasSetpoint() {
        return hasSetpoint;
    }

    /**
     * Returns the selected setpoint in native plant units.
     */
    public double getSetpoint() {
        return setpoint;
    }

    /**
     * Returns the measured value in native plant units.
     */
    public double getMeasurement() {
        return measurement;
    }

    /**
     * Returns setpoint - measurement in native plant units.
     */
    public double getError() {
        return error;
    }

    /**
     * Returns true when measurement is within the planner's at-setpoint tolerance.
     */
    public boolean atSetpoint() {
        return atSetpoint;
    }

    /**
     * Returns true when the original request was actually satisfied, not just clamped.
     */
    public boolean requestSatisfied() {
        return requestSatisfied;
    }

    /**
     * Returns the selected candidate id or an empty string when blocked.
     */
    public String selectedCandidateId() {
        return selectedCandidateId;
    }

    /**
     * Returns the no-setpoint reason when blocked, otherwise an empty string.
     */
    public String blockedReason() {
        return blockedReason;
    }

    @Override
    public String toString() {
        return "ScalarSetpointStatus{hasSetpoint=" + hasSetpoint
                + ", setpoint=" + setpoint
                + ", measurement=" + measurement
                + ", error=" + error
                + ", atSetpoint=" + atSetpoint
                + ", requestSatisfied=" + requestSatisfied
                + ", reachable=" + reachable
                + ", clamped=" + clamped
                + ", selectedCandidateId='" + selectedCandidateId + '\''
                + ", blockedReason='" + blockedReason + "'}";
    }
}
