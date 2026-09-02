package edu.ftcsushi.fw.actuation;

import java.util.Objects;

import edu.ftcsushi.fw.core.source.ScalarTarget;

/**
 * Immutable capture of one scalar {@link Plant}'s cached public facts.
 *
 * <p>A snapshot does not update the Plant, poll hardware, advance a source, or add another
 * heartbeat. The default {@link Plant#snapshot()} implementation reads each cached Plant query
 * once and freezes those results. Callers should capture after an owner lifecycle operation such
 * as {@link Plant#update(edu.ftcsushi.fw.core.time.LoopClock)} or {@link Plant#stop()} has returned.
 * The capture is not an atomic-publication or cross-thread synchronization contract, so a call
 * reentrant from inside a Plant update or external callback may observe that operation's current
 * in-progress public facts.</p>
 *
 * <p>If a custom Plant query throws or violates a required non-null result contract, snapshot
 * creation throws and returns no partial value. Ordinary unavailable evidence remains represented
 * by the Plant vocabulary: {@link Double#NaN}, {@code false}, an unavailable
 * {@link PlantTargetResolution}, or the applicable {@link PlantTargetStatus}.</p>
 */
public class PlantSnapshot {

    private final boolean hasCommandTarget;
    private final double commandTarget;
    private final double requestedTarget;
    private final double appliedTarget;
    private final PlantTargetResolution targetResolution;
    private final PlantTargetStatus targetStatus;
    private final boolean hasFeedback;
    private final boolean hasMeasurement;
    private final double measurement;
    private final double requestedTargetError;
    private final double appliedTargetError;
    private final boolean atTarget;
    private final boolean atCommandTarget;

    /** Capture one Plant through its cached public query surface. */
    PlantSnapshot(Plant source) {
        Plant plant = Objects.requireNonNull(source, "source");

        boolean capturedHasCommandTarget = plant.hasCommandTarget();
        ScalarTarget capturedCommand = null;
        double capturedCommandTarget = Double.NaN;
        if (capturedHasCommandTarget) {
            capturedCommand = Objects.requireNonNull(
                    plant.commandTarget(),
                    "Plant.hasCommandTarget() was true but commandTarget() returned null");
            capturedCommandTarget = capturedCommand.get();
        }

        double capturedRequestedTarget = plant.getRequestedTarget();
        double capturedAppliedTarget = plant.getAppliedTarget();
        PlantTargetResolution capturedTargetResolution = Objects.requireNonNull(
                plant.getTargetResolution(), "Plant.getTargetResolution() returned null");
        PlantTargetStatus capturedTargetStatus = Objects.requireNonNull(
                plant.getTargetStatus(), "Plant.getTargetStatus() returned null");
        boolean capturedHasFeedback = plant.hasFeedback();
        double capturedMeasurement = plant.getMeasurement();
        boolean capturedHasMeasurement = Double.isFinite(capturedMeasurement);
        double capturedRequestedTargetError = capturedHasMeasurement
                ? capturedRequestedTarget - capturedMeasurement
                : Double.NaN;
        double capturedAppliedTargetError = capturedHasMeasurement
                ? capturedAppliedTarget - capturedMeasurement
                : Double.NaN;
        boolean capturedAtTarget = plant.atTarget();

        boolean capturedAtCommandTarget = false;
        if (capturedCommand != null && Double.isFinite(capturedCommandTarget)) {
            if (capturedTargetResolution.reportsCommandResolutionFor(capturedCommand)) {
                boolean commandSelected = capturedTargetResolution.satisfiesCommand(
                        capturedCommand, capturedCommandTarget);
                capturedAtCommandTarget = commandSelected
                        && plant.atTarget(capturedTargetResolution.target());
            } else {
                // Custom Plants without framework command provenance keep the literal command-value
                // completion contract used by feedback-aware ScalarTasks.
                capturedAtCommandTarget = plant.atTarget(capturedCommandTarget);
            }
        }

        hasCommandTarget = capturedHasCommandTarget;
        commandTarget = capturedCommandTarget;
        requestedTarget = capturedRequestedTarget;
        appliedTarget = capturedAppliedTarget;
        targetResolution = capturedTargetResolution;
        targetStatus = capturedTargetStatus;
        hasFeedback = capturedHasFeedback;
        hasMeasurement = capturedHasMeasurement;
        measurement = capturedMeasurement;
        requestedTargetError = capturedRequestedTargetError;
        appliedTargetError = capturedAppliedTargetError;
        atTarget = capturedAtTarget;
        atCommandTarget = capturedAtCommandTarget;
    }

    /** Whether the Plant's final resolver graph carries a stable command target. */
    public final boolean hasCommandTarget() {
        return hasCommandTarget;
    }

    /**
     * Return the captured live command value, in Plant units, or {@link Double#NaN} when this
     * Plant has no command target.
     *
     * <p>The value is captured independently from the last resolved requested target. An overlay
     * may mask it, an equivalent-position resolver may select another physical representative,
     * and a command changed since the last Plant update may not yet have been resolved.</p>
     */
    public final double commandTarget() {
        return commandTarget;
    }

    /** Raw target selected by the resolver on the most recent Plant update. */
    public final double requestedTarget() {
        return requestedTarget;
    }

    /** Final mechanism target selected after Plant bounds and guards. */
    public final double appliedTarget() {
        return appliedTarget;
    }

    /** Diagnostic explanation of how the most recent requested target was selected. */
    public final PlantTargetResolution targetResolution() {
        return targetResolution;
    }

    /** Diagnostic explanation of how the most recent requested target became applied. */
    public final PlantTargetStatus targetStatus() {
        return targetStatus;
    }

    /** Whether the Plant has a meaningful authoritative feedback capability. */
    public final boolean hasFeedback() {
        return hasFeedback;
    }

    /** Whether the captured measurement is finite. */
    public final boolean hasMeasurement() {
        return hasMeasurement;
    }

    /** Captured authoritative measurement in Plant units, or {@link Double#NaN}. */
    public final double measurement() {
        return measurement;
    }

    /** Captured requested-target error: {@code requestedTarget() - measurement()}. */
    public final double requestedTargetError() {
        return requestedTargetError;
    }

    /** Captured applied-target error: {@code appliedTarget() - measurement()}. */
    public final double appliedTargetError() {
        return appliedTargetError;
    }

    /** Whether the Plant reported arrival at its most recently resolved requested target. */
    public final boolean atTarget() {
        return atTarget;
    }

    /**
     * Whether the captured live command is the selected intent and the Plant proves arrival at
     * the physical target that realizes it.
     *
     * <p>For framework target graphs, this query requires matching command identity and value,
     * successful intent provenance, and physical arrival at the selected resolution target. It is
     * therefore false when an overlay, fallback, hold, planner clamp, newer command, failed
     * actuation, or terminal stop prevents the captured command from being the realized intent. A
     * custom Plant that does not publish framework command provenance falls back to its literal
     * {@link Plant#atTarget(double)} query. Plants without a command target, or with a non-finite
     * live command, return {@code false}.</p>
     */
    public final boolean atCommandTarget() {
        return atCommandTarget;
    }

    @Override
    public String toString() {
        return "PlantSnapshot{commandTarget=" + commandTarget
                + ", requestedTarget=" + requestedTarget
                + ", appliedTarget=" + appliedTarget
                + ", targetStatus=" + targetStatus.kind()
                + ", measurement=" + measurement
                + ", atTarget=" + atTarget
                + ", atCommandTarget=" + atCommandTarget + "}";
    }
}
