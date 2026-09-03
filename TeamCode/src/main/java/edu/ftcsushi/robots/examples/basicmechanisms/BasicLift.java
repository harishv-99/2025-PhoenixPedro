package edu.ftcsushi.robots.examples.basicmechanisms;

import java.util.Objects;

import edu.ftcsushi.fw.actuation.PositionPlantSnapshot;
import edu.ftcsushi.fw.actuation.SemanticScalarSnapshot;
import edu.ftcsushi.fw.task.Task;

/**
 * Small semantic lift API shared by the teaching TeleOps and Autos.
 *
 * <p>Callers choose a named height and inspect evidence; only the mechanism knows encoder units,
 * motor power, or how the bottom switch establishes a reference.</p>
 */
public interface BasicLift {

    /** Named robot positions used everywhere instead of repeating encoder values. */
    enum Height {
        STOWED,
        LOW,
        HIGH
    }

    /**
     * Immutable capability-shaped view of one semantic request and one Plant evidence capture.
     *
     * <p>This wrapper owns only the source snapshot. It does not republish fields, poll hardware,
     * or maintain a second status cache.</p>
     */
    final class Status {
        private final SemanticScalarSnapshot<Height, PositionPlantSnapshot> snapshot;

        /** Adapts one source-backed semantic position snapshot. */
        public Status(SemanticScalarSnapshot<Height, PositionPlantSnapshot> snapshot) {
            this.snapshot = Objects.requireNonNull(snapshot, "snapshot");
        }

        /** Returns the named height in this captured request. */
        public Height requestedHeight() {
            return snapshot.request().semantic();
        }

        /** Returns the numeric target paired with the semantic request, in mechanism inches. */
        public double requestedPositionIn() {
            return snapshot.request().commandTarget();
        }

        /** Returns the Plant's cached final target after bounds and guards, in mechanism inches. */
        public double appliedPositionIn() {
            return snapshot.plant().appliedTarget();
        }

        /** Returns the cached encoder measurement, in mechanism inches, or {@code NaN}. */
        public double measuredPositionIn() {
            return snapshot.plant().measurement();
        }

        /** Returns whether the captured Plant evidence has an established position reference. */
        public boolean referenced() {
            return snapshot.plant().isReferenced();
        }

        /** Returns whether this exact semantic request is selected and physically at target. */
        public boolean atTarget() {
            return snapshot.currentRequestAtTarget();
        }

        /** Returns the complete immutable position-Plant capture for advanced diagnostics. */
        public PositionPlantSnapshot plantSnapshot() {
            return snapshot.plant();
        }
    }

    /** Selects a persistent semantic height without waiting for physical arrival. */
    void setHeight(Height height);

    /**
     * Builds a fresh feedback-aware move to one semantic height.
     *
     * <p>Success requires the selected semantic request still to be current and its cached feedback
     * to report arrival. Timeout and active cancellation do not overwrite the latest persistent
     * request, which may have been superseded while this Task was active.</p>
     *
     * @param height non-null destination
     * @return fresh single-use move Task
     */
    Task moveTo(Height height);

    /**
     * Builds a fresh non-blocking search for the bottom reference switch.
     *
     * <p>A successful search establishes zero, then selects {@link Height#STOWED} before the
     * mechanism's downstream output phase. Timeout and active cancellation retain their truthful
     * outcomes and preserve the latest coherent semantic and numeric height request.</p>
     *
     * @return fresh single-use homing Task
     */
    Task home();

    /**
     * Captures the current semantic request together with the Plant's cached position evidence.
     * Reading this snapshot does not poll hardware or advance the mechanism.
     */
    Status status();
}
