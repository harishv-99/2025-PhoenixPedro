package edu.ftcsushi.robots.examples.reference.capability.lift;

import java.util.Objects;

import edu.ftcsushi.fw.actuation.PositionPlantSnapshot;
import edu.ftcsushi.fw.actuation.SemanticScalarCommand;
import edu.ftcsushi.fw.actuation.SemanticScalarSnapshot;
import edu.ftcsushi.fw.task.Task;

/** Season-neutral referenced lift capability. */
public interface ReferenceLift {
    enum Height {
        STOWED,
        LOW,
        HIGH
    }

    /** Domain-named view of one coherent lift request and cached Plant snapshot. */
    final class Status {
        private final SemanticScalarSnapshot<Height, PositionPlantSnapshot> delegate;

        /** Wraps one coherent semantic request and position snapshot. */
        public Status(SemanticScalarSnapshot<Height, PositionPlantSnapshot> delegate) {
            this.delegate = Objects.requireNonNull(delegate, "delegate");
        }

        /** Returns the selected semantic height. */
        public Height requestedHeight() {
            return delegate.request().semantic();
        }

        /** Returns the position paired with the selected semantic height, in mechanism inches. */
        public double requestedPositionIn() {
            return delegate.request().commandTarget();
        }

        /** Returns the Plant's cached final target after bounds and guards, in mechanism inches. */
        public double appliedPositionIn() {
            return delegate.plant().appliedTarget();
        }

        /** Returns the cached position measurement in mechanism inches, or {@code NaN}. */
        public double measuredPositionIn() {
            return delegate.plant().measurement();
        }

        /** Returns whether the Plant has established its position reference. */
        public boolean referenced() {
            return delegate.plant().isReferenced();
        }

        /** Returns whether cached feedback proves arrival for this exact request. */
        public boolean atTarget() {
            return delegate.currentRequestAtTarget();
        }

        /** Returns the underlying immutable Plant snapshot for advanced diagnostics. */
        public PositionPlantSnapshot plantSnapshot() {
            return delegate.plant();
        }

        boolean isAtTargetFor(SemanticScalarCommand.Request<Height> request) {
            return delegate.request() == request && delegate.currentRequestAtTarget();
        }
    }

    /** Selects a persistent semantic height without waiting for physical arrival. */
    void setHeight(Height height);

    /**
     * Builds a fresh feedback-aware move to one semantic height.
     *
     * <p>The request is made when the returned Task starts. Success requires that semantic height
     * still to be current and its cached feedback to report arrival. Timeout and active
     * cancellation do not overwrite the latest persistent request, which may have been superseded
     * while this Task was active.</p>
     *
     * @param height non-null semantic destination
     * @return fresh single-use feedback move
     */
    Task moveTo(Height height);

    /**
     * Builds a fresh non-blocking bottom-switch reference search.
     *
     * <p>Success establishes the position reference, then selects {@link Height#STOWED} before the
     * mechanism's downstream output phase. A search timeout remains {@code TIMEOUT}, and active
     * cancellation remains {@code CANCELLED}; both preserve the latest coherent semantic and
     * numeric height request.</p>
     *
     * @return fresh single-use homing Task
     */
    Task home();

    /** Returns a domain-named view of the current request and cached position evidence. */
    Status status();
}
