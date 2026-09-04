package edu.ftcsushi.robots.examples.basicflywheel;

import java.util.Objects;

import edu.ftcsushi.fw.actuation.PlantSnapshot;
import edu.ftcsushi.fw.task.Task;

/**
 * One-motor flywheel capability whose public intent is velocity in encoder ticks per second.
 *
 * <p>The number is the complete request, so this capability does not add a semantic enum or infer
 * a name from velocity. The mechanism implementation owns the command, feedback Plant, update,
 * and stop lifecycle.</p>
 */
public interface BasicFlywheel {

    /** Thin domain-named view of one immutable scalar Plant capture. */
    final class Status {
        private final PlantSnapshot snapshot;

        /** Package-owned construction prevents callers from fabricating flywheel evidence. */
        Status(PlantSnapshot snapshot) {
            this.snapshot = Objects.requireNonNull(snapshot, "snapshot");
        }

        /** Returns the captured persistent request, in native encoder ticks per second. */
        public double requestedVelocityTicksPerSec() {
            return snapshot.commandTarget();
        }

        /** Returns the captured final target after Plant bounds and guards, in ticks per second. */
        public double appliedVelocityTicksPerSec() {
            return snapshot.appliedTarget();
        }

        /** Returns the captured encoder measurement, in ticks per second, or {@code NaN}. */
        public double measuredVelocityTicksPerSec() {
            return snapshot.measurement();
        }

        /** Returns whether the captured Plant evidence proves arrival at this captured request. */
        public boolean atRequestedVelocity() {
            return snapshot.atCommandTarget();
        }

        /** Returns the underlying immutable Plant capture for advanced diagnostics. */
        public PlantSnapshot plantSnapshot() {
            return snapshot;
        }
    }

    /**
     * Replaces the persistent flywheel request without waiting for physical arrival.
     *
     * @param velocityTicksPerSec finite velocity in the configured inclusive range
     */
    void setVelocityTicksPerSec(double velocityTicksPerSec);

    /**
     * Builds a fresh single-use Task that requests velocity and waits for feedback arrival.
     *
     * <p>Timeout leaves the persistent request unchanged. Active cancellation requests zero; the
     * mechanism's next normal output heartbeat realizes that request.</p>
     *
     * @param velocityTicksPerSec finite velocity in the configured inclusive range
     * @return fresh feedback-aware velocity Task
     */
    Task setVelocityTask(double velocityTicksPerSec);

    /** Captures the current request and the Plant's latest cached feedback facts. */
    Status status();
}
