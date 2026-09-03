package edu.ftcsushi.robots.examples.reference.capability.launcher;

import java.util.Objects;

import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.robots.examples.reference.capability.flywheel.ReferenceFlywheels;

/** Object-release capability composed over the separately owned paired flywheels. */
public interface ReferenceLauncher {

    /** Immutable launcher evidence composed from one flywheel publication and feed state. */
    final class Status {
        private final ReferenceFlywheels.Status flywheels;
        private final boolean objectPresent;
        private final boolean transferPulseActive;

        /** Package-owned construction keeps the composed publication coherent. */
        Status(ReferenceFlywheels.Status flywheels,
               boolean objectPresent,
               boolean transferPulseActive) {
            this.flywheels = Objects.requireNonNull(flywheels, "flywheels");
            this.objectPresent = objectPresent;
            this.transferPulseActive = transferPulseActive;
        }

        /** Returns the complete grouped and independent paired-flywheel evidence. */
        public ReferenceFlywheels.Status flywheels() {
            return flywheels;
        }

        /** Returns the captured conditioned object-sensor state. */
        public boolean objectPresent() {
            return objectPresent;
        }

        /** Returns whether the captured temporary transfer override was active. */
        public boolean transferPulseActive() {
            return transferPulseActive;
        }
    }

    /** Returns the separately owned numeric paired-flywheel capability. */
    ReferenceFlywheels flywheels();

    /**
     * Invalidates every launch Task created before this call and requests active-match idle.
     *
     * <p>The request clears temporary transfer work, retracts the release, and requests zero
     * flywheel velocity. A later normal output heartbeat realizes those source requests.</p>
     */
    void abortLaunches();

    /**
     * Builds a fresh single-use spin-up, release, transfer, and cleanup Task.
     *
     * <p>Both independent wheels must be ready before feed begins. Success, spin-up timeout, and
     * active cancellation all request launcher idle during terminal cleanup. Success means this
     * software sequence completed; it does not prove that an object released or scored.</p>
     */
    Task launchOne();

    /** Returns the latest complete cached flywheel, object, and transfer publication. */
    Status status();
}
