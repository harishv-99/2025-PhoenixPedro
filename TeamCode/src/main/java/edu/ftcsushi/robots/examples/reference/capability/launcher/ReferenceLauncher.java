package edu.ftcsushi.robots.examples.reference.capability.launcher;

import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.robots.examples.reference.capability.flywheel.ReferenceFlywheels;
import edu.ftcsushi.robots.examples.reference.capability.inventory.ReferenceInventoryStatusService;

/**
 * One bounded, evidence-confirmed feed in an independent software teaching fixture.
 *
 * <p>The first inventory position is the authored staged location. Observing that position become
 * vacant does not prove that feeding caused movement, that an object launched, or that it scored.
 * Idle/retraction and loaded-wheel criteria require a separate adopting-robot safety review.</p>
 */
public interface ReferenceLauncher {
    /** Current phase, or the last phase in which the latest owned attempt ended. */
    enum Phase { IDLE, SETTLING, RELEASING, TRANSFERRING, CONFIRMING }

    /** Cached reason for an attempt's ending; NONE means no ending has been selected. */
    enum Reason {
        NONE, DEPARTURE_OBSERVED, PREREQUISITE_TIMEOUT, DEPARTURE_TIMEOUT,
        EVIDENCE_LOST, WHEEL_SPEED_LOST, REQUEST_CHANGED, CANCELLED, ABORTED,
        STOPPED, CLOCK_RESET, BUSY, RECOVERY_REQUIRED, INVALIDATED, FAILED
    }

    /** No-motion acknowledgement result; each rejection names the prerequisite to inspect. */
    enum RecoveryResult {
        ACKNOWLEDGED, NOT_REQUIRED, ATTEMPT_ACTIVE, WAITING_FOR_IDLE,
        INVENTORY_UNAVAILABLE, STOPPED
    }

    /**
     * Immutable cached evidence and latest owned-attempt policy, never a fresh hardware poll.
     * Request/terminal changes can republish policy without refreshing the sampling timestamp.
     */
    final class Status {
        private final ReferenceFlywheels.Status flywheels;
        private final ReferenceInventoryStatusService.Status inventory;
        private final Phase phase;
        private final Reason reason;
        private final boolean attemptActive;
        private final boolean recoveryRequired;
        private final boolean transferActive;
        private final LoopTimestamp sampledAt;
        private final long sampleCycle;

        /** Only the mechanism owner assembles complete cached publications. */
        Status(ReferenceFlywheels.Status flywheels,
               ReferenceInventoryStatusService.Status inventory, Phase phase, Reason reason,
               boolean attemptActive, boolean recoveryRequired, boolean transferActive,
               LoopTimestamp sampledAt, long sampleCycle) {
            this.flywheels = flywheels;
            this.inventory = inventory;
            this.phase = phase;
            this.reason = reason;
            this.attemptActive = attemptActive;
            this.recoveryRequired = recoveryRequired;
            this.transferActive = transferActive;
            this.sampledAt = sampledAt;
            this.sampleCycle = sampleCycle;
        }

        /** Last complete independent wheel observations and their request provenance. */
        public ReferenceFlywheels.Status flywheels() { return flywheels; }

        /** Last observations; firstPositionOccupied is this fixture's staged sensor. */
        public ReferenceInventoryStatusService.Status inventory() { return inventory; }

        /** Latest owned attempt's current or terminal phase, retained after idle cleanup. */
        public Phase phase() { return phase; }

        /** Latest owned attempt's reason; an old Task retains its own reason in debugDump. */
        public Reason reason() { return reason; }

        /** Whether one started attempt currently owns the feed commands. */
        public boolean attemptActive() { return attemptActive; }

        /** Whether an uncertain feed requires explicit no-motion acknowledgement before retry. */
        public boolean recoveryRequired() { return recoveryRequired; }

        /** Whether the last complete output publication applied nonzero transfer power. */
        public boolean transferActive() { return transferActive; }

        /** Complete output/software sampling time; unavailable before sampling or after STOP. */
        public LoopTimestamp sampledAt() { return sampledAt; }

        /** Successful output cycle, or -1 when unavailable. */
        public long sampleCycle() { return sampleCycle; }
    }

    /** Other wheel requests reset settling; once feeding begins, a new request ends that attempt. */
    ReferenceFlywheels flywheels();

    /**
     * Invalidates all previously constructed feed Tasks and requests idle through the normal graph.
     * Active feeding requires acknowledgement afterward. The next output realizes idle; this is
     * not terminal Plant stop, a queue resume, or proof that physical interruption is safe.
     */
    void abortFeedAttempts();

    /**
     * Builds one fresh single-use attempt without starting work or reserving hardware.
     *
     * <p>Start claims exclusive ownership, requests wheel speed, then waits up to the configured
     * prerequisite bound for fresh staging and sampled paired settling. Release and transfer are
     * bounded. SUCCESS requires a later conditioned staged-vacated observation after an occupied
     * observation at/after release realization, plus successful timed phases. TIMEOUT names an
     * elapsed prerequisite/departure bound; named evidence/ownership losses are CANCELLED with a
     * separate Reason. Every owned ending requests idle. Uncertain feeding latches recovery.</p>
     *
     * <p>Concurrent starts, invalidated Tasks, or recovery-required starts end without commanding.
     * The mechanism owns no internal queue. A client's root runner can still queue fresh Tasks;
     * TeleOp must explicitly choose whether extra button presses should be admitted.</p>
     */
    Task feedOne();

    /**
     * Attempts software re-arming without commanding motion or reviving previously created work.
     * Requires no active attempt, a fresh post-failure idle-command publication, and fresh consistent
     * inventory. Idle commands do not establish stopped wheels; acknowledgement never proves a jam
     * cleared. This reads cached evidence against the owner's shared clock, not hardware.
     */
    RecoveryResult acknowledgeRecovery();

    /** Returns cached observations and policy; never samples or advances the mechanism. */
    Status status();
}
