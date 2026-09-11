package edu.ftcsushi.fw.sensing.observation;

import java.util.Objects;

import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.localization.PoseEstimate;

/**
 * Immutable historical choice of one actual remembered field-location entry.
 *
 * <p>Only the selector creates results; there is no primitive factory that can manufacture live
 * memory membership. The exact input publication and chosen entry remain unchanged after memory
 * refresh, reset, failure, eviction, expiry, or STOP. {@link #hasSelection()} describes that past
 * decision; {@link #isUsable(LoopClock)} additionally checks the original last-sighting age and
 * the memory owner's current lifetime validity. A key is not a physical ball ID.</p>
 */
public final class FieldTargetSelectionResult {
    private final FieldTargetMemory.Snapshot snapshot;
    private final FieldTargetMemory.Entry entry;
    private final double maxAgeSec;
    private final double metricValue;
    private final String reason;
    private final PoseEstimate rankingPose;

    /** Retains exact evidence; only package-owned selection factories can assemble it. */
    private FieldTargetSelectionResult(FieldTargetMemory.Snapshot snapshot,
            FieldTargetMemory.Entry entry, double maxAgeSec, double metricValue, String reason,
            PoseEstimate rankingPose) {
        this.snapshot = Objects.requireNonNull(snapshot, "snapshot");
        if (!Double.isFinite(maxAgeSec) || maxAgeSec < 0) {
            throw new IllegalArgumentException("maxAgeSec must be finite and >= 0");
        }
        this.entry = entry;
        this.maxAgeSec = maxAgeSec;
        this.metricValue = metricValue;
        this.reason = Objects.requireNonNull(reason, "reason");
        if (reason.trim().isEmpty()) throw new IllegalArgumentException("reason must be nonblank");
        this.rankingPose = rankingPose;
    }

    /** Creates a choice of an exact member, never a separately authored location or key. */
    static FieldTargetSelectionResult selected(FieldTargetMemory.Snapshot snapshot,
            FieldTargetMemory.Entry entry, double maxAgeSec, double metricValue, String reason,
            PoseEstimate rankingPose) {
        Objects.requireNonNull(snapshot, "snapshot");
        Objects.requireNonNull(entry, "entry");
        if (!snapshot.entries().contains(entry)) {
            throw new IllegalArgumentException("selected entry must be an actual member of the memory snapshot");
        }
        if (!Double.isFinite(metricValue)) throw new IllegalArgumentException("metricValue must be finite");
        return new FieldTargetSelectionResult(snapshot, entry, maxAgeSec, metricValue, reason, rankingPose);
    }

    /** Retains the exact publication and optional rejected pose when no entry qualifies. */
    static FieldTargetSelectionResult none(FieldTargetMemory.Snapshot snapshot, double maxAgeSec,
            String reason, PoseEstimate rankingPose) {
        return new FieldTargetSelectionResult(snapshot, null, maxAgeSec, Double.NaN, reason, rankingPose);
    }

    /** Whether this historical result selected an entry; not current visibility or eligibility. */
    public boolean hasSelection() { return entry != null; }

    /**
     * Checks the chosen entry's owner-backed lifetime and inclusive original-sighting age.
     * A refreshed entry cannot move or redate this old choice. Historical ranking-pose age is
     * not a substitute for the downstream solve's independent current-pose freshness check.
     */
    public boolean isUsable(LoopClock clock) {
        Objects.requireNonNull(clock, "clock");
        return entry != null && entry.isUsable(clock) && entry.lastSighting().isFresh(clock, maxAgeSec);
    }

    /**
     * Returns the exact chosen historical entry.
     * @throws IllegalStateException if this result has no selection; check {@link #hasSelection()}
     */
    public FieldTargetMemory.Entry entry() {
        if (entry == null) throw new IllegalStateException("no field target selected: " + reason);
        return entry;
    }

    /** Exact input memory publication, not a synthetic frame with a common capture timestamp. */
    public FieldTargetMemory.Snapshot snapshot() { return snapshot; }

    /** Inclusive last-sighting age limit in seconds, inherited or explicitly made stricter. */
    public double maxAgeSec() { return maxAgeSec; }

    /** Policy distance in inches, or NaN if no location qualified; never measured confidence. */
    public double metricValue() { return metricValue; }

    /** Ranking or absence explanation without claiming verified identity or capture success. */
    public String reason() { return reason; }

    /**
     * Exact pose consulted for ranking, including rejected evidence, or null when none was read.
     * This is historical decision evidence, not the later spatial solve's robot pose.
     */
    public PoseEstimate rankingPose() { return rankingPose; }

    /** Returns a compact description of this immutable decision without sampling memory. */
    @Override public String toString() {
        return "FieldTargetSelectionResult{" + reason + ", hasSelection=" + hasSelection()
                + ", metricInches=" + metricValue + ", maxAgeSec=" + maxAgeSec + '}';
    }
}
