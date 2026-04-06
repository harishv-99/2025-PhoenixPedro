package edu.ftcphoenix.fw.sensing.vision.apriltag;

import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Objects;
import java.util.Set;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Factory helpers for building {@link TagSelectionSource}s.
 *
 * <p>The selector builder is intentionally small and principle-driven:</p>
 * <ul>
 *   <li><b>One loop, one heartbeat:</b> selectors are stateful sources, sampled once per loop.</li>
 *   <li><b>Selection is shared:</b> build one selector and reuse it for guidance, telemetry,
 *       shooter logic, and gating.</li>
 *   <li><b>Stickiness is explicit:</b> continuous preview, sticky-while-enabled, and sticky-until-
 *       reset are different workflows and should read differently at the call site.</li>
 * </ul>
 */
public final class TagSelections {

    private TagSelections() {
        // Utility class.
    }

    /**
     * Starts building a selection source from raw detections.
     */
    public static Builder from(Source<AprilTagDetections> detections) {
        return new Builder(detections);
    }

    /**
     * Mutable builder used to assemble a tag selector.
     */
    public static final class Builder {
        private final Source<AprilTagDetections> detections;
        private Set<Integer> candidateIds = Collections.emptySet();
        private double maxAgeSec = 0.25;
        private TagSelectionPolicy policy = TagSelectionPolicies.closestRange();
        private Mode mode = Mode.CONTINUOUS;
        private BooleanSource enabled = null;
        private double reacquireAfterLossSec = Double.POSITIVE_INFINITY;

        Builder(Source<AprilTagDetections> detections) {
            this.detections = Objects.requireNonNull(detections, "detections");
        }

        /**
         * Candidate tag IDs to choose among.
         */
        public Builder among(Set<Integer> candidateIds) {
            Objects.requireNonNull(candidateIds, "candidateIds");
            if (candidateIds.isEmpty()) {
                throw new IllegalArgumentException("candidateIds must not be empty");
            }
            LinkedHashSet<Integer> ids = new LinkedHashSet<Integer>();
            for (Integer id : candidateIds) {
                if (id == null) {
                    throw new IllegalArgumentException("candidateIds must not contain null");
                }
                if (id.intValue() < 0) {
                    throw new IllegalArgumentException("candidateIds must be non-negative");
                }
                ids.add(id);
            }
            this.candidateIds = Collections.unmodifiableSet(ids);
            return this;
        }

        /**
         * Freshness window for candidates.
         */
        public Builder freshWithin(double maxAgeSec) {
            this.maxAgeSec = maxAgeSec;
            return this;
        }

        /**
         * Selection policy.
         */
        public Builder choose(TagSelectionPolicy policy) {
            this.policy = Objects.requireNonNull(policy, "policy");
            return this;
        }

        /**
         * The selected tag tracks the current preview every loop.
         */
        public Builder continuous() {
            this.mode = Mode.CONTINUOUS;
            this.enabled = null;
            return this;
        }

        /**
         * Latch a tag while {@code enabled} is true, previewing while false.
         */
        public Builder stickyWhen(BooleanSource enabled) {
            this.mode = Mode.STICKY_WHEN;
            this.enabled = Objects.requireNonNull(enabled, "enabled");
            return this;
        }

        /**
         * Latch the first valid tag and keep it until {@link TagSelectionSource#reset()} is called.
         */
        public Builder stickyUntilReset() {
            this.mode = Mode.STICKY_UNTIL_RESET;
            this.enabled = null;
            return this;
        }

        /**
         * Allows a sticky selector to clear / reacquire after the selected tag has been lost for
         * this long. Leave at positive infinity to keep the selected identity until reset.
         */
        public Builder reacquireAfterLoss(double reacquireAfterLossSec) {
            this.reacquireAfterLossSec = reacquireAfterLossSec;
            return this;
        }

        /**
         * Builds the selector.
         */
        public TagSelectionSource build() {
            if (candidateIds.isEmpty()) {
                throw new IllegalStateException("TagSelections requires among(...) before build()");
            }
            if (!Double.isFinite(maxAgeSec) || maxAgeSec < 0.0) {
                throw new IllegalStateException("freshWithin(...): maxAgeSec must be >= 0");
            }
            if (!Double.isFinite(reacquireAfterLossSec) && reacquireAfterLossSec != Double.POSITIVE_INFINITY) {
                throw new IllegalStateException("reacquireAfterLoss(...): value must be finite or +infinity");
            }
            if (Double.isFinite(reacquireAfterLossSec) && reacquireAfterLossSec < 0.0) {
                throw new IllegalStateException("reacquireAfterLoss(...): value must be >= 0");
            }
            if (mode == Mode.STICKY_WHEN && enabled == null) {
                throw new IllegalStateException("stickyWhen(...) requires a BooleanSource");
            }
            return new BuiltSelectionSource(detections, candidateIds, maxAgeSec, policy, mode, enabled, reacquireAfterLossSec);
        }
    }

    private enum Mode {
        CONTINUOUS,
        STICKY_WHEN,
        STICKY_UNTIL_RESET
    }

    private static final class BuiltSelectionSource implements TagSelectionSource {
        private final Source<AprilTagDetections> detections;
        private final Set<Integer> candidateIds;
        private final double maxAgeSec;
        private final TagSelectionPolicy policy;
        private final Mode mode;
        private final BooleanSource enabled;
        private final double reacquireAfterLossSec;

        private long lastCycle = Long.MIN_VALUE;
        private TagSelectionResult last = TagSelectionResult.none(Collections.<Integer>emptySet());

        private boolean prevEnabled = false;
        private int selectedTagId = -1;
        private boolean latched = false;
        private String lastPolicyName = "none";
        private String lastReason = "no selection";
        private double lastMetricValue = Double.NaN;
        private double lostSinceSec = Double.NaN;

        BuiltSelectionSource(Source<AprilTagDetections> detections,
                             Set<Integer> candidateIds,
                             double maxAgeSec,
                             TagSelectionPolicy policy,
                             Mode mode,
                             BooleanSource enabled,
                             double reacquireAfterLossSec) {
            this.detections = detections;
            this.candidateIds = candidateIds;
            this.maxAgeSec = maxAgeSec;
            this.policy = policy;
            this.mode = mode;
            this.enabled = enabled;
            this.reacquireAfterLossSec = reacquireAfterLossSec;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public Set<Integer> candidateIds() {
            return candidateIds;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public TagSelectionResult get(LoopClock clock) {
            long cyc = clock.cycle();
            if (cyc == lastCycle) {
                return last;
            }
            lastCycle = cyc;

            AprilTagDetections dets = Objects.requireNonNull(detections.get(clock), "detections returned null");
            List<AprilTagObservation> candidates = dets.freshMatching(candidateIds, maxAgeSec);
            LinkedHashSet<Integer> visibleIds = new LinkedHashSet<Integer>();
            for (AprilTagObservation obs : candidates) {
                visibleIds.add(obs.id);
            }

            TagSelectionChoice preview = policy.choose(candidates);
            if (preview != null) {
                lastPolicyName = preview.policyName;
                lastReason = preview.reason;
                lastMetricValue = preview.metricValue;
            }

            switch (mode) {
                case CONTINUOUS:
                    last = buildContinuousResult(preview, visibleIds);
                    prevEnabled = false;
                    return last;
                case STICKY_WHEN:
                    return stepStickyWhen(clock, preview, candidates, visibleIds);
                case STICKY_UNTIL_RESET:
                default:
                    return stepStickyUntilReset(clock, preview, candidates, visibleIds);
            }
        }

        private TagSelectionResult buildContinuousResult(TagSelectionChoice preview, Set<Integer> visibleIds) {
            if (preview == null) {
                return new TagSelectionResult(
                        false,
                        -1,
                        AprilTagObservation.noTarget(Double.POSITIVE_INFINITY),
                        false,
                        -1,
                        false,
                        false,
                        AprilTagObservation.noTarget(Double.POSITIVE_INFINITY),
                        visibleIds,
                        lastPolicyName,
                        "no preview candidate",
                        Double.NaN
                );
            }
            return new TagSelectionResult(
                    true,
                    preview.observation.id,
                    preview.observation,
                    true,
                    preview.observation.id,
                    false,
                    true,
                    preview.observation,
                    visibleIds,
                    preview.policyName,
                    preview.reason,
                    preview.metricValue
            );
        }

        private TagSelectionResult stepStickyWhen(LoopClock clock,
                                                  TagSelectionChoice preview,
                                                  List<AprilTagObservation> candidates,
                                                  Set<Integer> visibleIds) {
            boolean enabledNow = enabled.getAsBoolean(clock);
            boolean rising = enabledNow && !prevEnabled;
            prevEnabled = enabledNow;

            if (!enabledNow) {
                clearSelection();
                last = new TagSelectionResult(
                        preview != null,
                        preview != null ? preview.observation.id : -1,
                        preview != null ? preview.observation : AprilTagObservation.noTarget(Double.POSITIVE_INFINITY),
                        false,
                        -1,
                        false,
                        false,
                        AprilTagObservation.noTarget(Double.POSITIVE_INFINITY),
                        visibleIds,
                        preview != null ? preview.policyName : lastPolicyName,
                        preview != null ? preview.reason : "selection inactive",
                        preview != null ? preview.metricValue : Double.NaN
                );
                return last;
            }

            if (rising || selectedTagId < 0) {
                if (preview != null) {
                    latch(preview);
                }
            }

            last = buildStickyResult(clock, preview, candidates, visibleIds);
            return last;
        }

        private TagSelectionResult stepStickyUntilReset(LoopClock clock,
                                                        TagSelectionChoice preview,
                                                        List<AprilTagObservation> candidates,
                                                        Set<Integer> visibleIds) {
            if (selectedTagId < 0 && preview != null) {
                latch(preview);
            }
            last = buildStickyResult(clock, preview, candidates, visibleIds);
            return last;
        }

        private TagSelectionResult buildStickyResult(LoopClock clock,
                                                     TagSelectionChoice preview,
                                                     List<AprilTagObservation> candidates,
                                                     Set<Integer> visibleIds) {
            AprilTagObservation selectedObs = findById(candidates, selectedTagId);
            boolean hasFreshSelected = selectedObs != null && selectedObs.hasTarget;

            if (selectedTagId >= 0) {
                if (hasFreshSelected) {
                    lostSinceSec = Double.NaN;
                } else if (Double.isNaN(lostSinceSec)) {
                    lostSinceSec = clock.nowSec();
                }

                if (!hasFreshSelected && Double.isFinite(reacquireAfterLossSec)
                        && clock.nowSec() - lostSinceSec >= reacquireAfterLossSec) {
                    if (preview != null) {
                        latch(preview);
                        selectedObs = preview.observation;
                        hasFreshSelected = true;
                    } else {
                        clearSelection();
                        selectedObs = AprilTagObservation.noTarget(Double.POSITIVE_INFINITY);
                        hasFreshSelected = false;
                    }
                }
            }

            return new TagSelectionResult(
                    preview != null,
                    preview != null ? preview.observation.id : -1,
                    preview != null ? preview.observation : AprilTagObservation.noTarget(Double.POSITIVE_INFINITY),
                    selectedTagId >= 0,
                    selectedTagId,
                    latched,
                    hasFreshSelected,
                    hasFreshSelected ? selectedObs : AprilTagObservation.noTarget(Double.POSITIVE_INFINITY),
                    visibleIds,
                    preview != null ? preview.policyName : lastPolicyName,
                    preview != null ? preview.reason : lastReason,
                    preview != null ? preview.metricValue : lastMetricValue
            );
        }

        private static AprilTagObservation findById(List<AprilTagObservation> candidates, int id) {
            if (id < 0) {
                return null;
            }
            for (AprilTagObservation obs : candidates) {
                if (obs != null && obs.hasTarget && obs.id == id) {
                    return obs;
                }
            }
            return null;
        }

        private void latch(TagSelectionChoice choice) {
            selectedTagId = choice.observation.id;
            latched = true;
            lastPolicyName = choice.policyName;
            lastReason = choice.reason;
            lastMetricValue = choice.metricValue;
            lostSinceSec = Double.NaN;
        }

        private void clearSelection() {
            selectedTagId = -1;
            latched = false;
            lostSinceSec = Double.NaN;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public void reset() {
            detections.reset();
            if (enabled != null) {
                enabled.reset();
            }
            lastCycle = Long.MIN_VALUE;
            last = TagSelectionResult.none(Collections.<Integer>emptySet());
            prevEnabled = false;
            clearSelection();
            lastPolicyName = "none";
            lastReason = "no selection";
            lastMetricValue = Double.NaN;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public void debugDump(DebugSink dbg, String prefix) {
            if (dbg == null) return;
            String p = (prefix == null || prefix.isEmpty()) ? "tagSelection" : prefix;
            dbg.addData(p + ".candidateIds", candidateIds.toString())
                    .addData(p + ".maxAgeSec", maxAgeSec)
                    .addData(p + ".mode", mode.name())
                    .addData(p + ".selectedTagId", selectedTagId)
                    .addData(p + ".latched", latched)
                    .addData(p + ".reason", lastReason)
                    .addData(p + ".metricValue", lastMetricValue);
            detections.debugDump(dbg, p + ".detections");
        }
    }
}
