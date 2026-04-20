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
 * <p>A tag selector turns raw AprilTag detections into one semantic answer to "which tag does this
 * behavior mean right now?" The staged builder intentionally asks the required questions in order:</p>
 * <ol>
 *   <li>which tag IDs are candidates,</li>
 *   <li>how fresh a detection must be,</li>
 *   <li>which policy chooses among visible candidates, and</li>
 *   <li>whether the selector previews continuously or latches a sticky selection.</li>
 * </ol>
 *
 * <p>Sticky-only loss behavior is only available after choosing a sticky mode. That keeps continuous
 * preview selectors from seeing irrelevant options such as reacquisition timing.</p>
 */
public final class TagSelections {

    private TagSelections() {
        // Utility class.
    }

    /**
     * Starts building a selection source from raw detections.
     *
     * @param detections source of raw AprilTag detections sampled by this selector
     * @return the first staged builder step, which requires candidate IDs
     */
    public static CandidateStep from(Source<AprilTagDetections> detections) {
        return new Builder(detections);
    }

    /**
     * First selection-builder step. Choose the set of IDs the selector is allowed to consider.
     */
    public interface CandidateStep {
        /**
         * Candidate tag IDs to choose among.
         *
         * <p>The order is preserved for policies that care about configured priority, but the
         * default policies may ignore order.</p>
         *
         * @param candidateIds non-empty set of non-negative AprilTag IDs
         * @return the freshness step
         */
        FreshnessStep among(Set<Integer> candidateIds);
    }

    /**
     * Freshness step. Choose how old a detection may be before it is ignored.
     */
    public interface FreshnessStep {
        /**
         * Uses only detections whose sensor-frame age is less than or equal to this many seconds.
         *
         * @param maxAgeSec maximum allowed detection age in seconds; must be finite and non-negative
         * @return the policy step
         */
        PolicyStep freshWithinSec(double maxAgeSec);
    }

    /**
     * Policy step. Choose how a preview winner is selected from the fresh candidates.
     */
    public interface PolicyStep {
        /**
         * Uses the supplied stateless policy to choose the preview winner each loop.
         *
         * @param policy policy that chooses among fresh candidate observations
         * @return the mode step
         */
        ModeStep choose(TagSelectionPolicy policy);
    }

    /**
     * Mode step. Choose whether selection is continuous or sticky.
     */
    public interface ModeStep {
        /**
         * Tracks the current preview every loop. The preview and selected tag are the same.
         *
         * @return a build step with no sticky-only options
         */
        BuildStep continuous();

        /**
         * Latches a selected tag while {@code enabled} is true and clears the selection while false.
         *
         * <p>After this choice, explicitly choose whether the selector should hold the original
         * tag while enabled or reacquire a different visible tag after loss.</p>
         *
         * @param enabled source that owns the sticky-enabled lifecycle
         * @return the sticky-while-enabled loss-behavior step
         */
        StickyWhenLossStep stickyWhen(BooleanSource enabled);

        /**
         * Latches the first valid tag and keeps selection state until {@link TagSelectionSource#reset()}
         * or until the configured sticky loss behavior clears/reacquires it.
         *
         * @return the sticky-until-reset loss-behavior step
         */
        StickyUntilResetLossStep stickyUntilReset();
    }

    /**
     * Loss-behavior step for selectors that are sticky only while an enable source is true.
     */
    public interface StickyWhenLossStep {
        /**
         * Keeps the selected tag identity while enabled, even if it is temporarily not fresh.
         * The selected identity is cleared as soon as the enable source becomes false.
         *
         * @return the build step
         */
        BuildStep holdUntilDisabled();

        /**
         * Allows the selector to clear or reacquire after the selected tag has been lost for the
         * supplied number of seconds while still enabled.
         *
         * @param reacquireAfterLossSec loss duration in seconds before a sticky selector may
         *                              reacquire; must be finite and non-negative
         * @return the build step
         */
        BuildStep reacquireAfterLossSec(double reacquireAfterLossSec);
    }

    /**
     * Loss-behavior step for selectors that stay sticky until reset.
     */
    public interface StickyUntilResetLossStep {
        /**
         * Keeps the selected tag identity until {@link TagSelectionSource#reset()} is called.
         *
         * @return the build step
         */
        BuildStep holdUntilReset();

        /**
         * Allows the selector to clear or reacquire after the selected tag has been lost for the
         * supplied number of seconds.
         *
         * @param reacquireAfterLossSec loss duration in seconds before a sticky selector may
         *                              reacquire; must be finite and non-negative
         * @return the build step
         */
        BuildStep reacquireAfterLossSec(double reacquireAfterLossSec);
    }

    /**
     * Final build step. All required selection questions have been answered.
     */
    public interface BuildStep {
        /**
         * Builds the stateful selector source.
         */
        TagSelectionSource build();
    }

    private static final class Builder implements CandidateStep,
            FreshnessStep,
            PolicyStep,
            ModeStep,
            StickyWhenLossStep,
            StickyUntilResetLossStep,
            BuildStep {
        private final Source<AprilTagDetections> detections;
        private Set<Integer> candidateIds;
        private double maxAgeSec = Double.NaN;
        private TagSelectionPolicy policy;
        private Mode mode;
        private BooleanSource enabled;
        private double reacquireAfterLossSec = Double.POSITIVE_INFINITY;

        Builder(Source<AprilTagDetections> detections) {
            this.detections = Objects.requireNonNull(detections, "detections");
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public FreshnessStep among(Set<Integer> candidateIds) {
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
         * {@inheritDoc}
         */
        @Override
        public PolicyStep freshWithinSec(double maxAgeSec) {
            if (!Double.isFinite(maxAgeSec) || maxAgeSec < 0.0) {
                throw new IllegalArgumentException("maxAgeSec must be finite and >= 0");
            }
            this.maxAgeSec = maxAgeSec;
            return this;
        }

        /** {@inheritDoc} */
        @Override
        public ModeStep choose(TagSelectionPolicy policy) {
            this.policy = Objects.requireNonNull(policy, "policy");
            return this;
        }

        /** {@inheritDoc} */
        @Override
        public BuildStep continuous() {
            this.mode = Mode.CONTINUOUS;
            this.enabled = null;
            this.reacquireAfterLossSec = Double.POSITIVE_INFINITY;
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public StickyWhenLossStep stickyWhen(BooleanSource enabled) {
            this.mode = Mode.STICKY_WHEN;
            this.enabled = Objects.requireNonNull(enabled, "enabled");
            this.reacquireAfterLossSec = Double.POSITIVE_INFINITY;
            return this;
        }

        /** {@inheritDoc} */
        @Override
        public StickyUntilResetLossStep stickyUntilReset() {
            this.mode = Mode.STICKY_UNTIL_RESET;
            this.enabled = null;
            this.reacquireAfterLossSec = Double.POSITIVE_INFINITY;
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public BuildStep holdUntilDisabled() {
            requireMode(Mode.STICKY_WHEN, "holdUntilDisabled()");
            this.reacquireAfterLossSec = Double.POSITIVE_INFINITY;
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public BuildStep holdUntilReset() {
            requireMode(Mode.STICKY_UNTIL_RESET, "holdUntilReset()");
            this.reacquireAfterLossSec = Double.POSITIVE_INFINITY;
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public BuildStep reacquireAfterLossSec(double reacquireAfterLossSec) {
            if (this.mode != Mode.STICKY_WHEN && this.mode != Mode.STICKY_UNTIL_RESET) {
                throw new IllegalStateException("reacquireAfterLossSec(...) is only valid after choosing a sticky mode");
            }
            if (!Double.isFinite(reacquireAfterLossSec) || reacquireAfterLossSec < 0.0) {
                throw new IllegalArgumentException("reacquireAfterLossSec must be finite and >= 0");
            }
            this.reacquireAfterLossSec = reacquireAfterLossSec;
            return this;
        }

        /** {@inheritDoc} */
        @Override
        public TagSelectionSource build() {
            if (candidateIds == null || candidateIds.isEmpty()) {
                throw new IllegalStateException("TagSelections requires among(...) before build()");
            }
            if (!Double.isFinite(maxAgeSec) || maxAgeSec < 0.0) {
                throw new IllegalStateException("TagSelections requires freshWithinSec(...) before build()");
            }
            if (policy == null) {
                throw new IllegalStateException("TagSelections requires choose(...) before build()");
            }
            if (mode == null) {
                throw new IllegalStateException("TagSelections requires continuous(), stickyWhen(...), or stickyUntilReset() before build()");
            }
            if (mode == Mode.STICKY_WHEN && enabled == null) {
                throw new IllegalStateException("stickyWhen(...) requires a BooleanSource");
            }
            return new BuiltSelectionSource(detections, candidateIds, maxAgeSec, policy, mode, enabled, reacquireAfterLossSec);
        }

        private void requireMode(Mode expected, String methodName) {
            if (mode != expected) {
                throw new IllegalStateException(methodName + " is only valid after "
                        + (expected == Mode.STICKY_WHEN ? "stickyWhen(...)" : "stickyUntilReset()"));
            }
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
                    .addData(p + ".reacquireAfterLossSec", reacquireAfterLossSec)
                    .addData(p + ".reason", lastReason)
                    .addData(p + ".metricValue", lastMetricValue);
            detections.debugDump(dbg, p + ".detections");
        }
    }
}
