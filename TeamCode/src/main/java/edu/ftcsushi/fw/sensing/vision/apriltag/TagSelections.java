package edu.ftcsushi.fw.sensing.vision.apriltag;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Objects;
import java.util.Set;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.source.TimeAwareSource;
import edu.ftcsushi.fw.core.source.TimeAwareSources;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.field.TagLayout;
import edu.ftcsushi.fw.field.TagLayouts;
import edu.ftcsushi.fw.localization.AbsolutePoseEstimator;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;

/**
 * Choose one tag identity from either actual visible tags or known field tags ranked using the
 * authoritative robot pose. The two sources share policies and one sticky-selection engine;
 * neither silently falls back to the other. Candidate geometry uses the one supplied camera mount.
 *
 * <p>All live inputs are borrowed: construction never samples them, selection never updates an
 * estimator, and reset never resets sensors, enable signals, estimators, or mount history. A
 * fixed layout is snapshotted when built. Historical mounts are requested only at the accepted
 * evidence timestamp; a null/throwing lookup fails the read, without current-mount substitution.</p>
 *
 * <p>One successful snapshot commits per clock cycle. Failed reads may retry without partial
 * latch changes. Reentrant reads/reset are errors. Explicit reset and clock-epoch changes clear
 * local selection state. A sticky enable is a sampled lifetime: its owner must make release
 * observable or explicitly reset this selector before starting a new attempt.</p>
 */
public final class TagSelections {
    private TagSelections() { }

    /** Begin selection from actual detections interpreted with a fixed camera mount. */
    public static VisibleCandidateStep fromVisibleTags(Source<AprilTagDetections> detections,
                                                        CameraMountConfig mount) {
        return fromVisibleTags(detections, TimeAwareSources.fixed(Objects.requireNonNull(mount, "mount")));
    }

    /** Advanced borrowed history: lookup uses the camera frame's original capture timestamp. */
    public static VisibleCandidateStep fromVisibleTags(Source<AprilTagDetections> detections,
                                                        TimeAwareSource<CameraMountConfig> mount) {
        Builder builder = new Builder(Objects.requireNonNull(detections, "detections"), null, null, mount);
        return ids -> {
            builder.among(ids);
            return age -> { builder.freshWithinSec(age); return builder; };
        };
    }

    /** Begin selection among fixed field tags using a borrowed authoritative pose and fixed mount. */
    public static FieldPoseCandidateStep fromFieldPose(AbsolutePoseEstimator pose, TagLayout layout,
                                                       CameraMountConfig mount) {
        return fromFieldPose(pose, layout, TimeAwareSources.fixed(Objects.requireNonNull(mount, "mount")));
    }

    /** Advanced borrowed history: mount lookup uses the accepted pose evidence timestamp. */
    public static FieldPoseCandidateStep fromFieldPose(AbsolutePoseEstimator pose, TagLayout layout,
                                                       TimeAwareSource<CameraMountConfig> mount) {
        Builder builder = new Builder(null, Objects.requireNonNull(pose, "pose"),
                Objects.requireNonNull(layout, "layout"), mount);
        return ids -> {
            builder.among(ids);
            return age -> {
                builder.freshWithinSec(age);
                return quality -> { builder.minQuality(quality); return builder; };
            };
        };
    }

    /** Choose the eligible observed tag IDs. */
    public interface VisibleCandidateStep {
        /** Snapshot a nonempty set of non-negative IDs. */
        VisibleFreshnessStep among(Set<Integer> candidateIds);
    }

    /** Choose the inclusive maximum age of an actual frame. */
    public interface VisibleFreshnessStep {
        /** Require finite non-negative age in seconds. */
        PolicyStep freshWithinSec(double maxAgeSec);
    }

    /** Choose eligible fixed tag IDs; every ID must occur in the retained field layout. */
    public interface FieldPoseCandidateStep {
        /** Snapshot a nonempty set of non-negative IDs. */
        FieldPoseFreshnessStep among(Set<Integer> candidateIds);
    }

    /** Choose the inclusive maximum age of the robot-pose evidence. */
    public interface FieldPoseFreshnessStep {
        /** Require finite non-negative age in seconds. */
        FieldPoseQualityStep freshWithinSec(double maxAgeSec);
    }

    /** Pose selection requires an explicit producer-quality gate, not a visibility claim. */
    public interface FieldPoseQualityStep {
        /** Require a finite score in [0,1]; even zero rejects unknown or invalid runtime quality. */
        PolicyStep minQuality(double minQuality);
    }

    /** Choose the same stateless ranking policy for either evidence source. */
    public interface PolicyStep {
        /** A custom policy must return an exact candidate from its supplied immutable list. */
        ModeStep choose(TagSelectionPolicy policy);
    }

    /** Choose continuous identity or a bounded application-owned commitment lifetime. */
    public interface ModeStep {
        /** Select the current preview every cycle. */
        BuildStep continuous();
        /** Hold a selection while this borrowed enable is true; false releases it. */
        StickyWhenLossStep stickyWhen(BooleanSource enabled);
        /** Hold a selection until explicit reset or a clock epoch change. */
        StickyUntilResetLossStep stickyUntilReset();
    }

    /** Explicit loss policy for an enabled attempt. */
    public interface StickyWhenLossStep {
        /** Retain identity without current geometry until disabled. */
        BuildStep holdUntilDisabled();
        /** Allow another choice after an inclusive finite, non-negative loss duration in seconds. */
        BuildStep reacquireAfterLossSec(double seconds);
    }

    /** Explicit loss policy for a reset-owned commitment. */
    public interface StickyUntilResetLossStep {
        /** Retain identity without current geometry until local reset or clock epoch change. */
        BuildStep holdUntilReset();
        /** Allow another choice after an inclusive finite, non-negative loss duration in seconds. */
        BuildStep reacquireAfterLossSec(double seconds);
    }

    /** All required questions answered. */
    public interface BuildStep {
        /** Build an independent local selector; live dependencies remain borrowed. */
        TagSelectionSource build();
    }

    private enum Mode { CONTINUOUS, STICKY_WHEN, STICKY_UNTIL_RESET }

    /** Shared configuration and sticky stages; source-specific required stages remain distinct. */
    private static final class Builder implements PolicyStep, ModeStep, StickyWhenLossStep,
            StickyUntilResetLossStep, BuildStep {
        private final Source<AprilTagDetections> detections;
        private final AbsolutePoseEstimator pose;
        private final TagLayout layout;
        private final TimeAwareSource<CameraMountConfig> mount;
        private Set<Integer> ids;
        private double maxAgeSec = Double.NaN;
        private double minQuality = Double.NaN;
        private TagSelectionPolicy policy;
        private Mode mode;
        private BooleanSource enabled;
        private double reacquireSec = Double.POSITIVE_INFINITY;
        private boolean lossChosen;

        Builder(Source<AprilTagDetections> detections, AbsolutePoseEstimator pose, TagLayout layout,
                TimeAwareSource<CameraMountConfig> mount) {
            this.detections = detections;
            this.pose = pose;
            this.layout = layout;
            this.mount = Objects.requireNonNull(mount, "mount");
        }

        private void among(Set<Integer> candidates) {
            Objects.requireNonNull(candidates, "candidateIds");
            if (candidates.isEmpty()) throw new IllegalArgumentException("candidateIds must not be empty");
            LinkedHashSet<Integer> copy = new LinkedHashSet<>();
            for (Integer id : candidates) {
                if (id == null || id < 0) {
                    throw new IllegalArgumentException("candidateIds must contain non-negative IDs");
                }
                copy.add(id);
            }
            ids = Collections.unmodifiableSet(copy);
        }

        private void freshWithinSec(double age) {
            requireDuration(age, "maxAgeSec");
            maxAgeSec = age;
        }

        private void minQuality(double quality) {
            if (!Double.isFinite(quality) || quality < 0 || quality > 1) {
                throw new IllegalArgumentException("minQuality must be finite and in [0,1]");
            }
            minQuality = quality;
        }

        @Override public ModeStep choose(TagSelectionPolicy policy) {
            this.policy = Objects.requireNonNull(policy, "policy");
            return this;
        }

        @Override public BuildStep continuous() {
            mode = Mode.CONTINUOUS;
            enabled = null;
            lossChosen = true;
            reacquireSec = Double.POSITIVE_INFINITY;
            return this;
        }

        @Override public StickyWhenLossStep stickyWhen(BooleanSource enabled) {
            mode = Mode.STICKY_WHEN;
            this.enabled = Objects.requireNonNull(enabled, "enabled");
            lossChosen = false;
            return this;
        }

        @Override public StickyUntilResetLossStep stickyUntilReset() {
            mode = Mode.STICKY_UNTIL_RESET;
            enabled = null;
            lossChosen = false;
            return this;
        }

        @Override public BuildStep holdUntilDisabled() {
            if (mode != Mode.STICKY_WHEN) throw new IllegalStateException("choose stickyWhen(...) first");
            return hold();
        }

        @Override public BuildStep holdUntilReset() {
            if (mode != Mode.STICKY_UNTIL_RESET) throw new IllegalStateException("choose stickyUntilReset() first");
            return hold();
        }

        private BuildStep hold() {
            lossChosen = true;
            reacquireSec = Double.POSITIVE_INFINITY;
            return this;
        }

        @Override public BuildStep reacquireAfterLossSec(double seconds) {
            if (mode != Mode.STICKY_WHEN && mode != Mode.STICKY_UNTIL_RESET) {
                throw new IllegalStateException("choose a sticky mode before reacquireAfterLossSec(...)");
            }
            requireDuration(seconds, "reacquireAfterLossSec");
            reacquireSec = seconds;
            lossChosen = true;
            return this;
        }

        @Override public TagSelectionSource build() {
            if (ids == null || !Double.isFinite(maxAgeSec) || policy == null || mode == null || !lossChosen
                    || (pose != null && !Double.isFinite(minQuality))) {
                throw new IllegalStateException("answer candidate IDs, freshness, pose quality, policy, and mode before build()");
            }
            TagLayout fixedLayout = layout == null ? null : TagLayouts.snapshot(layout);
            if (fixedLayout != null) {
                for (Integer id : ids) fixedLayout.requireFieldToTagPose(id);
            }
            return new BuiltSelectionSource(this, fixedLayout);
        }
    }

    private static void requireDuration(double seconds, String name) {
        if (!Double.isFinite(seconds) || seconds < 0) {
            throw new IllegalArgumentException(name + " must be finite and >= 0");
        }
    }

    /** Owns only local cache, latch, and loss timing. */
    private static final class BuiltSelectionSource implements TagSelectionSource {
        private final Source<AprilTagDetections> detections;
        private final AbsolutePoseEstimator pose;
        private final TagLayout layout;
        private final TimeAwareSource<CameraMountConfig> mount;
        private final Set<Integer> ids;
        private final double maxAgeSec;
        private final double minQuality;
        private final TagSelectionPolicy policy;
        private final Mode mode;
        private final BooleanSource enabled;
        private final double reacquireSec;
        private long lastCycle = Long.MIN_VALUE;
        private LoopTimestamp lastSampleTimestamp = LoopTimestamp.unavailable();
        private TagSelectionResult last = TagSelectionResult.none();
        private State state = new State();
        private boolean operationInProgress;

        BuiltSelectionSource(Builder builder, TagLayout layout) {
            detections = builder.detections;
            pose = builder.pose;
            this.layout = layout;
            mount = builder.mount;
            ids = builder.ids;
            maxAgeSec = builder.maxAgeSec;
            minQuality = builder.minQuality;
            policy = builder.policy;
            mode = builder.mode;
            enabled = builder.enabled;
            reacquireSec = builder.reacquireSec;
        }

        @Override public Set<Integer> candidateIds() { return ids; }

        @Override public TagSelectionResult get(LoopClock clock) {
            if (operationInProgress) throw reentrantLifecycle("sample");
            Objects.requireNonNull(clock, "clock");
            if (lastSampleTimestamp.isAvailable() && !Double.isFinite(lastSampleTimestamp.ageSec(clock))) {
                clearLocal();
            }
            if (lastCycle == clock.cycle()) return last;
            operationInProgress = true;
            try {
                long cycle = clock.cycle();
                LoopTimestamp sampledAt = clock.nowTimestamp();
                State pending = new State(state);
                Evidence evidence = detections == null ? fieldEvidence(clock) : observedEvidence(clock);
                TagSelectionChoice preview = policy.choose(evidence.candidates);
                if (preview != null && !containsIdentity(evidence.candidates, preview.candidate)) {
                    throw new IllegalArgumentException("TagSelectionPolicy must choose an exact candidate supplied to this invocation");
                }
                if (mode == Mode.CONTINUOUS) {
                    pending.decision = preview;
                    pending.lostSinceSec = Double.NaN;
                } else {
                    boolean enabledNow = mode != Mode.STICKY_WHEN || enabled.getAsBoolean(clock);
                    if (!enabledNow) {
                        pending.decision = null;
                        pending.lostSinceSec = Double.NaN;
                    } else {
                        if (!pending.prevEnabled || pending.decision == null) pending.decision = preview;
                        stepLoss(pending, evidence.candidates, preview, clock.nowSec());
                    }
                    pending.prevEnabled = enabledNow;
                }
                int selectedId = pending.decision == null ? -1 : pending.decision.candidate.tagId;
                TagSelectionResult result = new TagSelectionResult(preview, selectedId,
                        mode != Mode.CONTINUOUS && selectedId >= 0, pending.decision,
                        findById(evidence.candidates, selectedId), evidence.visibleIds, evidence.visibilityTime);
                // A callback must not move the shared heartbeat while this transaction is in flight.
                if (clock.cycle() != cycle || !Double.isFinite(sampledAt.ageSec(clock))) {
                    throw new IllegalStateException("TagSelectionSource clock changed during sampling");
                }
                state = pending;
                last = result;
                lastSampleTimestamp = sampledAt;
                lastCycle = cycle;
                return result;
            } finally {
                operationInProgress = false;
            }
        }

        private void stepLoss(State pending, List<TagSelectionCandidate> candidates,
                              TagSelectionChoice preview, double nowSec) {
            if (pending.decision == null) return;
            if (findById(candidates, pending.decision.candidate.tagId) != null) {
                pending.lostSinceSec = Double.NaN;
                return;
            }
            if (Double.isNaN(pending.lostSinceSec)) pending.lostSinceSec = nowSec;
            if (Double.isFinite(reacquireSec) && nowSec - pending.lostSinceSec >= reacquireSec) {
                pending.decision = preview;
                pending.lostSinceSec = Double.NaN;
            }
        }

        private Evidence observedEvidence(LoopClock clock) {
            AprilTagDetections frame = Objects.requireNonNull(detections.get(clock), "detections returned null");
            if (!frame.isFresh(clock, maxAgeSec)) return Evidence.empty();
            ArrayList<TagSelectionCandidate> candidates = new ArrayList<>();
            LinkedHashSet<Integer> visible = new LinkedHashSet<>();
            CameraMountConfig frameMount = null;
            for (AprilTagObservation observation : frame.observations) {
                if (!ids.contains(observation.id) || !visible.add(observation.id)) continue;
                if (!finitePose(observation.cameraToTagPose) || !finiteRange(observation.cameraToTagPose)) continue;
                if (frameMount == null) frameMount = requiredMount(clock, frame.frameTimestamp());
                Pose3d robotToTag = frameMount.robotToCameraPose().then(observation.cameraToTagPose);
                if (!finitePose(robotToTag)) continue;
                candidates.add(new TagSelectionCandidate(observation.id, observation.cameraToTagPose,
                        robotToTag, TagSelectionCandidate.EvidenceKind.OBSERVED,
                        frame.frameTimestamp(), observation));
            }
            return new Evidence(candidates, visible, frame.frameTimestamp());
        }

        private Evidence fieldEvidence(LoopClock clock) {
            PoseEstimate estimate = pose.getEstimate();
            if (estimate == null || !estimate.hasPose || !finitePose(estimate.fieldToRobotPose)
                    || !estimate.timestamp.isFresh(clock, maxAgeSec)
                    || !Double.isFinite(estimate.quality) || estimate.quality < minQuality
                    || estimate.quality > 1) return Evidence.empty();
            CameraMountConfig frameMount = requiredMount(clock, estimate.timestamp);
            Pose3d fieldToCamera = estimate.fieldToRobotPose.then(frameMount.robotToCameraPose());
            if (!finitePose(fieldToCamera)) return Evidence.empty();
            Pose3d cameraToField = fieldToCamera.inverse();
            Pose3d robotToField = estimate.fieldToRobotPose.inverse();
            ArrayList<TagSelectionCandidate> candidates = new ArrayList<>();
            for (Integer id : ids) {
                Pose3d fieldToTag = layout.requireFieldToTagPose(id);
                Pose3d cameraToTag = cameraToField.then(fieldToTag);
                Pose3d robotToTag = robotToField.then(fieldToTag);
                if (!finitePose(cameraToTag) || !finiteRange(cameraToTag) || !finitePose(robotToTag)) continue;
                candidates.add(new TagSelectionCandidate(id, cameraToTag, robotToTag,
                        TagSelectionCandidate.EvidenceKind.FIELD_POSE, estimate.timestamp, null));
            }
            return new Evidence(candidates, Collections.emptySet(), LoopTimestamp.unavailable());
        }

        private CameraMountConfig requiredMount(LoopClock clock, LoopTimestamp timestamp) {
            return Objects.requireNonNull(mount.getAt(clock, timestamp),
                    "camera mount history must return a mount at the evidence timestamp; do not substitute current mount");
        }

        @Override public void reset() {
            if (operationInProgress) throw reentrantLifecycle("reset");
            clearLocal();
        }

        private void clearLocal() {
            state = new State();
            last = TagSelectionResult.none();
            lastCycle = Long.MIN_VALUE;
            lastSampleTimestamp = LoopTimestamp.unavailable();
        }

        @Override public void debugDump(DebugSink dbg, String prefix) {
            if (dbg == null) return;
            String p = prefix == null || prefix.isEmpty() ? "tagSelection" : prefix;
            dbg.addData(p + ".candidateIds", ids.toString())
                    .addData(p + ".maxAgeSec", maxAgeSec)
                    .addData(p + ".mode", mode.name())
                    .addData(p + ".selectedTagId", last.selectedTagId)
                    .addData(p + ".latched", last.latched)
                    .addData(p + ".reacquireAfterLossSec", reacquireSec)
                    .addData(p + ".reason", state.decision == null ? "no selection" : state.decision.reason)
                    .addData(p + ".metricValue", state.decision == null ? Double.NaN : state.decision.metricValue);
        }
    }

    private static boolean finitePose(Pose3d pose) {
        return pose != null && Double.isFinite(pose.xInches) && Double.isFinite(pose.yInches)
                && Double.isFinite(pose.zInches) && Double.isFinite(pose.yawRad)
                && Double.isFinite(pose.pitchRad) && Double.isFinite(pose.rollRad);
    }

    /** A finite coordinate triple can still have an unrepresentable three-dimensional norm. */
    private static boolean finiteRange(Pose3d pose) {
        return Double.isFinite(Math.hypot(Math.hypot(pose.xInches, pose.yInches), pose.zInches));
    }

    private static boolean containsIdentity(List<TagSelectionCandidate> candidates, TagSelectionCandidate chosen) {
        for (TagSelectionCandidate candidate : candidates) if (candidate == chosen) return true;
        return false;
    }

    private static TagSelectionCandidate findById(List<TagSelectionCandidate> candidates, int id) {
        for (TagSelectionCandidate candidate : candidates) if (candidate.tagId == id) return candidate;
        return null;
    }

    private static IllegalStateException reentrantLifecycle(String operation) {
        return new IllegalStateException("TagSelectionSource cannot " + operation
                + " reentrantly while another sample or reset is in progress");
    }

    /** Unpublished transactional selection state. */
    private static final class State {
        boolean prevEnabled;
        TagSelectionChoice decision;
        double lostSinceSec = Double.NaN;
        State() { }
        State(State prior) {
            prevEnabled = prior.prevEnabled;
            decision = prior.decision;
            lostSinceSec = prior.lostSinceSec;
        }
    }

    /** One source-normalized snapshot; field geometry deliberately has no visibility evidence. */
    private static final class Evidence {
        final List<TagSelectionCandidate> candidates;
        final Set<Integer> visibleIds;
        final LoopTimestamp visibilityTime;
        Evidence(List<TagSelectionCandidate> candidates, Set<Integer> visibleIds, LoopTimestamp visibilityTime) {
            this.candidates = Collections.unmodifiableList(candidates);
            this.visibleIds = visibleIds;
            this.visibilityTime = visibilityTime;
        }
        static Evidence empty() {
            return new Evidence(Collections.emptyList(), Collections.emptySet(), LoopTimestamp.unavailable());
        }
    }
}
