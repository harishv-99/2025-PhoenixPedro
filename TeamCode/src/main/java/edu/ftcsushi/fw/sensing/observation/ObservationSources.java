package edu.ftcsushi.fw.sensing.observation;

import java.util.Objects;
import java.util.ArrayList;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.source.TimeAwareSource;
import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.localization.PlanarPoseHistory;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.sensing.vision.CameraMountLogic;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcsushi.fw.sensing.vision.apriltag.TagSelectionResult;
import edu.ftcsushi.fw.sensing.vision.apriltag.TagSelectionCandidate;
import edu.ftcsushi.fw.sensing.vision.apriltag.TagSelectionSource;

/**
 * Borrowed read projections using the one {@link Source#get(LoopClock)} observation grammar.
 * Reset never resets supplied sensors, selectors, or pose history.
 */
public final class ObservationSources {

    private ObservationSources() {
        // Utility class.
    }

    /**
     * Project current actual observed geometry from a shared {@link TagSelectionSource}.
     * The selector already applied its one camera mount; there is no second mount answer.
     *
     * <p>The selector decides which tag is semantically relevant; this helper simply converts the
     * selector's <em>fresh selected observation</em> into a planar robot-frame observation.
     * When no fresh selected observation exists, the source returns {@link TargetObservation2d#none()}.
     * The selected observation's exact camera-frame timestamp is forwarded; this source does not
     * create a new timestamp when a retained selection is sampled.</p>
     */
    public static Source<TargetObservation2d> aprilTag(TagSelectionSource selection) {
        Objects.requireNonNull(selection, "selection must not be null");

        return new Source<TargetObservation2d>() {
            /**
             * {@inheritDoc}
             */
            @Override
            public TargetObservation2d get(LoopClock clock) {
                TagSelectionResult sel = selection.get(clock);
                if (!sel.hasFreshSelectedObservation) {
                    return TargetObservation2d.none();
                }
                TagSelectionCandidate candidate = sel.currentSelectedCandidate;
                if (!Double.isFinite(candidate.evidenceTimestamp.ageSec(clock))) {
                    return TargetObservation2d.none();
                }
                Pose3d robotToTag = candidate.robotToTagPose;
                return TargetObservation2d.ofRobotRelativePose(candidate.tagId,
                        robotToTag.xInches, robotToTag.yInches, robotToTag.yawRad,
                        Double.NaN, candidate.evidenceTimestamp);
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) {
                    return;
                }
                String p = (prefix == null || prefix.isEmpty()) ? "obs2d" : prefix;
                dbg.addData(p + ".candidateIds", selection.candidateIds().toString());
                dbg.addData(p + ".class", getClass().getSimpleName());
                dbg.addData(p + ".type", "apriltagSelection");
            }
        };
    }

    /**
     * Converts every usable tag in one frame to robot-at-capture geometry. IDs and target heading
     * remain supported tag facts; confidence remains unknown because the raw pose adapter does
     * not supply a calibrated confidence score. Retains empty frames and the exact capture time.
     */
    public static Source<TargetObservations2d> aprilTags(Source<AprilTagDetections> detections,
                                                       CameraMountConfig mount) {
        Objects.requireNonNull(detections, "detections");
        Objects.requireNonNull(mount, "mount");
        return Source.of(clock -> {
            AprilTagDetections frame = Objects.requireNonNull(detections.get(clock), "tag frame");
            if (!frame.frameTimestamp().isAvailable()
                    || !Double.isFinite(frame.frameTimestamp().ageSec(clock))) {
                return TargetObservations2d.unavailable("tag frame timestamp unavailable or reset");
            }
            if (frame.observations.size() > TargetObservations2d.MAX_OBSERVATIONS) {
                return TargetObservations2d.unavailable("tag frame exceeds observation bound");
            }
            ArrayList<TargetObservation2d> converted = new ArrayList<>();
            for (AprilTagObservation observation : frame.observations) {
                if (!observation.hasTarget || observation.id < 0) continue;
                Pose3d pose = CameraMountLogic.robotToTagPose(mount, observation.cameraToTagPose);
                if (!Double.isFinite(pose.xInches) || !Double.isFinite(pose.yInches)
                        || !Double.isFinite(pose.yawRad)) continue;
                converted.add(TargetObservation2d.ofRobotRelativePose(observation.id,
                        pose.xInches, pose.yInches, pose.yawRad, Double.NaN, frame.frameTimestamp()));
            }
            return TargetObservations2d.fromFrame(frame.frameTimestamp(), converted);
        });
    }

    /**
     * Adds field coordinates using only the supplied history lookup at the original frame time.
     * The history remains borrowed; this source never advances or resets localization. An
     * unavailable lookup stays attached to each observation while its robot geometry is retained.
     *
     * <p>No frame-identity cache is used: every read asks history again, so reset/eviction cannot
     * hide behind a repeated camera image. There is no fallback to current pose. History lookup
     * failures propagate without publishing a partial frame; a later read can retry.</p>
     */
    public static Source<TargetObservations2d> inField(Source<TargetObservations2d> observations,
            TimeAwareSource<PlanarPoseHistory.Lookup> fieldPoseHistory) {
        Objects.requireNonNull(observations, "observations");
        Objects.requireNonNull(fieldPoseHistory, "fieldPoseHistory");
        return Source.of(clock -> {
            TargetObservations2d frame = Objects.requireNonNull(observations.get(clock), "frame");
            if (!frame.isAvailable()) return frame;
            PlanarPoseHistory.Lookup lookup = Objects.requireNonNull(
                    fieldPoseHistory.getAt(clock, frame.timestamp()), "field pose lookup");
            if (lookup.timestamp() != frame.timestamp()) {
                throw new IllegalArgumentException("field pose lookup must retain the requested frame timestamp");
            }
            ArrayList<TargetObservation2d> converted = new ArrayList<>(frame.observations().size());
            for (TargetObservation2d observation : frame.observations()) {
                converted.add(observation.withFieldPoseLookup(lookup));
            }
            return TargetObservations2d.fromFrame(frame.timestamp(), converted);
        });
    }
}
