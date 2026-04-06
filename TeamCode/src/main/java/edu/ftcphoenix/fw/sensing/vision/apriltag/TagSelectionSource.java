package edu.ftcphoenix.fw.sensing.vision.apriltag;

import java.util.Set;

import edu.ftcphoenix.fw.core.source.Source;

/**
 * Stateful, inspectable selected-tag source built on top of raw AprilTag detections.
 *
 * <p>A selector is the shared answer to questions like:</p>
 * <ul>
 *   <li>Which scoring tag would win right now?</li>
 *   <li>Which tag did the driver latch when aim mode was enabled?</li>
 *   <li>Is that selected tag still freshly visible?</li>
 * </ul>
 *
 * <p>Drive guidance, shooter gating, telemetry, and mechanism logic should all reuse the same
 * selector when they mean the same semantic target family. That avoids duplicate “best tag”
 * logic scattered across the robot.</p>
 */
public interface TagSelectionSource extends Source<TagSelectionResult> {

    /**
     * Returns the candidate IDs this selector may choose from.
     */
    Set<Integer> candidateIds();
}
