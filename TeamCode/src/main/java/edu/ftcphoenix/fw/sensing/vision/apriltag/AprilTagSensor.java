package edu.ftcphoenix.fw.sensing.vision.apriltag;

import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Loop-memoized source of raw AprilTag detections.
 *
 * <p>Phoenix intentionally keeps the vision boundary narrow: an {@code AprilTagSensor}
 * answers only one question per loop — <em>what detections did the camera produce from its
 * latest processed frame?</em> It does <strong>not</strong> hide selection policy such as
 * “best tag”, “closest tag”, or “most centered tag”. Those higher-level decisions belong in
 * {@link TagSelectionSource}, guidance, localization, or robot-specific policy code.</p>
 *
 * <h2>Design principles</h2>
 * <ul>
 *   <li><b>One loop, one heartbeat:</b> implementations must be idempotent by
 *       {@link LoopClock#cycle()} so multiple consumers can safely share one sensor instance.</li>
 *   <li><b>Keep raw data available:</b> callers can inspect all visible tags from one frame,
 *       which enables multi-tag localization and explicit selection policies.</li>
 *   <li><b>Selection is separate from sensing:</b> if robot code wants “the tag we are currently
 *       aiming at”, build a {@link TagSelectionSource} on top of this sensor.</li>
 * </ul>
 *
 * <h2>Typical usage</h2>
 * <pre>{@code
 * FtcWebcamAprilTagVisionLane.Config config =
 *         FtcWebcamAprilTagVisionLane.Config.defaults();
 * config.webcamName = "Webcam 1";
 * FtcWebcamAprilTagVisionLane vision =
 *         new FtcWebcamAprilTagVisionLane(hardwareMap, config);
 * AprilTagSensor tags = vision.tagSensor();
 *
 * // Shared loop sample (memoized inside the sensor).
 * AprilTagDetections dets = tags.get(clock);
 *
 * // Raw inspection.
 * telemetry.addData("visibleIds", dets.visibleIds(0.25).toString());
 *
 * // Explicit lookup by ID.
 * AprilTagObservation obs = dets.forId(5, 0.25);
 * if (obs.hasTarget) {
 *     telemetry.addData("bearingDeg", Math.toDegrees(obs.cameraBearingRad()));
 * }
 *
 * // At OpMode shutdown, close the owner—not a borrowed sensor view.
 * vision.close();
 * }</pre>
 */
public interface AprilTagSensor extends Source<AprilTagDetections> {

    /**
     * Returns the latest processed detections for the current loop.
     *
     * <p>The returned snapshot must never be {@code null}. When there are no usable detections,
     * implementations should return {@link AprilTagDetections#none()} (or a snapshot whose
     * observation list is empty).</p>
     *
     * @param clock current loop clock (required)
     * @return immutable AprilTag detections snapshot for this loop
     */
    @Override
    AprilTagDetections get(LoopClock clock);
}
