package edu.ftcphoenix.fw.sensing.vision.apriltag;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcphoenix.fw.ftc.FtcVision;

/**
 * High-level helpers for working with AprilTags in robot code.
 *
 * <p>The returned {@link AprilTagSensor} is a Phoenix {@code Source<AprilTagDetections>}:
 * each loop it yields the latest immutable snapshot of all AprilTag detections from one processed
 * camera frame. The sensor itself is responsible for cycle-idempotent memoization, so multiple
 * consumers (telemetry, selection, localization, guidance) may safely share one sensor instance.</p>
 *
 * <h2>Beginner usage</h2>
 *
 * <pre>{@code
 * public class MyTeleOp extends OpMode {
 *     private AprilTagSensor tags;
 *
 *     @Override
 *     public void init() {
 *         tags = Tags.aprilTags(hardwareMap, "Webcam 1");
 *     }
 *
 *     @Override
 *     public void loop() {
 *         LoopClock clock = ...;
 *         AprilTagDetections dets = tags.get(clock);
 *         for (AprilTagObservation obs : dets.freshObservations(0.3)) {
 *             telemetry.addData("Tag " + obs.id,
 *                     String.format("range=%.1f in bearing=%.1f deg",
 *                             obs.cameraRangeInches(),
 *                             Math.toDegrees(obs.cameraBearingRad())));
 *         }
 *     }
 * }
 * }</pre>
 */
public final class Tags {

    private Tags() {
        // Static factory only.
    }

    /**
     * Create an {@link AprilTagSensor} backed by the FTC SDK AprilTag pipeline, using a webcam.
     *
     * <p>The returned sensor reports observations in <b>Phoenix framing</b>: +X forward, +Y left,
     * +Z up. Bearing and range are derived from each observation's {@code cameraToTagPose} pose.</p>
     */
    public static AprilTagSensor aprilTags(HardwareMap hw, String cameraName) {
        Objects.requireNonNull(hw, "hardwareMap is required");
        Objects.requireNonNull(cameraName, "cameraName is required");
        return FtcVision.aprilTags(hw, cameraName);
    }
}
