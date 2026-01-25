package edu.ftcphoenix.fw.sensing.vision.apriltag;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcphoenix.fw.ftc.FtcVision;

/**
 * High-level helpers for working with AprilTags in robot code.
 *
 * <p>This class plays a similar role to {@code FtcDrives} and the plant factories in
 * {@code Actuators}: it provides a small number of static factory methods that hide the
 * FTC SDK details and return framework-friendly interfaces.</p>
 *
 * <h2>Beginner usage</h2>
 *
 * <pre>{@code
 * public class MyTeleOp extends OpMode {
 *     private AprilTagSensor tags;
 *
 *     @Override
 *     public void init() {
 *         // Wire up AprilTags using a named webcam from the hardware map.
 *         tags = Tags.aprilTags(hardwareMap, "Webcam 1");
 *     }
 *
 *     @Override
 *     public void loop() {
 *         // Get the closest tag in this set, if the camera frame is no older than 0.3 sec.
 *         AprilTagObservation obs = tags.best(Set.of(1, 2, 3), 0.3);
 *         if (obs.hasTarget) {
 *             telemetry.addData("Tag ID", obs.id);
 *             telemetry.addData("Range (in)", obs.cameraRangeInches());
 *             telemetry.addData("Bearing (deg)", Math.toDegrees(obs.cameraBearingRad()));
 *             // Note: bearing > 0 means the tag is to the LEFT of the camera forward axis.
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
     * <p>The returned sensor reports observations in <b>Phoenix framing</b>:
     * +X forward, +Y left, +Z up. Bearing and range are derived from the observation's
     * {@code cameraToTagPose} pose.</p>
     *
     * @param hw         hardware map from the current OpMode
     * @param cameraName hardware configuration name of the webcam
     * @return a ready-to-use {@link AprilTagSensor}
     * @throws IllegalArgumentException if {@code hw} or {@code cameraName}
     *                                  are {@code null}, or if the named
     *                                  camera cannot be found
     */
    public static AprilTagSensor aprilTags(HardwareMap hw, String cameraName) {
        Objects.requireNonNull(hw, "hardwareMap is required");
        Objects.requireNonNull(cameraName, "cameraName is required");
        return FtcVision.aprilTags(hw, cameraName);
    }
}
