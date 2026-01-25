package edu.ftcphoenix.fw.sensing;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcphoenix.fw.adapters.ftc.FtcVision;

/**
 * High-level helpers for working with AprilTags in robot code.
 *
 * <p>This class plays a similar role to {@code Drives} and {@code FtcPlants}:
 * it provides a small number of static factory methods that hide the FTC SDK
 * details and return framework-friendly interfaces.</p>
 *
 * <h2>Beginner usage</h2>
 *
 * <p>The typical pattern in a TeleOp or autonomous OpMode is:</p>
 *
 * <pre>{@code
 * public final class TeleOpWithTags extends PhoenixTeleOpBase {
 *     private AprilTagSensor tags;
 *
 *     @Override
 *     protected void onInitRobot() {
 *         // Wire up AprilTags using a named webcam from the hardware map.
 *         tags = Tags.aprilTags(hardwareMap, "Webcam 1");
 *     }
 *
 *     @Override
 *     protected void loopRobot(double dtSec) {
 *         AprilTagObservation obs = tags.best(Set.of(1, 2, 3), 0.3);
 *         if (obs.hasTarget) {
 *             telemetry.addData("Tag ID", obs.id);
 *             telemetry.addData("Range (in)", obs.rangeInches);
 *             telemetry.addData("Bearing (deg)", Math.toDegrees(obs.bearingRad));
 *         } else {
 *             telemetry.addLine("No tag in view");
 *         }
 *     }
 * }
 * }</pre>
 *
 * <p>Robot code never needs to interact directly with {@code VisionPortal} or
 * {@code AprilTagProcessor}. All FTC-specific vision details are handled by
 * the {@link FtcVision} adapter.</p>
 *
 * <h2>Design notes</h2>
 *
 * <ul>
 *   <li>This class is intentionally small and opinionated for the common case:
 *       one webcam, current-game tag library, inches for distance, radians
 *       for angles.</li>
 *   <li>More advanced teams who want custom VisionPortal options (resolution,
 *       multiple processors, etc.) can either:
 *     <ul>
 *       <li>Call {@link FtcVision#aprilTags(HardwareMap, String)} directly
 *           from their robot code, or</li>
 *       <li>Fork or extend {@link FtcVision} while still returning an
 *           {@link AprilTagSensor}.</li>
 *     </ul>
 *   </li>
 *   <li>Keeping this helper small makes it easy for new students to discover
 *       and understand: "Tags.aprilTags(...) gives me a sensor I can query
 *       for ID, distance, and bearing."</li>
 * </ul>
 */
public final class Tags {

    private Tags() {
        // utility holder; not instantiable
    }

    /**
     * Create an {@link AprilTagSensor} using a named FTC webcam and a default
     * configuration suitable for most robots.
     *
     * <p>This is the recommended entry point for teams that simply want to
     * read AprilTag ID, distance, and bearing from a single camera.</p>
     *
     * <p>Internally this delegates to
     * {@link FtcVision#aprilTags(HardwareMap, String)} and returns the
     * resulting {@link AprilTagSensor}.</p>
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
