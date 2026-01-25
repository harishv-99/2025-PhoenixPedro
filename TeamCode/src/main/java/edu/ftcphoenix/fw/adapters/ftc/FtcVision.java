package edu.ftcphoenix.fw.adapters.ftc;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Objects;
import java.util.Set;

import edu.ftcphoenix.fw.sensing.AprilTagObservation;
import edu.ftcphoenix.fw.sensing.AprilTagSensor;

/**
 * FTC-specific adapter that wires the SDK {@link VisionPortal} and
 * {@link AprilTagProcessor} into the framework's {@link AprilTagSensor}.
 *
 * <h2>Layering</h2>
 *
 * <ul>
 *   <li>Robot code should depend only on {@link AprilTagSensor} and
 *       {@link AprilTagObservation}.</li>
 *   <li>This class is the only place where Phoenix knows about FTC
 *       vision classes.</li>
 *   <li>Higher-level helpers (for example, a {@code Tags} utility in
 *       {@code fw.sensing}) can delegate to this adapter.</li>
 * </ul>
 *
 * <h2>Usage (beginner path)</h2>
 *
 * <pre>{@code
 * // In your robot container or TeleOp init:
 * AprilTagSensor tags = FtcVision.aprilTags(hardwareMap, "Webcam 1");
 *
 * // In your loop:
 * AprilTagObservation obs = tags.best(Set.of(1, 2, 3), 0.3); // tags 1/2/3, max age 0.3 s
 *
 * if (obs.hasTarget) {
 *     telemetry.addData("Tag", obs.id);
 *     telemetry.addData("Range (in)", obs.rangeInches);
 *     telemetry.addData("Bearing (deg)", Math.toDegrees(obs.bearingRad));
 * } else {
 *     telemetry.addLine("No tag in range");
 * }
 * }</pre>
 *
 * <p>Advanced teams who want to customize the VisionPortal / processor
 * can either fork this class or add another adapter that still returns
 * an {@link AprilTagSensor}.</p>
 */
public final class FtcVision {

    private static final double NANOS_PER_SECOND = 1_000_000_000.0;

    private FtcVision() {
        // utility holder
    }

    /**
     * Create an {@link AprilTagSensor} using a named FTC webcam and
     * a default configuration suitable for most teams.
     *
     * <p>Defaults:</p>
     *
     * <ul>
     *   <li>Camera: {@link WebcamName} looked up from {@link HardwareMap}.</li>
     *   <li>Tag library: {@link AprilTagGameDatabase#getCurrentGameTagLibrary()}.</li>
     *   <li>Output units: distance in inches, angles in radians
     *       (for {@link AprilTagObservation#rangeInches} and
     *       {@link AprilTagObservation#bearingRad}).</li>
     *   <li>Camera resolution: 640x480 (a common calibrated resolution).</li>
     * </ul>
     *
     * <p>The returned sensor owns the {@link VisionPortal} and keeps it
     * open for the lifetime of the OpMode. There is no explicit close
     * API in {@link AprilTagSensor}; the portal will be reclaimed when
     * the OpMode finishes.</p>
     *
     * @param hw         hardware map from the OpMode
     * @param cameraName hardware configuration name of the webcam
     * @return a ready-to-use {@link AprilTagSensor}
     */
    public static AprilTagSensor aprilTags(HardwareMap hw, String cameraName) {
        Objects.requireNonNull(hw, "hardwareMap is required");
        Objects.requireNonNull(cameraName, "cameraName is required");

        WebcamName webcam = hw.get(WebcamName.class, cameraName);

        // Configure the AprilTag processor: current-game library, inches + radians for pose.
        AprilTagProcessor.Builder tagBuilder = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS);

        AprilTagProcessor processor = tagBuilder.build();

        // Wire the processor into a VisionPortal using the webcam.
        VisionPortal.Builder portalBuilder = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(processor)
                .setCameraResolution(new Size(640, 480));

        VisionPortal portal = portalBuilder.build();

        return new PortalAprilTagSensor(portal, processor);
    }

    /**
     * Internal implementation of {@link AprilTagSensor} backed by a
     * {@link VisionPortal} and {@link AprilTagProcessor}.
     *
     * <p>This class is package-visible so tests or advanced helpers can
     * use it without exposing FTC vision types beyond the adapters
     * package.</p>
     */
    static final class PortalAprilTagSensor implements AprilTagSensor {

        @SuppressWarnings("unused")
        private final VisionPortal portal;   // kept for lifecycle; not directly used yet
        private final AprilTagProcessor processor;

        PortalAprilTagSensor(VisionPortal portal, AprilTagProcessor processor) {
            this.portal = Objects.requireNonNull(portal, "portal");
            this.processor = Objects.requireNonNull(processor, "processor");
        }

        @Override
        public AprilTagObservation bestAny(double maxAgeSec) {
            return selectBest(null, maxAgeSec);
        }

        @Override
        public AprilTagObservation best(Set<Integer> idsOfInterest, double maxAgeSec) {
            Objects.requireNonNull(idsOfInterest, "idsOfInterest");
            return selectBest(idsOfInterest, maxAgeSec);
        }

        /**
         * Core selection logic shared by {@link #bestAny(double)} and
         * {@link #best(Set, double)}.
         *
         * <p>Policy:</p>
         *
         * <ol>
         *   <li>Start from the latest detections from {@link AprilTagProcessor}.</li>
         *   <li>Optionally filter by {@code idsOrNull}.</li>
         *   <li>Ignore detections without metadata or pose.</li>
         *   <li>Reject detections older than {@code maxAgeSec} using
         *       {@link AprilTagDetection#frameAcquisitionNanoTime}.</li>
         *   <li>Among remaining detections, choose the one with the
         *       smallest range (closest tag).</li>
         * </ol>
         *
         * @param idsOrNull set of IDs to accept, or {@code null} for "any"
         * @param maxAgeSec maximum acceptable age (seconds)
         */
        private AprilTagObservation selectBest(Set<Integer> idsOrNull, double maxAgeSec) {
            if (maxAgeSec < 0.0) {
                // Negative max ages are not meaningful; treat as "no target acceptable".
                return AprilTagObservation.noTarget(Double.POSITIVE_INFINITY);
            }

            List<AprilTagDetection> detections = processor.getDetections();
            if (detections == null || detections.isEmpty()) {
                return AprilTagObservation.noTarget(Double.POSITIVE_INFINITY);
            }

            long nowNanos = System.nanoTime();

            AprilTagDetection bestDet = null;
            double bestRangeInches = Double.POSITIVE_INFINITY;
            double bestAgeSec = Double.POSITIVE_INFINITY;

            for (AprilTagDetection det : detections) {
                if (det == null) {
                    continue;
                }

                if (idsOrNull != null && !idsOrNull.contains(det.id)) {
                    // Caller is only interested in specific IDs.
                    continue;
                }

                // To use pose data (range/bearing) we must have metadata and ftcPose.
                if (det.metadata == null || det.ftcPose == null) {
                    continue;
                }

                // Compute age of this frame based on acquisition time.
                long frameTime = det.frameAcquisitionNanoTime;
                double ageSec = (frameTime == 0L)
                        ? 0.0
                        : (nowNanos - frameTime) / NANOS_PER_SECOND;

                if (ageSec > maxAgeSec) {
                    // Too old for caller's freshness requirement.
                    continue;
                }

                double rangeInches = det.ftcPose.range; // already configured as inches
                if (rangeInches < bestRangeInches) {
                    bestRangeInches = rangeInches;
                    bestDet = det;
                    bestAgeSec = ageSec;
                }
            }

            if (bestDet == null) {
                // Either no tags matched IDs, or none were fresh enough.
                return AprilTagObservation.noTarget(Double.POSITIVE_INFINITY);
            }

            double bearingRad = bestDet.ftcPose.bearing; // radians via setOutputUnits

            return AprilTagObservation.target(
                    bestDet.id,
                    bearingRad,
                    bestRangeInches,
                    bestAgeSec
            );
        }
    }
}
