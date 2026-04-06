package edu.ftcphoenix.fw.ftc;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseRaw;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.geometry.Mat3;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;

/**
 * FTC-specific vision adapter for AprilTag sensing.
 *
 * <p>This class wires up a {@link VisionPortal} and {@link AprilTagProcessor}
 * (defaulting to the current game's tag library), and exposes a framework-level
 * {@link AprilTagSensor} API so robot code can read tag observations without
 * depending on FTC vision classes.</p>
 *
 * <h2>Frames &amp; conversions</h2>
 *
 * <p>FTC AprilTag detections expose two pose representations:</p>
 * <ul>
 *   <li><b>{@code rawPose}</b>: the native AprilTag/OpenCV camera axes (+X right, +Y down, +Z forward)</li>
 *   <li><b>{@code ftcPose}</b>: an FTC convenience frame (+X right, +Y forward, +Z up) intended for
 *       human-friendly telemetry</li>
 * </ul>
 *
 * <p>Phoenix uses {@code rawPose} for geometry/math (localization, mount calibration) because it
 * matches the FTC game database tag metadata. This adapter converts that camera frame into the
 * Phoenix camera frame (+X forward, +Y left, +Z up) before constructing {@link AprilTagObservation}.</p>
 *
 * <h2>Range / bearing</h2>
 *
 * <p>Phoenix does not store redundant scalar bearing/range in {@link AprilTagObservation}.
 * Instead, robot code uses derived helpers:</p>
 *
 * <ul>
 *   <li>{@link AprilTagObservation#cameraBearingRad()}</li>
 *   <li>{@link AprilTagObservation#cameraRangeInches()}</li>
 * </ul>
 *
 * <h2>Camera mount / SDK robotPose</h2>
 *
 * <p>If you provide {@link Config#cameraMount}, this adapter applies the camera extrinsics to the FTC
 * {@link AprilTagProcessor.Builder} via {@link AprilTagProcessor.Builder#setCameraPose(Position, YawPitchRollAngles)}.
 * This enables FTC to compute per-detection robot pose when the SDK has sufficient metadata.</p>
 *
 * <p><b>Important:</b> FTC uses a separate camera-axes convention for {@code setCameraPose} (see
 * {@link FtcFrames} “Localization camera axes”). This adapter converts the Phoenix camera mount pose
 * into that convention before passing it to the SDK.</p>
 */
public final class FtcVision {

    /**
     * Default resource name used by the FTC Robot Controller app for the camera
     * monitor container.
     *
     * <p>The FTC SDK VisionPortal builder can target a specific live-view container
     * via {@code setLiveViewContainerId(int)}. Using this is required when multiple
     * VisionPortals exist over the lifetime of an OpMode (for example, when a
     * tester creates a portal, exits, then another tester creates a new one).</p>
     */
    private static final String DEFAULT_CAMERA_MONITOR_VIEW_ID_NAME = "cameraMonitorViewId";

    /**
     * Nano-seconds per second, used when converting FTC's
     * {@link AprilTagDetection#frameAcquisitionNanoTime} into seconds.
     */
    private static final double NANOS_PER_SECOND = 1_000_000_000.0;

    /**
     * Default camera resolution if none is provided.
     */
    private static final Size DEFAULT_RESOLUTION = new Size(640, 480);

    // -----------------------------------------------------------------------------------------
    // Live-view helpers
    // -----------------------------------------------------------------------------------------

    /**
     * Resolve the FTC Robot Controller's camera monitor container view id.
     *
     * <p>FTC samples commonly obtain this value via:
     * {@code hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", ...)}.
     * We wrap it here so vision adapters can behave correctly across OpModes and tester flows.</p>
     */
    private static int resolveDefaultLiveViewContainerId(HardwareMap hw) {
        if (hw == null || hw.appContext == null) {
            return 0;
        }
        try {
            // Primary: use whatever package the RC is running under.
            int id = hw.appContext.getResources().getIdentifier(
                    DEFAULT_CAMERA_MONITOR_VIEW_ID_NAME,
                    "id",
                    hw.appContext.getPackageName()
            );

            if (id != 0) {
                return id;
            }

            // Fallback: some builds/tooling can end up with an appContext whose package name
            // is not the canonical RC package. Try the common RC package names explicitly.
            String[] pkgs = new String[]{
                    "com.qualcomm.ftcrobotcontroller",
                    "org.firstinspires.ftc.robotcontroller"
            };

            for (String p : pkgs) {
                id = hw.appContext.getResources().getIdentifier(
                        DEFAULT_CAMERA_MONITOR_VIEW_ID_NAME,
                        "id",
                        p
                );
                if (id != 0) {
                    return id;
                }
            }

            return 0;
        } catch (Exception ignored) {
            return 0;
        }
    }

    /**
     * Apply a default live-view container id to a {@link VisionPortal.Builder}.
     *
     * <p>Why: the FTC SDK throws a runtime error if multiple vision portals exist and you attempt
     * to use {@code enableLiveView(...)}. Using {@code setLiveViewContainerId(int)} avoids that
     * failure mode and is safe even when only a single portal is used.</p>
     */
    private static void applyDefaultLiveViewContainerId(VisionPortal.Builder builder, HardwareMap hw) {
        if (builder == null) return;
        int id = resolveDefaultLiveViewContainerId(hw);
        if (id != 0) {
            builder.setLiveViewContainerId(id);
        }
    }

    /**
     * Configuration for {@link #aprilTags(HardwareMap, String, Config)}.
     */
    public static final class Config {

        /**
         * Camera streaming resolution. If {@code null}, defaults to 640x480.
         */
        public Size cameraResolution = DEFAULT_RESOLUTION;

        /**
         * Camera mount extrinsics (robot-frame camera pose).
         *
         * <p>If set, it is converted and applied to the FTC {@link AprilTagProcessor.Builder} via
         * {@link AprilTagProcessor.Builder#setCameraPose(Position, YawPitchRollAngles)} to enable
         * FTC SDK robot-pose estimation when supported.</p>
         */
        public CameraMountConfig cameraMount = null;

        /**
         * Optional pitch offset (radians) applied when converting {@link #cameraMount} into
         * the FTC {@link YawPitchRollAngles} used by {@code setCameraPose}.
         *
         * <p>Default is {@code 0}. Keep this at 0 unless you have a specific FTC sample or
         * measurement that indicates an offset is required for your configuration.</p>
         */
        public double sdkPitchRadOffset = 0.0;

        /**
         * Optional tag library override for the FTC AprilTag processor.
         *
         * <p>When {@code null}, Phoenix defaults to
         * {@link AprilTagGameDatabase#getCurrentGameTagLibrary()}.
         * Set this when you are using custom-printed tags (different IDs and/or sizes).</p>
         *
         * <p><b>Important:</b> this only tells the detector which tags exist and what their
         * physical sizes are. It does <em>not</em> define which tags are safe to treat as
         * field-fixed landmarks for localization or Drive Guidance. That policy lives in
         * {@link edu.ftcphoenix.fw.field.TagLayout} (for official FTC games, usually
         * {@link FtcGameTagLayout#currentGameFieldFixed()}).</p>
         */
        public AprilTagLibrary tagLibrary = null;

        private Config() {
            // Defaults set via field initializers.
        }

        /**
         * Create a new configuration instance with Phoenix defaults.
         */
        public static Config defaults() {
            return new Config();
        }

        /**
         * Convenience helper to attach a tag library override.
         *
         * @param library AprilTag library to use (may be {@code null} to use current game library)
         * @return this config for chaining
         */
        public Config withTagLibrary(AprilTagLibrary library) {
            this.tagLibrary = library;
            return this;
        }


        /**
         * Convenience helper to attach a camera mount without call-site boilerplate.
         *
         * @param mount camera mount (may be {@code null} to clear)
         * @return this config for chaining
         */
        public Config withCameraMount(CameraMountConfig mount) {
            this.cameraMount = mount;
            return this;
        }

        /**
         * Convenience helper to set camera resolution.
         *
         * @param resolution requested resolution (may be {@code null} to use default)
         * @return this config for chaining
         */
        public Config withCameraResolution(Size resolution) {
            this.cameraResolution = resolution;
            return this;
        }

        /**
         * Deep copy of this config.
         */
        public Config copy() {
            Config c = new Config();
            c.cameraResolution = this.cameraResolution;
            c.cameraMount = this.cameraMount;
            c.sdkPitchRadOffset = this.sdkPitchRadOffset;
            c.tagLibrary = this.tagLibrary;
            return c;
        }
    }

    /**
     * Create a basic AprilTag sensor using a webcam and the official game tag
     * layout from {@link AprilTagGameDatabase#getCurrentGameTagLibrary()}.
     *
     * <p>This overload uses default configuration. See
     * {@link #aprilTags(HardwareMap, String, Config)} for customization.</p>
     *
     * @param hw         robot {@link HardwareMap}
     * @param cameraName hardware configuration name of the webcam
     * @return a ready-to-use {@link AprilTagSensor}
     */
    public static AprilTagSensor aprilTags(HardwareMap hw, String cameraName) {
        return aprilTags(hw, cameraName, Config.defaults());
    }

    /**
     * Create an AprilTag sensor using a webcam and the official game tag layout.
     *
     * <p>The returned {@link AprilTagSensor} instance performs ID filtering
     * and age checks, and converts FTC's pose information into Phoenix framing
     * using {@link FtcFrames}.</p>
     *
     * <p>The FTC {@link AprilTagLibrary} configured here is detector metadata, not a fixed-field
     * localization layout. Callers that later want AprilTag-based field pose solving must still
     * supply a {@link edu.ftcphoenix.fw.field.TagLayout} containing only truly fixed tags.</p>
     *
     * @param hw         robot {@link HardwareMap}
     * @param cameraName hardware configuration name of the webcam
     * @param cfg        configuration options (must not be {@code null})
     * @return a ready-to-use {@link AprilTagSensor}
     */
    public static AprilTagSensor aprilTags(HardwareMap hw, String cameraName, Config cfg) {
        Objects.requireNonNull(hw, "hardwareMap is required");
        Objects.requireNonNull(cameraName, "cameraName is required");
        Objects.requireNonNull(cfg, "cfg is required");

        WebcamName webcam = hw.get(WebcamName.class, cameraName);

        // Configure the AprilTag processor: current-game library, inches + radians for pose.
        AprilTagProcessor.Builder tagBuilder = new AprilTagProcessor.Builder()
                .setTagLibrary(cfg.tagLibrary != null ? cfg.tagLibrary : AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS);

        // Optional: apply Phoenix camera extrinsics so FTC can compute robotPose.
        if (cfg.cameraMount != null) {
            applyCameraMountToAprilTagProcessor(tagBuilder, cfg.cameraMount, cfg.sdkPitchRadOffset);
        }

        AprilTagProcessor processor = tagBuilder.build();

        // Wire the processor into a VisionPortal using the webcam.
        Size resolution = (cfg.cameraResolution != null) ? cfg.cameraResolution : DEFAULT_RESOLUTION;
        VisionPortal.Builder portalBuilder = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(processor)
                .setCameraResolution(resolution);

        // Important FTC SDK quirk:
        // If more than one VisionPortal exists (even across previous testers/screens),
        // the SDK requires you to use setLiveViewContainerId(int) rather than
        // enableLiveView(bool). We always set the container id when it is available
        // to avoid hard-to-debug “multiple vision portals” startup errors.
        applyDefaultLiveViewContainerId(portalBuilder, hw);

        VisionPortal portal = portalBuilder.build();

        return new PortalAprilTagSensor(portal, processor);
    }

    /**
     * Apply a Phoenix {@link CameraMountConfig} to an FTC {@link AprilTagProcessor.Builder}.
     *
     * <p>This overload uses {@code sdkPitchRadOffset = 0}.</p>
     */
    public static void applyCameraMountToAprilTagProcessor(
            AprilTagProcessor.Builder builder,
            CameraMountConfig mount
    ) {
        applyCameraMountToAprilTagProcessor(builder, mount, 0.0);
    }

    /**
     * Apply a Phoenix {@link CameraMountConfig} to an FTC {@link AprilTagProcessor.Builder}.
     *
     * <p>This converts the mount pose from Phoenix framing into the FTC AprilTag Localization
     * “camera axes” convention (see {@link FtcFrames}) before passing it to
     * {@link AprilTagProcessor.Builder#setCameraPose(Position, YawPitchRollAngles)}.</p>
     *
     * @param builder           FTC AprilTag processor builder (non-null)
     * @param mount             Phoenix camera mount (robot frame) (non-null)
     * @param sdkPitchRadOffset optional pitch offset (radians) to apply after conversion
     */
    public static void applyCameraMountToAprilTagProcessor(
            AprilTagProcessor.Builder builder,
            CameraMountConfig mount,
            double sdkPitchRadOffset
    ) {
        Objects.requireNonNull(builder, "builder");
        Objects.requireNonNull(mount, "mount");

        // Convert robot->camera pose from Phoenix framing to FTC localization camera axes.
        Pose3d robotToCameraPose = mount.robotToCameraPose();
        Pose3d ftcLocCamPose = FtcFrames.toFtcLocalizationCameraAxesFromPhoenix(robotToCameraPose);

        Position pos = new Position(
                DistanceUnit.INCH,
                ftcLocCamPose.xInches,
                ftcLocCamPose.yInches,
                ftcLocCamPose.zInches,
                0
        );

        YawPitchRollAngles ypr = new YawPitchRollAngles(
                AngleUnit.RADIANS,
                ftcLocCamPose.yawRad,
                ftcLocCamPose.pitchRad + sdkPitchRadOffset,
                ftcLocCamPose.rollRad,
                0
        );

        builder.setCameraPose(pos, ypr);
    }

    // ---------------------------------------------------------------------------------------------
    // AprilTag pose conversions
    // ---------------------------------------------------------------------------------------------

    /**
     * Convert an FTC SDK {@link AprilTagPoseRaw} into Phoenix's {@link Pose3d} camera-to-tag pose.
     *
     * <p><b>Why rawPose?</b>
     * The FTC game database tag poses ({@code AprilTagMetadata.fieldPosition/fieldOrientation}) are
     * defined in the AprilTag/OpenCV coordinate conventions used by {@code rawPose}. The SDK also
     * provides {@code ftcPose}, which is a convenience re-framing for on-screen telemetry. Mixing
     * {@code ftcPose} with game-database tag orientations can yield inconsistent transforms.
     *
     * <p><b>Frames</b>
     * <ul>
     *   <li>{@code rawPose} camera frame: +X right, +Y down, +Z forward (out of the lens).</li>
     *   <li>Phoenix camera frame: +X forward, +Y left, +Z up.</li>
     *   <li>Tag frame: AprilTag native tag axes (as defined by the AprilTag library / FTC SDK).</li>
     * </ul>
     *
     * <p>We change the <em>camera</em> basis into Phoenix framing, but we intentionally do not
     * "re-basis" the tag frame here. That keeps the tag frame consistent with FTC's field tag
     * layouts.</p>
     */
    private static Pose3d cameraToTagFromRawPose(AprilTagPoseRaw rawPose) {
        if (rawPose == null || rawPose.R == null) {
            return null;
        }

        // Translation: raw camera (+X right, +Y down, +Z forward) -> Phoenix camera (+X forward, +Y left, +Z up)
        //   PhoenixX = rawZ
        //   PhoenixY = -rawX
        //   PhoenixZ = -rawY
        double xIn = rawPose.z;
        double yIn = -rawPose.x;
        double zIn = -rawPose.y;

        // Rotation: rawPose.R is a 3x3 rotation matrix for the pose (tag -> camera) in the raw camera axes.
        // Change camera basis only (left-multiply).
        Mat3 rRaw = mat3FromMatrixF(rawPose.R);
        Mat3 rCam = FtcFrames.phoenixFromAprilTagRawCameraFrame().mul(rRaw);

        Mat3.YawPitchRoll ypr = Mat3.toYawPitchRoll(rCam);
        return new Pose3d(xIn, yIn, zIn, ypr.yawRad, ypr.pitchRad, ypr.rollRad);
    }

    private static Mat3 mat3FromMatrixF(MatrixF m) {
        // MatrixF supports row/col indexing via get(r,c). rawPose.R is documented as a 3x3 rotation matrix.
        // If the SDK ever supplies a larger matrix, we still read the upper-left 3x3.
        return new Mat3(
                m.get(0, 0), m.get(0, 1), m.get(0, 2),
                m.get(1, 0), m.get(1, 1), m.get(1, 2),
                m.get(2, 0), m.get(2, 1), m.get(2, 2)
        );
    }

    /**
     * Internal implementation of {@link AprilTagSensor} backed by a
     * {@link VisionPortal} and {@link AprilTagProcessor}.
     *
     * <p>The sensor is memoized by {@link edu.ftcphoenix.fw.core.time.LoopClock#cycle()} so all
     * callers within one Phoenix loop observe the same raw detections snapshot.</p>
     */
    static final class PortalAprilTagSensor implements AprilTagSensor {

        private final VisionPortal portal;   // kept for lifecycle; used for close()
        private boolean closed = false;

        private final AprilTagProcessor processor;

        private long lastCycle = Long.MIN_VALUE;
        private edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections lastDetections =
                edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections.none();

        PortalAprilTagSensor(VisionPortal portal, AprilTagProcessor processor) {
            this.portal = Objects.requireNonNull(portal, "portal");
            this.processor = Objects.requireNonNull(processor, "processor");
        }

        /**
         * Release the underlying {@link VisionPortal}.
         */
        @Override
        public void close() {
            if (closed) {
                return;
            }
            closed = true;

            try {
                portal.close();
            } catch (Exception ignored) {
                // Non-throwing by contract.
            }
        }

        /**
         * Returns the raw detections snapshot for the current Phoenix loop.
         *
         * <p>This method is idempotent by {@link edu.ftcphoenix.fw.core.time.LoopClock#cycle()} so
         * guidance, localization, telemetry, and tag selection can all share the same sensor
         * instance without accidentally reading different frames.</p>
         */
        @Override
        public edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections get(edu.ftcphoenix.fw.core.time.LoopClock clock) {
            Objects.requireNonNull(clock, "clock");
            long cyc = clock.cycle();
            if (cyc == lastCycle) {
                return lastDetections;
            }
            lastCycle = cyc;
            lastDetections = readDetections();
            return lastDetections;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public void reset() {
            lastCycle = Long.MIN_VALUE;
            lastDetections = edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections.none();
        }

        /**
         * Framework-style debug dump (optional helper; not part of {@link AprilTagSensor}).
         *
         * @param dbg    debug sink (may be {@code null})
         * @param prefix key prefix (may be {@code null} or empty)
         */
        public void debugDump(DebugSink dbg, String prefix) {
            if (dbg == null) {
                return;
            }
            String p = (prefix == null || prefix.isEmpty()) ? "ftcVision.tags" : prefix;

            edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections dets = lastDetections;
            dbg.addLine(p + ": PortalAprilTagSensor");
            dbg.addData(p + ".detections.count", dets.observations.size());
            dbg.addData(p + ".detections.ageSec", dets.ageSec);
            dbg.addData(p + ".detections.visibleIds", dets.visibleIds(Double.POSITIVE_INFINITY).toString());
        }

        /**
         * Converts the current FTC detections list into one immutable per-frame Phoenix snapshot.
         */
        private edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections readDetections() {
            List<AprilTagDetection> detections = processor.getDetections();
            if (detections == null || detections.isEmpty()) {
                return edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections.none(Double.POSITIVE_INFINITY);
            }

            long nowNanos = System.nanoTime();
            java.util.ArrayList<AprilTagObservation> out = new java.util.ArrayList<AprilTagObservation>(detections.size());
            double snapshotAgeSec = Double.POSITIVE_INFINITY;

            for (AprilTagDetection det : detections) {
                if (det == null) {
                    continue;
                }

                // We need pose values to build cameraToTagPose.
                if (det.rawPose == null && det.ftcPose == null) {
                    continue;
                }

                long frameTime = det.frameAcquisitionNanoTime;
                double ageSec = (frameTime == 0L)
                        ? 0.0
                        : (nowNanos - frameTime) / NANOS_PER_SECOND;
                if (ageSec < snapshotAgeSec) {
                    snapshotAgeSec = ageSec;
                }

                Pose3d cameraToTagPose = (det.rawPose != null)
                        ? cameraToTagFromRawPose(det.rawPose)
                        : null;

                if (cameraToTagPose == null && det.ftcPose != null) {
                    Pose3d ftcCamToTag = new Pose3d(
                            det.ftcPose.x,
                            det.ftcPose.y,
                            det.ftcPose.z,
                            Math.toRadians(det.ftcPose.yaw),
                            Math.toRadians(det.ftcPose.roll),
                            Math.toRadians(det.ftcPose.pitch)
                    );
                    cameraToTagPose = FtcFrames.toPhoenixFromFtcDetectionFrame(ftcCamToTag);
                }

                if (cameraToTagPose == null) {
                    continue;
                }

                Pose3d fieldToRobotPose = null;
                if (det.robotPose != null) {
                    Position pos = det.robotPose.getPosition();
                    YawPitchRollAngles ypr = det.robotPose.getOrientation();

                    fieldToRobotPose = new Pose3d(
                            pos.x,
                            pos.y,
                            pos.z,
                            ypr.getYaw(AngleUnit.RADIANS),
                            ypr.getPitch(AngleUnit.RADIANS),
                            ypr.getRoll(AngleUnit.RADIANS)
                    );
                }

                AprilTagObservation obs = (fieldToRobotPose != null)
                        ? AprilTagObservation.target(det.id, cameraToTagPose, fieldToRobotPose, ageSec)
                        : AprilTagObservation.target(det.id, cameraToTagPose, ageSec);
                out.add(obs);
            }

            if (out.isEmpty()) {
                return edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections.none(snapshotAgeSec);
            }
            if (!Double.isFinite(snapshotAgeSec)) {
                snapshotAgeSec = 0.0;
            }
            return edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections.of(snapshotAgeSec, out);
        }
    }

}