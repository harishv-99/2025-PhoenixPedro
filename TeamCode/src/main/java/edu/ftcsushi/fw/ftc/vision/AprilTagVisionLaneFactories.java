package edu.ftcsushi.fw.ftc.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Convenience builders for common {@link AprilTagVisionLaneFactory} variants.
 *
 * <p>These helpers are mainly useful in testers and robot-layer factories where code wants to keep
 * backend selection explicit without duplicating anonymous opener lambdas everywhere.</p>
 */
public final class AprilTagVisionLaneFactories {

    /** Package-private construction seam for factory capture tests. */
    interface WebcamOwnerOpener {
        AprilTagVisionLane open(
                HardwareMap hardwareMap,
                FtcWebcamAprilTagVisionLane.ActiveConfig config
        );
    }

    /** Package-private construction seam for factory capture tests. */
    interface LimelightOwnerOpener {
        AprilTagVisionLane open(
                HardwareMap hardwareMap,
                FtcLimelightAprilTagVisionLane.Config config
        );
    }

    private static final WebcamOwnerOpener WEBCAM_OWNER_OPENER =
            new WebcamOwnerOpener() {
                @Override
                public AprilTagVisionLane open(
                        HardwareMap hardwareMap,
                        FtcWebcamAprilTagVisionLane.ActiveConfig config
                ) {
                    return new FtcWebcamAprilTagVisionLane(hardwareMap, config);
                }
            };

    private static final LimelightOwnerOpener LIMELIGHT_OWNER_OPENER =
            new LimelightOwnerOpener() {
                @Override
                public AprilTagVisionLane open(
                        HardwareMap hardwareMap,
                        FtcLimelightAprilTagVisionLane.Config config
                ) {
                    return new FtcLimelightAprilTagVisionLane(hardwareMap, config);
                }
            };

    private AprilTagVisionLaneFactories() {
    }

    /**
     * Returns a deferred opener for a webcam-backed AprilTag lane.
     *
     * <p>This method validates and captures the active config immediately without touching
     * {@link HardwareMap}. It deep-snapshots the selected FTC tag library. Each later
     * {@link AprilTagVisionLaneFactory#open(HardwareMap)} creates an independently validated owner
     * with a fresh private library snapshot.</p>
     *
     * @param config complete webcam lane authoring config to validate and capture
     * @return opener that creates {@link FtcWebcamAprilTagVisionLane}
     * @throws NullPointerException if the config or a required field is null
     * @throws IllegalArgumentException if the active config or tag metadata is invalid
     */
    public static AprilTagVisionLaneFactory webcam(FtcWebcamAprilTagVisionLane.Config config) {
        return webcam(config, new FtcWebcamVisionPortalLane.ResolutionReader() {
            @Override
            public int width(android.util.Size size) {
                return size.getWidth();
            }

            @Override
            public int height(android.util.Size size) {
                return size.getHeight();
            }
        }, WEBCAM_OWNER_OPENER);
    }

    static AprilTagVisionLaneFactory webcam(
            FtcWebcamAprilTagVisionLane.Config config,
            FtcWebcamVisionPortalLane.ResolutionReader resolutionReader
    ) {
        return webcam(config, resolutionReader, WEBCAM_OWNER_OPENER);
    }

    static AprilTagVisionLaneFactory webcam(
            FtcWebcamAprilTagVisionLane.Config config,
            FtcWebcamVisionPortalLane.ResolutionReader resolutionReader,
            WebcamOwnerOpener ownerOpener
    ) {
        final FtcWebcamAprilTagVisionLane.ActiveConfig cfg =
                FtcWebcamAprilTagVisionLane.captureActiveConfig(config, resolutionReader);
        final WebcamOwnerOpener opener = java.util.Objects.requireNonNull(
                ownerOpener,
                "ownerOpener"
        );
        return new AprilTagVisionLaneFactory() {
            @Override
            public AprilTagVisionLane open(HardwareMap hardwareMap) {
                return opener.open(hardwareMap, cfg);
            }

            @Override
            public String description() {
                return "webcam: " + cfg.webcamName();
            }
        };
    }

    /**
     * Returns a deferred opener for a Limelight-backed AprilTag lane.
     *
     * <p>This method validates and captures the active config immediately without touching
     * {@link HardwareMap}. Each later {@link AprilTagVisionLaneFactory#open(HardwareMap)} creates
     * an independently validated owner.</p>
     *
     * @param config complete Limelight lane authoring config to validate and capture
     * @return opener that creates {@link FtcLimelightAprilTagVisionLane}
     * @throws NullPointerException if the config is null
     * @throws IllegalArgumentException if the active config is invalid
     */
    public static AprilTagVisionLaneFactory limelight(FtcLimelightAprilTagVisionLane.Config config) {
        return limelight(config, LIMELIGHT_OWNER_OPENER);
    }

    static AprilTagVisionLaneFactory limelight(
            FtcLimelightAprilTagVisionLane.Config config,
            LimelightOwnerOpener ownerOpener
    ) {
        final FtcLimelightAprilTagVisionLane.Config cfg =
                FtcLimelightAprilTagVisionLane.validatedCopy(config);
        final LimelightOwnerOpener opener = java.util.Objects.requireNonNull(
                ownerOpener,
                "ownerOpener"
        );
        return new AprilTagVisionLaneFactory() {
            @Override
            public AprilTagVisionLane open(HardwareMap hardwareMap) {
                return opener.open(hardwareMap, cfg);
            }

            @Override
            public String description() {
                return "limelight: " + cfg.hardwareName;
            }
        };
    }
}
