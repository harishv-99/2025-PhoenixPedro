package edu.ftcsushi.robots.phoenix;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcsushi.fw.ftc.vision.AprilTagVisionLane;
import edu.ftcsushi.fw.ftc.vision.FtcLimelightAprilTagVisionLane;
import edu.ftcsushi.fw.ftc.vision.FtcWebcamAprilTagVisionLane;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;

/**
 * Robot-owned factory that selects Phoenix's concrete AprilTag vision lane backend.
 *
 * <p>Phoenix keeps this selection wrapper in the robot layer so the rest of the robot consumes only
 * the backend-neutral {@link AprilTagVisionLane} seam. The framework still owns the concrete FTC
 * boundary implementations for each supported backend.</p>
 */
public final class PhoenixVisionFactory {

    /** Supported concrete AprilTag vision backends for Phoenix. */
    public enum Backend {
        /** Use the standard FTC webcam-backed AprilTag lane. */
        WEBCAM,
        /** Use the FTC Limelight-backed AprilTag lane. */
        LIMELIGHT
    }

    /** Mutable data-only choice of Phoenix AprilTag backend and its two authoring drafts. */
    public static final class Config {

        /** Backend selected for this Phoenix runtime. */
        public Backend backend = Backend.WEBCAM;

        /** Webcam draft inspected only when {@link #backend} is {@link Backend#WEBCAM}. */
        public FtcWebcamAprilTagVisionLane.Config webcam = webcamDefaults();

        /** Limelight draft inspected only when {@link #backend} is {@link Backend#LIMELIGHT}. */
        public FtcLimelightAprilTagVisionLane.Config limelight = limelightDefaults();

        private Config() {
        }

        /**
         * Returns a fresh complete Phoenix vision draft.
         *
         * <p>The device names and camera mounts are current Phoenix physical claims. This factory
         * supplies software-valid authoring data; it does not prove either device or mount.</p>
         *
         * @return fresh mutable backend-selection draft
         */
        public static Config defaults() {
            return new Config();
        }

        /**
         * Returns the raw device name from only the selected backend draft.
         *
         * @return selected backend's authored FTC device name, which may still be invalid
         * @throws NullPointerException if the backend or its selected draft is {@code null}
         */
        public String activeDeviceName() {
            Backend selected = Objects.requireNonNull(
                    backend,
                    "PhoenixVisionFactory.Config.backend"
            );
            switch (selected) {
                case WEBCAM:
                    return Objects.requireNonNull(
                            webcam,
                            "PhoenixVisionFactory.Config.webcam"
                    ).webcamName;
                case LIMELIGHT:
                    return Objects.requireNonNull(
                            limelight,
                            "PhoenixVisionFactory.Config.limelight"
                    ).hardwareName;
                default:
                    throw new IllegalArgumentException("Unsupported Phoenix vision backend: " + selected);
            }
        }

        /**
         * Returns the raw camera mount from only the selected backend draft.
         *
         * @return selected backend's authored camera mount, which may still be invalid
         * @throws NullPointerException if the backend or its selected draft is {@code null}
         */
        public CameraMountConfig activeCameraMount() {
            Backend selected = Objects.requireNonNull(
                    backend,
                    "PhoenixVisionFactory.Config.backend"
            );
            switch (selected) {
                case WEBCAM:
                    return Objects.requireNonNull(
                            webcam,
                            "PhoenixVisionFactory.Config.webcam"
                    ).cameraMount;
                case LIMELIGHT:
                    return Objects.requireNonNull(
                            limelight,
                            "PhoenixVisionFactory.Config.limelight"
                    ).cameraMount;
                default:
                    throw new IllegalArgumentException("Unsupported Phoenix vision backend: " + selected);
            }
        }
    }

    private PhoenixVisionFactory() {
    }

    /**
     * Creates the concrete AprilTag vision lane requested by the supplied Phoenix Config.
     *
     * <p>The selected concrete owner validates and snapshots its complete active configuration
     * before looking up the requested device. This robot selector does not validate the inactive
     * backend.</p>
     *
     * @param hardwareMap FTC hardware map used to acquire the chosen vision device
     * @param cfg Phoenix AprilTag backend-selection config
     * @return concrete vision lane for the active Phoenix backend
     * @throws NullPointerException if the map, Config, selected backend, or selected backend draft
     *                              is {@code null}
     * @throws IllegalArgumentException if the selected backend draft is invalid
     */
    public static AprilTagVisionLane create(HardwareMap hardwareMap, Config cfg) {
        Objects.requireNonNull(hardwareMap, "hardwareMap");
        Objects.requireNonNull(cfg, "PhoenixVisionFactory.Config");

        Backend selected = Objects.requireNonNull(
                cfg.backend,
                "PhoenixVisionFactory.Config.backend"
        );
        switch (selected) {
            case WEBCAM:
                return new FtcWebcamAprilTagVisionLane(
                        hardwareMap,
                        Objects.requireNonNull(
                                cfg.webcam,
                                "PhoenixVisionFactory.Config.webcam"
                        )
                );
            case LIMELIGHT:
                return new FtcLimelightAprilTagVisionLane(
                        hardwareMap,
                        Objects.requireNonNull(
                                cfg.limelight,
                                "PhoenixVisionFactory.Config.limelight"
                        )
                );
            default:
                throw new IllegalArgumentException("Unsupported Phoenix vision backend: " + selected);
        }
    }

    private static FtcWebcamAprilTagVisionLane.Config webcamDefaults() {
        FtcWebcamAprilTagVisionLane.Config config =
                FtcWebcamAprilTagVisionLane.Config.defaults();
        config.webcamName = "Webcam 1";
        config.cameraMount = currentCameraMount();
        return config;
    }

    private static FtcLimelightAprilTagVisionLane.Config limelightDefaults() {
        FtcLimelightAprilTagVisionLane.Config config =
                FtcLimelightAprilTagVisionLane.Config.defaults();
        config.hardwareName = "limelight";
        config.pipelineIndex = 0;
        config.pollRateHz = 100;
        config.cameraMount = currentCameraMount();
        return config;
    }

    private static CameraMountConfig currentCameraMount() {
        return CameraMountConfig.ofDegrees(
                9.97,
                -1.80,
                13.68,
                1.9,
                -18.2,
                -1.7
        );
    }
}
