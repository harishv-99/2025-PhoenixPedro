package edu.ftcsushi.fw.ftc.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Objects;

/** Deferred camera factories for tools that acquire one selected camera later. */
public final class AprilTagCameraFactories {
    private AprilTagCameraFactories() {}

    interface WebcamOpener {
        OwnedAprilTagCamera open(HardwareMap hardwareMap, FtcWebcamVisionLane.ActiveConfig config);
    }
    interface LimelightOpener {
        OwnedAprilTagCamera open(HardwareMap hardwareMap, FtcLimelightVisionLane.Config config);
    }

    /** Captures an explicitly tag-enabled webcam configuration before later acquisition. */
    public static AprilTagCameraFactory webcam(FtcWebcamVisionLane.Config config) {
        return webcam(config, null, (map, captured) -> {
            FtcWebcamVisionLane camera = new FtcWebcamVisionLane(map, captured.freshConfig());
            return new OwnedAprilTagCamera(camera, camera.aprilTags());
        });
    }

    static AprilTagCameraFactory webcam(FtcWebcamVisionLane.Config config,
                                        FtcWebcamVisionLane.ResolutionReader reader) {
        return webcam(config, reader, (map, captured) -> {
            FtcWebcamVisionLane camera = new FtcWebcamVisionLane(map, captured.freshConfig());
            return new OwnedAprilTagCamera(camera, camera.aprilTags());
        });
    }

    static AprilTagCameraFactory webcam(FtcWebcamVisionLane.Config config,
                                        FtcWebcamVisionLane.ResolutionReader reader,
                                        WebcamOpener opener) {
        FtcWebcamVisionLane.ActiveConfig captured = reader == null
                ? FtcWebcamVisionLane.captureActiveConfig(config)
                : FtcWebcamVisionLane.captureActiveConfig(config, reader);
        Objects.requireNonNull(opener, "opener");
        return new AprilTagCameraFactory() {
            @Override public OwnedAprilTagCamera open(HardwareMap hardwareMap) {
                return Objects.requireNonNull(
                        opener.open(hardwareMap, captured), "webcam opener returned null");
            }
            @Override public String description() { return "webcam: " + captured.webcamName(); }
        };
    }

    /** Captures an explicitly tag-enabled Limelight configuration before later acquisition. */
    public static AprilTagCameraFactory limelight(FtcLimelightVisionLane.Config config) {
        return limelight(config, (map, captured) -> {
            FtcLimelightVisionLane camera = new FtcLimelightVisionLane(map, captured);
            return new OwnedAprilTagCamera(camera, camera.aprilTags());
        });
    }

    static AprilTagCameraFactory limelight(FtcLimelightVisionLane.Config config,
                                           LimelightOpener opener) {
        FtcLimelightVisionLane.Config captured = Objects.requireNonNull(config, "FtcLimelightVisionLane.Config")
                .validatedCopy("FtcLimelightVisionLane.Config");
        if (captured.aprilTags == null) throw new IllegalArgumentException(
                "AprilTagCameraFactories.limelight requires Config.aprilTags");
        Objects.requireNonNull(opener, "opener");
        return new AprilTagCameraFactory() {
            @Override public OwnedAprilTagCamera open(HardwareMap hardwareMap) {
                return Objects.requireNonNull(
                        opener.open(hardwareMap, captured.copy()), "Limelight opener returned null");
            }
            @Override public String description() { return "limelight: " + captured.hardwareName; }
        };
    }
}
