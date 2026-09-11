package edu.ftcsushi.robots.examples.tagalignment;

import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.drive.guidance.DriveGuidancePlan;
import edu.ftcsushi.fw.drive.guidance.DriveGuidanceTask;
import edu.ftcsushi.fw.ftc.FtcDrives;
import edu.ftcsushi.fw.ftc.vision.FtcWebcamVisionLane;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;

/** Complete, independent authoring draft; every physical number is illustrative, not calibrated. */
public final class TagAlignmentProfile {
    /** Set true only after reviewing this entire profile and testing the robot safely. */
    public boolean allowMotion = false;
    public final FtcDrives.MecanumConfig drive = FtcDrives.MecanumConfig.defaults();
    public final FtcWebcamVisionLane.Config camera = FtcWebcamVisionLane.Config.defaults();
    /** Must exist in the configured detector library, with the correct physical printed tag size. */
    public int tagId = 1;
    /** Desired robot-center position in the tag frame: +X outward from its face, +Y left. */
    public double tagForwardInches = 18.0;
    public double tagLeftInches = 0.0;
    /** PI points the robot toward a tag whose +X points outward. */
    public double tagHeadingRad = Math.PI;
    /** Camera-frame evidence older than this is unavailable, not an arrival. */
    public double maxTagAgeSec = 0.20;
    public DriveGuidancePlan.Tuning tuning = DriveGuidancePlan.Tuning.defaults()
            .withMaxTranslateCmd(0.20).withMaxOmegaCmd(0.20);
    public final DriveGuidanceTask.Config auto = new DriveGuidanceTask.Config();

    private TagAlignmentProfile() {
        drive.wiring.frontLeftName = "frontLeftMotor";
        drive.wiring.frontRightName = "frontRightMotor";
        drive.wiring.backLeftName = "backLeftMotor";
        drive.wiring.backRightName = "backRightMotor";
        drive.wiring.frontLeftDirection = Direction.FORWARD;
        drive.wiring.frontRightDirection = Direction.REVERSE;
        drive.wiring.backLeftDirection = Direction.FORWARD;
        drive.wiring.backRightDirection = Direction.REVERSE;
        drive.enableZeroPowerBrake = true;
        drive.drivebase.maxAxial = 0.25;
        drive.drivebase.maxLateral = 0.25;
        drive.drivebase.maxOmega = 0.20;
        camera.webcamName = "Webcam 1";
        camera.cameraMount = CameraMountConfig.of(4, 0, 8, 0, 0, 0);
        camera.aprilTags = FtcWebcamVisionLane.AprilTagConfig.defaults();
        // Null library means the SDK's current-game detector metadata, not a field layout.
        // Set camera.aprilTags.tagLibrary for different IDs or printed physical sizes.
        auto.positionTolInches = 1.0;
        auto.headingTolRad = Math.toRadians(3.0);
        auto.timeoutSec = 5.0;
        auto.maxNoGuidanceSec = 0.30;
    }

    /** Returns a fresh draft. Active framework owners snapshot the values they consume. */
    public static TagAlignmentProfile example() { return new TagAlignmentProfile(); }

    /** Fails before opening any device when the physical review has not been acknowledged. */
    public void requireMotionAllowed() {
        if (!allowMotion) {
            throw new IllegalStateException("TagAlignmentProfile.allowMotion is false: review motor "
                    + "wiring, camera mount, tag metadata, approach clearance, and tuning first");
        }
    }
}
