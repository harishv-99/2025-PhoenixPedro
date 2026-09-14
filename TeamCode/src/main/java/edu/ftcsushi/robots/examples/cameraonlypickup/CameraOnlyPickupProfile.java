package edu.ftcsushi.robots.examples.cameraonlypickup;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.drive.guidance.DriveGuidancePlan;
import edu.ftcsushi.fw.ftc.FtcDrives;
import edu.ftcsushi.fw.ftc.vision.FtcFloorObjectVision;
import edu.ftcsushi.fw.ftc.vision.FtcWebcamVisionLane;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.sensing.vision.FloorTargetModel;

/** Complete independent authoring draft; these illustrative values are not a calibrated robot. */
public final class CameraOnlyPickupProfile {
    /** Keep false until every physical fact and motion limit has passed supervised review. */
    public boolean allowMotion = false;
    /** Four-motor wiring and final command limits; the FTC drive snapshots this draft. */
    public final FtcDrives.MecanumConfig drive = FtcDrives.MecanumConfig.defaults();
    /** Fixed webcam configuration; no AprilTag processor or localization is needed here. */
    public final FtcWebcamVisionLane.Config camera = FtcWebcamVisionLane.Config.defaults();
    /** Independent intake motor and occupancy switch wiring. */
    public final CameraOnlyPickupIntake.Config intake = CameraOnlyPickupIntake.Config.defaults();
    /** Robot-to-intake transform in inches/radians; intake +X defines the final command direction. */
    public Pose2d robotToIntake = new Pose2d(3, 0, 0);
    /** Target lies this positive distance ahead of the intake when verification begins. */
    public double standOffInches = 5.0;
    /** Original camera captures older than this many seconds cannot guide or verify. */
    public double maxObservationAgeSec = 0.20;
    /** Controller gains and normalized command caps, not physical velocity limits. */
    public DriveGuidancePlan.Tuning tuning = DriveGuidancePlan.Tuning.defaults()
            .withMaxTranslateCmd(0.20).withMaxOmegaCmd(0.20);
    /** Zero-command settling interval in seconds; elapsed time cannot prove physical rest. */
    public double settleSec = 0.10;
    /** Maximum approach-position error in inches for the unique-candidate verification window. */
    public double positionToleranceInches = 0.50;
    /** Maximum intake-facing error in radians for that same verification window. */
    public double headingToleranceRad = Math.toRadians(5);
    /** Full verification phase deadline, including settling, in seconds. */
    public double verificationTimeoutSec = 0.60;
    /** Positive normalized final straight-translation command; not inches per second. */
    public double finalCommand = 0.10;
    /** Final intake duration limit in seconds; does not bound measured travel distance. */
    public double maxFinalSec = 0.50;
    /** Maximum original sensor-observation age in seconds. */
    public double maxCaptureAgeSec = 0.10;
    /** Whole-attempt deadline in seconds, including guidance and verification. */
    public double maxAttemptSec = 3.0;

    private CameraOnlyPickupProfile() {
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
        camera.cameraMount = CameraMountConfig.of(4, 0, 10, 0, Math.toRadians(45), 0);
        camera.floorObjects = FtcFloorObjectVision.Config.defaults();
        camera.floorObjects.targetModel = FloorTargetModel.atHeightInches(2)
                .withMaxRangeInches(48);
    }

    /** Returns a fresh motion-disabled draft; each active owner snapshots the values it consumes. */
    public static CameraOnlyPickupProfile example() { return new CameraOnlyPickupProfile(); }

    /**
     * Fails before device acquisition unless motion is reviewed and all five motor names are
     * nonblank and distinct after trimming. Different owners must not write one configured motor.
     * Distinct names cannot detect two hardware-map aliases for the same physical device.
     */
    public void requireMotionAllowed() {
        if (!allowMotion) {
            throw new IllegalStateException("CameraOnlyPickupProfile.allowMotion is false: validate "
                    + "camera calibration/mount/visibility, intake sensor, drive/intake direction, "
                    + "open-floor clearance, command/time limits, and STOP before allowing motion");
        }
        requireDistinctMotorOwners();
    }

    /** Rejects detectable cross-owner and within-drive wiring collisions before opening devices. */
    private void requireDistinctMotorOwners() {
        if (drive.wiring == null) {
            throw new IllegalStateException("CameraOnlyPickupProfile.drive.wiring is required");
        }
        String[] names = {drive.wiring.frontLeftName, drive.wiring.frontRightName,
                drive.wiring.backLeftName, drive.wiring.backRightName, intake.motorName};
        String[] fields = {"drive.wiring.frontLeftName", "drive.wiring.frontRightName",
                "drive.wiring.backLeftName", "drive.wiring.backRightName", "intake.motorName"};
        for (int i = 0; i < names.length; i++) {
            if (names[i] == null || names[i].trim().isEmpty()) {
                throw new IllegalStateException("CameraOnlyPickupProfile." + fields[i]
                        + " must name an FTC motor before acquiring devices");
            }
            String key = names[i].trim();
            for (int earlier = 0; earlier < i; earlier++) {
                if (key.equals(names[earlier].trim())) {
                    throw new IllegalStateException("Camera-only pickup motor ownership collision: "
                            + fields[earlier] + " and " + fields[i] + " both resolve to FTC key \""
                            + key + "\". Configure distinct motor names before acquiring devices.");
                }
            }
        }
    }
}
