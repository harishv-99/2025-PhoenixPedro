package edu.ftcsushi.robots.examples.cameraonlypickup;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcsushi.fw.drive.DriveSource;
import edu.ftcsushi.fw.drive.guidance.GuidedApproach;
import edu.ftcsushi.fw.drive.source.GamepadDriveSource;
import edu.ftcsushi.fw.ftc.FtcDrives;
import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.ftc.input.GamepadDevice;

/** Complete camera-only open-floor graph, disabled until an adopting robot validates its profile. */
@TeleOp(name = "FW Camera-only Pickup", group = "FW Examples")
@Disabled
public final class CameraOnlyPickupTeleOp extends FtcRobotOpMode {
    @Override protected void configure(RobotProgram program) {
        CameraOnlyPickupProfile profile = CameraOnlyPickupProfile.example();
        profile.requireMotionAllowed();
        CameraOnlyPickupCamera camera = program.service(new CameraOnlyPickupCamera(hardwareMap, profile.camera));
        CameraOnlyPickupIntake intake = program.output(new CameraOnlyPickupIntake(hardwareMap, profile.intake));
        GamepadDevice driver = new GamepadDevice(gamepad1);
        DriveSource manualDrive = manualDrive(driver);
        CameraOnlyPickup pickup = program.service(new CameraOnlyPickup(profile,
                camera.objects(), intake.occupancy(), intake::setCollecting, manualDrive));
        CameraOnlyPickupControls controls = new CameraOnlyPickupControls(driver.rightBumper(), driver.b());
        controls.bind(program.callbackBindings(), program.taskBindings(), pickup);
        program.drive(pickup.driveSource(), FtcDrives.mecanum(hardwareMap, profile.drive));
        program.presenter((clock, telemetry) -> {
            GuidedApproach.Status status = pickup.status();
            telemetry.addData("pickup phase", status.phase);
            telemetry.addData("pickup outcome", status.hasFailure ? "FAILED" : status.outcome);
            telemetry.addData("pickup reason", status.reason);
            telemetry.addData("new verification frames", status.verifiedCaptureCount);
            telemetry.addData("controls", "Hold right bumper: one pickup; release/B: manual control");
        });
    }

    /** Explicit robot-centric idle drive: left stick translates and right X turns without localization. */
    private static DriveSource manualDrive(GamepadDevice driver) {
        driver.setAxisDeadband(0.02);
        GamepadDriveSource.Config config = GamepadDriveSource.Config.defaults();
        config.deadband = 0.05;
        config.translateExpo = 1.5;
        config.rotateExpo = 1.5;
        config.translateScale = 1.0;
        config.rotateScale = 1.0;
        return new GamepadDriveSource(driver.leftX(), driver.leftY(), driver.rightX(), config);
    }
}
