package org.firstinspires.ftc.teamcode.robots;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "WebcamTester_Raghavan (Blocks to Java)")

public class WebcamTester_Raghavan extends LinearOpMode {

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {
        AprilTagProcessor.Builder myAprilTagProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;
        AprilTagProcessor myAprilTagProcessor;
        VisionPortal myVisionPortal;

        // Put initialization blocks here.
        // Create a new AprilTagProcessor.Builder object and assign it to a variable.
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        // Set whether or not to draw the cube projection on detections.
        myAprilTagProcessorBuilder.setDrawCubeProjection(true);
        // Set whether or not to draw the tag ID on detections.
        myAprilTagProcessorBuilder.setDrawTagID(true);
        // Set whether or not to draw the tag outline on detections.
        myAprilTagProcessorBuilder.setDrawTagOutline(true);
        // Camera Calibration
        // myAprilTagProcessorBuilder.setLensIntrinsics(917.676, 917.676, 658.647, 378.213);
        // Build the AprilTag processor and assign it to a variable.
        myAprilTagProcessor = myAprilTagProcessorBuilder.build();
        // Create a VisionPortal.Builder object so you can specify attributes about the cameras.
        myVisionPortalBuilder = new VisionPortal.Builder();
        // Set the camera to the specified webcam name.
        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        // Add the AprilTag processor.
        myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
        // Set the camera resolution.
        myVisionPortalBuilder.setCameraResolution(new Size(1280, 720));
        // Set the stream format.
        myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        // Set whether the VisionPortal should automatically start streaming
        // when you issue a .build() call on this VisionPortal.Builder object.
        myVisionPortalBuilder.setAutoStartStreamOnBuild(true);
        // Set whether to automatically stop the LiveView (RC preview) when all vision processors are disabled.
        myVisionPortalBuilder.setAutoStopLiveView(true);
        // Build the VisionPortal object and assign it to a variable.
        myVisionPortal = myVisionPortalBuilder.build();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                telemetry.update();
            }
        }
    }
}