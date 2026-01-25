package org.firstinspires.ftc.teamcode.robots;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Disabled
@TeleOp(name = "Hrithika_TestCodes")
public class Hrithika_TestCodes extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{
//        Camera camera = new Camera(hardwareMap, telemetry, "Webcam 1");

        waitForStart();

        while(!isStopRequested() && opModeIsActive()) {

            if (gamepad1.x) {
                telemetry.addLine("|- " + Float.toString(gamepad1.left_stick_x));
                telemetry.addData("A", gamepad1.x);
                telemetry.addData("x axis", gamepad1.left_stick_x);
                telemetry.update();
            }

//            ArrayList<Integer> ids;
//            ids = camera.getAprilTagIds();
//            for(int i : ids) {
//                telemetry.addLine(Integer.toString(i));
//            }


//            if(!tagProcessor.getDetections().isEmpty()) {
//                tag = tagProcessor.getDetections().get(0);
//                if (tag.metadata != null) {
//                    telemetry.addData("id", tag.metadata.id);
//                    telemetry.addData("x", tag.ftcPose.x);
//                    telemetry.addData("y", tag.ftcPose.y);
//                    telemetry.addData("z", tag.ftcPose.z);
//                    telemetry.addData("roll", tag.ftcPose.roll);
//                    telemetry.addData("pitch", tag.ftcPose.pitch);
//                    telemetry.addData("yaw", tag.ftcPose.yaw);
//                }
//            }
//            telemetry.update();
        }
    }

//    @Override
    public void runOpMode2() throws InterruptedException{
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();
        waitForStart();

        while(!isStopRequested() && opModeIsActive()) {
            AprilTagDetection tag;
            if(!tagProcessor.getDetections().isEmpty()) {
                tag = tagProcessor.getDetections().get(0);
                if (tag.metadata != null) {
                    telemetry.addData("id", tag.metadata.id);
                    telemetry.addData("x", tag.ftcPose.x);
                    telemetry.addData("y", tag.ftcPose.y);
                    telemetry.addData("z", tag.ftcPose.z);
                    telemetry.addData("roll", tag.ftcPose.roll);
                    telemetry.addData("pitch", tag.ftcPose.pitch);
                    telemetry.addData("yaw", tag.ftcPose.yaw);
                    if (tag.metadata.id == 23) {
                        telemetry.addLine("This tag is number 23");
                    }
                    telemetry.update();
                }
            }
        }

    }
}