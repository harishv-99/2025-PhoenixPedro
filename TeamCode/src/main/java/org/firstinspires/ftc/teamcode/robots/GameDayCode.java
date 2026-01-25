package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@Disabled
@TeleOp(name = "GameDayCode")
public class GameDayCode extends LinearOpMode {

    private DcMotor slideMotor;
    private DcMotor armRaiserMotor;
    private DcMotor frontleftMotor;
    private DcMotor backleftMotor;
    private DcMotor frontrightMotor;
    private DcMotor backrightMotor;
    private Servo clawservo;
    private CRServo armExtender;

    float errG1LX;
    float errG1LY;
    double y;
    float errG1RX;
    float errG1RY;
    float errG2LX;
    float errG2RY;
    float errG2LY;
    float errG2RX;

    /**
     * Describe this function...
     */
    private void joystickPrintNewPos() {
        telemetry.addData("newPosG1LX", joystickCorrection(gamepad1.left_stick_x, errG1LX));
        telemetry.addData("newPosG1LY", joystickCorrection(gamepad1.left_stick_y, errG1LY));
        telemetry.addData("newPosG1RX", joystickCorrection(gamepad1.right_stick_x, errG1RX));
        telemetry.addData("newPosG1RY", joystickCorrection(gamepad1.right_stick_y, errG1RY));
        telemetry.addData("newPosG2LX", joystickCorrection(gamepad2.left_stick_x, errG2LX));
        telemetry.addData("newPosG2LY", joystickCorrection(gamepad2.left_stick_y, errG2LY));
        telemetry.addData("newPosG2RX", joystickCorrection(gamepad2.right_stick_x, errG2RX));
        telemetry.addData("newPosG2RY", joystickCorrection(gamepad2.right_stick_y, errG2RY));
    }

    /**
     * Describe this function...
     */
    private void joystickPrintPos() {
        telemetry.addData("posG1LX", gamepad1.left_stick_x);
        telemetry.addData("posG1LY", gamepad1.left_stick_y);
        telemetry.addData("posG1RX", gamepad1.right_stick_x);
        telemetry.addData("posG1RY", gamepad1.right_stick_y);
        telemetry.addData("posG2LX", gamepad2.left_stick_x);
        telemetry.addData("posG2LY", gamepad2.left_stick_y);
        telemetry.addData("posG2RX", gamepad2.right_stick_x);
        telemetry.addData("posG2RY", gamepad2.right_stick_y);
    }

    /**
     * Describe this function...
     */
    private void joystickPrintErr() {
        telemetry.addData("errG1LX", errG1LX);
        telemetry.addData("errG1LY", errG1LY);
        telemetry.addData("errG1RX", errG1RX);
        telemetry.addData("errG1RY", errG1RY);
        telemetry.addData("errG2LX", errG2LX);
        telemetry.addData("errG2LY", errG2LY);
        telemetry.addData("errG2RX", errG2RX);
        telemetry.addData("errG2RY", errG2RY);
    }

    /**
     * Describe this function...
     */
    private void joystickErr() {
        errG1LX = gamepad1.left_stick_x;
        errG1LY = gamepad1.left_stick_y;
        errG1RX = gamepad1.right_stick_x;
        errG1RY = gamepad1.right_stick_y;
        errG2LX = gamepad2.left_stick_x;
        errG2LY = gamepad2.left_stick_y;
        errG2RX = gamepad2.right_stick_x;
        errG2RY = gamepad2.right_stick_y;
    }

    /**
     * Describe this function...
     */
    private double joystickCorrection(float pos, double err) {
        double newpos;

        // First subtract the error from the position, then stretch or shrink
        if (pos >= err) {
            newpos = (pos - err) / (1 - err);
        }
        else {
            newpos = (pos - err) / (1 + err);
        }
        return newpos;
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
//        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
//        armRaiserMotor = hardwareMap.get(DcMotor.class, "armRaiserMotor");
//        frontleftMotor = hardwareMap.get(DcMotor.class, "frontleftMotor");
//        backleftMotor = hardwareMap.get(DcMotor.class, "backleftMotor");
//        frontrightMotor = hardwareMap.get(DcMotor.class, "frontrightMotor");
//        backrightMotor = hardwareMap.get(DcMotor.class, "backrightMotor");
//        clawservo = hardwareMap.get(Servo.class, "clawservo");
//        armExtender = hardwareMap.get(CRServo.class, "armExtender");
//
//        // Reverse the right side motors.  This may be wrong for your setup.
//        // If your robot moves backwards when commanded to go forwards, reverse the left side instead.
//        slideMotor.setDirection(DcMotor.Direction.REVERSE);
//        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        armRaiserMotor.setDirection(DcMotor.Direction.REVERSE);
//        armRaiserMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontleftMotor.setDirection(DcMotor.Direction.REVERSE);
//        backleftMotor.setDirection(DcMotor.Direction.REVERSE);
//        frontrightMotor.setDirection(DcMotor.Direction.FORWARD);
//        backrightMotor.setDirection(DcMotor.Direction.REVERSE);
        joystickErr();
        joystickPrintErr();
        waitForStart();
        while (opModeIsActive()) {
            if (false) {
                moveIntake();
                moveRobot();
                moveSlides();
                armExtenderServo();
                armRaiser();
            } else {
                joystickTest();
            }
            telemetry.update();
        }
    }

    /**
     * Describe this function...
     */
    private void moveIntake() {
        if (gamepad2.y) {
            telemetry.addLine("Intake Close");
            clawservo.setPosition(0);
        } else if (gamepad2.a) {
            telemetry.addLine("Intake Open");
            clawservo.setPosition(1);
        }
    }

    /**
     * Describe this function...
     */
    private void armExtenderServo() {
        armExtender.setDirection(CRServo.Direction.REVERSE);
        if (gamepad2.dpad_up) {
            armExtender.setPower(1);
        } else if (gamepad2.dpad_down) {
            armExtender.setPower(-1);
        } else {
            armExtender.setPower(0);
        }
    }

    /**
     * Describe this function...
     */
    private void joystickTest() {
        joystickPrintErr();
        joystickPrintPos();
        joystickPrintNewPos();
    }

    /**
     * Describe this function...
     */
    private void armRaiser() {
        double armExtenderUp;

        armExtenderUp = -joystickCorrection(gamepad2.right_stick_y, errG2RY) * 50;
        telemetry.addData("Target", armRaiserMotor.getCurrentPosition() + armExtenderUp);
        telemetry.addData("armExtenderPosChange", armExtenderUp);
        if (armExtenderUp < 0 && armRaiserMotor.getCurrentPosition() > 0) {
            armRaiserMotor.setTargetPosition((int) (armRaiserMotor.getCurrentPosition() + armExtenderUp));
            telemetry.addLine("Going down");
        } else if (armExtenderUp > 0 && armRaiserMotor.getCurrentPosition() < 1500) {
            armRaiserMotor.setTargetPosition((int) (armRaiserMotor.getCurrentPosition() + armExtenderUp));
            telemetry.addLine("Going up");
        } else if (gamepad2.b) {
            armRaiserMotor.setTargetPosition((int) (armRaiserMotor.getCurrentPosition() + armExtenderUp));
            telemetry.addLine("Override");
        } else {
            armRaiserMotor.setTargetPosition(armRaiserMotor.getCurrentPosition() + 0);
        }
        armRaiserMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRaiserMotor.setPower(0.8);
    }

    /**
     * Describe this function...
     */
    private void moveSlides() {
        // Use gamepad_and will press_and _to move the slides up and down, height may vary
        // Remember, Y stick value is reversed
        telemetry.addData("SlidePos", slideMotor.getCurrentPosition());
        y = -(joystickCorrection(gamepad2.left_stick_y, errG2LY) * 1);
        telemetry.addData("SlidePower", y);
        if (y < 0 && slideMotor.getCurrentPosition() > 0) {
            slideMotor.setPower(y);
            telemetry.addLine("Going down");
        } else if (y > 0 && slideMotor.getCurrentPosition() < 27000) {
            slideMotor.setPower(y);
            telemetry.addLine("Going up");
        } else if (gamepad2.b) {
            slideMotor.setPower(y);
            telemetry.addLine("Override");
        } else {
            slideMotor.setPower(0);
        }
    }

    /**
     * Describe this function...
     */
    private void moveRobot() {
        double x;
        double rx;
        double denominator;

        // Remember, Y stick value is reversed
        y = -joystickCorrection(gamepad1.left_stick_y, errG1LY) * 0.5;
        // Factor to counteract imperfect strafing
        x = joystickCorrection(gamepad1.left_stick_x, errG1LX) * 0.5;
        rx = joystickCorrection(gamepad1.right_stick_x, errG1RX) * 0.5;
        telemetry.addData("y", y);
        telemetry.addData("x", x);
        telemetry.addData("rx", rx);
        // Denominator is the largest motor power (absolute value) or 1.
        // This ensures all powers maintain the same ratio, but only if one is outside of the range [-1, 1].
        denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(y), Math.abs(x), Math.abs(rx))), 1));
        // Make sure your ID's match your configuration
        frontleftMotor.setPower((y + x + rx) / denominator);
        backleftMotor.setPower(((y - x) + rx) / denominator);
        frontrightMotor.setPower(((y - x) - rx) / denominator);
        backrightMotor.setPower(((y + x) - rx) / denominator);
    }
}

