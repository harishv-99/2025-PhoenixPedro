package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "phoenix3")
public class Phoenix3 extends LinearOpMode {
    private DcMotor frontright;
    private DcMotor backright;
    private DcMotor frontleft;
    private DcMotor backleft;
    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;
    private Servo pusher;
    private CRServo transferLeft;
    private CRServo transferRight;
    float errG1LX;
    float errG1LY;
    double y;
    float errG1RX;
    float errG1RY;
    float errG2LX;
    float errG2RY;
    float errG2LY;
    float errG2RX;
    double SHOOTER_LEFT_TARGET_VELOCITY;
    double SHOOTER_RIGHT_TARGET_VELOCITY;
    double velocity = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        frontleft = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backleft = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontright = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backright = hardwareMap.get(DcMotor.class, "backRightMotor");
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        pusher = hardwareMap.get(Servo.class, "pusher");
        transferLeft = hardwareMap.get(CRServo.class, "transferLeft");
        transferRight = hardwareMap.get(CRServo.class, "transferRight");

        frontleft.setDirection(DcMotor.Direction.FORWARD);
        backleft.setDirection(DcMotor.Direction.FORWARD);
        frontright.setDirection(DcMotor.Direction.REVERSE);
        backright.setDirection(DcMotor.Direction.FORWARD);
    }
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
    private double joystickCorrection(float pos, double err) {
        double newpos;

        // First subtract the error from the position, then stretch or shrink
        if (pos >= err) {
            newpos = (pos - err) / (1 - err);
        } else {
            newpos = (pos - err) / (1 + err);
        }
        return newpos;
    }
    public void moveRobot() {
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
        frontleft.setPower((y + x + rx) / denominator);
        backleft.setPower(((y - x) + rx) / denominator);
        frontright.setPower(((y - x) - rx) / denominator);
        backright.setPower(((y + x) - rx) / denominator);

    }
    public void shoot() {
        if(gamepad2.dpadUpWasPressed()) {
            velocity = velocity + 200;
            shooterLeft.setVelocity(velocity);
            shooterRight.setVelocity(velocity);
        }
        else if(gamepad2.dpadDownWasPressed()) {
            velocity = velocity - 200;
            shooterLeft.setVelocity(velocity);
            shooterRight.setVelocity(velocity);
        }
    }
}
