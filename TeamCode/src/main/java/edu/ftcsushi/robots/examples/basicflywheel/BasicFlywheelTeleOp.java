package edu.ftcsushi.robots.examples.basicflywheel;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.ftc.input.GamepadDevice;

/** Disabled flywheel-only host for isolated review of one device-managed velocity Plant. */
@TeleOp(name = "FW Basic: Flywheel velocity", group = "FW Examples")
@Disabled
public final class BasicFlywheelTeleOp extends FtcRobotOpMode {

    /** Declares only the flywheel owner, its controls, and its cached status presenter. */
    @Override
    protected void configure(RobotProgram program) {
        BasicFlywheelProfile profile = BasicFlywheelProfile.current();
        BasicFlywheelProfile.requireMotionAllowed(profile, "Basic Flywheel TeleOp");
        BasicFlywheelProfile.requireCandidateInConfiguredRange(
                profile.flywheel, profile.candidateVelocityTicksPerSec);

        BasicFlywheelMechanism flywheel = program.output(
                new BasicFlywheelMechanism(hardwareMap, profile.flywheel));
        BasicFlywheelControls controls = new BasicFlywheelControls(
                new GamepadDevice(gamepad1), profile.candidateVelocityTicksPerSec);
        controls.bind(program.callbackBindings(), flywheel);

        program.presenter((clock, telemetry) -> {
            BasicFlywheel.Status status = flywheel.status();
            telemetry.addData("flywheel.requestedTicksPerSec",
                    status.requestedVelocityTicksPerSec());
            telemetry.addData("flywheel.appliedTicksPerSec",
                    status.appliedVelocityTicksPerSec());
            telemetry.addData("flywheel.measuredTicksPerSec",
                    status.measuredVelocityTicksPerSec());
            telemetry.addData("flywheel.atRequestedVelocity",
                    status.atRequestedVelocity());
            telemetry.addLine("A: request candidate | B: request zero");
        });
    }
}
