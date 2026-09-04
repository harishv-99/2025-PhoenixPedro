package edu.ftcsushi.robots.examples.basicflywheel;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Objects;

import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.ftc.input.GamepadDevice;

/** Disabled flywheel-only host for isolated review of one device-managed velocity Plant. */
@TeleOp(name = "FW Basic 4: Flywheel velocity", group = "FW Examples")
@Disabled
public final class BasicFlywheelTeleOp extends FtcRobotOpMode {
    private static final boolean MOTION_REVIEWED = false;
    private static final double CANDIDATE_VELOCITY_TICKS_PER_SEC = 250.0;

    /** Declares only the flywheel owner, its controls, and its cached status presenter. */
    @Override
    protected void configure(RobotProgram program) {
        if (!MOTION_REVIEWED) {
            throw new IllegalStateException(
                    "Basic flywheel motion is locked. Verify motor identity, direction, restraint, "
                            + "encoder units, candidate range, and emergency stop before setting "
                            + "MOTION_REVIEWED=true and removing @Disabled.");
        }

        BasicFlywheelMechanism.Config config = BasicFlywheelMechanism.Config.defaults();
        requireCandidateInConfiguredRange(config, CANDIDATE_VELOCITY_TICKS_PER_SEC);
        BasicFlywheelMechanism flywheel = program.output(
                new BasicFlywheelMechanism(hardwareMap, config));
        BasicFlywheelControls controls = new BasicFlywheelControls(
                new GamepadDevice(gamepad1), CANDIDATE_VELOCITY_TICKS_PER_SEC);
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

    /** Rejects a candidate that stopped feedback could satisfy or that lies outside the range. */
    static void requireCandidateInConfiguredRange(BasicFlywheelMechanism.Config config,
                                                  double candidateVelocityTicksPerSec) {
        BasicFlywheelMechanism.Config c = Objects.requireNonNull(config, "config");
        if (!Double.isFinite(candidateVelocityTicksPerSec)
                || !Double.isFinite(c.velocityToleranceTicksPerSec)
                || !Double.isFinite(c.maximumVelocityTicksPerSec)
                || candidateVelocityTicksPerSec <= c.velocityToleranceTicksPerSec
                || candidateVelocityTicksPerSec > c.maximumVelocityTicksPerSec) {
            throw new IllegalArgumentException(
                    "CANDIDATE_VELOCITY_TICKS_PER_SEC must be finite, greater than "
                            + "config.velocityToleranceTicksPerSec, and no greater than "
                            + "config.maximumVelocityTicksPerSec; got candidate "
                            + candidateVelocityTicksPerSec + ", tolerance "
                            + c.velocityToleranceTicksPerSec + ", and maximum "
                            + c.maximumVelocityTicksPerSec);
        }
    }
}
