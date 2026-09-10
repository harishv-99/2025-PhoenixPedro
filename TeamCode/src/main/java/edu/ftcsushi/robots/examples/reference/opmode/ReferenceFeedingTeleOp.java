package edu.ftcsushi.robots.examples.reference.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.ftc.input.GamepadDevice;
import edu.ftcsushi.robots.examples.reference.capability.launcher.ReferenceLauncher;
import edu.ftcsushi.robots.examples.reference.capability.launcher.ReferenceLauncherMechanism;
import edu.ftcsushi.robots.examples.reference.control.ReferenceFeedingControls;

/** Disabled and separately locked teaching host; software departure is not proof of a scored shot. */
@TeleOp(name = "FW Reference: Feedback-confirmed Feeding", group = "FW Examples")
@Disabled
public final class ReferenceFeedingTeleOp extends FtcRobotOpMode {
    private static final boolean MOTION_REVIEWED = false;

    @Override
    protected void configure(RobotProgram program) {
        if (!MOTION_REVIEWED) {
            throw new IllegalStateException("Reference feeding motion is locked. Review device "
                    + "identity, staged-sensor placement, directions, servo endpoints, loaded-wheel "
                    + "behavior, interruption/retraction, containment and emergency STOP before "
                    + "setting MOTION_REVIEWED=true and removing @Disabled.");
        }

        ReferenceLauncherMechanism.Config config = ReferenceLauncherMechanism.Config.defaults();
        ReferenceLauncherMechanism launcher = program.output(
                new ReferenceLauncherMechanism(hardwareMap, config));
        ReferenceFeedingControls controls = new ReferenceFeedingControls(new GamepadDevice(gamepad1));
        controls.bind(program.callbackBindings(), program.taskBindings(), launcher);

        program.presenter((clock, telemetry) -> {
            ReferenceLauncher.Status status = launcher.status();
            telemetry.addData("feed.phase", status.phase());
            telemetry.addData("feed.reason", status.reason());
            telemetry.addData("feed.active", status.attemptActive());
            telemetry.addData("feed.recoveryRequired", status.recoveryRequired());
            telemetry.addData("feed.lastAcknowledgement", controls.lastRecoveryResult() == null
                    ? "NOT_REQUESTED" : controls.lastRecoveryResult());
            telemetry.addLine("A: one feed (no backlog) | B: abort | X: acknowledge, no motion");
        });
    }
}
