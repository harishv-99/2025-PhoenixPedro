package edu.ftcphoenix.robots.examples.reference.robot;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

import edu.ftcphoenix.fw.ftc.FtcDrives;
import edu.ftcphoenix.fw.ftc.RobotProgram;
import edu.ftcphoenix.fw.ftc.input.GamepadDevice;
import edu.ftcphoenix.fw.task.Tasks;
import edu.ftcphoenix.robots.examples.reference.capability.launcher.ReferenceLauncher;
import edu.ftcphoenix.robots.examples.reference.capability.launcher.ReferenceLauncherMechanism;
import edu.ftcphoenix.robots.examples.reference.capability.lift.ReferenceLift;
import edu.ftcphoenix.robots.examples.reference.capability.lift.ReferenceLiftMechanism;

/** Composition root for the season-independent mechanism reference. */
public final class ReferenceRobot {
    private final HardwareMap hardwareMap;

    public ReferenceRobot(HardwareMap hardwareMap) {
        this.hardwareMap = Objects.requireNonNull(hardwareMap, "hardwareMap");
    }

    /** Declares robot-centric drive, semantic controls, owned mechanisms, and concise evidence. */
    public void declareTeleOp(RobotProgram program, ReferenceProfile profile, Gamepad gamepad1) {
        Objects.requireNonNull(program, "program");
        ReferenceProfile p = Objects.requireNonNull(profile, "profile");
        requireAllowed("allowDriveMotion", p.allowDriveMotion);
        requireAllowed("allowLiftMotion", p.allowLiftMotion);
        requireAllowed("allowLauncherMotion", p.allowLauncherMotion);

        ReferenceLiftMechanism lift = program.output(
                new ReferenceLiftMechanism(hardwareMap, p.lift));
        ReferenceLauncherMechanism launcher = program.output(
                new ReferenceLauncherMechanism(hardwareMap, p.launcher));
        ReferenceTeleOpControls controls = new ReferenceTeleOpControls(
                new GamepadDevice(Objects.requireNonNull(gamepad1, "gamepad1")));
        controls.bind(program.callbackBindings(), program.taskBindings(), lift, launcher);
        program.drive(controls.driveSource(), FtcDrives.mecanum(hardwareMap, p.drive));
        program.presenter((clock, telemetry) -> present(telemetry, lift, launcher));
    }

    /** Declares a tiny Auto using the same capability vocabulary as TeleOp. */
    public void declareAuto(RobotProgram program, ReferenceProfile profile) {
        Objects.requireNonNull(program, "program");
        ReferenceProfile p = Objects.requireNonNull(profile, "profile");
        requireAllowed("allowLiftMotion", p.allowLiftMotion);
        requireAllowed("allowLauncherMotion", p.allowLauncherMotion);
        ReferenceLiftMechanism lift = program.output(
                new ReferenceLiftMechanism(hardwareMap, p.lift));
        ReferenceLauncherMechanism launcher = program.output(
                new ReferenceLauncherMechanism(hardwareMap, p.launcher));
        program.rootTask(Tasks.branchOnOutcome(
                lift.home(),
                Tasks.sequence(
                Tasks.runOnce(() -> lift.setHeight(ReferenceLift.Height.LOW)),
                launcher.launchOne()),
                Tasks.runOnce(launcher::idle)));
        program.presenter((clock, telemetry) -> present(telemetry, lift, launcher));
    }

    private static void present(Telemetry telemetry,
                                ReferenceLift lift,
                                ReferenceLauncher launcher) {
        ReferenceLift.Status liftStatus = lift.status();
        ReferenceLauncher.Status launcherStatus = launcher.status();
        telemetry.addData("lift", "%s %.2f/%.2f in referenced=%s",
                liftStatus.requestedHeight, liftStatus.measuredPositionIn,
                liftStatus.requestedPositionIn, liftStatus.referenced);
        telemetry.addData("launcher", "%.0f/%.0f ready=%s object=%s",
                launcherStatus.measuredVelocity, launcherStatus.targetVelocity,
                launcherStatus.ready, launcherStatus.objectPresent);
    }

    private static void requireAllowed(String field, boolean allowed) {
        if (!allowed) {
            throw new IllegalStateException(
                    "ReferenceProfile." + field + " must be true only after the adopting team "
                            + "reviews its physical configuration and supervised-motion criteria");
        }
    }
}
