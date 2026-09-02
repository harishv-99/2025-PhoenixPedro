package edu.ftcsushi.robots.examples.reference.robot;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

import edu.ftcsushi.fw.ftc.FtcDrives;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.ftc.input.GamepadDevice;
import edu.ftcsushi.robots.examples.reference.capability.launcher.ReferenceLauncher;
import edu.ftcsushi.robots.examples.reference.capability.launcher.ReferenceLauncherMechanism;
import edu.ftcsushi.robots.examples.reference.capability.lift.ReferenceLift;
import edu.ftcsushi.robots.examples.reference.capability.lift.ReferenceLiftMechanism;

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
        Gamepad requiredGamepad = Objects.requireNonNull(gamepad1, "gamepad1");
        requireAllowed("allowDriveMotion", p.allowDriveMotion);
        requireAllowed("allowLiftMotion", p.allowLiftMotion);
        requireAllowed("allowLauncherMotion", p.allowLauncherMotion);
        requireDistinctMotorOwners(p, true);

        ReferenceLiftMechanism lift = program.output(
                new ReferenceLiftMechanism(hardwareMap, p.lift));
        ReferenceLauncherMechanism launcher = program.output(
                new ReferenceLauncherMechanism(hardwareMap, p.launcher));
        ReferenceTeleOpControls controls = new ReferenceTeleOpControls(
                new GamepadDevice(requiredGamepad));
        controls.bind(program.callbackBindings(), program.taskBindings(), lift, launcher);
        program.drive(controls.driveSource(), FtcDrives.mecanum(hardwareMap, p.drive));
        program.presenter((clock, telemetry) -> present(telemetry, lift, launcher));
    }

    /**
     * Declares Auto's active owners and returns their mode-neutral capability vocabulary.
     *
     * @return capability handoff for the Auto client's selected routine
     */
    public ReferenceCapabilities declareAuto(RobotProgram program, ReferenceProfile profile) {
        Objects.requireNonNull(program, "program");
        ReferenceProfile p = Objects.requireNonNull(profile, "profile");
        requireAllowed("allowLiftMotion", p.allowLiftMotion);
        requireAllowed("allowLauncherMotion", p.allowLauncherMotion);
        requireDistinctMotorOwners(p, false);
        ReferenceLiftMechanism lift = program.output(
                new ReferenceLiftMechanism(hardwareMap, p.lift));
        ReferenceLauncherMechanism launcher = program.output(
                new ReferenceLauncherMechanism(hardwareMap, p.launcher));
        program.presenter((clock, telemetry) -> present(telemetry, lift, launcher));
        return new ReferenceCapabilities(lift, launcher);
    }

    private static void present(Telemetry telemetry,
                                ReferenceLift lift,
                                ReferenceLauncher launcher) {
        ReferenceLift.Status liftStatus = lift.status();
        ReferenceLauncher.Status launcherStatus = launcher.status();
        telemetry.addData("lift", "%s %.2f/%.2f in referenced=%s",
                liftStatus.requestedHeight(), liftStatus.measuredPositionIn(),
                liftStatus.requestedPositionIn(), liftStatus.referenced());
        telemetry.addData("launcher",
                "target=%.0f ticks/sec left=%.0f right=%.0f ready=%s object=%s",
                launcherStatus.targetVelocityTicksPerSec,
                launcherStatus.leftMeasuredVelocityTicksPerSec,
                launcherStatus.rightMeasuredVelocityTicksPerSec,
                launcherStatus.ready, launcherStatus.objectPresent);
    }

    private static void requireDistinctMotorOwners(ReferenceProfile profile,
                                                   boolean includeDrive) {
        String[] paths = includeDrive
                ? new String[]{
                        "ReferenceProfile.drive.wiring.frontLeftName",
                        "ReferenceProfile.drive.wiring.frontRightName",
                        "ReferenceProfile.drive.wiring.backLeftName",
                        "ReferenceProfile.drive.wiring.backRightName",
                        "ReferenceProfile.lift.motorName",
                        "ReferenceProfile.launcher.leftFlywheelName",
                        "ReferenceProfile.launcher.rightFlywheelName"
                }
                : new String[]{
                        "ReferenceProfile.lift.motorName",
                        "ReferenceProfile.launcher.leftFlywheelName",
                        "ReferenceProfile.launcher.rightFlywheelName"
                };
        String[] names = includeDrive
                ? new String[]{
                        driveName(profile, 0),
                        driveName(profile, 1),
                        driveName(profile, 2),
                        driveName(profile, 3),
                        liftName(profile),
                        leftFlywheelName(profile),
                        rightFlywheelName(profile)
                }
                : new String[]{
                        liftName(profile),
                        leftFlywheelName(profile),
                        rightFlywheelName(profile)
                };

        for (int second = 0; second < names.length; second++) {
            if (isBlank(names[second])) continue;
            String secondKey = names[second].trim();
            for (int first = 0; first < second; first++) {
                if (isBlank(names[first])) continue;
                String firstKey = names[first].trim();
                if (firstKey.equals(secondKey)) {
                    String mode = includeDrive ? "TeleOp" : "Auto";
                    throw new IllegalStateException(
                            "Reference " + mode + " motor ownership collision: "
                                    + paths[first] + " and " + paths[second]
                                    + " both resolve to FTC hardware key \"" + firstKey
                                    + "\". Configure distinct motor names.");
                }
            }
        }
    }

    private static String driveName(ReferenceProfile profile, int index) {
        if (profile.drive == null || profile.drive.wiring == null) return null;
        switch (index) {
            case 0:
                return profile.drive.wiring.frontLeftName;
            case 1:
                return profile.drive.wiring.frontRightName;
            case 2:
                return profile.drive.wiring.backLeftName;
            case 3:
                return profile.drive.wiring.backRightName;
            default:
                throw new AssertionError("unsupported drive motor index " + index);
        }
    }

    private static String liftName(ReferenceProfile profile) {
        return profile.lift == null ? null : profile.lift.motorName;
    }

    private static String leftFlywheelName(ReferenceProfile profile) {
        return profile.launcher == null ? null : profile.launcher.leftFlywheelName;
    }

    private static String rightFlywheelName(ReferenceProfile profile) {
        return profile.launcher == null ? null : profile.launcher.rightFlywheelName;
    }

    private static boolean isBlank(String value) {
        return value == null || value.trim().isEmpty();
    }

    private static void requireAllowed(String field, boolean allowed) {
        if (!allowed) {
            throw new IllegalStateException(
                    "ReferenceProfile." + field + " must be true only after the adopting team "
                            + "reviews its physical configuration and supervised-motion criteria");
        }
    }
}
