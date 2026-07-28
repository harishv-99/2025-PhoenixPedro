package edu.ftcphoenix.robots.examples.starter;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

import edu.ftcphoenix.fw.core.lifecycle.CleanupActions;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.MecanumDrivebase;
import edu.ftcphoenix.fw.ftc.FtcDrives;
import edu.ftcphoenix.fw.input.GamepadDevice;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskRunner;

/**
 * Small composition root for the modern starter robot.
 *
 * <p>It owns construction, one clock heartbeat, visible loop order, one telemetry-frame commit,
 * and complete cleanup. Button meanings and final Plant realization stay in their smaller owners.</p>
 */
public final class StarterRobot {

    private enum Mode {
        NONE,
        TELEOP,
        AUTO
    }

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final StarterProfile profile;
    private final LoopClock clock = new LoopClock();

    private Mode mode = Mode.NONE;
    private StarterIntakeMechanism intake;
    private MecanumDrivebase drive;
    private StarterTeleOpControls controls;
    private TaskRunner autoRunner;
    private Task autoRoutine;

    private boolean started;
    private boolean stopped;

    /** Retains FTC boundaries and an independent profile snapshot; hardware is built in init. */
    public StarterRobot(HardwareMap hardwareMap,
                        Telemetry telemetry,
                        StarterProfile profile) {
        this.hardwareMap = Objects.requireNonNull(hardwareMap, "hardwareMap");
        this.telemetry = Objects.requireNonNull(telemetry, "telemetry");
        this.profile = Objects.requireNonNull(profile, "profile").copy();
    }

    /** Builds the direct drive, shared intake, and TeleOp controls exactly once. */
    public void initTeleOp(Gamepad gamepad1) {
        requireCanInitialize();
        try {
            Objects.requireNonNull(gamepad1, "gamepad1");
            profile.requireReadyForTeleOp();
            intake = new StarterIntakeMechanism(hardwareMap, profile.intake);
            drive = FtcDrives.mecanum(hardwareMap, profile.drive);
            controls = new StarterTeleOpControls(
                    new GamepadDevice(gamepad1),
                    intake);
            mode = Mode.TELEOP;
        } catch (RuntimeException failure) {
            throw failStop(failure);
        }
    }

    /** Builds only the shared intake and one simple Auto task runner. */
    public void initAuto() {
        requireCanInitialize();
        try {
            profile.requireReadyForAuto();
            intake = new StarterIntakeMechanism(hardwareMap, profile.intake);
            autoRunner = new TaskRunner();
            mode = Mode.AUTO;
        } catch (RuntimeException failure) {
            throw failStop(failure);
        }
    }

    /** Returns the mode-neutral intake capability after either mode has initialized. */
    public StarterIntake intake() {
        if (intake == null) {
            throw new IllegalStateException("Initialize StarterRobot before requesting intake()");
        }
        return intake;
    }

    /** Installs the one Auto root during INIT. */
    public void installAutoRoutine(Task routine) {
        if (mode != Mode.AUTO || autoRunner == null) {
            throw new IllegalStateException("installAutoRoutine requires initAuto() first");
        }
        if (stopped) {
            throw new IllegalStateException("StarterRobot cannot install a routine after stop()");
        }
        if (started) {
            throw new IllegalStateException("installAutoRoutine must run before start()");
        }
        if (autoRoutine != null) {
            throw new IllegalStateException("StarterRobot already has an Auto routine");
        }
        autoRoutine = Objects.requireNonNull(routine, "routine");
    }

    /** Resets the shared clock and queues the installed Auto root, if this is Auto. */
    public void start(double runtimeSec) {
        if (mode == Mode.NONE) {
            throw new IllegalStateException("Initialize StarterRobot before start()");
        }
        if (stopped) {
            throw new IllegalStateException("StarterRobot cannot start after stop()");
        }
        if (started) {
            throw new IllegalStateException("StarterRobot start() may be called only once");
        }
        try {
            requireFiniteRuntime(runtimeSec);
            if (mode == Mode.AUTO && autoRoutine == null) {
                throw new IllegalStateException("Install an Auto routine before start()");
            }
            clock.reset(runtimeSec);
            if (mode == Mode.AUTO) {
                autoRunner.enqueue(autoRoutine);
            }
            started = true;
        } catch (RuntimeException failure) {
            throw failStop(failure);
        }
    }

    /** Advances the selected mode's complete graph once in its documented order. */
    public void update(double runtimeSec) {
        if (stopped) {
            return;
        }
        try {
            requireActive();
            requireFiniteRuntime(runtimeSec);
            clock.update(runtimeSec);
            if (mode == Mode.TELEOP) {
                controls.update(clock);
                DriveSignal command = controls.driveSource().get(clock).clamped();
                drive.drive(command);
                intake.update(clock);
                emitFrame();
            } else {
                autoRunner.update(clock);
                intake.update(clock);
                emitFrame();
            }
        } catch (RuntimeException failure) {
            throw failStop(failure);
        }
    }

    /** Best-effort cancels behavior and attempts a stopped command on every constructed output. */
    public void stop() {
        if (stopped) {
            return;
        }
        stopped = true;
        started = false;
        CleanupActions.attemptAll(
                () -> {
                    if (autoRunner != null) {
                        autoRunner.cancelAndClear();
                    }
                },
                () -> {
                    if (intake != null) {
                        intake.stop();
                    }
                },
                () -> {
                    if (drive != null) {
                        drive.stop();
                    }
                });
    }

    private void emitFrame() {
        StarterIntake.Status status = intake.status();
        telemetry.addData("intake.mode", status.mode());
        telemetry.addData("intake.appliedTargetPower", status.appliedTargetPower());
        if (mode == Mode.AUTO) {
            telemetry.addData("auto.idle", autoRunner.isIdle());
        }
        telemetry.update();
    }

    private void requireCanInitialize() {
        if (mode != Mode.NONE) {
            throw new IllegalStateException("StarterRobot may initialize only one mode");
        }
        if (stopped) {
            throw new IllegalStateException("StarterRobot cannot initialize after stop()");
        }
    }

    private void requireActive() {
        if (mode == Mode.NONE) {
            throw new IllegalStateException("Initialize StarterRobot before update()");
        }
        if (!started) {
            throw new IllegalStateException("Call start(runtimeSec) before updating StarterRobot");
        }
    }

    private RuntimeException failStop(RuntimeException failure) {
        return CleanupActions.attemptAllAfterFailure(failure, this::stop);
    }

    private static void requireFiniteRuntime(double runtimeSec) {
        if (!Double.isFinite(runtimeSec)) {
            throw new IllegalArgumentException(
                    "runtimeSec must be finite, got " + runtimeSec);
        }
    }
}
