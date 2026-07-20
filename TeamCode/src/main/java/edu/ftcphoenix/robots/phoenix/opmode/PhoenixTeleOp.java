package edu.ftcphoenix.robots.phoenix.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

import edu.ftcphoenix.robots.phoenix.PhoenixMatchHandoff;
import edu.ftcphoenix.robots.phoenix.PhoenixRobot;

/**
 * Main Phoenix TeleOp Driver Station entry.
 *
 * <p>The OpMode is intentionally thin: FTC lifecycle calls are forwarded to {@link PhoenixRobot},
 * while all gamepad meanings live in Phoenix's TeleOp controls object and all subsystem behavior
 * lives inside the robot container. After ordinary localization initialization, one fresh final
 * Auto pose may be consumed and applied before START; otherwise the normally initialized TeleOp
 * pose remains active.</p>
 */
@TeleOp(name = "Phoenix TeleOp", group = "Phoenix")
public final class PhoenixTeleOp extends OpMode {
    private PhoenixRobot robot;
    private PhoenixMatchHandoff.RestoreResult autoHandoffRestore;

    /**
     * Build the Phoenix TeleOp runtime.
     */
    @Override
    public void init() {
        PhoenixRobot builtRobot =
                new PhoenixRobot(hardwareMap, telemetry, gamepad1, gamepad2);
        try {
            builtRobot.initAny();
            builtRobot.initTeleOp();
        } catch (RuntimeException initializationFailure) {
            throw stopAfterFailure(initializationFailure, builtRobot);
        }

        autoHandoffRestore = restoreInitializedRobotOrStop(this, builtRobot);
        try {
            publishAutoHandoffTelemetry(telemetry, autoHandoffRestore);
        } catch (RuntimeException telemetryFailure) {
            throw stopAfterFailure(telemetryFailure, builtRobot);
        }
        robot = builtRobot;
    }

    /**
     * Reset shared loop timing and start TeleOp-specific state.
     */
    @Override
    public void start() {
        robot.startAny(getRuntime());
        robot.startTeleOp();
    }

    /**
     * Advance one TeleOp loop.
     */
    @Override
    public void loop() {
        robot.updateAny(getRuntime());
        robot.updateTeleOp();
    }

    /**
     * Stop TeleOp-specific and shared Phoenix resources.
     */
    @Override
    public void stop() {
        PhoenixRobot runtime = robot;
        robot = null;
        if (runtime != null) {
            runtime.stop();
        }
    }

    /**
     * Publishes the one-time handoff result while the Driver Station is still in INIT.
     *
     * <p>Package visibility keeps this mode-boundary presentation directly host-testable without
     * adding another public Phoenix API.</p>
     */
    static void publishAutoHandoffTelemetry(
            Telemetry telemetry,
            PhoenixMatchHandoff.RestoreResult restoreResult
    ) {
        Telemetry requiredTelemetry = Objects.requireNonNull(
                telemetry,
                "Phoenix TeleOp handoff telemetry is required"
        );
        PhoenixMatchHandoff.RestoreResult requiredResult = Objects.requireNonNull(
                restoreResult,
                "Phoenix TeleOp handoff restore result is required"
        );
        requiredTelemetry.addData("teleop.autoHandoff", requiredResult);
        if (requiredResult != PhoenixMatchHandoff.RestoreResult.RESTORED) {
            requiredTelemetry.addLine(
                    "Auto handoff was not restored; using the normally initialized TeleOp pose."
            );
        }
        // initTeleOp() staged ordinary controls/readiness first. Commit one combined INIT frame so
        // the driver sees both that guidance and this mode-boundary result before START.
        requiredTelemetry.update();
    }

    /**
     * Restore one already-initialized TeleOp runtime and fail-stop it if pose application fails.
     *
     * <p>Package visibility keeps the failure-ordering seam directly testable without creating a
     * second public Phoenix lifecycle API.</p>
     */
    static PhoenixMatchHandoff.RestoreResult restoreInitializedRobotOrStop(
            OpMode consumer,
            PhoenixRobot initializedRobot
    ) {
        try {
            return PhoenixMatchHandoff.restoreForTeleOp(consumer, initializedRobot);
        } catch (RuntimeException restoreFailure) {
            throw stopAfterFailure(restoreFailure, initializedRobot);
        }
    }

    private static RuntimeException stopAfterFailure(
            RuntimeException primary,
            PhoenixRobot initializedRobot
    ) {
        try {
            initializedRobot.stop();
        } catch (RuntimeException cleanupFailure) {
            if (cleanupFailure != primary) {
                primary.addSuppressed(cleanupFailure);
            }
        }
        return primary;
    }
}
