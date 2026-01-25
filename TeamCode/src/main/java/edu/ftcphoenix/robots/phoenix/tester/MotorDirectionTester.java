package edu.ftcphoenix.robots.phoenix.tester;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.ftcphoenix.fw.actuation.Actuators;
import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.PlantTasks;
import edu.ftcphoenix.fw.input.Gamepads;
import edu.ftcphoenix.fw.input.binding.Bindings;
import edu.ftcphoenix.fw.task.TaskRunner;
import edu.ftcphoenix.fw.util.LoopClock;
import edu.ftcphoenix.robots.phoenix.RobotConfig;

/**
 * Central robot class for Phoenix-based robots.
 *
 * <p>Beginners should mostly edit <b>this file</b>. TeleOp and Auto OpModes are
 * kept very thin and simply delegate into the methods here.</p>
 *
 * <h2>Responsibilities</h2>
 * <ul>
 *   <li>Wire all hardware once (drive, intake, transfer, shooter, pusher, vision).</li>
 *   <li>Define gamepad mappings in one place via {@link Bindings}.</li>
 *   <li>Own shared logic: auto-aim, shooter velocity, macros, autos.</li>
 *   <li>Expose simple entry points for TeleOp and Auto.</li>
 * </ul>
 */
public final class MotorDirectionTester {
    private final LoopClock clock = new LoopClock();
    private final HardwareMap hw;
    private final Telemetry telemetry;
    private final Gamepads gamepads;
    private final Bindings bindings = new Bindings();
    private final TaskRunner taskRunner = new TaskRunner();

    private Plant plantFL;
    private Plant plantFR;
    private Plant plantBL;
    private Plant plantBR;

    public MotorDirectionTester(HardwareMap hw, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        this.hw = hw;
        this.gamepads = Gamepads.create(gamepad1, gamepad2);
        this.telemetry = telemetry;
    }

    public void initTeleOp() {
        plantFL = Actuators.plant(hw)
                .motor(RobotConfig.DriveTrain.nameMotorFrontLeft,
                        RobotConfig.DriveTrain.invertMotorFrontLeft)
                .power()
                .build();
        plantFR = Actuators.plant(hw)
                .motor(RobotConfig.DriveTrain.nameMotorFrontRight,
                        RobotConfig.DriveTrain.invertMotorFrontRight)
                .power()
                .build();
        plantBL = Actuators.plant(hw)
                .motor(RobotConfig.DriveTrain.nameMotorBackLeft,
                        RobotConfig.DriveTrain.invertMotorBackLeft)
                .power()
                .build();
        plantBR = Actuators.plant(hw)
                .motor(RobotConfig.DriveTrain.nameMotorBackRight,
                        RobotConfig.DriveTrain.invertMotorBackRight)
                .power()
                .build();

        // Create bindings
        bindings.whileHeld(gamepads.p1().x(),
                () -> taskRunner.enqueue(PlantTasks.setInstant(plantFL, 0.5)),
                () -> taskRunner.enqueue(PlantTasks.setInstant(plantFL, 0)));
        bindings.whileHeld(gamepads.p1().y(),
                () -> taskRunner.enqueue(PlantTasks.setInstant(plantFR, 0.5)),
                () -> taskRunner.enqueue(PlantTasks.setInstant(plantFR, 0)));
        bindings.whileHeld(gamepads.p1().a(),
                () -> taskRunner.enqueue(PlantTasks.setInstant(plantBL, 0.5)),
                () -> taskRunner.enqueue(PlantTasks.setInstant(plantBL, 0)));
        bindings.whileHeld(gamepads.p1().b(),
                () -> taskRunner.enqueue(PlantTasks.setInstant(plantBR, 0.5)),
                () -> taskRunner.enqueue(PlantTasks.setInstant(plantBR, 0)));
    }

    public void startAny(double runtime) {
        // Initialize loop timing.
        clock.reset(runtime);
    }

    public void startTeleOp() {
    }

    public void updateAny(double runtime) {
        // --- 1) Clock ---
        clock.update(runtime);
    }

    public void updateTeleOp() {
        // --- 2) Inputs + bindings ---
        gamepads.update(clock.dtSec());
        bindings.update(clock.dtSec());

        // --- 3) Macros ---
        taskRunner.update(clock);

        // When no macro is active, hold a safe default state.
        if (!taskRunner.hasActiveTask()) {
            plantFL.setTarget(0.0);
            plantFR.setTarget(0.0);
            plantBL.setTarget(0.0);
            plantBR.setTarget(0.0);
        }

        // --- 4) Mechanism updates ---
        plantFL.update(clock.dtSec());
        plantFR.update(clock.dtSec());
        plantBL.update(clock.dtSec());
        plantBR.update(clock.dtSec());

        // --- 5) Telemetry / debug ---
        telemetry.addLine("Test motor direction; should move forward");
        telemetry.addLine("   X for front-left");
        telemetry.addLine("   Y for front-right");
        telemetry.addLine("   A for back-left");
        telemetry.addLine("   B for back-right");
        telemetry.update();
    }

    public void stopAny() {
    }

    public void stopTeleOp() {
    }
}
