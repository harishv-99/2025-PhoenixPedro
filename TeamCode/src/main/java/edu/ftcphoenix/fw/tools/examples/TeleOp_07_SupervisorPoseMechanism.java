package edu.ftcphoenix.fw.tools.examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.PlantTargetSource;
import edu.ftcphoenix.fw.actuation.PlantTargets;
import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.ftc.FtcActuators;
import edu.ftcphoenix.fw.input.Gamepads;
import edu.ftcphoenix.fw.input.binding.Bindings;
import edu.ftcphoenix.fw.supervisor.HeldValue;
import edu.ftcphoenix.fw.task.OutputTaskRunner;
import edu.ftcphoenix.fw.task.Tasks;

/**
 * <h1>Example 07: Supervisor + Subsystem (Discrete Poses)</h1>
 *
 * <p>This example demonstrates the recommended Phoenix architecture when you have a mechanism that
 * supports a small set of valid positions (poses). It is the stepping stone to the fully
 * explicit three-layer example in <b>Example 09</b>.</p>
 *
 * <p>It shows two ideas:</p>
 * <ul>
 *   <li><b>Pose requests are held state</b>: last request wins and stays active until replaced.</li>
 *   <li><b>Temporary overrides use an OutputTaskRunner</b>: a short pulse can override the base pose without permanently changing the held request.</li>
 * </ul>
 *
 * <h2>Controls</h2>
 * <ul>
 *   <li>P1 A: request INTAKE pose</li>
 *   <li>P1 B: request SCORE pose</li>
 *   <li>P1 Y: request STOW pose</li>
 *   <li>P1 X: pulse OPEN for 0.20s (override)</li>
 * </ul>
 *
 * <p>Hardware assumed:</p>
 * <ul>
 *   <li>One positional servo: {@value #HW_WRIST}</li>
 * </ul>
 */
@TeleOp(name = "FW Ex 07: Supervisor Pose", group = "Framework Examples")
@Disabled
public final class TeleOp_07_SupervisorPoseMechanism extends OpMode {

    private static final String HW_WRIST = "wristServo";

    // Example servo positions (0..1). Tune these for your robot.
    private static final double POS_STOW = 0.10;
    private static final double POS_INTAKE = 0.45;
    private static final double POS_SCORE = 0.80;
    private static final double POS_OPEN = 0.95;

    private final LoopClock clock = new LoopClock();

    private Gamepads gamepads;
    private final Bindings bindings = new Bindings();

    private WristSubsystem wrist;
    private WristSupervisor supervisor;

    /**
     * {@inheritDoc}
     */
    @Override
    public void init() {
        gamepads = Gamepads.create(gamepad1, gamepad2);

        wrist = new WristSubsystem(hardwareMap);
        supervisor = new WristSupervisor(wrist);

        // Pattern A: bindings call supervisor intent methods.
        bindings.onRise(gamepads.p1().a(), supervisor::requestIntake);
        bindings.onRise(gamepads.p1().b(), supervisor::requestScore);
        bindings.onRise(gamepads.p1().y(), supervisor::requestStow);

        // A temporary override: pulse open for 0.20 seconds.
        bindings.onRise(gamepads.p1().x(), supervisor::pulseOpen);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void loop() {
        clock.update(getRuntime());
        bindings.update(clock);

        // In more complex robots, supervisors may have periodic update logic.
        supervisor.update(clock);

        // The subsystem updates the source-driven Plant it owns.
        wrist.update(clock);

        telemetry.addData("wrist.pose", wrist.desiredPose());
        telemetry.addData("wrist.queueActive", wrist.overrideActive(clock));
        telemetry.update();
    }

    // ----------------------------------------------------------------------
    // Subsystem: owns hardware + final Plant target source
    // ----------------------------------------------------------------------

    private static final class WristSubsystem {
        enum Pose {STOW, INTAKE, SCORE}

        private final HeldValue<Pose> desiredPose = new HeldValue<Pose>(Pose.STOW);

        // Optional: temporary overrides (pulses, jogs, etc.)
        private final OutputTaskRunner overrides = Tasks.outputQueue(0.0);
        private final Plant plant;

        WristSubsystem(HardwareMap hardwareMap) {
            PlantTargetSource finalTarget = PlantTargets.overlay(ScalarSource.of(() -> poseTarget(desiredPose.get())))
                    .add("openPulse", overrides.activeSource(), overrides)
                    .build();

            this.plant = FtcActuators.plant(hardwareMap)
                    .servo(HW_WRIST, Direction.FORWARD)
                    .position()
                    .linear()
                    .bounded(0.0, 1.0)
                    .nativeUnits()
                    .targetedBy(finalTarget)
                    .build();
        }

        Pose desiredPose() {
            return desiredPose.get();
        }

        void requestPose(Pose pose) {
            desiredPose.set(pose);
        }

        void enqueueOverride(double target, double seconds) {
            overrides.enqueue(Tasks.outputForSeconds("wristOverride", target, seconds));
        }

        boolean overrideActive(LoopClock clock) {
            return overrides.activeSource().getAsBoolean(clock);
        }

        void update(LoopClock clock) {
            // Update override queue so the overlay sees this loop's pulse output.
            overrides.update(clock);

            // The Plant samples its PlantTargets.overlay(...) source and applies one safe target.
            plant.update(clock);
        }

        private double poseTarget(Pose pose) {
            switch (pose) {
                case STOW:
                    return POS_STOW;
                case INTAKE:
                    return POS_INTAKE;
                case SCORE:
                    return POS_SCORE;
                default:
                    return POS_STOW;
            }
        }
    }

    // ----------------------------------------------------------------------
    // Supervisor: policy + intent translation
    // ----------------------------------------------------------------------

    private static final class WristSupervisor {
        private final WristSubsystem wrist;

        WristSupervisor(WristSubsystem wrist) {
            this.wrist = wrist;
        }

        void requestStow() {
            wrist.requestPose(WristSubsystem.Pose.STOW);
        }

        void requestIntake() {
            wrist.requestPose(WristSubsystem.Pose.INTAKE);
        }

        void requestScore() {
            wrist.requestPose(WristSubsystem.Pose.SCORE);
        }

        void pulseOpen() {
            wrist.enqueueOverride(POS_OPEN, 0.20);
        }

        void update(LoopClock clock) {
            // For this simple mechanism, there is no periodic supervisor logic.
            // More complex supervisors might:
            //  - consume RequestCounters
            //  - manage cooldowns
            //  - enqueue tasks based on sensor signals
        }
    }
}
