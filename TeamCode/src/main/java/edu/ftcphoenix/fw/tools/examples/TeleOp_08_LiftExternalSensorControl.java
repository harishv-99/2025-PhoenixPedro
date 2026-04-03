package edu.ftcphoenix.fw.tools.examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcphoenix.fw.actuation.Actuators;
import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.core.control.Pid;
import edu.ftcphoenix.fw.core.control.ScalarControllers;
import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.ftc.FtcSensors;
import edu.ftcphoenix.fw.input.Gamepads;
import edu.ftcphoenix.fw.input.binding.Bindings;

/**
 * <h1>Example 08: External Sensor + Scalar Regulation</h1>
 *
 * <p>This example shows Phoenix lane 2: a single measured variable, a controller, and a plant.</p>
 */
@TeleOp(name = "FW Ex 08: Lift External Sensor", group = "Framework Examples")
@Disabled
public final class TeleOp_08_LiftExternalSensorControl extends OpMode {

    private static final String HW_LIFT_MOTOR = "liftMotor";
    private static final String HW_LIFT_POT = "liftPot";

    private static final double HEIGHT_LOW_IN = 2.0;
    private static final double HEIGHT_MID_IN = 8.0;
    private static final double HEIGHT_HIGH_IN = 14.0;

    private static final double SENSOR_MIN_V = 0.35;
    private static final double SENSOR_MAX_V = 3.05;
    private static final double HEIGHT_MIN_IN = 0.0;
    private static final double HEIGHT_MAX_IN = 15.0;

    private final LoopClock clock = new LoopClock();
    private final Bindings bindings = new Bindings();

    private Gamepads gamepads;
    private Plant liftPowerPlant;

    private ScalarSource liftHeightIn;
    private ScalarSource desiredHeightIn;
    private ScalarSource liftPowerCmd;

    private double desiredHeight = HEIGHT_LOW_IN;

    @Override
    public void init() {
        gamepads = Gamepads.create(gamepad1, gamepad2);

        liftPowerPlant = Actuators.plant(hardwareMap)
                .motor(HW_LIFT_MOTOR, Direction.FORWARD)
                .power()
                .build();

        ScalarSource potVoltage = FtcSensors.analogVoltage(hardwareMap, HW_LIFT_POT);

        liftHeightIn = new ScalarSource() {
            @Override
            public double getAsDouble(LoopClock clock) {
                double volts = potVoltage.getAsDouble(clock);
                double t = (volts - SENSOR_MIN_V) / (SENSOR_MAX_V - SENSOR_MIN_V);
                t = Math.max(0.0, Math.min(1.0, t));
                return HEIGHT_MIN_IN + t * (HEIGHT_MAX_IN - HEIGHT_MIN_IN);
            }
        };

        desiredHeightIn = new ScalarSource() {
            @Override
            public double getAsDouble(LoopClock clock) {
                return desiredHeight;
            }
        };

        Pid liftPid = Pid.withGains(0.12, 0.0, 0.0)
                .setOutputLimits(-0.55, 0.55);

        liftPowerCmd = ScalarControllers.pid(desiredHeightIn, liftHeightIn, liftPid);

        bindings.onRise(gamepads.p1().a(), () -> desiredHeight = HEIGHT_LOW_IN);
        bindings.onRise(gamepads.p1().b(), () -> desiredHeight = HEIGHT_MID_IN);
        bindings.onRise(gamepads.p1().y(), () -> desiredHeight = HEIGHT_HIGH_IN);
    }

    @Override
    public void loop() {
        clock.update(getRuntime());
        bindings.update(clock);

        double cmd = liftPowerCmd.getAsDouble(clock);
        liftPowerPlant.setTarget(cmd);
        liftPowerPlant.update(clock.dtSec());

        telemetry.addData("lift.targetIn", desiredHeightIn.getAsDouble(clock));
        telemetry.addData("lift.heightIn", liftHeightIn.getAsDouble(clock));
        telemetry.addData("lift.powerCmd", cmd);
        telemetry.update();
    }

    @Override
    public void stop() {
        liftPowerPlant.stop();
        liftPowerCmd.reset();
    }
}
