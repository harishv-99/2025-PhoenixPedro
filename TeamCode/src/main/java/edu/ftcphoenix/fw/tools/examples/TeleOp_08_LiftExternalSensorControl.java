package edu.ftcphoenix.fw.tools.examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcphoenix.fw.actuation.PositionPlant;
import edu.ftcphoenix.fw.core.control.Pid;
import edu.ftcphoenix.fw.core.control.ScalarRegulators;
import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.ftc.FtcActuators;
import edu.ftcphoenix.fw.ftc.FtcSensors;
import edu.ftcphoenix.fw.input.Gamepads;
import edu.ftcphoenix.fw.input.binding.Bindings;
import edu.ftcphoenix.fw.supervisor.HeldValue;

/**
 * <h1>Example 08: Lift with Held Height Selection + External Sensor Feedback</h1>
 *
 * <p>This example shows a simple layered mechanism without a lot of ceremony:</p>
 * <ul>
 *   <li><b>Held selection</b>: A/B/Y choose a desired lift height that stays active until replaced.</li>
 *   <li><b>Trivial execution</b>: the example simply says “hold the selected height.”</li>
 *   <li><b>Plant-owned realization</b>: a regulated position plant closes the loop using an external analog sensor.</li>
 * </ul>
 *
 * <p>It is intentionally simple. The point is to show that the framework's layering philosophy
 * still applies even when the middle execution step is almost a pass-through. If you want
 * the same philosophy written with explicit <b>Requests → Behavior → Realization</b> classes,
 * continue to <b>Example 09</b>.</p>
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
    private PositionPlant liftPlant;
    private ScalarSource liftHeightIn;

    private final HeldValue<Double> desiredHeightIn = new HeldValue<Double>(HEIGHT_LOW_IN);

    @Override
    public void init() {
        gamepads = Gamepads.create(gamepad1, gamepad2);

        ScalarSource potVoltage = FtcSensors.analogVoltage(hardwareMap, HW_LIFT_POT);
        liftHeightIn = new ScalarSource() {
            @Override
            public double getAsDouble(LoopClock clock) {
                double volts = potVoltage.getAsDouble(clock);
                double t = (volts - SENSOR_MIN_V) / (SENSOR_MAX_V - SENSOR_MIN_V);
                t = Math.max(0.0, Math.min(1.0, t));
                return HEIGHT_MIN_IN + t * (HEIGHT_MAX_IN - HEIGHT_MIN_IN);
            }
        }.memoized();

        Pid liftPid = Pid.withGains(0.12, 0.0, 0.0)
                .setOutputLimits(-0.55, 0.55);

        liftPlant = FtcActuators.plant(hardwareMap)
                .motor(HW_LIFT_MOTOR, Direction.FORWARD)
                .position()
                .regulated()
                .nativeFeedback(FtcActuators.PositionFeedback.fromSource(liftHeightIn))
                .regulator(ScalarRegulators.pid(liftPid))
                .linear()
                .bounded(HEIGHT_MIN_IN, HEIGHT_MAX_IN)
                .nativeUnits()
                .alreadyReferenced()
                .positionTolerance(0.50)
                .targetedByDefaultWritable(0.0)
                .build();

        bindings.onRise(gamepads.p1().a(), () -> desiredHeightIn.set(HEIGHT_LOW_IN));
        bindings.onRise(gamepads.p1().b(), () -> desiredHeightIn.set(HEIGHT_MID_IN));
        bindings.onRise(gamepads.p1().y(), () -> desiredHeightIn.set(HEIGHT_HIGH_IN));
    }

    @Override
    public void loop() {
        clock.update(getRuntime());
        bindings.update(clock);

        // Execution is intentionally trivial here: hold the currently selected height.
        liftPlant.writableTarget().set(desiredHeightIn.get());
        liftPlant.update(clock);

        telemetry.addData("lift.targetIn", liftPlant.getRequestedTarget());
        telemetry.addData("lift.heightIn", liftPlant.getMeasurement());
        telemetry.addData("lift.targetErrorIn", liftPlant.getTargetError());
        telemetry.addData("lift.atTarget", liftPlant.atTarget());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (liftPlant != null) {
            liftPlant.stop();
        }
    }
}
