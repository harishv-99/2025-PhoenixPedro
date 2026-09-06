package edu.ftcsushi.robots.examples.basicsensing;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Proxy;
import java.util.HashMap;
import java.util.Map;

import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.testing.ftc.FtcTestHardware;

/** Supplied test plumbing: one software input, recorded telemetry, and the production managed graph. */
final class BasicSwitchTestRig {
    final BasicSwitchService.Config config;
    final FtcTestHardware hardware = new FtcTestHardware();
    final FtcTestHardware.DigitalProbe input;
    final Map<String, Object> rows = new HashMap<String, Object>();
    final ManagedSwitchOpMode mode;
    int commits;

    /** Starts with the same configuration entry point used by the production OpMode. */
    BasicSwitchTestRig() {
        this(BasicSwitchService.Config.defaults());
    }

    /** Supplies explicitly varied configuration to the same production declaration. */
    BasicSwitchTestRig(BasicSwitchService.Config config) {
        this.config = config;
        input = hardware.addDigitalInput(config.switchName);
        mode = new ManagedSwitchOpMode();
        mode.hardwareMap = hardware;
        mode.gamepad1 = new Gamepad();
        mode.gamepad2 = new Gamepad();
        mode.telemetry = (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(), new Class<?>[]{Telemetry.class},
                (proxy, method, args) -> {
                    if (method.getDeclaringClass() == Object.class) {
                        if ("equals".equals(method.getName())) return proxy == args[0];
                        if ("hashCode".equals(method.getName())) return System.identityHashCode(proxy);
                        return "BasicSwitchTelemetry";
                    }
                    if ("addData".equals(method.getName())) {
                        rows.put((String) args[0], args[1]);
                    } else if ("update".equals(method.getName())) {
                        commits++;
                        return true;
                    }
                    return method.getReturnType() == boolean.class ? false : null;
                });
    }

    /** Authors an electrical level and absolute runtime, then lets the real host run one loop. */
    void observeAt(double runtimeSec, boolean high) {
        input.setHigh(high);
        mode.runtimeSec = runtimeSec;
        mode.loop();
    }

    /** Reads the production owner's cache, never the software input. */
    BasicSwitchService.Status status() {
        return mode.limitSwitch.status();
    }

    /** Replaces only the FTC runtime while preserving the complete production declaration. */
    final class ManagedSwitchOpMode extends FtcRobotOpMode {
        BasicSwitchService limitSwitch;
        double runtimeSec;

        @Override
        protected void configure(RobotProgram program) {
            limitSwitch = BasicSwitchTeleOp.declare(program, hardwareMap, config);
        }

        @Override
        public double getRuntime() {
            return runtimeSec;
        }
    }
}
