package edu.ftcphoenix.robots.examples.starter.support;

import edu.ftcphoenix.robots.examples.starter.robot.StarterProfile;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.ftcphoenix.fw.ftc.FtcRobotOpMode;
import edu.ftcphoenix.fw.testing.ftc.FtcTestHardware;

/** Shared FTC probes for the focused Starter suites. */
public final class StarterTestHardware {

    private StarterTestHardware() {
        // Test fixture.
    }

    public static <T extends FtcRobotOpMode> T prepare(
            T mode,
            HardwareMap hardwareMap,
            TelemetryProbe telemetry,
            Gamepad gamepad1) {
        mode.hardwareMap = hardwareMap;
        mode.telemetry = telemetry.proxy();
        mode.gamepad1 = gamepad1;
        mode.gamepad2 = new Gamepad();
        mode.resetRuntime();
        return mode;
    }

    public static FtcTestHardware fullTeleOpHardware(StarterProfile profile) {
        return fullTeleOpHardware(profile, null);
    }

    public static FtcTestHardware fullTeleOpHardware(
            StarterProfile profile,
            List<String> events) {
        FtcTestHardware hardwareMap = new FtcTestHardware(events);
        hardwareMap.addMotor(profile.intake.motorName);
        hardwareMap.addMotor(profile.drive.wiring.frontLeftName);
        hardwareMap.addMotor(profile.drive.wiring.frontRightName);
        hardwareMap.addMotor(profile.drive.wiring.backLeftName);
        hardwareMap.addMotor(profile.drive.wiring.backRightName);
        return hardwareMap;
    }

    /** Records complete-frame commits without imposing another production seam. */
    public static final class TelemetryProbe implements InvocationHandler {
        private final Telemetry telemetry = (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                this);
        private final List<String> dataKeys = new ArrayList<String>();
        private final List<String> lastFrameKeys = new ArrayList<String>();
        private final Map<String, Object> dataValues = new HashMap<String, Object>();
        private int updateCalls;
        private int dataRowsAtLastUpdate;
        private int committedDataRows;
        private boolean failNextUpdate;
        private final List<String> events;

        public TelemetryProbe() {
            this(null);
        }

        public TelemetryProbe(List<String> events) {
            this.events = events;
        }

        public Telemetry proxy() {
            return telemetry;
        }

        public int updateCalls() {
            return updateCalls;
        }

        public int dataRowsAtLastUpdate() {
            return dataRowsAtLastUpdate;
        }

        public List<String> lastFrameKeys() {
            return lastFrameKeys;
        }

        public Object dataValue(String key) {
            return dataValues.get(key);
        }

        public void failNextUpdate() {
            failNextUpdate = true;
        }

        @Override
        public Object invoke(Object proxy, Method method, Object[] args) {
            if (method.getDeclaringClass() == Object.class) {
                return objectMethod(proxy, method.getName(), args, "TelemetryProbe");
            }
            if ("update".equals(method.getName())) {
                if (events != null) {
                    events.add("telemetry.commit");
                }
                if (failNextUpdate) {
                    failNextUpdate = false;
                    throw new IllegalStateException("telemetry update failed");
                }
                dataRowsAtLastUpdate = dataKeys.size() - committedDataRows;
                lastFrameKeys.clear();
                lastFrameKeys.addAll(dataKeys.subList(committedDataRows, dataKeys.size()));
                committedDataRows = dataKeys.size();
                updateCalls++;
                return true;
            }
            if ("addData".equals(method.getName())) {
                dataKeys.add((String) args[0]);
                dataValues.put((String) args[0], args[1]);
                if (events != null) {
                    events.add("telemetry.row:" + args[0]);
                }
                return null;
            }
            return defaultValue(method.getReturnType());
        }
    }

    private static Object objectMethod(Object proxy,
                                       String methodName,
                                       Object[] args,
                                       String label) {
        if ("equals".equals(methodName)) {
            return proxy == args[0];
        }
        if ("hashCode".equals(methodName)) {
            return System.identityHashCode(proxy);
        }
        if ("toString".equals(methodName)) {
            return label;
        }
        return null;
    }

    private static Object defaultValue(Class<?> type) {
        if (!type.isPrimitive()) {
            return null;
        }
        if (type == boolean.class) {
            return false;
        }
        if (type == byte.class) {
            return (byte) 0;
        }
        if (type == short.class) {
            return (short) 0;
        }
        if (type == int.class) {
            return 0;
        }
        if (type == long.class) {
            return 0L;
        }
        if (type == float.class) {
            return 0.0f;
        }
        if (type == double.class) {
            return 0.0;
        }
        if (type == char.class) {
            return '\0';
        }
        return null;
    }
}
