package edu.ftcphoenix.robots.phoenix;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;

import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.List;

import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.localization.PoseEstimate;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/** Verifies that Phoenix presentation remains additive under the managed lifecycle. */
public final class PhoenixTelemetryOwnershipTest {

    @Test
    public void presenterStagesTeleOpAndAutoRowsWithoutClearingOrCommitting() {
        RecordingTelemetry telemetry = new RecordingTelemetry();
        PhoenixTelemetryPresenter presenter =
                new PhoenixTelemetryPresenter(PhoenixProfile.current());
        PoseEstimate noPose = PoseEstimate.noPose(LoopTimestamp.unavailable());

        telemetry.proxy().addData("upstream.sidecar", "retained");
        presenter.emitTeleOp(
                telemetry.proxy(), null, null, null, null, null, noPose, noPose
        );
        presenter.emitAuto(
                telemetry.proxy(), null, null, null, null, noPose, noPose
        );

        assertEquals(0, telemetry.updateAttempts);
        assertEquals(0, telemetry.clearCalls);
        assertTrue(telemetry.contains("upstream.sidecar"));
        assertTrue(telemetry.contains("pose.global"));
        assertTrue(telemetry.contains("auto.routine"));
        assertTrue(telemetry.contains("auto.routineOutcome"));
    }

    private static final class RecordingTelemetry implements InvocationHandler {
        private final List<String> entries = new ArrayList<>();
        private final Telemetry proxy = (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                this
        );
        private int updateAttempts;
        private int clearCalls;

        private Telemetry proxy() {
            return proxy;
        }

        private boolean contains(String text) {
            for (String entry : entries) {
                if (entry.contains(text)) {
                    return true;
                }
            }
            return false;
        }

        @Override
        public Object invoke(Object ignored, Method method, Object[] args) {
            String name = method.getName();
            if ("addData".equals(name) || "addLine".equals(name)) {
                entries.add(render(args));
            } else if ("update".equals(name)) {
                updateAttempts++;
            } else if ("clear".equals(name) || "clearAll".equals(name)) {
                clearCalls++;
            }
            return defaultValue(method.getReturnType());
        }

        private static String render(Object[] args) {
            StringBuilder result = new StringBuilder();
            if (args != null) {
                for (Object arg : args) {
                    if (result.length() > 0) {
                        result.append(' ');
                    }
                    result.append(arg);
                }
            }
            return result.toString();
        }
    }

    private static Object defaultValue(Class<?> type) {
        if (!type.isPrimitive()) {
            return null;
        }
        if (type == boolean.class) {
            return true;
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
