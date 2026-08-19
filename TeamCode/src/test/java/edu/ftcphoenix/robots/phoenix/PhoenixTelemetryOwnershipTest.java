package edu.ftcphoenix.robots.phoenix;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.List;

import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.ftc.localization.FtcOdometryAprilTagLocalizationLane;
import edu.ftcphoenix.fw.localization.PoseEstimate;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/** Verifies that Phoenix presentation remains additive under the managed lifecycle. */
public final class PhoenixTelemetryOwnershipTest {

    @Test
    public void presenterIsPackagePrivateAndAcceptsOnlyItsTwoDisplayFacts() {
        assertTrue(Modifier.isFinal(PhoenixTelemetryPresenter.class.getModifiers()));
        assertFalse(Modifier.isPublic(PhoenixTelemetryPresenter.class.getModifiers()));

        Constructor<?>[] constructors =
                PhoenixTelemetryPresenter.class.getDeclaredConstructors();
        assertEquals(1, constructors.length);
        assertFalse(Modifier.isPublic(constructors[0].getModifiers()));
        assertEquals(2, constructors[0].getParameterTypes().length);
        assertEquals(
                FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode.class,
                constructors[0].getParameterTypes()[0]
        );
        assertEquals(
                FtcOdometryAprilTagLocalizationLane.CorrectionSourceMode.class,
                constructors[0].getParameterTypes()[1]
        );
    }

    @Test
    public void presenterStagesTeleOpAndAutoRowsWithoutClearingOrCommitting() {
        RecordingTelemetry telemetry = new RecordingTelemetry();
        PhoenixProfile profile = PhoenixProfile.current();
        PhoenixTelemetryPresenter presenter = new PhoenixTelemetryPresenter(
                profile.localization.estimation.correctedEstimatorMode,
                profile.localization.estimation.correctionSource.mode
        );
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
