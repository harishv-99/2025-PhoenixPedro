package edu.ftcphoenix.fw.ftc;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;

import edu.ftcphoenix.fw.core.time.LoopClock;

public final class FtcBulkCachingManagedLifecycleTest {

    @Test
    public void firstServiceStartsAndUpdatesBeforeSensingAndStopsAfterIt() {
        List<String> events = new ArrayList<>();
        RecordingHub hub = new RecordingHub("hub", LynxModule.BulkCachingMode.OFF, events);
        RecordingService sensing = new RecordingService("sensing", events);
        TestOpMode mode = configuredMode();
        mode.configureAction = program -> {
            program.service(new FtcManualBulkCachingService(Arrays.asList(hub)));
            program.service(sensing);
        };

        mode.init();
        assertTrue(events.isEmpty());

        mode.start();
        mode.runtimeSec = 1.0;
        mode.loop();
        mode.stop();

        assertEquals(Arrays.asList(
                "hub.read", "hub.set(MANUAL)", "hub.clear", "sensing.start",
                "hub.clear", "sensing.update",
                "sensing.stop", "hub.clear", "hub.set(OFF)"), events);
        assertSame(LynxModule.BulkCachingMode.OFF, hub.mode);
    }

    @Test
    public void hostCleansUpCapturedModesAfterStartFailure() {
        List<String> events = new ArrayList<>();
        RecordingHub hub = new RecordingHub("hub", LynxModule.BulkCachingMode.AUTO, events);
        RuntimeException startFailure = new RuntimeException("start-set");
        hub.failSet(1, startFailure);
        RecordingService sensing = new RecordingService("sensing", events);
        TestOpMode mode = configuredMode();
        mode.configureAction = program -> {
            program.service(new FtcManualBulkCachingService(Arrays.asList(hub)));
            program.service(sensing);
        };
        mode.init();

        assertSame(startFailure, expectRuntime(mode::start));
        assertEquals(Arrays.asList(
                "hub.read", "hub.set(MANUAL)",
                "sensing.stop", "hub.clear", "hub.set(AUTO)"), events);
        assertSame(LynxModule.BulkCachingMode.AUTO, hub.mode);

        int terminalEffects = events.size();
        mode.loop();
        mode.stop();
        assertEquals(terminalEffects, events.size());
    }

    @Test
    public void hostPreservesUpdateFailureAndSuppressesCleanupFailure() {
        List<String> events = new ArrayList<>();
        RecordingHub hub = new RecordingHub("hub", LynxModule.BulkCachingMode.OFF, events);
        RuntimeException updateFailure = new RuntimeException("active-clear");
        RuntimeException cleanupFailure = new RuntimeException("cleanup-clear");
        hub.failClear(2, updateFailure);
        hub.failClear(3, cleanupFailure);
        RecordingService sensing = new RecordingService("sensing", events);
        TestOpMode mode = configuredMode();
        mode.configureAction = program -> {
            program.service(new FtcManualBulkCachingService(Arrays.asList(hub)));
            program.service(sensing);
        };
        mode.init();
        mode.start();
        events.clear();
        mode.runtimeSec = 1.0;

        RuntimeException thrown = expectRuntime(mode::loop);
        assertSame(updateFailure, thrown);
        assertArrayEquals(new Throwable[]{cleanupFailure}, thrown.getSuppressed());
        assertEquals(Arrays.asList(
                "hub.clear", "sensing.stop", "hub.clear", "hub.set(OFF)"), events);
        assertSame(LynxModule.BulkCachingMode.OFF, hub.mode);
    }

    @Test
    public void blockedStartNeverMutatesOrClearsBulkCache() {
        List<String> events = new ArrayList<>();
        RecordingHub hub = new RecordingHub("hub", LynxModule.BulkCachingMode.OFF, events);
        RecordingService sensing = new RecordingService("sensing", events);
        TestOpMode mode = configuredMode();
        mode.configureAction = program -> {
            program.prestart(new RobotProgram.Prestart() {
                @Override
                public void update(LoopClock clock) {
                    // No prestart observation is needed for this fixed blocked policy.
                }

                @Override
                public RobotProgram.StartDisposition freezeForStart() {
                    return RobotProgram.StartDisposition.BLOCKED;
                }
            });
            program.service(new FtcManualBulkCachingService(Arrays.asList(hub)));
            program.service(sensing);
        };

        mode.init();
        mode.start();
        mode.runtimeSec = 1.0;
        mode.loop();
        mode.stop();

        assertEquals(Arrays.asList("sensing.stop"), events);
        assertEquals(0, hub.readCalls);
        assertEquals(0, hub.setCalls);
        assertEquals(0, hub.clearCalls);
    }

    @Test
    public void programWithoutBulkCachingOptInKeepsOrdinaryServiceLifecycle() {
        List<String> events = new ArrayList<>();
        RecordingService sensing = new RecordingService("sensing", events);
        TestOpMode mode = configuredMode();
        mode.configureAction = program -> program.service(sensing);

        mode.init();
        mode.start();
        mode.runtimeSec = 1.0;
        mode.loop();
        mode.stop();

        assertEquals(Arrays.asList(
                "sensing.start", "sensing.update", "sensing.stop"), events);
    }

    private static TestOpMode configuredMode() {
        TestOpMode mode = new TestOpMode();
        mode.telemetry = telemetryProxy();
        mode.hardwareMap = new HardwareMap(null, null);
        mode.gamepad1 = new Gamepad();
        mode.gamepad2 = new Gamepad();
        return mode;
    }

    private static Telemetry telemetryProxy() {
        return (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                (proxy, method, args) -> defaultValue(method));
    }

    private static Object defaultValue(Method method) {
        Class<?> type = method.getReturnType();
        if (!type.isPrimitive()) {
            return null;
        }
        if (type == boolean.class) {
            return false;
        }
        if (type == char.class) {
            return '\0';
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
        return null;
    }

    private static RuntimeException expectRuntime(Runnable operation) {
        try {
            operation.run();
            fail("Expected RuntimeException");
            throw new AssertionError("unreachable");
        } catch (RuntimeException failure) {
            return failure;
        }
    }

    private static final class TestOpMode extends FtcRobotOpMode {
        Consumer<RobotProgram> configureAction = program -> { };
        double runtimeSec;

        @Override
        protected void configure(RobotProgram program) {
            configureAction.accept(program);
        }

        @Override
        public double getRuntime() {
            return runtimeSec;
        }
    }

    private static final class RecordingService implements RobotProgram.Service {
        private final String name;
        private final List<String> events;

        RecordingService(String name, List<String> events) {
            this.name = name;
            this.events = events;
        }

        @Override
        public void start(LoopClock clock) {
            events.add(name + ".start");
        }

        @Override
        public void update(LoopClock clock) {
            events.add(name + ".update");
        }

        @Override
        public void stop() {
            events.add(name + ".stop");
        }
    }

    private static final class RecordingHub implements FtcBulkCachingHub {
        private final String name;
        private final List<String> events;
        private final Map<Integer, RuntimeException> setFailures = new HashMap<>();
        private final Map<Integer, RuntimeException> clearFailures = new HashMap<>();
        LynxModule.BulkCachingMode mode;
        int readCalls;
        int setCalls;
        int clearCalls;

        RecordingHub(
                String name,
                LynxModule.BulkCachingMode mode,
                List<String> events) {
            this.name = name;
            this.mode = mode;
            this.events = events;
        }

        @Override
        public LynxModule.BulkCachingMode readMode() {
            readCalls++;
            events.add(name + ".read");
            return mode;
        }

        @Override
        public void setMode(LynxModule.BulkCachingMode requestedMode) {
            int call = ++setCalls;
            events.add(name + ".set(" + requestedMode + ")");
            RuntimeException failure = setFailures.get(call);
            if (failure != null) {
                throw failure;
            }
            mode = requestedMode;
        }

        @Override
        public void clearCache() {
            int call = ++clearCalls;
            events.add(name + ".clear");
            RuntimeException failure = clearFailures.get(call);
            if (failure != null) {
                throw failure;
            }
        }

        void failSet(int call, RuntimeException failure) {
            setFailures.put(call, failure);
        }

        void failClear(int call, RuntimeException failure) {
            clearFailures.put(call, failure);
        }
    }
}
