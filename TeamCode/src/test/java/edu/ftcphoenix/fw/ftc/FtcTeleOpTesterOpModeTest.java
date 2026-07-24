package edu.ftcphoenix.fw.ftc;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;

import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.List;

import edu.ftcphoenix.fw.tools.tester.TeleOpTester;
import edu.ftcphoenix.fw.tools.tester.TesterContext;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the FTC tester host's one-shot, fail-stop lifecycle ownership. */
public final class FtcTeleOpTesterOpModeTest {

    @Test
    public void nullFactoryResultIsActionableAndTerminal() {
        RecordingTelemetry telemetry = new RecordingTelemetry();
        TestOpMode mode = configuredMode(telemetry);

        RuntimeException thrown = expectRuntimeException(mode::init);

        assertTrue(thrown instanceof IllegalStateException);
        assertTrue(thrown.getMessage().contains("return a configured TeleOpTester"));
        assertEquals(1, mode.createCalls);
        assertTrue(telemetry.contains("createTester() returned null"));

        mode.init();
        mode.init_loop();
        mode.start();
        mode.loop();
        mode.stop();
        assertEquals(1, mode.createCalls);
    }

    @Test
    public void factoryFailureIsActionablePreservesPrimaryAndIsTerminal() {
        RecordingTelemetry telemetry = new RecordingTelemetry();
        TestOpMode mode = configuredMode(telemetry);
        RuntimeException primary = new IllegalStateException("controlled factory failure");
        mode.createFailure = primary;

        assertSame(primary, expectRuntimeException(mode::init));
        assertTrue(telemetry.contains("controlled factory failure"));
        assertTrue(telemetry.contains("acquire hardware and resources in init()"));

        mode.init();
        mode.init_loop();
        mode.start();
        mode.loop();
        mode.stop();
        assertEquals(1, mode.createCalls);
    }

    @Test
    public void successfulRootCannotBeReplacedByASecondInitCall() {
        RecordingTelemetry telemetry = new RecordingTelemetry();
        TestOpMode mode = configuredMode(telemetry);
        RecordingTester tester = new RecordingTester();
        mode.created = tester;
        mode.init();

        RuntimeException thrown = expectRuntimeException(mode::init);

        assertTrue(thrown instanceof IllegalStateException);
        assertTrue(thrown.getMessage().contains("only once"));
        assertEquals(1, mode.createCalls);
        assertEquals(1, tester.initCalls);
        mode.stop();
        assertEquals(1, tester.stopCalls);
    }

    @Test
    public void reentrantStopDuringCreationPreventsOwnershipTransfer() {
        RecordingTelemetry telemetry = new RecordingTelemetry();
        TestOpMode mode = configuredMode(telemetry);
        RecordingTester tester = new RecordingTester();
        mode.created = tester;
        mode.createAction = mode::stop;

        mode.init();

        assertEquals(1, mode.createCalls);
        assertEquals(0, tester.initCalls);
        assertEquals(0, tester.nameCalls);
        assertEquals(0, tester.stopCalls);
        assertTerminalCallbacksDoNothing(mode, tester);
    }

    @Test
    public void reentrantStopDuringInitOrNamePreventsLaterCallbacks() {
        RecordingTelemetry initTelemetry = new RecordingTelemetry();
        TestOpMode initMode = configuredMode(initTelemetry);
        RecordingTester initTester = new RecordingTester();
        initMode.created = initTester;
        initTester.initAction = initMode::stop;

        initMode.init();

        assertEquals(1, initTester.initCalls);
        assertEquals(0, initTester.nameCalls);
        assertEquals(1, initTester.stopCalls);
        assertTerminalCallbacksDoNothing(initMode, initTester);

        RecordingTelemetry nameTelemetry = new RecordingTelemetry();
        TestOpMode nameMode = configuredMode(nameTelemetry);
        RecordingTester nameTester = new RecordingTester();
        nameMode.created = nameTester;
        nameTester.nameAction = nameMode::stop;

        nameMode.init();

        assertEquals(1, nameTester.initCalls);
        assertEquals(1, nameTester.nameCalls);
        assertEquals(1, nameTester.stopCalls);
        assertFalse(nameTelemetry.contains("Ready:"));
        assertTerminalCallbacksDoNothing(nameMode, nameTester);
    }

    @Test
    public void initFailureStopsRetainedRootOnceAndSuppressesCleanupFailure() {
        RecordingTelemetry telemetry = new RecordingTelemetry();
        TestOpMode mode = configuredMode(telemetry);
        RecordingTester tester = new RecordingTester();
        RuntimeException primary = new IllegalStateException("controlled init failure");
        RuntimeException cleanup = new IllegalArgumentException("controlled stop failure");
        tester.initFailure = primary;
        tester.stopFailure = cleanup;
        mode.created = tester;

        RuntimeException thrown = expectRuntimeException(mode::init);

        assertSame(primary, thrown);
        assertArrayEquals(new Throwable[]{cleanup}, thrown.getSuppressed());
        assertEquals(1, tester.initCalls);
        assertEquals(1, tester.stopCalls);
        assertTerminalCallbacksDoNothing(mode, tester);
    }

    @Test
    public void nameFailureStopsInitializedRootOnce() {
        RecordingTelemetry telemetry = new RecordingTelemetry();
        TestOpMode mode = configuredMode(telemetry);
        RecordingTester tester = new RecordingTester();
        RuntimeException primary = new IllegalStateException("controlled name failure");
        tester.nameFailure = primary;
        mode.created = tester;

        assertSame(primary, expectRuntimeException(mode::init));

        assertEquals(1, tester.initCalls);
        assertEquals(1, tester.nameCalls);
        assertEquals(1, tester.stopCalls);
        assertTerminalCallbacksDoNothing(mode, tester);
    }

    @Test
    public void readyTelemetryFailureAlsoFailStopsTheOwnedRoot() {
        RecordingTelemetry telemetry = new RecordingTelemetry();
        TestOpMode mode = configuredMode(telemetry);
        RecordingTester tester = new RecordingTester();
        RuntimeException telemetryFailure =
                new IllegalStateException("controlled ready telemetry failure");
        mode.created = tester;
        telemetry.updateFailure = telemetryFailure;

        assertSame(telemetryFailure, expectRuntimeException(mode::init));

        assertEquals(1, tester.initCalls);
        assertEquals(1, tester.nameCalls);
        assertEquals(1, tester.stopCalls);
        assertTerminalCallbacksDoNothing(mode, tester);
    }

    @Test
    public void runtimeCallbackFailuresStopOnceAndTerminalizeTheHost() {
        for (Callback callback : Callback.values()) {
            RecordingTelemetry telemetry = new RecordingTelemetry();
            TestOpMode mode = configuredMode(telemetry);
            RecordingTester tester = new RecordingTester();
            RuntimeException primary =
                    new IllegalStateException("controlled " + callback + " failure");
            RuntimeException cleanup =
                    new IllegalArgumentException("controlled " + callback + " stop failure");
            mode.created = tester;
            mode.init();
            callback.installFailure(tester, primary);
            tester.stopFailure = cleanup;

            RuntimeException thrown =
                    expectRuntimeException(() -> callback.invoke(mode));

            assertSame(primary, thrown);
            assertArrayEquals(new Throwable[]{cleanup}, thrown.getSuppressed());
            assertEquals(1, tester.stopCalls);
            assertTerminalCallbacksDoNothing(mode, tester);
        }
    }

    @Test
    public void explicitStopDetachesBeforeReentrantCallbackAndRunsExactlyOnce() {
        RecordingTelemetry telemetry = new RecordingTelemetry();
        TestOpMode mode = configuredMode(telemetry);
        RecordingTester tester = new RecordingTester();
        mode.created = tester;
        mode.init();
        tester.stopAction = mode::stop;

        mode.stop();
        mode.stop();
        mode.init_loop();
        mode.start();
        mode.loop();

        assertEquals(1, tester.stopCalls);
        assertEquals(0, tester.initLoopCalls);
        assertEquals(0, tester.startCalls);
        assertEquals(0, tester.loopCalls);
    }

    @Test
    public void explicitStopFailureIsNotRetried() {
        RecordingTelemetry telemetry = new RecordingTelemetry();
        TestOpMode mode = configuredMode(telemetry);
        RecordingTester tester = new RecordingTester();
        RuntimeException stopFailure = new IllegalStateException("controlled explicit stop failure");
        tester.stopFailure = stopFailure;
        mode.created = tester;
        mode.init();

        assertSame(stopFailure, expectRuntimeException(mode::stop));

        mode.stop();
        mode.init_loop();
        mode.start();
        mode.loop();
        assertEquals(1, tester.stopCalls);
    }

    @Test
    public void clockStillAdvancesOnceBeforeEachLoopCallbackAndResetsAtStart() {
        RecordingTelemetry telemetry = new RecordingTelemetry();
        TestOpMode mode = configuredMode(telemetry);
        RecordingTester tester = new RecordingTester();
        mode.created = tester;
        mode.runtimeSec = 10.0;
        mode.init();

        mode.runtimeSec = 10.25;
        mode.init_loop();
        assertEquals(0.25, tester.initLoopDtSec, 0.0);
        assertEquals(2L, tester.initLoopCycle);

        mode.runtimeSec = 20.0;
        mode.start();
        assertEquals(0.0, tester.startDtSec, 0.0);
        assertEquals(3L, tester.startCycle);

        mode.runtimeSec = 20.4;
        mode.loop();
        assertEquals(0.4, tester.loopDtSec, 0.0000000001);
        assertEquals(4L, tester.loopCycle);

        mode.stop();
    }

    @Test
    public void errorIsNotCaughtOrConvertedToRuntimeFailure() {
        RecordingTelemetry telemetry = new RecordingTelemetry();
        TestOpMode mode = configuredMode(telemetry);
        RecordingTester tester = new RecordingTester();
        AssertionError error = new AssertionError("fatal");
        tester.initError = error;
        mode.created = tester;

        try {
            mode.init();
            fail("Expected Error");
        } catch (AssertionError thrown) {
            assertSame(error, thrown);
        }

        assertEquals(0, tester.stopCalls);
        mode.stop();
        assertEquals(1, tester.stopCalls);
    }

    private static void assertTerminalCallbacksDoNothing(
            TestOpMode mode,
            RecordingTester tester
    ) {
        int initCalls = tester.initCalls;
        int nameCalls = tester.nameCalls;
        int initLoopCalls = tester.initLoopCalls;
        int startCalls = tester.startCalls;
        int loopCalls = tester.loopCalls;
        int stopCalls = tester.stopCalls;

        mode.init();
        mode.init_loop();
        mode.start();
        mode.loop();
        mode.stop();

        assertEquals(initCalls, tester.initCalls);
        assertEquals(nameCalls, tester.nameCalls);
        assertEquals(initLoopCalls, tester.initLoopCalls);
        assertEquals(startCalls, tester.startCalls);
        assertEquals(loopCalls, tester.loopCalls);
        assertEquals(stopCalls, tester.stopCalls);
    }

    private static TestOpMode configuredMode(RecordingTelemetry telemetry) {
        TestOpMode mode = new TestOpMode();
        mode.telemetry = telemetry.proxy();
        mode.hardwareMap = new HardwareMap(null, null);
        mode.gamepad1 = new Gamepad();
        mode.gamepad2 = new Gamepad();
        return mode;
    }

    private static RuntimeException expectRuntimeException(Runnable operation) {
        try {
            operation.run();
            fail("Expected RuntimeException");
            throw new AssertionError("unreachable");
        } catch (RuntimeException expected) {
            return expected;
        }
    }

    private enum Callback {
        INIT_LOOP {
            @Override
            void installFailure(RecordingTester tester, RuntimeException failure) {
                tester.initLoopFailure = failure;
            }

            @Override
            void invoke(TestOpMode mode) {
                mode.init_loop();
            }
        },
        START {
            @Override
            void installFailure(RecordingTester tester, RuntimeException failure) {
                tester.startFailure = failure;
            }

            @Override
            void invoke(TestOpMode mode) {
                mode.start();
            }
        },
        LOOP {
            @Override
            void installFailure(RecordingTester tester, RuntimeException failure) {
                tester.loopFailure = failure;
            }

            @Override
            void invoke(TestOpMode mode) {
                mode.loop();
            }
        };

        abstract void installFailure(RecordingTester tester, RuntimeException failure);

        abstract void invoke(TestOpMode mode);
    }

    private static final class TestOpMode extends FtcTeleOpTesterOpMode {
        TeleOpTester created;
        RuntimeException createFailure;
        Runnable createAction;
        int createCalls;
        double runtimeSec;

        @Override
        protected TeleOpTester createTester() {
            createCalls++;
            if (createAction != null) {
                createAction.run();
            }
            if (createFailure != null) {
                throw createFailure;
            }
            return created;
        }

        @Override
        public double getRuntime() {
            return runtimeSec;
        }
    }

    private static final class RecordingTester implements TeleOpTester {
        TesterContext ctx;
        RuntimeException nameFailure;
        RuntimeException initFailure;
        RuntimeException initLoopFailure;
        RuntimeException startFailure;
        RuntimeException loopFailure;
        RuntimeException stopFailure;
        AssertionError initError;
        Runnable nameAction;
        Runnable initAction;
        Runnable stopAction;
        int nameCalls;
        int initCalls;
        int initLoopCalls;
        int startCalls;
        int loopCalls;
        int stopCalls;
        double initLoopDtSec;
        double startDtSec;
        double loopDtSec;
        long initLoopCycle;
        long startCycle;
        long loopCycle;

        @Override
        public String name() {
            nameCalls++;
            if (nameAction != null) {
                nameAction.run();
            }
            if (nameFailure != null) {
                throw nameFailure;
            }
            return "Recording tester";
        }

        @Override
        public void init(TesterContext ctx) {
            initCalls++;
            this.ctx = ctx;
            if (initAction != null) {
                initAction.run();
            }
            if (initError != null) {
                throw initError;
            }
            if (initFailure != null) {
                throw initFailure;
            }
        }

        @Override
        public void initLoop(double dtSec) {
            initLoopCalls++;
            initLoopDtSec = dtSec;
            initLoopCycle = ctx.clock.cycle();
            if (initLoopFailure != null) {
                throw initLoopFailure;
            }
        }

        @Override
        public void start() {
            startCalls++;
            startDtSec = ctx.clock.dtSec();
            startCycle = ctx.clock.cycle();
            if (startFailure != null) {
                throw startFailure;
            }
        }

        @Override
        public void loop(double dtSec) {
            loopCalls++;
            loopDtSec = dtSec;
            loopCycle = ctx.clock.cycle();
            if (loopFailure != null) {
                throw loopFailure;
            }
        }

        @Override
        public void stop() {
            stopCalls++;
            if (stopAction != null) {
                stopAction.run();
            }
            if (stopFailure != null) {
                throw stopFailure;
            }
        }
    }

    private static final class RecordingTelemetry implements InvocationHandler {
        private final List<String> lines = new ArrayList<String>();
        private final Telemetry proxy = (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                this
        );
        RuntimeException updateFailure;

        Telemetry proxy() {
            return proxy;
        }

        boolean contains(String text) {
            for (String line : lines) {
                if (line.contains(text)) {
                    return true;
                }
            }
            return false;
        }

        @Override
        public Object invoke(Object proxy, Method method, Object[] args) {
            if ("addLine".equals(method.getName())
                    && args != null
                    && args.length == 1) {
                lines.add(String.valueOf(args[0]));
            }
            if ("update".equals(method.getName()) && updateFailure != null) {
                throw updateFailure;
            }
            return defaultValue(method.getReturnType());
        }

        private static Object defaultValue(Class<?> type) {
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
    }
}
