package edu.ftcphoenix.fw.tools.tester;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationHandler;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Supplier;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.ftc.ui.MenuItem;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

public final class TesterMenuLifecycleTest {

    @Test
    public void suiteRejectsEnabledNullFactoriesButKeepsDisabledPlaceholderItems() {
        TesterSuite suite = new TesterSuite();

        NullPointerException addFailure = assertThrows(
                NullPointerException.class,
                () -> suite.add("Shooter", null));
        assertTrue(addFailure.getMessage().contains("Shooter"));

        suite.addItem(new MenuItem<Supplier<TeleOpTester>>(
                "later",
                "Later",
                null,
                "TODO",
                false,
                "Not configured",
                null));

        IllegalArgumentException enabledFailure = assertThrows(
                IllegalArgumentException.class,
                () -> suite.addItem(new MenuItem<Supplier<TeleOpTester>>(
                        "bad",
                        "Bad",
                        null,
                        null,
                        true,
                        null,
                        null)));
        assertTrue(enabledFailure.getMessage().contains("Bad"));
    }

    @Test
    public void suiteFailedInitStopsOnceShowsNamedErrorAndAllowsFreshRetry() throws Exception {
        RecordingTelemetry telemetry = new RecordingTelemetry();
        TesterSuite suite = initializedSuite(telemetry);
        RuntimeException initFailure = new IllegalStateException("bad setup");
        FakeTester failed = new FakeTester();
        failed.initFailure = initFailure;

        enter(suite, "Shooter", () -> failed);

        assertEquals(1, failed.stopCalls);
        assertSame(initFailure, field(suite, "childFailure"));
        suite.initLoop(0.02);
        assertTrue(telemetry.contains("'Shooter' failed during init"));

        FakeTester replacement = new FakeTester();
        enter(suite, "Shooter retry", () -> replacement);

        assertEquals(1, replacement.initCalls);
        assertNull(field(suite, "childFailure"));
        suite.loop(0.02);
        assertEquals(1, replacement.loopCalls);
    }

    @Test
    public void suiteLoopCleanupFailureBlocksFactoryCallbacksAndNestedBack() throws Exception {
        RecordingTelemetry telemetry = new RecordingTelemetry();
        TesterSuite suite = initializedSuite(telemetry);
        RuntimeException loopFailure = new IllegalStateException("loop failed");
        RuntimeException stopFailure = new IllegalArgumentException("stop failed");
        FakeTester failed = new FakeTester();
        failed.loopFailure = loopFailure;
        failed.stopFailure = stopFailure;
        enter(suite, "Flywheel", () -> failed);

        suite.loop(0.02);

        assertSame(loopFailure, field(suite, "childFailure"));
        assertSame(stopFailure, loopFailure.getSuppressed()[0]);
        assertEquals(1, failed.loopCalls);
        assertEquals(1, failed.stopCalls);
        AtomicInteger replacementFactoryCalls = new AtomicInteger();
        enter(suite, "Unsafe replacement", () -> {
            replacementFactoryCalls.incrementAndGet();
            return new FakeTester();
        });
        assertEquals(0, replacementFactoryCalls.get());
        assertTrue(suite.onBackPressed());
        suite.loop(0.02);
        assertEquals(1, failed.loopCalls);
        assertTrue(telemetry.contains("Cleanup also failed"));
        assertTrue(telemetry.contains("Restart the OpMode"));

        suite.stop();
        suite.stop();
        assertEquals(1, failed.stopCalls);
    }

    @Test
    public void suiteStartAndBackFailuresUseSelectedLabelAndStopExactlyOnce() throws Exception {
        RecordingTelemetry telemetry = new RecordingTelemetry();
        TesterSuite suite = initializedSuite(telemetry);
        FakeTester startFailed = new FakeTester();
        RuntimeException startFailure = new IllegalStateException("start failed");
        startFailed.startFailure = startFailure;
        enter(suite, "Indexer", () -> startFailed);

        suite.start();
        suite.loop(0.02);

        assertEquals(1, startFailed.stopCalls);
        assertTrue(telemetry.contains("'Indexer' failed during start"));

        FakeTester backFailed = new FakeTester();
        RuntimeException backFailure = new IllegalStateException("back failed");
        backFailed.backFailure = backFailure;
        enter(suite, "Turret", () -> backFailed);
        assertTrue(suite.onBackPressed());
        suite.initLoop(0.02);

        assertEquals(1, backFailed.stopCalls);
        assertTrue(telemetry.contains("'Turret' failed during BACK"));
    }

    @Test
    public void nestedSuitesDeliverOneBackCallbackPerSharedClockCycle() throws Exception {
        RecordingTelemetry telemetry = new RecordingTelemetry();
        LoopClock clock = new LoopClock();
        clock.reset(0.0);
        Gamepad gamepad1 = new Gamepad();
        TesterContext context = new TesterContext(
                new HardwareMap(null, null),
                telemetry.proxy(),
                gamepad1,
                new Gamepad(),
                clock);

        TesterSuite parent = new TesterSuite();
        parent.init(context);
        TesterSuite nested = new TesterSuite();
        enter(parent, "Nested", () -> nested);
        FakeTester leaf = new FakeTester();
        leaf.handlesBack = true;
        enter(nested, "Vision", () -> leaf);

        // Prime both suites' rising-edge bindings with BACK released.
        clock.update(0.02);
        parent.initLoop(clock.dtSec());

        gamepad1.back = true;
        clock.update(0.04);
        parent.initLoop(clock.dtSec());

        assertEquals(1, leaf.backCalls);
        assertEquals(0, leaf.stopCalls);
        assertEquals(2, leaf.initLoopCalls);
    }

    @Test
    public void startClearsBackCacheWhenRootClockReusesAnInitCycleNumber() throws Exception {
        LoopClock clock = new LoopClock();
        clock.reset(0.0);
        TesterSuite suite = initializedSuite(new RecordingTelemetry(), clock);
        FakeTester leaf = new FakeTester();
        leaf.handlesBack = true;
        enter(suite, "Vision", () -> leaf);

        assertTrue(suite.onBackPressed());
        assertEquals(1, leaf.backCalls);

        clock.reset(1.0);
        suite.start();
        assertTrue(suite.onBackPressed());

        assertEquals(2, leaf.backCalls);
        assertEquals(1, leaf.startCalls);
        assertEquals(0, leaf.stopCalls);
    }

    @Test
    public void hardwareSelectorNullFactoryResultAllowsRetry() throws Exception {
        RecordingTelemetry telemetry = new RecordingTelemetry();
        AtomicInteger calls = new AtomicInteger();
        FakeTester replacement = new FakeTester();
        HardwareSelectingTester selector = initializedSelector(telemetry, name ->
                calls.getAndIncrement() == 0 ? null : replacement);

        enter(selector, "encoder");
        selector.initLoop(0.02);
        assertTrue(telemetry.contains("returned null for 'encoder'"));

        enter(selector, "encoder");
        assertEquals(2, calls.get());
        assertEquals(1, replacement.initCalls);
        selector.loop(0.02);
        assertEquals(1, replacement.loopCalls);
    }

    @Test
    public void hardwareSelectorCleanupFailureBlocksReplacementAndConsumesBack() throws Exception {
        RecordingTelemetry telemetry = new RecordingTelemetry();
        AtomicInteger calls = new AtomicInteger();
        RuntimeException initFailure = new IllegalStateException("init failed");
        RuntimeException stopFailure = new IllegalArgumentException("close failed");
        FakeTester failed = new FakeTester();
        failed.initFailure = initFailure;
        failed.stopFailure = stopFailure;
        HardwareSelectingTester selector = initializedSelector(telemetry, name -> {
            calls.incrementAndGet();
            return failed;
        });

        enter(selector, "throughBore");
        enter(selector, "replacement");

        assertEquals(1, calls.get());
        assertEquals(1, failed.stopCalls);
        assertSame(initFailure, field(selector, "childFailure"));
        assertSame(stopFailure, initFailure.getSuppressed()[0]);
        assertTrue(selector.onBackPressed());
        selector.initLoop(0.02);
        assertTrue(telemetry.contains("'throughBore' failed during init"));
        assertTrue(telemetry.contains("Retry disabled"));
        assertTrue(telemetry.contains("Restart the OpMode"));

        selector.stop();
        selector.stop();
        assertEquals(1, failed.stopCalls);
    }

    @Test
    public void hardwareSelectorFactoryErrorPropagatesButRuntimeFailureIsRetryable() throws Exception {
        RecordingTelemetry telemetry = new RecordingTelemetry();
        AssertionError error = new AssertionError("fatal factory");
        HardwareSelectingTester errorSelector = initializedSelector(telemetry, name -> {
            throw error;
        });

        assertSame(error, assertThrows(AssertionError.class, () -> enter(errorSelector, "motor")));

        RuntimeException runtimeFailure = new IllegalStateException("bad config");
        HardwareSelectingTester runtimeSelector = initializedSelector(telemetry, name -> {
            throw runtimeFailure;
        });
        enter(runtimeSelector, "motor");
        assertSame(runtimeFailure, field(runtimeSelector, "childFailure"));
    }

    private static TesterSuite initializedSuite(RecordingTelemetry telemetry) {
        LoopClock clock = new LoopClock();
        clock.reset(0.0);
        return initializedSuite(telemetry, clock);
    }

    private static TesterSuite initializedSuite(RecordingTelemetry telemetry, LoopClock clock) {
        TesterSuite suite = new TesterSuite();
        suite.init(context(telemetry, clock));
        return suite;
    }

    private static HardwareSelectingTester initializedSelector(
            RecordingTelemetry telemetry,
            java.util.function.Function<String, TeleOpTester> factory
    ) {
        HardwareSelectingTester selector = new HardwareSelectingTester(
                "Hardware test",
                DcMotor.class,
                null,
                null,
                null,
                factory);
        selector.init(context(telemetry));
        return selector;
    }

    private static TesterContext context(RecordingTelemetry telemetry) {
        LoopClock clock = new LoopClock();
        clock.reset(0.0);
        return context(telemetry, clock);
    }

    private static TesterContext context(RecordingTelemetry telemetry, LoopClock clock) {
        return new TesterContext(
                new HardwareMap(null, null),
                telemetry.proxy(),
                new Gamepad(),
                new Gamepad(),
                clock);
    }

    private static void enter(
            TesterSuite suite,
            String name,
            Supplier<TeleOpTester> factory
    ) throws Exception {
        invokePrivate(
                suite,
                "enter",
                new Class<?>[]{String.class, Supplier.class},
                name,
                factory);
    }

    private static void enter(HardwareSelectingTester selector, String hardwareName) throws Exception {
        invokePrivate(
                selector,
                "enter",
                new Class<?>[]{String.class},
                hardwareName);
    }

    private static void invokePrivate(
            Object target,
            String methodName,
            Class<?>[] parameterTypes,
            Object... args
    ) throws Exception {
        Method method = target.getClass().getDeclaredMethod(methodName, parameterTypes);
        method.setAccessible(true);
        try {
            method.invoke(target, args);
        } catch (InvocationTargetException invocationFailure) {
            Throwable cause = invocationFailure.getCause();
            if (cause instanceof Error) throw (Error) cause;
            if (cause instanceof RuntimeException) throw (RuntimeException) cause;
            throw invocationFailure;
        }
    }

    private static Object field(Object target, String fieldName) throws Exception {
        Field field = target.getClass().getDeclaredField(fieldName);
        field.setAccessible(true);
        return field.get(target);
    }

    private static final class FakeTester implements TeleOpTester {
        int initCalls;
        int initLoopCalls;
        int startCalls;
        int loopCalls;
        int backCalls;
        int stopCalls;
        RuntimeException initFailure;
        RuntimeException initLoopFailure;
        RuntimeException startFailure;
        RuntimeException loopFailure;
        RuntimeException backFailure;
        RuntimeException stopFailure;
        boolean handlesBack;

        @Override
        public String name() {
            return "Fake";
        }

        @Override
        public void init(TesterContext ctx) {
            initCalls++;
            if (initFailure != null) throw initFailure;
        }

        @Override
        public void initLoop(double dtSec) {
            initLoopCalls++;
            if (initLoopFailure != null) throw initLoopFailure;
        }

        @Override
        public void start() {
            startCalls++;
            if (startFailure != null) throw startFailure;
        }

        @Override
        public void loop(double dtSec) {
            loopCalls++;
            if (loopFailure != null) throw loopFailure;
        }

        @Override
        public boolean onBackPressed() {
            backCalls++;
            if (backFailure != null) throw backFailure;
            return handlesBack;
        }

        @Override
        public void stop() {
            stopCalls++;
            if (stopFailure != null) throw stopFailure;
        }
    }

    private static final class RecordingTelemetry implements InvocationHandler {
        private final List<String> entries = new ArrayList<String>();
        private final Telemetry telemetry = (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                this);

        Telemetry proxy() {
            return telemetry;
        }

        boolean contains(String text) {
            for (String entry : entries) {
                if (entry.contains(text)) return true;
            }
            return false;
        }

        @Override
        public Object invoke(Object proxy, Method method, Object[] args) {
            if (("addData".equals(method.getName()) || "addLine".equals(method.getName()))
                    && args != null) {
                StringBuilder entry = new StringBuilder();
                for (Object arg : args) {
                    entry.append(' ').append(arg);
                }
                entries.add(entry.toString());
            }
            Class<?> returnType = method.getReturnType();
            if (returnType == boolean.class) return true;
            if (returnType == int.class) return 0;
            return null;
        }
    }
}
