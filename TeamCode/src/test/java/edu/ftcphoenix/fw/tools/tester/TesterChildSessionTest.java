package edu.ftcphoenix.fw.tools.tester;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

public final class TesterChildSessionTest {

    @Test
    public void failedInitRetainsBeforeCallbackStopsOnceAndAllowsSafeRetry() {
        TesterChildSession session = new TesterChildSession();
        RuntimeException initFailure = new IllegalStateException("init failed");
        FakeTester first = new FakeTester();
        first.initFailure = initFailure;
        first.onInit = () -> assertTrue(session.hasActive());

        session.retain(first);
        RuntimeException actual = session.init(null);

        assertSame(initFailure, actual);
        assertSame(initFailure, session.lastFailure());
        assertEquals(1, first.initCalls);
        assertEquals(1, first.stopCalls);
        assertFalse(session.hasActive());
        assertTrue(session.canActivate());

        FakeTester replacement = new FakeTester();
        session.retain(replacement);
        assertNull(session.init(null));
        assertTrue(session.hasActive());
        assertNull(session.lastFailure());
    }

    @Test
    public void cleanupFailureIsSuppressedAndBlocksEveryLaterOperation() {
        TesterChildSession session = new TesterChildSession();
        RuntimeException loopFailure = new IllegalStateException("loop failed");
        RuntimeException stopFailure = new IllegalArgumentException("stop failed");
        FakeTester child = new FakeTester();
        child.loopFailure = loopFailure;
        child.stopFailure = stopFailure;

        session.retain(child);
        RuntimeException actual = session.loop(0.02);

        assertSame(loopFailure, actual);
        assertSame(loopFailure, session.lastFailure());
        assertEquals(1, actual.getSuppressed().length);
        assertSame(stopFailure, actual.getSuppressed()[0]);
        assertTrue(session.cleanupBlocked());
        assertFalse(session.canActivate());
        assertFalse(session.hasActive());

        assertNull(session.initLoop(0.02));
        assertNull(session.start());
        assertNull(session.loop(0.02));
        assertFalse(session.backPressed().handled());
        assertNull(session.backPressed().failure());
        assertNull(session.stopForReplacement());
        assertEquals(1, child.loopCalls);
        assertEquals(1, child.stopCalls);
        assertThrows(IllegalStateException.class, () -> session.retain(new FakeTester()));

        assertNull(session.stopTerminal());
        assertNull(session.stopTerminal());
        assertEquals(1, child.stopCalls);
    }

    @Test
    public void startInitLoopAndBackFailuresUseTheSameFailStopPath() {
        assertPhaseFails((session, child) -> session.start(), child -> child.startFailure =
                new IllegalStateException("start"));
        assertPhaseFails((session, child) -> session.initLoop(0.1), child -> child.initLoopFailure =
                new IllegalStateException("initLoop"));
        assertPhaseFails((session, child) -> session.backPressed().failure(),
                child -> child.backFailure = new IllegalStateException("back"));
    }

    @Test
    public void handledBackLeavesChildActiveAndUnhandledBackLeavesStopDecisionToOwner() {
        TesterChildSession session = new TesterChildSession();
        FakeTester child = new FakeTester();
        child.handlesBack = true;
        session.retain(child);

        TesterChildSession.BackResult handled = session.backPressed();
        assertTrue(handled.handled());
        assertNull(handled.failure());
        assertTrue(session.hasActive());
        assertEquals(0, child.stopCalls);

        child.handlesBack = false;
        TesterChildSession.BackResult unhandled = session.backPressed();
        assertFalse(unhandled.handled());
        assertNull(unhandled.failure());
        assertTrue(session.hasActive());
        assertEquals(0, child.stopCalls);
    }

    @Test
    public void terminalStopDetachesBeforeCallbackAndIsReentrantAndExactOnce() {
        TesterChildSession session = new TesterChildSession();
        FakeTester child = new FakeTester();
        child.onStop = () -> {
            assertFalse(session.hasActive());
            assertFalse(session.canActivate());
            assertNull(session.stopTerminal());
        };
        session.retain(child);

        assertNull(session.stopTerminal());
        assertNull(session.stopTerminal());

        assertEquals(1, child.stopCalls);
        assertFalse(session.hasActive());
        assertFalse(session.canActivate());
        assertNull(session.loop(0.02));
        assertThrows(IllegalStateException.class, () -> session.retain(new FakeTester()));
    }

    @Test
    public void replacementStopFailureBlocksRetryAndIsNeverRetried() {
        TesterChildSession session = new TesterChildSession();
        RuntimeException stopFailure = new IllegalStateException("stop failed");
        FakeTester child = new FakeTester();
        child.stopFailure = stopFailure;
        session.retain(child);

        assertSame(stopFailure, session.stopForReplacement());
        assertSame(stopFailure, session.lastFailure());
        assertTrue(session.cleanupBlocked());
        assertNull(session.stopForReplacement());
        assertNull(session.stopTerminal());
        assertEquals(1, child.stopCalls);
    }

    @Test
    public void errorPropagatesWithoutBeingConvertedToRuntimeFailure() {
        TesterChildSession session = new TesterChildSession();
        AssertionError error = new AssertionError("fatal");
        FakeTester child = new FakeTester();
        child.initError = error;
        session.retain(child);

        assertSame(error, assertThrows(AssertionError.class, () -> session.init(null)));
        assertTrue(session.hasActive());
        assertEquals(0, child.stopCalls);
        assertNull(session.lastFailure());

        session.stopTerminal();
        assertEquals(1, child.stopCalls);
    }

    private static void assertPhaseFails(SessionCall call, FakeConfiguration configure) {
        TesterChildSession session = new TesterChildSession();
        FakeTester child = new FakeTester();
        configure.apply(child);
        session.retain(child);

        RuntimeException expected;
        if (child.startFailure != null) {
            expected = child.startFailure;
        } else if (child.initLoopFailure != null) {
            expected = child.initLoopFailure;
        } else {
            expected = child.backFailure;
        }

        assertSame(expected, call.run(session, child));
        assertSame(expected, session.lastFailure());
        assertEquals(1, child.stopCalls);
        assertTrue(session.canActivate());
    }

    private interface SessionCall {
        RuntimeException run(TesterChildSession session, FakeTester child);
    }

    private interface FakeConfiguration {
        void apply(FakeTester child);
    }

    private static final class FakeTester implements TeleOpTester {
        int initCalls;
        int initLoopCalls;
        int startCalls;
        int loopCalls;
        int backCalls;
        int stopCalls;
        boolean handlesBack;
        Runnable onInit;
        Runnable onStop;
        RuntimeException initFailure;
        RuntimeException initLoopFailure;
        RuntimeException startFailure;
        RuntimeException loopFailure;
        RuntimeException backFailure;
        RuntimeException stopFailure;
        Error initError;

        @Override
        public String name() {
            return "Fake";
        }

        @Override
        public void init(TesterContext ctx) {
            initCalls++;
            if (onInit != null) onInit.run();
            if (initError != null) throw initError;
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
            if (onStop != null) onStop.run();
            if (stopFailure != null) throw stopFailure;
        }
    }
}
