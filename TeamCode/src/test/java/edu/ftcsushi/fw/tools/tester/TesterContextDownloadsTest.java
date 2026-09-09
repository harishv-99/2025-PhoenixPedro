package edu.ftcsushi.fw.tools.tester;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.junit.Test;

import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.ftc.ResultDownloads;

import static org.junit.Assert.*;

/** Owner lifetimes and epoch leases are distinct; neither owns a heartbeat or hardware. */
public final class TesterContextDownloadsTest {
    @Test public void offlineContextIsUnavailableAndExplicitHostInjectionRejectsNull() {
        TesterContext offline = new TesterContext(null, null, new Gamepad(), new Gamepad(), clock());
        assertFalse(offline.downloads.publish("anything.txt", "value"));
        assertNull(offline.downloads.url());
        offline.downloads.clear();
        offline.downloads.clear();
        assertThrows(NullPointerException.class,
                () -> new TesterContext(null, null, null, null, clock(), null));
    }

    @Test public void baseStartRenewsLeaseBeforeHookWithoutReinitializingAnyRuntimeObject() {
        Port port = new Port();
        TesterContext initial = context(port);
        ScopeTester tester = new ScopeTester();
        tester.init(initial);
        ResultDownloads old = initial.downloads;
        assertTrue(old.publish("init.txt", "init"));
        long cycle = initial.clock.cycle();
        tester.onStartAction = () -> {
            assertFalse(old.publish("late.txt", "late"));
            assertNull(old.url());
            assertTrue(tester.current().downloads.publish("run.txt", "run"));
        };
        tester.start();
        TesterContext running = tester.current();
        assertNotSame(initial, running);
        assertSame(initial.hw, running.hw);
        assertSame(initial.telemetry, running.telemetry);
        assertSame(initial.gamepad1, running.gamepad1);
        assertSame(initial.gamepad2, running.gamepad2);
        assertSame(initial.clock, running.clock);
        assertEquals(cycle, running.clock.cycle());
        assertEquals(1, tester.initCalls);
        assertEquals(1, tester.startCalls);
        old.clear();
        assertEquals("run", port.text);
        assertEquals("download:run.txt", running.downloads.url());
    }

    @Test public void parentRefreshPreservesNestedChildOwnerAndExitRevokesItsRenewedLease() {
        Port port = new Port();
        TesterContext root = context(port);
        ScopeTester parent = new ScopeTester();
        parent.init(root);
        TesterChildSession children = new TesterChildSession();
        ScopeTester child = new ScopeTester();
        children.retain(child);
        assertNull(children.init(parent.current()));
        TesterContext childBefore = child.current();
        ResultDownloads old = childBefore.downloads;
        parent.onStartAction = () -> assertNull(children.start());
        parent.start();
        assertFalse(old.publish("old.txt", "old"));
        assertTrue(child.current().downloads.publish("child.txt", "child"));
        assertSame(parent.current().clock, child.current().clock);
        assertSame(parent.current().gamepad1, child.current().gamepad1);
        child.onStopAction = () -> {
            assertNull(child.current().downloads.url());
            assertFalse(child.current().downloads.publish("stop.txt", "stop"));
        };
        assertNull(children.stopForReplacement());
        assertNull(port.text);
        assertFalse(child.current().downloads.publish("late.txt", "late"));
        assertEquals(1, child.stopCalls);
        assertTrue(parent.current().downloads.publish("parent.txt", "parent"));
        childBefore.revokeDownloads();
        child.current().downloads.clear();
        assertEquals("parent", port.text);
    }

    @Test public void oldChildCannotClearOrPublishOverItsReplacement() {
        Port port = new Port();
        TesterContext root = context(port);
        TesterChildSession children = new TesterChildSession();
        ScopeTester first = new ScopeTester();
        children.retain(first);
        assertNull(children.init(root));
        first.current().downloads.publish("first.txt", "first");
        assertNull(children.stopForReplacement());
        ScopeTester next = new ScopeTester();
        children.retain(next);
        assertNull(children.init(root));
        assertTrue(next.current().downloads.publish("next.txt", "next"));
        first.current().downloads.clear();
        assertFalse(first.current().downloads.publish("late.txt", "late"));
        assertNull(first.current().downloads.url());
        assertEquals("next", port.text);
        root.revokeDownloads();
        assertFalse(next.current().downloads.publish("ancestor-stopped.txt", "late"));
        assertNull(next.current().downloads.url());
    }

    @Test public void failedInjectedClearStillRevokesChildAndAttemptsHardwareCleanupExactlyOnce() {
        Port port = new Port();
        TesterChildSession children = new TesterChildSession();
        ScopeTester child = new ScopeTester();
        children.retain(child);
        assertNull(children.init(context(port)));
        RuntimeException clear = new IllegalStateException("download clear failed");
        RuntimeException stop = new IllegalArgumentException("hardware cleanup failed");
        port.clearFailure = clear;
        child.onStopAction = () -> {
            assertFalse(child.current().downloads.publish("during-stop.txt", "late"));
            throw stop;
        };
        assertSame(clear, children.stopForReplacement());
        assertArrayEquals(new Throwable[]{stop}, clear.getSuppressed());
        assertEquals(1, child.stopCalls);
        assertTrue(children.cleanupBlocked());
        assertNull(children.stopTerminal());
        assertNull(children.stopTerminal());
        assertEquals(1, child.stopCalls);
        assertFalse(child.current().downloads.publish("after.txt", "late"));
    }

    @Test public void primaryLifecycleFailureSurvivesFailedClearAndCleanup() {
        Port port = new Port();
        TesterChildSession children = new TesterChildSession();
        ScopeTester child = new ScopeTester();
        children.retain(child);
        assertNull(children.init(context(port)));
        RuntimeException primary = new IllegalStateException("loop failed");
        RuntimeException clear = new IllegalArgumentException("clear failed");
        RuntimeException stop = new IllegalStateException("stop failed");
        child.onLoopAction = () -> { throw primary; };
        child.onStopAction = () -> { throw stop; };
        port.clearFailure = clear;
        assertSame(primary, children.loop(.02));
        assertArrayEquals(new Throwable[]{clear}, primary.getSuppressed());
        assertArrayEquals(new Throwable[]{stop}, clear.getSuppressed());
        assertEquals(1, child.stopCalls);
    }

    @Test public void startClearFailureDisablesDownloadsButStillRunsNormalStartHook() {
        Port port = new Port();
        ScopeTester tester = new ScopeTester();
        TesterContext old = context(port);
        tester.init(old);
        port.clearFailure = new IllegalStateException("optional transport unavailable");
        tester.onStartAction = () -> assertFalse(tester.current().downloads.publish("trial.txt", "x"));
        tester.start();
        assertEquals(1, tester.startCalls);
        assertNotSame(old, tester.current());
        assertSame(old.clock, tester.current().clock);
        assertFalse(old.downloads.publish("old.txt", "x"));
        assertNull(tester.current().downloads.url());
        tester.stop();
        assertEquals(1, tester.stopCalls);
    }

    @Test public void directCustomChildIsFailClosedWithoutChangingItsLifecycle() {
        Port port = new Port();
        TesterChildSession children = new TesterChildSession();
        final TesterContext[] received = new TesterContext[1];
        final int[] stops = new int[1];
        children.retain(new TeleOpTester() {
            @Override public String name() { return "Custom direct tester"; }
            @Override public void init(TesterContext ctx) { received[0] = ctx; }
            @Override public void loop(double dtSec) { }
            @Override public void stop() { stops[0]++; }
        });
        TesterContext supplied = context(port);
        assertNull(children.init(supplied));
        assertSame(supplied.clock, received[0].clock);
        assertFalse(received[0].downloads.publish("custom.txt", "not retained"));
        assertNull(received[0].downloads.url());
        assertNull(children.start());
        assertNull(children.stopTerminal());
        assertEquals(1, stops[0]);
        assertEquals(0, port.publishes);
    }

    @Test public void reentrantClearStopsOwnerBeforeStartHookAndCannotReviveItsLease() {
        Port port = new Port();
        TesterChildSession children = new TesterChildSession();
        ScopeTester child = new ScopeTester();
        children.retain(child);
        assertNull(children.init(context(port)));
        ResultDownloads before = child.current().downloads;
        port.onClear = () -> assertNull(children.stopTerminal());
        assertNull(children.start());
        assertEquals(1, child.stopCalls);
        assertEquals(0, child.startCalls);
        assertFalse(children.hasActive());
        assertFalse(before.publish("old.txt", "late"));
        assertFalse(child.current().downloads.publish("new.txt", "late"));
        assertNull(children.start());
        assertNull(children.stopTerminal());
        assertEquals(1, child.stopCalls);
    }

    private static LoopClock clock() {
        LoopClock clock = new LoopClock();
        clock.reset(0);
        return clock;
    }

    private static TesterContext context(Port port) {
        return new TesterContext(null, null, new Gamepad(), new Gamepad(), clock(), port);
    }

    private static final class ScopeTester extends BaseTeleOpTester {
        int initCalls;
        int startCalls;
        int stopCalls;
        Runnable onStartAction;
        Runnable onLoopAction;
        Runnable onStopAction;
        TesterContext current() { return ctx; }
        @Override public String name() { return "Scoped"; }
        @Override protected void onInit() { initCalls++; }
        @Override protected void onStart() {
            startCalls++;
            if (onStartAction != null) onStartAction.run();
        }
        @Override protected void onLoop(double dtSec) {
            if (onLoopAction != null) onLoopAction.run();
        }
        @Override protected void onStop() {
            stopCalls++;
            if (onStopAction != null) onStopAction.run();
        }
    }

    private static final class Port implements ResultDownloads {
        String text;
        String filename;
        int publishes;
        RuntimeException clearFailure;
        Runnable onClear;
        @Override public boolean publish(String filename, String frozenUtf8Text) {
            publishes++;
            this.filename = filename;
            text = frozenUtf8Text;
            return true;
        }
        @Override public String url() { return text == null ? null : "download:" + filename; }
        @Override public void clear() {
            if (onClear != null) {
                Runnable action = onClear;
                onClear = null;
                action.run();
            }
            if (clearFailure != null) throw clearFailure;
            text = null;
        }
    }
}
