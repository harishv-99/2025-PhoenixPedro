package edu.ftcphoenix.fw.ftc;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.ftcphoenix.fw.core.time.LoopClock;

public final class FtcBulkCachingTest {

    @Test
    public void publicSurfaceIsOneFactoryWithNoPublicImplementationType() throws Exception {
        assertTrue(Modifier.isPublic(FtcBulkCaching.class.getModifiers()));
        assertTrue(Modifier.isFinal(FtcBulkCaching.class.getModifiers()));

        Constructor<?>[] constructors = FtcBulkCaching.class.getDeclaredConstructors();
        assertEquals(1, constructors.length);
        assertTrue(Modifier.isPrivate(constructors[0].getModifiers()));

        List<Method> publicMethods = new ArrayList<>();
        for (Method method : FtcBulkCaching.class.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers())) {
                publicMethods.add(method);
            }
        }
        assertEquals(1, publicMethods.size());
        Method manual = publicMethods.get(0);
        assertEquals("manual", manual.getName());
        assertTrue(Modifier.isStatic(manual.getModifiers()));
        assertEquals(RobotProgram.Service.class, manual.getReturnType());
        assertArrayEquals(new Class<?>[]{HardwareMap.class}, manual.getParameterTypes());

        for (Class<?> nested : FtcBulkCaching.class.getDeclaredClasses()) {
            assertFalse(Modifier.isPublic(nested.getModifiers()));
        }
        assertFalse(Modifier.isPublic(FtcBulkCachingHub.class.getModifiers()));
        assertFalse(Modifier.isPublic(FtcManualBulkCachingService.class.getModifiers()));
        assertTrue(Modifier.isFinal(FtcManualBulkCachingService.class.getModifiers()));
        assertTrue(RobotProgram.Service.class.isAssignableFrom(
                FtcManualBulkCachingService.class));
    }

    @Test
    public void publicFactoryValidatesNullEmptyAndThrowingDiscovery() {
        IllegalArgumentException nullMap = expect(
                IllegalArgumentException.class,
                () -> FtcBulkCaching.manual(null));
        assertContainsPublicOwner(nullMap);

        DiscoveryHardwareMap empty = DiscoveryHardwareMap.returning(Collections.emptyList());
        IllegalStateException emptyFailure = expect(
                IllegalStateException.class,
                () -> FtcBulkCaching.manual(empty));
        assertContainsPublicOwner(emptyFailure);
        assertEquals(1, empty.calls);
        assertSame(LynxModule.class, empty.requestedType);

        RuntimeException discoveryFailure = new RuntimeException("discovery");
        DiscoveryHardwareMap throwing = DiscoveryHardwareMap.throwing(discoveryFailure);
        assertSame(discoveryFailure, expect(
                RuntimeException.class,
                () -> FtcBulkCaching.manual(throwing)));
        assertEquals(1, throwing.calls);
        assertSame(LynxModule.class, throwing.requestedType);
    }

    @Test
    public void constructorValidatesAndDefensivelyCopiesWithoutCacheEffects() {
        expect(IllegalArgumentException.class,
                () -> new FtcManualBulkCachingService(null));
        expect(IllegalStateException.class,
                () -> new FtcManualBulkCachingService(Collections.emptyList()));
        expect(IllegalArgumentException.class,
                () -> new FtcManualBulkCachingService(Arrays.asList(
                        new RecordingHub("a", LynxModule.BulkCachingMode.OFF), null)));

        List<String> events = new ArrayList<>();
        RecordingHub retained = new RecordingHub(
                "retained", LynxModule.BulkCachingMode.AUTO, events);
        RecordingHub replacement = new RecordingHub(
                "replacement", LynxModule.BulkCachingMode.OFF, events);
        List<FtcBulkCachingHub> source = new ArrayList<>();
        source.add(retained);

        FtcManualBulkCachingService service = new FtcManualBulkCachingService(source);
        assertTrue(events.isEmpty());
        source.clear();
        source.add(replacement);

        service.start(clock());
        assertEquals(Arrays.asList(
                "retained.read",
                "retained.set(MANUAL)",
                "retained.clear"), events);
        assertEquals(0, replacement.readCalls);
        assertEquals(0, replacement.setCalls);
        assertEquals(0, replacement.clearCalls);
    }

    @Test
    public void startSnapshotsAllModesBeforeMutationAndOwnsOneClearPerCycle() {
        List<String> events = new ArrayList<>();
        RecordingHub off = new RecordingHub("off", LynxModule.BulkCachingMode.OFF, events);
        RecordingHub auto = new RecordingHub("auto", LynxModule.BulkCachingMode.AUTO, events);
        RecordingHub manual = new RecordingHub(
                "manual", LynxModule.BulkCachingMode.MANUAL, events);
        FtcManualBulkCachingService service = service(off, auto, manual);
        LoopClock clock = clock();

        service.start(clock);
        assertEquals(Arrays.asList(
                "off.read", "auto.read", "manual.read",
                "off.set(MANUAL)", "auto.set(MANUAL)", "manual.set(MANUAL)",
                "off.clear", "auto.clear", "manual.clear"), events);
        assertSame(LynxModule.BulkCachingMode.MANUAL, off.mode);
        assertSame(LynxModule.BulkCachingMode.MANUAL, auto.mode);
        assertSame(LynxModule.BulkCachingMode.MANUAL, manual.mode);

        int startEventCount = events.size();
        service.update(clock);
        assertEquals(startEventCount, events.size());

        clock.update(1.0);
        service.update(clock);
        assertEquals(Arrays.asList(
                "off.clear", "auto.clear", "manual.clear"),
                events.subList(startEventCount, events.size()));

        int beforeStop = events.size();
        service.stop();
        assertEquals(Arrays.asList(
                "off.clear", "auto.clear", "manual.clear",
                "off.set(OFF)", "auto.set(AUTO)", "manual.set(MANUAL)"),
                events.subList(beforeStop, events.size()));
        assertSame(LynxModule.BulkCachingMode.OFF, off.mode);
        assertSame(LynxModule.BulkCachingMode.AUTO, auto.mode);
        assertSame(LynxModule.BulkCachingMode.MANUAL, manual.mode);

        int terminalEventCount = events.size();
        service.stop();
        assertEquals(terminalEventCount, events.size());
    }

    @Test
    public void snapshotFailureConsumesStartWithoutMutationOrCleanupEffects() {
        List<String> events = new ArrayList<>();
        RecordingHub first = new RecordingHub("first", LynxModule.BulkCachingMode.OFF, events);
        RecordingHub second = new RecordingHub("second", LynxModule.BulkCachingMode.AUTO, events);
        RecordingHub third = new RecordingHub("third", LynxModule.BulkCachingMode.MANUAL, events);
        RuntimeException snapshotFailure = new RuntimeException("snapshot");
        second.failRead(1, snapshotFailure);
        FtcManualBulkCachingService service = service(first, second, third);
        LoopClock clock = clock();

        assertSame(snapshotFailure, expect(RuntimeException.class, () -> service.start(clock)));
        assertEquals(Arrays.asList("first.read", "second.read"), events);
        IllegalStateException retryFailure = expect(
                IllegalStateException.class,
                () -> service.start(clock));
        assertContainsPublicOwner(retryFailure);
        assertEquals(2, events.size());

        service.stop();
        assertEquals(2, events.size());
    }

    @Test
    public void nullSnapshotConsumesStartWithoutMutationOrCleanupEffects() {
        List<String> events = new ArrayList<>();
        RecordingHub first = new RecordingHub("first", LynxModule.BulkCachingMode.OFF, events);
        RecordingHub second = new RecordingHub("second", null, events);
        RecordingHub third = new RecordingHub("third", LynxModule.BulkCachingMode.AUTO, events);
        FtcManualBulkCachingService service = service(first, second, third);
        LoopClock clock = clock();

        IllegalStateException nullMode = expect(
                IllegalStateException.class,
                () -> service.start(clock));
        assertContainsPublicOwner(nullMode);
        assertTrue(nullMode.getMessage().contains("index 1"));
        assertEquals(Arrays.asList("first.read", "second.read"), events);
        expect(IllegalStateException.class, () -> service.start(clock));
        service.stop();
        assertEquals(2, events.size());
    }

    @Test
    public void nullClocksAreRejectedBeforeLifecycleStateOrEffectsAreClaimed() {
        List<String> events = new ArrayList<>();
        RecordingHub hub = new RecordingHub("hub", LynxModule.BulkCachingMode.OFF, events);
        FtcManualBulkCachingService service = service(hub);
        LoopClock clock = clock();

        IllegalArgumentException nullStart = expect(
                IllegalArgumentException.class,
                () -> service.start(null));
        assertContainsPublicOwner(nullStart);
        assertTrue(events.isEmpty());

        IllegalArgumentException nullUpdate = expect(
                IllegalArgumentException.class,
                () -> service.update(null));
        assertContainsPublicOwner(nullUpdate);
        assertTrue(events.isEmpty());
        expect(IllegalStateException.class, () -> service.update(clock));
        assertTrue(events.isEmpty());

        service.start(clock);
        assertEquals(1, hub.readCalls);
        assertEquals(1, hub.setCalls);
        assertEquals(1, hub.clearCalls);
    }

    @Test
    public void lifecycleRejectsDuplicateDifferentRegressedAndTerminalCallsWithoutEffects()
            throws Exception {
        List<String> events = new ArrayList<>();
        RecordingHub hub = new RecordingHub("hub", LynxModule.BulkCachingMode.AUTO, events);
        FtcManualBulkCachingService service = service(hub);
        LoopClock clock = clock();
        service.start(clock);
        int afterStart = events.size();

        assertContainsPublicOwner(expect(
                IllegalStateException.class,
                () -> service.start(clock)));
        LoopClock differentClock = clock();
        assertContainsPublicOwner(expect(
                IllegalStateException.class,
                () -> service.update(differentClock)));
        assertEquals(afterStart, events.size());

        clock.update(1.0);
        service.update(clock);
        int afterUpdate = events.size();
        setCycle(clock, clock.cycle() - 1L);
        assertContainsPublicOwner(expect(
                IllegalStateException.class,
                () -> service.update(clock)));
        assertEquals(afterUpdate, events.size());

        service.stop();
        int afterStop = events.size();
        assertContainsPublicOwner(expect(
                IllegalStateException.class,
                () -> service.update(clock)));
        assertEquals(afterStop, events.size());
    }

    @Test
    public void setFailureHaltsStartAndCleanupCoversEveryCapturedHub() {
        List<String> events = new ArrayList<>();
        RecordingHub first = new RecordingHub("first", LynxModule.BulkCachingMode.OFF, events);
        RecordingHub second = new RecordingHub("second", LynxModule.BulkCachingMode.AUTO, events);
        RecordingHub third = new RecordingHub("third", LynxModule.BulkCachingMode.MANUAL, events);
        RuntimeException setFailure = new RuntimeException("set");
        second.failSet(1, setFailure);
        FtcManualBulkCachingService service = service(first, second, third);

        assertSame(setFailure, expect(
                RuntimeException.class,
                () -> service.start(clock())));
        assertEquals(Arrays.asList(
                "first.read", "second.read", "third.read",
                "first.set(MANUAL)", "second.set(MANUAL)"), events);

        events.clear();
        service.stop();
        assertEquals(Arrays.asList(
                "first.clear", "second.clear", "third.clear",
                "first.set(OFF)", "second.set(AUTO)", "third.set(MANUAL)"), events);
        assertSame(LynxModule.BulkCachingMode.OFF, first.mode);
        assertSame(LynxModule.BulkCachingMode.AUTO, second.mode);
        assertSame(LynxModule.BulkCachingMode.MANUAL, third.mode);
    }

    @Test
    public void clearFailureHaltsStartAndCleanupCoversEveryCapturedHub() {
        List<String> events = new ArrayList<>();
        RecordingHub first = new RecordingHub("first", LynxModule.BulkCachingMode.OFF, events);
        RecordingHub second = new RecordingHub("second", LynxModule.BulkCachingMode.AUTO, events);
        RecordingHub third = new RecordingHub("third", LynxModule.BulkCachingMode.MANUAL, events);
        RuntimeException clearFailure = new RuntimeException("clear");
        second.failClear(1, clearFailure);
        FtcManualBulkCachingService service = service(first, second, third);

        assertSame(clearFailure, expect(
                RuntimeException.class,
                () -> service.start(clock())));
        assertEquals(Arrays.asList(
                "first.read", "second.read", "third.read",
                "first.set(MANUAL)", "second.set(MANUAL)", "third.set(MANUAL)",
                "first.clear", "second.clear"), events);

        events.clear();
        service.stop();
        assertEquals(Arrays.asList(
                "first.clear", "second.clear", "third.clear",
                "first.set(OFF)", "second.set(AUTO)", "third.set(MANUAL)"), events);
    }

    @Test
    public void activeFailureAttemptsAllHubsRetainsSameCycleAndRetriesLater() {
        List<String> events = new ArrayList<>();
        RecordingHub first = new RecordingHub("first", LynxModule.BulkCachingMode.OFF, events);
        RecordingHub second = new RecordingHub("second", LynxModule.BulkCachingMode.AUTO, events);
        RecordingHub third = new RecordingHub("third", LynxModule.BulkCachingMode.MANUAL, events);
        FtcManualBulkCachingService service = service(first, second, third);
        LoopClock clock = clock();
        service.start(clock);
        events.clear();

        RuntimeException firstFailure = new RuntimeException("first-clear");
        RuntimeException thirdFailure = new RuntimeException("third-clear");
        first.failClear(2, firstFailure);
        third.failClear(2, thirdFailure);
        clock.update(1.0);

        RuntimeException thrown = expect(RuntimeException.class, () -> service.update(clock));
        assertSame(firstFailure, thrown);
        assertArrayEquals(new Throwable[]{thirdFailure}, thrown.getSuppressed());
        assertEquals(Arrays.asList("first.clear", "second.clear", "third.clear"), events);

        int failedCycleEvents = events.size();
        assertSame(thrown, expect(RuntimeException.class, () -> service.update(clock)));
        assertEquals(failedCycleEvents, events.size());

        clock.update(2.0);
        service.update(clock);
        assertEquals(Arrays.asList(
                "first.clear", "second.clear", "third.clear"),
                events.subList(failedCycleEvents, events.size()));
    }

    @Test
    public void reentrantStartFailsBeforeAnyInnerHubEffectAndMayBeCaughtByAdapter() {
        List<String> escapingEvents = new ArrayList<>();
        RecordingHub escapingHub = new RecordingHub(
                "escaping", LynxModule.BulkCachingMode.OFF, escapingEvents);
        FtcManualBulkCachingService[] escapingService = new FtcManualBulkCachingService[1];
        LoopClock escapingClock = clock();
        RuntimeException[] innerFailure = new RuntimeException[1];
        escapingHub.onRead(1, () -> {
            innerFailure[0] = expect(
                    IllegalStateException.class,
                    () -> escapingService[0].start(escapingClock));
            throw innerFailure[0];
        });
        escapingService[0] = service(escapingHub);

        RuntimeException outerFailure = expect(
                RuntimeException.class,
                () -> escapingService[0].start(escapingClock));
        assertSame(innerFailure[0], outerFailure);
        assertEquals(Collections.singletonList("escaping.read"), escapingEvents);

        List<String> caughtEvents = new ArrayList<>();
        RecordingHub caughtHub = new RecordingHub(
                "caught", LynxModule.BulkCachingMode.AUTO, caughtEvents);
        FtcManualBulkCachingService[] caughtService = new FtcManualBulkCachingService[1];
        LoopClock caughtClock = clock();
        caughtHub.onRead(1, () -> assertContainsPublicOwner(expect(
                IllegalStateException.class,
                () -> caughtService[0].start(caughtClock))));
        caughtService[0] = service(caughtHub);

        caughtService[0].start(caughtClock);
        assertEquals(Arrays.asList(
                "caught.read", "caught.set(MANUAL)", "caught.clear"), caughtEvents);
    }

    @Test
    public void escapingReentrantStartDuringSetBecomesOuterFailureThenCleansEveryHub() {
        List<String> events = new ArrayList<>();
        RecordingHub first = new RecordingHub("first", LynxModule.BulkCachingMode.OFF, events);
        RecordingHub second = new RecordingHub("second", LynxModule.BulkCachingMode.AUTO, events);
        RecordingHub third = new RecordingHub(
                "third", LynxModule.BulkCachingMode.MANUAL, events);
        FtcManualBulkCachingService[] owner = new FtcManualBulkCachingService[1];
        LoopClock clock = clock();
        RuntimeException[] innerFailure = new RuntimeException[1];
        first.onSet(1, () -> {
            innerFailure[0] = expect(
                    IllegalStateException.class,
                    () -> owner[0].start(clock));
            throw innerFailure[0];
        });
        owner[0] = service(first, second, third);

        RuntimeException outerFailure = expect(
                RuntimeException.class,
                () -> owner[0].start(clock));
        assertSame(innerFailure[0], outerFailure);
        assertEquals(
                "FtcBulkCaching.manual(...) service allows one START attempt; construct and "
                        + "declare a fresh FtcBulkCaching.manual(...) service instead of "
                        + "retrying START",
                outerFailure.getMessage());
        assertEquals(Arrays.asList(
                "first.read", "second.read", "third.read",
                "first.set(MANUAL)"), events);

        events.clear();
        owner[0].stop();
        assertEquals(Arrays.asList(
                "first.clear", "second.clear", "third.clear",
                "first.set(OFF)", "second.set(AUTO)", "third.set(MANUAL)"), events);
        assertSame(LynxModule.BulkCachingMode.OFF, first.mode);
        assertSame(LynxModule.BulkCachingMode.AUTO, second.mode);
        assertSame(LynxModule.BulkCachingMode.MANUAL, third.mode);
    }

    @Test
    public void updateReentryBecomesRetainedCycleFailureWithoutInnerHubEffect() {
        List<String> events = new ArrayList<>();
        RecordingHub first = new RecordingHub("first", LynxModule.BulkCachingMode.OFF, events);
        RecordingHub second = new RecordingHub("second", LynxModule.BulkCachingMode.AUTO, events);
        FtcManualBulkCachingService[] owner = new FtcManualBulkCachingService[1];
        LoopClock clock = clock();
        owner[0] = service(first, second);
        owner[0].start(clock);
        events.clear();

        RuntimeException[] innerFailure = new RuntimeException[1];
        first.onClear(2, () -> {
            innerFailure[0] = expect(
                    IllegalStateException.class,
                    () -> owner[0].update(clock));
            throw innerFailure[0];
        });
        clock.update(1.0);

        RuntimeException outerFailure = expect(
                RuntimeException.class,
                () -> owner[0].update(clock));
        assertSame(innerFailure[0], outerFailure);
        assertEquals(Arrays.asList("first.clear", "second.clear"), events);
        int effectCount = events.size();
        assertSame(outerFailure, expect(
                RuntimeException.class,
                () -> owner[0].update(clock)));
        assertEquals(effectCount, events.size());
    }

    @Test
    public void stopDuringSnapshotTerminalizesWithoutMutationOrCleanup() {
        List<String> events = new ArrayList<>();
        RecordingHub first = new RecordingHub("first", LynxModule.BulkCachingMode.OFF, events);
        RecordingHub second = new RecordingHub("second", LynxModule.BulkCachingMode.AUTO, events);
        FtcManualBulkCachingService[] owner = new FtcManualBulkCachingService[1];
        first.onRead(1, () -> owner[0].stop());
        owner[0] = service(first, second);

        IllegalStateException failure = expect(
                IllegalStateException.class,
                () -> owner[0].start(clock()));
        assertContainsPublicOwner(failure);
        assertTrue(failure.getMessage().contains("stopped reentrantly"));
        assertEquals(Collections.singletonList("first.read"), events);
        owner[0].stop();
        assertEquals(1, events.size());
    }

    @Test
    public void stopDuringSnapshotPreservesTheEscapingCallbackFailure() {
        List<String> events = new ArrayList<>();
        RecordingHub first = new RecordingHub("first", LynxModule.BulkCachingMode.OFF, events);
        RecordingHub second = new RecordingHub("second", LynxModule.BulkCachingMode.AUTO, events);
        RuntimeException callbackFailure = new RuntimeException("snapshot-callback");
        FtcManualBulkCachingService[] owner = new FtcManualBulkCachingService[1];
        first.onRead(1, () -> {
            owner[0].stop();
            throw callbackFailure;
        });
        owner[0] = service(first, second);

        assertSame(callbackFailure, expect(
                RuntimeException.class,
                () -> owner[0].start(clock())));
        assertEquals(Collections.singletonList("first.read"), events);
        assertEquals(0, callbackFailure.getSuppressed().length);
        owner[0].stop();
        assertEquals(1, events.size());
    }

    @Test
    public void stopDuringStartCallbackDefersCleanupAndHaltsOrdinaryStartPass() {
        List<String> events = new ArrayList<>();
        RecordingHub first = new RecordingHub("first", LynxModule.BulkCachingMode.OFF, events);
        RecordingHub second = new RecordingHub("second", LynxModule.BulkCachingMode.AUTO, events);
        FtcManualBulkCachingService[] owner = new FtcManualBulkCachingService[1];
        first.onSet(1, () -> owner[0].stop());
        owner[0] = service(first, second);

        IllegalStateException failure = expect(
                IllegalStateException.class,
                () -> owner[0].start(clock()));
        assertTrue(failure.getMessage().contains("stopped reentrantly"));
        assertEquals(Arrays.asList(
                "first.read", "second.read",
                "first.set(MANUAL)",
                "first.clear", "second.clear",
                "first.set(OFF)", "second.set(AUTO)"), events);
        assertSame(LynxModule.BulkCachingMode.OFF, first.mode);
        assertSame(LynxModule.BulkCachingMode.AUTO, second.mode);
    }

    @Test
    public void stopDuringUpdateDefersCleanupAndHaltsRemainingOrdinaryClears() {
        List<String> events = new ArrayList<>();
        RecordingHub first = new RecordingHub("first", LynxModule.BulkCachingMode.OFF, events);
        RecordingHub second = new RecordingHub("second", LynxModule.BulkCachingMode.AUTO, events);
        FtcManualBulkCachingService[] owner = new FtcManualBulkCachingService[1];
        LoopClock clock = clock();
        owner[0] = service(first, second);
        owner[0].start(clock);
        events.clear();

        first.onClear(2, () -> {
            events.add("callback.before-stop");
            owner[0].stop();
            events.add("callback.after-stop");
        });
        clock.update(1.0);
        IllegalStateException failure = expect(
                IllegalStateException.class,
                () -> owner[0].update(clock));

        assertTrue(failure.getMessage().contains("stopped reentrantly"));
        assertEquals(Arrays.asList(
                "first.clear", "callback.before-stop", "callback.after-stop",
                "first.clear", "second.clear",
                "first.set(OFF)", "second.set(AUTO)"), events);
    }

    @Test
    public void reentrantStopPreservesInFlightFailureAndSuppressesCleanupFailures() {
        List<String> events = new ArrayList<>();
        RecordingHub first = new RecordingHub("first", LynxModule.BulkCachingMode.OFF, events);
        RecordingHub second = new RecordingHub("second", LynxModule.BulkCachingMode.AUTO, events);
        FtcManualBulkCachingService[] owner = new FtcManualBulkCachingService[1];
        LoopClock clock = clock();
        owner[0] = service(first, second);
        owner[0].start(clock);
        events.clear();

        RuntimeException inFlight = new RuntimeException("in-flight");
        RuntimeException cleanupClear = new RuntimeException("cleanup-clear");
        RuntimeException cleanupRestore = new RuntimeException("cleanup-restore");
        first.onClear(2, () -> owner[0].stop());
        first.failClear(2, inFlight);
        second.failClear(2, cleanupClear);
        first.failSet(2, cleanupRestore);
        clock.update(1.0);

        RuntimeException thrown = expect(RuntimeException.class, () -> owner[0].update(clock));
        assertSame(inFlight, thrown);
        assertArrayEquals(
                new Throwable[]{cleanupClear, cleanupRestore},
                thrown.getSuppressed());
        assertEquals(Arrays.asList(
                "first.clear",
                "first.clear", "second.clear",
                "first.set(OFF)", "second.set(AUTO)"), events);
    }

    @Test
    public void reentrantStopMergesPrecedingHubFailureBeforeCallbackAndCleanupFailures() {
        List<String> events = new ArrayList<>();
        RecordingHub first = new RecordingHub("first", LynxModule.BulkCachingMode.OFF, events);
        RecordingHub second = new RecordingHub("second", LynxModule.BulkCachingMode.AUTO, events);
        RecordingHub third = new RecordingHub(
                "third", LynxModule.BulkCachingMode.MANUAL, events);
        FtcManualBulkCachingService[] owner = new FtcManualBulkCachingService[1];
        LoopClock clock = clock();
        owner[0] = service(first, second, third);
        owner[0].start(clock);
        events.clear();

        RuntimeException firstOrdinary = new RuntimeException("first-ordinary");
        RuntimeException secondInFlight = new RuntimeException("second-in-flight");
        RuntimeException firstCleanupClear = new RuntimeException("first-cleanup-clear");
        RuntimeException thirdCleanupClear = new RuntimeException("third-cleanup-clear");
        RuntimeException secondCleanupRestore = new RuntimeException(
                "second-cleanup-restore");
        first.failClear(2, firstOrdinary);
        second.onClear(2, () -> owner[0].stop());
        second.failClear(2, secondInFlight);
        first.failClear(3, firstCleanupClear);
        third.failClear(2, thirdCleanupClear);
        second.failSet(2, secondCleanupRestore);
        clock.update(1.0);

        RuntimeException thrown = expect(RuntimeException.class, () -> owner[0].update(clock));
        assertSame(firstOrdinary, thrown);
        assertArrayEquals(new Throwable[]{
                secondInFlight,
                firstCleanupClear,
                thirdCleanupClear,
                secondCleanupRestore
        }, thrown.getSuppressed());
        assertEquals(Arrays.asList(
                "first.clear", "second.clear",
                "first.clear", "second.clear", "third.clear",
                "first.set(OFF)", "second.set(AUTO)", "third.set(MANUAL)"), events);
        assertEquals(2, third.clearCalls);
        assertEquals(2, third.setCalls);
    }

    @Test
    public void reentrantStopUsesCleanupFailureAsPrimaryWhenCallbackSucceeds() {
        List<String> events = new ArrayList<>();
        RecordingHub first = new RecordingHub("first", LynxModule.BulkCachingMode.OFF, events);
        RecordingHub second = new RecordingHub("second", LynxModule.BulkCachingMode.AUTO, events);
        RuntimeException cleanupFailure = new RuntimeException("cleanup-clear");
        FtcManualBulkCachingService[] owner = new FtcManualBulkCachingService[1];
        LoopClock clock = clock();
        owner[0] = service(first, second);
        owner[0].start(clock);
        events.clear();

        first.onClear(2, () -> owner[0].stop());
        second.failClear(2, cleanupFailure);
        clock.update(1.0);

        assertSame(cleanupFailure, expect(
                RuntimeException.class,
                () -> owner[0].update(clock)));
        assertEquals(Arrays.asList(
                "first.clear",
                "first.clear", "second.clear",
                "first.set(OFF)", "second.set(AUTO)"), events);
        assertSame(LynxModule.BulkCachingMode.OFF, first.mode);
        assertSame(LynxModule.BulkCachingMode.AUTO, second.mode);
    }

    @Test
    public void cleanupUsesGlobalPassesAggregatesFailuresAndIgnoresReentrantStop() {
        List<String> events = new ArrayList<>();
        RecordingHub first = new RecordingHub("first", LynxModule.BulkCachingMode.OFF, events);
        RecordingHub second = new RecordingHub("second", LynxModule.BulkCachingMode.AUTO, events);
        FtcManualBulkCachingService[] owner = new FtcManualBulkCachingService[1];
        owner[0] = service(first, second);
        owner[0].start(clock());
        events.clear();

        RuntimeException firstClear = new RuntimeException("first-clear");
        RuntimeException secondClear = new RuntimeException("second-clear");
        RuntimeException firstRestore = new RuntimeException("first-restore");
        RuntimeException secondRestore = new RuntimeException("second-restore");
        first.onClear(2, () -> owner[0].stop());
        first.failClear(2, firstClear);
        second.failClear(2, secondClear);
        first.failSet(2, firstRestore);
        second.failSet(2, secondRestore);

        RuntimeException thrown = expect(RuntimeException.class, () -> owner[0].stop());
        assertSame(firstClear, thrown);
        assertArrayEquals(
                new Throwable[]{secondClear, firstRestore, secondRestore},
                thrown.getSuppressed());
        assertEquals(Arrays.asList(
                "first.clear", "second.clear",
                "first.set(OFF)", "second.set(AUTO)"), events);

        int effectCount = events.size();
        owner[0].stop();
        assertEquals(effectCount, events.size());
    }

    @Test
    public void stopBeforeStartHasNoEffectAndTerminalizesTheSingleUseService() {
        List<String> events = new ArrayList<>();
        RecordingHub hub = new RecordingHub("hub", LynxModule.BulkCachingMode.AUTO, events);
        FtcManualBulkCachingService service = service(hub);
        LoopClock clock = clock();

        service.stop();
        service.stop();
        assertTrue(events.isEmpty());
        assertContainsPublicOwner(expect(
                IllegalStateException.class,
                () -> service.start(clock)));
        assertContainsPublicOwner(expect(
                IllegalStateException.class,
                () -> service.update(clock)));
        assertTrue(events.isEmpty());
    }

    private static FtcManualBulkCachingService service(RecordingHub... hubs) {
        return new FtcManualBulkCachingService(Arrays.asList(hubs));
    }

    private static LoopClock clock() {
        LoopClock clock = new LoopClock();
        clock.reset(0.0);
        return clock;
    }

    private static void setCycle(LoopClock clock, long cycle) throws Exception {
        Field cycleField = LoopClock.class.getDeclaredField("cycle");
        cycleField.setAccessible(true);
        cycleField.setLong(clock, cycle);
    }

    private static void assertContainsPublicOwner(RuntimeException failure) {
        assertTrue(failure.getMessage(),
                failure.getMessage().contains("FtcBulkCaching.manual(...)"));
    }

    private static <T extends RuntimeException> T expect(
            Class<T> expectedType,
            Runnable operation) {
        try {
            operation.run();
            fail("Expected " + expectedType.getSimpleName());
            throw new AssertionError("unreachable");
        } catch (RuntimeException failure) {
            if (!expectedType.isInstance(failure)) {
                throw failure;
            }
            return expectedType.cast(failure);
        }
    }

    private static final class DiscoveryHardwareMap extends HardwareMap {
        private final List<?> discovered;
        private final RuntimeException failure;
        int calls;
        Class<?> requestedType;

        private DiscoveryHardwareMap(List<?> discovered, RuntimeException failure) {
            super(null, null);
            this.discovered = discovered;
            this.failure = failure;
        }

        static DiscoveryHardwareMap returning(List<?> discovered) {
            return new DiscoveryHardwareMap(discovered, null);
        }

        static DiscoveryHardwareMap throwing(RuntimeException failure) {
            return new DiscoveryHardwareMap(null, failure);
        }

        @Override
        @SuppressWarnings("unchecked")
        public <T> List<T> getAll(Class<? extends T> classOrInterface) {
            calls++;
            requestedType = classOrInterface;
            if (failure != null) {
                throw failure;
            }
            return (List<T>) discovered;
        }
    }

    private static final class RecordingHub implements FtcBulkCachingHub {
        final String name;
        final List<String> events;
        final Map<Integer, RuntimeException> readFailures = new HashMap<>();
        final Map<Integer, RuntimeException> setFailures = new HashMap<>();
        final Map<Integer, RuntimeException> clearFailures = new HashMap<>();
        final Map<Integer, Runnable> readCallbacks = new HashMap<>();
        final Map<Integer, Runnable> setCallbacks = new HashMap<>();
        final Map<Integer, Runnable> clearCallbacks = new HashMap<>();
        LynxModule.BulkCachingMode mode;
        int readCalls;
        int setCalls;
        int clearCalls;

        RecordingHub(String name, LynxModule.BulkCachingMode mode) {
            this(name, mode, new ArrayList<>());
        }

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
            int call = ++readCalls;
            events.add(name + ".read");
            run(readCallbacks.get(call));
            throwIfPresent(readFailures.get(call));
            return mode;
        }

        @Override
        public void setMode(LynxModule.BulkCachingMode requestedMode) {
            int call = ++setCalls;
            events.add(name + ".set(" + requestedMode + ")");
            run(setCallbacks.get(call));
            throwIfPresent(setFailures.get(call));
            mode = requestedMode;
        }

        @Override
        public void clearCache() {
            int call = ++clearCalls;
            events.add(name + ".clear");
            run(clearCallbacks.get(call));
            throwIfPresent(clearFailures.get(call));
        }

        void failRead(int call, RuntimeException failure) {
            readFailures.put(call, failure);
        }

        void failSet(int call, RuntimeException failure) {
            setFailures.put(call, failure);
        }

        void failClear(int call, RuntimeException failure) {
            clearFailures.put(call, failure);
        }

        void onRead(int call, Runnable callback) {
            readCallbacks.put(call, callback);
        }

        void onSet(int call, Runnable callback) {
            setCallbacks.put(call, callback);
        }

        void onClear(int call, Runnable callback) {
            clearCallbacks.put(call, callback);
        }

        private static void run(Runnable callback) {
            if (callback != null) {
                callback.run();
            }
        }

        private static void throwIfPresent(RuntimeException failure) {
            if (failure != null) {
                throw failure;
            }
        }
    }
}
