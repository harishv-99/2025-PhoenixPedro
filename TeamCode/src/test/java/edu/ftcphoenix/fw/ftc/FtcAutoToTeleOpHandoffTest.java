package edu.ftcphoenix.fw.ftc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.LongSupplier;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the typed, one-shot, process-local Auto-to-TeleOp handoff contract. */
public final class FtcAutoToTeleOpHandoffTest {

    private static final double EPSILON = 1e-12;

    @Test
    public void publicApiHasOneFactoryAndNoPublicConstructor() {
        for (Constructor<?> constructor
                : FtcAutoToTeleOpHandoff.class.getDeclaredConstructors()) {
            assertTrue(Modifier.isPrivate(constructor.getModifiers()));
        }

        List<String> publicStaticMethods = new ArrayList<>();
        for (Method method : FtcAutoToTeleOpHandoff.class.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers())
                    && Modifier.isStatic(method.getModifiers())) {
                publicStaticMethods.add(method.getName());
            }
        }
        assertEquals(Collections.singletonList("create"), publicStaticMethods);
    }

    @Test
    public void creationRejectsInvalidConfiguration() {
        assertThrowsContains(
                IllegalArgumentException.class,
                () -> FtcAutoToTeleOpHandoff.create(null, Snapshot.class, 1.0),
                "debugName",
                "nonblank");
        assertThrowsContains(
                IllegalArgumentException.class,
                () -> FtcAutoToTeleOpHandoff.create("  ", Snapshot.class, 1.0),
                "debugName",
                "nonblank");
        assertThrowsContains(
                NullPointerException.class,
                () -> FtcAutoToTeleOpHandoff.create("match", null, 1.0),
                "payloadType");
        assertThrowsContains(
                IllegalArgumentException.class,
                () -> FtcAutoToTeleOpHandoff.create("match", int.class, 1.0),
                "payloadType",
                "reference type",
                "primitive int");
        assertInvalidAge(0.0);
        assertInvalidAge(-1.0);
        assertInvalidAge(Double.NaN);
        assertInvalidAge(Double.POSITIVE_INFINITY);
        assertInvalidAge(Double.NEGATIVE_INFINITY);
    }

    @Test
    public void freshPayloadIsDeliveredExactlyOnceWithDiagnostics() {
        MutableNanoClock clock = new MutableNanoClock(1_000_000_000L);
        FtcAutoToTeleOpHandoff<Snapshot> handoff = handoff(clock, 2.0);
        AutoMode producer = new AutoMode();
        TeleOpMode consumer = new TeleOpMode();
        Snapshot snapshot = new Snapshot("final field pose");

        handoff.publishFromAuto(producer, snapshot);
        clock.nowNanos = 2_250_000_000L;

        FtcAutoToTeleOpHandoff.ConsumeResult<Snapshot> delivered =
                handoff.consumeForTeleOp(consumer);
        assertEquals(FtcAutoToTeleOpHandoff.ConsumeStatus.DELIVERED, delivered.status());
        assertSame(snapshot, delivered.payloadOrNull());
        assertEquals(AutoMode.class.getName(), delivered.producerClassNameOrNull());
        assertEquals(TeleOpMode.class.getName(), delivered.consumerClassName());
        assertEquals(1.25, delivered.ageSec(), EPSILON);

        FtcAutoToTeleOpHandoff.ConsumeResult<Snapshot> repeated =
                handoff.consumeForTeleOp(new OtherTeleOpMode());
        assertEquals(
                FtcAutoToTeleOpHandoff.ConsumeStatus.ALREADY_CONSUMED,
                repeated.status());
        assertNull(repeated.payloadOrNull());
        assertEquals(AutoMode.class.getName(), repeated.producerClassNameOrNull());
        assertEquals(OtherTeleOpMode.class.getName(), repeated.consumerClassName());
        assertTrue(Double.isNaN(repeated.ageSec()));
    }

    @Test
    public void firstMissingConsumeClosesCycleUntilExplicitClear() {
        MutableNanoClock clock = new MutableNanoClock(0L);
        FtcAutoToTeleOpHandoff<Snapshot> handoff = handoff(clock, 1.0);

        FtcAutoToTeleOpHandoff.ConsumeResult<Snapshot> missing =
                handoff.consumeForTeleOp(new TeleOpMode());
        assertEquals(FtcAutoToTeleOpHandoff.ConsumeStatus.MISSING, missing.status());
        assertNull(missing.payloadOrNull());
        assertNull(missing.producerClassNameOrNull());
        assertEquals(TeleOpMode.class.getName(), missing.consumerClassName());
        assertTrue(Double.isNaN(missing.ageSec()));

        assertEquals(
                FtcAutoToTeleOpHandoff.ConsumeStatus.ALREADY_CONSUMED,
                handoff.consumeForTeleOp(new TeleOpMode()).status());
        assertThrowsContains(
                IllegalStateException.class,
                () -> handoff.publishFromAuto(new AutoMode(), new Snapshot("too late")),
                "already consumed",
                "clear()");

        handoff.clear();
        Snapshot next = new Snapshot("next match");
        handoff.publishFromAuto(new AutoMode(), next);
        assertSame(next, handoff.consumeForTeleOp(new TeleOpMode()).payloadOrNull());
    }

    @Test
    public void clearDiscardsPendingPayloadAndOpensEmptyCycle() {
        MutableNanoClock clock = new MutableNanoClock(0L);
        FtcAutoToTeleOpHandoff<Snapshot> handoff = handoff(clock, 1.0);
        handoff.publishFromAuto(new AutoMode(), new Snapshot("discard me"));

        handoff.clear();

        FtcAutoToTeleOpHandoff.ConsumeResult<Snapshot> result =
                handoff.consumeForTeleOp(new TeleOpMode());
        assertEquals(FtcAutoToTeleOpHandoff.ConsumeStatus.MISSING, result.status());
        assertNull(result.producerClassNameOrNull());
    }

    @Test
    public void duplicatePublishFailsWithoutReplacingFirstPayload() {
        MutableNanoClock clock = new MutableNanoClock(0L);
        FtcAutoToTeleOpHandoff<Snapshot> handoff = handoff(clock, 1.0);
        Snapshot first = new Snapshot("first");
        handoff.publishFromAuto(new AutoMode(), first);

        assertThrowsContains(
                IllegalStateException.class,
                () -> handoff.publishFromAuto(new OtherAutoMode(), new Snapshot("second")),
                "already has",
                "clear()");

        assertSame(first, handoff.consumeForTeleOp(new TeleOpMode()).payloadOrNull());
    }

    @Test
    public void exactMaximumAgeIsAcceptedAndGreaterAgeIsStale() {
        MutableNanoClock clock = new MutableNanoClock(4_000_000_000L);
        FtcAutoToTeleOpHandoff<Snapshot> handoff = handoff(clock, 1.5);
        Snapshot exact = new Snapshot("exact boundary");
        handoff.publishFromAuto(new AutoMode(), exact);
        clock.nowNanos = 5_500_000_000L;

        FtcAutoToTeleOpHandoff.ConsumeResult<Snapshot> accepted =
                handoff.consumeForTeleOp(new TeleOpMode());
        assertEquals(FtcAutoToTeleOpHandoff.ConsumeStatus.DELIVERED, accepted.status());
        assertSame(exact, accepted.payloadOrNull());
        assertEquals(1.5, accepted.ageSec(), EPSILON);

        handoff.clear();
        clock.nowNanos = 10_000_000_000L;
        handoff.publishFromAuto(new AutoMode(), new Snapshot("stale"));
        clock.nowNanos = 11_500_000_001L;

        FtcAutoToTeleOpHandoff.ConsumeResult<Snapshot> stale =
                handoff.consumeForTeleOp(new TeleOpMode());
        assertEquals(FtcAutoToTeleOpHandoff.ConsumeStatus.STALE, stale.status());
        assertNull(stale.payloadOrNull());
        assertEquals(AutoMode.class.getName(), stale.producerClassNameOrNull());
        assertTrue(stale.ageSec() > 1.5);
        assertEquals(
                FtcAutoToTeleOpHandoff.ConsumeStatus.ALREADY_CONSUMED,
                handoff.consumeForTeleOp(new TeleOpMode()).status());
    }

    @Test
    public void regressingMonotonicClockIsStale() {
        MutableNanoClock clock = new MutableNanoClock(2_000L);
        FtcAutoToTeleOpHandoff<Snapshot> handoff = handoff(clock, 10.0);
        handoff.publishFromAuto(new AutoMode(), new Snapshot("future"));
        clock.nowNanos = 1_999L;

        FtcAutoToTeleOpHandoff.ConsumeResult<Snapshot> result =
                handoff.consumeForTeleOp(new TeleOpMode());

        assertEquals(FtcAutoToTeleOpHandoff.ConsumeStatus.STALE, result.status());
        assertNull(result.payloadOrNull());
        assertTrue(result.ageSec() < 0.0);
    }

    @SuppressWarnings({"rawtypes", "unchecked"})
    @Test
    public void rawGenericWrongTypeFailsBeforeMutatingState() {
        MutableNanoClock clock = new MutableNanoClock(0L);
        FtcAutoToTeleOpHandoff<Snapshot> typed = handoff(clock, 1.0);
        FtcAutoToTeleOpHandoff raw = typed;

        assertThrowsContains(
                IllegalArgumentException.class,
                () -> raw.publishFromAuto(new AutoMode(), "wrong type"),
                Snapshot.class.getName(),
                String.class.getName());

        Snapshot valid = new Snapshot("valid after rejection");
        typed.publishFromAuto(new AutoMode(), valid);
        assertSame(valid, typed.consumeForTeleOp(new TeleOpMode()).payloadOrNull());
    }

    @Test
    public void invalidCallArgumentsDoNotMutateOpenOrPublishedState() {
        MutableNanoClock clock = new MutableNanoClock(0L);
        FtcAutoToTeleOpHandoff<Snapshot> handoff = handoff(clock, 1.0);

        assertThrowsContains(
                NullPointerException.class,
                () -> handoff.publishFromAuto(null, new Snapshot("value")),
                "producer OpMode");
        assertThrowsContains(
                NullPointerException.class,
                () -> handoff.publishFromAuto(new AutoMode(), null),
                "immutable payload");
        assertThrowsContains(
                NullPointerException.class,
                () -> handoff.consumeForTeleOp(null),
                "consumer OpMode");

        Snapshot valid = new Snapshot("still open");
        handoff.publishFromAuto(new AutoMode(), valid);
        assertThrowsContains(
                NullPointerException.class,
                () -> handoff.consumeForTeleOp(null),
                "consumer OpMode");
        assertSame(valid, handoff.consumeForTeleOp(new TeleOpMode()).payloadOrNull());
    }

    @Test
    public void instancesDoNotShareState() {
        MutableNanoClock clock = new MutableNanoClock(0L);
        FtcAutoToTeleOpHandoff<Snapshot> first = handoff(clock, 1.0);
        FtcAutoToTeleOpHandoff<Snapshot> second =
                FtcAutoToTeleOpHandoff.createForTest(
                        "other match",
                        Snapshot.class,
                        1.0,
                        clock);
        Snapshot payload = new Snapshot("first only");
        first.publishFromAuto(new AutoMode(), payload);

        assertEquals(
                FtcAutoToTeleOpHandoff.ConsumeStatus.MISSING,
                second.consumeForTeleOp(new TeleOpMode()).status());
        assertSame(payload, first.consumeForTeleOp(new TeleOpMode()).payloadOrNull());
    }

    @Test
    public void carrierStorageDeclaresNoProducerOrConsumerOpModeReference() {
        assertDeclaresNoOpModeField(FtcAutoToTeleOpHandoff.class);
        for (Class<?> nestedType : FtcAutoToTeleOpHandoff.class.getDeclaredClasses()) {
            assertDeclaresNoOpModeField(nestedType);
        }
    }

    @Test
    public void concurrentPublishAndConsumeHaveOnlyLinearizedOutcomes() throws Exception {
        ExecutorService pool = Executors.newFixedThreadPool(2);
        try {
            for (int iteration = 0; iteration < 100; iteration++) {
                MutableNanoClock clock = new MutableNanoClock(iteration);
                FtcAutoToTeleOpHandoff<Snapshot> handoff = handoff(clock, 1.0);
                Snapshot payload = new Snapshot("publish-consume-" + iteration);
                CountDownLatch ready = new CountDownLatch(2);
                CountDownLatch start = new CountDownLatch(1);

                Future<Boolean> publication = pool.submit(() -> {
                    ready.countDown();
                    await(start);
                    try {
                        handoff.publishFromAuto(new AutoMode(), payload);
                        return true;
                    } catch (IllegalStateException consumedFirst) {
                        return false;
                    }
                });
                Future<FtcAutoToTeleOpHandoff.ConsumeResult<Snapshot>> consumption =
                        pool.submit(() -> {
                            ready.countDown();
                            await(start);
                            return handoff.consumeForTeleOp(new TeleOpMode());
                        });

                assertTrue(ready.await(5, TimeUnit.SECONDS));
                start.countDown();
                boolean publishWon = publication.get(5, TimeUnit.SECONDS);
                FtcAutoToTeleOpHandoff.ConsumeResult<Snapshot> result =
                        consumption.get(5, TimeUnit.SECONDS);

                if (publishWon) {
                    assertEquals(
                            FtcAutoToTeleOpHandoff.ConsumeStatus.DELIVERED,
                            result.status());
                    assertSame(payload, result.payloadOrNull());
                } else {
                    assertEquals(
                            FtcAutoToTeleOpHandoff.ConsumeStatus.MISSING,
                            result.status());
                    assertNull(result.payloadOrNull());
                }
                assertEquals(
                        FtcAutoToTeleOpHandoff.ConsumeStatus.ALREADY_CONSUMED,
                        handoff.consumeForTeleOp(new TeleOpMode()).status());
            }
        } finally {
            pool.shutdownNow();
        }
    }

    @Test
    public void concurrentClearAndPublishEitherRetainOrDiscardTheWholePublication()
            throws Exception {
        ExecutorService pool = Executors.newFixedThreadPool(2);
        try {
            for (int iteration = 0; iteration < 100; iteration++) {
                MutableNanoClock clock = new MutableNanoClock(iteration);
                FtcAutoToTeleOpHandoff<Snapshot> handoff = handoff(clock, 1.0);
                Snapshot payload = new Snapshot("clear-publish-" + iteration);
                CountDownLatch ready = new CountDownLatch(2);
                CountDownLatch start = new CountDownLatch(1);

                Future<?> clearing = pool.submit(() -> {
                    ready.countDown();
                    await(start);
                    handoff.clear();
                });
                Future<?> publication = pool.submit(() -> {
                    ready.countDown();
                    await(start);
                    handoff.publishFromAuto(new AutoMode(), payload);
                });

                assertTrue(ready.await(5, TimeUnit.SECONDS));
                start.countDown();
                clearing.get(5, TimeUnit.SECONDS);
                publication.get(5, TimeUnit.SECONDS);

                FtcAutoToTeleOpHandoff.ConsumeResult<Snapshot> result =
                        handoff.consumeForTeleOp(new TeleOpMode());
                if (result.status() == FtcAutoToTeleOpHandoff.ConsumeStatus.DELIVERED) {
                    assertSame(payload, result.payloadOrNull());
                } else {
                    assertEquals(
                            FtcAutoToTeleOpHandoff.ConsumeStatus.MISSING,
                            result.status());
                    assertNull(result.payloadOrNull());
                }
            }
        } finally {
            pool.shutdownNow();
        }
    }

    @Test
    public void clearDuringInFlightPublishRetriesIntoTheNewCycleWithFreshTime()
            throws Exception {
        BlockingReadClock clock = new BlockingReadClock(0L, 1);
        FtcAutoToTeleOpHandoff<Snapshot> handoff = handoff(clock, 1.0);
        Snapshot payload = new Snapshot("new cycle");
        ExecutorService pool = Executors.newSingleThreadExecutor();
        try {
            Future<?> publication =
                    pool.submit(() -> handoff.publishFromAuto(new AutoMode(), payload));
            assertTrue(clock.blockedReadEntered.await(5, TimeUnit.SECONDS));

            handoff.clear();
            clock.nowNanos = 2_000_000_000L;
            clock.releaseBlockedRead.countDown();
            publication.get(5, TimeUnit.SECONDS);

            FtcAutoToTeleOpHandoff.ConsumeResult<Snapshot> result =
                    handoff.consumeForTeleOp(new TeleOpMode());
            assertEquals(FtcAutoToTeleOpHandoff.ConsumeStatus.DELIVERED, result.status());
            assertSame(payload, result.payloadOrNull());
            assertEquals(0.0, result.ageSec(), EPSILON);
        } finally {
            clock.releaseBlockedRead.countDown();
            pool.shutdownNow();
        }
    }

    @Test
    public void clearDuringInFlightConsumePreventsOldStateCasFromCrossingCycles()
            throws Exception {
        BlockingReadClock clock = new BlockingReadClock(0L, 2);
        FtcAutoToTeleOpHandoff<Snapshot> handoff = handoff(clock, 1.0);
        handoff.publishFromAuto(new AutoMode(), new Snapshot("old cycle"));
        ExecutorService pool = Executors.newSingleThreadExecutor();
        try {
            Future<FtcAutoToTeleOpHandoff.ConsumeResult<Snapshot>> consumption =
                    pool.submit(() -> handoff.consumeForTeleOp(new TeleOpMode()));
            assertTrue(clock.blockedReadEntered.await(5, TimeUnit.SECONDS));

            handoff.clear();
            clock.releaseBlockedRead.countDown();

            FtcAutoToTeleOpHandoff.ConsumeResult<Snapshot> result =
                    consumption.get(5, TimeUnit.SECONDS);
            assertEquals(FtcAutoToTeleOpHandoff.ConsumeStatus.MISSING, result.status());
            assertNull(result.payloadOrNull());
            assertNull(result.producerClassNameOrNull());
            assertEquals(
                    FtcAutoToTeleOpHandoff.ConsumeStatus.ALREADY_CONSUMED,
                    handoff.consumeForTeleOp(new TeleOpMode()).status());
        } finally {
            clock.releaseBlockedRead.countDown();
            pool.shutdownNow();
        }
    }

    @Test
    public void concurrentPublishAcceptsExactlyOnePayload() throws Exception {
        MutableNanoClock clock = new MutableNanoClock(0L);
        FtcAutoToTeleOpHandoff<Snapshot> handoff = handoff(clock, 1.0);
        int contenders = 8;
        ExecutorService pool = Executors.newFixedThreadPool(contenders);
        CountDownLatch ready = new CountDownLatch(contenders);
        CountDownLatch start = new CountDownLatch(1);
        AtomicInteger successes = new AtomicInteger();
        AtomicInteger lifecycleFailures = new AtomicInteger();
        List<Snapshot> candidates = new ArrayList<>();
        List<Future<?>> futures = new ArrayList<>();
        try {
            for (int i = 0; i < contenders; i++) {
                Snapshot candidate = new Snapshot("candidate-" + i);
                candidates.add(candidate);
                futures.add(pool.submit(() -> {
                    ready.countDown();
                    await(start);
                    try {
                        handoff.publishFromAuto(new AutoMode(), candidate);
                        successes.incrementAndGet();
                    } catch (IllegalStateException expected) {
                        lifecycleFailures.incrementAndGet();
                    }
                }));
            }
            assertTrue(ready.await(5, TimeUnit.SECONDS));
            start.countDown();
            awaitAll(futures);
        } finally {
            pool.shutdownNow();
        }

        assertEquals(1, successes.get());
        assertEquals(contenders - 1, lifecycleFailures.get());
        Snapshot delivered =
                handoff.consumeForTeleOp(new TeleOpMode()).payloadOrNull();
        assertTrue(candidates.contains(delivered));
    }

    @Test
    public void concurrentConsumeDeliversToExactlyOneCaller() throws Exception {
        MutableNanoClock clock = new MutableNanoClock(0L);
        FtcAutoToTeleOpHandoff<Snapshot> handoff = handoff(clock, 1.0);
        Snapshot payload = new Snapshot("one winner");
        handoff.publishFromAuto(new AutoMode(), payload);

        int contenders = 8;
        ExecutorService pool = Executors.newFixedThreadPool(contenders);
        CountDownLatch ready = new CountDownLatch(contenders);
        CountDownLatch start = new CountDownLatch(1);
        List<Future<FtcAutoToTeleOpHandoff.ConsumeResult<Snapshot>>> futures =
                new ArrayList<>();
        try {
            for (int i = 0; i < contenders; i++) {
                futures.add(pool.submit(() -> {
                    ready.countDown();
                    await(start);
                    return handoff.consumeForTeleOp(new TeleOpMode());
                }));
            }
            assertTrue(ready.await(5, TimeUnit.SECONDS));
            start.countDown();

            int deliveredCount = 0;
            int alreadyConsumedCount = 0;
            for (Future<FtcAutoToTeleOpHandoff.ConsumeResult<Snapshot>> future : futures) {
                FtcAutoToTeleOpHandoff.ConsumeResult<Snapshot> result =
                        future.get(5, TimeUnit.SECONDS);
                if (result.status() == FtcAutoToTeleOpHandoff.ConsumeStatus.DELIVERED) {
                    deliveredCount++;
                    assertSame(payload, result.payloadOrNull());
                } else if (result.status()
                        == FtcAutoToTeleOpHandoff.ConsumeStatus.ALREADY_CONSUMED) {
                    alreadyConsumedCount++;
                    assertNull(result.payloadOrNull());
                } else {
                    fail("Unexpected concurrent result: " + result.status());
                }
            }
            assertEquals(1, deliveredCount);
            assertEquals(contenders - 1, alreadyConsumedCount);
        } finally {
            pool.shutdownNow();
        }
    }

    private static FtcAutoToTeleOpHandoff<Snapshot> handoff(LongSupplier clock,
                                                             double maxAgeSec) {
        return FtcAutoToTeleOpHandoff.createForTest(
                "Phoenix match handoff",
                Snapshot.class,
                maxAgeSec,
                clock);
    }

    private static void assertInvalidAge(double ageSec) {
        assertThrowsContains(
                IllegalArgumentException.class,
                () -> FtcAutoToTeleOpHandoff.create(
                        "match",
                        Snapshot.class,
                        ageSec),
                "maxAgeSec",
                "finite",
                "greater than zero");
    }

    private static void assertDeclaresNoOpModeField(Class<?> type) {
        for (Field field : type.getDeclaredFields()) {
            assertTrue(
                    type.getName() + "." + field.getName()
                            + " must not retain a producer or consumer OpMode",
                    !OpMode.class.isAssignableFrom(field.getType()));
        }
    }

    private static void assertThrowsContains(Class<? extends Throwable> expectedType,
                                             Runnable action,
                                             String... expectedMessageParts) {
        try {
            action.run();
            fail("Expected " + expectedType.getSimpleName());
        } catch (Throwable thrown) {
            assertTrue(
                    "Expected " + expectedType.getName() + " but got " + thrown,
                    expectedType.isInstance(thrown));
            String message = thrown.getMessage();
            for (String part : expectedMessageParts) {
                assertTrue(
                        "Expected message containing \"" + part + "\" but got: " + message,
                        message != null && message.contains(part));
            }
        }
    }

    private static void await(CountDownLatch latch) {
        try {
            if (!latch.await(5, TimeUnit.SECONDS)) {
                throw new AssertionError("Timed out waiting for concurrent test gate");
            }
        } catch (InterruptedException interrupted) {
            Thread.currentThread().interrupt();
            throw new AssertionError("Interrupted while waiting for concurrent test gate", interrupted);
        }
    }

    private static void awaitAll(List<Future<?>> futures) throws Exception {
        for (Future<?> future : futures) {
            future.get(5, TimeUnit.SECONDS);
        }
    }

    private static final class MutableNanoClock implements LongSupplier {
        volatile long nowNanos;

        MutableNanoClock(long nowNanos) {
            this.nowNanos = nowNanos;
        }

        @Override
        public long getAsLong() {
            return nowNanos;
        }
    }

    private static final class BlockingReadClock implements LongSupplier {
        private volatile long nowNanos;
        private final int blockedReadNumber;
        private final AtomicInteger reads = new AtomicInteger();
        private final CountDownLatch blockedReadEntered = new CountDownLatch(1);
        private final CountDownLatch releaseBlockedRead = new CountDownLatch(1);

        BlockingReadClock(long nowNanos, int blockedReadNumber) {
            this.nowNanos = nowNanos;
            this.blockedReadNumber = blockedReadNumber;
        }

        @Override
        public long getAsLong() {
            long capturedNanos = nowNanos;
            if (reads.incrementAndGet() == blockedReadNumber) {
                blockedReadEntered.countDown();
                await(releaseBlockedRead);
            }
            return capturedNanos;
        }
    }

    private static final class Snapshot {
        final String description;

        Snapshot(String description) {
            this.description = description;
        }
    }

    private static class AutoMode extends OpMode {
        @Override
        public void init() {
        }

        @Override
        public void loop() {
        }
    }

    private static final class OtherAutoMode extends AutoMode {
    }

    private static class TeleOpMode extends OpMode {
        @Override
        public void init() {
        }

        @Override
        public void loop() {
        }
    }

    private static final class OtherTeleOpMode extends TeleOpMode {
    }
}
