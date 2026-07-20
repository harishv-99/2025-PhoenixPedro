package edu.ftcphoenix.fw.ftc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.Objects;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.LongSupplier;

/**
 * Carries one immutable, short-lived value from an FTC Auto OpMode to the following TeleOp.
 *
 * <p>The FTC SDK creates a new {@link OpMode} and robot runtime for TeleOp, so ordinary instance
 * fields cannot carry the final Auto state across that boundary. A robot should retain one handoff
 * instance in a private robot-owned wrapper, call {@link #clear()} when a new match Auto begins,
 * publish only after Auto cleanup succeeds, and call {@link #consumeForTeleOp(OpMode)} once during
 * TeleOp initialization.</p>
 *
 * <p>This carrier is deliberately process-local. It does not use the FTC blackboard, a global
 * registry, or persistent storage. Restarting the Robot Controller process, reloading the
 * classloader, or redeploying the app discards the handoff. Freshness uses
 * {@link System#nanoTime()} rather than a Phoenix loop clock because Auto and TeleOp do not share
 * one loop heartbeat.</p>
 *
 * <p>The payload must be immutable or a defensive snapshot. This class stores the supplied
 * reference and cannot deep-copy an arbitrary {@code T}. The producer and consumer OpMode
 * arguments are used only to capture class names for diagnostics; they are neither retained nor
 * authenticated. The robot-owned wrapper remains responsible for deciding which OpModes may
 * publish and consume.</p>
 *
 * @param <T> immutable robot-owned snapshot type
 */
public final class FtcAutoToTeleOpHandoff<T> {

    private static final double NANOS_PER_SECOND = 1_000_000_000.0;

    private final String debugName;
    private final Class<T> payloadType;
    private final double maxAgeSec;
    private final LongSupplier nanoTimeSource;
    private final AtomicReference<State<T>> state;

    private FtcAutoToTeleOpHandoff(String debugName,
                                   Class<T> payloadType,
                                   double maxAgeSec,
                                   LongSupplier nanoTimeSource) {
        this.debugName = debugName;
        this.payloadType = payloadType;
        this.maxAgeSec = maxAgeSec;
        this.nanoTimeSource = nanoTimeSource;
        this.state = new AtomicReference<>(State.empty());
    }

    /**
     * Create one isolated, process-local Auto-to-TeleOp handoff.
     *
     * <p>The new handoff begins with one open, empty cycle. A long-lived robot wrapper should still
     * call {@link #clear()} at the beginning of every new Auto so an earlier match cannot supply
     * data to the current one.</p>
     *
     * @param debugName nonblank robot-facing name used in actionable errors
     * @param payloadType runtime type token used to reject raw-generic misuse before state changes
     * @param maxAgeSec maximum accepted age in seconds; an age exactly at this boundary is accepted
     * @param <T> immutable robot-owned snapshot type
     * @return a new handoff whose state is isolated from every other handoff
     * @throws IllegalArgumentException if {@code debugName} is blank, {@code payloadType} is a
     *                                  primitive type, or {@code maxAgeSec} is not finite and
     *                                  positive
     * @throws NullPointerException if {@code payloadType} is null
     */
    public static <T> FtcAutoToTeleOpHandoff<T> create(String debugName,
                                                       Class<T> payloadType,
                                                       double maxAgeSec) {
        return createForTest(debugName, payloadType, maxAgeSec, System::nanoTime);
    }

    /**
     * Create a handoff with a controllable monotonic clock for package-local contract tests.
     */
    static <T> FtcAutoToTeleOpHandoff<T> createForTest(String debugName,
                                                       Class<T> payloadType,
                                                       double maxAgeSec,
                                                       LongSupplier nanoTimeSource) {
        String checkedDebugName = requireDebugName(debugName);
        Class<T> checkedPayloadType = Objects.requireNonNull(
                payloadType,
                checkedDebugName + " payloadType is required");
        if (checkedPayloadType.isPrimitive()) {
            throw new IllegalArgumentException(
                    checkedDebugName + " payloadType must be a reference type; got primitive "
                            + checkedPayloadType.getName());
        }
        if (!Double.isFinite(maxAgeSec) || maxAgeSec <= 0.0) {
            throw new IllegalArgumentException(
                    checkedDebugName + " maxAgeSec must be finite and greater than zero; got "
                            + maxAgeSec);
        }
        LongSupplier checkedTimeSource = Objects.requireNonNull(
                nanoTimeSource,
                checkedDebugName + " nanoTimeSource is required");
        return new FtcAutoToTeleOpHandoff<>(
                checkedDebugName,
                checkedPayloadType,
                maxAgeSec,
                checkedTimeSource);
    }

    /**
     * Discard any pending or consumed state and open one fresh, empty handoff cycle.
     *
     * <p>Call this once when a new match Auto begins, before Auto can publish. Clearing an
     * unconsumed payload intentionally makes the next TeleOp consumption return
     * {@link ConsumeStatus#MISSING}. This operation is atomic and safe to race with publication or
     * consumption; whichever atomic state change occurs last determines the open cycle.</p>
     */
    public void clear() {
        state.set(State.empty());
    }

    /**
     * Publish one immutable Auto snapshot into the current open, empty cycle.
     *
     * <p>Only one publication is accepted before the cycle is consumed or cleared. A rejected
     * null value, wrong raw-generic payload type, or invalid lifecycle state leaves the current
     * handoff unchanged.</p>
     *
     * @param producer Auto OpMode publishing the snapshot; used only for its class name
     * @param payload immutable or defensively copied snapshot
     * @throws NullPointerException if {@code producer} or {@code payload} is null
     * @throws IllegalArgumentException if raw-generic code supplies a payload outside the type
     *                                  selected by {@link #create(String, Class, double)}
     * @throws IllegalStateException if this cycle already contains a publication or has already
     *                               been consumed
     */
    public void publishFromAuto(OpMode producer, T payload) {
        OpMode checkedProducer = Objects.requireNonNull(
                producer,
                debugName + " publishFromAuto(...) requires a producer OpMode");
        T checkedPayload = requirePayloadType(payload);
        String producerClassName = checkedProducer.getClass().getName();

        while (true) {
            State<T> current = state.get();
            if (current.phase == Phase.PUBLISHED) {
                throw new IllegalStateException(
                        debugName + " already has an Auto payload in the current cycle; "
                                + "call clear() at the beginning of the next Auto");
            }
            if (current.phase == Phase.CONSUMED) {
                throw new IllegalStateException(
                        debugName + " was already consumed in the current cycle; "
                                + "call clear() at the beginning of the next Auto");
            }
            long publishedAtNanos = nanoTimeSource.getAsLong();
            State<T> publication =
                    State.published(checkedPayload, producerClassName, publishedAtNanos);
            if (state.compareAndSet(current, publication)) {
                return;
            }
        }
    }

    /**
     * Atomically consume this cycle once and return an explicit result.
     *
     * <p>The first call closes the current cycle even when no payload exists or the payload is
     * stale. Later calls return {@link ConsumeStatus#ALREADY_CONSUMED} until {@link #clear()} opens
     * a new cycle. A payload is stale when its age is greater than {@code maxAgeSec}, or when the
     * monotonic clock appears to regress. An age exactly equal to {@code maxAgeSec} is delivered.</p>
     *
     * @param consumer TeleOp OpMode consuming the snapshot; used only for its class name
     * @return immutable result describing whether a snapshot was delivered
     * @throws NullPointerException if {@code consumer} is null
     */
    public ConsumeResult<T> consumeForTeleOp(OpMode consumer) {
        OpMode checkedConsumer = Objects.requireNonNull(
                consumer,
                debugName + " consumeForTeleOp(...) requires a consumer OpMode");
        String consumerClassName = checkedConsumer.getClass().getName();

        while (true) {
            State<T> current = state.get();
            if (current.phase == Phase.CONSUMED) {
                return new ConsumeResult<>(
                        ConsumeStatus.ALREADY_CONSUMED,
                        null,
                        current.producerClassNameOrNull,
                        consumerClassName,
                        Double.NaN);
            }

            if (current.phase == Phase.EMPTY) {
                State<T> consumed = State.consumed(null);
                if (state.compareAndSet(current, consumed)) {
                    return new ConsumeResult<>(
                            ConsumeStatus.MISSING,
                            null,
                            null,
                            consumerClassName,
                            Double.NaN);
                }
                continue;
            }

            long nowNanos = nanoTimeSource.getAsLong();
            long elapsedNanos = nowNanos - current.publishedAtNanos;
            double ageSec = elapsedNanos / NANOS_PER_SECOND;
            boolean stale = elapsedNanos < 0L || ageSec > maxAgeSec;
            State<T> consumed = State.consumed(current.producerClassNameOrNull);
            if (state.compareAndSet(current, consumed)) {
                return new ConsumeResult<>(
                        stale ? ConsumeStatus.STALE : ConsumeStatus.DELIVERED,
                        stale ? null : current.payloadOrNull,
                        current.producerClassNameOrNull,
                        consumerClassName,
                        ageSec);
            }
        }
    }

    private T requirePayloadType(T payload) {
        Object checkedPayload = Objects.requireNonNull(
                payload,
                debugName + " publishFromAuto(...) requires an immutable payload");
        if (!payloadType.isInstance(checkedPayload)) {
            throw new IllegalArgumentException(
                    debugName + " expected payload type " + payloadType.getName()
                            + " but publishFromAuto(...) received "
                            + checkedPayload.getClass().getName()
                            + "; do not bypass the handoff's generic type");
        }
        return payloadType.cast(checkedPayload);
    }

    private static String requireDebugName(String debugName) {
        if (debugName == null || debugName.trim().isEmpty()) {
            throw new IllegalArgumentException(
                    "FtcAutoToTeleOpHandoff debugName must be nonblank");
        }
        return debugName.trim();
    }

    /**
     * Outcome of one {@link #consumeForTeleOp(OpMode)} attempt.
     */
    public enum ConsumeStatus {
        /** A fresh payload was delivered and removed from the handoff. */
        DELIVERED,
        /** The first consumer found no publication in the current open cycle. */
        MISSING,
        /** The first consumer discarded a publication whose monotonic age was invalid or too old. */
        STALE,
        /** An earlier consumer already closed this cycle. */
        ALREADY_CONSUMED
    }

    /**
     * Immutable snapshot of one handoff-consumption attempt.
     *
     * <p>{@link #payloadOrNull()} is non-null only for {@link ConsumeStatus#DELIVERED}.
     * {@link #ageSec()} is a measured value only for {@link ConsumeStatus#DELIVERED} or
     * {@link ConsumeStatus#STALE}; it is {@link Double#NaN} otherwise.</p>
     */
    public static final class ConsumeResult<T> {
        private final ConsumeStatus status;
        private final T payloadOrNull;
        private final String producerClassNameOrNull;
        private final String consumerClassName;
        private final double ageSec;

        private ConsumeResult(ConsumeStatus status,
                              T payloadOrNull,
                              String producerClassNameOrNull,
                              String consumerClassName,
                              double ageSec) {
            this.status = status;
            this.payloadOrNull = payloadOrNull;
            this.producerClassNameOrNull = producerClassNameOrNull;
            this.consumerClassName = consumerClassName;
            this.ageSec = ageSec;
        }

        /**
         * Return the explicit outcome of this consumption attempt.
         */
        public ConsumeStatus status() {
            return status;
        }

        /**
         * Return the delivered immutable snapshot, or null for every non-delivery status.
         */
        public T payloadOrNull() {
            return payloadOrNull;
        }

        /**
         * Return the publishing OpMode's class name when this cycle had a publication, else null.
         */
        public String producerClassNameOrNull() {
            return producerClassNameOrNull;
        }

        /**
         * Return the class name of the consumer that made this attempt.
         */
        public String consumerClassName() {
            return consumerClassName;
        }

        /**
         * Return publication age in seconds, or {@link Double#NaN} when no publication was examined.
         *
         * <p>A negative value is retained for diagnostics and always accompanies
         * {@link ConsumeStatus#STALE}.</p>
         */
        public double ageSec() {
            return ageSec;
        }
    }

    private enum Phase {
        EMPTY,
        PUBLISHED,
        CONSUMED
    }

    /**
     * Immutable atomic state; each transition uses a fresh identity to avoid crossing clear cycles.
     */
    private static final class State<T> {
        final Phase phase;
        final T payloadOrNull;
        final String producerClassNameOrNull;
        final long publishedAtNanos;

        private State(Phase phase,
                      T payloadOrNull,
                      String producerClassNameOrNull,
                      long publishedAtNanos) {
            this.phase = phase;
            this.payloadOrNull = payloadOrNull;
            this.producerClassNameOrNull = producerClassNameOrNull;
            this.publishedAtNanos = publishedAtNanos;
        }

        static <T> State<T> empty() {
            return new State<>(Phase.EMPTY, null, null, 0L);
        }

        static <T> State<T> published(T payload,
                                      String producerClassName,
                                      long publishedAtNanos) {
            return new State<>(
                    Phase.PUBLISHED,
                    payload,
                    producerClassName,
                    publishedAtNanos);
        }

        static <T> State<T> consumed(String producerClassNameOrNull) {
            return new State<>(
                    Phase.CONSUMED,
                    null,
                    producerClassNameOrNull,
                    0L);
        }
    }
}
