package edu.ftcphoenix.fw.core.debug;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.LongSupplier;

import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Observes elapsed wall time across named, sequential phases of one robot loop.
 *
 * <p>This is an optional development diagnostic owned by a composition root. It does not advance,
 * replace, or supply behavior time for the robot's one {@link LoopClock}. The profiler owns a
 * private monotonic stopwatch only to measure work that happens after {@link #startCycle(LoopClock)}
 * and before {@link #finishCycle(LoopClock)}.</p>
 *
 * <p>Call {@link #finishPhase(String)} immediately after each useful high-level phase. It attributes
 * the interval since the cycle began or the previous phase finished to the phase that just
 * completed. This keeps loop order visible and deliberately does not introduce paired or nested
 * phase scopes.</p>
 *
 * <pre>{@code
 * private static final boolean PROFILE_LOOP_PHASES = false;
 * private final LoopPhaseProfiler loopPhases =
 *         LoopPhaseProfiler.create(PROFILE_LOOP_PHASES);
 *
 * // After the composition root advances its one LoopClock:
 * loopPhases.startCycle(clock);
 * localization.update(clock);
 * loopPhases.finishPhase("localization");
 * tasks.update(clock);
 * loopPhases.finishPhase("tasks");
 * loopPhases.finishCycle(clock);
 * }</pre>
 *
 * <p>Elapsed values are wall-clock observations, not CPU time and not the complete period between
 * FTC loop callbacks. They may include OS scheduling pauses. This class is confined to the OpMode
 * loop thread and is not thread-safe.</p>
 */
public final class LoopPhaseProfiler {

    /** Sentinel returned when no completed cycle has been observed. */
    public static final long NO_CYCLE = -1L;

    /** Fixed registry bound that prevents dynamic phase names from growing memory indefinitely. */
    static final int MAX_PHASES = 32;

    private static final double NANOS_TO_SEC = 1.0e-9;
    private static final LongSupplier SYSTEM_NANO_TIME = System::nanoTime;

    private static final Snapshot DISABLED_SNAPSHOT = new Snapshot(
            false,
            0L,
            0L,
            NO_CYCLE,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            Collections.<PhaseTiming>emptyList()
    );
    private static final LoopPhaseProfiler DISABLED = new LoopPhaseProfiler();

    private final boolean enabled;
    private final LongSupplier nanoTimeSource;
    private final Map<String, Integer> phaseIndexByName;
    private final String[] phaseNames;
    private final long[] phaseSampleCounts;
    private final long[] phaseLastObservedCycles;
    private final long[] phaseLatestNanos;
    private final double[] phaseAverageNanos;
    private final long[] phaseMaximumNanos;
    private final boolean[] scratchPhaseSeen;
    private final long[] scratchPhaseNanos;

    private int phaseCount;
    private int phaseCountAtCycleStart;
    private LoopClock boundClock;
    private boolean cycleActive;
    private long activeCycle = NO_CYCLE;
    private boolean hasStartedCycle;
    private long lastStartedCycle = NO_CYCLE;
    private long cycleStartNanos;
    private long nextPhaseStartNanos;
    private long completedCycleCount;
    private long incompleteCycleCount;
    private long lastCompletedCycle = NO_CYCLE;

    private long latestObservedNanos;
    private double averageObservedNanos;
    private long maximumObservedNanos;
    private long latestUnattributedNanos;
    private double averageUnattributedNanos;
    private long maximumUnattributedNanos;

    /** Create the shared disabled instance without allocating measurement state. */
    private LoopPhaseProfiler() {
        this.enabled = false;
        this.nanoTimeSource = null;
        this.phaseIndexByName = null;
        this.phaseNames = null;
        this.phaseSampleCounts = null;
        this.phaseLastObservedCycles = null;
        this.phaseLatestNanos = null;
        this.phaseAverageNanos = null;
        this.phaseMaximumNanos = null;
        this.scratchPhaseSeen = null;
        this.scratchPhaseNanos = null;
    }

    /** Create one enabled profiler around an internal monotonic source. */
    private LoopPhaseProfiler(LongSupplier nanoTimeSource) {
        this.enabled = true;
        this.nanoTimeSource = Objects.requireNonNull(nanoTimeSource, "nanoTimeSource");
        this.phaseIndexByName = new LinkedHashMap<>();
        this.phaseNames = new String[MAX_PHASES];
        this.phaseSampleCounts = new long[MAX_PHASES];
        this.phaseLastObservedCycles = new long[MAX_PHASES];
        this.phaseLatestNanos = new long[MAX_PHASES];
        this.phaseAverageNanos = new double[MAX_PHASES];
        this.phaseMaximumNanos = new long[MAX_PHASES];
        this.scratchPhaseSeen = new boolean[MAX_PHASES];
        this.scratchPhaseNanos = new long[MAX_PHASES];
        Arrays.fill(this.phaseLastObservedCycles, NO_CYCLE);
    }

    /**
     * Create an optional loop-phase profiler.
     *
     * <p>Disabled creation returns a shared inert instance. Its methods perform no validation,
     * stopwatch reads, lookup, allocation, output, or state mutation, so identical instrumentation
     * may remain in robot code while profiling is off.</p>
     *
     * @param enabled whether phase observation should be active for this measurement window
     * @return an enabled profiler or the shared disabled profiler
     */
    public static LoopPhaseProfiler create(boolean enabled) {
        return enabled ? new LoopPhaseProfiler(SYSTEM_NANO_TIME) : DISABLED;
    }

    /** Create a deterministic profiler for package-local tests without exposing a public clock. */
    static LoopPhaseProfiler createForTest(boolean enabled, LongSupplier nanoTimeSource) {
        return enabled ? new LoopPhaseProfiler(nanoTimeSource) : DISABLED;
    }

    /**
     * Start observing the supplied {@link LoopClock} cycle.
     *
     * <p>If an older cycle was left unfinished, its scratch data is discarded and counted before
     * this later cycle starts. A same-cycle overlapping start still fails fast. Once the profiler
     * has started a cycle, the supplied clock and cycle must advance until {@link #reset()} starts
     * an explicitly new observation window. Because {@code LoopClock} exposes no reset generation,
     * callers must pair a deliberate clock reset with a profiler reset rather than relying on the
     * profiler to infer every reset after intervening clock updates.</p>
     *
     * @param clock the composition root's one behavior clock, already advanced for this loop
     * @throws NullPointerException  if enabled and {@code clock} is null
     * @throws IllegalStateException if another clock is supplied or the supplied cycle did not
     *                               advance beyond the profiler's last observation attempt
     */
    public void startCycle(LoopClock clock) {
        if (!enabled) {
            return;
        }
        Objects.requireNonNull(clock, "clock");

        if (boundClock != null && boundClock != clock) {
            if (cycleActive) {
                discardActiveCycle();
            }
            throw new IllegalStateException(
                    "LoopPhaseProfiler is bound to a different LoopClock; call reset() before "
                            + "starting a new clock"
            );
        }

        long cycle = clock.cycle();
        if (cycleActive) {
            long previousActiveCycle = activeCycle;
            discardActiveCycle();
            if (cycle <= previousActiveCycle) {
                throw new IllegalStateException(
                        "LoopPhaseProfiler cycle " + previousActiveCycle
                                + " was already active; call finishCycle(clock) once before "
                                + "another same-cycle start, or call reset() after LoopClock.reset()"
                );
            }
        }

        if (hasStartedCycle && cycle <= lastStartedCycle) {
            throw new IllegalStateException(
                    "LoopPhaseProfiler requires an advancing LoopClock cycle after its last "
                            + "observation attempt at cycle " + lastStartedCycle + ", got " + cycle
                            + "; call reset() alongside a deliberate LoopClock.reset()"
            );
        }

        boundClock = clock;
        phaseCountAtCycleStart = phaseCount;
        Arrays.fill(scratchPhaseSeen, false);
        Arrays.fill(scratchPhaseNanos, 0L);
        long startNanos = nanoTimeSource.getAsLong();
        cycleStartNanos = startNanos;
        nextPhaseStartNanos = startNanos;
        activeCycle = cycle;
        cycleActive = true;
        lastStartedCycle = cycle;
        hasStartedCycle = true;
    }

    /**
     * Finish one named sequential phase and begin observing the next unnamed interval.
     *
     * <p>The first stopwatch sample closes the just-completed phase. A second sample after internal
     * lookup and accounting begins the next phase, leaving profiler bookkeeping visible as
     * unattributed time instead of silently charging it to the next robot owner. Repeated names in
     * one cycle are summed into one sample when the cycle completes.</p>
     *
     * @param phaseName stable, nonblank high-level phase name; do not generate a new name per cycle
     * @throws NullPointerException  if enabled and {@code phaseName} is null
     * @throws IllegalArgumentException if enabled and {@code phaseName} is blank
     * @throws IllegalStateException if no cycle is active, monotonic time regresses, accumulated
     *                               duration overflows, or a 33rd distinct name is introduced
     */
    public void finishPhase(String phaseName) {
        if (!enabled) {
            return;
        }
        requireActive("finishPhase(name) requires startCycle(clock) first");

        long phaseEndNanos = nanoTimeSource.getAsLong();
        long elapsedNanos = requireElapsedNanos(
                nextPhaseStartNanos,
                phaseEndNanos,
                "monotonic time regressed while finishing phase"
        );

        if (phaseName == null) {
            discardActiveCycle();
            throw new NullPointerException("phaseName");
        }
        if (isBlank(phaseName)) {
            discardActiveCycle();
            throw new IllegalArgumentException("phaseName must contain a non-whitespace character");
        }

        int phaseIndex = findOrRegisterPhase(phaseName);
        try {
            scratchPhaseNanos[phaseIndex] = Math.addExact(
                    scratchPhaseNanos[phaseIndex],
                    elapsedNanos
            );
        } catch (ArithmeticException overflow) {
            discardActiveCycle();
            throw new IllegalStateException(
                    "LoopPhaseProfiler duration overflow while accumulating phase '"
                            + phaseName + "'",
                    overflow
            );
        }
        scratchPhaseSeen[phaseIndex] = true;

        long nextStartNanos = nanoTimeSource.getAsLong();
        requireElapsedNanos(
                phaseEndNanos,
                nextStartNanos,
                "monotonic time regressed during profiler bookkeeping"
        );
        nextPhaseStartNanos = nextStartNanos;
    }

    /**
     * Finish and atomically commit the active cycle's completed measurements.
     *
     * <p>The supplied clock must be the same object and still expose the cycle passed to
     * {@link #startCycle(LoopClock)}. Time after the last {@link #finishPhase(String)} is not given a
     * phase name; it remains part of the observed total and unattributed duration.</p>
     *
     * @param clock the same composition-root clock and cycle supplied at cycle start
     * @throws NullPointerException  if enabled and {@code clock} is null
     * @throws IllegalStateException if no cycle is active, the clock/cycle changed, time regresses,
     *                               or phase duration accounting is inconsistent
     */
    public void finishCycle(LoopClock clock) {
        if (!enabled) {
            return;
        }
        requireActive("finishCycle(clock) requires startCycle(clock) first");
        if (clock == null) {
            discardActiveCycle();
            throw new NullPointerException("clock");
        }
        if (clock != boundClock) {
            discardActiveCycle();
            throw new IllegalStateException(
                    "finishCycle(clock) must use the same LoopClock supplied to startCycle(clock)"
            );
        }
        if (clock.cycle() != activeCycle) {
            long actualCycle = clock.cycle();
            long expectedCycle = activeCycle;
            discardActiveCycle();
            throw new IllegalStateException(
                    "LoopClock advanced or reset before LoopPhaseProfiler finished cycle "
                            + expectedCycle + "; got cycle " + actualCycle
            );
        }

        long cycleEndNanos = nanoTimeSource.getAsLong();
        long observedNanos = requireElapsedNanos(
                cycleStartNanos,
                cycleEndNanos,
                "monotonic time regressed while finishing the observed cycle"
        );

        long attributedNanos = 0L;
        try {
            for (int i = 0; i < phaseCount; i++) {
                if (scratchPhaseSeen[i]) {
                    attributedNanos = Math.addExact(attributedNanos, scratchPhaseNanos[i]);
                }
            }
        } catch (ArithmeticException overflow) {
            discardActiveCycle();
            throw new IllegalStateException(
                    "LoopPhaseProfiler attributed duration overflow; use fewer stable high-level "
                            + "phases",
                    overflow
            );
        }
        if (attributedNanos > observedNanos) {
            discardActiveCycle();
            throw new IllegalStateException(
                    "LoopPhaseProfiler phase durations exceeded the observed cycle span; "
                            + "the monotonic source is inconsistent"
            );
        }
        long unattributedNanos = observedNanos - attributedNanos;

        long completedSampleNumber = completedCycleCount + 1L;
        for (int i = 0; i < phaseCount; i++) {
            if (scratchPhaseSeen[i]) {
                commitPhaseSample(i, activeCycle, scratchPhaseNanos[i]);
            }
        }
        latestObservedNanos = observedNanos;
        averageObservedNanos = updateMean(
                averageObservedNanos,
                observedNanos,
                completedSampleNumber
        );
        maximumObservedNanos = completedCycleCount == 0L
                ? observedNanos
                : Math.max(maximumObservedNanos, observedNanos);
        latestUnattributedNanos = unattributedNanos;
        averageUnattributedNanos = updateMean(
                averageUnattributedNanos,
                unattributedNanos,
                completedSampleNumber
        );
        maximumUnattributedNanos = completedCycleCount == 0L
                ? unattributedNanos
                : Math.max(maximumUnattributedNanos, unattributedNanos);

        completedCycleCount = completedSampleNumber;
        lastCompletedCycle = activeCycle;
        cycleActive = false;
        activeCycle = NO_CYCLE;
    }

    /**
     * Clear all completed measurements, known names, and clock/cycle binding.
     *
     * <p>Use this only for an explicitly new measurement window, including when one profiler was
     * already used before a deliberate {@link LoopClock#reset(double)}. Ordinary active-loop use
     * does not require resets.</p>
     *
     * @throws IllegalStateException if enabled and a cycle is active
     */
    public void reset() {
        if (!enabled) {
            return;
        }
        if (cycleActive) {
            throw new IllegalStateException(
                    "LoopPhaseProfiler cannot reset an active cycle; finish the cycle first"
            );
        }

        phaseIndexByName.clear();
        Arrays.fill(phaseNames, null);
        Arrays.fill(phaseSampleCounts, 0L);
        Arrays.fill(phaseLastObservedCycles, NO_CYCLE);
        Arrays.fill(phaseLatestNanos, 0L);
        Arrays.fill(phaseAverageNanos, 0.0);
        Arrays.fill(phaseMaximumNanos, 0L);
        Arrays.fill(scratchPhaseSeen, false);
        Arrays.fill(scratchPhaseNanos, 0L);
        phaseCount = 0;
        phaseCountAtCycleStart = 0;
        boundClock = null;
        cycleActive = false;
        activeCycle = NO_CYCLE;
        hasStartedCycle = false;
        lastStartedCycle = NO_CYCLE;
        completedCycleCount = 0L;
        incompleteCycleCount = 0L;
        lastCompletedCycle = NO_CYCLE;
        latestObservedNanos = 0L;
        averageObservedNanos = 0.0;
        maximumObservedNanos = 0L;
        latestUnattributedNanos = 0L;
        averageUnattributedNanos = 0.0;
        maximumUnattributedNanos = 0L;
    }

    /**
     * Return an immutable copy of completed diagnostic measurements.
     *
     * <p>This method never samples the stopwatch or changes an active cycle. A snapshot obtained
     * while a cycle is active therefore contains only earlier completed cycles.</p>
     */
    public Snapshot snapshot() {
        if (!enabled) {
            return DISABLED_SNAPSHOT;
        }

        List<PhaseTiming> phases = new ArrayList<>(phaseCount);
        for (int i = 0; i < phaseCount; i++) {
            if (phaseSampleCounts[i] > 0L) {
                phases.add(new PhaseTiming(
                        phaseNames[i],
                        phaseSampleCounts[i],
                        phaseLastObservedCycles[i],
                        nanosToSec(phaseLatestNanos[i]),
                        nanosToSec(phaseAverageNanos[i]),
                        nanosToSec(phaseMaximumNanos[i])
                ));
            }
        }
        return new Snapshot(
                true,
                completedCycleCount,
                incompleteCycleCount,
                lastCompletedCycle,
                nanosToSec(latestObservedNanos),
                nanosToSec(averageObservedNanos),
                nanosToSec(maximumObservedNanos),
                nanosToSec(latestUnattributedNanos),
                nanosToSec(averageUnattributedNanos),
                nanosToSec(maximumUnattributedNanos),
                Collections.unmodifiableList(phases)
        );
    }

    /**
     * Emit completed phase diagnostics using stable keys.
     *
     * <p>Disabled profilers and null sinks produce no output. While a cycle is active, statistics
     * still describe only previously completed cycles so rendering cannot expose partial data.</p>
     *
     * @param dbg    debug sink, or {@code null} to suppress output
     * @param prefix key prefix; defaults to {@code loopProfile} when null or empty
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (!enabled || dbg == null) {
            return;
        }
        String p = prefix == null || prefix.isEmpty() ? "loopProfile" : prefix;
        dbg.addData(p + ".enabled", Boolean.TRUE)
                .addData(p + ".completedCycleCount", Long.valueOf(completedCycleCount))
                .addData(p + ".incompleteCycleCount", Long.valueOf(incompleteCycleCount))
                .addData(p + ".lastCompletedCycle", Long.valueOf(lastCompletedCycle))
                .addData(p + ".latestObservedDurationSec", nanosToSec(latestObservedNanos))
                .addData(p + ".averageObservedDurationSec", nanosToSec(averageObservedNanos))
                .addData(p + ".maxObservedDurationSec", nanosToSec(maximumObservedNanos))
                .addData(
                        p + ".latestUnattributedDurationSec",
                        nanosToSec(latestUnattributedNanos)
                )
                .addData(
                        p + ".averageUnattributedDurationSec",
                        nanosToSec(averageUnattributedNanos)
                )
                .addData(
                        p + ".maxUnattributedDurationSec",
                        nanosToSec(maximumUnattributedNanos)
                );
        for (int i = 0; i < phaseCount; i++) {
            if (phaseSampleCounts[i] == 0L) {
                continue;
            }
            String phasePrefix = p + ".phase." + phaseNames[i];
            dbg.addData(
                            phasePrefix + ".sampleCount",
                            Long.valueOf(phaseSampleCounts[i])
                    )
                    .addData(
                            phasePrefix + ".lastObservedCycle",
                            Long.valueOf(phaseLastObservedCycles[i])
                    )
                    .addData(
                            phasePrefix + ".latestDurationSec",
                            nanosToSec(phaseLatestNanos[i])
                    )
                    .addData(
                            phasePrefix + ".averageDurationSec",
                            nanosToSec(phaseAverageNanos[i])
                    )
                    .addData(
                            phasePrefix + ".maxDurationSec",
                            nanosToSec(phaseMaximumNanos[i])
                    );
        }
    }

    /** Find an existing phase or register one new stable name within the fixed bound. */
    private int findOrRegisterPhase(String phaseName) {
        Integer existing = phaseIndexByName.get(phaseName);
        if (existing != null) {
            return existing.intValue();
        }
        if (phaseCount >= MAX_PHASES) {
            discardActiveCycle();
            throw new IllegalStateException(
                    "LoopPhaseProfiler supports at most " + MAX_PHASES
                            + " stable phase names; cannot add '" + phaseName
                            + "'. Use fixed high-level names instead of dynamically generated names, "
                            + "or reset() for a deliberately new measurement window"
            );
        }
        int index = phaseCount++;
        phaseNames[index] = phaseName;
        phaseIndexByName.put(phaseName, Integer.valueOf(index));
        return index;
    }

    /** Commit one phase sample after the complete cycle has passed every validation. */
    private void commitPhaseSample(int index, long cycle, long durationNanos) {
        long previousCount = phaseSampleCounts[index];
        long nextCount = previousCount + 1L;
        phaseLatestNanos[index] = durationNanos;
        phaseAverageNanos[index] = updateMean(
                phaseAverageNanos[index],
                durationNanos,
                nextCount
        );
        phaseMaximumNanos[index] = previousCount == 0L
                ? durationNanos
                : Math.max(phaseMaximumNanos[index], durationNanos);
        phaseSampleCounts[index] = nextCount;
        phaseLastObservedCycles[index] = cycle;
    }

    /** Validate a short monotonic interval while allowing {@code nanoTime()} origin/wrap behavior. */
    private long requireElapsedNanos(long startNanos, long endNanos, String message) {
        long elapsedNanos = endNanos - startNanos;
        if (elapsedNanos < 0L) {
            discardActiveCycle();
            throw new IllegalStateException(message);
        }
        return elapsedNanos;
    }

    /** Fail when a lifecycle operation needs an active observation but none exists. */
    private void requireActive(String message) {
        if (!cycleActive) {
            throw new IllegalStateException(message);
        }
    }

    /** Discard one active scratch cycle and retain a truthful health count. */
    private void discardActiveCycle() {
        if (!cycleActive) {
            return;
        }
        rollbackUncommittedPhaseNames();
        cycleActive = false;
        activeCycle = NO_CYCLE;
        incompleteCycleCount++;
    }

    /** Remove names first encountered by a cycle that never committed. */
    private void rollbackUncommittedPhaseNames() {
        for (int i = phaseCount - 1; i >= phaseCountAtCycleStart; i--) {
            phaseIndexByName.remove(phaseNames[i]);
            phaseNames[i] = null;
            phaseSampleCounts[i] = 0L;
            phaseLastObservedCycles[i] = NO_CYCLE;
            phaseLatestNanos[i] = 0L;
            phaseAverageNanos[i] = 0.0;
            phaseMaximumNanos[i] = 0L;
            scratchPhaseSeen[i] = false;
            scratchPhaseNanos[i] = 0L;
        }
        phaseCount = phaseCountAtCycleStart;
    }

    /** Return whether a name lacks any non-whitespace character without allocating a trimmed copy. */
    private static boolean isBlank(String value) {
        for (int i = 0; i < value.length(); i++) {
            if (!Character.isWhitespace(value.charAt(i))) {
                return false;
            }
        }
        return true;
    }

    /** Update an online arithmetic mean without retaining per-cycle history. */
    private static double updateMean(double previousMean, long sampleNanos, long sampleCount) {
        return previousMean + (((double) sampleNanos) - previousMean) / (double) sampleCount;
    }

    /** Convert an integer nanosecond duration to explicit public seconds. */
    private static double nanosToSec(long nanos) {
        return ((double) nanos) * NANOS_TO_SEC;
    }

    /** Convert an online nanosecond mean to explicit public seconds. */
    private static double nanosToSec(double nanos) {
        return nanos * NANOS_TO_SEC;
    }

    /** Immutable completed diagnostic measurements from one profiler. */
    public static final class Snapshot {

        private final boolean enabled;
        private final long completedCycleCount;
        private final long incompleteCycleCount;
        private final long lastCompletedCycle;
        private final double latestObservedDurationSec;
        private final double averageObservedDurationSec;
        private final double maxObservedDurationSec;
        private final double latestUnattributedDurationSec;
        private final double averageUnattributedDurationSec;
        private final double maxUnattributedDurationSec;
        private final List<PhaseTiming> phases;

        /** Construct an immutable profiler-owned copy. */
        private Snapshot(boolean enabled,
                         long completedCycleCount,
                         long incompleteCycleCount,
                         long lastCompletedCycle,
                         double latestObservedDurationSec,
                         double averageObservedDurationSec,
                         double maxObservedDurationSec,
                         double latestUnattributedDurationSec,
                         double averageUnattributedDurationSec,
                         double maxUnattributedDurationSec,
                         List<PhaseTiming> phases) {
            this.enabled = enabled;
            this.completedCycleCount = completedCycleCount;
            this.incompleteCycleCount = incompleteCycleCount;
            this.lastCompletedCycle = lastCompletedCycle;
            this.latestObservedDurationSec = latestObservedDurationSec;
            this.averageObservedDurationSec = averageObservedDurationSec;
            this.maxObservedDurationSec = maxObservedDurationSec;
            this.latestUnattributedDurationSec = latestUnattributedDurationSec;
            this.averageUnattributedDurationSec = averageUnattributedDurationSec;
            this.maxUnattributedDurationSec = maxUnattributedDurationSec;
            this.phases = phases;
        }

        /** Return whether this snapshot came from an enabled profiler. */
        public boolean isEnabled() {
            return enabled;
        }

        /** Return the number of cycles committed atomically into this snapshot. */
        public long completedCycleCount() {
            return completedCycleCount;
        }

        /** Return the number of active scratch cycles discarded before this snapshot. */
        public long incompleteCycleCount() {
            return incompleteCycleCount;
        }

        /** Return the last completed {@link LoopClock#cycle()}, or {@link LoopPhaseProfiler#NO_CYCLE}. */
        public long lastCompletedCycle() {
            return lastCompletedCycle;
        }

        /** Return the latest complete observed span in seconds. */
        public double latestObservedDurationSec() {
            return latestObservedDurationSec;
        }

        /** Return the mean complete observed span since creation or reset, in seconds. */
        public double averageObservedDurationSec() {
            return averageObservedDurationSec;
        }

        /** Return the maximum complete observed span since creation or reset, in seconds. */
        public double maxObservedDurationSec() {
            return maxObservedDurationSec;
        }

        /** Return the latest observed-but-unnamed or observer interval in seconds. */
        public double latestUnattributedDurationSec() {
            return latestUnattributedDurationSec;
        }

        /** Return the mean observed-but-unnamed or observer interval in seconds. */
        public double averageUnattributedDurationSec() {
            return averageUnattributedDurationSec;
        }

        /** Return the maximum observed-but-unnamed or observer interval in seconds. */
        public double maxUnattributedDurationSec() {
            return maxUnattributedDurationSec;
        }

        /** Return immutable phase entries in stable first-registration order. */
        public List<PhaseTiming> phases() {
            return phases;
        }
    }

    /** Immutable completed statistics for one stable phase name. */
    public static final class PhaseTiming {

        private final String name;
        private final long sampleCount;
        private final long lastObservedCycle;
        private final double latestDurationSec;
        private final double averageDurationSec;
        private final double maxDurationSec;

        /** Construct an immutable profiler-owned phase entry. */
        private PhaseTiming(String name,
                            long sampleCount,
                            long lastObservedCycle,
                            double latestDurationSec,
                            double averageDurationSec,
                            double maxDurationSec) {
            this.name = name;
            this.sampleCount = sampleCount;
            this.lastObservedCycle = lastObservedCycle;
            this.latestDurationSec = latestDurationSec;
            this.averageDurationSec = averageDurationSec;
            this.maxDurationSec = maxDurationSec;
        }

        /** Return the stable name supplied to {@link LoopPhaseProfiler#finishPhase(String)}. */
        public String name() {
            return name;
        }

        /** Return completed cycles in which this phase occurred at least once. */
        public long sampleCount() {
            return sampleCount;
        }

        /** Return the last completed {@link LoopClock#cycle()} containing this phase. */
        public long lastObservedCycle() {
            return lastObservedCycle;
        }

        /** Return this phase's summed duration in its latest observed cycle, in seconds. */
        public double latestDurationSec() {
            return latestDurationSec;
        }

        /** Return this phase's mean summed duration across cycles containing it, in seconds. */
        public double averageDurationSec() {
            return averageDurationSec;
        }

        /** Return this phase's maximum summed duration in one completed cycle, in seconds. */
        public double maxDurationSec() {
            return maxDurationSec;
        }
    }
}
